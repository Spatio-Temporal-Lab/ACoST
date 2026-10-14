#include "experiments/e2_ablation/cost_acost_family_compressor.h"

#include "utils/double.h"
#include "utils/elias_gamma_codec.h"
#include "utils/zig_zag_codec.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kTurnTanMin = 0.08748866352592401; // tan(5 degrees)
constexpr double kAdaptiveTailOnset = 100.0;         // timestamp units are 0.1 s
constexpr int kSpeedTailSegments = 3;
constexpr int kAccelTailSegments = 2;
constexpr double kMaxSpeedMetersPerSecond = 200.0;
constexpr double kMaxDisplacementMeters = 50000.0;

struct MotionPrediction { int64_t q_lon; int64_t q_lat; int model; };

template <typename History>
void weightedTailAverage(const double* vx, const double* vy,
                         size_t count, size_t tail,
                         double& out_vx, double& out_vy) {
    out_vx = out_vy = 0.0;
    if (count == 0 || tail == 0) return;
    const size_t start = count > tail ? count - tail : 0;
    double weight_sum = 0.0;
    double weight = 1.0;
    for (size_t i = start; i < count; ++i, weight += 1.0) {
        out_vx += weight * vx[i];
        out_vy += weight * vy[i];
        weight_sum += weight;
    }
    if (weight_sum > 0.0) {
        out_vx /= weight_sum;
        out_vy /= weight_sum;
    }
}

template <typename History>
MotionPrediction predictFamily(const std::vector<History>& history,
                               int64_t current_q_lon,
                               int64_t current_q_lat,
                               uint64_t current_timestamp,
                               uint64_t target_timestamp,
                               double lon_scale,
                               double quant_step) {
    const size_t n = history.size();
    if (n < 2) return {current_q_lon, current_q_lat, 0};
    const size_t num_segments = n - 1;
    double local_x[8], local_y[8], vx[7], vy[7], segment_dt[7];

    for (size_t i = 0; i < n; ++i) {
        local_x[i] = static_cast<double>(history[i].q_lon - current_q_lon) * lon_scale;
        local_y[i] = static_cast<double>(history[i].q_lat - current_q_lat);
    }
    double avg_dt = 0.0;
    for (size_t i = 0; i < num_segments; ++i) {
        const int64_t delta_ts = static_cast<int64_t>(history[i + 1].timestamp) -
                                 static_cast<int64_t>(history[i].timestamp);
        const double dt = delta_ts > 0 ? static_cast<double>(delta_ts) : 1.0;
        vx[i] = (local_x[i + 1] - local_x[i]) / dt;
        vy[i] = (local_y[i + 1] - local_y[i]) / dt;
        segment_dt[i] = dt;
        avg_dt += dt;
    }
    avg_dt /= static_cast<double>(num_segments);

    const int64_t delta_ts = static_cast<int64_t>(target_timestamp) -
                             static_cast<int64_t>(current_timestamp);
    const double dt = delta_ts > 0 ? static_cast<double>(delta_ts) : 1.0;
    const double tail_factor = kAdaptiveTailOnset /
        std::max(avg_dt, kAdaptiveTailOnset);
    const size_t speed_tail = std::min(num_segments, static_cast<size_t>(
        std::max(1.0, std::ceil(tail_factor * kSpeedTailSegments))));
    const size_t accel_tail = std::min(num_segments, static_cast<size_t>(
        std::max(1.0, std::ceil(tail_factor * kAccelTailSegments))));

    double avg_vx, avg_vy;
    weightedTailAverage<History>(vx, vy, num_segments, speed_tail, avg_vx, avg_vy);

    // STOP is a deterministic member of the ACoST motion family, not the
    // ACoST ZP-selection mechanism. It uses the family predictor's one-cell test.
    const double dx = local_x[n - 1] - local_x[n - 2];
    const double dy = local_y[n - 1] - local_y[n - 2];
    if (dx * dx + dy * dy < lon_scale * lon_scale + 1.0)
        return {current_q_lon, current_q_lat, 0};

    const double speed_sq = avg_vx * avg_vx + avg_vy * avg_vy;
    if (num_segments < 2) {
        double px = avg_vx * dt;
        double py = avg_vy * dt;
        const double meters_to_quant_units =
            1.0 / (quant_step * (kPi / 180.0) * kEarthRadiusMeters);
        const double max_displacement = std::min(
            kMaxSpeedMetersPerSecond * meters_to_quant_units * dt,
            kMaxDisplacementMeters * meters_to_quant_units);
        px = std::max(-max_displacement, std::min(px, max_displacement));
        py = std::max(-max_displacement, std::min(py, max_displacement));
        return {current_q_lon + static_cast<int64_t>(std::round(px / lon_scale)),
                current_q_lat + static_cast<int64_t>(std::round(py)), 1};
    }

    double previous_vx, previous_vy;
    weightedTailAverage<History>(vx, vy, num_segments - 1, accel_tail,
                                 previous_vx, previous_vy);
    double dt_history = segment_dt[num_segments - 1];
    if (dt_history <= 0.0) dt_history = 1.0;

    const double cross = previous_vx * avg_vy - previous_vy * avg_vx;
    const double dot = previous_vx * avg_vx + previous_vy * avg_vy;
    const bool is_turning = dot > 0.0 &&
        cross * cross > kTurnTanMin * kTurnTanMin * dot * dot;

    // Full ACoST turn-aware vector acceleration, with damping omitted.
    double ax = (avg_vx - previous_vx) / dt_history;
    double ay = (avg_vy - previous_vy) / dt_history;
    if (!is_turning && speed_sq > 1.0e-24) {
        const double projection = (ax * avg_vx + ay * avg_vy) / speed_sq;
        ax = projection * avg_vx;
        ay = projection * avg_vy;
    }
    double px = avg_vx * dt + 0.5 * ax * dt * dt;
    double py = avg_vy * dt + 0.5 * ay * dt * dt;
    if (!std::isfinite(px) || !std::isfinite(py)) {
        px = avg_vx * dt;
        py = avg_vy * dt;
    }
    const double meters_to_quant_units =
        1.0 / (quant_step * (kPi / 180.0) * kEarthRadiusMeters);
    const double max_displacement = std::min(
        kMaxSpeedMetersPerSecond * meters_to_quant_units * dt,
        kMaxDisplacementMeters * meters_to_quant_units);
    px = std::max(-max_displacement, std::min(px, max_displacement));
    py = std::max(-max_displacement, std::min(py, max_displacement));

    return {current_q_lon + static_cast<int64_t>(std::round(px / lon_scale)),
            current_q_lat + static_cast<int64_t>(std::round(py)),
            is_turning ? 2 : 1};
}

void writeResidual(int64_t residual, OutputBitStream* output, int& bit_count) {
    bit_count += EliasGammaCodec::Encode(ZigZagCodec::Encode(residual) + 1, output);
}
} // namespace

CoSTFamilyCompressor::CoSTFamilyCompressor(int block_size, double epsilon,
                                           int evaluation_window,
                                           bool use_time_window,
                                           uint64_t time_window_seconds)
    : block_size_(block_size), epsilon_(epsilon * 0.999),
      quant_step_(2.0 * epsilon * 0.999),
      evaluation_window_(evaluation_window), use_time_window_(use_time_window),
      time_window_seconds_(time_window_seconds),
      output_(std::make_unique<OutputBitStream>(std::max(4096, block_size * 128))) {
    history_.reserve(kMaxHistorySize);
}

void CoSTFamilyCompressor::WriteFirstPoint(const GpsPoint& point) {
    compressed_size_in_bits_ += output_->WriteInt(block_size_, 16);
    compressed_size_in_bits_ += output_->WriteLong(Double::DoubleToLongBits(epsilon_), 64);
    compressed_size_in_bits_ += output_->WriteInt(evaluation_window_, 16);
    compressed_size_in_bits_ += output_->WriteBit(use_time_window_);
    if (use_time_window_)
        compressed_size_in_bits_ += output_->WriteInt(time_window_seconds_, 32);
    compressed_size_in_bits_ += output_->WriteLong(Double::DoubleToLongBits(point.longitude), 64);
    compressed_size_in_bits_ += output_->WriteLong(Double::DoubleToLongBits(point.latitude), 64);
    compressed_size_in_bits_ += output_->WriteLong(point.timestamp, 64);
    stats_.timestamp_bits += 64;
    current_q_lon_ = static_cast<int64_t>(std::round(point.longitude / quant_step_));
    current_q_lat_ = static_cast<int64_t>(std::round(point.latitude / quant_step_));
    current_timestamp_ = point.timestamp;
    lon_scale_ = std::max(1.0e-6, std::abs(std::cos(point.latitude * kPi / 180.0)));
    history_.push_back({current_q_lon_, current_q_lat_, current_timestamp_});
    stats_.total_points++;
    first_point_ = false;
}

CoSTFamilyCompressor::Prediction CoSTFamilyCompressor::Predict(uint64_t target_timestamp) const {
    const MotionPrediction result = predictFamily(history_, current_q_lon_, current_q_lat_,
        current_timestamp_, target_timestamp, lon_scale_, quant_step_);
    return {result.q_lon, result.q_lat, result.model};
}

void CoSTFamilyCompressor::AddGpsPoint(const GpsPoint& point) {
    if (first_point_) {
        WriteFirstPoint(point);
        return;
    }
    const Prediction prediction = Predict(point.timestamp);
    const int64_t actual_q_lon = static_cast<int64_t>(std::round(point.longitude / quant_step_));
    const int64_t actual_q_lat = static_cast<int64_t>(std::round(point.latitude / quant_step_));
    const int64_t residual_lon = actual_q_lon - prediction.q_lon;
    const int64_t residual_lat = actual_q_lat - prediction.q_lat;

    const uint64_t delta_ts = point.timestamp - current_timestamp_;
    const int ts_bits = output_->WriteLong(delta_ts, 64);
    compressed_size_in_bits_ += ts_bits;
    stats_.timestamp_bits += ts_bits;
    writeResidual(residual_lon, output_.get(), compressed_size_in_bits_);
    writeResidual(residual_lat, output_.get(), compressed_size_in_bits_);

    current_q_lon_ = prediction.q_lon + residual_lon;
    current_q_lat_ = prediction.q_lat + residual_lat;
    current_timestamp_ = point.timestamp;
    UpdateHistory(current_q_lon_, current_q_lat_, current_timestamp_);
    ++stats_.total_points;
    if (prediction.model == kStop) ++stats_.stop_count;
    else if (prediction.model == kCv) ++stats_.cv_count;
    else ++stats_.ctra_count;
}

void CoSTFamilyCompressor::UpdateHistory(int64_t q_lon, int64_t q_lat, uint64_t timestamp) {
    history_.push_back({q_lon, q_lat, timestamp});
    if (history_.size() > static_cast<size_t>(kMaxHistorySize)) history_.erase(history_.begin());
}

void CoSTFamilyCompressor::Close() {
    output_->Flush();
    stats_.total_bits = compressed_size_in_bits_;
}

Array<uint8_t> CoSTFamilyCompressor::GetCompressedData() const {
    const int byte_length = (compressed_size_in_bits_ + 7) / 8;
    return output_->GetBuffer(byte_length);
}

CoSTFamilyDecompressor::CoSTFamilyDecompressor(uint8_t* data, int size)
    : input_(std::make_unique<InputBitStream>(data, size)) {
    ReadHeader();
}

void CoSTFamilyDecompressor::ReadHeader() {
    const int block_size = static_cast<int>(input_->ReadInt(16));
    epsilon_ = Double::LongBitsToDouble(input_->ReadLong(64));
    quant_step_ = 2.0 * epsilon_;
    const int evaluation_window = static_cast<int>(input_->ReadInt(16));
    const bool use_time_window = input_->ReadBit() != 0;
    if (use_time_window) (void)input_->ReadInt(32);
    if (block_size <= 0 || evaluation_window <= 0 || !std::isfinite(quant_step_) || quant_step_ <= 0.0)
        throw std::runtime_error("invalid CoST-family stream header");
}

CoSTFamilyDecompressor::Prediction CoSTFamilyDecompressor::Predict(uint64_t target_timestamp) const {
    const MotionPrediction result = predictFamily(history_, current_q_lon_, current_q_lat_,
        current_timestamp_, target_timestamp, lon_scale_, quant_step_);
    return {result.q_lon, result.q_lat, result.model};
}

bool CoSTFamilyDecompressor::ReadNextPoint(GpsPoint& point) {
    if (finished_) return false;
    try {
        if (first_point_) {
            const double lon = Double::LongBitsToDouble(input_->ReadLong(64));
            const double lat = Double::LongBitsToDouble(input_->ReadLong(64));
            const uint64_t timestamp = input_->ReadLong(64);
            current_q_lon_ = static_cast<int64_t>(std::round(lon / quant_step_));
            current_q_lat_ = static_cast<int64_t>(std::round(lat / quant_step_));
            current_timestamp_ = timestamp;
            lon_scale_ = std::max(1.0e-6, std::abs(std::cos(lat * kPi / 180.0)));
            history_.push_back({current_q_lon_, current_q_lat_, current_timestamp_});
            first_point_ = false;
            point = GpsPoint(lon, lat, timestamp);
            return true;
        }

        const uint64_t timestamp = current_timestamp_ + input_->ReadLong(64);
        const Prediction prediction = Predict(timestamp);
        const int64_t residual_lon = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_.get()) - 1);
        const int64_t residual_lat = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_.get()) - 1);
        current_q_lon_ = prediction.q_lon + residual_lon;
        current_q_lat_ = prediction.q_lat + residual_lat;
        current_timestamp_ = timestamp;
        UpdateHistory(current_q_lon_, current_q_lat_, current_timestamp_);
        point = GpsPoint(current_q_lon_ * quant_step_, current_q_lat_ * quant_step_, timestamp);
        return true;
    } catch (...) {
        finished_ = true;
        return false;
    }
}

void CoSTFamilyDecompressor::UpdateHistory(int64_t q_lon, int64_t q_lat, uint64_t timestamp) {
    history_.push_back({q_lon, q_lat, timestamp});
    if (history_.size() > static_cast<size_t>(kMaxHistorySize)) history_.erase(history_.begin());
}
