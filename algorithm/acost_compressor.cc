#include "algorithm/acost_compressor.h"
#include "utils/elias_gamma_codec.h"
#include "utils/zig_zag_codec.h"
#include "utils/double.h"
#include <algorithm>
#include <cmath>

#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kDegToRad          = M_PI / 180.0;

// Weighted average of last `tail` elements in an array of `count` doubles.
inline double wtailAvg(const double* v, size_t count, size_t tail) {
    if (count == 0 || tail == 0) return 0.0;
    size_t start = (count > tail) ? count - tail : 0;
    double sum = 0.0, wsum = 0.0, w = 1.0;
    for (size_t i = start; i < count; ++i, w += 1.0) {
        sum  += w * v[i];
        wsum += w;
    }
    return wsum > 0.0 ? sum / wsum : 0.0;
}

// Weighted average velocity vector (vx, vy) for last `tail` elements.
inline void wtailVecAvg(const double* vx, const double* vy,
                        size_t count, size_t tail,
                        double& out_vx, double& out_vy) {
    out_vx = out_vy = 0.0;
    if (count == 0 || tail == 0) return;
    size_t start = (count > tail) ? count - tail : 0;
    double wsum = 0.0, w = 1.0;
    for (size_t i = start; i < count; ++i, w += 1.0) {
        out_vx += w * vx[i];
        out_vy += w * vy[i];
        wsum   += w;
    }
    if (wsum > 0.0) { out_vx /= wsum; out_vy /= wsum; }
}

// 鈹€鈹€鈹€ Bit-cost helpers 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
//
// Returns the number of bits that EliasGamma would spend on (ZigZag(delta)+1)
// without actually writing anything.  Used to compare ZP vs motion predictor.
static inline int egDimCost(int64_t delta) {
    uint64_t n = static_cast<uint64_t>(ZigZagCodec::Encode(delta)) + 1;
    int k = 0;
    for (uint64_t tmp = n >> 1; tmp > 0; tmp >>= 1) ++k;
    return 2 * k + 1;
}

// Mean interval (timestamp units) across all segments currently in history.
static inline double historyAvgDt(
    const std::vector<ACoSTCompressor::HistoryState>& h)
{
    size_t n = h.size();
    if (n < 2) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i + 1 < n; ++i)
        sum += static_cast<double>(
            static_cast<int64_t>(h[i + 1].timestamp) -
            static_cast<int64_t>(h[i].timestamp));
    return sum / static_cast<double>(n - 1);
}

// Low-cost approximation for exp(-x).
//
// For x >= 0, 1 / (1 + x + x虏/2 + x鲁/6) is a stable reciprocal-polynomial
// approximation.  It is not bit-identical to std::exp(-x), but compressor and
// decompressor call the same function, so prediction remains synchronized.
// Clamp large x to zero because the damping contribution is already negligible.
inline double fastExpNeg(double x) {
    if (x <= 0.0) return 1.0;
    if (x >= 8.0) return 0.0;
    double x2 = x * x;
    double x3 = x2 * x;
    return 1.0 / (1.0 + x + 0.5 * x2 + x3 / 6.0);
}

// 鈹€鈹€鈹€ Core predictor 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
//
// Key difference from the original ACoST implementation:
//   鈥?No LOW-RATE branch; always uses the vector motion predictor.
//   鈥?No atan2/sin/cos/exp in the hot path.  Turn detection is based on
//     dot/cross products, and the old CTRA circular-arc integration is replaced
//     with a turn-aware vector constant-acceleration predictor.
//   鈥?Damping still degrades acceleration to pure CV at large intervals, but
//     uses fastExpNeg() instead of std::exp().
//   鈥?Adaptive tail: for large avg_dt, use only the most recent 1-2 segments
//     instead of the full history, avoiding velocity averaging over stale data.
//
ACoSTCompressor::PredictionResult predictACoST(
    const std::vector<ACoSTCompressor::HistoryState>& history,
    double cur_ql, double cur_qa, uint64_t cur_ts,
    const ACoSTConfig& cfg,
    double lon_scale,
    double quant_step,
    uint64_t target_ts)
{
    const size_t n = history.size();
    if (n < 2) {
        return {cur_ql, cur_qa, ACoSTMotionModel::STOP};
    }
    const size_t num_seg = n - 1;

    // 鈹€鈹€ Convert history to local isotropic quant-unit coordinates 鈹€鈹€
    // Stack-allocated (n 鈮?8 so this is always small).
    double lx[9], ly[9];
    double vx[8], vy[8], sdt[8];

    for (size_t i = 0; i < n; ++i) {
        lx[i] = (history[i].q_lon - cur_ql) * lon_scale;
        ly[i] = history[i].q_lat - cur_qa;
    }
    for (size_t i = 0; i < num_seg; ++i) {
        int64_t ds = static_cast<int64_t>(history[i + 1].timestamp) -
                     static_cast<int64_t>(history[i].timestamp);
        double d = ds > 0 ? static_cast<double>(ds) : 1.0;
        vx[i]  = (lx[i + 1] - lx[i]) / d;
        vy[i]  = (ly[i + 1] - ly[i]) / d;
        sdt[i] = d;
    }

    // 鈹€鈹€ Average interval from recent segments 鈹€鈹€
    double avg_dt = 0.0;
    for (size_t i = 0; i < num_seg; ++i) avg_dt += sdt[i];
    avg_dt /= static_cast<double>(num_seg);

    int64_t dt_signed = static_cast<int64_t>(target_ts) -
                        static_cast<int64_t>(cur_ts);
    double dt = dt_signed > 0 ? static_cast<double>(dt_signed) : 1.0;

    // 鈹€鈹€ Adaptive tail: shrink at large intervals 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // At avg_dt 鈮?onset the full tail is used.
    // At 2脳onset only 1 segment is used (= most recent velocity only).
    // This prevents averaging stale velocities when a vehicle was doing
    // something different 40鈥?0 s ago.
    double onset = cfg.ctra_damp_onset;
    double tail_factor = onset / std::max(avg_dt, onset);  // 鈭?(0, 1]
    size_t sp_tail = static_cast<size_t>(
        std::max(1.0, std::ceil(tail_factor * cfg.speed_tail_segments)));
    size_t ac_tail = static_cast<size_t>(
        std::max(1.0, std::ceil(tail_factor * cfg.accel_tail_segments)));
    sp_tail = std::min(sp_tail, num_seg);
    ac_tail = std::min(ac_tail, num_seg);

    // 鈹€鈹€ Current and previous weighted velocity vectors 鈹€鈹€
    double avg_vx, avg_vy;
    wtailVecAvg(vx, vy, num_seg, sp_tail, avg_vx, avg_vy);
    double v_sq = avg_vx * avg_vx + avg_vy * avg_vy;

    // 鈹€鈹€ STOP detection 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // Use only the displacement-cell test.
    // A speed-threshold STOP would misfire on pedestrian/slow data (Geolife).
    double m_to_qu = 1.0 / (quant_step * kDegToRad * kEarthRadiusMeters);
    {
        double dx = lx[n - 1] - lx[n - 2];
        double dy = ly[n - 1] - ly[n - 2];
        if (dx * dx + dy * dy < lon_scale * lon_scale + 1.0) {
            return {cur_ql, cur_qa, ACoSTMotionModel::STOP};
        }
    }

    // 鈹€鈹€ Smooth low-cost damping 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // damp 鈫?1 at avg_dt 鈮?onset, damp 鈫?0 as avg_dt 鈫?鈭?
    // Use fastExpNeg() instead of std::exp() to avoid expensive transcendental
    // calls in the hot prediction path.
    double excess = std::max(0.0, avg_dt - onset);
    double tau    = std::max(1.0e-12, cfg.ctra_damp_tau);
    double damp   = fastExpNeg(excess / tau);

    double px = 0.0, py = 0.0;

    if (num_seg < 2 || ac_tail < 1) {
        // Constant velocity extrapolation (0 trig)
        px = avg_vx * dt;
        py = avg_vy * dt;
        if (!std::isfinite(px) || !std::isfinite(py)) {
            return {cur_ql, cur_qa, ACoSTMotionModel::CV};
        }
        double max_d = std::min(cfg.max_speed_ms * m_to_qu * dt,
                                cfg.max_cap_abs_m * m_to_qu);
        px = std::max(-max_d, std::min(px, max_d));
        py = std::max(-max_d, std::min(py, max_d));
        double pql = cur_ql + px / lon_scale;
        double pqa = cur_qa + py;
        return {pql, pqa, ACoSTMotionModel::CV};
    }

    // 鈹€鈹€ Previous velocity vector (for acceleration + turn detection) 鈹€鈹€
    double prev_vx, prev_vy;
    wtailVecAvg(vx, vy, num_seg - 1, ac_tail, prev_vx, prev_vy);

    double dt_hist = sdt[num_seg - 1];
    if (dt_hist <= 0.0) dt_hist = 1.0;

    // Cross and dot products.  No atan2 is needed:
    // tan(theta) = cross / dot, so compare cross虏 and tan虏(theta_min)*dot虏.
    double cross = prev_vx * avg_vy  - prev_vy * avg_vx;
    double dot_v = prev_vx * avg_vx  + prev_vy * avg_vy;

    // 鈹€鈹€ Check if turning with an angle threshold 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // dot_v <= 0 indicates a near-reversal or unreliable angle estimate.
    // Treat it as non-turning here and let vector acceleration handle the
    // velocity change, instead of over-triggering the old CTRA path.
    double turn_tan = std::max(0.0, cfg.ctra_turn_tan_min);
    bool is_turning = false;
    if (dot_v > 0.0) {
        double lhs = cross * cross;
        double rhs = turn_tan * turn_tan * dot_v * dot_v;
        is_turning = (lhs > rhs);
    }

    // 鈹€鈹€ Trig-free vector CA prediction 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // This replaces the old CTRA circular-arc integration.
    //
    // Raw vector acceleration:
    //   a = (v_cur - v_prev) / dt_hist
    //
    // For non-turning motion, remove the lateral component of acceleration to
    // avoid amplifying small GPS direction jitter.  For real turns, keep the
    // full vector acceleration.  Both branches use only arithmetic operations.
    double ax = (avg_vx - prev_vx) / dt_hist * damp;
    double ay = (avg_vy - prev_vy) / dt_hist * damp;

    if (!is_turning && v_sq > 1.0e-24) {
        // Project acceleration onto the current velocity direction.
        // a_parallel = ((a路v) / |v|虏) v
        double scale = (ax * avg_vx + ay * avg_vy) / v_sq;
        ax = scale * avg_vx;
        ay = scale * avg_vy;
    }

    double dt2 = dt * dt;
    px = avg_vx * dt + 0.5 * ax * dt2;
    py = avg_vy * dt + 0.5 * ay * dt2;

    if (!std::isfinite(px) || !std::isfinite(py)) {
        // Fallback to CV
        px = avg_vx * dt;
        py = avg_vy * dt;
    }

    double max_d = std::min(cfg.max_speed_ms * m_to_qu * dt,
                            cfg.max_cap_abs_m * m_to_qu);
    px = std::max(-max_d, std::min(px, max_d));
    py = std::max(-max_d, std::min(py, max_d));

    double pred_ql = cur_ql + px / lon_scale;
    double pred_qa = cur_qa + py;

    ACoSTMotionModel model = is_turning ? ACoSTMotionModel::CTRA
                                         : ACoSTMotionModel::CV;
    return {pred_ql, pred_qa, model};
}

}  // anonymous namespace


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
//  ACoSTCompressor
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

ACoSTCompressor::ACoSTCompressor(int block_size, double epsilon,
                                     const ACoSTConfig& config)
    : kBlockSize(block_size),
      kEpsilon(epsilon * 0.999),
      kQuantStep(2.0 * epsilon * 0.999),
      config_(config)
{
    int cap = std::max(4096, block_size * 128);
    output_bit_stream_ = std::make_unique<OutputBitStream>(cap);
    history_states_.reserve(kMaxHistorySize);
}

void ACoSTCompressor::ProcessFirstPoint(const GpsPoint& point) {
    first_point_ = false;

    compressed_size_in_bits_ += output_bit_stream_->WriteInt(kBlockSize, 16);
    compressed_size_in_bits_ += output_bit_stream_->WriteLong(
        Double::DoubleToLongBits(kEpsilon), 64);
    compressed_size_in_bits_ += output_bit_stream_->WriteLong(
        Double::DoubleToLongBits(point.longitude), 64);
    compressed_size_in_bits_ += output_bit_stream_->WriteLong(
        Double::DoubleToLongBits(point.latitude), 64);
    int ts_bits = output_bit_stream_->WriteLong(point.timestamp, 64);
    compressed_size_in_bits_ += ts_bits;
    stats_.timestamp_bits += ts_bits;

    // Keep the first raw point in fractional quantization units. The first
    // point is stored losslessly in the stream, so encoder and decoder share it.
    current_q_lon_ = point.longitude / kQuantStep;
    current_q_lat_ = point.latitude  / kQuantStep;
    current_ts_    = point.timestamp;
    lon_scale_     = std::max(1e-6,
                              std::abs(std::cos(point.latitude * (M_PI / 180.0))));
    history_states_.emplace_back(current_q_lon_, current_q_lat_, point.timestamp);
}

void ACoSTCompressor::AddGpsPoint(const GpsPoint& point) {
    stats_.total_points++;

    if (first_point_) {
        ProcessFirstPoint(point);
        return;
    }

    // Predict from reconstructed history first; quantize only the prediction
    // residual, rather than quantizing the input coordinate before prediction.
    double actual_ql = point.longitude / kQuantStep;
    double actual_qa = point.latitude  / kQuantStep;

    PredictionResult result = Predict(point.timestamp);

    // Encode timestamp (raw 64-bit).
    int ts_bits = output_bit_stream_->WriteLong(point.timestamp, 64);
    compressed_size_in_bits_ += ts_bits;
    stats_.timestamp_bits    += ts_bits;

    // 鈹€鈹€ ZP-selection for large-interval data 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // Use the current step dt (target_ts - current_ts) as the trigger, NOT
    // historyAvgDt.  historyAvgDt can spike at trajectory boundaries, causing
    // spurious flags on high-frequency data (Geolife ~1 s intervals).
    if (result.model != ACoSTMotionModel::STOP) {
        int64_t step_dt = static_cast<int64_t>(point.timestamp) -
                          static_cast<int64_t>(current_ts_);
        if (step_dt > static_cast<int64_t>(config_.zp_selection_onset)) {
            int zp_cost = egDimCost(static_cast<int64_t>(std::round(actual_ql - current_q_lon_)))
                        + egDimCost(static_cast<int64_t>(std::round(actual_qa - current_q_lat_)));
            int mp_cost = egDimCost(static_cast<int64_t>(std::round(actual_ql - result.q_lon)))
                        + egDimCost(static_cast<int64_t>(std::round(actual_qa - result.q_lat)));
            if (zp_cost < mp_cost) {
                // ZP wins: flag = 0, override prediction to current position
                compressed_size_in_bits_ += output_bit_stream_->WriteBit(false);
                result.q_lon = current_q_lon_;
                result.q_lat = current_q_lat_;
                result.model = ACoSTMotionModel::STOP;
            } else {
                // Motion predictor wins: flag = 1, keep result as-is
                compressed_size_in_bits_ += output_bit_stream_->WriteBit(true);
            }
        }
    }

    int64_t res_ql = static_cast<int64_t>(std::round(actual_ql - result.q_lon));
    int64_t res_qa = static_cast<int64_t>(std::round(actual_qa - result.q_lat));

    int bits_before = compressed_size_in_bits_;
    compressed_size_in_bits_ += EncodeQuantizedResidual(res_ql, res_qa);
    stats_.quantized_data_bits += (compressed_size_in_bits_ - bits_before);

    double recon_ql = result.q_lon + static_cast<double>(res_ql);
    double recon_qa = result.q_lat + static_cast<double>(res_qa);

    UpdateHistory(recon_ql, recon_qa, point.timestamp);
    current_q_lon_ = recon_ql;
    current_q_lat_ = recon_qa;
    current_ts_    = point.timestamp;
    prev_model_    = result.model;

    switch (result.model) {
    case ACoSTMotionModel::STOP: stats_.stop_count++; break;
    case ACoSTMotionModel::CV:   stats_.cv_count++;   break;
    case ACoSTMotionModel::CTRA: stats_.ctra_count++; break;
    }

    GpsPoint rp(recon_ql * kQuantStep, recon_qa * kQuantStep, point.timestamp);
    double err = CalculateDistance(point, rp);
    stats_.total_prediction_error += err;
    stats_.max_prediction_error    = std::max(stats_.max_prediction_error, err);
    stats_.prediction_errors.push_back(err);
}

ACoSTCompressor::PredictionResult
ACoSTCompressor::Predict(uint64_t target_ts) const {
    return predictACoST(history_states_, current_q_lon_, current_q_lat_,
                         current_ts_, config_, lon_scale_, kQuantStep, target_ts);
}

int ACoSTCompressor::EncodeQuantizedResidual(int64_t ql, int64_t qa) {
    uint64_t zl = static_cast<uint64_t>(ZigZagCodec::Encode(ql));
    uint64_t za = static_cast<uint64_t>(ZigZagCodec::Encode(qa));
    return EncodeValue(zl + 1) + EncodeValue(za + 1);
}

int ACoSTCompressor::EncodeValue(uint64_t value) {
    return EliasGammaCodec::Encode(static_cast<int64_t>(value),
                                   output_bit_stream_.get());
}

void ACoSTCompressor::UpdateHistory(double ql, double qa, uint64_t ts) {
    history_states_.emplace_back(ql, qa, ts);
    if (history_states_.size() > static_cast<size_t>(kMaxHistorySize))
        history_states_.erase(history_states_.begin());
}

double ACoSTCompressor::CalculateDistance(const GpsPoint& p1,
                                            const GpsPoint& p2) const {
    double dx = p1.longitude - p2.longitude;
    double dy = p1.latitude  - p2.latitude;
    return std::sqrt(dx * dx + dy * dy);
}

void ACoSTCompressor::Close() {
    output_bit_stream_->Flush();
    stats_.total_bits = compressed_size_in_bits_;
}

Array<uint8_t> ACoSTCompressor::GetCompressedData() const {
    int bytes = (compressed_size_in_bits_ + 7) / 8;
    return output_bit_stream_->GetBuffer(bytes);
}


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
//  ACoSTDecompressor
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

ACoSTDecompressor::ACoSTDecompressor(uint8_t* data, int size,
                                         const ACoSTConfig& config)
    : config_(config)
{
    input_bit_stream_ = std::make_unique<InputBitStream>(data, size);
    ReadHeader();
}

void ACoSTDecompressor::ReadHeader() {
    block_size_  = input_bit_stream_->ReadInt(16);
    epsilon_     = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
    quant_step_  = 2.0 * epsilon_;
}

bool ACoSTDecompressor::ReadNextPoint(GpsPoint& point) {
    if (first_point_) {
        first_point_ = false;
        points_read_ = 1;
        double lon = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
        double lat = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
        uint64_t ts = input_bit_stream_->ReadLong(64);
        point = GpsPoint(lon, lat, ts);
        current_q_lon_ = lon / quant_step_;
        current_q_lat_ = lat / quant_step_;
        current_ts_    = ts;
        lon_scale_     = std::max(1e-6,
                                  std::abs(std::cos(lat * (M_PI / 180.0))));
        history_states_.emplace_back(current_q_lon_, current_q_lat_, ts);
        return true;
    }

    try {
        uint64_t ts = input_bit_stream_->ReadLong(64);
        ACoSTCompressor::PredictionResult result = Predict(ts);

        // 鈹€鈹€ ZP-selection mirror (must match compressor logic exactly) 鈹€鈹€鈹€鈹€鈹€鈹€
        if (result.model != ACoSTMotionModel::STOP) {
            int64_t step_dt = static_cast<int64_t>(ts) -
                              static_cast<int64_t>(current_ts_);
            if (step_dt > static_cast<int64_t>(config_.zp_selection_onset)) {
                uint32_t flag = input_bit_stream_->ReadBit();
                if (flag == 0) {
                    result.q_lon = current_q_lon_;
                    result.q_lat = current_q_lat_;
                    result.model = ACoSTMotionModel::STOP;
                }
            }
        }

        int64_t res_ql = 0, res_qa = 0;
        DecodeQuantizedResidual(res_ql, res_qa);

        double recon_ql = result.q_lon + static_cast<double>(res_ql);
        double recon_qa = result.q_lat + static_cast<double>(res_qa);

        UpdateHistory(recon_ql, recon_qa, ts);
        current_q_lon_ = recon_ql;
        current_q_lat_ = recon_qa;
        current_ts_    = ts;
        prev_model_    = result.model;

        point = GpsPoint(recon_ql * quant_step_, recon_qa * quant_step_, ts);
        ++points_read_;
        return true;
    } catch (...) {
        return false;
    }
}

std::vector<ACoSTDecompressor::GpsPoint>
ACoSTDecompressor::ReadAllPoints() {
    std::vector<GpsPoint> pts;
    GpsPoint p;
    while (ReadNextPoint(p)) pts.push_back(p);
    return pts;
}

ACoSTCompressor::PredictionResult
ACoSTDecompressor::Predict(uint64_t target_ts) const {
    return predictACoST(history_states_, current_q_lon_, current_q_lat_,
                         current_ts_, config_, lon_scale_, quant_step_, target_ts);
}

void ACoSTDecompressor::DecodeQuantizedResidual(int64_t& ql, int64_t& qa) {
    int64_t zl = DecodeValue() - 1;
    int64_t za = DecodeValue() - 1;
    ql = ZigZagCodec::Decode(static_cast<uint64_t>(zl));
    qa = ZigZagCodec::Decode(static_cast<uint64_t>(za));
}

int64_t ACoSTDecompressor::DecodeValue() {
    return EliasGammaCodec::Decode(input_bit_stream_.get());
}

void ACoSTDecompressor::UpdateHistory(double ql, double qa, uint64_t ts) {
    history_states_.emplace_back(ql, qa, ts);
    if (history_states_.size() > static_cast<size_t>(kMaxHistorySize))
        history_states_.erase(history_states_.begin());
}
  
