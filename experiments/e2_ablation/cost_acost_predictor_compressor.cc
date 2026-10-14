#include "experiments/e2_ablation/cost_acost_predictor_compressor.h"
#include "utils/elias_gamma_codec.h"
#include "utils/zig_zag_codec.h"
#include "utils/input_bit_stream.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace {
constexpr double kAcostPi = 3.14159265358979323846;
constexpr double kAcostEarthRadiusMeters = 6371000.0;
constexpr double kAcostTurnTanMin = 0.08748866352592401;  // tan(5 degrees)
constexpr double kAcostAdaptiveTailOnset = 100.0;          // 10 seconds in 0.1 s units
constexpr int kAcostSpeedTailSegments = 3;
constexpr int kAcostAccelTailSegments = 2;
constexpr double kAcostMaxSpeedMetersPerSecond = 200.0;
constexpr double kAcostMaxDisplacementMeters = 50000.0;

inline void weightedTailAverage(const double* vx, const double* vy,
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

CoSTCompressor::GpsPoint predictWithAcostMotion(
    const std::vector<CoSTCompressor::HistoryState>& history,
    const CoSTCompressor::GpsPoint& current,
    uint64_t target_timestamp,
    double quant_step) {
    const size_t n = history.size();
    if (n < 2) return CoSTCompressor::GpsPoint(
        current.longitude, current.latitude, target_timestamp);

    const size_t num_segments = n - 1;
    const double lon_scale = std::max(1.0e-6,
        std::abs(std::cos(current.latitude * (kAcostPi / 180.0))));
    const int64_t current_q_lon = static_cast<int64_t>(std::round(current.longitude / quant_step));
    const int64_t current_q_lat = static_cast<int64_t>(std::round(current.latitude / quant_step));
    double local_x[8], local_y[8], vx[7], vy[7], segment_dt[7];

    for (size_t i = 0; i < n; ++i) {
        const int64_t q_lon = static_cast<int64_t>(
            std::round(history[i].reconstructed_point.longitude / quant_step));
        const int64_t q_lat = static_cast<int64_t>(
            std::round(history[i].reconstructed_point.latitude / quant_step));
        local_x[i] = static_cast<double>(q_lon - current_q_lon) * lon_scale;
        local_y[i] = static_cast<double>(q_lat - current_q_lat);
    }
    double avg_dt = 0.0;
    for (size_t i = 0; i < num_segments; ++i) {
        const int64_t delta_ts = static_cast<int64_t>(history[i + 1].reconstructed_point.timestamp) -
                                 static_cast<int64_t>(history[i].reconstructed_point.timestamp);
        const double dt = delta_ts > 0 ? static_cast<double>(delta_ts) : 1.0;
        vx[i] = (local_x[i + 1] - local_x[i]) / dt;
        vy[i] = (local_y[i + 1] - local_y[i]) / dt;
        segment_dt[i] = dt;
        avg_dt += dt;
    }
    avg_dt /= static_cast<double>(num_segments);

    const int64_t delta_ts = static_cast<int64_t>(target_timestamp) -
                             static_cast<int64_t>(current.timestamp);
    const double dt = delta_ts > 0 ? static_cast<double>(delta_ts) : 1.0;
    const double tail_factor = kAcostAdaptiveTailOnset /
        std::max(avg_dt, kAcostAdaptiveTailOnset);
    const size_t speed_tail = std::min(num_segments, static_cast<size_t>(
        std::max(1.0, std::ceil(tail_factor * kAcostSpeedTailSegments))));
    const size_t accel_tail = std::min(num_segments, static_cast<size_t>(
        std::max(1.0, std::ceil(tail_factor * kAcostAccelTailSegments))));

    double avg_vx, avg_vy;
    weightedTailAverage(vx, vy, num_segments, speed_tail, avg_vx, avg_vy);

    double px = avg_vx * dt;
    double py = avg_vy * dt;
    if (num_segments >= 2) {
        double previous_vx, previous_vy;
        weightedTailAverage(vx, vy, num_segments - 1, accel_tail,
                            previous_vx, previous_vy);
        double dt_history = segment_dt[num_segments - 1];
        if (dt_history <= 0.0) dt_history = 1.0;

        const double cross = previous_vx * avg_vy - previous_vy * avg_vx;
        const double dot = previous_vx * avg_vx + previous_vy * avg_vy;
        const bool is_turning = dot > 0.0 &&
            cross * cross > kAcostTurnTanMin * kAcostTurnTanMin * dot * dot;

        // ACoST turn-aware vector acceleration. Damping is deliberately omitted.
        double ax = (avg_vx - previous_vx) / dt_history;
        double ay = (avg_vy - previous_vy) / dt_history;
        const double speed_sq = avg_vx * avg_vx + avg_vy * avg_vy;
        if (!is_turning && speed_sq > 1.0e-24) {
            const double projection = (ax * avg_vx + ay * avg_vy) / speed_sq;
            ax = projection * avg_vx;
            ay = projection * avg_vy;
        }
        const double dt2_half = 0.5 * dt * dt;
        px = avg_vx * dt + ax * dt2_half;
        py = avg_vy * dt + ay * dt2_half;
    }

    if (!std::isfinite(px) || !std::isfinite(py)) {
        px = avg_vx * dt;
        py = avg_vy * dt;
    }
    const double meters_to_quant_units =
        1.0 / (quant_step * (kAcostPi / 180.0) * kAcostEarthRadiusMeters);
    const double max_displacement = std::min(
        kAcostMaxSpeedMetersPerSecond * meters_to_quant_units * dt,
        kAcostMaxDisplacementMeters * meters_to_quant_units);
    px = std::max(-max_displacement, std::min(px, max_displacement));
    py = std::max(-max_displacement, std::min(py, max_displacement));

    const int64_t predicted_q_lon = current_q_lon +
        static_cast<int64_t>(std::round(px / lon_scale));
    const int64_t predicted_q_lat = current_q_lat +
        static_cast<int64_t>(std::round(py));
    return CoSTCompressor::GpsPoint(
        predicted_q_lon * quant_step, predicted_q_lat * quant_step, target_timestamp);
}
}  // namespace


// ==================== CoST Compressor Implementation ====================

CoSTCompressor::CoSTCompressor(
    int block_size, double epsilon, int evaluation_window, 
    bool use_time_window, uint64_t time_window_seconds)
    : kBlockSize(block_size), 
      kEpsilon(epsilon * 0.999), 
      kQuantStep(2 * epsilon * 0.999),
      kEvaluationWindow(evaluation_window),
      use_time_window_(use_time_window),
      kTimeWindowSeconds(time_window_seconds) {
        int initial_capacity_bytes = std::max(4096, block_size * 128);
        output_bit_stream_ = std::make_unique<OutputBitStream>(initial_capacity_bytes);
    history_states_.reserve(kMaxHistorySize);
    
 // Huffman （）
    predictor_frequency_[PREDICTOR_LDR] = 60;  // LDR
    predictor_frequency_[PREDICTOR_CP] = 10;   // CP
    predictor_frequency_[PREDICTOR_ZP] = 30;   // ZP
    UpdateHuffmanCodes();
}

void CoSTCompressor::AddGpsPoint(const GpsPoint& point) {
    stats_.total_points++;
    
    if (first_point_) {
        ProcessFirstPoint(point);
        return;
    }
    
 // === 1. （） ===
    GpsPoint pred_ldr, pred_cp, pred_zp;
    ParallelPredict(pred_ldr, pred_cp, pred_zp, point.timestamp);
    
    // (comment removed)
    GpsPoint error_ldr = point - pred_ldr;
    GpsPoint error_cp = point - pred_cp;
    GpsPoint error_zp = point - pred_zp;
    
    int cost_ldr_error = EstimateErrorEncodingCost(error_ldr);
    int cost_cp_error = EstimateErrorEncodingCost(error_cp);
    int cost_zp_error = EstimateErrorEncodingCost(error_zp);
    
 // （：Huffman + ）
    int cost_ldr_total = GetHuffmanBitCost(PREDICTOR_LDR) + cost_ldr_error;
    int cost_cp_total = GetHuffmanBitCost(PREDICTOR_CP) + cost_cp_error;
    int cost_zp_total = GetHuffmanBitCost(PREDICTOR_ZP) + cost_zp_error;
    
    PredictorType best_predictor;
    GpsPoint best_prediction;
    int best_cost;
    
    if (cost_ldr_total <= cost_cp_total && cost_ldr_total <= cost_zp_total) {
        best_predictor = PREDICTOR_LDR;
        best_prediction = pred_ldr;
        best_cost = cost_ldr_total;
    } else if (cost_cp_total <= cost_zp_total) {
        best_predictor = PREDICTOR_CP;
        best_prediction = pred_cp;
        best_cost = cost_cp_total;
    } else {
        best_predictor = PREDICTOR_ZP;
        best_prediction = pred_zp;
        best_cost = cost_zp_total;
    }
    
 // === 2. （） ===
    int multi_model_cost = best_cost;                    // ：
    int ldr_only_model_cost = cost_ldr_error;            // LDR-Only：LDR（）
    
 // （）
    UpdateCostWindows(multi_model_cost, ldr_only_model_cost, point.timestamp);
    
 // === 3. ===
    if (current_mode_ == MODE_LDR_ONLY) {
        EncodeLDROnly(point);
        stats_.ldr_only_mode_points++;
    } else {  // MODE_MULTI_PREDICTOR
        EncodeMultiPredictor(point);
        stats_.multi_predictor_mode_points++;
    }
    
 // === 4. ===
    bool should_evaluate = false;
    
    if (use_time_window_) {
        // (comment removed)
        if (last_evaluation_timestamp_ == 0) {
 // ：
            last_evaluation_timestamp_ = point.timestamp;
        }
        
        uint64_t time_since_last_eval = point.timestamp - last_evaluation_timestamp_;
        
 // （），
        if (point.timestamp >= last_evaluation_timestamp_ && 
            time_since_last_eval >= kTimeWindowSeconds) {
            should_evaluate = true;
            last_evaluation_timestamp_ = point.timestamp;
        }
    } else {
 // （）
        if (stats_.total_points % kEvaluationWindow == 0) {
            should_evaluate = true;
        }
    }
    
    if (should_evaluate) {
 // cost window，；
        size_t min_window_size = use_time_window_ ? 5 : kEvaluationWindow;  // 5
        if (point_costs_multi_.size() >= min_window_size) {
            EvaluateAndSwitchModeBasedOnCost();
        }
        
 // ，1（）
        // 0 = Multi-Predictor, 1 = LDR-Only
        bool mode_bit = (current_mode_ == MODE_LDR_ONLY);
        output_bit_stream_->WriteBit(mode_bit);
        compressed_size_in_bits_ += 1;
    }
}

void CoSTCompressor::EncodeMultiPredictor(const GpsPoint& point) {
 // （timestamp）
    GpsPoint pred_ldr, pred_cp, pred_zp;
    ParallelPredict(pred_ldr, pred_cp, pred_zp, point.timestamp);
    
 // （）
    GpsPoint best_prediction;
    int best_cost;
    PredictorType best_predictor = SelectBestPredictorByCost(
        point, pred_ldr, pred_cp, pred_zp, best_prediction, best_cost);
    
 // 、timestamp
    EncodePrediction(best_predictor, point, best_prediction);
    
    // (comment removed)
    last_used_predictor_ = best_predictor;
    
    // (comment removed)
    switch (best_predictor) {
        case PREDICTOR_LDR: stats_.ldr_count++; break;
        case PREDICTOR_CP: stats_.cp_count++; break;
        case PREDICTOR_ZP: stats_.zp_count++; break;
    }
}

void CoSTCompressor::EncodeLDROnly(const GpsPoint& point) {
 // LDR
    GpsPoint pred_ldr, pred_cp, pred_zp;
    ParallelPredict(pred_ldr, pred_cp, pred_zp, point.timestamp);
    
 // LDR-Only：，timestamp
 // 1. timestamp delta（TrajSP，）
 // ：timestamp（），int64_tdelta
    int64_t timestamp_delta_signed = static_cast<int64_t>(point.timestamp) - static_cast<int64_t>(current_reconstructed_point_.timestamp);
    uint64_t timestamp_delta = static_cast<uint64_t>(timestamp_delta_signed);
    int ts_bits = output_bit_stream_->WriteLong(timestamp_delta, 64);
    compressed_size_in_bits_ += ts_bits;
    stats_.timestamp_bits += ts_bits;
    
 // 2. LDR
    GpsPoint delta = point - pred_ldr;
    
    // (comment removed)
    int64_t quantized_delta_lon = static_cast<int64_t>(std::round(delta.longitude / kQuantStep));
    int64_t quantized_delta_lat = static_cast<int64_t>(std::round(delta.latitude / kQuantStep));
    
 // （ZigZag + Elias Gamma）
    int bits_before = compressed_size_in_bits_;
    compressed_size_in_bits_ += EliasGammaCodec::Encode(
        ZigZagCodec::Encode(quantized_delta_lon) + 1, output_bit_stream_.get());
    compressed_size_in_bits_ += EliasGammaCodec::Encode(
        ZigZagCodec::Encode(quantized_delta_lat) + 1, output_bit_stream_.get());
    stats_.quantized_data_bits += (compressed_size_in_bits_ - bits_before);
    
 // （）
    GpsPoint reconstructed_delta(
        quantized_delta_lon * kQuantStep,
        quantized_delta_lat * kQuantStep,
        0
    );
    GpsPoint reconstructed_point = pred_ldr + reconstructed_delta;
    reconstructed_point.timestamp = point.timestamp;  // 
    
    // (comment removed)
    UpdateReconstructedState(reconstructed_point);
    
    // (comment removed)
    stats_.ldr_count++;
    double error = CalculateDistance(point, reconstructed_point);
    stats_.total_prediction_error += error;
    stats_.max_prediction_error = std::max(stats_.max_prediction_error, error);
    stats_.prediction_errors.push_back(error);
}

// (comment removed)

// 1，EncodeModeSwitch

CoSTCompressor::PredictorType CoSTCompressor::SelectBestPredictorByCost(
    const GpsPoint& current_point,
    const GpsPoint& pred_ldr,
    const GpsPoint& pred_cp,
    const GpsPoint& pred_zp,
    GpsPoint& best_prediction,
    int& best_cost) {
    
    // (comment removed)
    GpsPoint error_ldr = current_point - pred_ldr;
    GpsPoint error_cp = current_point - pred_cp;
    GpsPoint error_zp = current_point - pred_zp;
    
 // （Huffman + ）
    int cost_ldr = GetHuffmanBitCost(PREDICTOR_LDR) + EstimateErrorEncodingCost(error_ldr);
    int cost_cp = GetHuffmanBitCost(PREDICTOR_CP) + EstimateErrorEncodingCost(error_cp);
    int cost_zp = GetHuffmanBitCost(PREDICTOR_ZP) + EstimateErrorEncodingCost(error_zp);
    
    // (comment removed)
    if (cost_ldr <= cost_cp && cost_ldr <= cost_zp) {
        best_prediction = pred_ldr;
        best_cost = cost_ldr;
        return PREDICTOR_LDR;
    } else if (cost_cp <= cost_zp) {
        best_prediction = pred_cp;
        best_cost = cost_cp;
        return PREDICTOR_CP;
    } else {
        best_prediction = pred_zp;
        best_cost = cost_zp;
        return PREDICTOR_ZP;
    }
}

int CoSTCompressor::EstimateEliasGammaBits(int64_t value) const {
    if (value <= 0) return 1;  // 
    
 // __builtin_clzll (count leading zeros) log2 
    // log2(value) = 63 - clz(value)
    // Elias Gamma bits = 2 * log2(value) + 1
    int log2_val = 63 - __builtin_clzll(static_cast<uint64_t>(value));
    return 2 * log2_val + 1;
}

int CoSTCompressor::EstimateErrorEncodingCost(const GpsPoint& error) const {
    // (comment removed)
    int64_t quantized_lon = static_cast<int64_t>(std::round(error.longitude / kQuantStep));
    int64_t quantized_lat = static_cast<int64_t>(std::round(error.latitude / kQuantStep));
    
 // ZigZag
    uint64_t zigzag_lon = ZigZagCodec::Encode(quantized_lon);
    uint64_t zigzag_lat = ZigZagCodec::Encode(quantized_lat);
    
 // Elias Gamma
    int bits_lon = EstimateEliasGammaBits(zigzag_lon + 1);
    int bits_lat = EstimateEliasGammaBits(zigzag_lat + 1);
    
    return bits_lon + bits_lat;
}

// ========== ==========

void CoSTCompressor::UpdateCostWindows(int multi_cost, int ldr_only_cost, uint64_t timestamp) {
 // （）
    point_costs_multi_.push_back(CostRecord(multi_cost, timestamp));
    window_total_cost_multi_ += multi_cost;
    
    point_costs_ldr_only_.push_back(CostRecord(ldr_only_cost, timestamp));
    window_total_cost_ldr_only_ += ldr_only_cost;
    
    if (use_time_window_) {
 // ：
        uint64_t window_start_time = timestamp > kTimeWindowSeconds ? 
                                      timestamp - kTimeWindowSeconds : 0;
        
        while (!point_costs_multi_.empty() && 
               point_costs_multi_.front().timestamp < window_start_time) {
            window_total_cost_multi_ -= point_costs_multi_.front().cost;
            point_costs_multi_.pop_front();
        }
        
        while (!point_costs_ldr_only_.empty() && 
               point_costs_ldr_only_.front().timestamp < window_start_time) {
            window_total_cost_ldr_only_ -= point_costs_ldr_only_.front().cost;
            point_costs_ldr_only_.pop_front();
        }
    } else {
 // ：（）
        if (point_costs_multi_.size() > static_cast<size_t>(kEvaluationWindow)) {
            window_total_cost_multi_ -= point_costs_multi_.front().cost;
            point_costs_multi_.pop_front();
            
            window_total_cost_ldr_only_ -= point_costs_ldr_only_.front().cost;
            point_costs_ldr_only_.pop_front();
        }
    }
}

// ========== （） ==========

void CoSTCompressor::EvaluateAndSwitchModeBasedOnCost() {
    // (comment removed)
    if (point_costs_multi_.size() < static_cast<size_t>(kEvaluationWindow)) return;
    
 // 1（）
    const int kActualSwitchCost = 1;
    
    if (current_mode_ == MODE_MULTI_PREDICTOR) {
 // ：（1）
        if (window_total_cost_ldr_only_ < window_total_cost_multi_ - kActualSwitchCost) {
            current_mode_ = MODE_LDR_ONLY;
            stats_.mode_switch_count++;
 // kClearWindowAfterSwitch false，
        }
    } else {  // current_mode_ == MODE_LDR_ONLY
 // ：（1）
        if (window_total_cost_multi_ < window_total_cost_ldr_only_ - kActualSwitchCost) {
            current_mode_ = MODE_MULTI_PREDICTOR;
            stats_.mode_switch_count++;
 // kClearWindowAfterSwitch false，
        }
    }
}

// ：

// ========== Huffman ==========

int CoSTCompressor::GetHuffmanBitCost(PredictorType predictor) const {
    return huffman_codes_[predictor].length;
}

void CoSTCompressor::UpdateHuffmanCodes() {
    // (comment removed)
    struct PredictorFreq {
        PredictorType type;
        int frequency;
    };
    
    std::vector<PredictorFreq> freq_list = {
        {PREDICTOR_LDR, predictor_frequency_[PREDICTOR_LDR]},
        {PREDICTOR_CP, predictor_frequency_[PREDICTOR_CP]},
        {PREDICTOR_ZP, predictor_frequency_[PREDICTOR_ZP]}
    };
    
 // ：
    std::sort(freq_list.begin(), freq_list.end(), 
              [](const PredictorFreq& a, const PredictorFreq& b) {
                  if (a.frequency != b.frequency) {
                      return a.frequency > b.frequency;
                  }
 // ，
                  return static_cast<int>(a.type) < static_cast<int>(b.type);
              });
    
    
 // Huffman：0 (1 bit)，10 (2 bits)，11 (2 bits)
 // ，
    huffman_codes_[freq_list[0].type] = HuffmanCode({false});                      // 0 (1 bit)
    huffman_codes_[freq_list[1].type] = HuffmanCode({true, false});                // 10 (2 bits)
    huffman_codes_[freq_list[2].type] = HuffmanCode({true, true});                 // 11 (2 bits)
}

void CoSTCompressor::EncodeWithHuffman(PredictorType predictor) {
    const HuffmanCode& code = huffman_codes_[predictor];
    for (bool bit : code.bits) {
        output_bit_stream_->WriteBit(bit);
    }
    compressed_size_in_bits_ += code.length;
    
    // (comment removed)
    AddPredictorToWindow(predictor);
}

void CoSTCompressor::AddPredictorToWindow(PredictorType predictor) {
    predictor_window_.push_back(predictor);
    predictor_frequency_[predictor]++;
    
    if (predictor_window_.size() > kSlidingWindowSize) {
        PredictorType old_predictor = predictor_window_.front();
        predictor_window_.pop_front();
        predictor_frequency_[old_predictor]--;
    }
    
 // Huffman
    if (predictor_window_.size() % 100 == 0) {
        UpdateHuffmanCodes();
    }
}

void CoSTCompressor::Close() {
    output_bit_stream_->Flush();
    stats_.total_bits = compressed_size_in_bits_;
}

Array<uint8_t> CoSTCompressor::GetCompressedData() {
    int byte_length = (compressed_size_in_bits_ + 7) / 8;
    return output_bit_stream_->GetBuffer(byte_length);
}

void CoSTCompressor::ProcessFirstPoint(const GpsPoint& point) {
    first_point_ = false;
    
    // (comment removed)
    compressed_size_in_bits_ += output_bit_stream_->WriteInt(kBlockSize, 16);
    compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(kEpsilon), 64);
    compressed_size_in_bits_ += output_bit_stream_->WriteInt(kEvaluationWindow, 16);  // （）
    
 // （）
    compressed_size_in_bits_ += output_bit_stream_->WriteBit(use_time_window_);  // 1：
    if (use_time_window_) {
        compressed_size_in_bits_ += output_bit_stream_->WriteInt(kTimeWindowSeconds, 32);  // 32：（）
    }
    
    // (comment removed)
    compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(point.longitude), 64);
    compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(point.latitude), 64);
    
 // timestamp（64）
    int ts_bits = output_bit_stream_->WriteLong(point.timestamp, 64);
    compressed_size_in_bits_ += ts_bits;
    stats_.timestamp_bits += ts_bits;
    
    // (comment removed)
    current_reconstructed_point_ = point;
    history_states_.emplace_back(point, GpsPoint(0, 0, 0));  // 0
    
 // （）
    if (use_time_window_) {
        last_evaluation_timestamp_ = point.timestamp;
    }
}

void CoSTCompressor::ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp) {
    pred_zp = current_reconstructed_point_;
    if (history_states_.size() < 2) {
        pred_ldr = current_reconstructed_point_;
        pred_cp = current_reconstructed_point_;
        return;
    }
    const int64_t delta_time_signed = static_cast<int64_t>(current_timestamp) -
                                      static_cast<int64_t>(current_reconstructed_point_.timestamp);
    const double dt = delta_time_signed > 0 ? static_cast<double>(delta_time_signed) : 1.0;
    const GpsPoint velocity = history_states_.back().velocity;
    pred_ldr = GpsPoint(current_reconstructed_point_.longitude + velocity.longitude * dt,
                        current_reconstructed_point_.latitude + velocity.latitude * dt,
                        current_timestamp);
    pred_cp = predictWithAcostMotion(history_states_, current_reconstructed_point_,
                                      current_timestamp, kQuantStep);
}

void CoSTCompressor::EncodePrediction(PredictorType predictor,
                                                       const GpsPoint& current_point,
                                                       const GpsPoint& predicted_point) {
 // 1. （Huffman）
    int bits_before_flag = compressed_size_in_bits_;
    EncodeWithHuffman(predictor);
    stats_.predictor_flag_bits += (compressed_size_in_bits_ - bits_before_flag);
    
 // 2. timestamp delta（TrajSP：Huffman，）
 // ：timestamp（），int64_tdelta，uint64_t
    int64_t timestamp_delta_signed = static_cast<int64_t>(current_point.timestamp) - static_cast<int64_t>(current_reconstructed_point_.timestamp);
    uint64_t timestamp_delta = static_cast<uint64_t>(timestamp_delta_signed);  // bit pattern
    int ts_bits = output_bit_stream_->WriteLong(timestamp_delta, 64);
    compressed_size_in_bits_ += ts_bits;
    stats_.timestamp_bits += ts_bits;
    
 // 3. 
    GpsPoint delta = current_point - predicted_point;
    
    // (comment removed)
    int64_t quantized_delta_lon = static_cast<int64_t>(std::round(delta.longitude / kQuantStep));
    int64_t quantized_delta_lat = static_cast<int64_t>(std::round(delta.latitude / kQuantStep));
    
    // (comment removed)
    int bits_before_data = compressed_size_in_bits_;
    compressed_size_in_bits_ += EliasGammaCodec::Encode(
        ZigZagCodec::Encode(quantized_delta_lon) + 1, output_bit_stream_.get());
    compressed_size_in_bits_ += EliasGammaCodec::Encode(
        ZigZagCodec::Encode(quantized_delta_lat) + 1, output_bit_stream_.get());
    stats_.quantized_data_bits += (compressed_size_in_bits_ - bits_before_data);
    
 // （）
    GpsPoint reconstructed_delta(
        quantized_delta_lon * kQuantStep,
        quantized_delta_lat * kQuantStep,
        0
    );
    GpsPoint reconstructed_point = predicted_point + reconstructed_delta;
    reconstructed_point.timestamp = current_point.timestamp;  // 
    
    // (comment removed)
    UpdateReconstructedState(reconstructed_point);
    
    // (comment removed)
    double error = CalculateDistance(current_point, reconstructed_point);
    stats_.total_prediction_error += error;
    stats_.max_prediction_error = std::max(stats_.max_prediction_error, error);
    stats_.prediction_errors.push_back(error);
}

void CoSTCompressor::UpdateHistory(const GpsPoint& reconstructed_point) {
    GpsPoint velocity(0, 0, 0);
    
    if (!history_states_.empty()) {
        const GpsPoint& prev_point = history_states_.back().reconstructed_point;
        int64_t delta_time_signed = static_cast<int64_t>(reconstructed_point.timestamp) - static_cast<int64_t>(prev_point.timestamp);
        
        if (delta_time_signed > 0) {
 // （/）
            double dt = static_cast<double>(delta_time_signed);
            velocity = GpsPoint(
                (reconstructed_point.longitude - prev_point.longitude) / dt,
                (reconstructed_point.latitude - prev_point.latitude) / dt,
                0
            );
        }
    }
    
    history_states_.emplace_back(reconstructed_point, velocity);
    
    if (history_states_.size() > kMaxHistorySize) {
        history_states_.erase(history_states_.begin());
    }
}

void CoSTCompressor::UpdateReconstructedState(const GpsPoint& reconstructed_point) {
    UpdateHistory(reconstructed_point);
    current_reconstructed_point_ = reconstructed_point;
}

double CoSTCompressor::CalculateDistance(const GpsPoint& p1, const GpsPoint& p2) const {
    double dx = p1.longitude - p2.longitude;
    double dy = p1.latitude - p2.latitude;
    return std::sqrt(dx * dx + dy * dy);
}

// (comment removed)
void CoSTCompressor::CompressionStats::PrintStats() const {
    std::cout << "\n=== TrajCompress-SP-Adaptive  ===" << std::endl;
    std::cout << ": " << total_points << std::endl;
    std::cout << ": LDR=" << ldr_count << ", CP=" << cp_count << ", ZP=" << zp_count << std::endl;
    std::cout << ": " << mode_switch_count << std::endl;
    std::cout << "LDR-Only: " << ldr_only_mode_points << " (" 
              << (100.0 * ldr_only_mode_points / total_points) << "%)" << std::endl;
    std::cout << "Multi-Predictor: " << multi_predictor_mode_points << " (" 
              << (100.0 * multi_predictor_mode_points / total_points) << "%)" << std::endl;
    std::cout << ": " << total_bits << std::endl;
    std::cout << "  : " << predictor_flag_bits << " bits" << std::endl;
    std::cout << "  : " << mode_switch_bits << " bits" << std::endl;
    std::cout << "  : " << quantized_data_bits << " bits" << std::endl;
}

void CoSTCompressor::CompressionStats::PrintDetailedStats() const {
    PrintStats();
    
    if (!prediction_errors.empty()) {
        std::vector<double> sorted_errors = prediction_errors;
        std::sort(sorted_errors.begin(), sorted_errors.end());
        
        size_t p50 = sorted_errors.size() * 0.50;
        size_t p90 = sorted_errors.size() * 0.90;
        size_t p99 = sorted_errors.size() * 0.99;
        
        std::cout << "\n:" << std::endl;
        std::cout << "  : " << (total_prediction_error / prediction_errors.size()) << std::endl;
        std::cout << "  : " << max_prediction_error << std::endl;
        std::cout << "  P50: " << sorted_errors[p50] << std::endl;
        std::cout << "  P90: " << sorted_errors[p90] << std::endl;
        std::cout << "  P99: " << sorted_errors[p99] << std::endl;
    }
}

// ==================== ====================

CoSTDecompressor::CoSTDecompressor(uint8_t* compressed_data, int data_size) {
    input_bit_stream_ = std::make_unique<InputBitStream>(compressed_data, data_size);
    ReadHeader();
}

void CoSTDecompressor::ReadHeader() {
    block_size_ = input_bit_stream_->ReadInt(16);
    epsilon_ = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
    evaluation_window_ = input_bit_stream_->ReadInt(16);  // （）
    
 // （）
    use_time_window_ = input_bit_stream_->ReadBit();  // 1：
    if (use_time_window_) {
        time_window_seconds_ = input_bit_stream_->ReadInt(32);  // 32：（）
    } else {
        time_window_seconds_ = 0;
    }
    
    quant_step_ = 2 * epsilon_;  // epsilon  0.999 
    
 // Huffman
    predictor_frequency_[PredictorType::PREDICTOR_LDR] = 60;
    predictor_frequency_[PredictorType::PREDICTOR_CP] = 10;
    predictor_frequency_[PredictorType::PREDICTOR_ZP] = 30;
    UpdateHuffmanDecoder();
}

bool CoSTDecompressor::ReadNextPoint(GpsPoint& point) {
    if (first_point_) {
        first_point_ = false;
        points_read_ = 1;  // 
        
        // (comment removed)
        double lon = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
        double lat = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
        
 // timestamp
        uint64_t timestamp = input_bit_stream_->ReadLong(64);
        
        point = GpsPoint(lon, lat, timestamp);
        
        current_reconstructed_point_ = point;
        history_states_.emplace_back(point, GpsPoint(0, 0, 0));
        
 // （）
        if (use_time_window_) {
            last_evaluation_timestamp_ = timestamp;
        }
        
        return true;
    }
    
 // （TrajSP）
    GpsPoint predicted_point;
    uint64_t current_timestamp;
    
    if (current_mode_ == CompressionMode::MODE_LDR_ONLY) {
 // LDR-Only：，timestamp
 // 1. timestamp delta (uint64_t，int64_t)
        uint64_t timestamp_delta_bits = input_bit_stream_->ReadLong(64);
        int64_t timestamp_delta = static_cast<int64_t>(timestamp_delta_bits);
        current_timestamp = current_reconstructed_point_.timestamp + timestamp_delta;
        
 // 2. timestampLDR
        GpsPoint pred_ldr, pred_cp, pred_zp;
        ParallelPredict(pred_ldr, pred_cp, pred_zp, current_timestamp);
        predicted_point = pred_ldr;
        
    } else {
 // Multi-Predictor：Huffman，timestamp，（TrajSP）
 // 1. 
        PredictorType predictor = DecodeWithHuffman();
        
 // 2. timestamp delta（Huffman，）(uint64_t，int64_t)
        uint64_t timestamp_delta_bits = input_bit_stream_->ReadLong(64);
        int64_t timestamp_delta = static_cast<int64_t>(timestamp_delta_bits);
        current_timestamp = current_reconstructed_point_.timestamp + timestamp_delta;
        
 // 3. timestamp
        GpsPoint pred_ldr, pred_cp, pred_zp;
        ParallelPredict(pred_ldr, pred_cp, pred_zp, current_timestamp);
        
        switch (predictor) {
            case PredictorType::PREDICTOR_LDR: predicted_point = pred_ldr; break;
            case PredictorType::PREDICTOR_CP: predicted_point = pred_cp; break;
            case PredictorType::PREDICTOR_ZP: predicted_point = pred_zp; break;
        }
        last_used_predictor_ = predictor;
    }
    
    // (comment removed)
    try {
        uint64_t encoded_lon = EliasGammaCodec::Decode(input_bit_stream_.get());
        uint64_t encoded_lat = EliasGammaCodec::Decode(input_bit_stream_.get());
        
        int64_t quantized_delta_lon = ZigZagCodec::Decode(encoded_lon - 1);
        int64_t quantized_delta_lat = ZigZagCodec::Decode(encoded_lat - 1);
    
        // (comment removed)
        GpsPoint reconstructed_delta(
            quantized_delta_lon * quant_step_,
            quantized_delta_lat * quant_step_
        );
        GpsPoint reconstructed_point = predicted_point + reconstructed_delta;
        reconstructed_point.timestamp = current_timestamp;  // timestamp
        
        // (comment removed)
        UpdateHistory(reconstructed_point);
        current_reconstructed_point_ = reconstructed_point;
        
        point = reconstructed_point;
        points_read_++;  // 
        
    } catch (...) {
        // (comment removed)
        return false;
    }
    
 // ：
    bool should_evaluate = false;
    
    if (use_time_window_) {
        // (comment removed)
        uint64_t time_since_last_eval = current_timestamp - last_evaluation_timestamp_;
        
 // （），
        if (current_timestamp >= last_evaluation_timestamp_ && 
            time_since_last_eval >= time_window_seconds_) {
            should_evaluate = true;
            last_evaluation_timestamp_ = current_timestamp;
        }
    } else {
 // （）
        if (points_read_ % evaluation_window_ == 0) {
            should_evaluate = true;
        }
    }
    
 // ，1
    if (should_evaluate) {
        try {
            bool mode_bit = input_bit_stream_->ReadBit();
            // 0 = Multi-Predictor, 1 = LDR-Only
            current_mode_ = mode_bit ? CompressionMode::MODE_LDR_ONLY : CompressionMode::MODE_MULTI_PREDICTOR;
        } catch (...) {
 // （），（）
        }
    }
    
    return true;
}


std::vector<CoSTDecompressor::GpsPoint> 
CoSTDecompressor::ReadAllPoints() {
    std::vector<GpsPoint> points;
    GpsPoint point;
    
    for (int i = 0; i < block_size_; ++i) {
        if (ReadNextPoint(point)) {
            points.push_back(point);
        } else {
            break;
        }
    }
    
    return points;
}

void CoSTDecompressor::ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp) {
    pred_zp = current_reconstructed_point_;
    if (history_states_.size() < 2) {
        pred_ldr = current_reconstructed_point_;
        pred_cp = current_reconstructed_point_;
        return;
    }
    const int64_t delta_time_signed = static_cast<int64_t>(current_timestamp) -
                                      static_cast<int64_t>(current_reconstructed_point_.timestamp);
    const double dt = delta_time_signed > 0 ? static_cast<double>(delta_time_signed) : 1.0;
    const GpsPoint velocity = history_states_.back().velocity;
    pred_ldr = GpsPoint(current_reconstructed_point_.longitude + velocity.longitude * dt,
                        current_reconstructed_point_.latitude + velocity.latitude * dt,
                        current_timestamp);
    pred_cp = predictWithAcostMotion(history_states_, current_reconstructed_point_,
                                      current_timestamp, quant_step_);
}

void CoSTDecompressor::UpdateHistory(const GpsPoint& reconstructed_point) {
    GpsPoint velocity(0, 0, 0);
    
    if (!history_states_.empty()) {
        const GpsPoint& prev_point = history_states_.back().reconstructed_point;
        int64_t delta_time_signed = static_cast<int64_t>(reconstructed_point.timestamp) - static_cast<int64_t>(prev_point.timestamp);
        
        if (delta_time_signed > 0) {
 // （/）
            double dt = static_cast<double>(delta_time_signed);
            velocity = GpsPoint(
                (reconstructed_point.longitude - prev_point.longitude) / dt,
                (reconstructed_point.latitude - prev_point.latitude) / dt,
                0
            );
        }
    }
    
    history_states_.emplace_back(reconstructed_point, velocity);
    
    if (history_states_.size() > kMaxHistorySize) {
        history_states_.erase(history_states_.begin());
    }
}

void CoSTDecompressor::UpdateHuffmanDecoder() {
    struct PredictorFreq {
        PredictorType type;
        int frequency;
    };
    
    std::vector<PredictorFreq> freq_list = {
        {PredictorType::PREDICTOR_LDR, predictor_frequency_[PredictorType::PREDICTOR_LDR]},
        {PredictorType::PREDICTOR_CP, predictor_frequency_[PredictorType::PREDICTOR_CP]},
        {PredictorType::PREDICTOR_ZP, predictor_frequency_[PredictorType::PREDICTOR_ZP]}
    };
    
 // ：（）
    std::sort(freq_list.begin(), freq_list.end(), 
              [](const PredictorFreq& a, const PredictorFreq& b) {
                  if (a.frequency != b.frequency) {
                      return a.frequency > b.frequency;
                  }
 // ，
                  return static_cast<int>(a.type) < static_cast<int>(b.type);
              });
    
    
    huffman_decoder_map_[0] = freq_list[0].type;  // 0 -> 
    huffman_decoder_map_[1] = freq_list[1].type;  // 10 -> 
    huffman_decoder_map_[2] = freq_list[2].type;  // 11 -> 
}

// Huffman（：0, 10, 11）
CoSTDecompressor::PredictorType 
CoSTDecompressor::DecodeWithHuffman() {
    bool first_bit = input_bit_stream_->ReadBit();
    
    PredictorType decoded_predictor;
    
    if (!first_bit) {
 // 0 -> (1 bit)
        decoded_predictor = huffman_decoder_map_[0];
    } else {
 // 1，
        bool second_bit = input_bit_stream_->ReadBit();
        
        if (!second_bit) {
 // 10 -> (2 bits)
            decoded_predictor = huffman_decoder_map_[1];
        } else {
 // 11 -> (2 bits)
            decoded_predictor = huffman_decoder_map_[2];
        }
    }
    
 // （）
    AddPredictorToWindow(decoded_predictor);
    return decoded_predictor;
}

void CoSTDecompressor::AddPredictorToWindow(PredictorType predictor) {
    predictor_window_.push_back(predictor);
    predictor_frequency_[predictor]++;
    
    if (predictor_window_.size() > kSlidingWindowSize) {
        PredictorType old_predictor = predictor_window_.front();
        predictor_window_.pop_front();
        predictor_frequency_[old_predictor]--;
    }
    
    if (predictor_window_.size() % 100 == 0) {
        UpdateHuffmanDecoder();
    }
}

