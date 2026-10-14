#include "baselines/trajcompress/trajcompress_sp_adaptive_compressor.h"
#include "utils/elias_gamma_codec.h"
#include "utils/zig_zag_codec.h"
#include "utils/input_bit_stream.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// ==================== ====================

TrajCompressSPAdaptiveCompressor::TrajCompressSPAdaptiveCompressor(
 int block_size, double epsilon,
 bool enable_adaptive,
 int min_window,
 int max_window,
 int observe_window)
 : kBlockSize(block_size), 
 kEpsilon(epsilon * 0.999), 
 kQuantStep(2 * epsilon * 0.999),
 kCostWindowSize((min_window + max_window) / 2), // 
 kSwitchCost(1), // ：1 bit（Simple）
 kStabilityMargin(1), // 1
 kEvaluationInterval(16),
 kClearWindowAfterSwitch(false),
 kEnableAdaptive(enable_adaptive),
 kMinWindowSize(min_window),
 kMaxWindowSize(max_window),
 kObserveWindowSize(observe_window) {
 output_bit_stream_ = std::make_unique<OutputBitStream>(2 * block_size * 8);
 history_states_.reserve(kMaxHistorySize);
 predictor_window_.reserve(kSlidingWindowSize);
 // predictor_history_ deque， reserve
    
 // Huffman （）
 predictor_frequency_[PREDICTOR_LDR] = 60; // LDR
 predictor_frequency_[PREDICTOR_CP] = 10; // CP
 predictor_frequency_[PREDICTOR_ZP] = 30; // ZP
 UpdateHuffmanCodes();
}

void TrajCompressSPAdaptiveCompressor::AddGpsPoint(const GpsPoint& point) {
 stats_.total_points++;
    
 if (first_point_) {
 ProcessFirstPoint(point);
 return;
 }
    
 // === 1. （） ===
 GpsPoint pred_ldr, pred_cp, pred_zp;
 ParallelPredict(pred_ldr, pred_cp, pred_zp);
    
    //
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
 int multi_model_cost = best_cost; // ：
 int ldr_only_model_cost = cost_ldr_error; // LDR-Only：LDR（）
    
    //
 UpdateCostWindows(multi_model_cost, ldr_only_model_cost);
    
 // （）
 predictor_history_.push_back(best_predictor);
 if (predictor_history_.size() > static_cast<size_t>(kObserveWindowSize)) {
 predictor_history_.pop_front();
 }
    
 // === 3. ===
 if (current_mode_ == MODE_LDR_ONLY) {
 EncodeLDROnly(point);
 stats_.ldr_only_mode_points++;
 } else { // MODE_MULTI_PREDICTOR
 EncodeMultiPredictor(point);
 stats_.multi_predictor_mode_points++;
 }
    
 // === 4. （） ===
 points_since_last_param_update_++;
 if (kEnableAdaptive && points_since_last_param_update_ >= kObserveWindowSize) {
 UpdateAdaptiveParameters();
 points_since_last_param_update_ = 0;
 }
    
 // === 5. （） ===
 if (stats_.total_points % kEvaluationInterval == 0) {
 // ，
 if (stats_.total_points > static_cast<size_t>(kCostWindowSize) && 
 point_costs_multi_.size() >= static_cast<size_t>(kCostWindowSize)) {
 EvaluateAndSwitchModeBasedOnCost();
 }
        
 // ，1-bit（Simple）
 // 0 = Multi-Predictor, 1 = LDR-Only
 bool mode_bit = (current_mode_ == MODE_LDR_ONLY);
 output_bit_stream_->WriteBit(mode_bit);
 compressed_size_in_bits_ += 1;
 }
}

void TrajCompressSPAdaptiveCompressor::EncodeMultiPredictor(const GpsPoint& point) {
    //
 GpsPoint pred_ldr, pred_cp, pred_zp;
 ParallelPredict(pred_ldr, pred_cp, pred_zp);
    
 // （）
 GpsPoint best_prediction;
 int best_cost;
 PredictorType best_predictor = SelectBestPredictorByCost(
 point, pred_ldr, pred_cp, pred_zp, best_prediction, best_cost);
    
    //
 EncodePrediction(best_predictor, point, best_prediction);
    
    //
 last_used_predictor_ = best_predictor;
    
    //
 switch (best_predictor) {
 case PREDICTOR_LDR: stats_.ldr_count++; break;
 case PREDICTOR_CP: stats_.cp_count++; break;
 case PREDICTOR_ZP: stats_.zp_count++; break;
 }
}

void TrajCompressSPAdaptiveCompressor::EncodeLDROnly(const GpsPoint& point) {
 // LDR
 GpsPoint pred_ldr, pred_cp, pred_zp;
 ParallelPredict(pred_ldr, pred_cp, pred_zp);
    
 // LDR，
 GpsPoint delta = point - pred_ldr;
    
    //
 int64_t quantized_delta_lon = static_cast<int64_t>(std::round(delta.longitude / kQuantStep));
 int64_t quantized_delta_lat = static_cast<int64_t>(std::round(delta.latitude / kQuantStep));
    
 // （ZigZag + Elias Gamma）
 int bits_before = compressed_size_in_bits_;
 compressed_size_in_bits_ += EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lon) + 1, output_bit_stream_.get());
 compressed_size_in_bits_ += EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lat) + 1, output_bit_stream_.get());
 stats_.quantized_data_bits += (compressed_size_in_bits_ - bits_before);
    
    //
 GpsPoint reconstructed_delta(
 quantized_delta_lon * kQuantStep,
 quantized_delta_lat * kQuantStep
 );
 GpsPoint reconstructed_point = pred_ldr + reconstructed_delta;
    
    //
 UpdateReconstructedState(reconstructed_point);
    
    //
 stats_.ldr_count++;
 double error = CalculateDistance(point, reconstructed_point);
 stats_.total_prediction_error += error;
 stats_.max_prediction_error = std::max(stats_.max_prediction_error, error);
 stats_.prediction_errors.push_back(error);
}

//

void TrajCompressSPAdaptiveCompressor::EncodeModeSwitch(CompressionMode new_mode) {
 // 1-bit（Simple）
 // 0 = Multi-Predictor, 1 = LDR-Only
 bool mode_bit = (new_mode == MODE_LDR_ONLY);
 output_bit_stream_->WriteBit(mode_bit);
    
 compressed_size_in_bits_ += 1;
 stats_.mode_switch_bits += 1;
 stats_.mode_switch_count++;
}

TrajCompressSPAdaptiveCompressor::PredictorType TrajCompressSPAdaptiveCompressor::SelectBestPredictorByCost(
 const GpsPoint& current_point,
 const GpsPoint& pred_ldr,
 const GpsPoint& pred_cp,
 const GpsPoint& pred_zp,
 GpsPoint& best_prediction,
 int& best_cost) {
    
    //
 GpsPoint error_ldr = current_point - pred_ldr;
 GpsPoint error_cp = current_point - pred_cp;
 GpsPoint error_zp = current_point - pred_zp;
    
 // （Huffman + ）
 int cost_ldr = GetHuffmanBitCost(PREDICTOR_LDR) + EstimateErrorEncodingCost(error_ldr);
 int cost_cp = GetHuffmanBitCost(PREDICTOR_CP) + EstimateErrorEncodingCost(error_cp);
 int cost_zp = GetHuffmanBitCost(PREDICTOR_ZP) + EstimateErrorEncodingCost(error_zp);
    
    //
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

int TrajCompressSPAdaptiveCompressor::EstimateEliasGammaBits(int64_t value) const {
 if (value <= 0) return 1; // 
 int log2_val = static_cast<int>(std::floor(std::log2(value)));
 return 2 * log2_val + 1;
}

int TrajCompressSPAdaptiveCompressor::EstimateErrorEncodingCost(const GpsPoint& error) const {
    //
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

void TrajCompressSPAdaptiveCompressor::UpdateCostWindows(int multi_cost, int ldr_only_cost) {
    //
 point_costs_multi_.push_back(multi_cost);
 window_total_cost_multi_ += multi_cost;
    
 point_costs_ldr_only_.push_back(ldr_only_cost);
 window_total_cost_ldr_only_ += ldr_only_cost;
    
 // （ kStabilityMargin）
 int cost_diff = multi_cost - ldr_only_cost;
 cost_diffs_.push_back(cost_diff);
 if (cost_diffs_.size() > static_cast<size_t>(kCostWindowSize)) {
 cost_diffs_.pop_front();
 }
    
 // ，
 if (point_costs_multi_.size() > static_cast<size_t>(kCostWindowSize)) {
 window_total_cost_multi_ -= point_costs_multi_.front();
 point_costs_multi_.pop_front();
        
 window_total_cost_ldr_only_ -= point_costs_ldr_only_.front();
 point_costs_ldr_only_.pop_front();
 }
}

// ========== （） ==========

void TrajCompressSPAdaptiveCompressor::EvaluateAndSwitchModeBasedOnCost() {
    //
 if (point_costs_multi_.size() < static_cast<size_t>(kCostWindowSize)) return;
    
 if (current_mode_ == MODE_MULTI_PREDICTOR) {
 // LDR-Only 
 // ：LDR-Only < Multi - - 
 if (window_total_cost_ldr_only_ < window_total_cost_multi_ - kSwitchCost - kStabilityMargin) {
 // ，1-bitAddGpsPoint
 current_mode_ = MODE_LDR_ONLY;
 stats_.mode_switch_count++;
            
            //
 if (kClearWindowAfterSwitch) {
 point_costs_multi_.clear();
 point_costs_ldr_only_.clear();
 window_total_cost_multi_ = 0;
 window_total_cost_ldr_only_ = 0;
 }
 // ，
 }
 } else { // current_mode_ == MODE_LDR_ONLY
 // Multi-Predictor 
 // ：Multi < LDR-Only - - 
 if (window_total_cost_multi_ < window_total_cost_ldr_only_ - kSwitchCost - kStabilityMargin) {
 // ，1-bitAddGpsPoint
 current_mode_ = MODE_MULTI_PREDICTOR;
 stats_.mode_switch_count++;
            
            //
 if (kClearWindowAfterSwitch) {
 point_costs_multi_.clear();
 point_costs_ldr_only_.clear();
 window_total_cost_multi_ = 0;
 window_total_cost_ldr_only_ = 0;
 }
 // ，
 }
 }
}

// ========== （） ==========

void TrajCompressSPAdaptiveCompressor::UpdateAdaptiveParameters() {
 if (!kEnableAdaptive) return;
    
 // 1. （Churn Rate）
 double churn_rate = CalculateChurnRate();
    
 // 2. kCostWindowSize
 // ****：
 // （，Track）→ （LDR-Only）
 // （，Geolife）→ （）
 kCostWindowSize = kMinWindowSize + static_cast<int>((kMaxWindowSize - kMinWindowSize) * churn_rate);
    
 // 3. 
 double cost_diff_stddev = CalculateCostDiffStdDev();
    
 // 4. kStabilityMargin
 // ****：
 // （，stddev）→ （，）
 // （，stddev）→ （，）
 const double MAX_REASONABLE_STDDEV = 10.0; // 
 double normalized_stddev = std::min(cost_diff_stddev, MAX_REASONABLE_STDDEV) / MAX_REASONABLE_STDDEV;
    
 const int MAX_MARGIN = 4;
 const int MIN_MARGIN = 1;
 kStabilityMargin = MAX_MARGIN - static_cast<int>((MAX_MARGIN - MIN_MARGIN) * normalized_stddev);
    
    //
 kCostWindowSize = std::max(kMinWindowSize, std::min(kCostWindowSize, kMaxWindowSize));
 kStabilityMargin = std::max(MIN_MARGIN, std::min(kStabilityMargin, MAX_MARGIN));
}

double TrajCompressSPAdaptiveCompressor::CalculateChurnRate() const {
 if (predictor_history_.size() < 10) {
 return 0.5; // ，
 }
    
    //
 int switches = 0;
 for (size_t i = 1; i < predictor_history_.size(); i++) {
 if (predictor_history_[i] != predictor_history_[i - 1]) {
 switches++;
 }
 }
    
 // = / ( - 1)
 double churn_rate = static_cast<double>(switches) / (predictor_history_.size() - 1);
    
 return std::min(std::max(churn_rate, 0.0), 1.0); // [0, 1]
}

double TrajCompressSPAdaptiveCompressor::CalculateCostDiffStdDev() const {
 if (cost_diffs_.size() < 10) {
 return 2.0; // ，
 }
    
    //
 double mean = 0.0;
 for (int diff : cost_diffs_) {
 mean += diff;
 }
 mean /= cost_diffs_.size();
    
    //
 double variance = 0.0;
 for (int diff : cost_diffs_) {
 double deviation = diff - mean;
 variance += deviation * deviation;
 }
 variance /= cost_diffs_.size();
    
 return std::sqrt(variance);
}

// ========== Huffman ==========

int TrajCompressSPAdaptiveCompressor::GetHuffmanBitCost(PredictorType predictor) const {
 return huffman_codes_[predictor].length;
}

void TrajCompressSPAdaptiveCompressor::UpdateHuffmanCodes() {
    //
 struct PredictorFreq {
 PredictorType type;
 int frequency;
 };
    
 std::vector<PredictorFreq> freq_list = {
 {PREDICTOR_LDR, predictor_frequency_[PREDICTOR_LDR]},
 {PREDICTOR_CP, predictor_frequency_[PREDICTOR_CP]},
 {PREDICTOR_ZP, predictor_frequency_[PREDICTOR_ZP]}
 };
    
 // ：（Simple）
 std::sort(freq_list.begin(), freq_list.end(), 
 [](const PredictorFreq& a, const PredictorFreq& b) {
 if (a.frequency != b.frequency) {
 return a.frequency > b.frequency;
 }
 // ，
 return static_cast<int>(a.type) < static_cast<int>(b.type);
 });
    
 // Huffman：0 (1 bit)，10 (2 bits)，11 (2 bits)
 huffman_codes_[freq_list[0].type] = HuffmanCode({false}); // 0
 huffman_codes_[freq_list[1].type] = HuffmanCode({true, false}); // 10
 huffman_codes_[freq_list[2].type] = HuffmanCode({true, true}); // 11
}

void TrajCompressSPAdaptiveCompressor::EncodeWithHuffman(PredictorType predictor) {
 const HuffmanCode& code = huffman_codes_[predictor];
 for (bool bit : code.bits) {
 output_bit_stream_->WriteBit(bit);
 }
 compressed_size_in_bits_ += code.length;
    
    //
 AddPredictorToWindow(predictor);
}

void TrajCompressSPAdaptiveCompressor::AddPredictorToWindow(PredictorType predictor) {
 predictor_window_.push_back(predictor);
 predictor_frequency_[predictor]++;
    
 if (predictor_window_.size() > kSlidingWindowSize) {
 PredictorType old_predictor = predictor_window_.front();
 predictor_window_.erase(predictor_window_.begin());
 predictor_frequency_[old_predictor]--;
 }
    
 // Huffman
 if (predictor_window_.size() % 100 == 0) {
 UpdateHuffmanCodes();
 }
}

void TrajCompressSPAdaptiveCompressor::Close() {
 output_bit_stream_->Flush();
 stats_.total_bits = compressed_size_in_bits_;
}

Array<uint8_t> TrajCompressSPAdaptiveCompressor::GetCompressedData() {
 int byte_length = (compressed_size_in_bits_ + 7) / 8;
 return output_bit_stream_->GetBuffer(byte_length);
}

void TrajCompressSPAdaptiveCompressor::ProcessFirstPoint(const GpsPoint& point) {
 first_point_ = false;
    
    //
 compressed_size_in_bits_ += output_bit_stream_->WriteInt(kBlockSize, 16);
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(kEpsilon), 64);
    
    //
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(point.longitude), 64);
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(point.latitude), 64);
    
    //
 current_reconstructed_point_ = point;
 history_states_.emplace_back(point, GpsPoint(0, 0));
}

void TrajCompressSPAdaptiveCompressor::ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp) {
 // （ZP）
 pred_zp = current_reconstructed_point_;
    
 if (history_states_.size() < 2) {
 pred_ldr = current_reconstructed_point_;
 pred_cp = current_reconstructed_point_;
 return;
 }
    
 // （LDR）
 GpsPoint velocity = history_states_[history_states_.size() - 1].velocity;
 pred_ldr = current_reconstructed_point_ + velocity;
    
 // （CP）
 if (history_states_.size() >= 5) {
 GpsPoint v1 = history_states_[history_states_.size() - 1].velocity;
 GpsPoint v2 = history_states_[history_states_.size() - 2].velocity;
 GpsPoint v3 = history_states_[history_states_.size() - 3].velocity;
        
 GpsPoint smoothed_velocity(
 0.45 * v1.longitude + 0.35 * v2.longitude + 0.20 * v3.longitude,
 0.45 * v1.latitude + 0.35 * v2.latitude + 0.20 * v3.latitude
 );
        
 GpsPoint a1 = v1 - v2;
 GpsPoint a2 = v2 - v3;
 GpsPoint jerk = a1 - a2;
        
 pred_cp = current_reconstructed_point_ + smoothed_velocity + a1 + jerk * 0.5;
 } else if (history_states_.size() >= 4) {
 GpsPoint v1 = history_states_[history_states_.size() - 1].velocity;
 GpsPoint v2 = history_states_[history_states_.size() - 2].velocity;
 GpsPoint v3 = history_states_[history_states_.size() - 3].velocity;
        
 GpsPoint smoothed_velocity(
 0.45 * v1.longitude + 0.35 * v2.longitude + 0.20 * v3.longitude,
 0.45 * v1.latitude + 0.35 * v2.latitude + 0.20 * v3.latitude
 );
        
 GpsPoint acceleration = v1 - v2;
 pred_cp = current_reconstructed_point_ + smoothed_velocity + acceleration;
 } else if (history_states_.size() >= 3) {
 GpsPoint velocity = history_states_[history_states_.size() - 1].velocity;
 GpsPoint prev_velocity = history_states_[history_states_.size() - 2].velocity;
 GpsPoint acceleration = velocity - prev_velocity;
 pred_cp = current_reconstructed_point_ + velocity + acceleration;
 } else {
 pred_cp = pred_ldr;
 }
}

void TrajCompressSPAdaptiveCompressor::EncodePrediction(PredictorType predictor,
 const GpsPoint& current_point,
 const GpsPoint& predicted_point) {
 // （Huffman）
 int bits_before_flag = compressed_size_in_bits_;
 EncodeWithHuffman(predictor);
 stats_.predictor_flag_bits += (compressed_size_in_bits_ - bits_before_flag);
    
    //
 GpsPoint delta = current_point - predicted_point;
    
    //
 int64_t quantized_delta_lon = static_cast<int64_t>(std::round(delta.longitude / kQuantStep));
 int64_t quantized_delta_lat = static_cast<int64_t>(std::round(delta.latitude / kQuantStep));
    
    //
 int bits_before_data = compressed_size_in_bits_;
 compressed_size_in_bits_ += EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lon) + 1, output_bit_stream_.get());
 compressed_size_in_bits_ += EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lat) + 1, output_bit_stream_.get());
 stats_.quantized_data_bits += (compressed_size_in_bits_ - bits_before_data);
    
    //
 GpsPoint reconstructed_delta(
 quantized_delta_lon * kQuantStep,
 quantized_delta_lat * kQuantStep
 );
 GpsPoint reconstructed_point = predicted_point + reconstructed_delta;
    
    //
 UpdateReconstructedState(reconstructed_point);
    
    //
 double error = CalculateDistance(current_point, reconstructed_point);
 stats_.total_prediction_error += error;
 stats_.max_prediction_error = std::max(stats_.max_prediction_error, error);
 stats_.prediction_errors.push_back(error);
}

void TrajCompressSPAdaptiveCompressor::UpdateHistory(const GpsPoint& reconstructed_point) {
 GpsPoint velocity(0, 0);
 if (!history_states_.empty()) {
 velocity = reconstructed_point - history_states_.back().reconstructed_point;
 }
    
 history_states_.emplace_back(reconstructed_point, velocity);
    
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.erase(history_states_.begin());
 }
}

void TrajCompressSPAdaptiveCompressor::UpdateReconstructedState(const GpsPoint& reconstructed_point) {
 UpdateHistory(reconstructed_point);
 current_reconstructed_point_ = reconstructed_point;
}

double TrajCompressSPAdaptiveCompressor::CalculateDistance(const GpsPoint& p1, const GpsPoint& p2) const {
 double dx = p1.longitude - p2.longitude;
 double dy = p1.latitude - p2.latitude;
 return std::sqrt(dx * dx + dy * dy);
}

//
void TrajCompressSPAdaptiveCompressor::CompressionStats::PrintStats() const {
 std::cout << "\n=== TrajCompress-SP-Adaptive ===" << std::endl;
 std::cout << ": " << total_points << std::endl;
 std::cout << ": LDR=" << ldr_count << ", CP=" << cp_count << ", ZP=" << zp_count << std::endl;
 std::cout << ": " << mode_switch_count << std::endl;
 std::cout << "LDR-Only: " << ldr_only_mode_points << " (" 
 << (100.0 * ldr_only_mode_points / total_points) << "%)" << std::endl;
 std::cout << "Multi-Predictor: " << multi_predictor_mode_points << " (" 
 << (100.0 * multi_predictor_mode_points / total_points) << "%)" << std::endl;
 std::cout << ": " << total_bits << std::endl;
 std::cout << " : " << predictor_flag_bits << " bits" << std::endl;
 std::cout << " : " << mode_switch_bits << " bits" << std::endl;
 std::cout << " : " << quantized_data_bits << " bits" << std::endl;
}

void TrajCompressSPAdaptiveCompressor::CompressionStats::PrintDetailedStats() const {
 PrintStats();
    
 if (!prediction_errors.empty()) {
 std::vector<double> sorted_errors = prediction_errors;
 std::sort(sorted_errors.begin(), sorted_errors.end());
        
 size_t p50 = sorted_errors.size() * 0.50;
 size_t p90 = sorted_errors.size() * 0.90;
 size_t p99 = sorted_errors.size() * 0.99;
        
 std::cout << "\n:" << std::endl;
 std::cout << " : " << (total_prediction_error / prediction_errors.size()) << std::endl;
 std::cout << " : " << max_prediction_error << std::endl;
 std::cout << " P50: " << sorted_errors[p50] << std::endl;
 std::cout << " P90: " << sorted_errors[p90] << std::endl;
 std::cout << " P99: " << sorted_errors[p99] << std::endl;
 }
}

// ==================== ====================

TrajCompressSPAdaptiveDecompressor::TrajCompressSPAdaptiveDecompressor(uint8_t* compressed_data, int data_size) {
 input_bit_stream_ = std::make_unique<InputBitStream>(compressed_data, data_size);
 predictor_window_.reserve(kSlidingWindowSize);
 ReadHeader();
}

void TrajCompressSPAdaptiveDecompressor::ReadHeader() {
 block_size_ = input_bit_stream_->ReadInt(16);
 epsilon_ = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
 quant_step_ = 2 * epsilon_;
 evaluation_interval_ = 16; // （）
    
 // Huffman
 predictor_frequency_[PredictorType::PREDICTOR_LDR] = 60;
 predictor_frequency_[PredictorType::PREDICTOR_CP] = 10;
 predictor_frequency_[PredictorType::PREDICTOR_ZP] = 30;
 UpdateHuffmanDecoder();
}

bool TrajCompressSPAdaptiveDecompressor::ReadNextPoint(GpsPoint& point) {
 if (first_point_) {
 first_point_ = false;
        
        //
 double lon = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
 double lat = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
 point = GpsPoint(lon, lat);
        
 current_reconstructed_point_ = point;
 history_states_.emplace_back(point, GpsPoint(0, 0));
        
 // points_read_，total_points
 points_read_ = 1;
 return true;
 }
    
 try {
 GpsPoint pred_ldr, pred_cp, pred_zp;
 ParallelPredict(pred_ldr, pred_cp, pred_zp);
        
 GpsPoint predicted_point;
        
 if (current_mode_ == CompressionMode::MODE_LDR_ONLY) {
 // LDR-Only：LDR
 predicted_point = pred_ldr;
 } else {
 // Multi-Predictor：（Huffman）
 PredictorType predictor = DecodeWithHuffman();
            
 switch (predictor) {
 case PredictorType::PREDICTOR_LDR: predicted_point = pred_ldr; break;
 case PredictorType::PREDICTOR_CP: predicted_point = pred_cp; break;
 case PredictorType::PREDICTOR_ZP: predicted_point = pred_zp; break;
 }
            
 last_used_predictor_ = predictor;
 }
        
        //
 uint64_t encoded_lon = EliasGammaCodec::Decode(input_bit_stream_.get());
 uint64_t encoded_lat = EliasGammaCodec::Decode(input_bit_stream_.get());
        
 int64_t quantized_delta_lon = ZigZagCodec::Decode(encoded_lon - 1);
 int64_t quantized_delta_lat = ZigZagCodec::Decode(encoded_lat - 1);
        
        //
 GpsPoint reconstructed_delta(
 quantized_delta_lon * quant_step_,
 quantized_delta_lat * quant_step_
 );
 GpsPoint reconstructed_point = predicted_point + reconstructed_delta;
        
        //
 UpdateHistory(reconstructed_point);
 current_reconstructed_point_ = reconstructed_point;
        
 point = reconstructed_point;
 points_read_++;
 } catch (...) {
 return false; // ，
 }
    
 // 1-bit（Simple）
 if (points_read_ % evaluation_interval_ == 0) {
 try {
 bool mode_bit = input_bit_stream_->ReadBit();
 current_mode_ = mode_bit ? CompressionMode::MODE_LDR_ONLY : CompressionMode::MODE_MULTI_PREDICTOR;
 } catch (...) {
 // bit（），，
 }
 }
    
 return true;
}

bool TrajCompressSPAdaptiveDecompressor::CheckForModeSwitch() {
 // 3， 111
 bool bit1 = input_bit_stream_->ReadBit();
 if (!bit1) {
 // 0，，
 // ：InputBitStream，
 // peek
 return false;
 }
    
 bool bit2 = input_bit_stream_->ReadBit();
 if (!bit2) {
 // 10...，
 return false;
 }
    
 bool bit3 = input_bit_stream_->ReadBit();
 if (!bit3) {
 // 110，
 return false;
 }
    
 // 111，，
 bool mode_flag = input_bit_stream_->ReadBit();
 current_mode_ = mode_flag ? CompressionMode::MODE_MULTI_PREDICTOR : CompressionMode::MODE_LDR_ONLY;
    
 return true;
}

std::vector<TrajCompressSPAdaptiveDecompressor::GpsPoint> 
TrajCompressSPAdaptiveDecompressor::ReadAllPoints() {
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

void TrajCompressSPAdaptiveDecompressor::ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp) {
 pred_zp = current_reconstructed_point_;
    
 if (history_states_.size() < 2) {
 pred_ldr = current_reconstructed_point_;
 pred_cp = current_reconstructed_point_;
 return;
 }
    
 GpsPoint velocity = history_states_[history_states_.size() - 1].velocity;
 pred_ldr = current_reconstructed_point_ + velocity;
    
 if (history_states_.size() >= 3) {
 GpsPoint v1 = history_states_[history_states_.size() - 1].velocity;
 GpsPoint v2 = history_states_[history_states_.size() - 2].velocity;
 GpsPoint acceleration = v1 - v2;
 pred_cp = current_reconstructed_point_ + v1 + acceleration;
 } else {
 pred_cp = pred_ldr;
 }
}

void TrajCompressSPAdaptiveDecompressor::UpdateHistory(const GpsPoint& reconstructed_point) {
 GpsPoint velocity(0, 0);
 if (!history_states_.empty()) {
 velocity = reconstructed_point - history_states_.back().reconstructed_point;
 }
    
 history_states_.emplace_back(reconstructed_point, velocity);
    
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.erase(history_states_.begin());
 }
}

void TrajCompressSPAdaptiveDecompressor::UpdateHuffmanDecoder() {
 struct PredictorFreq {
 PredictorType type;
 int frequency;
 };
    
 std::vector<PredictorFreq> freq_list = {
 {PredictorType::PREDICTOR_LDR, predictor_frequency_[PredictorType::PREDICTOR_LDR]},
 {PredictorType::PREDICTOR_CP, predictor_frequency_[PredictorType::PREDICTOR_CP]},
 {PredictorType::PREDICTOR_ZP, predictor_frequency_[PredictorType::PREDICTOR_ZP]}
 };
    
 // ：（，Simple）
 std::sort(freq_list.begin(), freq_list.end(), 
 [](const PredictorFreq& a, const PredictorFreq& b) {
 if (a.frequency != b.frequency) {
 return a.frequency > b.frequency;
 }
 // ，
 return static_cast<int>(a.type) < static_cast<int>(b.type);
 });
    
 huffman_decoder_map_[0] = freq_list[0].type; // 0 -> 
 huffman_decoder_map_[1] = freq_list[1].type; // 10 -> 
 huffman_decoder_map_[2] = freq_list[2].type; // 11 -> 
}

TrajCompressSPAdaptiveDecompressor::PredictorType TrajCompressSPAdaptiveDecompressor::DecodeWithHuffman() {
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
    
 // （，Simple）
 AddPredictorToWindow(decoded_predictor);
 return decoded_predictor;
}

void TrajCompressSPAdaptiveDecompressor::AddPredictorToWindow(PredictorType predictor) {
 predictor_window_.push_back(predictor);
 predictor_frequency_[predictor]++;
    
 if (predictor_window_.size() > kSlidingWindowSize) {
 PredictorType old_predictor = predictor_window_.front();
 predictor_window_.erase(predictor_window_.begin());
 predictor_frequency_[old_predictor]--;
 }
    
 if (predictor_window_.size() % 100 == 0) {
 UpdateHuffmanDecoder();
 }
}

