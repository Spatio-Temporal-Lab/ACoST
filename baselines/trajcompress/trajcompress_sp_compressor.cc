#include "baselines/trajcompress/trajcompress_sp_compressor.h"
#include "utils/elias_gamma_codec.h"
#include "utils/zig_zag_codec.h"
#include "utils/input_bit_stream.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

// ==================== ====================

TrajCompressSPCompressor::TrajCompressSPCompressor(int block_size, double epsilon)
 : kBlockSize(block_size), kEpsilon(epsilon * 0.999), kQuantStep(2 * epsilon * 0.999) {
 output_bit_stream_ = std::make_unique<OutputBitStream>(2 * block_size * 8);
 // ：deque，reserve
    
 // Huffman （）
 // ：LDR ，CP ，ZP 
 predictor_frequency_[PREDICTOR_LDR] = 60; // LDR 60%
 predictor_frequency_[PREDICTOR_CP] = 10; // CP 10%
 predictor_frequency_[PREDICTOR_ZP] = 30; // ZP 30%
 UpdateHuffmanCodes();
}

void TrajCompressSPCompressor::AddGpsPoint(const GpsPoint& point) {
 stats_.total_points++;
    
 if (first_point_) {
 ProcessFirstPoint(point);
 return;
 }
    
 // === Simple， ===
    
 // 1. （ParallelPredict）
 GpsPoint pred_zp = current_reconstructed_point_;
 GpsPoint pred_ldr, pred_cp;
    
 if (history_states_.size() >= 2) {
        //
 int64_t delta_time_signed = static_cast<int64_t>(point.timestamp) - static_cast<int64_t>(current_reconstructed_point_.timestamp);
 double dt = (delta_time_signed > 0) ? static_cast<double>(delta_time_signed) : 1.0;
        
 // LDR
 GpsPoint velocity = history_states_.back().velocity;
 pred_ldr = GpsPoint(
 current_reconstructed_point_.longitude + velocity.longitude * dt,
 current_reconstructed_point_.latitude + velocity.latitude * dt,
 point.timestamp
 );
        
 // CP（）
 if (history_states_.size() >= 3 && dt > 0) {
 GpsPoint prev_velocity = history_states_[history_states_.size() - 2].velocity;
 GpsPoint acceleration = velocity - prev_velocity;
 double dt_sq_half = dt * dt * 0.5;
 pred_cp = GpsPoint(
 current_reconstructed_point_.longitude + velocity.longitude * dt + acceleration.longitude * dt_sq_half,
 current_reconstructed_point_.latitude + velocity.latitude * dt + acceleration.latitude * dt_sq_half,
 point.timestamp
 );
 } else {
 pred_cp = pred_ldr;
 }
 } else {
 pred_ldr = current_reconstructed_point_;
 pred_cp = current_reconstructed_point_;
 }
    
 // 2. （SelectBestPredictor，sqrt）
 double dx_ldr = point.longitude - pred_ldr.longitude;
 double dy_ldr = point.latitude - pred_ldr.latitude;
 double error_sq_ldr = dx_ldr * dx_ldr + dy_ldr * dy_ldr;
    
 double dx_cp = point.longitude - pred_cp.longitude;
 double dy_cp = point.latitude - pred_cp.latitude;
 double error_sq_cp = dx_cp * dx_cp + dy_cp * dy_cp;
    
 double dx_zp = point.longitude - pred_zp.longitude;
 double dy_zp = point.latitude - pred_zp.latitude;
 double error_sq_zp = dx_zp * dx_zp + dy_zp * dy_zp;
    
 PredictorType best_predictor;
 GpsPoint best_prediction;
    
 if (error_sq_ldr <= error_sq_cp && error_sq_ldr <= error_sq_zp) {
 best_predictor = PREDICTOR_LDR;
 best_prediction = pred_ldr;
 stats_.ldr_count++;
 } else if (error_sq_cp <= error_sq_zp) {
 best_predictor = PREDICTOR_CP;
 best_prediction = pred_cp;
 stats_.cp_count++;
 } else {
 best_predictor = PREDICTOR_ZP;
 best_prediction = pred_zp;
 stats_.zp_count++;
 }
    
 // 3. Huffman（）
 const HuffmanCode& code = huffman_codes_[best_predictor];
 for (bool bit : code.bits) {
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(bit);
 }
 stats_.predictor_flag_bits += code.length;
    
 // 4. Huffman（）
 if (window_size_ == kSlidingWindowSize) {
 predictor_frequency_[predictor_window_[window_head_]]--;
 window_head_ = (window_head_ + 1) % kSlidingWindowSize;
 window_size_--;
 }
 int tail = (window_head_ + window_size_) % kSlidingWindowSize;
 predictor_window_[tail] = best_predictor;
 predictor_frequency_[best_predictor]++;
 window_size_++;
 if (window_size_ % 100 == 0 && window_size_ >= 100) {
 UpdateHuffmanCodes();
 }
    
 // 5. timestamp
 uint64_t timestamp_delta = point.timestamp - current_reconstructed_point_.timestamp;
 int ts_bits = output_bit_stream_->WriteLong(timestamp_delta, 64);
 compressed_size_in_bits_ += ts_bits;
 stats_.timestamp_bits += ts_bits;
    
 // 6. （）
 GpsPoint delta = point - best_prediction;
 int64_t quantized_delta_lon = static_cast<int64_t>(std::round(delta.longitude / kQuantStep));
 int64_t quantized_delta_lat = static_cast<int64_t>(std::round(delta.latitude / kQuantStep));
    
 int bits_lon = EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lon) + 1, output_bit_stream_.get());
 int bits_lat = EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lat) + 1, output_bit_stream_.get());
    
 compressed_size_in_bits_ += bits_lon + bits_lat;
 stats_.quantization_bits += bits_lon + bits_lat;
    
 // 7. （）
 GpsPoint reconstructed_point = best_prediction + GpsPoint(
 quantized_delta_lon * kQuantStep,
 quantized_delta_lat * kQuantStep
 );
 reconstructed_point.timestamp = point.timestamp;
    
    //
 GpsPoint velocity(0, 0, 0);
 if (!history_states_.empty()) {
 const GpsPoint& prev_point = history_states_.back().reconstructed_point;
 if (reconstructed_point.timestamp > 0 && prev_point.timestamp > 0) {
 int64_t dt_signed = static_cast<int64_t>(reconstructed_point.timestamp) - static_cast<int64_t>(prev_point.timestamp);
 if (dt_signed > 0) {
 double dt = static_cast<double>(dt_signed);
 velocity = GpsPoint(
 (reconstructed_point.longitude - prev_point.longitude) / dt,
 (reconstructed_point.latitude - prev_point.latitude) / dt,
 0
 );
 } else {
 velocity = reconstructed_point - prev_point;
 }
 } else {
 velocity = reconstructed_point - prev_point;
 }
 }
    
 current_reconstructed_point_ = reconstructed_point;
 history_states_.emplace_back(reconstructed_point, velocity);
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.pop_front();
 }
    
 last_used_predictor_ = best_predictor;
}

void TrajCompressSPCompressor::Close() {
 output_bit_stream_->Flush();
 stats_.total_bits = compressed_size_in_bits_;
}

Array<uint8_t> TrajCompressSPCompressor::GetCompressedData() {
 int byte_length = (compressed_size_in_bits_ + 7) / 8;
 return output_bit_stream_->GetBuffer(byte_length);
}

void TrajCompressSPCompressor::ProcessFirstPoint(const GpsPoint& point) {
 first_point_ = false;
    
    //
 compressed_size_in_bits_ += output_bit_stream_->WriteInt(kBlockSize, 16);
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(kEpsilon), 64);
    
    //
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(point.longitude), 64);
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(point.latitude), 64);
    
 // timestamp（64）
 int ts_bits = output_bit_stream_->WriteLong(point.timestamp, 64);
 compressed_size_in_bits_ += ts_bits;
 stats_.timestamp_bits += ts_bits;
    
    //
 current_reconstructed_point_ = point;
    
 // （）
 history_states_.emplace_back(point, GpsPoint(0, 0, 0));
}

void TrajCompressSPCompressor::ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp) {
 // （ZP）：
 pred_zp = current_reconstructed_point_;
    
 if (history_states_.size() < 2) {
 // ，
 pred_ldr = current_reconstructed_point_;
 pred_cp = current_reconstructed_point_;
 return;
 }
    
 // dt（），int64_ttimestampunderflow
 int64_t delta_time_signed = static_cast<int64_t>(current_timestamp) - static_cast<int64_t>(current_reconstructed_point_.timestamp);
 double dt = (delta_time_signed > 0) ? static_cast<double>(delta_time_signed) : 1.0; // 1fallback
    
 // （LDR）：（/）
 GpsPoint velocity = history_states_[history_states_.size() - 1].velocity;
 pred_ldr = GpsPoint(
 current_reconstructed_point_.longitude + velocity.longitude * dt,
 current_reconstructed_point_.latitude + velocity.latitude * dt,
 current_timestamp
 );
    
 // （CP）：（）
 // ：Simple，jerk
 if (history_states_.size() >= 3 && dt > 0) {
 // （）
 GpsPoint velocity = history_states_[history_states_.size() - 1].velocity;
 GpsPoint prev_velocity = history_states_[history_states_.size() - 2].velocity;
 GpsPoint acceleration = velocity - prev_velocity;
        
 // ：position + velocity*dt + 0.5*acceleration*dt^2
 double dt_sq_half = dt * dt * 0.5;
 pred_cp = GpsPoint(
 current_reconstructed_point_.longitude + velocity.longitude * dt + acceleration.longitude * dt_sq_half,
 current_reconstructed_point_.latitude + velocity.latitude * dt + acceleration.latitude * dt_sq_half,
 current_timestamp
 );
 } else {
 // ，
 pred_cp = pred_ldr;
 }
}

TrajCompressSPCompressor::PredictorType TrajCompressSPCompressor::SelectBestPredictor(
 const GpsPoint& current_point,
 const GpsPoint& pred_ldr,
 const GpsPoint& pred_cp,
 const GpsPoint& pred_zp,
 GpsPoint& best_prediction) {
    
 // ：（3sqrt）
 // ，
 double dx_ldr = current_point.longitude - pred_ldr.longitude;
 double dy_ldr = current_point.latitude - pred_ldr.latitude;
 double error_sq_ldr = dx_ldr * dx_ldr + dy_ldr * dy_ldr;
    
 double dx_cp = current_point.longitude - pred_cp.longitude;
 double dy_cp = current_point.latitude - pred_cp.latitude;
 double error_sq_cp = dx_cp * dx_cp + dy_cp * dy_cp;
    
 double dx_zp = current_point.longitude - pred_zp.longitude;
 double dy_zp = current_point.latitude - pred_zp.latitude;
 double error_sq_zp = dx_zp * dx_zp + dy_zp * dy_zp;
    
 // （）
 PredictorType best_predictor;
 double best_error_sq;
    
 if (error_sq_ldr <= error_sq_cp && error_sq_ldr <= error_sq_zp) {
 best_prediction = pred_ldr;
 best_predictor = PREDICTOR_LDR;
 best_error_sq = error_sq_ldr;
 } else if (error_sq_cp <= error_sq_zp) {
 best_prediction = pred_cp;
 best_predictor = PREDICTOR_CP;
 best_error_sq = error_sq_cp;
 } else {
 best_prediction = pred_zp;
 best_predictor = PREDICTOR_ZP;
 best_error_sq = error_sq_zp;
 }
    
 // （，）
 double best_error = std::sqrt(best_error_sq);
 stats_.total_prediction_error += best_error;
 stats_.max_prediction_error = std::max(stats_.max_prediction_error, best_error);
 // ：push_back，vector
 // stats_.prediction_errors.push_back(best_error);
    
 return best_predictor;
}

void TrajCompressSPCompressor::EncodePrediction(PredictorType predictor,
 const GpsPoint& current_point,
 const GpsPoint& predicted_point) {
 // 1. （2 bits）
 // 00 = LDR, 01 = CP, 10 = ZP, 11 = reserved
 switch (predictor) {
 case PREDICTOR_LDR:
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(false);
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(false);
 stats_.predictor_flag_bits += 2;
 break;
 case PREDICTOR_CP:
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(false);
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(true);
 stats_.predictor_flag_bits += 2;
 break;
 case PREDICTOR_ZP:
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(true);
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(false);
 stats_.predictor_flag_bits += 2;
 break;
 }
    
 // 2. （）
 GpsPoint delta = current_point - predicted_point;
    
 // 3. （kQuantStep = 2 * epsilon * 0.999，Serf-QT）
 int64_t quantized_delta_lon = static_cast<int64_t>(std::round(delta.longitude / kQuantStep));
 int64_t quantized_delta_lat = static_cast<int64_t>(std::round(delta.latitude / kQuantStep));
    
 // 4. ZigZag + Elias Gamma 
 int bits_lon = EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lon) + 1,
 output_bit_stream_.get()
 );
 int bits_lat = EliasGammaCodec::Encode(
 ZigZagCodec::Encode(quantized_delta_lat) + 1,
 output_bit_stream_.get()
 );
    
 compressed_size_in_bits_ += bits_lon + bits_lat;
 stats_.quantization_bits += bits_lon + bits_lat;
    
 // 5. （）
 GpsPoint reconstructed_delta(
 quantized_delta_lon * kQuantStep,
 quantized_delta_lat * kQuantStep
 );
 GpsPoint reconstructed_point = predicted_point + reconstructed_delta;
    
 // 6. 
 UpdateHistory(reconstructed_point);
}

void TrajCompressSPCompressor::UpdateHistory(const GpsPoint& reconstructed_point) {
 // （/），int64_ttimestampunderflow
 GpsPoint velocity(0, 0, 0);
 if (reconstructed_point.timestamp > 0 && current_reconstructed_point_.timestamp > 0) {
 int64_t delta_time_signed = static_cast<int64_t>(reconstructed_point.timestamp) - static_cast<int64_t>(current_reconstructed_point_.timestamp);
 if (delta_time_signed > 0) {
 double dt = static_cast<double>(delta_time_signed);
 velocity = GpsPoint(
 (reconstructed_point.longitude - current_reconstructed_point_.longitude) / dt,
 (reconstructed_point.latitude - current_reconstructed_point_.latitude) / dt,
 0
 );
 } else {
 // Timestamp，fallback
 velocity = reconstructed_point - current_reconstructed_point_;
 }
 } else {
 // ，（dt=1）
 velocity = reconstructed_point - current_reconstructed_point_;
 }
    
    //
 current_reconstructed_point_ = reconstructed_point;
    
    //
 history_states_.emplace_back(reconstructed_point, velocity);
    
 // （：dequepop_frontO(1)）
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.pop_front();
 }
}

double TrajCompressSPCompressor::CalculateDistance(const GpsPoint& p1, const GpsPoint& p2) const {
 double dx = p1.longitude - p2.longitude;
 double dy = p1.latitude - p2.latitude;
 return std::sqrt(dx * dx + dy * dy);
}

void TrajCompressSPCompressor::CompressionStats::PrintStats() const {
 if (total_points == 0) {
 std::cout << "" << std::endl;
 return;
 }
    
 std::cout << "\n=== TrajCompress-SP ===" << std::endl;
 std::cout << ": " << total_points << std::endl;
    
 std::cout << "\n=== ===" << std::endl;
 int moving_points = total_points - 1; // 
 if (moving_points > 0) {
 std::cout << "LDR (): " << ldr_count << " (" 
 << std::fixed << std::setprecision(2)
 << (100.0 * ldr_count / moving_points) << "%)" << std::endl;
 std::cout << "CP (): " << cp_count << " (" 
 << (100.0 * cp_count / moving_points) << "%)" << std::endl;
 std::cout << "ZP (): " << zp_count << " (" 
 << (100.0 * zp_count / moving_points) << "%)" << std::endl;
 }
    
 std::cout << "\n=== ===" << std::endl;
 if (moving_points > 0) {
 std::cout << ": " << predictor_reuse_count << " (" 
 << std::fixed << std::setprecision(2)
 << (100.0 * predictor_reuse_count / moving_points) << "%)" << std::endl;
 std::cout << ": " << predictor_switch_count << " (" 
 << (100.0 * predictor_switch_count / moving_points) << "%)" << std::endl;
        
 if (predictor_reuse_count + predictor_switch_count > 0) {
 double reuse_ratio = 100.0 * predictor_reuse_count / (predictor_reuse_count + predictor_switch_count);
 std::cout << ": " << std::setprecision(2) << reuse_ratio << "%" << std::endl;
 }
 }
    
 std::cout << "\n=== ===" << std::endl;
 if (moving_points > 0) {
 double avg_error = total_prediction_error / moving_points;
 std::cout << ": " << std::scientific << std::setprecision(6) << avg_error 
 << " (" << std::fixed << std::setprecision(2) << (avg_error * 111000) << " )" << std::endl;
 std::cout << ": " << std::scientific << max_prediction_error 
 << " (" << std::fixed << (max_prediction_error * 111000) << " )" << std::endl;
 }
    
 std::cout << "\n=== ===" << std::endl;
 std::cout << ": " << std::setw(8) << predictor_flag_bits << " bits (" 
 << std::setprecision(1) << (100.0 * predictor_flag_bits / total_bits) << "%)" << std::endl;
 std::cout << ": " << std::setw(8) << quantization_bits << " bits (" 
 << (100.0 * quantization_bits / total_bits) << "%)" << std::endl;
 std::cout << ": " << std::setw(8) << total_bits << " bits" << std::endl;
    
 std::cout << "\n=== ===" << std::endl;
 double avg_bits_per_point = static_cast<double>(total_bits) / total_points;
 std::cout << ": " << std::setprecision(2) << avg_bits_per_point << " bits/" << std::endl;
    
 // ：2double = 128 bits
 double compression_ratio = 128.0 / avg_bits_per_point;
 std::cout << ": " << std::setprecision(2) << compression_ratio << ":1" << std::endl;
 std::cout << ": " << std::setprecision(1) << (100.0 - 100.0 / compression_ratio) << "%" << std::endl;
}

void TrajCompressSPCompressor::CompressionStats::PrintDetailedStats() const {
 PrintStats();
    
 std::cout << "\n=== ===" << std::endl;
    
 int moving_points = total_points - 1;
    
    //
 if (moving_points > 0) {
 std::cout << "\n:" << std::endl;
 std::cout << " : " << std::setprecision(1) 
 << (100.0 * ldr_count / moving_points) << "% - ";
 if (ldr_count > moving_points * 0.6) {
 std::cout << "，，" << std::endl;
 } else {
 std::cout << "" << std::endl;
 }
        
 std::cout << " : " << (100.0 * cp_count / moving_points) << "% - ";
 if (cp_count > moving_points * 0.3) {
 std::cout << "，" << std::endl;
 } else {
 std::cout << "" << std::endl;
 }
        
 std::cout << " : " << (100.0 * zp_count / moving_points) << "% - ";
 if (zp_count > moving_points * 0.3) {
 std::cout << "，" << std::endl;
 } else {
 std::cout << "" << std::endl;
 }
 }
    
    //
 if (!prediction_errors.empty()) {
 std::cout << "\n:" << std::endl;
        
        //
 std::vector<double> sorted_errors = prediction_errors;
 std::sort(sorted_errors.begin(), sorted_errors.end());
        
 size_t p50_idx = sorted_errors.size() * 0.50;
 size_t p90_idx = sorted_errors.size() * 0.90;
 size_t p95_idx = sorted_errors.size() * 0.95;
 size_t p99_idx = sorted_errors.size() * 0.99;
        
 std::cout << " P50 (): " << std::scientific << sorted_errors[p50_idx] 
 << " (" << std::fixed << std::setprecision(2) 
 << (sorted_errors[p50_idx] * 111000) << " )" << std::endl;
 std::cout << " P90: " << std::scientific << sorted_errors[p90_idx] 
 << " (" << std::fixed << (sorted_errors[p90_idx] * 111000) << " )" << std::endl;
 std::cout << " P95: " << std::scientific << sorted_errors[p95_idx] 
 << " (" << std::fixed << (sorted_errors[p95_idx] * 111000) << " )" << std::endl;
 std::cout << " P99: " << std::scientific << sorted_errors[p99_idx] 
 << " (" << std::fixed << (sorted_errors[p99_idx] * 111000) << " )" << std::endl;
 }
    
    //
 std::cout << "\n:" << std::endl;
 if (moving_points > 0) {
 double avg_pred_bits = static_cast<double>(predictor_flag_bits) / moving_points;
 double avg_quant_bits = static_cast<double>(quantization_bits) / moving_points;
        
 std::cout << " : " << std::setprecision(2) << avg_pred_bits << " bits/" << std::endl;
 std::cout << " : " << avg_quant_bits << " bits/" << std::endl;
 std::cout << " : " << (avg_pred_bits + avg_quant_bits) << " bits/" << std::endl;
 }
}

// ==================== ====================

TrajCompressSPDecompressor::TrajCompressSPDecompressor(uint8_t* compressed_data, int data_size) {
 input_bit_stream_ = std::make_unique<InputBitStream>(compressed_data, data_size);
 // ：，reserve
    
 // （）
 predictor_frequency_[TrajCompressSPCompressor::PREDICTOR_LDR] = 60;
 predictor_frequency_[TrajCompressSPCompressor::PREDICTOR_CP] = 10;
 predictor_frequency_[TrajCompressSPCompressor::PREDICTOR_ZP] = 30;
 UpdateHuffmanDecoder();
    
 ReadHeader();
}

void TrajCompressSPDecompressor::ReadHeader() {
    //
 block_size_ = input_bit_stream_->ReadInt(16);
 epsilon_ = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
 quant_step_ = 2 * epsilon_; // = 2 * epsilon（epsilon0.999）
    
    //
 double first_lon = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
 double first_lat = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
    
 // timestamp
 uint64_t first_timestamp = input_bit_stream_->ReadLong(64);
    
    //
 current_reconstructed_point_ = GpsPoint(first_lon, first_lat, first_timestamp);
 history_states_.emplace_back(current_reconstructed_point_, GpsPoint(0, 0, 0));
}

bool TrajCompressSPDecompressor::ReadNextPoint(GpsPoint& point) {
 if (first_point_) {
        //
 first_point_ = false;
 point = current_reconstructed_point_;
 return true;
 }
    
 // （ Huffman ）
 // ，
 try {
 // Huffman 
 PredictorType predictor = DecodeWithHuffman();
        
 // （）
 AddPredictorToWindow(predictor);
        
        //
 last_used_predictor_ = predictor;
        
 // timestamp delta（）
 uint64_t timestamp_delta = input_bit_stream_->ReadLong(64);
 uint64_t current_timestamp = current_reconstructed_point_.timestamp + timestamp_delta;
        
 // （timestamp）
 GpsPoint pred_ldr, pred_cp, pred_zp;
 ParallelPredict(pred_ldr, pred_cp, pred_zp, current_timestamp);
        
        //
 GpsPoint predicted_point;
 switch (predictor) {
 case TrajCompressSPCompressor::PREDICTOR_LDR: predicted_point = pred_ldr; break;
 case TrajCompressSPCompressor::PREDICTOR_CP: predicted_point = pred_cp; break;
 case TrajCompressSPCompressor::PREDICTOR_ZP: predicted_point = pred_zp; break;
 }
        
        //
 int64_t quantized_delta_lon = ZigZagCodec::Decode(
 EliasGammaCodec::Decode(input_bit_stream_.get()) - 1
 );
 int64_t quantized_delta_lat = ZigZagCodec::Decode(
 EliasGammaCodec::Decode(input_bit_stream_.get()) - 1
 );
        
 // （quant_step_，）
 GpsPoint reconstructed_delta(
 quantized_delta_lon * quant_step_,
 quantized_delta_lat * quant_step_
 );
 GpsPoint reconstructed_point = predicted_point + reconstructed_delta;
 reconstructed_point.timestamp = current_timestamp; // timestamp
    
        //
 UpdateHistory(reconstructed_point);
        
 point = reconstructed_point;
 return true;
 } catch (...) {
 // ，
 return false;
 }
}

std::vector<TrajCompressSPDecompressor::GpsPoint> TrajCompressSPDecompressor::ReadAllPoints() {
 std::vector<GpsPoint> points;
 GpsPoint point;
    
 while (ReadNextPoint(point)) {
 points.push_back(point);
 }
    
 return points;
}

void TrajCompressSPDecompressor::ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp) {
 // （ZP）
 pred_zp = current_reconstructed_point_;
    
 if (history_states_.size() < 2) {
 pred_ldr = current_reconstructed_point_;
 pred_cp = current_reconstructed_point_;
 return;
 }
    
 // ，int64_ttimestampunderflow
 int64_t delta_time_signed = static_cast<int64_t>(current_timestamp) - static_cast<int64_t>(current_reconstructed_point_.timestamp);
 double dt = (delta_time_signed > 0) ? static_cast<double>(delta_time_signed) : 1.0; // 1fallback
    
 // （LDR）：
 GpsPoint velocity = history_states_[history_states_.size() - 1].velocity;
 pred_ldr = GpsPoint(
 current_reconstructed_point_.longitude + velocity.longitude * dt,
 current_reconstructed_point_.latitude + velocity.latitude * dt,
 current_timestamp
 );
    
 // （CP）：（）
 // ：Simple，jerk
 if (history_states_.size() >= 3 && dt > 0) {
 // （）
 GpsPoint velocity = history_states_[history_states_.size() - 1].velocity;
 GpsPoint prev_velocity = history_states_[history_states_.size() - 2].velocity;
 GpsPoint acceleration = velocity - prev_velocity;
        
 // ：position + velocity*dt + 0.5*acceleration*dt^2
 double dt_sq_half = dt * dt * 0.5;
 pred_cp = GpsPoint(
 current_reconstructed_point_.longitude + velocity.longitude * dt + acceleration.longitude * dt_sq_half,
 current_reconstructed_point_.latitude + velocity.latitude * dt + acceleration.latitude * dt_sq_half,
 current_timestamp
 );
 } else {
 pred_cp = pred_ldr;
 }
}

void TrajCompressSPDecompressor::UpdateHistory(const GpsPoint& reconstructed_point) {
 // （/），int64_ttimestampunderflow
 GpsPoint velocity(0, 0, 0);
 if (reconstructed_point.timestamp > 0 && current_reconstructed_point_.timestamp > 0) {
 int64_t delta_time_signed = static_cast<int64_t>(reconstructed_point.timestamp) - static_cast<int64_t>(current_reconstructed_point_.timestamp);
 if (delta_time_signed > 0) {
 double dt = static_cast<double>(delta_time_signed);
 velocity = GpsPoint(
 (reconstructed_point.longitude - current_reconstructed_point_.longitude) / dt,
 (reconstructed_point.latitude - current_reconstructed_point_.latitude) / dt,
 0
 );
 } else {
 // Timestamp，fallback
 velocity = reconstructed_point - current_reconstructed_point_;
 }
 } else {
 // ，
 velocity = reconstructed_point - current_reconstructed_point_;
 }
    
    //
 current_reconstructed_point_ = reconstructed_point;
    
    //
 history_states_.emplace_back(reconstructed_point, velocity);
    
 // （：dequepop_frontO(1)）
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.pop_front();
 }
}
void TrajCompressSPCompressor::EncodePredictionOptimized(PredictorType predictor,
 const GpsPoint& current_point,
 const GpsPoint& predicted_point) {
 // 1. Huffman 
 EncodeWithHuffman(predictor);
    
 // 2. 
 AddPredictorToWindow(predictor);
    
 // 2.5. timestamp delta（，）
 uint64_t timestamp_delta = current_point.timestamp - current_reconstructed_point_.timestamp;
 int ts_bits = output_bit_stream_->WriteLong(timestamp_delta, 64);
 compressed_size_in_bits_ += ts_bits;
 stats_.timestamp_bits += ts_bits;
    
 // 3. （）
 GpsPoint delta = current_point - predicted_point;
    
 // 4. （kQuantStep = 2 * epsilon * 0.999，Serf-QT）
 // ： ≤ kQuantStep/2 = epsilon * 0.999
 // ≤ sqrt(2) * epsilon * 0.999 ≈ 1.41*epsilon*0.999
 // （），+
 int64_t quantized_delta_lon = static_cast<int64_t>(std::round(delta.longitude / kQuantStep));
 int64_t quantized_delta_lat = static_cast<int64_t>(std::round(delta.latitude / kQuantStep));
    
 // 5. ZigZag（）
 uint64_t zigzag_lon = ZigZagCodec::Encode(quantized_delta_lon);
 uint64_t zigzag_lat = ZigZagCodec::Encode(quantized_delta_lat);
    
 // 6. Elias Gamma（+1）
 int bits_lon = EliasGammaCodec::Encode(zigzag_lon + 1, output_bit_stream_.get());
 int bits_lat = EliasGammaCodec::Encode(zigzag_lat + 1, output_bit_stream_.get());
    
 compressed_size_in_bits_ += bits_lon + bits_lat;
 stats_.quantization_bits += bits_lon + bits_lat;
    
 // 7. （）
 GpsPoint reconstructed_point = predicted_point + GpsPoint(
 quantized_delta_lon * kQuantStep,
 quantized_delta_lat * kQuantStep
 );
 // timestamp
 reconstructed_point.timestamp = current_point.timestamp;
    
    //
 UpdateReconstructedState(reconstructed_point);
}


void TrajCompressSPCompressor::UpdateReconstructedState(const GpsPoint& reconstructed_point) {
 // （/），int64_ttimestampunderflow
 GpsPoint velocity(0, 0, 0);
 if (history_states_.size() >= 1) {
 const GpsPoint& prev_point = history_states_.back().reconstructed_point;
 if (reconstructed_point.timestamp > 0 && prev_point.timestamp > 0) {
 int64_t delta_time_signed = static_cast<int64_t>(reconstructed_point.timestamp) - static_cast<int64_t>(prev_point.timestamp);
 if (delta_time_signed > 0) {
 double dt = static_cast<double>(delta_time_signed);
 velocity = GpsPoint(
 (reconstructed_point.longitude - prev_point.longitude) / dt,
 (reconstructed_point.latitude - prev_point.latitude) / dt,
 0
 );
 } else {
 // Timestamp，fallback
 velocity = reconstructed_point - prev_point;
 }
 } else {
 // ，（dt=1）
 velocity = reconstructed_point - prev_point;
 }
 }
    
    //
 current_reconstructed_point_ = reconstructed_point;
    
    //
 history_states_.emplace_back(reconstructed_point, velocity);
    
 // （：dequepop_frontO(1)）
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.pop_front();
 }
}

// ==================== Huffman ====================

void TrajCompressSPCompressor::UpdateHuffmanCodes() {
 // Huffman 
 // ， Huffman 
    
 // -，（）
 struct FreqPair {
 int freq;
 PredictorType predictor;
        
 bool operator<(const FreqPair& other) const {
 return freq > other.freq; // 
 }
 };
    
 std::vector<FreqPair> freq_pairs = {
 {predictor_frequency_[PREDICTOR_LDR], PREDICTOR_LDR},
 {predictor_frequency_[PREDICTOR_CP], PREDICTOR_CP},
 {predictor_frequency_[PREDICTOR_ZP], PREDICTOR_ZP}
 };
    
 std::sort(freq_pairs.begin(), freq_pairs.end());
    
    //
 // ：1 bit (0)
 // ：2 bits (10)
 // ：2 bits (11)
    
 if (freq_pairs[0].freq > 0) {
 huffman_codes_[freq_pairs[0].predictor] = HuffmanCode({false}); // "0"
 }
 if (freq_pairs[1].freq > 0) {
 huffman_codes_[freq_pairs[1].predictor] = HuffmanCode({true, false}); // "10"
 }
 if (freq_pairs[2].freq > 0) {
 huffman_codes_[freq_pairs[2].predictor] = HuffmanCode({true, true}); // "11"
 }
}

void TrajCompressSPCompressor::EncodeWithHuffman(PredictorType predictor) {
 // Huffman 
 const HuffmanCode& code = huffman_codes_[predictor];
    
 for (bool bit : code.bits) {
 compressed_size_in_bits_ += output_bit_stream_->WriteBit(bit);
 }
    
 stats_.predictor_flag_bits += code.length;
}

void TrajCompressSPCompressor::AddPredictorToWindow(PredictorType predictor) {
 // ：vector，erase(begin())O(n)
    
 // ，
 if (window_size_ == kSlidingWindowSize) {
 PredictorType oldest = predictor_window_[window_head_];
 predictor_frequency_[oldest]--;
 window_head_ = (window_head_ + 1) % kSlidingWindowSize;
 window_size_--;
 }
    
    //
 int tail = (window_head_ + window_size_) % kSlidingWindowSize;
 predictor_window_[tail] = predictor;
 predictor_frequency_[predictor]++;
 window_size_++;
    
 // 100 Huffman 
 // ，
 if (window_size_ % 100 == 0 && window_size_ >= 100) {
 UpdateHuffmanCodes();
 }
}

// ==================== Huffman ====================

void TrajCompressSPDecompressor::UpdateHuffmanDecoder() {
 // Huffman 
    //
    
 struct FreqPair {
 int freq;
 PredictorType predictor;
        
 bool operator<(const FreqPair& other) const {
 return freq > other.freq;
 }
 };
    
 std::vector<FreqPair> freq_pairs = {
 {predictor_frequency_[TrajCompressSPCompressor::PREDICTOR_LDR], 
 TrajCompressSPCompressor::PREDICTOR_LDR},
 {predictor_frequency_[TrajCompressSPCompressor::PREDICTOR_CP], 
 TrajCompressSPCompressor::PREDICTOR_CP},
 {predictor_frequency_[TrajCompressSPCompressor::PREDICTOR_ZP], 
 TrajCompressSPCompressor::PREDICTOR_ZP}
 };
    
 std::sort(freq_pairs.begin(), freq_pairs.end());
    
    //
 // ：0 -> , 10 -> , 11 -> 
 huffman_decoder_map_[0] = freq_pairs[0].predictor; // 
 huffman_decoder_map_[1] = freq_pairs[1].predictor; // 
 huffman_decoder_map_[2] = freq_pairs[2].predictor; // 
}

TrajCompressSPCompressor::PredictorType TrajCompressSPDecompressor::DecodeWithHuffman() {
 // Huffman （）
 // ：，
    
    //
 bool first_bit = input_bit_stream_->ReadBit();
    
 if (!first_bit) {
 // 0 -> 
 return huffman_decoder_map_[0];
 } else {
        //
 bool second_bit = input_bit_stream_->ReadBit();
 if (!second_bit) {
 // 10 -> 
 return huffman_decoder_map_[1];
 } else {
 // 11 -> 
 return huffman_decoder_map_[2];
 }
 }
}

void TrajCompressSPDecompressor::AddPredictorToWindow(PredictorType predictor) {
 // ：vector，erase(begin())O(n)
    
 // ，
 if (window_size_ == kSlidingWindowSize) {
 PredictorType oldest = predictor_window_[window_head_];
 predictor_frequency_[oldest]--;
 window_head_ = (window_head_ + 1) % kSlidingWindowSize;
 window_size_--;
 }
    
    //
 int tail = (window_head_ + window_size_) % kSlidingWindowSize;
 predictor_window_[tail] = predictor;
 predictor_frequency_[predictor]++;
 window_size_++;
    
 // 100 （）
 if (window_size_ % 100 == 0 && window_size_ >= 100) {
 UpdateHuffmanDecoder();
 }
}
