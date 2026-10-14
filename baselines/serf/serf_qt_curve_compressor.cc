#include "baselines/serf/serf_qt_curve_compressor.h"
#include <cmath>
#include "utils/double.h"
#include "utils/elias_gamma_codec.h"
#include "utils/zig_zag_codec.h"

SerfQtCurveCompressor::SerfQtCurveCompressor(int block_size, double max_diff) 
 : kBlockSize(block_size), kMaxDiff(max_diff * 0.999) {
 output_bit_stream_ = std::make_unique<OutputBitStream>(2 * block_size * 8);
 history_states_.reserve(kMaxHistorySize);
}

double SerfQtCurveCompressor::CurvePredict(uint64_t current_timestamp) const {
 if (history_states_.empty()) {
 return 2.0; // default value
 }
  
 size_t history_size = history_states_.size();
  
 if (history_size == 1) {
 // ，
 return history_states_[0].value;
 }
  
 // dt（），int64_ttimestampunderflow
 double dt;
 if (current_timestamp > 0 && history_states_[history_size - 1].timestamp > 0) {
 int64_t delta_time_signed = static_cast<int64_t>(current_timestamp) - static_cast<int64_t>(history_states_[history_size - 1].timestamp);
 dt = (delta_time_signed > 0) ? static_cast<double>(delta_time_signed) : 1.0; // 1fallback
 } else {
 dt = 1.0; // 1，
 }
  
 if (history_size == 2) {
 // ，（ * dt）
 double current = history_states_[1].value;
 double velocity = history_states_[1].velocity;
 return current + velocity * dt;
 }
  
 // TrajCompress-SPCP（）
 double current = history_states_[history_size - 1].value;
  
 if (history_size >= 5) {
 // ：5，jerk
 double v1 = history_states_[history_size - 1].velocity;
 double v2 = history_states_[history_size - 2].velocity;
 double v3 = history_states_[history_size - 3].velocity;
    
 // ：3，
 double smoothed_velocity = 0.45 * v1 + 0.35 * v2 + 0.20 * v3;
    
 // ：2（：/²）
 double a1 = v1 - v2;
 double a2 = v2 - v3;
    
 // Jerk（，：/³）
 double jerk = a1 - a2;
    
 // ：dt、jerk
 // smoothed_velocity * dt + a1 * dt + 0.5 * jerk * dt
 return current + smoothed_velocity * dt + a1 * dt + 0.5 * jerk * dt;
    
 } else if (history_size == 4) {
 // （）
 double v1 = history_states_[history_size - 1].velocity;
 double v2 = history_states_[history_size - 2].velocity;
 double v3 = history_states_[history_size - 3].velocity;
    
 double smoothed_velocity = 0.45 * v1 + 0.35 * v2 + 0.20 * v3;
 double acceleration = v1 - v2;
    
 // dt
 return current + smoothed_velocity * dt + acceleration * dt;
    
 } else { // history_size == 3
    //
 double velocity = history_states_[history_size - 1].velocity;
 double prev_velocity = history_states_[history_size - 2].velocity;
 double acceleration = velocity - prev_velocity;
    
 // dt
 return current + velocity * dt + acceleration * dt;
 }
}

void SerfQtCurveCompressor::AddValue(double v, uint64_t timestamp) {
 if (first_) {
 first_ = false;
    //
 compressed_size_in_bits_ += output_bit_stream_->WriteInt(kBlockSize, 32); // 32
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(Double::DoubleToLongBits(kMaxDiff), 64);
 // timestamp（TrajSP）
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(timestamp, 64);
    
 // ：(2.0)
 long q = static_cast<long>(std::round((v - 2.0) / (2 * kMaxDiff)));
 double recoverValue = 2.0 + 2 * kMaxDiff * static_cast<double>(q);
 compressed_size_in_bits_ += EliasGammaCodec::Encode(
 ZigZagCodec::Encode(static_cast<int64_t>(q)) + 1,
 output_bit_stream_.get());
    
 // （0）
 history_states_.emplace_back(recoverValue, 0.0, timestamp);
 return;
 }
  
 // timestamp delta（TrajSP）
 int64_t timestamp_delta_signed = static_cast<int64_t>(timestamp) - static_cast<int64_t>(history_states_.back().timestamp);
 uint64_t timestamp_delta = static_cast<uint64_t>(timestamp_delta_signed);
 compressed_size_in_bits_ += output_bit_stream_->WriteLong(timestamp_delta, 64);
  
 // （）
 double predicted = CurvePredict(timestamp);
  
  //
 long q = static_cast<long>(std::round((v - predicted) / (2 * kMaxDiff)));
 double recoverValue = predicted + 2 * kMaxDiff * static_cast<double>(q);
 compressed_size_in_bits_ += EliasGammaCodec::Encode(
 ZigZagCodec::Encode(static_cast<int64_t>(q)) + 1,
 output_bit_stream_.get());
  
 // （/），
 const HistoryState& last_state = history_states_.back();
 double velocity = 0.0;
  
 if (timestamp > 0 && last_state.timestamp > 0) {
 // int64_tdeltatimestampunderflow
 int64_t delta_time_signed = static_cast<int64_t>(timestamp) - static_cast<int64_t>(last_state.timestamp);
 if (delta_time_signed > 0) {
 double dt = static_cast<double>(delta_time_signed);
 velocity = (recoverValue - last_state.value) / dt; // /
 } else {
 // Timestamp，fallback（）
 velocity = recoverValue - last_state.value;
 }
 } else {
 // ，（dt=1）
 velocity = recoverValue - last_state.value;
 }
  
  //
 history_states_.emplace_back(recoverValue, velocity, timestamp);
  
  //
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.erase(history_states_.begin());
 }
}

Array<uint8_t> SerfQtCurveCompressor::compressed_bytes() {
 return compressed_bytes_;
}

void SerfQtCurveCompressor::Close() {
 output_bit_stream_->Flush();
 compressed_bytes_ = output_bit_stream_->GetBuffer(std::ceil(compressed_size_in_bits_ / 8.0));
 output_bit_stream_->Refresh();
 first_ = true;
 history_states_.clear();
 stored_compressed_size_in_bits_ = compressed_size_in_bits_;
 compressed_size_in_bits_ = 0;
}

long SerfQtCurveCompressor::get_compressed_size_in_bits() const {
 return stored_compressed_size_in_bits_;
}
