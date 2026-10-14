#include "baselines/serf/serf_qt_curve_decompressor.h"
#include "utils/double.h"
#include "utils/zig_zag_codec.h"
#include "utils/elias_gamma_codec.h"

std::vector<double> SerfQtCurveDecompressor::Decompress(const Array<uint8_t> &bs) {
 input_bit_stream_->SetBuffer(bs);
 block_size_ = input_bit_stream_->ReadInt(32); // 32
 max_diff_ = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
  
 // timestamp（TrajSP）
 uint64_t first_timestamp = input_bit_stream_->ReadLong(64);
  
 // Reset state
 history_states_.clear();
 history_states_.reserve(kMaxHistorySize);
 first_ = true;
  
 std::vector<double> decompressed_value_list;
 decompressed_value_list.reserve(block_size_);
  
  //
 int64_t decodeValue = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_bit_stream_.get()) - 1);
 double recoverValue = 2.0 + 2 * max_diff_ * static_cast<double>(decodeValue);
 history_states_.emplace_back(recoverValue, 0.0, first_timestamp);
 decompressed_value_list.emplace_back(recoverValue);
 first_ = false;
 block_size_--;
  
  //
 while (block_size_--) {
 decompressed_value_list.emplace_back(NextValue());
 }
  
 return decompressed_value_list;
}

double SerfQtCurveDecompressor::CurvePredict(uint64_t current_timestamp) const {
 if (history_states_.empty()) {
 return 2.0; // default value
 }
  
 size_t history_size = history_states_.size();
  
 if (history_size == 1) {
 return history_states_[0].value;
 }
  
 // dt（）
 double dt;
 if (current_timestamp > 0 && history_states_[history_size - 1].timestamp > 0) {
 int64_t delta_time_signed = static_cast<int64_t>(current_timestamp) - static_cast<int64_t>(history_states_[history_size - 1].timestamp);
 dt = (delta_time_signed > 0) ? static_cast<double>(delta_time_signed) : 1.0;
 } else {
 dt = 1.0;
 }
  
 if (history_size == 2) {
 double current = history_states_[1].value;
 double velocity = history_states_[1].velocity;
 return current + velocity * dt;
 }
  
 // TrajCompress-SPCP
 double current = history_states_[history_size - 1].value;
  
 if (history_size >= 5) {
 double v1 = history_states_[history_size - 1].velocity;
 double v2 = history_states_[history_size - 2].velocity;
 double v3 = history_states_[history_size - 3].velocity;
    
 double smoothed_velocity = 0.45 * v1 + 0.35 * v2 + 0.20 * v3;
 double a1 = v1 - v2;
 double a2 = v2 - v3;
 double jerk = a1 - a2;
    
 return current + smoothed_velocity * dt + a1 * dt + 0.5 * jerk * dt;
    
 } else if (history_size == 4) {
 double v1 = history_states_[history_size - 1].velocity;
 double v2 = history_states_[history_size - 2].velocity;
 double v3 = history_states_[history_size - 3].velocity;
    
 double smoothed_velocity = 0.45 * v1 + 0.35 * v2 + 0.20 * v3;
 double acceleration = v1 - v2;
    
 return current + smoothed_velocity * dt + acceleration * dt;
    
 } else { // history_size == 3
 double velocity = history_states_[history_size - 1].velocity;
 double prev_velocity = history_states_[history_size - 2].velocity;
 double acceleration = velocity - prev_velocity;
    
 return current + velocity * dt + acceleration * dt;
 }
}

double SerfQtCurveDecompressor::NextValue() {
 // timestamp delta，（）
 uint64_t timestamp_delta_bits = input_bit_stream_->ReadLong(64);
 int64_t timestamp_delta = static_cast<int64_t>(timestamp_delta_bits);
 uint64_t current_timestamp = history_states_.back().timestamp + timestamp_delta;
  
  //
 double predicted = CurvePredict(current_timestamp);
  
  //
 int64_t decodeValue = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_bit_stream_.get()) - 1);
 double recoverValue = predicted + 2 * max_diff_ * static_cast<double>(decodeValue);
  
  //
 const HistoryState& last_state = history_states_.back();
 double velocity = 0.0;
  
 if (current_timestamp > 0 && last_state.timestamp > 0) {
 int64_t delta_time_signed = static_cast<int64_t>(current_timestamp) - static_cast<int64_t>(last_state.timestamp);
 if (delta_time_signed > 0) {
 double dt = static_cast<double>(delta_time_signed);
 velocity = (recoverValue - last_state.value) / dt;
 } else {
 velocity = recoverValue - last_state.value;
 }
 } else {
 velocity = recoverValue - last_state.value;
 }
  
  //
 history_states_.emplace_back(recoverValue, velocity, current_timestamp);
  
  //
 if (history_states_.size() > kMaxHistorySize) {
 history_states_.erase(history_states_.begin());
 }
  
 return recoverValue;
}

