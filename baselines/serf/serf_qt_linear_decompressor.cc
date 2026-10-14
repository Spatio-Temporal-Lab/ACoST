#include "baselines/serf/serf_qt_linear_decompressor.h"
#include "utils/double.h"
#include "utils/zig_zag_codec.h"
#include "utils/elias_gamma_codec.h"

std::vector<double> SerfQtLinearDecompressor::Decompress(const Array<uint8_t> &bs) {
 input_bit_stream_->SetBuffer(bs);
 block_size_ = input_bit_stream_->ReadInt(32); // 32
 max_diff_ = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
  
 // timestamp（TrajSP）
 prev_timestamp1_ = input_bit_stream_->ReadLong(64);
  
 // Reset state
 prev_value1_ = 2;
 prev_value2_ = 2;
 prev_timestamp2_ = 0;
 first_ = true;
 second_ = true;
  
 std::vector<double> decompressed_value_list;
 decompressed_value_list.reserve(block_size_);
  
 while (block_size_--) {
 decompressed_value_list.emplace_back(NextValue());
 }
  
 return decompressed_value_list;
}

double SerfQtLinearDecompressor::LinearPredict(uint64_t current_timestamp) const {
 if (first_) {
 return 2.0; // default value for first point
 } else if (second_) {
 return prev_value1_; // use previous value for second point
 } else {
 // （TrajSP）
 if (current_timestamp > 0 && prev_timestamp1_ > 0 && prev_timestamp2_ > 0) {
 int64_t dt1_signed = static_cast<int64_t>(prev_timestamp1_) - static_cast<int64_t>(prev_timestamp2_);
 int64_t dt2_signed = static_cast<int64_t>(current_timestamp) - static_cast<int64_t>(prev_timestamp1_);
      
 if (dt1_signed > 0 && dt2_signed > 0) {
 double velocity = (prev_value1_ - prev_value2_) / static_cast<double>(dt1_signed);
 return prev_value1_ + velocity * static_cast<double>(dt2_signed);
 }
 }
    
 // Fallback: 
 return 2.0 * prev_value1_ - prev_value2_;
 }
}

double SerfQtLinearDecompressor::NextValue() {
 double predicted;
 if (first_) {
 first_ = false;
 // ：
 int64_t decodeValue = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_bit_stream_.get()) - 1);
 predicted = 2.0; // default prediction for first value
 double recoverValue = predicted + 2 * max_diff_ * static_cast<double>(decodeValue);
 prev_value1_ = recoverValue;
 return recoverValue;
 } else if (second_) {
 second_ = false;
 // ：timestamp delta，（TrajSP）
 uint64_t timestamp_delta_bits = input_bit_stream_->ReadLong(64);
 int64_t timestamp_delta = static_cast<int64_t>(timestamp_delta_bits);
 uint64_t current_timestamp = prev_timestamp1_ + timestamp_delta;
    
 int64_t decodeValue = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_bit_stream_.get()) - 1);
 predicted = prev_value1_; // use previous value as prediction for second value
 double recoverValue = predicted + 2 * max_diff_ * static_cast<double>(decodeValue);
 prev_value2_ = prev_value1_;
 prev_timestamp2_ = prev_timestamp1_;
 prev_value1_ = recoverValue;
 prev_timestamp1_ = current_timestamp;
 return recoverValue;
 } else {
 // ：timestamp delta，（TrajSP）
 uint64_t timestamp_delta_bits = input_bit_stream_->ReadLong(64);
 int64_t timestamp_delta = static_cast<int64_t>(timestamp_delta_bits);
 uint64_t current_timestamp = prev_timestamp1_ + timestamp_delta;
    
 int64_t decodeValue = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_bit_stream_.get()) - 1);
 // Linear prediction for third value onwards (timestamp)
 predicted = LinearPredict(current_timestamp);
 double recoverValue = predicted + 2 * max_diff_ * static_cast<double>(decodeValue);
    
 // Update history
 prev_value2_ = prev_value1_;
 prev_timestamp2_ = prev_timestamp1_;
 prev_value1_ = recoverValue;
 prev_timestamp1_ = current_timestamp;
 return recoverValue;
 }
}
