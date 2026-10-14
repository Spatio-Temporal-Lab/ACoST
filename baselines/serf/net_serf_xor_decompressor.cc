#include "baselines/serf/net_serf_xor_decompressor.h"

NetSerfXORDecompressor::NetSerfXORDecompressor(int window_size, long adjust_digit) : kWindowSize(window_size),
                                                                                     kAdjustDigit(adjust_digit) {}

double NetSerfXORDecompressor::Decompress(Array<uint8_t> &bs) {
  input_bit_stream_->SetBuffer(bs);
  return Double::LongBitsToDouble(ReadValue()) - kAdjustDigit;
}

uint64_t NetSerfXORDecompressor::ReadValue() {
  // empty read 4 bits for getting rid of transmit header
  input_bit_stream_->ReadInt(4);
  if (number_of_values_ >= kWindowSize) {
    UpdatePositionsIfNeeded();
  }
  NextValue();
  ++number_of_values_;
  return stored_val_;
}

void NetSerfXORDecompressor::NextValue() {
  uint64_t value;
  int center_bits;
  if (input_bit_stream_->ReadInt(1) == 1) {
    // case 1
    center_bits = 64 - stored_leading_zeros_ - stored_trailing_zeros_;

    value = input_bit_stream_->ReadLong(center_bits) << stored_trailing_zeros_;
    value = stored_val_ ^ value;
    stored_val_ = value;
  } else if (input_bit_stream_->ReadInt(1) == 0) {
    // case 00
    int lead_and_trail = input_bit_stream_->ReadInt(
        leading_bits_per_value_ + trailing_bits_per_value_);
    int lead = lead_and_trail >> trailing_bits_per_value_;
    int trail = ~(0xffffffff << trailing_bits_per_value_) & lead_and_trail;
    stored_leading_zeros_ = leading_representation_[lead];
    stored_trailing_zeros_ = trailing_representation_[trail];
    center_bits = 64 - stored_leading_zeros_ - stored_trailing_zeros_;

    value = input_bit_stream_->ReadLong(center_bits) << stored_trailing_zeros_;
    value = stored_val_ ^ value;
    stored_val_ = value;
  }
}

void NetSerfXORDecompressor::UpdatePositionsIfNeeded() {
  if (input_bit_stream_->ReadBit() == 1) {
    UpdateLeadingRepresentation();
    UpdateTrailingRepresentation();
  }
  number_of_values_ = 0;
}

void NetSerfXORDecompressor::UpdateLeadingRepresentation() {
  int num = input_bit_stream_->ReadInt(5);
  if (num == 0) {
    num = 32;
  }
  leading_bits_per_value_ = PostOfficeSolver::kPositionLength2Bits[num];
  leading_representation_ = Array<int>(num);
  for (int i = 0; i < num; i++) {
    leading_representation_[i] = input_bit_stream_->ReadInt(6);
  }
}

void NetSerfXORDecompressor::UpdateTrailingRepresentation() {
  int num = input_bit_stream_->ReadInt(5);
  if (num == 0) {
    num = 32;
  }
  trailing_bits_per_value_ = PostOfficeSolver::kPositionLength2Bits[num];
  trailing_representation_ = Array<int>(num);
  for (int i = 0; i < num; i++) {
    trailing_representation_[i] = input_bit_stream_->ReadInt(6);
  }
}
