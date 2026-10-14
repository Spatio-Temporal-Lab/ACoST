#include "baselines/serf/serf_qt_decompressor.h"

std::vector<double> SerfQtDecompressor::Decompress(const Array<uint8_t> &bs) {
 input_bit_stream_->SetBuffer(bs);
 block_size_ = input_bit_stream_->ReadInt(32); // 32
 max_diff_ = Double::LongBitsToDouble(input_bit_stream_->ReadLong(64));
 pre_value_ = 2;
 std::vector<double> decompressed_value_list;
 decompressed_value_list.reserve(block_size_);
 while (block_size_--) decompressed_value_list.emplace_back(NextValue());
 return decompressed_value_list;
}

double SerfQtDecompressor::NextValue() {
 int64_t decodeValue = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_bit_stream_.get()) - 1);
 double recoverValue = pre_value_ + 2 * max_diff_ * static_cast<double>(decodeValue);
 pre_value_ = recoverValue;
 return recoverValue;
}
