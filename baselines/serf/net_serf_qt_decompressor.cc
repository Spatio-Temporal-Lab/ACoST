#include "baselines/serf/net_serf_qt_decompressor.h"

NetSerfQtDecompressor::NetSerfQtDecompressor(double max_diff) : kMaxDiff(max_diff * 0.999) {}

double NetSerfQtDecompressor::Decompress(Array<uint8_t> &bs) {
  input_bit_stream_->SetBuffer(bs);
  input_bit_stream_->ReadInt(4);
  long decodeValue = ZigZagCodec::Decode(EliasGammaCodec::Decode(input_bit_stream_.get()) - 1);
  pre_value_ = pre_value_ + 2 * kMaxDiff * static_cast<double>(decodeValue);
  return pre_value_;
}
