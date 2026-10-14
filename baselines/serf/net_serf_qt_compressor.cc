#include "baselines/serf/net_serf_qt_compressor.h"

NetSerfQtCompressor::NetSerfQtCompressor(double error_bound) : kMaxDiff(error_bound * 0.999) {}

Array<uint8_t> NetSerfQtCompressor::Compress(double v) {
 int written_bits_count = output_bit_stream_->WriteInt(0, 4);
 long q = static_cast<long>(std::round((v - pre_value_) / (2 * kMaxDiff)));
 double recover_value = pre_value_ + 2 * kMaxDiff * static_cast<double>(q);
 written_bits_count +=
 EliasGammaCodec::Encode(ZigZagCodec::Encode(static_cast<int64_t>(q)) + 1, output_bit_stream_.get());
 pre_value_ = recover_value;
 output_bit_stream_->Flush();
  
 // bit （）
 last_compressed_bits_ = written_bits_count;
  
 Array<uint8_t> result = output_bit_stream_->GetBuffer(std::ceil(written_bits_count / 8.0));
 output_bit_stream_->Refresh();
 return result;
}
