#ifndef NET_SERF_QT_DECOMPRESSOR_H
#define NET_SERF_QT_DECOMPRESSOR_H

#include <cstdint>
#include <memory>

#include "utils/input_bit_stream.h"
#include "utils/zig_zag_codec.h"
#include "utils/elias_gamma_codec.h"

class NetSerfQtDecompressor {
 public:
  explicit NetSerfQtDecompressor(double max_diff);

  double Decompress(Array<uint8_t> &bs);

 private:
  const double kMaxDiff;
  double pre_value_ = 2;
  std::unique_ptr<InputBitStream> input_bit_stream_ = std::make_unique<InputBitStream>();
};

#endif  // NET_SERF_QT_DECOMPRESSOR_H
