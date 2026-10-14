#ifndef SERF_QT_DECOMPRESSOR_H
#define SERF_QT_DECOMPRESSOR_H

#include <memory>
#include <vector>

#include "utils/double.h"
#include "utils/input_bit_stream.h"
#include "utils/zig_zag_codec.h"
#include "utils/elias_gamma_codec.h"

class SerfQtDecompressor {
 public:
  SerfQtDecompressor() = default;
  std::vector<double> Decompress(const Array<uint8_t> &bs);

 private:
  int block_size_ = 0;
  double max_diff_ = 0;
  std::unique_ptr<InputBitStream> input_bit_stream_ = std::make_unique<InputBitStream>();
  double pre_value_ = 2;

  double NextValue();
};

#endif //SERF_QT_DECOMPRESSOR_H
