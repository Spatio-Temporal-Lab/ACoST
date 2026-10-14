#ifndef SERF_QT_LINEAR_DECOMPRESSOR_H
#define SERF_QT_LINEAR_DECOMPRESSOR_H

#include <memory>
#include <vector>

#include "utils/input_bit_stream.h"
#include "utils/array.h"

class SerfQtLinearDecompressor {
 public:
  SerfQtLinearDecompressor() = default;
  std::vector<double> Decompress(const Array<uint8_t> &bs);

 private:
  int block_size_ = 0;
  double max_diff_ = 0;
  std::unique_ptr<InputBitStream> input_bit_stream_ = std::make_unique<InputBitStream>();
  double prev_value1_ = 2;  // most recent value
  double prev_value2_ = 2;  // second most recent value
  uint64_t prev_timestamp1_ = 0;  // most recent timestamp
  uint64_t prev_timestamp2_ = 0;  // second most recent timestamp
  bool first_ = true;
  bool second_ = true;

  double NextValue();
  double LinearPredict(uint64_t current_timestamp) const;
};

#endif //SERF_QT_LINEAR_DECOMPRESSOR_H
