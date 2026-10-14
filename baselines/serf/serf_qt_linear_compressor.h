#ifndef SERF_QT_LINEAR_COMPRESSOR_H
#define SERF_QT_LINEAR_COMPRESSOR_H

#include <cstdint>
#include <memory>

#include "utils/output_bit_stream.h"
#include "utils/array.h"

/*
 * Serf-QT with Linear Prediction instead of Previous Value Prediction
 * +------------+-----------------+---------------+
 * |16bits - len|64bits - max_diff|Encoded Content|
 * +------------+-----------------+---------------+
 */

class SerfQtLinearCompressor {
 public:
 SerfQtLinearCompressor(int block_size, double max_diff);

 void AddValue(double v, uint64_t timestamp = 0); // （）

 Array<uint8_t> compressed_bytes();

 void Close();

 long get_compressed_size_in_bits() const;

 private:
 const double kMaxDiff;
 const int kBlockSize;
 bool first_ = true;
 bool second_ = true;
 std::unique_ptr<OutputBitStream> output_bit_stream_;
 Array<uint8_t> compressed_bytes_;
 double prev_value1_ = 2; // most recent value
 double prev_value2_ = 2; // second most recent value
 uint64_t prev_timestamp1_ = 0; // timestamp for prev_value1_
 uint64_t prev_timestamp2_ = 0; // timestamp for prev_value2_
 long compressed_size_in_bits_ = 0;
 long stored_compressed_size_in_bits_ = 0;
  
 double LinearPredict(uint64_t current_timestamp) const;
};

#endif // SERF_QT_LINEAR_COMPRESSOR_H
