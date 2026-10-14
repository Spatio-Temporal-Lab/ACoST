#ifndef NET_SERF_XOR_COMPRESSOR_H
#define NET_SERF_XOR_COMPRESSOR_H

#include <cmath>
#include <cstdint>

#include "utils/array.h"
#include "utils/double.h"
#include "utils/serf_utils_64.h"
#include "utils/output_bit_stream.h"
#include "utils/post_office_solver.h"

class NetSerfXORCompressor {
 public:
 NetSerfXORCompressor(int window_size, double max_diff, long adjust_digit);

 Array<uint8_t> Compress(double v);
  
 // bit （）
 int GetLastCompressedBits() const { return last_compressed_bits_; }

 private:
 int last_compressed_bits_ = 0; // bit 
 const double kMaxDiff;
 const long kAdjustDigit;
 const int kWindowSize;

 uint64_t stored_val_ = Double::DoubleToLongBits(2);
 std::unique_ptr<OutputBitStream> output_buffer_;

 long compressed_size_this_window_ = 0;
 int number_of_values_this_window_ = 0;
 double compression_ratio_last_window_ = 0;

 Array<int> leading_representation_ = {
 0, 0, 0, 0, 0, 0, 0, 0,
 1, 1, 1, 1, 2, 2, 2, 2,
 3, 3, 4, 4, 5, 5, 6, 6,
 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7
 };
 Array<int> leading_round_ = {
 0, 0, 0, 0, 0, 0, 0, 0,
 8, 8, 8, 8, 12, 12, 12, 12,
 16, 16, 18, 18, 20, 20, 22, 22,
 24, 24, 24, 24, 24, 24, 24, 24,
 24, 24, 24, 24, 24, 24, 24, 24,
 24, 24, 24, 24, 24, 24, 24, 24,
 24, 24, 24, 24, 24, 24, 24, 24,
 24, 24, 24, 24, 24, 24, 24, 24
 };
 Array<int> trailing_representation_ = {
 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 1, 1, 2, 2, 2, 2,
 3, 3, 3, 3, 4, 4, 4, 4,
 5, 5, 6, 6, 6, 6, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7,
 };
 Array<int> trailing_round_ = {
 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 22, 22,
 22, 22, 22, 22, 28, 28, 28, 28,
 32, 32, 32, 32, 36, 36, 36, 36,
 40, 40, 42, 42, 42, 42, 46, 46,
 46, 46, 46, 46, 46, 46, 46, 46,
 46, 46, 46, 46, 46, 46, 46, 46,
 };
 int leading_bits_per_value_ = 3;
 int trailing_bits_per_value_ = 3;
 Array<int> lead_distribution_ = Array<int>(64);
 Array<int> trail_distribution_ = Array<int>(64);
 int stored_leading_zeros_ = std::numeric_limits<int>::max();
 int stored_trailing_zeros_ = std::numeric_limits<int>::max();

 Array<uint8_t> AddValue(uint64_t value);

 int CompressValue(uint64_t value);

 int UpdatePositionsIfNeeded();
};

#endif // NET_SERF_XOR_COMPRESSOR_H
