#ifndef SERF_XOR_DECOMPRESSOR_H_
#define SERF_XOR_DECOMPRESSOR_H_

/*
 * Give hints to the compiler for branch prediction optimization.
 */
#if defined(__clang__) || (defined(__GNUC__) && (__GNUC__ > 2))
#define SERF_LIKELY(c) (__builtin_expect(!!(c), 1))
#define SERF_UNLIKELY(c) (__builtin_expect(!!(c), 0))
#else
#define SERF_LIKELY(c) (c)
#define SERF_UNLIKELY(c) (c)
#endif

#include <cstdint>
#include <memory>
#include <vector>

#include "utils/post_office_solver.h"
#include "utils/input_bit_stream.h"
#include "utils/double.h"
#include "utils/array.h"
#include "utils/serf_utils_64.h"

class SerfXORDecompressor {
 public:
  explicit SerfXORDecompressor(long adjust_digit) : adjust_digit_(adjust_digit) {};

  std::vector<double> Decompress(const Array<uint8_t> &bs);

 private:
  uint64_t stored_val_ = Double::DoubleToLongBits(2);
  int stored_leading_zeros_ = std::numeric_limits<int>::max();
  int stored_trailing_zeros_ = std::numeric_limits<int>::max();
  InputBitStream input_bit_stream_;
  Array<int> leading_representation_ = {0, 8, 12, 16, 18, 20, 22, 24};
  Array<int> trailing_representation_ = {0, 22, 28, 32, 36, 40, 42, 46};
  int leading_bits_per_value_ = 3;
  int trailing_bits_per_value_ = 3;
  long adjust_digit_;

  int branch_less_table[32] = {32, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                               11, 12, 13, 14, 15, 16, 17, 18, 19,
                               20, 21, 22, 23, 24, 25, 26, 27, 28,
                               29, 30, 31};

  uint64_t ReadValue();
  void UpdatePositionsIfNeeded();
  void UpdateLeadingRepresentation();
  void UpdateTrailingRepresentation();
};

#endif // SERF_XOR_DECOMPRESSOR_H_
