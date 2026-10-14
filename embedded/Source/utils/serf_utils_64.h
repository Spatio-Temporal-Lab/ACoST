#ifndef SERF_64_UTILS_H
#define SERF_64_UTILS_H

/*
 * Give hints to the compiler for branch prediction optimization.
 */
#if defined(__clang__) || (defined(__GNUC__) && (__GNUC__ > 2))
#define SERF_LIKELY(c) (__builtin_expect(!!(c), 1))
#define SERF_UNLIKELY(c) (__builtin_expect(!!(c), 0))
#else
#define FASTLZ_LIKELY(c) (c)
#define FASTLZ_UNLIKELY(c) (c)
#endif

#include <cstdint>

#include "utils/double.h"

class SerfUtils64 {
 public:
  static uint64_t FindAppLong(double min, double max, double v, uint64_t last_long, double max_diff,
                              double adjust_digit);

  static uint64_t FindAppLongNoPlus(double min, double max, double v, uint64_t last_long, double max_diff,
                                    double adjust_digit);

  static uint64_t FindAppLongNoFast(double min, double max, double v, uint64_t last_long, double max_diff,
                                    double adjust_digit);

 private:
  static constexpr uint64_t kBitWeight[64] = {
      1ULL, 2ULL, 4ULL, 8ULL, 16ULL, 32ULL, 64ULL, 128ULL,
      256ULL, 512ULL, 1024ULL, 2048ULL, 4096ULL, 8192ULL, 16384ULL,
      32768ULL,
      65536ULL, 131072ULL, 262144ULL, 524288ULL, 1048576ULL, 2097152ULL,
      4194304ULL, 8388608ULL,
      16777216ULL, 33554432ULL, 67108864ULL, 134217728ULL, 268435456ULL,
      536870912ULL, 1073741824ULL,
      2147483648ULL,
      4294967296ULL, 8589934592ULL, 17179869184ULL, 34359738368ULL,
      68719476736ULL, 137438953472ULL,
      274877906944ULL, 549755813888ULL,
      1099511627776ULL, 2199023255552ULL, 4398046511104ULL,
      8796093022208ULL, 17592186044416ULL,
      35184372088832ULL, 70368744177664ULL, 140737488355328ULL,
      281474976710656ULL, 562949953421312ULL, 1125899906842624ULL,
      2251799813685248ULL, 4503599627370496ULL,
      9007199254740992ULL, 18014398509481984ULL, 36028797018963968ULL,
      72057594037927936ULL, 144115188075855872ULL, 288230376151711744ULL,
      576460752303423488ULL,
      1152921504606846976ULL, 2305843009213693952ULL,
      4611686018427387904ULL, 9223372036854775808ULL
  };

  static uint64_t FindAppLong(double min_double, double max_double, uint64_t sign,
                              double original, uint64_t last_long, double max_diff,
                              double adjust_digit);

  static uint64_t FindAppLongNoPlus(double min_double, double max_double, uint64_t sign,
                                    double original, uint64_t last_long, double max_diff,
                                    double adjust_digit);

  static uint64_t FindAppLongNoFast(double min_double, double max_double, uint64_t sign,
                                    double original, uint64_t last_long, double max_diff,
                                    double adjust_digit);
};

#endif  // SERF_64_UTILS_H
