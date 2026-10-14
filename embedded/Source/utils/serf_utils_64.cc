#include "utils/serf_utils_64.h"

uint64_t SerfUtils64::FindAppLong(double min, double max, double v, uint64_t last_long, double max_diff,
                                  double adjust_digit) {
  if (SERF_LIKELY(min >= 0)) {
    // both positive
    return FindAppLong(min, max, 0, v, last_long, max_diff, adjust_digit);
  } else if (max <= 0) {
    // both negative
    return FindAppLong(-max, -min, 0x8000000000000000ULL, v, last_long,
                       max_diff, adjust_digit);
  } else if (last_long >> 63 == 0) {
    // consider positive part only, to make more leading zeros
    return FindAppLong(0, max, 0, v, last_long, max_diff, adjust_digit);
  } else {
    // consider negative part only, to make more leading zeros
    return FindAppLong(0, -min, 0x8000000000000000ULL, v, last_long,
                       max_diff, adjust_digit);
  }
}

uint64_t SerfUtils64::FindAppLong(double min_double, double max_double, uint64_t sign, double original,
                                  uint64_t last_long, double max_diff, double adjust_digit) {
  // may be negative zero
  uint64_t min = Double::DoubleToLongBits(min_double) & 0x7fffffffffffffffULL;
  uint64_t max = Double::DoubleToLongBits(max_double);
  int leading_zeros = __builtin_clzll(min ^ max);
  int64_t front_mask = 0xffffffffffffffff << (64 - leading_zeros);
  int shift = 64 - leading_zeros;
  uint64_t result_long;
  double diff;
  uint64_t append;
  for (; shift >= 0; --shift) {
    // 计算 front 和 rear，无分支
    uint64_t front = front_mask & min;
    uint64_t rear = (~front_mask) & last_long;

    append = rear | front;

    // 条件判断优化为掩码操作
    bool condition1 = (append >= min && append <= max);
    result_long = condition1 ? (append ^ sign) : 0;

    // 无分支计算 diff，并验证是否满足 diff 条件
    diff = Double::LongBitsToDouble(result_long) - adjust_digit - original;
    bool diff_satisfied = (diff >= -max_diff && diff <= max_diff);

    // 如果满足条件，直接返回
    if (condition1 && diff_satisfied) {
      return result_long;
    }

    // 尝试避免溢出操作
    append = (append + kBitWeight[shift]) & 0x7fffffffffffffffULL;

    bool condition2 = append <= max;
    result_long = condition2 ? (append ^ sign) : 0;

    // 再次检查 diff
    diff = Double::LongBitsToDouble(result_long) - adjust_digit - original;
    diff_satisfied = (diff >= -max_diff && diff <= max_diff);

    if (condition2 && diff_satisfied) {
      return result_long;
    }

    // 更新前置掩码
    front_mask >>= 1;
  }

  // we do not find a satisfied value, so we return the original value
  return Double::DoubleToLongBits(original + adjust_digit);
}

uint64_t SerfUtils64::FindAppLongNoPlus(double min, double max, double v, uint64_t last_long, double max_diff,
                                        double adjust_digit) {
  if (min >= 0) {
    // both positive
    return FindAppLongNoPlus(min, max, 0, v, last_long, max_diff, adjust_digit);
  } else if (max <= 0) {
    // both negative
    return FindAppLongNoPlus(-max, -min, 0x8000000000000000ULL, v, last_long,
                             max_diff, adjust_digit);
  } else if (last_long >> 63 == 0) {
    // consider positive part only, to make more leading zeros
    return FindAppLongNoPlus(0, max, 0, v, last_long, max_diff, adjust_digit);
  } else {
    // consider negative part only, to make more leading zeros
    return FindAppLongNoPlus(0, -min, 0x8000000000000000ULL, v, last_long,
                             max_diff, adjust_digit);
  }
}

uint64_t SerfUtils64::FindAppLongNoPlus(double min_double, double max_double, uint64_t sign, double original,
                                        uint64_t last_long, double max_diff, double adjust_digit) {
  // may be negative zero
  uint64_t min = Double::DoubleToLongBits(min_double) & 0x7fffffffffffffffULL;
  uint64_t max = Double::DoubleToLongBits(max_double);
  int leading_zeros = __builtin_clzll(min ^ max);
  int64_t front_mask = 0xffffffffffffffff << (64 - leading_zeros);
  int shift = 64 - leading_zeros;
  uint64_t result_long;
  double diff;
  uint64_t append;
  while (shift >= 0) {
    uint64_t front = front_mask & min;
    uint64_t rear = (~front_mask) & last_long;

    append = rear | front;
    if (append >= min && append <= max) {
      result_long = append ^ sign;
      diff = Double::LongBitsToDouble(result_long) - adjust_digit - original;
      if (diff >= -max_diff && diff <= max_diff) {
        return result_long;
      }
    }

    front_mask = front_mask >> 1;

    --shift;
  }

  // we do not find a satisfied value, so we return the original value
  return Double::DoubleToLongBits(original + adjust_digit);
}

uint64_t SerfUtils64::FindAppLongNoFast(double min, double max, double v, uint64_t last_long, double max_diff,
                                        double adjust_digit) {
  if (min >= 0) {
    // both positive
    return FindAppLongNoFast(min, max, 0, v, last_long, max_diff, adjust_digit);
  } else if (max <= 0) {
    // both negative
    return FindAppLongNoFast(-max, -min, 0x8000000000000000ULL, v, last_long,
                             max_diff, adjust_digit);
  } else if (last_long >> 63 == 0) {
    // consider positive part only, to make more leading zeros
    return FindAppLongNoFast(0, max, 0, v, last_long, max_diff, adjust_digit);
  } else {
    // consider negative part only, to make more leading zeros
    return FindAppLongNoFast(0, -min, 0x8000000000000000ULL, v, last_long,
                             max_diff, adjust_digit);
  }
}

uint64_t SerfUtils64::FindAppLongNoFast(double min_double, double max_double, uint64_t sign, double original,
                                        uint64_t last_long, double max_diff, double adjust_digit) {
  uint64_t min = Double::DoubleToLongBits(min_double) & 0x7fffffffffffffffULL; // may be negative zero
  uint64_t max = Double::DoubleToLongBits(max_double);
  int64_t front_mask = 0xffffffffffffffff;
  uint64_t resultLong;
  double diff;
  uint64_t append;
  for (int i = 1; i <= 64; ++i) {
    uint64_t mask = front_mask << (64 - i);
    append = (last_long & ~mask) | (min & mask);

    if (min <= append && append <= max) {
      resultLong = append ^ sign;
      diff = Double::LongBitsToDouble(resultLong) - adjust_digit - original;
      if (diff >= -max_diff && diff <= max_diff) {
        return resultLong;
      }
    }

    // may be overflow
    append = (append + kBitWeight[64 - i]) & 0x7fffffffffffffffL;
    if (append <= max) {
      // append must be greater than min
      resultLong = append ^ sign;
      diff = Double::LongBitsToDouble(resultLong) - adjust_digit - original;
      if (diff >= -max_diff && diff <= max_diff) {
        return resultLong;
      }
    }
  }

  // we do not find a satisfied value, so we return the original value
  return Double::DoubleToLongBits(original + adjust_digit);
}
