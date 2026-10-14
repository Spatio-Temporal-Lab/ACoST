#ifndef SERF_ELIAS_DELTA_CODEC_H_
#define SERF_ELIAS_DELTA_CODEC_H_

#include <cmath>
#include "utils/output_bit_stream.h"
#include "utils/input_bit_stream.h"

/**
 * Elias Delta编码
 * 相比Elias Gamma，对大数更高效
 * 编码格式：
 *   1. N' = floor(log2(N))
 *   2. 用Elias Gamma编码N'+1
 *   3. 写入N的后N'位（不包括最高位1）
 */
class EliasDeltaCodec {
public:
    /**
     * 编码一个正整数
     * @param number 要编码的数（必须 >= 1）
     * @param output_bit_stream_ptr 输出比特流
     * @return 使用的比特数
     */
    static int Encode(int64_t number, OutputBitStream *output_bit_stream_ptr);

    /**
     * 解码一个整数
     * @param input_bit_stream_ptr 输入比特流
     * @return 解码后的数
     */
    static int64_t Decode(InputBitStream *input_bit_stream_ptr);
};

#endif  // SERF_ELIAS_DELTA_CODEC_H_


