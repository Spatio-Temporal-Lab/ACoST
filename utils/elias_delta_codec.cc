#include "elias_delta_codec.h"
#include <cmath>

int EliasDeltaCodec::Encode(int64_t number, OutputBitStream *output_bit_stream_ptr) {
    if (number <= 0) {
        // Elias Delta无法编码0或负数，需要调用者先处理
        return 0;
    }
    
    int compressed_size_in_bits = 0;
    
    if (number == 1) {
        // 特殊情况：1 编码为单个1 bit
        // L=0, Gamma(L+1)=Gamma(1)=单个1, 后续0位
        compressed_size_in_bits += output_bit_stream_ptr->WriteBit(1);
        return compressed_size_in_bits;
    }
    
    // 计算 L = floor(log2(number))
    int L = static_cast<int>(floor(log2(number)));
    
    // 步骤1：用Elias Gamma编码 L+1
    int len_of_L = L + 1;
    int n;
    if (len_of_L <= 16) {
        // 使用预计算的log2表
        static const double kLog2Table[17] = {
            0,  // 占位
            0, 1, 1.584962500721156, 2, 2.321928094887362,
            2.584962500721156, 2.807354922057604, 3,
            3.169925001442312, 3.321928094887362, 3.459431618637297,
            3.584962500721156, 3.700439718141092, 3.807354922057604,
            3.906890595608519, 4
        };
        n = static_cast<int>(floor(kLog2Table[len_of_L]));
    } else {
        n = static_cast<int>(floor(log2(len_of_L)));
    }
    
    // Gamma编码 L+1：写n个0，然后写(L+1)的二进制（n+1位）
    compressed_size_in_bits += output_bit_stream_ptr->WriteInt(0, n);
    compressed_size_in_bits += output_bit_stream_ptr->WriteLong(len_of_L, n + 1);
    
    // 步骤2：写入number的后L位（去掉最高位的1）
    if (L > 0) {
        uint64_t remainder = number & ((1ULL << L) - 1);  // 取后L位
        compressed_size_in_bits += output_bit_stream_ptr->WriteLong(remainder, L);
    }
    
    return compressed_size_in_bits;
}

int64_t EliasDeltaCodec::Decode(InputBitStream *input_bit_stream_ptr) {
    // 步骤1：用Elias Gamma解码得到L+1
    
    // 读取前导0的个数
    int n = 0;
    while (!input_bit_stream_ptr->ReadBit()) {
        n++;
    }
    
    // 读取L+1的值
    int64_t len_of_L;
    if (n == 0) {
        len_of_L = 1;
    } else {
        len_of_L = (1LL << n) | input_bit_stream_ptr->ReadLong(n);
    }
    
    int L = static_cast<int>(len_of_L - 1);
    
    if (L == 0) {
        return 1;  // 特殊情况
    }
    
    // 步骤2：读取后续L位
    uint64_t remainder = input_bit_stream_ptr->ReadLong(L);
    
    // 步骤3：重构number = (1 << L) | remainder
    return (1LL << L) | remainder;
}


