#include "elias_gamma_codec.h"

int EliasGammaCodec::Encode(int64_t number, OutputBitStream *output_bit_stream_ptr) {
  int compressed_size_in_bits = 0;
  int n;
  if (number <= 16) {
    n = std::floor(kLog2Table[number]);
  } else {
    n = std::floor(std::log2(number));
  }
  // 写入n个0比特
  compressed_size_in_bits += output_bit_stream_ptr->WriteInt(0, n);
  // 写入number的二进制表示（n+1位）- 使用WriteLong以支持大于32位的数
  compressed_size_in_bits += output_bit_stream_ptr->WriteLong(number, n + 1);
  return compressed_size_in_bits;
}

int64_t EliasGammaCodec::Decode(InputBitStream *input_bit_stream_ptr) {
  int n = 0;
  while (!input_bit_stream_ptr->ReadBit()) n++;
  // 使用ReadLong以支持大于32位的数
  return n == 0 ? 1 :  (1LL << n) | input_bit_stream_ptr->ReadLong(n);
}
