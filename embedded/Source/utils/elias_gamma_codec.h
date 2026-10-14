#ifndef SERF_ELIAS_GAMMA_CODEC_H_
#define SERF_ELIAS_GAMMA_CODEC_H_

#include <cmath>

#include "utils/output_bit_stream.h"
#include "utils/input_bit_stream.h"
#include "utils/double.h"

class EliasGammaCodec {
 public:
  static int Encode(int64_t number, OutputBitStream *output_bit_stream_ptr);

  static int64_t Decode(InputBitStream *input_bit_stream_ptr);

 private:
  constexpr static double kLog2Table[17] = {
      Double::kNan,
      0,
      1,
      1.584962500721156,
      2,
      2.321928094887362,
      2.584962500721156,
      2.807354922057604,
      3,
      3.169925001442312,
      3.321928094887362,
      3.459431618637297,
      3.584962500721156,
      3.700439718141092,
      3.807354922057604,
      3.906890595608519,
      4
  };
};

#endif //SERF_ELIAS_GAMMA_CODEC_H_
