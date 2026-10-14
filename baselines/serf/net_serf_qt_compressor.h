#ifndef NET_SERF_QT_COMPRESSOR_H
#define NET_SERF_QT_COMPRESSOR_H

#include <memory>
#include <cstdint>
#include <cmath>

#include "utils/array.h"
#include "utils/output_bit_stream.h"
#include "utils/double.h"
#include "utils/elias_gamma_codec.h"
#include "utils/zig_zag_codec.h"

class NetSerfQtCompressor {
 public:
 explicit NetSerfQtCompressor(double error_bound);

 Array<uint8_t> Compress(double v);
  
 // bit （）
 int GetLastCompressedBits() const { return last_compressed_bits_; }

 private:
 const double kMaxDiff;
 double pre_value_ = 2;
 std::unique_ptr<OutputBitStream> output_bit_stream_ = std::make_unique<OutputBitStream>(5 * 8);
 int last_compressed_bits_ = 0; // bit 
};

#endif // NET_SERF_QT_COMPRESSOR_H
