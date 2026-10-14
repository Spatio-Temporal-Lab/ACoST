#ifndef SERF_QT_CURVE_COMPRESSOR_H
#define SERF_QT_CURVE_COMPRESSOR_H

#include <cstdint>
#include <memory>

#include "utils/output_bit_stream.h"
#include "utils/array.h"

/*
 * Serf-QT with Curve Prediction (Quadratic Prediction)
 * ：predicted = 3*v1 - 3*v2 + v3
 * +------------+-----------------+---------------+
 * |16bits - len|64bits - max_diff|Encoded Content|
 * +------------+-----------------+---------------+
 */

class SerfQtCurveCompressor {
 public:
 SerfQtCurveCompressor(int block_size, double max_diff);

 void AddValue(double v, uint64_t timestamp = 0); // 

 Array<uint8_t> compressed_bytes();

 void Close();

 long get_compressed_size_in_bits() const;

 private:
 const double kMaxDiff;
 const int kBlockSize;
 bool first_ = true;
 std::unique_ptr<OutputBitStream> output_bit_stream_;
 Array<uint8_t> compressed_bytes_;
  
 // 、（TrajCompress-SPCP）
 struct HistoryState {
 double value;
 double velocity;
 uint64_t timestamp;
 HistoryState() : value(2.0), velocity(0.0), timestamp(0) {}
 HistoryState(double v, double vel, uint64_t ts) : value(v), velocity(vel), timestamp(ts) {}
 };
 std::vector<HistoryState> history_states_;
 static constexpr int kMaxHistorySize = 5; // 5
  
 long compressed_size_in_bits_ = 0;
 long stored_compressed_size_in_bits_ = 0;
  
 double CurvePredict(uint64_t current_timestamp) const;
};

#endif // SERF_QT_CURVE_COMPRESSOR_H

