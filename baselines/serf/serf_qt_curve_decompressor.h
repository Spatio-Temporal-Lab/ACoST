#ifndef SERF_QT_CURVE_DECOMPRESSOR_H
#define SERF_QT_CURVE_DECOMPRESSOR_H

#include <memory>
#include <vector>

#include "utils/input_bit_stream.h"
#include "utils/array.h"

class SerfQtCurveDecompressor {
 public:
 SerfQtCurveDecompressor() = default;
 std::vector<double> Decompress(const Array<uint8_t> &bs);

 private:
 struct HistoryState {
 double value;
 double velocity; // （/）
 uint64_t timestamp;
    
 HistoryState(double v, double vel, uint64_t ts) 
 : value(v), velocity(vel), timestamp(ts) {}
 };
  
 static constexpr size_t kMaxHistorySize = 5;
  
 int block_size_ = 0;
 double max_diff_ = 0;
 std::unique_ptr<InputBitStream> input_bit_stream_ = std::make_unique<InputBitStream>();
 std::vector<HistoryState> history_states_;
 bool first_ = true;

 double NextValue();
 double CurvePredict(uint64_t current_timestamp) const;
};

#endif //SERF_QT_CURVE_DECOMPRESSOR_H

