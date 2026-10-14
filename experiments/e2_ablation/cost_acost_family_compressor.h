#ifndef COST_ACOST_FAMILY_COMPRESSOR_H
#define COST_ACOST_FAMILY_COMPRESSOR_H

#include "utils/array.h"
#include "utils/input_bit_stream.h"
#include "utils/output_bit_stream.h"

#include <cstdint>
#include <memory>
#include <vector>

// CoST residual coder driven by one deterministic ACoST STOP/CV/CTRA
// motion predictor.  CoST LDR/CP/ZP candidate selection and mode switching
// are intentionally not part of this variant.
class CoSTFamilyCompressor {
public:
    struct GpsPoint {
        double longitude = 0.0;
        double latitude = 0.0;
        uint64_t timestamp = 0;
        GpsPoint() = default;
        GpsPoint(double lon, double lat, uint64_t ts = 0)
            : longitude(lon), latitude(lat), timestamp(ts) {}
    };

    struct CompressionStats {
        int total_points = 0;
        int total_bits = 0;
        int timestamp_bits = 0;
        int stop_count = 0;
        int cv_count = 0;
        int ctra_count = 0;
    };

    CoSTFamilyCompressor(int block_size, double epsilon,
                         int evaluation_window = 96,
                         bool use_time_window = false,
                         uint64_t time_window_seconds = 60);
    void AddGpsPoint(const GpsPoint& point);
    void Close();
    Array<uint8_t> GetCompressedData() const;
    int GetCompressedSizeInBits() const { return compressed_size_in_bits_; }
    const CompressionStats& GetStats() const { return stats_; }

private:
    struct HistoryState { int64_t q_lon; int64_t q_lat; uint64_t timestamp; };
    struct Prediction { int64_t q_lon; int64_t q_lat; int model; };
    static constexpr int kStop = 0;
    static constexpr int kCv = 1;
    static constexpr int kCtra = 2;
    static constexpr int kMaxHistorySize = 8;

    const int block_size_;
    const double epsilon_;
    const double quant_step_;
    const int evaluation_window_;
    const bool use_time_window_;
    const uint64_t time_window_seconds_;
    std::unique_ptr<OutputBitStream> output_;
    int compressed_size_in_bits_ = 0;
    bool first_point_ = true;
    uint64_t current_timestamp_ = 0;
    int64_t current_q_lon_ = 0;
    int64_t current_q_lat_ = 0;
    double lon_scale_ = 1.0;
    std::vector<HistoryState> history_;
    CompressionStats stats_;

    void WriteFirstPoint(const GpsPoint& point);
    Prediction Predict(uint64_t target_timestamp) const;
    void UpdateHistory(int64_t q_lon, int64_t q_lat, uint64_t timestamp);
};

class CoSTFamilyDecompressor {
public:
    using GpsPoint = CoSTFamilyCompressor::GpsPoint;
    CoSTFamilyDecompressor(uint8_t* data, int size);
    bool ReadNextPoint(GpsPoint& point);

private:
    struct HistoryState { int64_t q_lon; int64_t q_lat; uint64_t timestamp; };
    struct Prediction { int64_t q_lon; int64_t q_lat; int model; };
    static constexpr int kStop = 0;
    static constexpr int kCv = 1;
    static constexpr int kCtra = 2;
    static constexpr int kMaxHistorySize = 8;

    std::unique_ptr<InputBitStream> input_;
    double epsilon_ = 0.0;
    double quant_step_ = 0.0;
    int64_t current_q_lon_ = 0;
    int64_t current_q_lat_ = 0;
    uint64_t current_timestamp_ = 0;
    double lon_scale_ = 1.0;
    bool first_point_ = true;
    bool finished_ = false;
    std::vector<HistoryState> history_;

    void ReadHeader();
    Prediction Predict(uint64_t target_timestamp) const;
    void UpdateHistory(int64_t q_lon, int64_t q_lat, uint64_t timestamp);
};

#endif
