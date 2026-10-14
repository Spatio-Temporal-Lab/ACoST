#pragma once

#include "utils/output_bit_stream.h"
#include "utils/input_bit_stream.h"
#include "utils/array.h"
#include <vector>
#include <memory>
#include <cstdint>

/** ACoST trajectory compressor and decompressor. */

enum class ACoSTMotionModel : uint8_t { STOP, CV, CTRA };

struct ACoSTConfig {
    // 鈹€鈹€ Extended CTRA range 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    double dt_threshold = 2000.0;   // keep CTRA mode up to 200 real seconds

    // 鈹€鈹€ Velocity averaging 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    int    speed_tail_segments  = 3;   // base tail (adaptively reduced)
    int    accel_tail_segments  = 2;   // base accel tail (adaptively reduced)

    // 鈹€鈹€ STOP detection 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // Only the 1-cell displacement test is used.
    // No speed threshold: avoids misfiring on pedestrian/slow-movement data.

    // 鈹€鈹€ Turn-aware vector CA damping 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // ctra_omega_eps is retained for API/backward compatibility.  The hot
    // predictor no longer computes angular velocity with atan2/sin/cos.
    double ctra_omega_eps       = 1.0e-8;
    double ctra_damp_onset      = 100.0;  // 10 real seconds in 0.1 s time units
    double ctra_damp_tau        = 200.0;  // 20 real seconds in 0.1 s time units

    // 鈹€鈹€ Vector turn detection threshold 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // Turn detection uses |cross/dot| > tan(theta_min), implemented as
    // cross虏 > tan虏(theta_min) * dot虏.  The default is tan(5掳), stored as a
    // constant to avoid calling tan() at runtime.  Increase this value to make
    // the predictor less sensitive to small GPS jitter.
    double ctra_turn_tan_min    = 0.08748866352592401;

    // 鈹€鈹€ ZP (Zero Prediction) selection 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // When history avg_dt > this onset, write a 1-bit flag per non-STOP point
    // and pick the cheaper predictor (ZP vs motion) by comparing Elias-Gamma
    // residual bit costs.  Set equal to ctra_damp_onset by default so the
    // same 10 s boundary triggers both CTRA damping and ZP consideration.
    double zp_selection_onset   = 100.0; // 10 real seconds; step_dt (not historyAvgDt) avoids 1s false triggers

    // 鈹€鈹€ Displacement cap 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    double max_speed_ms         = 200.0; // 200 m/s hard cap on predictions
    double max_cap_dt_mult      = 200.0; // max_disp = min(max_speed*dt, cap)
    double max_cap_abs_m        = 50000.0;
};

class ACoSTCompressor {
public:
    struct GpsPoint {
        double longitude, latitude;
        uint64_t timestamp;
        GpsPoint() : longitude(0), latitude(0), timestamp(0) {}
        GpsPoint(double lon, double lat, uint64_t ts = 0)
            : longitude(lon), latitude(lat), timestamp(ts) {}
    };

    struct HistoryState {
        double   q_lon, q_lat;  // reconstructed coordinates in quantization units
        uint64_t timestamp;
        HistoryState() : q_lon(0), q_lat(0), timestamp(0) {}
        HistoryState(double ql, double qa, uint64_t ts)
            : q_lon(ql), q_lat(qa), timestamp(ts) {}
    };

    struct CompressionStats {
        int    total_points           = 0;
        int    total_bits             = 0;
        int    quantized_data_bits    = 0;
        int    timestamp_bits         = 0;
        int    stop_count             = 0;
        int    cv_count               = 0;
        int    ctra_count             = 0;
        double total_prediction_error = 0.0;
        double max_prediction_error   = 0.0;
        std::vector<double> prediction_errors;
    };

    struct PredictionResult {
        double          q_lon, q_lat;
        ACoSTMotionModel model;
    };

    ACoSTCompressor(int block_size, double epsilon,
                     const ACoSTConfig& config = ACoSTConfig());

    void            AddGpsPoint(const GpsPoint& point);
    void            Close();
    Array<uint8_t>  GetCompressedData() const;

    int                    get_compressed_size_in_bits() const { return compressed_size_in_bits_; }
    const CompressionStats& GetStats() const { return stats_; }

private:
    static constexpr int kMaxHistorySize = 8;

    const int    kBlockSize;
    const double kEpsilon;
    const double kQuantStep;
    ACoSTConfig config_;

    std::unique_ptr<OutputBitStream> output_bit_stream_;
    int     compressed_size_in_bits_ = 0;
    bool    first_point_ = true;

    double   current_q_lon_ = 0.0;
    double   current_q_lat_ = 0.0;
    uint64_t current_ts_    = 0;
    double   lon_scale_     = 1.0;

    std::vector<HistoryState> history_states_;
    ACoSTMotionModel         prev_model_ = ACoSTMotionModel::CV;
    CompressionStats          stats_;

    void ProcessFirstPoint(const GpsPoint& point);
    PredictionResult Predict(uint64_t target_ts) const;
    int  EncodeQuantizedResidual(int64_t ql, int64_t qa);
    int  EncodeValue(uint64_t value);
    void UpdateHistory(double q_lon, double q_lat, uint64_t ts);
    double CalculateDistance(const GpsPoint& p1, const GpsPoint& p2) const;
};


class ACoSTDecompressor {
public:
    using GpsPoint     = ACoSTCompressor::GpsPoint;
    using HistoryState = ACoSTCompressor::HistoryState;

    ACoSTDecompressor(uint8_t* data, int size,
                       const ACoSTConfig& config = ACoSTConfig());

    bool              ReadNextPoint(GpsPoint& point);
    std::vector<GpsPoint> ReadAllPoints();

private:
    static constexpr int kMaxHistorySize = 8;

    std::unique_ptr<InputBitStream> input_bit_stream_;
    int    block_size_  = 0;
    double epsilon_     = 0.0;
    double quant_step_  = 0.0;

    bool     first_point_ = true;
    int      points_read_ = 0;

    double   current_q_lon_ = 0.0;
    double   current_q_lat_ = 0.0;
    uint64_t current_ts_    = 0;
    double   lon_scale_     = 1.0;

    std::vector<HistoryState> history_states_;
    ACoSTMotionModel         prev_model_ = ACoSTMotionModel::CV;
    ACoSTConfig              config_;

    void ReadHeader();
    ACoSTCompressor::PredictionResult Predict(uint64_t target_ts) const;
    void DecodeQuantizedResidual(int64_t& ql, int64_t& qa);
    int64_t DecodeValue();
    void UpdateHistory(double q_lon, double q_lat, uint64_t ts);
};
