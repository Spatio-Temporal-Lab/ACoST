#pragma once

#include "utils/output_bit_stream.h"
#include "utils/input_bit_stream.h"
#include "utils/array.h"
#include <vector>
#include <deque>
#include <memory>

/**
 * CoST Compressor: Cost-aware Trajectory Compression
 * 
 * Key features:
 * 1. Cost-based predictor selection (LDR/CP/ZP)
 * 2. Intelligent mode switching (Multi-Predictor / LDR-Only)
 * 3. Adaptive Huffman coding for predictor flags
 * 4. Error-bounded compression with user-specified threshold
 */
class CoSTCompressor {
public:
 // GPS
    struct GpsPoint {
        double longitude;
        double latitude;
        uint64_t timestamp;  // Unix（）
        
        GpsPoint() : longitude(0), latitude(0), timestamp(0) {}
        GpsPoint(double lon, double lat, uint64_t ts = 0) 
            : longitude(lon), latitude(lat), timestamp(ts) {}
        
        GpsPoint operator+(const GpsPoint& other) const {
            return GpsPoint(longitude + other.longitude, latitude + other.latitude, timestamp);
        }
        
        GpsPoint operator-(const GpsPoint& other) const {
            return GpsPoint(longitude - other.longitude, latitude - other.latitude, 0);
        }
        
        GpsPoint operator*(double scale) const {
            return GpsPoint(longitude * scale, latitude * scale, timestamp);
        }
    };
    
    // (comment removed)
    enum PredictorType {
        PREDICTOR_LDR = 0,    // Linear Dead Reckoning - 
        PREDICTOR_CP = 1,     // Curve Predictor - （）
        PREDICTOR_ZP = 2      // Zero Predictor - （Serf-QT）
    };
    
    // (comment removed)
    enum CompressionMode {
        MODE_MULTI_PREDICTOR = 0,  // （）
        MODE_LDR_ONLY = 1          // 
    };
    
    // (comment removed)
    struct HistoryState {
        GpsPoint reconstructed_point;
        GpsPoint velocity;  // （）
        
        HistoryState() {}
        HistoryState(const GpsPoint& point, const GpsPoint& vel)
            : reconstructed_point(point), velocity(vel) {}
    };
    
    // (comment removed)
    struct CompressionStats {
        int total_points = 0;
        
        // (comment removed)
        int ldr_count = 0;
        int cp_count = 0;
        int zp_count = 0;
        
        // (comment removed)
        int mode_switch_count = 0;
        int ldr_only_mode_points = 0;
        int multi_predictor_mode_points = 0;
        
        // (comment removed)
        int total_bits = 0;
        int predictor_flag_bits = 0;
        int mode_switch_bits = 0;
        int quantized_data_bits = 0;
        int timestamp_bits = 0;  // timestamp（spatial）
        
        // (comment removed)
        double total_prediction_error = 0;
        double max_prediction_error = 0;
        std::vector<double> prediction_errors;
        
        void PrintStats() const;
        void PrintDetailedStats() const;
    };

    /**
     * 
     * @param block_size （）
     * @param epsilon （），0.999，Serf-QT
     * @param evaluation_window （，，96）
     * @param use_time_window （，false）
     * @param time_window_seconds （，use_time_window=true，60）
     */
    CoSTCompressor(int block_size, double epsilon, 
                                          int evaluation_window = 96,
                                          bool use_time_window = false,
                                          uint64_t time_window_seconds = 60);
    
    /**
     * GPS
     * @param point GPS
     */
    void AddGpsPoint(const GpsPoint& point);
    
    /**
     * ，
     */
    void Close();
    
    /**
     * 
     */
    Array<uint8_t> GetCompressedData();
    
    /**
     * （）
     */
    int GetCompressedSizeInBits() const { return compressed_size_in_bits_; }
    
    /**
     * 
     */
    const CompressionStats& GetStats() const { return stats_; }

private:
    // (comment removed)
    const int kBlockSize;
    const double kEpsilon;          // （0.999）
    const double kQuantStep;        //  = 2 * epsilon * 0.999
    
 // ：（）
    const int kEvaluationWindow;                        // （，use_time_window_=false）
    const bool use_time_window_;                        // 
    const uint64_t kTimeWindowSeconds;                  // （，use_time_window_=true）
    static constexpr bool kClearWindowAfterSwitch = false;  // 
    
    // (comment removed)
    std::unique_ptr<OutputBitStream> output_bit_stream_;
    int compressed_size_in_bits_ = 0;
    bool first_point_ = true;
    
    // (comment removed)
    CompressionMode current_mode_ = MODE_MULTI_PREDICTOR;
    
    // (comment removed)
    PredictorType last_used_predictor_ = PREDICTOR_ZP;
    
 // （）
    GpsPoint current_reconstructed_point_;
    std::vector<HistoryState> history_states_;
    static constexpr int kMaxHistorySize = 8;
    
    // (comment removed)
    CompressionStats stats_;
    
 // Huffman （）
    static constexpr int kSlidingWindowSize = 1000;
    std::deque<PredictorType> predictor_window_;
    int predictor_frequency_[3] = {0, 0, 0};  // [LDR, CP, ZP]
    
    struct HuffmanCode {
        std::vector<bool> bits;
        int length;
        HuffmanCode() : length(0) {}
        HuffmanCode(const std::vector<bool>& b) : bits(b), length(b.size()) {}
    };
    HuffmanCode huffman_codes_[3];
    
 // ：
    static constexpr int kSwitchCost = 4;   // （）
    
 // （）
    struct CostRecord {
        int cost;
        uint64_t timestamp;
        CostRecord(int c = 0, uint64_t ts = 0) : cost(c), timestamp(ts) {}
    };
    
 // （Ring Buffer） deque，
    template<typename T, int MaxSize>
    class RingBuffer {
    private:
        T buffer_[MaxSize];
        int head_ = 0;  // 
        int tail_ = 0;  // （）
        int size_ = 0;  // 
        
    public:
        void push_back(const T& value) {
            buffer_[tail_] = value;
            tail_ = (tail_ + 1) % MaxSize;
            if (size_ < MaxSize) {
                size_++;
            } else {
 // ，
                head_ = (head_ + 1) % MaxSize;
            }
        }
        
        void pop_front() {
            if (size_ > 0) {
                head_ = (head_ + 1) % MaxSize;
                size_--;
            }
        }
        
        const T& front() const {
            return buffer_[head_];
        }
        
        bool empty() const {
            return size_ == 0;
        }
        
        size_t size() const {
            return size_;
        }
        
        void clear() {
            head_ = 0;
            tail_ = 0;
            size_ = 0;
        }
    };
    
 // deque（ 256，）
    static constexpr int kMaxCostWindowSize = 256;
    RingBuffer<CostRecord, kMaxCostWindowSize> point_costs_multi_;       // 
    RingBuffer<CostRecord, kMaxCostWindowSize> point_costs_ldr_only_;    // LDR-Only
    long long window_total_cost_multi_ = 0;          // 
    long long window_total_cost_ldr_only_ = 0;       // LDR-Only
    
    // (comment removed)
    uint64_t last_evaluation_timestamp_ = 0;         // 
    
    // (comment removed)
    void ProcessFirstPoint(const GpsPoint& point);
    
    // (comment removed)
    void ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp);
    
 // （）
    PredictorType SelectBestPredictorByCost(const GpsPoint& current_point,
                                            const GpsPoint& pred_ldr,
                                            const GpsPoint& pred_cp,
                                            const GpsPoint& pred_zp,
                                            GpsPoint& best_prediction,
                                            int& best_cost);
    
    // (comment removed)
    void EncodeMultiPredictor(const GpsPoint& point);
    
 // LDR-Only
    void EncodeLDROnly(const GpsPoint& point);
    
    // (comment removed)
    void EncodePrediction(PredictorType predictor,
                         const GpsPoint& current_point,
                         const GpsPoint& predicted_point);
    
 // （，）
    void EvaluateAndSwitchModeBasedOnCost();
    void EncodeModeSwitch(CompressionMode new_mode);
    
    // (comment removed)
    int EstimateEliasGammaBits(int64_t value) const;
    int EstimateErrorEncodingCost(const GpsPoint& error) const;
    
    // (comment removed)
    void UpdateCostWindows(int multi_cost, int ldr_only_cost, uint64_t timestamp);
    
 // Huffman 
    void UpdateHuffmanCodes();
    void EncodeWithHuffman(PredictorType predictor);
    void AddPredictorToWindow(PredictorType predictor);
    int GetHuffmanBitCost(PredictorType predictor) const;
    
    // (comment removed)
    void UpdateHistory(const GpsPoint& reconstructed_point);
    void UpdateReconstructedState(const GpsPoint& reconstructed_point);
    
    // (comment removed)
    double CalculateDistance(const GpsPoint& p1, const GpsPoint& p2) const;
};

/**
 * CoST Decompressor
 */
class CoSTDecompressor {
public:
    using GpsPoint = CoSTCompressor::GpsPoint;
    using PredictorType = CoSTCompressor::PredictorType;
    using CompressionMode = CoSTCompressor::CompressionMode;
    using HistoryState = CoSTCompressor::HistoryState;
    
    CoSTDecompressor(uint8_t* compressed_data, int data_size);
    
    bool ReadNextPoint(GpsPoint& point);
    std::vector<GpsPoint> ReadAllPoints();

private:
    std::unique_ptr<InputBitStream> input_bit_stream_;
    
    // (comment removed)
    int block_size_;
    double epsilon_;
    double quant_step_;
    int evaluation_window_;  // （）
    bool use_time_window_;   // 
    uint64_t time_window_seconds_;  // （）
    
    // (comment removed)
    bool first_point_ = true;
    int points_read_ = 0;  // （）
    uint64_t last_evaluation_timestamp_ = 0;  // （）
    CompressionMode current_mode_ = CompressionMode::MODE_MULTI_PREDICTOR;
    PredictorType last_used_predictor_ = PredictorType::PREDICTOR_ZP;
    
    // (comment removed)
    GpsPoint current_reconstructed_point_;
    std::vector<HistoryState> history_states_;
    static constexpr int kMaxHistorySize = 8;
    
 // Huffman 
    static constexpr int kSlidingWindowSize = 1000;
    std::deque<PredictorType> predictor_window_;
    int predictor_frequency_[3] = {0, 0, 0};
    PredictorType huffman_decoder_map_[3];
    
    // (comment removed)
    void ReadHeader();
    void ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp);
    void UpdateHistory(const GpsPoint& reconstructed_point);
    void UpdateHuffmanDecoder();
    PredictorType DecodeWithHuffman();  // Huffman（）
    void AddPredictorToWindow(PredictorType predictor);
};

