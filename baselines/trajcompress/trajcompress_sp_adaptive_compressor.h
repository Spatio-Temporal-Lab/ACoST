#pragma once

#include "utils/output_bit_stream.h"
#include "utils/array.h"
#include <vector>
#include <deque>
#include <memory>

/**
 * TrajCompress-SP-Adaptive: Trajectory Compression with Adaptive Switched Predictors
 * 自适应多预测器轨迹压缩算法
 * 
 * 核心创新：
 * 1. 自适应模式切换（Multi-Predictor模式 <-> LDR-Only模式）
 * 2. 基于成本的预测器选择（考虑标志位成本 + 误差编码成本）
 * 3. 逃逸码机制（使用111标记模式切换）
 * 4. 动态霍夫曼编码优化
 */
class TrajCompressSPAdaptiveCompressor {
public:
    // GPS点结构
    struct GpsPoint {
        double longitude;
        double latitude;
        
        GpsPoint() : longitude(0), latitude(0) {}
        GpsPoint(double lon, double lat) : longitude(lon), latitude(lat) {}
        
        GpsPoint operator+(const GpsPoint& other) const {
            return GpsPoint(longitude + other.longitude, latitude + other.latitude);
        }
        
        GpsPoint operator-(const GpsPoint& other) const {
            return GpsPoint(longitude - other.longitude, latitude - other.latitude);
        }
        
        GpsPoint operator*(double scale) const {
            return GpsPoint(longitude * scale, latitude * scale);
        }
    };
    
    // 预测器类型枚举
    enum PredictorType {
        PREDICTOR_LDR = 0,    // Linear Dead Reckoning - 线性航位推算
        PREDICTOR_CP = 1,     // Curve Predictor - 曲线预测（二阶）
        PREDICTOR_ZP = 2      // Zero Predictor - 零预测（Serf-QT风格）
    };
    
    // 压缩模式枚举
    enum CompressionMode {
        MODE_MULTI_PREDICTOR = 0,  // 多预测器模式（默认）
        MODE_LDR_ONLY = 1          // 纯线性预测模式
    };
    
    // 历史状态结构
    struct HistoryState {
        GpsPoint reconstructed_point;
        GpsPoint velocity;  // 速度向量（经纬度差）
        
        HistoryState() {}
        HistoryState(const GpsPoint& point, const GpsPoint& vel)
            : reconstructed_point(point), velocity(vel) {}
    };
    
    // 统计信息结构
    struct CompressionStats {
        int total_points = 0;
        
        // 预测器使用统计
        int ldr_count = 0;
        int cp_count = 0;
        int zp_count = 0;
        
        // 模式切换统计
        int mode_switch_count = 0;
        int ldr_only_mode_points = 0;
        int multi_predictor_mode_points = 0;
        
        // 编码成本统计
        int total_bits = 0;
        int predictor_flag_bits = 0;
        int mode_switch_bits = 0;
        int quantized_data_bits = 0;
        
        // 预测误差统计
        double total_prediction_error = 0;
        double max_prediction_error = 0;
        std::vector<double> prediction_errors;
        
        void PrintStats() const;
        void PrintDetailedStats() const;
    };

    /**
     * 构造函数（支持参数调优）
     * @param block_size 块大小（用于预分配缓冲区）
     * @param epsilon 最大允许误差（度），内部会乘以0.999系数，与Serf-QT保持一致
     * @param enable_adaptive 是否启用动态参数调整（默认true）
     * @param min_window 最小窗口大小（默认32）
     * @param max_window 最大窗口大小（默认128）
     * @param observe_window 观察窗口大小（默认256）
     */
    TrajCompressSPAdaptiveCompressor(int block_size, double epsilon,
                                    bool enable_adaptive = true,
                                    int min_window = 32,
                                    int max_window = 128,
                                    int observe_window = 256);
    
    /**
     * 添加GPS点进行压缩
     * @param point GPS点
     */
    void AddGpsPoint(const GpsPoint& point);
    
    /**
     * 结束压缩，刷新缓冲区
     */
    void Close();
    
    /**
     * 获取压缩后的数据
     */
    Array<uint8_t> GetCompressedData();
    
    /**
     * 获取压缩大小（比特）
     */
    int GetCompressedSizeInBits() const { return compressed_size_in_bits_; }
    
    /**
     * 获取统计信息
     */
    const CompressionStats& GetStats() const { return stats_; }

private:
    // 参数
    const int kBlockSize;
    const double kEpsilon;          // 误差阈值（经过0.999系数处理）
    const double kQuantStep;        // 量化步长 = 2 * epsilon * 0.999
    
    // 注意：原始基于频率的模式切换参数已移除，现在使用基于成本的智能决策
    
    // 状态
    std::unique_ptr<OutputBitStream> output_bit_stream_;
    int compressed_size_in_bits_ = 0;
    bool first_point_ = true;
    
    // 当前压缩模式
    CompressionMode current_mode_ = MODE_MULTI_PREDICTOR;
    
    // 注意：原始基于频率的模式切换控制已移除，现在使用基于成本的智能决策
    
    // 预测器选择
    PredictorType last_used_predictor_ = PREDICTOR_ZP;
    
    // 重构状态（与解压器保持同步）
    GpsPoint current_reconstructed_point_;
    std::vector<HistoryState> history_states_;
    static constexpr int kMaxHistorySize = 3;
    
    // 统计信息
    CompressionStats stats_;
    
    // Huffman 编码相关（动态自适应）
    static constexpr int kSlidingWindowSize = 1000;
    std::vector<PredictorType> predictor_window_;
    int predictor_frequency_[3] = {0, 0, 0};  // [LDR, CP, ZP]
    
    struct HuffmanCode {
        std::vector<bool> bits;
        int length;
        HuffmanCode() : length(0) {}
        HuffmanCode(const std::vector<bool>& b) : bits(b), length(b.size()) {}
    };
    HuffmanCode huffman_codes_[3];
    
    // 基于成本的模式切换系统（可配置参数）
    int kCostWindowSize;                    // 成本评估窗口大小（动态调整：32-128）
    const int kSwitchCost;                  // 模式切换成本（固定1 bit，对齐Simple版本）
    int kStabilityMargin;                   // 防抖动边际（动态调整：基于成本差标准差）
    const int kEvaluationInterval;          // 评估间隔（默认16）
    const bool kClearWindowAfterSwitch;     // 切换后是否清空成本窗口（默认true）
    
    // 动态参数调整系统
    const bool kEnableAdaptive;             // 是否启用动态参数调整
    const int kMinWindowSize;               // 最小窗口（默认32）
    const int kMaxWindowSize;               // 最大窗口（默认128）
    const int kObserveWindowSize;           // 观察窗口大小（用于计算流失率，默认256）
    std::deque<PredictorType> predictor_history_;  // 预测器选择历史（用于计算流失率）
    std::deque<int> cost_diffs_;            // 逐点成本差（multi - ldr_only）
    int points_since_last_param_update_ = 0; // 距离上次参数更新的点数
    
    std::deque<int> point_costs_multi_;          // 多预测器模型窗口成本
    std::deque<int> point_costs_ldr_only_;       // LDR-Only模型窗口成本
    long long window_total_cost_multi_ = 0;      // 多预测器模型总成本
    long long window_total_cost_ldr_only_ = 0;   // LDR-Only模型总成本
    
    // 核心算法函数
    void ProcessFirstPoint(const GpsPoint& point);
    
    // 并行预测
    void ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp);
    
    // 基于成本的预测器选择（策略二）
    PredictorType SelectBestPredictorByCost(const GpsPoint& current_point,
                                            const GpsPoint& pred_ldr,
                                            const GpsPoint& pred_cp,
                                            const GpsPoint& pred_zp,
                                            GpsPoint& best_prediction,
                                            int& best_cost);
    
    // 多预测器模式编码
    void EncodeMultiPredictor(const GpsPoint& point);
    
    // LDR-Only模式编码
    void EncodeLDROnly(const GpsPoint& point);
    
    // 编码预测结果
    void EncodePrediction(PredictorType predictor,
                         const GpsPoint& current_point,
                         const GpsPoint& predicted_point);
    
    // 模式切换控制（基于成本的智能决策，无魔法数字）
    void EvaluateAndSwitchModeBasedOnCost();
    void EncodeModeSwitch(CompressionMode new_mode);
    
    // 估算编码成本
    int EstimateEliasGammaBits(int64_t value) const;
    int EstimateErrorEncodingCost(const GpsPoint& error) const;
    
    // 成本窗口管理
    void UpdateCostWindows(int multi_cost, int ldr_only_cost);
    
    // 动态参数调整（基于轨迹可预测性）
    void UpdateAdaptiveParameters();
    double CalculateChurnRate() const;
    double CalculateCostDiffStdDev() const;
    
    // Huffman 编码相关
    void UpdateHuffmanCodes();
    void EncodeWithHuffman(PredictorType predictor);
    void AddPredictorToWindow(PredictorType predictor);
    int GetHuffmanBitCost(PredictorType predictor) const;
    
    // 历史管理
    void UpdateHistory(const GpsPoint& reconstructed_point);
    void UpdateReconstructedState(const GpsPoint& reconstructed_point);
    
    // 辅助函数
    double CalculateDistance(const GpsPoint& p1, const GpsPoint& p2) const;
};

/**
 * TrajCompress-SP-Adaptive 解压缩器
 */
class TrajCompressSPAdaptiveDecompressor {
public:
    using GpsPoint = TrajCompressSPAdaptiveCompressor::GpsPoint;
    using PredictorType = TrajCompressSPAdaptiveCompressor::PredictorType;
    using CompressionMode = TrajCompressSPAdaptiveCompressor::CompressionMode;
    using HistoryState = TrajCompressSPAdaptiveCompressor::HistoryState;
    
    TrajCompressSPAdaptiveDecompressor(uint8_t* compressed_data, int data_size);
    
    bool ReadNextPoint(GpsPoint& point);
    std::vector<GpsPoint> ReadAllPoints();

private:
    std::unique_ptr<class InputBitStream> input_bit_stream_;
    
    // 参数
    int block_size_;
    double epsilon_;
    double quant_step_;
    int evaluation_interval_;  // 评估间隔（用于同步模式标志）
    
    // 状态
    bool first_point_ = true;
    int points_read_ = 0;  // 已读取的点数（用于同步评估窗口）
    CompressionMode current_mode_ = CompressionMode::MODE_MULTI_PREDICTOR;
    PredictorType last_used_predictor_ = PredictorType::PREDICTOR_ZP;
    
    // 重构状态
    GpsPoint current_reconstructed_point_;
    std::vector<HistoryState> history_states_;
    static constexpr int kMaxHistorySize = 3;
    
    // Huffman 解码相关
    static constexpr int kSlidingWindowSize = 1000;
    std::vector<PredictorType> predictor_window_;
    int predictor_frequency_[3] = {0, 0, 0};
    PredictorType huffman_decoder_map_[3];
    
    // 辅助函数
    void ReadHeader();
    void ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp);
    void UpdateHistory(const GpsPoint& reconstructed_point);
    void UpdateHuffmanDecoder();
    PredictorType DecodeWithHuffman();
    void AddPredictorToWindow(PredictorType predictor);
    bool CheckForModeSwitch();  // 检查是否有模式切换标志
};

