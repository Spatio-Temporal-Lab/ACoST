#pragma once

#include "utils/output_bit_stream.h"
#include "utils/array.h"
#include "utils/double.h"
#include <vector>
#include <deque>
#include <memory>
#include <cmath>
#include <string>

/**
 * TrajCompress-SP: Trajectory Compression with Switched Predictors
 * 基于Serf-QT优化的GPS轨迹有损压缩算法
 * 
 * 核心创新：
 * 1. 多预测器系统（LDR线性预测、CP曲线预测、ZP零预测）
 * 2. 状态感知（静止/运动状态检测）
 * 3. 动态预测器切换
 * 4. 静止状态RLE编码优化
 */
class TrajCompressSPCompressor {
public:
    // GPS点结构
    struct GpsPoint {
        double longitude;
        double latitude;
        uint64_t timestamp;
        
        GpsPoint() : longitude(0), latitude(0), timestamp(0) {}
        GpsPoint(double lon, double lat, uint64_t ts = 0) : longitude(lon), latitude(lat), timestamp(ts) {}
        
        GpsPoint operator+(const GpsPoint& other) const {
            return GpsPoint(longitude + other.longitude, latitude + other.latitude, timestamp);
        }
        
        GpsPoint operator-(const GpsPoint& other) const {
            return GpsPoint(longitude - other.longitude, latitude - other.latitude, 0);
        }
        
        GpsPoint operator*(double scale) const {
            return GpsPoint(longitude * scale, latitude * scale, 0);
        }
    };
    
    // 预测器类型枚举
    enum PredictorType {
        PREDICTOR_LDR = 0,    // Linear Dead Reckoning - 线性航位推算
        PREDICTOR_CP = 1,     // Curve Predictor - 曲线预测（二阶）
        PREDICTOR_ZP = 2      // Zero Predictor - 零预测（Serf-QT风格）
    };
    
    // 预测器标志编码优化
    enum PredictorFlag {
        FLAG_REUSE = 0,       // 重用上一个预测器
        FLAG_SWITCH_LDR = 1,   // 切换到LDR预测器
        FLAG_SWITCH_CP = 2,    // 切换到CP预测器
        FLAG_SWITCH_ZP = 3     // 切换到ZP预测器
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
        
        // 预测器重用统计
        int predictor_reuse_count = 0;      // 连续使用同一预测器的次数
        int predictor_switch_count = 0;     // 切换预测器的次数
        
        // 编码成本统计
        int total_bits = 0;
        int predictor_flag_bits = 0;
        int quantization_bits = 0;
        int quantized_data_bits = 0;
        int timestamp_bits = 0;  // timestamp编码比特数（不计入spatial压缩比）
        
        // 预测误差统计
        double total_prediction_error = 0;
        double max_prediction_error = 0;
        std::vector<double> prediction_errors;
        
        void PrintStats() const;
        void PrintDetailedStats() const;
    };

    /**
     * 构造函数
     * @param block_size 块大小（用于预分配缓冲区）
     * @param epsilon 最大允许误差（度），内部会乘以0.999系数，与Serf-QT保持一致
     */
    TrajCompressSPCompressor(int block_size, double epsilon);
    
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
    const double kEpsilon;          // 误差阈值（经过0.999系数处理，与Serf-QT保持一致）
    const double kQuantStep;        // 量化步长 = 2 * epsilon * 0.999（与Serf-QT一维量化完全对应）
    
    // 状态
    std::unique_ptr<OutputBitStream> output_bit_stream_;
    int compressed_size_in_bits_ = 0;
    bool first_point_ = true;
    
    // 预测器重用优化
    PredictorType last_used_predictor_ = PREDICTOR_ZP;  // 初始化为ZP
    
    // 重构状态（与解压器保持同步）
    GpsPoint current_reconstructed_point_;
    std::deque<HistoryState> history_states_;  // 性能优化：使用deque，pop_front是O(1)
    static constexpr int kMaxHistorySize = 3;  // 只需要保留3个历史状态
    
    // 统计信息
    CompressionStats stats_;
    
    // 动态 Huffman 编码相关（滑动窗口）
    static constexpr int kSlidingWindowSize = 1000;  // 滑动窗口大小
    
    // 性能优化：使用循环数组替代vector，避免erase(begin())的O(n)开销
    PredictorType predictor_window_[kSlidingWindowSize];  // 循环数组
    int window_head_ = 0;                                  // 队首索引
    int window_size_ = 0;                                  // 当前窗口大小
    
    int predictor_frequency_[3] = {0, 0, 0};         // 预测器频率统计 [LDR, CP, ZP]
    
    // Huffman 编码表（根据频率动态生成）
    struct HuffmanCode {
        std::vector<bool> bits;  // 编码比特序列
        int length;              // 编码长度
        
        HuffmanCode() : length(0) {}
        HuffmanCode(const std::vector<bool>& b) : bits(b), length(b.size()) {}
    };
    HuffmanCode huffman_codes_[3];  // 三个预测器的 Huffman 编码
    
    // Huffman 编码辅助函数
    void UpdateHuffmanCodes();
    void EncodeWithHuffman(PredictorType predictor);
    void AddPredictorToWindow(PredictorType predictor);
    
    // 核心算法函数
    
    /**
     * 处理第一个点（写入原始坐标）
     */
    void ProcessFirstPoint(const GpsPoint& point);
    
    /**
     * 并行预测：计算所有预测器的预测结果
     */
    void ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp);
    
    /**
     * 选择最优预测器
     */
    PredictorType SelectBestPredictor(const GpsPoint& current_point,
                                     const GpsPoint& pred_ldr,
                                     const GpsPoint& pred_cp,
                                     const GpsPoint& pred_zp,
                                     GpsPoint& best_prediction);
    
    /**
     * 编码预测器标志和量化误差
     */
    void EncodePrediction(PredictorType predictor,
                         const GpsPoint& current_point,
                         const GpsPoint& predicted_point);
    
    /**
     * 优化编码预测器标志和量化误差（支持预测器重用）
     */
    void EncodePredictionOptimized(PredictorType predictor,
                                 const GpsPoint& current_point,
                                 const GpsPoint& predicted_point);
    
    /**
     * 更新历史状态
     */
    void UpdateHistory(const GpsPoint& reconstructed_point);
    
    /**
     * 更新重构状态（用于优化编码）
     */
    void UpdateReconstructedState(const GpsPoint& reconstructed_point);
    
    
    /**
     * 辅助函数：计算两点间欧氏距离
     */
    double CalculateDistance(const GpsPoint& p1, const GpsPoint& p2) const;
};

/**
 * TrajCompress-SP 解压缩器
 */
class TrajCompressSPDecompressor {
public:
    using GpsPoint = TrajCompressSPCompressor::GpsPoint;
    using PredictorType = TrajCompressSPCompressor::PredictorType;
    using HistoryState = TrajCompressSPCompressor::HistoryState;
    
    /**
     * 构造函数
     * @param compressed_data 压缩数据
     * @param data_size 数据大小（字节）
     */
    TrajCompressSPDecompressor(uint8_t* compressed_data, int data_size);
    
    /**
     * 读取下一个GPS点
     * @return 是否成功读取
     */
    bool ReadNextPoint(GpsPoint& point);
    
    /**
     * 获取所有解压后的点
     */
    std::vector<GpsPoint> ReadAllPoints();

private:
    std::unique_ptr<class InputBitStream> input_bit_stream_;
    
    // 参数（从压缩数据头读取）
    int block_size_;
    double epsilon_;
    double quant_step_;
    
    // 重构状态
    bool first_point_ = true;
    GpsPoint current_reconstructed_point_;
    std::deque<HistoryState> history_states_;  // 性能优化：使用deque，pop_front是O(1)
    static constexpr int kMaxHistorySize = 3;
    
    // 预测器重用状态
    PredictorType last_used_predictor_ = TrajCompressSPCompressor::PREDICTOR_ZP;  // 初始化为ZP
    
    // 动态 Huffman 编码相关（滑动窗口）
    static constexpr int kSlidingWindowSize = 1000;  // 滑动窗口大小
    
    // 性能优化：使用循环数组替代vector，避免erase(begin())的O(n)开销
    PredictorType predictor_window_[kSlidingWindowSize];  // 循环数组
    int window_head_ = 0;                                  // 队首索引
    int window_size_ = 0;                                  // 当前窗口大小
    
    int predictor_frequency_[3] = {0, 0, 0};         // 预测器频率统计
    
    // Huffman 解码表（缓存，与编码器同步）
    PredictorType huffman_decoder_map_[3];  // [0]=最高频率, [1]=第二频率, [2]=最低频率
    
    // Huffman 解码辅助函数
    void UpdateHuffmanDecoder();
    PredictorType DecodeWithHuffman();
    void AddPredictorToWindow(PredictorType predictor);
    
    // 辅助函数
    void ReadHeader();
    void ParallelPredict(GpsPoint& pred_ldr, GpsPoint& pred_cp, GpsPoint& pred_zp, uint64_t current_timestamp);
    void UpdateHistory(const GpsPoint& reconstructed_point);
};

