#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef COST_ACOST_PREDICTOR
#include "experiments/e2_ablation/cost_acost_predictor_compressor.h"
#else
#include "algorithm/cost_compressor.h"
#endif
#include "utils/benchmark_utils.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {
constexpr int kBlockSize = 10000;
constexpr double kMaxDiff = 1.0e-5;
constexpr int kEvaluationWindow = 96;

struct InputPoint { double lon; double lat; uint64_t timestamp; };
struct Record {
    long long bits = 0, points = 0, trajectories = 0;
    long long compress_ns = 0, decompress_ns = 0;
    long long ldr_count = 0, cp_count = 0, zp_count = 0;
    long long multi_mode_points = 0, ldr_only_points = 0;
    double max_error = 0.0;
};

void RunTrajectory(const std::vector<InputPoint>& trajectory,
                   const std::string& dataset, const std::string& method,
                   Record& record) {
    std::vector<CoSTCompressor::GpsPoint> points;
    points.reserve(trajectory.size());
    for (const auto& point : trajectory)
        points.emplace_back(point.lon, point.lat, point.timestamp);

    CoSTCompressor compressor(kBlockSize, kMaxDiff, kEvaluationWindow, false);
    benchmark_utils::DoNotOptimize(points.data());
    benchmark_utils::ClobberMemory();
    const auto c0 = std::chrono::steady_clock::now();
    for (const auto& point : points) compressor.AddGpsPoint(point);
    compressor.Close();
    benchmark_utils::ClobberMemory();
    const auto c1 = std::chrono::steady_clock::now();

    const auto& stats = compressor.GetStats();
    Array<uint8_t> compressed = compressor.GetCompressedData();
    benchmark_utils::ClobberMemory();
    const auto d0 = std::chrono::steady_clock::now();
    CoSTDecompressor decompressor(&compressed[0], compressed.length());
    std::vector<CoSTCompressor::GpsPoint> decoded;
    decoded.reserve(points.size());
    CoSTCompressor::GpsPoint point;
    for (size_t i = 0; i < points.size(); ++i) {
        if (!decompressor.ReadNextPoint(point)) break;
        decoded.push_back(point);
    }
    benchmark_utils::ClobberMemory();
    const auto d1 = std::chrono::steady_clock::now();

    if (decoded.size() != trajectory.size())
        throw std::runtime_error("decompression point count mismatch");
    for (size_t i = 0; i < trajectory.size(); ++i) {
        record.max_error = std::max(record.max_error,
            std::hypot(trajectory[i].lon - decoded[i].longitude,
                       trajectory[i].lat - decoded[i].latitude));
    }
    record.bits += static_cast<long long>(stats.total_bits) - stats.timestamp_bits;
    record.compress_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(c1 - c0).count();
    record.decompress_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(d1 - d0).count();
    record.points += static_cast<long long>(trajectory.size());
    record.trajectories++;
    record.ldr_count += stats.ldr_count;
    record.cp_count += stats.cp_count;
    record.zp_count += stats.zp_count;
    record.multi_mode_points += stats.multi_predictor_mode_points;
    record.ldr_only_points += stats.ldr_only_mode_points;
}

bool ReadU32(uint32_t& value) {
    std::cin.read(reinterpret_cast<char*>(&value), sizeof(value));
    return static_cast<bool>(std::cin);
}
}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    if (argc != 3) {
        std::cerr << "usage: benchmark METHOD DATASET < binary_trajectory_stream\n";
        return 2;
    }
    const std::string method = argv[1];
    const std::string dataset = argv[2];
    Record record;
    uint32_t trajectory_count = 0;
    if (!ReadU32(trajectory_count)) {
        std::cerr << "missing trajectory count\n";
        return 2;
    }
    for (uint32_t i = 0; i < trajectory_count; ++i) {
        uint32_t point_count = 0;
        if (!ReadU32(point_count)) {
            std::cerr << "truncated trajectory header\n";
            return 2;
        }
        std::vector<InputPoint> trajectory(point_count);
        for (auto& point : trajectory) {
            std::cin.read(reinterpret_cast<char*>(&point.lon), sizeof(double));
            std::cin.read(reinterpret_cast<char*>(&point.lat), sizeof(double));
            std::cin.read(reinterpret_cast<char*>(&point.timestamp), sizeof(uint64_t));
            if (!std::cin) {
                std::cerr << "truncated trajectory point data\n";
                return 2;
            }
        }
        RunTrajectory(trajectory, dataset, method, record);
    }
    if (record.points == 0) {
        std::cerr << "no points processed\n";
        return 2;
    }
    const double bpp = static_cast<double>(record.bits) / record.points;
    const double ratio = static_cast<double>(record.bits) / (record.points * 128.0);
    const double compress_us = static_cast<double>(record.compress_ns) / 1000.0 / record.points;
    const double decompress_us = static_cast<double>(record.decompress_ns) / 1000.0 / record.points;
    std::cout << "Method,DataSet,TrajectoryCount,PointCount,BlockSize,MaxDiff,"
                 "CompressionRatio,BPP,CompressionTime(us/point),DecompressionTime(us/point),"
                 "CompressedBits,MaxRoundtripError,LDRCount,CPCount,ZPCount,"
                 "MultiPredictorModePoints,LDROnlyModePoints\n";
    std::cout << method << ',' << dataset << ',' << record.trajectories << ',' << record.points
              << ',' << kBlockSize << ',' << std::scientific << std::setprecision(6) << kMaxDiff
              << ',' << std::fixed << std::setprecision(6) << ratio << ',' << bpp
              << ',' << compress_us << ',' << decompress_us << ',' << record.bits
              << ',' << std::scientific << record.max_error << ',' << record.ldr_count
              << ',' << record.cp_count << ',' << record.zp_count << ','
              << record.multi_mode_points << ',' << record.ldr_only_points << '\n';
    return 0;
}
