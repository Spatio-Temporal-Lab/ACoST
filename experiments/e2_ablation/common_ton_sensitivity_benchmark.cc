#include "algorithm/acost_compressor.h"
#include "utils/benchmark_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {
struct InputPoint { double lon, lat; uint64_t timestamp; };
struct Record {
    long long bits = 0, points = 0, trajectories = 0;
    long long compress_ns = 0, decompress_ns = 0;
    double max_error = 0.0;
};

bool ReadU32(uint32_t& value) {
    std::cin.read(reinterpret_cast<char*>(&value), sizeof(value));
    return static_cast<bool>(std::cin);
}
}

int main(int argc, char** argv) {
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    if (argc != 6) {
        std::cerr << "usage: benchmark METHOD DATASET EPSILON TAU T_ON < binary_trajectory_stream\n";
        return 2;
    }
    const std::string method = argv[1];
    const std::string dataset = argv[2];
    const double epsilon = std::stod(argv[3]);
    const double tau = std::stod(argv[4]);
    const double t_on = std::stod(argv[5]);
    if (!(epsilon > 0.0) || !(tau > 0.0) || !(t_on > 0.0)) {
        std::cerr << "epsilon, tau and T_on must be positive\n";
        return 2;
    }

    uint32_t trajectory_count = 0;
    if (!ReadU32(trajectory_count)) {
        std::cerr << "missing trajectory count\n";
        return 2;
    }
    Record record;
    ACoSTConfig config;
    config.ctra_damp_onset = t_on;
    config.zp_selection_onset = t_on;
    config.ctra_damp_tau = tau;

    for (uint32_t t = 0; t < trajectory_count; ++t) {
        uint32_t point_count = 0;
        if (!ReadU32(point_count)) {
            std::cerr << "truncated trajectory header\n";
            return 2;
        }
        std::vector<ACoSTCompressor::GpsPoint> points;
        std::vector<InputPoint> original(point_count);
        points.reserve(point_count);
        for (uint32_t p = 0; p < point_count; ++p) {
            std::cin.read(reinterpret_cast<char*>(&original[p].lon), sizeof(double));
            std::cin.read(reinterpret_cast<char*>(&original[p].lat), sizeof(double));
            std::cin.read(reinterpret_cast<char*>(&original[p].timestamp), sizeof(uint64_t));
            if (!std::cin) {
                std::cerr << "truncated trajectory point data\n";
                return 2;
            }
            points.emplace_back(original[p].lon, original[p].lat, original[p].timestamp);
        }

        ACoSTCompressor compressor(10000, epsilon, config);
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
        // The decoder receives exactly the same T_on and tau configuration as
        // the encoder so mirrored prediction and ZP decisions stay aligned.
        ACoSTDecompressor decompressor(&compressed[0], compressed.length(), config);
        std::vector<ACoSTCompressor::GpsPoint> decoded;
        decoded.reserve(points.size());
        ACoSTCompressor::GpsPoint point;
        for (size_t i = 0; i < points.size(); ++i) {
            if (!decompressor.ReadNextPoint(point)) break;
            decoded.push_back(point);
        }
        benchmark_utils::ClobberMemory();
        const auto d1 = std::chrono::steady_clock::now();

        if (decoded.size() != points.size()) {
            std::cerr << "decompression point count mismatch\n";
            return 3;
        }
        for (size_t i = 0; i < original.size(); ++i) {
            record.max_error = std::max(record.max_error,
                std::hypot(original[i].lon - decoded[i].longitude,
                           original[i].lat - decoded[i].latitude));
        }
        record.bits += static_cast<long long>(stats.total_bits) - stats.timestamp_bits;
        record.compress_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(c1 - c0).count();
        record.decompress_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(d1 - d0).count();
        record.points += static_cast<long long>(points.size());
        ++record.trajectories;
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
                 "CompressedBits,MaxRoundtripError\n";
    std::cout << method << ',' << dataset << ',' << record.trajectories << ',' << record.points
              << ",10000," << std::scientific << std::setprecision(6) << epsilon
              << ',' << std::fixed << std::setprecision(6) << ratio << ',' << bpp
              << ',' << compress_us << ',' << decompress_us << ',' << record.bits
              << ',' << std::scientific << record.max_error << '\n';
    return 0;
}
