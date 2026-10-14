// E1: Overall comparison of the paper's 12 methods on normalized trajectories.
//
// Algorithms:
//   Ours:      ACoST, CoST
//   Serf-base: Gorilla, Serf-Qt, Serf-XOR, Elf
//   Trad:      OPWTR, DR, SQUISH-E, VOLTCom
//   Baseline:  Raw
//
// Input is staged by the E1 runner from the six paper datasets.
// Epsilon: configurable (default 1e-5)
//
// Build:  g++ -O3 -std=c++17 -I../../ e1_overall_benchmark.cc \
//           ../../algorithm/acost_compressor.cc ../../algorithm/cost_compressor.cc \
//           ../../baselines/serf/serf_qt_compressor.cc ../../baselines/serf/serf_qt_decompressor.cc \
//           ../../baselines/serf/serf_xor_compressor.cc ../../baselines/serf/serf_xor_decompressor.cc \
//           ../../baselines/gorilla/gorilla_compressor.cc ../../baselines/gorilla/gorilla_decompressor.cc \
//           ../../baselines/elf/elf.cc \
//           ../../utils/output_bit_stream.cc ../../utils/input_bit_stream.cc \
//           ../../utils/elias_gamma_codec.cc ../../utils/elias_delta_codec.cc \
//           -o e1_overall_benchmark

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ─── Our algorithms ───
#include "../../algorithm/acost_compressor.h"
#include "../../algorithm/cost_compressor.h"

// ─── Serf baselines ───
#include "../../baselines/serf/serf_qt_compressor.h"
#include "../../baselines/serf/serf_qt_decompressor.h"
#include "../../baselines/serf/serf_xor_compressor.h"
#include "../../baselines/serf/serf_xor_decompressor.h"
#include "../../baselines/gorilla/gorilla_compressor.h"
#include "../../baselines/gorilla/gorilla_decompressor.h"
#include "../../baselines/elf/elf.h"

// ─── Utils ───
#include "../../utils/benchmark_utils.h"

// ─── LZ77 (FastLZ) ───
extern "C" {
#include "../../baselines/lz77/fastlz.h"
}

namespace {

namespace fs = std::filesystem;

// ─── Configurable parameters ───
double   g_max_diff           = 1.0e-5;
int      g_streaming_block    = 10000;
int      g_serf_xor_window    = 1000;
int      g_cost_eval_window   = 96;
bool     g_cost_time_window   = false;
int      g_normalized_max     = 0;   // 0 = unlimited
int      g_tdrive_max         = 0;
std::string g_only_method;
int      g_min_points         = 1;
int      g_max_files          = 0;
int      g_sampling_interval  = 0;
std::string g_dataset_label;
std::vector<std::string> g_methods;
fs::path g_data_root          = "../../data";
fs::path g_output_path        = "../../results/e1_results.csv";
std::vector<std::string> g_datasets = {
    "Geolife", "WorldTrace", "T-dirve", "ais", "trajair", "Berlin"
};
constexpr int kNormalizedFileMax = 20000;
constexpr int kTDriveFileMax   = 11000;
constexpr size_t kMinTrajPts   = 10;

// ─── Traditional algorithm parameters (converted from degree-epsilon) ───
double g_trad_eps_deg = 1.0e-5;    // SED threshold in degrees
double g_trad_eps_m   = 1.11;      // ~1e-5 * 111000 m

struct Point2D {
    double   x, y;
    uint64_t timestamp;
};

struct PerfRecord {
    std::string method, dataset;
    long long total_compressed_bits       = 0;
    long long total_compression_time_us   = 0;
    long long total_decompression_time_us = 0;
    long long total_points                = 0;
    long long total_trajectories          = 0;
    double    max_roundtrip_error         = 0.0;
    long long emitted_keypoints           = 0;  // for trad algorithms

    double CR() const {
        long long ob = total_points * 128LL;
        return ob > 0 ? (double)total_compressed_bits / ob : 0.0;
    }
    double BPP() const {
        return total_points > 0 ? (double)total_compressed_bits / total_points : 0.0;
    }
    double AvgCT() const {
        return total_points > 0 ? (double)total_compression_time_us / total_points : 0.0;
    }
    double AvgDT() const {
        return total_points > 0 ? (double)total_decompression_time_us / total_points : 0.0;
    }
};

// ─── Timestamp parsing ───

uint64_t ParseDateTime(const std::string& s) {
    std::tm tm = {};
    std::istringstream ss(s.substr(0, 19));
    ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
    if (ss.fail()) throw std::runtime_error("parse fail: " + s);
    return static_cast<uint64_t>(std::mktime(&tm));
}

uint64_t ParseNormalizedTs(const std::string& s) {
    return ParseDateTime(s.substr(0, 10) + " " + s.substr(11, 8));
}

// ─── Error helpers ───

template <typename PointT>
void UpdateMaxErr(const std::vector<Point2D>& orig,
                  const std::vector<PointT>& dec, PerfRecord& r) {
    for (size_t i = 0; i < std::min(orig.size(), dec.size()); ++i) {
        double dx = orig[i].x - dec[i].longitude;
        double dy = orig[i].y - dec[i].latitude;
        r.max_roundtrip_error = std::max(r.max_roundtrip_error, std::sqrt(dx*dx + dy*dy));
    }
}

void UpdateMaxErrSerf(const std::vector<Point2D>& orig,
                      const std::vector<double>& xs,
                      const std::vector<double>& ys, PerfRecord& r) {
    for (size_t i = 0; i < orig.size(); ++i) {
        double dx = orig[i].x - xs[i], dy = orig[i].y - ys[i];
        r.max_roundtrip_error = std::max(r.max_roundtrip_error, std::sqrt(dx*dx + dy*dy));
    }
}

// =====================================================================
// Algorithm Runners
// =====================================================================

void RunACoST(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    std::vector<ACoSTCompressor::GpsPoint> pts;
    pts.reserve(traj.size());
    for (auto& p : traj) pts.emplace_back(p.x, p.y, p.timestamp);

    ACoSTCompressor comp(g_streaming_block, g_max_diff);
    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    for (auto& p : pts) comp.AddGpsPoint(p);
    comp.Close();
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    auto& stats = comp.GetStats();
    long long bits = (long long)stats.total_bits - stats.timestamp_bits;
    auto compressed = comp.GetCompressedData();

    benchmark_utils::ClobberMemory();
    auto t2 = std::chrono::steady_clock::now();
    ACoSTDecompressor decomp(&compressed[0], compressed.length());
    std::vector<ACoSTCompressor::GpsPoint> dec;
    dec.reserve(pts.size());
    ACoSTCompressor::GpsPoint pt;
    for (size_t i = 0; i < pts.size(); ++i) { if (!decomp.ReadNextPoint(pt)) break; dec.push_back(pt); }
    benchmark_utils::ClobberMemory();
    auto t3 = std::chrono::steady_clock::now();

    UpdateMaxErr(traj, dec, r);
    r.total_compressed_bits       += bits;
    r.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_decompression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

void RunCoST(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    std::vector<CoSTCompressor::GpsPoint> pts;
    pts.reserve(traj.size());
    for (auto& p : traj) pts.emplace_back(p.x, p.y, p.timestamp);

    CoSTCompressor comp(g_streaming_block, g_max_diff, g_cost_eval_window, g_cost_time_window);
    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    for (auto& p : pts) comp.AddGpsPoint(p);
    comp.Close();
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    auto& stats = comp.GetStats();
    long long bits = (long long)stats.total_bits - stats.timestamp_bits;
    auto compressed = comp.GetCompressedData();

    benchmark_utils::ClobberMemory();
    auto t2 = std::chrono::steady_clock::now();
    CoSTDecompressor decomp(&compressed[0], compressed.length());
    std::vector<CoSTCompressor::GpsPoint> dec;
    dec.reserve(pts.size());
    CoSTCompressor::GpsPoint pt;
    for (size_t i = 0; i < pts.size(); ++i) { if (!decomp.ReadNextPoint(pt)) break; dec.push_back(pt); }
    benchmark_utils::ClobberMemory();
    auto t3 = std::chrono::steady_clock::now();

    UpdateMaxErr(traj, dec, r);
    r.total_compressed_bits       += bits;
    r.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_decompression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

void RunGorilla(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    GorillaCompressor comp_x((int)traj.size()), comp_y((int)traj.size());

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    for (auto& p : traj) { comp_x.addValue(p.x); comp_y.addValue(p.y); }
    comp_x.close(); comp_y.close();
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    long long bits = comp_x.get_compress_size_in_bits() + comp_y.get_compress_size_in_bits();
    auto bx = comp_x.get_compress_pack(), by = comp_y.get_compress_pack();

    benchmark_utils::ClobberMemory();
    auto t2 = std::chrono::steady_clock::now();
    GorillaDecompressor decomp_x, decomp_y;
    auto dx = decomp_x.decompress(bx), dy = decomp_y.decompress(by);
    benchmark_utils::ClobberMemory();
    auto t3 = std::chrono::steady_clock::now();

    UpdateMaxErrSerf(traj, dx, dy, r);
    r.total_compressed_bits       += bits;
    r.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_decompression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

void RunSerfQt(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    SerfQtCompressor comp_x((int)traj.size(), g_max_diff), comp_y((int)traj.size(), g_max_diff);

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    for (auto& p : traj) { comp_x.AddValue(p.x); comp_y.AddValue(p.y); }
    comp_x.Close(); comp_y.Close();
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    long long bits = comp_x.get_compressed_size_in_bits() + comp_y.get_compressed_size_in_bits();
    auto bx = comp_x.compressed_bytes(), by = comp_y.compressed_bytes();

    benchmark_utils::ClobberMemory();
    auto t2 = std::chrono::steady_clock::now();
    SerfQtDecompressor decomp_x, decomp_y;
    auto dx = decomp_x.Decompress(bx), dy = decomp_y.Decompress(by);
    benchmark_utils::ClobberMemory();
    auto t3 = std::chrono::steady_clock::now();

    UpdateMaxErrSerf(traj, dx, dy, r);
    r.total_compressed_bits       += bits;
    r.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_decompression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

void RunSerfXor(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    SerfXORCompressor comp_x(g_serf_xor_window, g_max_diff, 0), comp_y(g_serf_xor_window, g_max_diff, 0);

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    for (auto& p : traj) { comp_x.AddValue(p.x); comp_y.AddValue(p.y); }
    comp_x.Close(); comp_y.Close();
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    long long bits = comp_x.compressed_size_last_block() + comp_y.compressed_size_last_block();
    auto bx = comp_x.compressed_bytes_last_block(), by = comp_y.compressed_bytes_last_block();

    benchmark_utils::ClobberMemory();
    auto t2 = std::chrono::steady_clock::now();
    SerfXORDecompressor decomp_x(0), decomp_y(0);
    auto dx = decomp_x.Decompress(bx), dy = decomp_y.Decompress(by);
    benchmark_utils::ClobberMemory();
    auto t3 = std::chrono::steady_clock::now();

    UpdateMaxErrSerf(traj, dx, dy, r);
    r.total_compressed_bits       += bits;
    r.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_decompression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

void RunElf(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    // Elf is batch-oriented: encode array of doubles
    // Note: Elf may assert/crash on some GPS data (known limitation)
    std::vector<double> xs(traj.size()), ys(traj.size());
    for (size_t i = 0; i < traj.size(); ++i) { xs[i] = traj[i].x; ys[i] = traj[i].y; }

    uint8_t *out_x = nullptr, *out_y = nullptr;

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    ssize_t enc_x = elf_encode(xs.data(), (ssize_t)xs.size(), &out_x, 0.0);
    ssize_t enc_y = elf_encode(ys.data(), (ssize_t)ys.size(), &out_y, 0.0);
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    if (enc_x <= 0 || enc_y <= 0) {
        if (out_x) free(out_x);
        if (out_y) free(out_y);
        // Skip this trajectory for Elf but still count points for fairness
        r.total_compressed_bits += (long long)traj.size() * 128LL; // fallback: raw size
        r.total_points += (long long)traj.size();
        r.total_trajectories++;
        return;
    }

    long long bits = (enc_x + enc_y) * 8LL;

    std::vector<double> dec_x(traj.size()), dec_y(traj.size());
    benchmark_utils::ClobberMemory();
    auto t2 = std::chrono::steady_clock::now();
    ssize_t decoded_x = elf_decode(out_x, enc_x, dec_x.data(), 0.0);
    ssize_t decoded_y = elf_decode(out_y, enc_y, dec_y.data(), 0.0);
    benchmark_utils::ClobberMemory();
    auto t3 = std::chrono::steady_clock::now();

    if (out_x) free(out_x);
    if (out_y) free(out_y);

    if (decoded_x != (ssize_t)traj.size() || decoded_y != (ssize_t)traj.size())
        throw std::runtime_error("Elf decoded value count mismatch");
    for (size_t i = 0; i < traj.size(); ++i) {
        if (std::memcmp(&traj[i].x, &dec_x[i], sizeof(double)) != 0 ||
            std::memcmp(&traj[i].y, &dec_y[i], sizeof(double)) != 0)
            throw std::runtime_error("Elf lossless round-trip mismatch");
    }

    UpdateMaxErrSerf(traj, dec_x, dec_y, r);
    r.total_compressed_bits       += bits;
    r.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_decompression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

// ─── Traditional trajectory compression algorithms ───
// These emit keypoints. CR = emitted × 128 / (total × 128)

// OPWTR: Online Piece-Wise Time Ratio (SED-based)
void RunOPWTR(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.size() < 2) return;

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();

    int emitted = 1; // first point always emitted
    size_t anchor = 0;
    for (size_t i = 2; i < traj.size(); ++i) {
        // Check all intermediate points against anchor→current segment
        bool over = false;
        double ax = traj[anchor].x, ay = traj[anchor].y;
        double bx = traj[i].x,      by = traj[i].y;
        double dt_ab = (double)(traj[i].timestamp - traj[anchor].timestamp);
        if (dt_ab <= 0) dt_ab = 1.0;

        for (size_t j = anchor + 1; j < i; ++j) {
            double t_frac = (double)(traj[j].timestamp - traj[anchor].timestamp) / dt_ab;
            double ix = ax + (bx - ax) * t_frac;
            double iy = ay + (by - ay) * t_frac;
            double dx = traj[j].x - ix, dy = traj[j].y - iy;
            if (std::sqrt(dx*dx + dy*dy) > g_trad_eps_deg) { over = true; break; }
        }
        if (over) {
            emitted++;     // emit point i-1
            anchor = i - 1;
        }
    }
    emitted++; // last point

    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    r.emitted_keypoints += emitted;
    r.total_compressed_bits += (long long)emitted * 128LL;
    r.total_compression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

// Dead Reckoning
void RunDR(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.size() < 2) return;

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();

    int emitted = 1;
    double pred_x = traj[0].x, pred_y = traj[0].y;
    double vx = 0.0, vy = 0.0;
    for (size_t i = 1; i < traj.size(); ++i) {
        double dt = (double)(traj[i].timestamp - traj[i-1].timestamp);
        if (dt <= 0) dt = 1.0;
        pred_x += vx * dt;
        pred_y += vy * dt;
        double dx = traj[i].x - pred_x, dy = traj[i].y - pred_y;
        if (std::sqrt(dx*dx + dy*dy) > g_trad_eps_deg) {
            emitted++;
            vx = (traj[i].x - traj[i-1].x) / dt;
            vy = (traj[i].y - traj[i-1].y) / dt;
            pred_x = traj[i].x;
            pred_y = traj[i].y;
        }
    }

    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    r.emitted_keypoints += emitted;
    r.total_compressed_bits += (long long)emitted * 128LL;
    r.total_compression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

// SQUISH-E: SED-based with priority queue eviction (simplified as greedy SED)
void RunSQUISHE(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.size() < 3) { r.total_points += (long long)traj.size(); r.total_trajectories++; return; }

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();

    // Simple greedy: keep points where SED > threshold
    std::vector<bool> keep(traj.size(), false);
    keep[0] = keep[traj.size()-1] = true;

    // Douglas-Peucker-like simplification (online variant)
    size_t anchor = 0;
    for (size_t i = 1; i < traj.size(); ++i) {
        if (i == traj.size() - 1) { keep[i] = true; break; }
        double ax = traj[anchor].x, ay = traj[anchor].y;
        // Look ahead to next kept point (simplified: use i+1 as tentative end)
        size_t end = std::min(i + 1, traj.size() - 1);
        double bx = traj[end].x, by = traj[end].y;
        double dt_ab = (double)(traj[end].timestamp - traj[anchor].timestamp);
        if (dt_ab <= 0) dt_ab = 1.0;
        double t_frac = (double)(traj[i].timestamp - traj[anchor].timestamp) / dt_ab;
        double ix = ax + (bx - ax) * t_frac;
        double iy = ay + (by - ay) * t_frac;
        double dx = traj[i].x - ix, dy = traj[i].y - iy;
        if (std::sqrt(dx*dx + dy*dy) > g_trad_eps_deg) {
            keep[i] = true;
            anchor = i;
        }
    }

    int emitted = 0;
    for (bool k : keep) if (k) emitted++;

    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    r.emitted_keypoints += emitted;
    r.total_compressed_bits += (long long)emitted * 128LL;
    r.total_compression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

// VOLTCom: Vector-based Online Trajectory Compression
void RunVOLTCom(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.size() < 2) return;

    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();

    int emitted = 1;
    size_t anchor = 0;
    for (size_t i = 1; i < traj.size(); ++i) {
        double dx = traj[i].x - traj[anchor].x;
        double dy = traj[i].y - traj[anchor].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        // Simplified Haversine: dist in degrees, compare with threshold
        if (dist > g_trad_eps_deg * 10.0) { // VOLTCom typically uses larger threshold
            emitted++;
            anchor = i;
        }
    }
    emitted++; // last point

    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    r.emitted_keypoints += emitted;
    r.total_compressed_bits += (long long)emitted * 128LL;
    r.total_compression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_points += (long long)traj.size();
    r.total_trajectories++;
}

// Raw: no compression
void RunRaw(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    r.total_compressed_bits += (long long)traj.size() * 128LL;
    r.total_points          += (long long)traj.size();
    r.total_trajectories++;
}

// LZ77 (FastLZ): lossless block compression on raw lon/lat bytes
void RunLZ77(const std::vector<Point2D>& traj, PerfRecord& r) {
    if (traj.empty()) return;
    // Pack lon+lat as raw doubles (16 bytes per point)
    size_t raw_size = traj.size() * 16;
    std::vector<uint8_t> raw_buf(raw_size);
    for (size_t i = 0; i < traj.size(); ++i) {
        std::memcpy(&raw_buf[i*16],     &traj[i].x, 8);
        std::memcpy(&raw_buf[i*16 + 8], &traj[i].y, 8);
    }

    // Output buffer: FastLZ needs at least 5% larger + 66
    size_t out_cap = raw_size + raw_size / 20 + 66;
    std::vector<uint8_t> comp_buf(out_cap);

    benchmark_utils::DoNotOptimize(raw_buf.data());
    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    int comp_len = fastlz_compress_level(1, raw_buf.data(), (int)raw_size, comp_buf.data());
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    r.total_compressed_bits       += (long long)comp_len * 8LL;
    r.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    r.total_points                += (long long)traj.size();
    r.total_trajectories++;
}

// ─── Method dispatch ───

constexpr int kMethodCount = 12;
const char* kMethodNames[kMethodCount] = {
    "acost", "cost", "gorilla", "serf_qt", "serf_xor", "elf",
    "opwtr", "dr", "squish_e", "voltcom", "lz77", "raw"
};

using RunFunc = void(*)(const std::vector<Point2D>&, PerfRecord&);
RunFunc kRunFuncs[kMethodCount] = {
    RunACoST, RunCoST, RunGorilla, RunSerfQt, RunSerfXor, RunElf,
    RunOPWTR, RunDR, RunSQUISHE, RunVOLTCom, RunLZ77, RunRaw
};

void RunAllMethods(const std::vector<Point2D>& traj, std::vector<PerfRecord>& records) {
    for (int i = 0; i < kMethodCount; ++i) {
        const bool selected = g_methods.empty()
            ? (g_only_method.empty() || g_only_method == kMethodNames[i])
            : std::find(g_methods.begin(), g_methods.end(), kMethodNames[i]) != g_methods.end();
        if (selected)
            kRunFuncs[i](traj, records[i]);
    }
}

std::vector<PerfRecord> InitRecords(const std::string& dataset) {
    std::vector<PerfRecord> records(kMethodCount);
    for (int i = 0; i < kMethodCount; ++i) {
        records[i].method  = kMethodNames[i];
        records[i].dataset = dataset;
    }
    return records;
}

// ─── Print / CSV ───

void PrintRecord(const PerfRecord& r) {
    std::cout << "  " << std::left << std::setw(12) << r.method
              << " traj=" << r.total_trajectories
              << " pts="  << r.total_points
              << " CR="   << std::fixed << std::setprecision(6) << r.CR()
              << " BPP="  << r.BPP()
              << " CT="   << std::setprecision(4) << r.AvgCT() << "us"
              << " DT="   << r.AvgDT() << "us"
              << std::defaultfloat << '\n';
}

void WriteCSVRow(std::ofstream& csv, const PerfRecord& r) {
    csv << r.method << ',' << r.dataset << ','
        << r.total_trajectories << ',' << r.total_points << ','
        << g_streaming_block << ',' << std::scientific << g_max_diff << ','
        << std::fixed << std::setprecision(6) << r.CR() << ','
        << r.BPP() << ',' << r.AvgCT() << ',' << r.AvgDT() << ','
        << r.total_compressed_bits << ','
        << std::scientific << r.max_roundtrip_error << '\n';
}

PerfRecord SumRecords(const std::string& method, const std::vector<PerfRecord>& rows) {
    PerfRecord t;
    t.method = method; t.dataset = "TOTAL";
    for (auto& r : rows) {
        t.total_compressed_bits       += r.total_compressed_bits;
        t.total_compression_time_us   += r.total_compression_time_us;
        t.total_decompression_time_us += r.total_decompression_time_us;
        t.total_points                += r.total_points;
        t.total_trajectories          += r.total_trajectories;
        t.max_roundtrip_error = std::max(t.max_roundtrip_error, r.max_roundtrip_error);
    }
    return t;
}

// =====================================================================
// Data Loaders (from experiment 33)
// =====================================================================

bool LoadNormalized(const fs::path& dir, int idx, std::vector<Point2D>& traj) {
    traj.clear();
    std::ifstream f(dir / (std::to_string(idx) + ".txt"));
    if (!f.is_open()) return false;
    std::string line;
    std::getline(f, line); // header
    uint64_t base = 0; bool hb = false;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string ts, lo, la;
        if (!std::getline(ss, ts, ',') || !std::getline(ss, lo, ',') || !std::getline(ss, la, ',')) continue;
        try {
            uint64_t s = ParseNormalizedTs(ts);
            if (!hb) { base = s; hb = true; }
            uint64_t t = (s >= base) ? (s - base) * 10ULL : 0ULL;
            if (!traj.empty() && t <= traj.back().timestamp) t = traj.back().timestamp + 1;
            traj.push_back({std::stod(lo), std::stod(la), t});
        } catch (...) {}
    }
    return !traj.empty();
}

bool LoadGeolife(const fs::path& path, std::vector<Point2D>& traj) {
    traj.clear();
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string line;
    for (int i = 0; i < 6 && std::getline(f, line); ++i) {}
    uint64_t base = 0; bool hb = false;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string la, lo, z, alt, d, dt, tm;
        if (!std::getline(ss,la,',') || !std::getline(ss,lo,',') || !std::getline(ss,z,',')
            || !std::getline(ss,alt,',') || !std::getline(ss,d,',')
            || !std::getline(ss,dt,',') || !std::getline(ss,tm,',')) continue;
        if (!tm.empty() && tm.back()=='\r') tm.pop_back();
        try {
            uint64_t s = ParseDateTime(dt + " " + tm);
            if (!hb) { base = s; hb = true; }
            uint64_t t = (s >= base) ? (s - base) * 10ULL : 0ULL;
            if (!traj.empty() && t <= traj.back().timestamp) t = traj.back().timestamp + 1;
            traj.push_back({std::stod(lo), std::stod(la), t});
        } catch (...) {}
    }
    return traj.size() >= kMinTrajPts;
}

bool LoadWorldTrace(const fs::path& path, std::vector<Point2D>& traj) {
    traj.clear();
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string line;
    std::getline(f, line); // header
    uint64_t base = 0; bool hb = false;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::vector<std::string> flds;
        std::string fld;
        while (std::getline(ss, fld, ',')) flds.push_back(fld);
        if (flds.size() < 4) continue;
        try {
            uint64_t s = ParseDateTime(flds[2]);
            if (!hb) { base = s; hb = true; }
            uint64_t t = (s >= base) ? (s - base) * 10ULL : 0ULL;
            if (!traj.empty() && t <= traj.back().timestamp) t = traj.back().timestamp + 1;
            traj.push_back({std::stod(flds[0]), std::stod(flds[1]), t});
        } catch (...) {}
    }
    return !traj.empty();
}

bool LoadTDrive(const fs::path& dir, int idx, std::vector<Point2D>& traj) {
    traj.clear();
    std::ifstream f(dir / (std::to_string(idx) + ".txt"));
    if (!f.is_open()) return false;
    std::string line;
    uint64_t base = 0; bool hb = false;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string id, dt, lo, la;
        if (!std::getline(ss,id,',') || !std::getline(ss,dt,',')
            || !std::getline(ss,lo,',') || !std::getline(ss,la,',')) continue;
        if (!la.empty() && la.back()=='\r') la.pop_back();
        try {
            uint64_t s = ParseDateTime(dt);
            if (!hb) { base = s; hb = true; }
            uint64_t t = (s >= base) ? (s - base) * 10ULL : 0ULL;
            if (!traj.empty() && t <= traj.back().timestamp) t = traj.back().timestamp + 1;
            traj.push_back({std::stod(lo), std::stod(la), t});
        } catch (...) {}
    }
    return !traj.empty();
}

std::vector<fs::path> ListFiles(const fs::path& dir, const std::string& ext) {
    std::vector<fs::path> v;
    for (auto& e : fs::directory_iterator(dir))
        if (e.is_regular_file() && e.path().extension() == ext) v.push_back(e.path());
    std::sort(v.begin(), v.end());
    return v;
}

std::vector<std::string> SplitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"');
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (c == ',' && !quoted) {
            fields.push_back(field);
            field.clear();
        } else {
            field.push_back(c);
        }
    }
    fields.push_back(field);
    return fields;
}

std::string LowerTrim(std::string value) {
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
        value.erase(value.begin());
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
        value.pop_back();
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool LoadCanonicalCsv(const fs::path& path, std::vector<Point2D>& traj) {
    traj.clear();
    std::ifstream input(path);
    std::string line;
    if (!input || !std::getline(input, line)) return false;
    const auto header = SplitCsvLine(line);
    int lon_col = -1, lat_col = -1, time_col = -1;
    for (size_t i = 0; i < header.size(); ++i) {
        const auto name = LowerTrim(header[i]);
        if (name == "longitude" || name == "lon") lon_col = static_cast<int>(i);
        if (name == "latitude" || name == "lat") lat_col = static_cast<int>(i);
        if (name == "timestamp" || name == "datetime" || name == "time")
            time_col = static_cast<int>(i);
    }
    if (lon_col < 0 || lat_col < 0 || time_col < 0) return false;

    uint64_t base = 0;
    bool has_base = false;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        const auto fields = SplitCsvLine(line);
        const auto max_col = std::max({lon_col, lat_col, time_col});
        if (static_cast<int>(fields.size()) <= max_col) continue;
        try {
            auto timestamp = fields[time_col];
            if (timestamp.size() > 10 && timestamp[10] == 'T') timestamp[10] = ' ';
            const uint64_t seconds = ParseDateTime(timestamp);
            if (!has_base) { base = seconds; has_base = true; }
            uint64_t tick = seconds >= base ? (seconds - base) * 10ULL : 0ULL;
            if (!traj.empty() && tick <= traj.back().timestamp)
                tick = traj.back().timestamp + 1;
            traj.push_back({std::stod(fields[lon_col]), std::stod(fields[lat_col]), tick});
        } catch (...) {
            // Ignore malformed CSV rows, matching the benchmark's prior loader.
        }
    }
    return traj.size() >= g_min_points;
}

void ProcessCanonicalDataset(std::vector<PerfRecord>& records, const fs::path& dir) {
    if (!fs::is_directory(dir))
        throw std::runtime_error("missing dataset directory: " + dir.string());
    std::vector<fs::path> files;
    for (const auto& entry : fs::recursive_directory_iterator(dir))
        if (entry.is_regular_file() && entry.path().extension() == ".csv")
            files.push_back(entry.path());
    std::sort(files.begin(), files.end());

    int used = 0;
    std::vector<Point2D> trajectory;
    for (const auto& file : files) {
        if (g_max_files > 0 && used >= g_max_files) break;
        if (!LoadCanonicalCsv(file, trajectory)) continue;
        RunAllMethods(trajectory, records);
        ++used;
        if (used % 500 == 0) std::cout << "    processed " << used << " trajectories\n";
    }
    if (used == 0)
        throw std::runtime_error("no valid CSV trajectories in " + dir.string());
    std::cout << "  " << dir.filename().string() << " done: " << used << " trajectories\n";
}

bool LoadSimpleText(const fs::path& path, std::vector<Point2D>& trajectory) {
    trajectory.clear();
    std::ifstream input(path);
    std::string line;
    uint64_t base = 0;
    bool has_base = false;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        auto fields = SplitCsvLine(line);
        if (fields.size() < 3) continue;
        try {
            int lon_col = 1, lat_col = 2, time_col = 0;
            if (fields.size() >= 4) { time_col = 1; lon_col = 2; lat_col = 3; }
            auto stamp = fields[time_col];
            if (stamp.size() > 10 && stamp[10] == 'T') stamp[10] = ' ';
            const uint64_t seconds = ParseDateTime(stamp);
            if (!has_base) { base = seconds; has_base = true; }
            uint64_t tick = seconds >= base ? (seconds - base) * 10ULL : 0ULL;
            if (!trajectory.empty() && tick <= trajectory.back().timestamp)
                tick = trajectory.back().timestamp + 1;
            trajectory.push_back({std::stod(fields[lon_col]),
                                  std::stod(fields[lat_col]), tick});
        } catch (...) {
            // Header and malformed rows are skipped.
        }
    }
    return trajectory.size() >= g_min_points;
}

bool LoadExperimentTrajectory(const fs::path& path, std::vector<Point2D>& trajectory) {
    if (path.extension() == ".plt") return LoadGeolife(path, trajectory);
    if (path.extension() == ".csv") return LoadCanonicalCsv(path, trajectory);
    if (path.extension() == ".txt") return LoadSimpleText(path, trajectory);
    return false;
}

void ApplySamplingInterval(std::vector<Point2D>& trajectory, int interval_seconds) {
    if (interval_seconds <= 0 || trajectory.size() < 2) return;
    const uint64_t interval = static_cast<uint64_t>(interval_seconds) * 10ULL;
    std::vector<Point2D> sampled;
    sampled.reserve(trajectory.size());
    sampled.push_back(trajectory.front());
    uint64_t last = trajectory.front().timestamp;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        if (trajectory[i].timestamp >= last && trajectory[i].timestamp - last >= interval) {
            sampled.push_back(trajectory[i]);
            last = trajectory[i].timestamp;
        }
    }
    trajectory.swap(sampled);
}

// ─── Dataset processors ───

void ProcessNormalized(std::vector<PerfRecord>& rec, const fs::path& dir) {
    std::vector<Point2D> traj; traj.reserve(4096);
    int n = 0;
    for (int i = 1; i <= kNormalizedFileMax; ++i) {
        if (g_normalized_max > 0 && n >= g_normalized_max) break;
        if (!LoadNormalized(dir, i, traj)) continue;
        RunAllMethods(traj, rec); n++;
        if (n % 500 == 0) std::cout << "    normalized: " << n << "\n";
    }
    std::cout << "  normalized done: " << n << "\n";
}

void ProcessGeolife(std::vector<PerfRecord>& rec, const fs::path& dir) {
    auto files = ListFiles(dir, ".plt");
    int n = 0;
    for (auto& f : files) {
        std::vector<Point2D> traj;
        if (!LoadGeolife(f, traj)) continue;
        RunAllMethods(traj, rec); n++;
        if (n % 50 == 0) std::cout << "    geolife: " << n << "\n";
    }
    std::cout << "  geolife done: " << n << "\n";
}

void ProcessWorldTrace(std::vector<PerfRecord>& rec, const fs::path& dir) {
    auto files = ListFiles(dir, ".csv");
    int n = 0;
    for (auto& f : files) {
        std::vector<Point2D> traj;
        if (!LoadWorldTrace(f, traj)) continue;
        RunAllMethods(traj, rec); n++;
    }
    std::cout << "  WorldTrace done: " << n << "\n";
}

void ProcessTDrive(std::vector<PerfRecord>& rec, const fs::path& dir) {
    std::vector<Point2D> traj; traj.reserve(4096);
    int n = 0;
    for (int i = 1; i <= kTDriveFileMax; ++i) {
        if (g_tdrive_max > 0 && n >= g_tdrive_max) break;
        if (!LoadTDrive(dir, i, traj)) continue;
        RunAllMethods(traj, rec); n++;
        if (n % 500 == 0) std::cout << "    T-Drive: " << n << "\n";
    }
    std::cout << "  T-Drive done: " << n << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    // Parse args
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a.rfind("--eps=", 0) == 0) {
            g_max_diff = std::stod(a.substr(6));
            g_trad_eps_deg = g_max_diff;
            g_trad_eps_m = g_max_diff * 111000.0;
        }
        else if (a.rfind("--max-normalized=", 0) == 0) g_normalized_max = std::stoi(a.substr(17));
        else if (a.rfind("--max-tdrive=", 0) == 0)  g_tdrive_max  = std::stoi(a.substr(13));
        else if (a.rfind("--data-root=", 0) == 0) g_data_root = a.substr(12);
        else if (a.rfind("--output=", 0) == 0) g_output_path = a.substr(9);
        else if (a.rfind("--max-files=", 0) == 0) g_max_files = std::stoi(a.substr(12));
        else if (a.rfind("--min-points=", 0) == 0) g_min_points = std::stoi(a.substr(13));
        else if (a.rfind("--sampling-interval=", 0) == 0) g_sampling_interval = std::stoi(a.substr(20));
        else if (a.rfind("--dataset-label=", 0) == 0) g_dataset_label = a.substr(16);
        else if (a.rfind("--only-method=", 0) == 0) g_only_method = a.substr(14);
        else if (a.rfind("--methods=", 0) == 0) {
            g_methods.clear();
            std::stringstream names(a.substr(10));
            std::string name;
            while (std::getline(names, name, ',')) if (!name.empty()) g_methods.push_back(name);
        }
        else if (a.rfind("--datasets=", 0) == 0) {
            g_datasets.clear();
            std::stringstream names(a.substr(11));
            std::string name;
            while (std::getline(names, name, ',')) if (!name.empty()) g_datasets.push_back(name);
        }
        else if (a == "--help" || a == "-h") {
            std::cout << "Usage: e1_overall_benchmark [--data-root=DIR] [--datasets=A,B,...] "
                         "[--output=FILE] [--eps=VALUE] [--methods=A,B,...] "
                         "[--max-files=N] [--min-points=N] "
                         "[--sampling-interval=SECONDS] [--dataset-label=NAME]\n";
            return 0;
        }
    }

    if (!g_only_method.empty() &&
        std::find(std::begin(kMethodNames), std::end(kMethodNames), g_only_method) == std::end(kMethodNames)) {
        std::cerr << "Unknown method: " << g_only_method << '\n';
        return 2;
    }
    if (!g_only_method.empty() && !g_methods.empty()) {
        std::cerr << "Use either --only-method or --methods, not both\n";
        return 2;
    }
    for (const auto& method : g_methods) {
        if (std::find(std::begin(kMethodNames), std::end(kMethodNames), method) == std::end(kMethodNames)) {
            std::cerr << "Unknown method: " << method << '\n';
            return 2;
        }
    }

    std::cout << "====================================================================\n"
              << "E1: Overall Benchmark — 12 methods on normalized trajectories\n"
              << "Epsilon = " << std::scientific << g_max_diff << "\n"
              << "====================================================================\n\n";

    if (g_min_points < 1 || g_max_files < 0 || g_datasets.empty()) {
        std::cerr << "Invalid dataset or point-count option\n";
        return 2;
    }
    std::vector<std::vector<PerfRecord>> per_dataset;
    for (const auto& dataset : g_datasets) {
        const fs::path directory = g_data_root / dataset;
        if (!fs::is_directory(directory)) {
            std::cerr << "Missing dataset directory: " << directory << '\n';
            return 1;
        }
        const std::string result_name = g_dataset_label.empty() ? dataset : g_dataset_label;
        auto records = InitRecords(result_name);
        std::vector<fs::path> files;
        for (const auto& entry : fs::recursive_directory_iterator(directory))
            if (entry.is_regular_file()) files.push_back(entry.path());
        std::sort(files.begin(), files.end());
        int used = 0;
        for (const auto& file : files) {
            if (g_max_files > 0 && used >= g_max_files) break;
            std::vector<Point2D> trajectory;
            if (!LoadExperimentTrajectory(file, trajectory)) continue;
            ApplySamplingInterval(trajectory, g_sampling_interval);
            if (trajectory.size() < static_cast<size_t>(g_min_points)) continue;
            RunAllMethods(trajectory, records);
            ++used;
            if (used % 500 == 0) std::cout << "  " << dataset << ": " << used << " trajectories\n";
        }
        if (used == 0) {
            std::cerr << "No usable trajectories in " << directory << '\n';
            return 1;
        }
        std::cout << dataset << ": " << used << " trajectories\n";
        per_dataset.push_back(std::move(records));
    }

    if (!g_output_path.parent_path().empty())
        fs::create_directories(g_output_path.parent_path());
    std::ofstream csv(g_output_path);
    if (!csv.is_open()) { std::cerr << "Cannot open " << g_output_path << '\n'; return 1; }
    csv << "Method,DataSet,TrajectoryCount,PointCount,BlockSize,MaxDiff,"
           "CompressionRatio,BPP,CompressionTime(us/point),DecompressionTime(us/point),"
           "CompressedBits,MaxRoundtripError\n";

    for (int mi = 0; mi < kMethodCount; ++mi) {
        if (!g_methods.empty()) {
            if (std::find(g_methods.begin(), g_methods.end(), kMethodNames[mi]) == g_methods.end()) continue;
        } else if (!g_only_method.empty() && g_only_method != kMethodNames[mi]) continue;
        std::vector<PerfRecord> rows;
        for (auto& ds : per_dataset) rows.push_back(ds[mi]);
        for (auto& r : rows) WriteCSVRow(csv, r);
        auto total = SumRecords(kMethodNames[mi], rows);
        WriteCSVRow(csv, total);
        std::cout << "  TOTAL [" << kMethodNames[mi] << "]: "; PrintRecord(total);
    }

    std::cout << "\nResults: " << g_output_path << '\n';
    return 0;
}
