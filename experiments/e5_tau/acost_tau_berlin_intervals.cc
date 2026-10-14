// E5: ACoST damping-parameter sensitivity on Berlin interval samples
//
// Tests six tau values on native and sampled Berlin trajectories (0, 5, 10, 20, 40, 80 s).
//
// MaxDiff = 1e-4 (matching the 32 sampling-sensitivity experiment).
//
// Results are written to the selected CSV path.
//
// Build and run with build.sh and run.sh in this directory.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "../../algorithm/acost_compressor.h"
#include "../../utils/benchmark_utils.h"

namespace {

namespace fs = std::filesystem;

double         g_max_diff             = 1.0e-4;
fs::path       g_data_root            = "../../data";
fs::path       g_output_path          = "../../results/e5_berlin_tau_results.csv";
int            g_max_files            = 0;
int            g_min_points           = 10;
constexpr int    kReportedBlockSize  = 10000;
constexpr int    kStreamingBlockSize = 10000;

struct Point2D {
    double   x;
    double   y;
    uint64_t timestamp;
};

struct RawPoint {
    double   x;
    double   y;
    uint64_t seconds;
};

struct PerfRecord {
    std::string method;
    std::string dataset;
    long long total_compressed_bits       = 0;
    long long total_compression_time_us   = 0;
    long long total_decompression_time_us = 0;
    long long total_points                = 0;
    long long total_trajectories          = 0;
    double    max_roundtrip_error         = 0.0;

    double CompressionRatioByPoints() const {
        long long original_bits = total_points * 128LL;
        return original_bits > 0 ? static_cast<double>(total_compressed_bits) / original_bits : 0.0;
    }
    double BitsPerPoint() const {
        return total_points > 0 ? static_cast<double>(total_compressed_bits) / total_points : 0.0;
    }
    double AvgCompressionTimePerPoint() const {
        return total_points > 0 ? static_cast<double>(total_compression_time_us) / total_points : 0.0;
    }
    double AvgDecompressionTimePerPoint() const {
        return total_points > 0 ? static_cast<double>(total_decompression_time_us) / total_points : 0.0;
    }
};

// ─── Tau configurations ───

struct TauConfig {
    const char* name;
    double      tau;  // internal units (0.1 real-second each)
};

static const TauConfig kTauConfigs[] = {
    { "acost_tau2s",   20.0   },
    { "acost_tau5s",   50.0   },
    { "acost_tau10s",  100.0  },
    { "acost_tau20s",  200.0  },   // current default
    { "acost_tau50s",  500.0  },
    { "acost_tau100s", 1000.0 },
};
constexpr int kTauCount = static_cast<int>(sizeof(kTauConfigs) / sizeof(kTauConfigs[0]));

// ─── Timestamp parsing ───

uint64_t ParseDateTimeSeconds(const std::string& s) {
    std::tm tm = {};
    std::istringstream ss(s.substr(0, 19));
    ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
    if (ss.fail()) throw std::runtime_error("failed to parse datetime: " + s);
    return static_cast<uint64_t>(std::mktime(&tm));
}

// ─── Error helper ───

template <typename PointT>
void UpdateMaxError(const std::vector<Point2D>& original,
                    const std::vector<PointT>& decoded,
                    PerfRecord& record) {
    if (decoded.size() != original.size())
        throw std::runtime_error("trajectory decompression size mismatch");
    for (size_t i = 0; i < original.size(); ++i) {
        double dx = original[i].x - decoded[i].longitude;
        double dy = original[i].y - decoded[i].latitude;
        record.max_roundtrip_error = std::max(record.max_roundtrip_error,
                                              std::sqrt(dx * dx + dy * dy));
    }
}

// ─── ACoST runner ───

void RunACoSTWithConfig(const std::vector<Point2D>& trajectory,
                          PerfRecord& record,
                          const ACoSTConfig& cfg) {
    if (trajectory.empty()) return;
    std::vector<ACoSTCompressor::GpsPoint> points;
    points.reserve(trajectory.size());
    for (const auto& p : trajectory) points.emplace_back(p.x, p.y, p.timestamp);

    ACoSTCompressor comp(kStreamingBlockSize, g_max_diff, cfg);
    benchmark_utils::DoNotOptimize(points.data());
    benchmark_utils::ClobberMemory();
    auto t0 = std::chrono::steady_clock::now();
    for (const auto& p : points) comp.AddGpsPoint(p);
    comp.Close();
    benchmark_utils::ClobberMemory();
    auto t1 = std::chrono::steady_clock::now();

    const auto& stats = comp.GetStats();
    long long bits = static_cast<long long>(stats.total_bits) - stats.timestamp_bits;
    Array<uint8_t> compressed = comp.GetCompressedData();

    benchmark_utils::ClobberMemory();
    auto t2 = std::chrono::steady_clock::now();
    ACoSTDecompressor decomp(&compressed[0], compressed.length(), cfg);
    std::vector<ACoSTCompressor::GpsPoint> decoded;
    decoded.reserve(points.size());
    ACoSTCompressor::GpsPoint pt;
    for (size_t i = 0; i < points.size(); ++i) {
        if (!decomp.ReadNextPoint(pt)) break;
        decoded.push_back(pt);
    }
    benchmark_utils::ClobberMemory();
    auto t3 = std::chrono::steady_clock::now();

    UpdateMaxError(trajectory, decoded, record);
    record.total_compressed_bits       += bits;
    record.total_compression_time_us   += std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    record.total_decompression_time_us += std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    record.total_points                += static_cast<long long>(trajectory.size());
    record.total_trajectories++;
}

// ─── Data loading ───

std::vector<fs::path> ListCsvFiles(const fs::path& dir) {
    std::vector<fs::path> paths;
    for (const auto& e : fs::directory_iterator(dir))
        if (e.is_regular_file() && e.path().extension() == ".csv")
            paths.push_back(e.path());
    std::sort(paths.begin(), paths.end());
    return paths;
}

bool BuildTrajectory(const std::vector<RawPoint>& raw, std::vector<Point2D>& traj, int interval_seconds) {
    traj.clear();
    if (raw.size() < static_cast<size_t>(g_min_points)) return false;
    uint64_t base = 0;
    bool has_base = false;
    traj.reserve(raw.size());
    for (const auto& rp : raw) {
        if (!has_base) { base = rp.seconds; has_base = true; }
        uint64_t ts = (rp.seconds >= base) ? (rp.seconds - base) * 10ULL : 0ULL;
        if (!traj.empty() && ts <= traj.back().timestamp)
            ts = traj.back().timestamp + 1;
        if (interval_seconds > 0 && !traj.empty() &&
            ts - traj.back().timestamp < static_cast<uint64_t>(interval_seconds) * 10ULL)
            continue;
        traj.push_back({rp.x, rp.y, ts});
    }
    return traj.size() >= static_cast<size_t>(g_min_points);
}

// Process one CSV file; accumulate into per-tau records for a given dataset slot.
// tau_records[tau_idx][ds_idx]
int ProcessBerlinFile(const fs::path& path,
                  std::vector<std::vector<PerfRecord>>& tau_records,
                  int ds_idx,
                  std::vector<Point2D>& traj,
                  int interval_seconds) {
    std::ifstream f(path);
    if (!f.is_open()) return 0;
    std::string line;
    std::getline(f, line);  // skip header

    std::vector<RawPoint> raw;
    raw.reserve(65536);

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        // columns: id, lon, lat, datetime, ...
        size_t c0 = line.find(',');
        if (c0 == std::string::npos) continue;
        size_t c1 = line.find(',', c0 + 1);
        if (c1 == std::string::npos) continue;
        size_t c2 = line.find(',', c1 + 1);
        if (c2 == std::string::npos) continue;
        size_t c3 = line.find(',', c2 + 1);
        try {
            double lon = std::stod(line.substr(c0 + 1, c1 - c0 - 1));
            double lat = std::stod(line.substr(c1 + 1, c2 - c1 - 1));
            std::string ts_str = (c3 != std::string::npos)
                ? line.substr(c2 + 1, c3 - c2 - 1)
                : line.substr(c2 + 1);
            if (!ts_str.empty() && ts_str.back() == '\r') ts_str.pop_back();
            uint64_t secs = ParseDateTimeSeconds(ts_str);
            raw.push_back({lon, lat, secs});
        } catch (...) {}
    }

    if (!BuildTrajectory(raw, traj, interval_seconds)) return 0;

    for (int ti = 0; ti < kTauCount; ++ti) {
        ACoSTConfig cfg;
        cfg.ctra_damp_tau = kTauConfigs[ti].tau;
        RunACoSTWithConfig(traj, tau_records[ti][ds_idx], cfg);
    }
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg.rfind("--data-root=", 0) == 0) g_data_root = arg.substr(12);
            else if (arg.rfind("--output=", 0) == 0) g_output_path = arg.substr(9);
            else if (arg.rfind("--max-files=", 0) == 0) g_max_files = std::stoi(arg.substr(12));
            else if (arg.rfind("--min-points=", 0) == 0) g_min_points = std::stoi(arg.substr(13));
            else if (arg.rfind("--eps=", 0) == 0) g_max_diff = std::stod(arg.substr(6));
            else if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: e5_acost_tau [--data-root=DIR] [--output=CSV] [--max-files=N] [--min-points=N] [--eps=VALUE]\n";
                return 0;
            } else throw std::runtime_error("unknown option: " + arg);
        }
        if (g_max_diff <= 0 || g_max_files < 0 || g_min_points < 1)
            throw std::runtime_error("invalid option value");

        const fs::path berlin_dir = g_data_root / "Berlin";
        if (!fs::is_directory(berlin_dir)) throw std::runtime_error("missing Berlin directory: " + berlin_dir.string());
        std::vector<fs::path> files = ListCsvFiles(berlin_dir);
        if (g_max_files > 0 && files.size() > static_cast<size_t>(g_max_files)) files.resize(g_max_files);
        const int intervals[] = {0, 5, 10, 20, 40, 80};
        const int dataset_count = static_cast<int>(std::size(intervals));

        // tau_records[tau_idx][dataset_idx]
        std::vector<std::vector<PerfRecord>> tau_records(
            kTauCount, std::vector<PerfRecord>(dataset_count));
        for (int ti = 0; ti < kTauCount; ++ti)
            for (int di = 0; di < dataset_count; ++di) {
                tau_records[ti][di].method  = kTauConfigs[ti].name;
                tau_records[ti][di].dataset = di == 0 ? "Berlin" : "Berlin_" + std::to_string(intervals[di]) + "s";
            }

        // ─── Process each dataset ───
        for (int di = 0; di < dataset_count; ++di) {
            std::cout << "Processing " << tau_records[0][di].dataset << " ...\n";
            std::vector<Point2D> traj;
            traj.reserve(65536);
            int processed = 0;

            for (const auto& p : files) {
                processed += ProcessBerlinFile(p, tau_records, di, traj, intervals[di]);
                if (processed % 100 == 0 && processed > 0)
                    std::cout << "    " << tau_records[0][di].dataset << ": " << processed << " trajectories...\n";
            }
            std::cout << "  " << tau_records[0][di].dataset << " done: " << processed << " trajectories\n";
        }

        // ─── Append to 32 CSV (no header) ───
        if (!g_output_path.parent_path().empty()) fs::create_directories(g_output_path.parent_path());
        std::ofstream csv(g_output_path);
        if (!csv.is_open()) {
            std::cerr << "Failed to open output: " << g_output_path << '\n';
            return 1;
        }
        csv << "Method,DataSet,TrajectoryCount,PointCount,BlockSize,MaxDiff,CompressionRatio,BPP,CompressionTime(us/point),DecompressionTime(us/point),CompressedBits,MaxRoundtripError\n";

        auto write_row = [&](const PerfRecord& r) {
            csv << r.method << ',' << r.dataset << ','
                << r.total_trajectories << ',' << r.total_points << ','
                << kReportedBlockSize << ',' << std::scientific << g_max_diff << ','
                << std::fixed << std::setprecision(6) << r.CompressionRatioByPoints() << ','
                << r.BitsPerPoint() << ','
                << r.AvgCompressionTimePerPoint() << ','
                << r.AvgDecompressionTimePerPoint() << ','
                << r.total_compressed_bits << ','
                << std::scientific << r.max_roundtrip_error << '\n';
        };

        // Write row-by-row: for each tau, all datasets in order
        for (int ti = 0; ti < kTauCount; ++ti)
            for (int di = 0; di < dataset_count; ++di)
                write_row(tau_records[ti][di]);

        std::cout << "Results: " << g_output_path << '\n';
        return 0;

    } catch (const std::exception& ex) {
        std::cerr << "ERROR: " << ex.what() << '\n';
        return 1;
    }
}
