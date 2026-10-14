#define main e1_benchmark_main
#include "../e1_overall/e1_overall_benchmark.cc"
#undef main

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    namespace fs = std::filesystem;
    const std::array<int, 6> intervals = {0, 5, 10, 20, 40, 80};
    const std::array<const char*, 5> epsilons = {"1e-5", "5e-5", "1e-4", "5e-4", "1e-3"};
    fs::path output = "../../results/e4_berlin_sampling_results.csv";
    std::vector<std::string> common;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: e4_sampling_benchmark [--data-root=DIR] [--max-files=N] [--output=CSV]\n";
            return 0;
        }
        if (arg.rfind("--output=", 0) == 0) output = arg.substr(9);
        else if (arg.rfind("--eps=", 0) == 0 || arg.rfind("--sampling-interval=", 0) == 0 ||
                 arg.rfind("--methods=", 0) == 0 || arg.rfind("--datasets=", 0) == 0 ||
                 arg.rfind("--dataset-label=", 0) == 0) continue;
        else common.push_back(std::move(arg));
    }
    try {
        if (!output.parent_path().empty()) fs::create_directories(output.parent_path());
        std::ofstream merged(output);
        if (!merged) throw std::runtime_error("cannot create output: " + output.string());
        bool header_written = false;
        std::vector<fs::path> temporary;
        for (int interval : intervals) {
            const std::string label = interval == 0 ? "Berlin" : "Berlin_" + std::to_string(interval) + "s";
            for (const char* epsilon : epsilons) {
                fs::path part = output;
                part += ".e4-part-" + std::to_string(interval) + "-" + epsilon;
                temporary.push_back(part);
                std::vector<std::string> args = {"e1", "--datasets=Berlin", "--methods=serf_qt,serf_xor,cost,acost",
                    "--dataset-label=" + label, "--sampling-interval=" + std::to_string(interval),
                    "--eps=" + std::string(epsilon), "--min-points=10", "--output=" + part.string()};
                args.insert(args.end(), common.begin(), common.end());
                std::vector<char*> ptrs;
                for (auto& s : args) ptrs.push_back(s.data());
                ptrs.push_back(nullptr);
                const int status = e1_benchmark_main(static_cast<int>(args.size()), ptrs.data());
                if (status != 0) throw std::runtime_error("E1 benchmark failed for " + label + " at epsilon " + epsilon);
                std::ifstream input(part);
                std::string line;
                if (!std::getline(input, line)) throw std::runtime_error("empty E1 output: " + part.string());
                if (!header_written) { merged << "SamplingInterval,Epsilon," << line << '\n'; header_written = true; }
                while (std::getline(input, line)) merged << (interval == 0 ? "original" : std::to_string(interval) + "s")
                    << ',' << epsilon << ',' << line << '\n';
                input.close();
                fs::remove(part);
                temporary.pop_back();
            }
        }
        std::cout << "Results: " << output << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "E4 error: " << e.what() << '\n';
        return 1;
    }
}
