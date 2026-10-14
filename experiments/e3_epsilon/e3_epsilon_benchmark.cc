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
    const std::array<const char*, 8> epsilons = {
        "1e-6", "5e-6", "1e-5", "5e-5", "1e-4", "2e-4", "5e-4", "1e-3"};
    fs::path output = "../../results/e3_epsilon_results.csv";
    std::vector<std::string> common;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: e3_epsilon_benchmark [E1 dataset/method options] [--output=CSV]\n";
            return 0;
        }
        if (arg.rfind("--output=", 0) == 0) output = arg.substr(9);
        else if (arg.rfind("--eps=", 0) != 0) common.push_back(std::move(arg));
    }
    try {
        if (!output.parent_path().empty()) fs::create_directories(output.parent_path());
        std::ofstream merged(output);
        if (!merged) throw std::runtime_error("cannot create output: " + output.string());
        bool header_written = false;
        std::vector<fs::path> temporary;
        for (size_t i = 0; i < epsilons.size(); ++i) {
            fs::path part = output;
            part += ".e3-part-" + std::to_string(i);
            temporary.push_back(part);
            std::vector<std::string> args = {"e1", "--eps=" + std::string(epsilons[i]),
                                             "--output=" + part.string()};
            args.insert(args.end(), common.begin(), common.end());
            std::vector<char*> ptrs;
            for (auto& s : args) ptrs.push_back(s.data());
            ptrs.push_back(nullptr);
            const int status = e1_benchmark_main(static_cast<int>(args.size()), ptrs.data());
            if (status != 0) throw std::runtime_error("E1 benchmark failed at epsilon " + std::string(epsilons[i]));
            std::ifstream input(part);
            std::string line;
            if (!std::getline(input, line)) throw std::runtime_error("empty E1 output: " + part.string());
            if (!header_written) { merged << "Epsilon," << line << '\n'; header_written = true; }
            while (std::getline(input, line)) merged << epsilons[i] << ',' << line << '\n';
            input.close();
            fs::remove(part);
            temporary.pop_back();
        }
        std::cout << "Results: " << output << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "E3 error: " << e.what() << '\n';
        return 1;
    }
}
