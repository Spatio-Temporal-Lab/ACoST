#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/../build/e3_epsilon}"
mkdir -p "$BUILD_DIR"
g++ -O3 -std=c++17 -I"$REPO_ROOT" -I"$REPO_ROOT/baselines/elf" \
    "$SCRIPT_DIR/e3_epsilon_benchmark.cc" \
    "$REPO_ROOT/algorithm/acost_compressor.cc" "$REPO_ROOT/algorithm/cost_compressor.cc" \
    "$REPO_ROOT/baselines/serf/serf_qt_compressor.cc" "$REPO_ROOT/baselines/serf/serf_qt_decompressor.cc" \
    "$REPO_ROOT/baselines/serf/serf_xor_compressor.cc" "$REPO_ROOT/baselines/serf/serf_xor_decompressor.cc" \
    "$REPO_ROOT/baselines/gorilla/gorilla_compressor.cc" "$REPO_ROOT/baselines/gorilla/gorilla_decompressor.cc" \
    "$REPO_ROOT/baselines/elf/ElfCompressor.cpp" "$REPO_ROOT/baselines/elf/ElfDecompressor.cpp" \
    "$REPO_ROOT/baselines/elf/utils.cc" "$REPO_ROOT/baselines/lz77/fastlz.c" \
    "$REPO_ROOT/utils/output_bit_stream.cc" "$REPO_ROOT/utils/input_bit_stream.cc" \
    "$REPO_ROOT/utils/elias_gamma_codec.cc" "$REPO_ROOT/utils/elias_delta_codec.cc" \
    "$REPO_ROOT/utils/serf_utils_64.cc" "$REPO_ROOT/utils/post_office_solver.cc" \
    -o "$BUILD_DIR/e3_epsilon_benchmark"
echo "Built: $BUILD_DIR/e3_epsilon_benchmark"
