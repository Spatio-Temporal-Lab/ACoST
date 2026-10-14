#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/../build/e2_ablation}"
mkdir -p "$BUILD_DIR"

UTILS=(
    "$REPO_ROOT/utils/output_bit_stream.cc"
    "$REPO_ROOT/utils/input_bit_stream.cc"
    "$REPO_ROOT/utils/elias_gamma_codec.cc"
    "$REPO_ROOT/utils/elias_delta_codec.cc"
    "$REPO_ROOT/utils/post_office_solver.cc"
)
COMMON=(
    "$REPO_ROOT/algorithm/acost_compressor.cc"
    "$REPO_ROOT/algorithm/cost_compressor.cc"
    "${UTILS[@]}"
)

g++ -O3 -std=c++17 -I"$REPO_ROOT" -DCOST_ACOST_FAMILY \
    "$SCRIPT_DIR/cost_acost_family_benchmark.cc" \
    "$SCRIPT_DIR/cost_acost_family_compressor.cc" "${COMMON[@]}" \
    -o "$BUILD_DIR/e2_family_benchmark"

g++ -O3 -std=c++17 -I"$REPO_ROOT" -DCOST_ACOST_PREDICTOR \
    "$SCRIPT_DIR/cost_acost_predictor_benchmark.cc" \
    "$SCRIPT_DIR/cost_acost_predictor_compressor.cc" "${UTILS[@]}" \
    -o "$BUILD_DIR/e2_predictor_benchmark"

g++ -O3 -std=c++17 -I"$REPO_ROOT" \
    "$SCRIPT_DIR/common_ton_sensitivity_benchmark.cc" \
    "$REPO_ROOT/algorithm/acost_compressor.cc" "${UTILS[@]}" \
    -o "$BUILD_DIR/e2_common_ton_benchmark"

printf 'E2 benchmarks built in %s\n' "$BUILD_DIR"
