#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/../build/e5_tau}"
mkdir -p "$BUILD_DIR"

g++ -O3 -std=c++17 -I"$REPO_ROOT" \
    "$SCRIPT_DIR/acost_tau_berlin_intervals.cc" \
    "$REPO_ROOT/algorithm/acost_compressor.cc" \
    "$REPO_ROOT/algorithm/cost_compressor.cc" \
    "$REPO_ROOT/utils/output_bit_stream.cc" \
    "$REPO_ROOT/utils/input_bit_stream.cc" \
    "$REPO_ROOT/utils/elias_gamma_codec.cc" \
    "$REPO_ROOT/utils/elias_delta_codec.cc" \
    "$REPO_ROOT/utils/post_office_solver.cc" \
    -o "$BUILD_DIR/e5_acost_tau"

echo "Built: $BUILD_DIR/e5_acost_tau"
