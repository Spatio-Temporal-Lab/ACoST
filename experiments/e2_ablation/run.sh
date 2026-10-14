#!/usr/bin/env bash
set -euo pipefail

if (($# < 2)); then
    echo "Usage: $0 {family|predictor|common-ton} METHOD DATASET [EPSILON TAU T_ON] < binary_trajectory_stream" >&2
    exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/../build/e2_ablation}"
"$SCRIPT_DIR/build.sh"

experiment="$1"
shift
case "$experiment" in
    family)    exec "$BUILD_DIR/e2_family_benchmark" "$@" ;;
    predictor) exec "$BUILD_DIR/e2_predictor_benchmark" "$@" ;;
    common-ton) exec "$BUILD_DIR/e2_common_ton_benchmark" "$@" ;;
    *) echo "Unknown E2 experiment: $experiment" >&2; exit 2 ;;
esac
