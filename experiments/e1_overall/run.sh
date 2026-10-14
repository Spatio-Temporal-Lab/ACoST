#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/../build/e1_overall}"
EXPERIMENT_ROOT="$(cd -- "$REPO_ROOT/.." && pwd)"

"$SCRIPT_DIR/build.sh"
cd "$SCRIPT_DIR"
exec "$BUILD_DIR/e1_overall_benchmark" \
    "--data-root=${DATA_ROOT:-$REPO_ROOT/data}" \
    "--output=${OUTPUT:-$EXPERIMENT_ROOT/results/e1_results.csv}" \
    "$@"
