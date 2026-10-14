#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
EXPERIMENT_ROOT="$(cd -- "$REPO_ROOT/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/../build/e5_tau}"

"$SCRIPT_DIR/build.sh"
exec "$BUILD_DIR/e5_acost_tau" \
    "--data-root=${DATA_ROOT:-$REPO_ROOT/data}" \
    "--output=${OUTPUT:-$EXPERIMENT_ROOT/results/e5_berlin_tau_results.csv}" \
    "$@"
