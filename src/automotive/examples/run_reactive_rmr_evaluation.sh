#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export METHODS=reactive-rmr
export RMR_CBR_LOW=${RMR_CBR_LOW:-0.33}
export RMR_CBR_HIGH=${RMR_CBR_HIGH:-0.67}
export OUT_DIR=${OUT_DIR:-results/reactive_rmr_$(date +%Y%m%d_%H%M%S)}

exec "$SCRIPT_DIR/run_hybrid_step5_evaluation.sh"
