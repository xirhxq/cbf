#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_DIR="${REPO_ROOT}/scripts/formation_reconfiguration"
PYTHON_BIN="${PYTHON_BIN:-python3}"

SIM_DIR="${REPO_ROOT}/outputs/sim"
GEO_DIR="${REPO_ROOT}/data/geo_outputs"
OUT_DIR="${REPO_ROOT}/outputs/figures"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim-dir)
      SIM_DIR="$2"
      shift 2
      ;;
    --geo-dir)
      GEO_DIR="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "Usage: $0 [--sim-dir DIR] [--geo-dir DIR] [--out-dir DIR]" >&2
      exit 2
      ;;
  esac
done

mkdir -p "${OUT_DIR}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/route_b_plot_data_closed_loop_tdd5.py" \
  --sim-dir "${SIM_DIR}" --geo-dir "${GEO_DIR}" --out-dir "${OUT_DIR}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/route_b_plot_data_mechanism_tdd8_10.py" \
  --sim-dir "${SIM_DIR}" --geo-dir "${GEO_DIR}" --out-dir "${OUT_DIR}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/route_b_plot_data_mission_snapshots.py" \
  --sim-dir "${SIM_DIR}" --out-dir "${OUT_DIR}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/route_b_plot_data_two_regime_tdd6.py" \
  --sim-dir "${SIM_DIR}" --geo-dir "${GEO_DIR}" --out-dir "${OUT_DIR}"
