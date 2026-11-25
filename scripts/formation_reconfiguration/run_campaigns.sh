#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"
OUT_DIR="${OUT_DIR:-${REPO_ROOT}/outputs/sim}"
MODE="${1:-all}"

mkdir -p "${OUT_DIR}"

cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_PIQP=ON \
  -DGRAND_FINALE_SOLVER_PROFILE=piqp

FAST_TEST_TARGETS=(
  testDistanceRangeAvailability
  testInterimMasterDekf
  testReferenceEligibility
  testCertifiedCoverageTracker
  testSearchPolygonDomain
  testCanonicalHardRows
  testCanonicalGammaStarFeedback
  testCanonicalHocbfQpController
  testTransitionCertifier
  testOpenSourceCheckpoint
  testRouteBCbf2026AndMinimalGating
  testRouteBPolarEntry
  testRouteBPipelinedTransitions
  testRouteBTwoRegimeMissions
  testRouteBMultiseedMonteCarlo
  testRouteBGeometryClosedLoopContrast
  testRouteBNonuniformPentagon
  testRouteBMissionSnapshots
  testRouteBCoverageAcceleration
  testRouteBYawControl
  testRouteBPiqpSolver
  testRouteBWave9Validations
  testRouteBWave10Validations
)

ALL_TEST_TARGETS=(
  "${FAST_TEST_TARGETS[@]}"
  testRouteBRegime2Breakeven
)

run_tests() {
  local -a targets=("$@")
  cmake --build "${BUILD_DIR}" -j8 --target "${targets[@]}"
  for t in "${targets[@]}"; do
    echo "=== Running ${t} ==="
    "${BUILD_DIR}/${t}"
  done
}

run_tdd7_parallel() {
  local -a pids=()
  for arm in nom_stay nom_switch deg_stay deg_switch; do
    "${BUILD_DIR}/GrandFinaleRouteBRunner" --output-dir "${OUT_DIR}" --mode tdd7_arm --arm "${arm}" &
    pids+=("$!")
  done
  for pid in "${pids[@]}"; do
    wait "${pid}"
  done
  "${BUILD_DIR}/GrandFinaleRouteBRunner" --output-dir "${OUT_DIR}" --mode tdd7_merge
}

case "${MODE}" in
  tests)
    run_tests "${FAST_TEST_TARGETS[@]}"
    ;;
  tests_full)
    run_tests "${ALL_TEST_TARGETS[@]}"
    ;;
  tdd7|tdd7_parallel)
    cmake --build "${BUILD_DIR}" -j8 --target GrandFinaleRouteBRunner
    run_tdd7_parallel
    ;;
  all)
    cmake --build "${BUILD_DIR}" -j8 --target GrandFinaleRouteBRunner
    for campaign in tdd5_pinball tdd5_multiscale tdd6 tdd8 tdd9 tdd10; do
      "${BUILD_DIR}/GrandFinaleRouteBRunner" --output-dir "${OUT_DIR}" --mode "${campaign}"
    done
    run_tdd7_parallel
    "${BUILD_DIR}/GrandFinaleRouteBRunner" --output-dir "${OUT_DIR}" --mode tdd7_snapshots
    "${BUILD_DIR}/GrandFinaleRouteBRunner" --output-dir "${OUT_DIR}" --mode wave9_validations
    "${BUILD_DIR}/GrandFinaleRouteBRunner" --output-dir "${OUT_DIR}" --mode wave10_validations
    ;;
  tdd5_pinball|tdd5_multiscale|tdd6|tdd8|tdd9|tdd10|tdd7_snapshots|wave9_validations|wave10_validations)
    cmake --build "${BUILD_DIR}" -j8 --target GrandFinaleRouteBRunner
    "${BUILD_DIR}/GrandFinaleRouteBRunner" --output-dir "${OUT_DIR}" --mode "${MODE}"
    ;;
  *)
    echo "Usage: $0 {tests|tests_full|all|tdd5_pinball|tdd5_multiscale|tdd6|tdd7|tdd7_parallel|tdd7_snapshots|tdd8|tdd9|tdd10|wave9_validations|wave10_validations}" >&2
    exit 2
    ;;
esac
