# CBF2026 Numerical Experiments (`ugdms-cbf`)

Numerical multi-UAV coverage experiments with control barrier functions, range-only EKF + Split Covariance Intersection (SCI) fusion, finite-range active-set ranging (`d_range <= 1200 m`), and finite-range comparator baselines (Oruganti, Cano, Goudar, and Artificial-Reference Tube DMPC). The Gazebo/ROS 2 closed-loop simulation package is maintained separately in `xirhxq/cbf-ros2` on branch `ugdms-cbf` and consumes this repository as its `cbf` submodule.

## Requirements

- C++17 toolchain, CMake >= 3.16, Boost headers, and Eigen >= 3.3 (tested with Eigen 3.4). Header-only **PIQP 0.6.1** (`BSD-2-Clause`) and **nlohmann/json** (`MIT`) are bundled under `vendor/`.
- Python 3.8+ with NumPy (`requirements.txt`). Optional finite-range comparators additionally require SciPy and Clarabel (`requirements-comparators.txt`).
- POSIX environment with named pipes (`zstd` optional for streaming compression; falls back to `gzip` if `zstd` is absent).
- Optional: Gurobi C++/C solver only if passing `-DENABLE_GUROBI=ON` (and `-DBUILD_HISTORICAL_VARIANTS=ON` for Round-1 legacy comparison snapshots).

## Build (Default PIQP Header-Only Backend)

```sh
cmake -S . -B build
cmake --build build -j4
```

If Eigen lives in a nonstandard prefix (e.g. multiple Eigen versions installed via Homebrew), pass `-DEIGEN_INCLUDE_DIR=/path/to/eigen3`. The default build produces `build/cbf2026_native`, which supports `n=10`, `n=14`, and `n=21` multi-squad configurations, all convex scene geometries, range-perturbation schedules, and the external comparator IPC interface.

## Run 1200 m Finite-Range SCI & Ablation Cases

`scripts/run_hard_cutoff.py` runs the proposed SCI estimator-in-the-loop controller and all finite-range sensitivity/perturbation ablations (`--duration 700.0` for full 700 s runs, or a short smoke duration such as `--duration 0.5`):

```sh
python3 scripts/run_hard_cutoff.py --case main14 --binary build/cbf2026_native --output /tmp/cbf_main14 --duration 0.5
```

Available `--case` values:
- Core swarm & scenes: `main14`, `n10`, `squad21`, `square2000`, `square2500`, `triangle`, `trapezoid`
- Margin & fusion ablations: `fixed1m`, `fixed3m`, `fixed9m`, `cr3`
- Ranging perturbations (1200 m active set): `bias`, `burst`, `dispersed20`, `dispersed40`, `dispersed100`

## Run 1200 m Finite-Range Comparator Baselines

`scripts/comparators/run_finite_comparator.py` runs the finite-range (`d <= 1200 m`, common EKF + SCI estimator) comparator baselines (`oruganti`, `cano`, `goudar`, `mpc`):

```sh
pip install -r requirements-comparators.txt
python3 scripts/comparators/run_finite_comparator.py --method mpc --binary build/cbf2026_native --output /tmp/cbf_mpc --duration 0.5
```

## Historical Round-1 Configuration Archive

Sanitized Round-1 JSON cases (`cases/`, `range-cases/`, `config/`) and source drivers (`scripts/run_case.py`, `scripts/run_range_error.py`, `variants/`) are retained for provenance alongside the updated 1200 m PIQP + SCI implementation above. Historical Round-1 frozen random-field digests are documented in `scripts/run_case.py`.

## License

Project source code is licensed under the MIT License (`LICENSE`). Bundled third-party headers carry their own licenses (`vendor/PIQP-LICENSE`, `vendor/include/nlohmann/json.hpp`).
