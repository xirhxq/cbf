# Localization-Constrained Swarm Search with Formation Reconfiguration (`formation-reconfiguration`)

Reference C++17 simulation and figure-reproduction code for the paper:

> **Localization-Constrained Swarm Search with Formation Changes for GNSS-Denied Maritime Surveillance**

This branch packages the closed-loop 4D Interim-Master D-EKF, finite-range UWB observation gate, symmetric three-reference Make-Before-Break handover controller, second-order HOCBF-QP safety filter, analytical geometry datasets, and paper plotting scripts with **zero external C++ solver dependencies** using bundled header-only **PIQP 0.6.1** (`vendor/include/piqp`).

## Requirements

- C++17 compiler toolchain and CMake >= 3.16
- All required C++ dependencies are bundled in-tree:
  - **Eigen** (`external/Eigen`, MPL-2.0)
  - **PIQP 0.6.1** (`vendor/include/piqp`, `vendor/PIQP-LICENSE`, BSD-2-Clause)
  - **nlohmann/json** (`external/nlohmann/json.hpp`, MIT)
  - **doctest** (`external/doctest/doctest.h`, MIT)
- Python 3.10+ with `numpy`, `matplotlib`, and `scipy` (see `scripts/formation_reconfiguration/requirements.txt`) for regenerating figures from simulation JSON outputs.

## Build (Default Vendored Header-Only PIQP Backend)

The default CMake configuration enables vendored header-only `PIQP 0.6.1` (`-DENABLE_PIQP=ON`) and keeps optional external solvers disabled (`OFF`):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8 --target GrandFinaleRouteBRunner
```

Optional external QP/LP backends (`ENABLE_GUROBI`, `ENABLE_OSQP`, `ENABLE_HIGHS`, `ENABLE_PROXQP`) remain configurable via standard `-DENABLE_<SOLVER>=ON` CMake options for legacy development targets.

## Run Unit Tests and Closed-Loop Campaigns

`scripts/formation_reconfiguration/run_campaigns.sh` builds the pure-PIQP targets and executes the test suite or closed-loop simulation campaigns under `outputs/sim/`:

```sh
# Fast unit and closed-loop verification suite (21 targets, skips long parameter sweep)
./scripts/formation_reconfiguration/run_campaigns.sh tests

# Full verification suite including the 52-case break-even sweep (22 targets)
./scripts/formation_reconfiguration/run_campaigns.sh tests_full

# Reproduce individual campaigns (examples)
./scripts/formation_reconfiguration/run_campaigns.sh tdd5_pinball
./scripts/formation_reconfiguration/run_campaigns.sh tdd10

# Reproduce all paper campaigns and snapshots into outputs/sim/
./scripts/formation_reconfiguration/run_campaigns.sh all
```

Supported campaign modes: `tdd5_pinball`, `tdd5_multiscale`, `tdd6`, `tdd7` (4-arm parallel execution + merge), `tdd7_snapshots`, `tdd8`, `tdd9`, `tdd10`, and `all`.

## Reproduce Paper Data Figures

After installing Python dependencies (`pip install -r scripts/formation_reconfiguration/requirements.txt`) and generating `outputs/sim/*.json`, regenerate Figures 4–7 (`data_closed_loop_reconfig.pdf`, `data_mechanism_and_multiseed.pdf`, `data_mission_snapshots_tdd7.pdf`, `data_two_regime_coverage.pdf`) into `outputs/figures/`:

```sh
./scripts/formation_reconfiguration/plot_figures.sh
```

Custom paths can be supplied via `--sim-dir`, `--geo-dir`, and `--out-dir`.

## Repository Layout

- `include/grand_finale/` — Closed-loop controller, 4D D-EKF, HOCBF-QP formulation, handover certifier, and search tracker
- `include/optimisers/PIQP.hpp` — Vendored PIQP 0.6.1 adapter matching the repository `Optimiser` interface
- `src/GrandFinaleRouteBRunner.cpp` — Closed-loop campaign runner exporting JSON outputs
- `tests/` — Unit and closed-loop doctest targets (including `testRouteBPiqpSolver.cpp`)
- `data/geo_outputs/` — Precomputed analytical geometry and depth-scaling JSON datasets
- `scripts/formation_reconfiguration/` — Self-contained campaign runner (`run_campaigns.sh`), figure scripts (`plot_figures.sh`), and Python dependencies (`requirements.txt`)
- `vendor/` — Bundled header-only PIQP 0.6.1 headers and license (`vendor/PIQP-LICENSE`)
- `external/` — Bundled Eigen, nlohmann/json, and doctest headers

## Third-Party Licenses

Bundled third-party components carry their respective upstream licenses:
- PIQP 0.6.1 (`vendor/PIQP-LICENSE`, BSD 2-Clause)
- Eigen (`external/Eigen/`, MPL-2.0)
- nlohmann/json (`external/nlohmann/json.hpp`, MIT)
- doctest (`external/doctest/doctest.h`, MIT)
