# CBF2026 numerical experiments

Numerical multi-UAV coverage experiments with control barrier functions, a range-only EKF, and comparisons based on Cano-inspired follower planning and GdMS. The main 14-UAV configuration uses a 0.1 s control interval, alpha = 0.1, signed `q = h_lower`, and the 3σ uncertainty scale. The package also includes alternative scenes, fixed and adaptive margins, and ranging perturbations. The controller used by the Gazebo simulation is maintained in the `mbzirc-simulation` branch and is used by the accompanying `cbf-ros2` package.

## Requirements

- C++17, CMake, Boost headers, Eigen 3.4, nlohmann/json 3.11, and a Gurobi C++/C installation and license. Gurobi and the other dependencies are not bundled.
- Python 3.10+, NumPy (`requirements.txt`), `zstd` on `PATH`, and a POSIX environment with named pipes.

## Build

```sh
cmake -S . -B build -DGUROBI_HOME="$GUROBI_HOME"
cmake --build build -j 2

# Historical main-method variant that permits coverage stopping after 350 s
cmake -S . -B build-min350 -DGUROBI_HOME="$GUROBI_HOME" -DCBF_MINIMUM_350_STOP=ON
cmake --build build-min350 --target cbf2026_native -j 2
```

If Eigen and nlohmann/json share a nonstandard include root, provide `-DCBF_EXTERNAL_INCLUDE=/path/to/include`; `EIGEN_INCLUDE_DIR`, `NLOHMANN_INCLUDE_DIR`, and `BOOST_INCLUDE_DIR` can also be specified individually. Builds create `cbf2026_native` and the comparison/scene executables named `cbf2026_<variant>`. The `min350` variant uses `build-min350/cbf2026_native`; all other variants use the correspondingly named executable from the default build.

## Run a configured case

`config/case-index.json` identifies each scene, sensitivity condition and comparison along with its configuration file, seed, random-field type and build variant. Inspect available cases with:

```sh
python scripts/run_case.py --help
python scripts/run_range_error.py --help
```

Supply `--case`, the matching `--binary`, and a new `--output` directory. Case indexes specify the variant; the driver records the supplied binary hash but does not verify its variant. For example:

```sh
python scripts/run_case.py --case paired-main--3sigma-seed2026102905 --binary build/cbf2026_native --output /path/to/new-output
```

Some cases require a historical 700-frame random field supplied with `--field`. The normal field SHA-256 is `91a2a0da75e37efa217d6973879ec6545f49330030545aa510f772af19aea2fa`; the shared-base field SHA-256 is `296e44b1e3056ad01477f89762a596af7caa192d916b2f7261e11ed0759515cd`. These files are not included. Cases marked `counter` generate the measurement field from the configured seed and do not accept a frozen field.

The drivers create a fresh output directory, monitor free space and an output-size limit, and do not retry failed runs. Select suitable limits and review the driver for the target environment. The adapted drivers have not been validated through end-to-end reproduction of the recorded trajectories.

## Ranging perturbations

`config/range-error-index.json` contains the 22 no-bias, +0.30 m fixed-bias, +3 m burst and dispersed conditions. Burst perturbations apply to available ranges during [50,55) s of each 100 s block. Dispersed cases require a matching schedule via `--schedule`; scheduled anomalies apply only to available ranges. A planned count need not equal the count observed on a new trajectory.

Build a matching schedule from a completed burst estimator log using `python scripts/build_matched_schedule.py --help`. The saved burst logs and schedules are not included. The driver verifies the historical schedule digest or a canonical digest excluding only its reference-run location. The EKF gate remains unchanged; the perturbation driver records gate decisions for injected outliers.

`scripts/run_experiment.py`, `scripts/run_cano.py`, and `scripts/run_comparison.py` provide additional interfaces for representative cases.

## License

Project license: MIT; see `LICENSE`. Each external dependency is subject to its own license.
