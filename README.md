# CBF-Based Multi-UAV Search

## Introduction

This repository implements a multi-UAV search problem using a **Control Barrier Function (CBF)**-based approach. CBFs are a powerful mathematical tool used to ensure safety in dynamical systems by encoding safety constraints directly into the control laws of robots. For more information on CBFs, you can refer to the foundational paper: [*Control Barrier Functions: Theory and Applications*](https://arxiv.org/abs/1512.07873).

The algorithm is implemented in C++, and the results are stored in `.json` files, which can be visualized using a Python script.

We recommend using **CLion** for development, as it provides an intuitive interface for managing CMake projects and supports Python scripting for result visualization.

## Installation

### Solver Configuration

This project supports three QP solvers for the CBF optimization:

| Solver | Type | Speed | Stability | License |
|--------|------|-------|-----------|---------|
| **OSQP** | Open-source | Fastest | Excellent | Apache 2.0 |
| **Gurobi** | Commercial | Fast | Excellent | Commercial |
| **HiGHS** | Open-source | Moderate | Good | MIT |

**Recommendation**: Use **OSQP** for best performance and numerical stability with CBF constraints.

#### Enabling Solvers

Each solver has a corresponding CMake option:

```bash
cmake -DENABLE_OSQP=ON -DENABLE_GUROBI=ON -DENABLE_HIGHS=ON ..
```

#### Installing OSQP

Install [OsqpEigen](https://github.com/robotology/osqp-eigen):

```bash
# macOS
brew install osqp-eigen

# Ubuntu
sudo apt install libosqp-eigen-dev
```

#### Installing Gurobi

Download from [Gurobi website](https://www.gurobi.com/downloads/) and specify the installation path:

```bash
cmake -DGUROBI_DIR=/Library/gurobi1200/macos_universal2 ..
```

The bundled `FindGUROBI.cmake` reads the CMake variable `GUROBI_DIR` or the
environment variable `GUROBI_HOME`. It does not read `GUROBI_ROOT`.

#### Installing HiGHS

```bash
# macOS
brew install highs

# Ubuntu
sudo apt install libhighs-dev
```

#### Selecting Solver

Set the `optimiser` field in `config/config.json`:

```json
"optimiser": "OSQP"
```

#### GrandFinale experiment profiles

The general solver recommendation above is not an experiment identity.
GrandFinale experiments freeze their solver profile in the parent repository's
preregistration and evidence. Configure a Gurobi-frozen GrandFinale build with:

```bash
cmake -S . -B build-grand-finale \
  -DCMAKE_BUILD_TYPE=Release \
  -DGRAND_FINALE_SOLVER_PROFILE=gurobi \
  -DGUROBI_DIR=/Library/gurobi1200/macos_universal2
```

Use `GRAND_FINALE_SOLVER_PROFILE=open_source` only when the corresponding
protocol explicitly freezes the HiGHS+OSQP profile. Do not infer a scientific
profile from this generic README. Task-specific input paths, commands, binary
hashes, expected exit codes, and run authorization live beside the evidence in
the GrandFinale parent repository; untracked `build-*` directories are not
portable or authoritative artifacts.

#### Common Issues
- **How to ensure CMake finds the correct shared libraries (`.so` or `.dylib`)?**
  Please verify that the shared libraries (either `.so` or `.dylib` files) found by CMake match the required version for your platform. File names and paths may differ across platforms, so be sure to confirm that the versions are correct based on your system and the installed solver version.


### Python Environment Setup

#### 1. **Install Anaconda or Miniconda**
Download [Anaconda](https://www.anaconda.com/download) (recommended for data science) or [Miniconda](https://docs.conda.io/en/latest/miniconda.html) (minimal installer), then follow default installation steps.

#### 2. **Create Python 3.11 Environment**
```bash
conda create -n cbf_env python=3.11
conda activate cbf_env
```  
This creates an isolated environment with Python 3.11.

#### 3. **Install FFmpeg**
```bash
conda install -c conda-forge ffmpeg  # Handles codec dependencies automatically
```

#### 4. **Install Project Dependencies**
```bash
pip install -r requirements.txt  # Use --force-reinstall if conflicts occur
```

#### 5. **CLion Integration**
- Open **File > Settings > Build > Python Interpreter**.
- Click ⚙️ → **Add** → **Conda Environment** → Specify path:
   - Unix: `conda_install_path/envs/cbf_env/bin/python`
   - Windows: `...\envs\cbf_env\python.exe`.

This streamlined workflow ensures dependency isolation and cross-platform compatibility. For advanced troubleshooting, refer to the original Chinese version.
