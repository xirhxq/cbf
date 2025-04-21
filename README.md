# CBF-Based Multi-UAV Search

## Introduction

This repository implements a multi-UAV search problem using a **Control Barrier Function (CBF)**-based approach. CBFs are a powerful mathematical tool used to ensure safety in dynamical systems by encoding safety constraints directly into the control laws of robots. For more information on CBFs, you can refer to the foundational paper: [*Control Barrier Functions: Theory and Applications*](https://arxiv.org/abs/1512.07873).

The algorithm is implemented in C++, and the results are stored in `.json` files, which can be visualized using a Python script.

We recommend using **CLion** for development, as it provides an intuitive interface for managing CMake projects and supports Python scripting for result visualization.

## Installation

### Solver Configuration Guide

#### Supported Solvers
This project supports two optimization solvers: **Gurobi** and **HiGHS**. Both solvers need to be installed separately by the user, and you can choose to enable or disable them based on your installation.

#### Enabling Solvers
Each solver has a corresponding `ENABLE_*` option in the CMake configuration. You can enable or disable them depending on the solver you have installed. For example:
- If **Gurobi** is installed, set `ENABLE_GUROBI` to `ON` to enable Gurobi.
- If **HiGHS** is installed, set `ENABLE_HIGHS` to `ON` to enable HiGHS.

#### Installing Gurobi
If you choose to use **Gurobi** as the solver, refer to the official Gurobi installation guide:  
[Gurobi Installation Guide](https://support.gurobi.com/hc/en-us/articles/4534161999889-How-do-I-install-Gurobi-Optimizer)

After installing Gurobi, ensure to specify the Gurobi installation path using the `-DGUROBI_HOME` option during the CMake configuration:
```bash
cmake -DGUROBI_HOME=/opt/gurobi1200/linux64 ..
```

It is recommended to use **CLion** for configuring and building the project, as it simplifies the setup process. If you need to set environment variables when using CMake in CLion, you can refer to the [CLion Environment Variables Guide](https://www.jetbrains.com/help/clion/cmake-profile.html#EnvVariables).

#### Installing HiGHS
If you choose to use **HiGHS** as the solver, refer to the official HiGHS installation guide:  
[HiGHS Installation Guide](https://github.com/ERGO-Code/HiGHS/tree/master/cmake#build)

Once HiGHS is installed, CMake will automatically find and link the HiGHS library.

#### CMake Configuration and Solver Integration
The integration of Gurobi and HiGHS with CMake is based on their official documentation. Below are the relevant guides for integrating them:
- [HiGHS Integration Guide for CMake Projects](https://github.com/ERGO-Code/HiGHS/tree/master/cmake#integrating-highs-in-your-cmake-project)
- [Gurobi CMake Project Guide](https://support.gurobi.com/hc/en-us/articles/360039499751-How-do-I-use-CMake-to-build-Gurobi-C-C-projects)

These documents provide detailed integration methods and configuration options to help you correctly link and use both solvers in your project.

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