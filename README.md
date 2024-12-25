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

This project requires Python 3.11. You can create a virtual environment and install the necessary Python dependencies from the `requirements.txt` file using either the command line or CLion.

#### Option 1: Create Virtual Environment via Command Line

1. **Create a Python 3.11 Virtual Environment:**
   If you don't have Python 3.11 installed, please install it first. You can create a virtual environment using the following command:
   ```bash
   python3.11 -m venv .venv
   ```

2. **Activate the Virtual Environment:**
   After creating the virtual environment, activate it by running the appropriate command for your operating system:
    - On Linux/macOS:
      ```bash
      source .venv/bin/activate
      ```
    - On Windows:
      ```bash
      .venv\Scripts\activate
      ```

3. **Install Dependencies:**
   Once the virtual environment is activated, install the required Python libraries using the following command:
   ```bash
   pip install -r requirements.txt
   ```

#### Option 2: Create Virtual Environment Using CLion

1. **Open the Project in CLion:**
   If you are using **CLion**, open the project and go to **File** > **Settings** (on Windows) or **CLion** > **Preferences** (on macOS).

2. **Configure Python Interpreter:**
    - Navigate to **Project: <Your Project Name>** > **Python Interpreter**.
    - Click the gear icon and select **Add**.
    - Choose **Virtualenv Environment** and select **Python 3.11** as the base interpreter.
    - CLion will automatically create and activate a virtual environment for your project.

3. **Install Dependencies:**
   Once the virtual environment is created, CLion will prompt you to install the dependencies listed in `requirements.txt`. You can also manually trigger the installation by running:
   ```bash
   pip install -r requirements.txt
   ```

Both methods will set up the environment and install all necessary Python packages required to run the project.
