# CBF

C++ control barrier function library for multi-UAV cooperative search and
maritime simulation. This branch provides the controller headers and EKF
support used with the companion [cbf-ros2](https://github.com/xirhxq/cbf-ros2)
package.

The core library uses C++17, Eigen3, nlohmann/json and Boost. Install the
desired optimizer separately: OSQP with OsqpEigen, HiGHS, or Gurobi. The
Gazebo configuration uses the `GurobiPython` bridge in `cbf-ros2` with a
separately installed and licensed `gurobipy`; the ROS C++ build also uses
OSQP/OsqpEigen. Python estimator support in `gazebo/` requires NumPy.

Configure and build with CMake:

```sh
cmake -S . -B build
cmake --build build
```

Set `ENABLE_GUROBI`, `ENABLE_HIGHS`, and `ENABLE_OSQP` as needed. Historical
doctest targets can be enabled with `-DCBF_BUILD_TESTS=ON` after installing
doctest separately. ROS 2, Gazebo, MBZIRC and their assets are separate
dependencies; see the companion repository for the simulation setup.

## License

The project is distributed under the [MIT License](LICENSE). External
dependencies retain their respective licenses.
