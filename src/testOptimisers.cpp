#include "optimisers/optimisers"
#include <iostream>
#include <Eigen/Dense>
#include <cassert>

void testOptimisers() {
    const int num_variables = 3;
    Eigen::VectorXd uNominal(num_variables);
    uNominal << 1.0, 2.0, 3.0; // Target solution

    Eigen::VectorXd linearConstraintCoefficients(num_variables);
    linearConstraintCoefficients << 1.0, 1.0, 1.0; // Constraint: x0 + x1 + x2 >= 5
    double rhs_value = 5.0;

    // Expected format: minimise ||x - uNominal||^2 with constraint

    // Gurobi Optimiser
    Gurobi gurobiOptimiser;
    gurobiOptimiser.start(num_variables);
    gurobiOptimiser.setObjective(uNominal);
    gurobiOptimiser.addLinearConstraint(linearConstraintCoefficients, rhs_value);
    Eigen::VectorXd gurobiSolution = gurobiOptimiser.solve();

    // HiGHS Optimiser
    HiGHS highsOptimiser;
    highsOptimiser.start(num_variables);
    highsOptimiser.setObjective(uNominal);
    highsOptimiser.addLinearConstraint(linearConstraintCoefficients, rhs_value);
    Eigen::VectorXd highsSolution = highsOptimiser.solve();

    // Compare solutions
    std::cout << "Gurobi Solution: " << gurobiSolution.transpose() << std::endl;
    std::cout << "HiGHS Solution:  " << highsSolution.transpose() << std::endl;

    // Check if solutions are approximately equal
    auto norm = (gurobiSolution - highsSolution).norm();
    std::cout << norm << std::endl;
    assert((gurobiSolution - highsSolution).norm() < 1e-6);

    std::cout << "Test passed: Solutions are consistent between Gurobi and HiGHS." << std::endl;
}

int main() {
    try {
        testOptimisers();
    } catch (const std::exception &e) {
        std::cerr << "Exception caught during testing: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
