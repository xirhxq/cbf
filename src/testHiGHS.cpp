#include "optimisers/HiGHS.hpp"
#include <Eigen/Dense>
#include <iostream>

int main() {
    HiGHS optimiser;

    optimiser.start(2, 3);

    Eigen::VectorXd costs(2);
    costs << 1.0, 1.0;
    optimiser.setObjective(costs, 3.0);

    Eigen::VectorXd lower_bounds(2);
    Eigen::VectorXd upper_bounds(2);
    lower_bounds << 0.0, 1.0;
    upper_bounds << 4.0, 1e30;
    optimiser.setBounds(lower_bounds, upper_bounds);

    Eigen::VectorXd constraint1(2);
    constraint1 << 0.0, 1.0;
    optimiser.addConstraint(constraint1, -1e30, 7.0);

    Eigen::VectorXd constraint2(2);
    constraint2 << 1.0, 2.0;
    optimiser.addConstraint(constraint2, 5.0, 15.0);

    Eigen::VectorXd constraint3(2);
    constraint3 << 3.0, 2.0;
    optimiser.addConstraint(constraint3, 6.0, 1e30);

    Eigen::VectorXd solution = optimiser.solve();
    std::cout << "Optimal solution: " << solution.transpose() << std::endl;
    std::cout << "Objective value: " << optimiser.getObjectiveValue() << std::endl;

    return 0;
}