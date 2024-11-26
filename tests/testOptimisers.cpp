#include "optimisers/optimisers"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <chrono>
#include <random>


void generateRandomProblem(int num_variables, Eigen::VectorXd &uNominal, Eigen::VectorXd &linearConstraintCoefficients, double &rhs_value) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-10.0, 10.0);

    uNominal.resize(num_variables);
    linearConstraintCoefficients.resize(num_variables);

    for (int i = 0; i < num_variables; ++i) {
        uNominal[i] = dis(gen);
        linearConstraintCoefficients[i] = dis(gen);
    }

    rhs_value = dis(gen);
}


TEST(OptimiserTest, SingleSolveConsistency) {
    const int num_variables = 3;
    Eigen::VectorXd uNominal(num_variables);
    uNominal << 1.0, 2.0, 3.0;

    Eigen::VectorXd linearConstraintCoefficients(num_variables);
    linearConstraintCoefficients << 1.0, 1.0, 1.0;
    double rhs_value = 5.0;

    Gurobi gurobiOptimiser;
    gurobiOptimiser.start(num_variables);
    gurobiOptimiser.setObjective(uNominal);
    gurobiOptimiser.addLinearConstraint(linearConstraintCoefficients, rhs_value);
    Eigen::VectorXd gurobiSolution = gurobiOptimiser.solve();

    HiGHS highsOptimiser;
    highsOptimiser.start(num_variables);
    highsOptimiser.setObjective(uNominal);
    highsOptimiser.addLinearConstraint(linearConstraintCoefficients, rhs_value);
    Eigen::VectorXd highsSolution = highsOptimiser.solve();

    ASSERT_NEAR((gurobiSolution - highsSolution).norm(), 0.0, 1e-6)
                                << "Gurobi Solution: " << gurobiSolution.transpose()
                                << ", HiGHS Solution: " << highsSolution.transpose();
}


TEST(PerformanceTest, RandomSolvePerformanceComparison) {
    const int num_variables = 10;
    const int num_tests = 100;
    double gurobi_total_time = 0.0;
    double highs_total_time = 0.0;

    for (int i = 0; i < num_tests; ++i) {
        Eigen::VectorXd uNominal, linearConstraintCoefficients;
        double rhs_value;
        generateRandomProblem(num_variables, uNominal, linearConstraintCoefficients, rhs_value);

        Gurobi gurobiOptimiser;
        gurobiOptimiser.start(num_variables);
        gurobiOptimiser.setObjective(uNominal);
        gurobiOptimiser.addLinearConstraint(linearConstraintCoefficients, rhs_value);

        auto gurobi_start = std::chrono::high_resolution_clock::now();
        Eigen::VectorXd gurobiSolution = gurobiOptimiser.solve();
        auto gurobi_end = std::chrono::high_resolution_clock::now();
        gurobi_total_time += std::chrono::duration<double, std::milli>(gurobi_end - gurobi_start).count();

        HiGHS highsOptimiser;
        highsOptimiser.start(num_variables);
        highsOptimiser.setObjective(uNominal);
        highsOptimiser.addLinearConstraint(linearConstraintCoefficients, rhs_value);

        auto highs_start = std::chrono::high_resolution_clock::now();
        Eigen::VectorXd highsSolution = highsOptimiser.solve();
        auto highs_end = std::chrono::high_resolution_clock::now();
        highs_total_time += std::chrono::duration<double, std::milli>(highs_end - highs_start).count();

        ASSERT_NEAR((gurobiSolution - highsSolution).norm(), 0.0, 1e-3)
                                    << "Random Test " << i + 1 << " failed. " << std::endl
                                    << "Gurobi Solution: " << gurobiSolution.transpose() << std::endl
                                    << ", HiGHS Solution: " << highsSolution.transpose() << std::endl;
    }

    double gurobi_avg_time = gurobi_total_time / num_tests;
    double highs_avg_time = highs_total_time / num_tests;

    std::cout << "Average Gurobi Time: " << gurobi_avg_time << " ms" << std::endl;
    std::cout << "Average HiGHS Time:  " << highs_avg_time << " ms" << std::endl;

    EXPECT_LT(highs_avg_time, gurobi_avg_time * 10)
                        << "HiGHS is significantly slower than Gurobi.";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}