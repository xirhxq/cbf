#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest/doctest.h"
#include "optimisers/optimisers"
#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <memory>
#include <vector>
#include <string>

void generateRandomProblem(int num_variables, Eigen::VectorXd &uNominal, Eigen::VectorXd &linearConstraintCoefficients,
                           double &rhs_value) {
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

void saveModels(const std::vector<std::unique_ptr<OptimiserBase>> &optimisers, int test_id) {
    for (size_t i = 0; i < optimisers.size(); ++i) {
        std::string filename = "model_" + std::to_string(test_id) + "_opt_" + std::to_string(i) + ".lp";
        optimisers[i]->write(filename);
    }
}

TEST_CASE("RandomSolvePerformanceComparison") {
    const int num_variables = 10;
    const int num_tests = 100;

    std::vector<std::unique_ptr<OptimiserBase>> optimisers;
    optimisers.emplace_back(std::make_unique<Gurobi>());
    optimisers.emplace_back(std::make_unique<HiGHS>());

    std::vector<double> total_times(optimisers.size(), 0.0);

    for (int test = 0; test < num_tests; ++test) {
        Eigen::VectorXd uNominal, linearConstraintCoefficients;
        double rhs_value;
        generateRandomProblem(num_variables, uNominal, linearConstraintCoefficients, rhs_value);

        Eigen::VectorXd reference_solution;
        bool test_failed = false;

        for (size_t i = 0; i < optimisers.size(); ++i) {
            auto &optimiser = optimisers[i];
            optimiser->clear();
            optimiser->start(num_variables);
            optimiser->setObjective(uNominal);
            optimiser->addLinearConstraint(linearConstraintCoefficients, rhs_value);

            auto start_time = std::chrono::high_resolution_clock::now();
            Eigen::VectorXd solution;
            try {
                solution = optimiser->solve();
            } catch (const std::exception &e) {
                saveModels(optimisers, test);
                throw std::runtime_error(std::string("Optimiser threw an exception: ") + e.what());
            }
            auto end_time = std::chrono::high_resolution_clock::now();

            total_times[i] += std::chrono::duration<double, std::milli>(end_time - start_time).count();

            if (i == 0) {
                reference_solution = solution;
            } else {
                if ((reference_solution - solution).norm() >= 1e-3) {
                    test_failed = true;
                }
            }
        }

        if (test_failed) {
            saveModels(optimisers, test);
            std::string msg = "Test failed for test case " + std::to_string(test);
            FAIL(msg);
        }
    }

    for (size_t i = 0; i < optimisers.size(); ++i) {
        std::cout << "Average Time for Optimiser " << i + 1 << ": " << total_times[i] / num_tests << " ms" << std::endl;
    }
}