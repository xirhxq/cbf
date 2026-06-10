#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "optimisers/optimisers"
#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <memory>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <algorithm>

struct OptimiserCase {
    std::string name;
    std::unique_ptr<OptimiserBase> optimiser;
};

struct ReplayProblem {
    Eigen::VectorXd nominal;
    Eigen::VectorXd expected_solution;
    std::vector<Eigen::VectorXd> coefficients;
    std::vector<double> rhs;
    double expected_objective = 0.0;
    double k_delta = 1.0;
};

double objectiveValue(const Eigen::VectorXd &nominal,
                      const Eigen::VectorXd &solution,
                      int u_size,
                      double k_delta) {
    double objective = (solution.head(u_size) - nominal).squaredNorm();
    for (int i = u_size; i < solution.size(); ++i) {
        objective += k_delta * solution[i];
    }
    return objective;
}

double maxConstraintViolation(const std::vector<Eigen::VectorXd> &coefficients,
                              const std::vector<double> &rhs,
                              const Eigen::VectorXd &solution,
                              int u_size) {
    double violation = 0.0;
    for (size_t i = 0; i < coefficients.size(); ++i) {
        violation = std::max(violation, rhs[i] - coefficients[i].dot(solution));
    }
    for (int i = u_size; i < solution.size(); ++i) {
        violation = std::max(violation, -solution[i]);
    }
    return violation;
}

bool isUsableStatus(const std::string &status) {
    return status == "optimal" || status == "optimal_inaccurate";
}

Eigen::VectorXd readControlVector(const json &control) {
    Eigen::VectorXd value(3);
    value << control.value("vx", 0.0),
             control.value("vy", 0.0),
             control.value("yawRateRad", 0.0);
    return value;
}

ReplayProblem readReplayProblem(const json &opt) {
    const int u_size = 3;
    const int slack_size = static_cast<int>(opt.at("slacks").size());
    const int total_size = u_size + slack_size;

    ReplayProblem problem;
    problem.nominal = readControlVector(opt.at("nominal"));
    problem.expected_solution.resize(total_size);
    problem.expected_solution.head(u_size) = readControlVector(opt.at("result"));

    for (int i = 0; i < slack_size; ++i) {
        problem.expected_solution[u_size + i] = opt.at("slacks").at(i).get<double>();
    }

    for (const auto &cbf : opt.at("cbfNoSlack")) {
        Eigen::VectorXd coefficient = Eigen::VectorXd::Zero(total_size);
        coefficient.head(u_size) = readControlVector(cbf.at("coe"));
        problem.coefficients.push_back(coefficient);
        problem.rhs.push_back(-cbf.at("const").get<double>());
    }

    int slack_index = 0;
    for (const auto &cbf : opt.at("cbfSlack")) {
        Eigen::VectorXd coefficient = Eigen::VectorXd::Zero(total_size);
        coefficient.head(u_size) = readControlVector(cbf.at("coe"));
        coefficient[u_size + slack_index] = 1.0;
        problem.coefficients.push_back(coefficient);
        problem.rhs.push_back(-cbf.at("const").get<double>());
        ++slack_index;
    }

    problem.expected_objective = opt.at("solver_info").at("objective_value").get<double>();
    return problem;
}

std::vector<ReplayProblem> loadReplayProblems(const std::filesystem::path &data_path, int max_problems) {
    std::ifstream input(data_path);
    if (!input) {
        throw std::runtime_error("Failed to open replay data: " + data_path.string());
    }

    json data;
    input >> data;

    std::vector<ReplayProblem> problems;
    const double k_delta = data.at("config").at("cbfs").at("objective-function").value("k_delta", 1.0);
    for (const auto &state : data.at("state")) {
        for (const auto &robot : state.at("robots")) {
            if (!robot.contains("opt")) {
                continue;
            }

            const auto &opt = robot.at("opt");
            if (opt.value("status", "") != "success") {
                continue;
            }
            if (!opt.contains("slacks") || !opt.contains("solver_info")) {
                continue;
            }

            ReplayProblem problem = readReplayProblem(opt);
            problem.k_delta = k_delta;
            problems.push_back(problem);
            if (static_cast<int>(problems.size()) >= max_problems) {
                return problems;
            }
        }
    }

    return problems;
}

Eigen::VectorXd solveReplayProblem(OptimiserBase &optimiser, const ReplayProblem &problem) {
    Eigen::VectorXd nominal = problem.nominal;
    optimiser.clear();
    optimiser.start(problem.expected_solution.size(), problem.nominal.size());
    optimiser.setObjective(nominal);

    for (size_t i = 0; i < problem.coefficients.size(); ++i) {
        optimiser.addLinearConstraint(problem.coefficients[i], problem.rhs[i]);
    }

    return optimiser.solve();
}

double replayObjective(const ReplayProblem &problem, const Eigen::VectorXd &solution) {
    return objectiveValue(problem.nominal, solution, problem.nominal.size(), problem.k_delta);
}

double maxConstraintViolation(const ReplayProblem &problem, const Eigen::VectorXd &solution) {
    return maxConstraintViolation(problem.coefficients, problem.rhs, solution, problem.nominal.size());
}

std::vector<OptimiserCase> makeOptimisers(double k_delta) {
    std::vector<OptimiserCase> optimisers;
#ifdef ENABLE_GUROBI
    json gurobi_settings = {{"k_delta", k_delta}};
    optimisers.push_back({"Gurobi", std::make_unique<Gurobi>(gurobi_settings)});
#endif
#ifdef ENABLE_HIGHS
    json highs_settings = {{"k_delta", k_delta}};
    optimisers.push_back({"HiGHS", std::make_unique<HiGHS>(highs_settings)});
#endif
#ifdef ENABLE_OSQP
    json osqp_settings = {{"k_delta", k_delta}};
    optimisers.push_back({"OSQP", std::make_unique<OSQP>(osqp_settings)});
#endif
#ifdef ENABLE_PROXQP
    json proxqp_settings = {{"k_delta", k_delta}};
    optimisers.push_back({"ProxQP", std::make_unique<ProxQP>(proxqp_settings)});
#endif
    return optimisers;
}

void generateRandomProblem(int num_variables,
                           int num_slack_variables,
                           Eigen::VectorXd &uNominal,
                           Eigen::VectorXd &linearConstraintCoefficients,
                           double &rhs_value,
                           std::mt19937 &gen) {
    static std::uniform_real_distribution<> dis(-10.0, 10.0);

    uNominal.resize(num_variables);
    linearConstraintCoefficients.resize(num_variables + num_slack_variables);

    // Random nominal control around zero
    for (int i = 0; i < num_variables; ++i) {
        uNominal[i] = dis(gen);
    }

    // Linear constraint coefficients: random for control vars, +1 for slack vars
    for (int i = 0; i < num_variables; ++i) {
        linearConstraintCoefficients[i] = dis(gen);
    }
    for (int j = 0; j < num_slack_variables; ++j) {
        linearConstraintCoefficients[num_variables + j] = 1.0; // s enters as +1 * s_j
    }

    // Random RHS
    rhs_value = dis(gen);
}

void saveModels(const std::vector<OptimiserCase> &optimisers, int test_id) {
    for (size_t i = 0; i < optimisers.size(); ++i) {
        std::string filename = "model_" + std::to_string(test_id) + "_" + optimisers[i].name + ".lp";
        optimisers[i].optimiser->write(filename);
    }
}

TEST_CASE("RealDataReplayProblemsCanBeLoaded") {
    const auto data_path = std::filesystem::path("data/2025-11-24_14-33-29/data.json");
    if (!std::filesystem::exists(data_path)) {
        MESSAGE("Skipping replay test because " << data_path << " is not available");
        return;
    }

    const auto problems = loadReplayProblems(data_path, 6);
    REQUIRE(problems.size() == 6);
    CHECK(problems.front().nominal.size() == 3);
    CHECK(problems.front().expected_solution.size() == 5);
    CHECK(problems.front().coefficients.size() == 4);
    CHECK(problems.front().rhs.size() == 4);
}

TEST_CASE("RealDataReplaySolversMatchSavedGurobiSolution") {
    const auto data_path = std::filesystem::path("data/2025-11-24_14-33-29/data.json");
    if (!std::filesystem::exists(data_path)) {
        MESSAGE("Skipping replay test because " << data_path << " is not available");
        return;
    }

    const auto problems = loadReplayProblems(data_path, 8);
    REQUIRE(problems.size() == 8);

    auto optimisers = makeOptimisers(problems.front().k_delta);
    REQUIRE(!optimisers.empty());

    for (size_t problem_index = 0; problem_index < problems.size(); ++problem_index) {
        const auto &problem = problems[problem_index];
        for (auto &optimiser : optimisers) {
            CAPTURE(problem_index);
            CAPTURE(optimiser.name);

            Eigen::VectorXd solution = solveReplayProblem(*optimiser.optimiser, problem);
            const auto status = optimiser.optimiser->getStatus();
            REQUIRE(solution.size() == problem.expected_solution.size());
            CHECK(isUsableStatus(status.value("status", "")));
            CHECK(status.value("vars_count", 0) == solution.size());
            CHECK(status.value("constraints_count", 0) == static_cast<int>(problem.coefficients.size()));
            CHECK(maxConstraintViolation(problem, solution) < 1e-4);
            CHECK(replayObjective(problem, solution) == doctest::Approx(problem.expected_objective).epsilon(1e-4).scale(1.0));
            CHECK((solution.head(3) - problem.expected_solution.head(3)).norm() < 1e-2);
        }
    }
}

TEST_CASE("RandomSolvePerformanceComparison") {
    const int num_variables = 10;
    const int num_slack_variables = 3;
    const int num_tests = 100;
    const double k_delta = 10.0;

    std::vector<OptimiserCase> optimisers = makeOptimisers(k_delta);
    REQUIRE(!optimisers.empty());

    std::vector<double> total_times(optimisers.size(), 0.0);
    std::mt19937 gen(0);

    for (int test = 0; test < num_tests; ++test) {
        Eigen::VectorXd uNominal, linearConstraintCoefficients;
        double rhs_value;
        generateRandomProblem(num_variables, num_slack_variables, uNominal, linearConstraintCoefficients, rhs_value, gen);

        Eigen::VectorXd reference_control;
        double reference_objective = 0.0;

        for (size_t i = 0; i < optimisers.size(); ++i) {
            auto &optimiser = optimisers[i].optimiser;
            optimiser->clear();
            optimiser->start(num_variables + num_slack_variables, num_variables);
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

            const double objective = objectiveValue(uNominal, solution, num_variables, k_delta);
            const std::vector<Eigen::VectorXd> coefficients = {linearConstraintCoefficients};
            const std::vector<double> rhs = {rhs_value};
            const auto status = optimiser->getStatus();

            CAPTURE(test);
            CAPTURE(optimisers[i].name);
            CHECK(isUsableStatus(status.value("status", "")));
            CHECK(maxConstraintViolation(coefficients, rhs, solution, num_variables) < 1e-3);

            if (i == 0) {
                reference_control = solution.head(num_variables);
                reference_objective = objective;
            } else {
                CHECK((reference_control - solution.head(num_variables)).norm() < 1e-3);
                CHECK(objective == doctest::Approx(reference_objective).epsilon(1e-3).scale(1.0));
            }
        }
    }

    for (size_t i = 0; i < optimisers.size(); ++i) {
        std::cout << "Average Time for " << optimisers[i].name << ": " << total_times[i] / num_tests << " ms" << std::endl;
    }
}
