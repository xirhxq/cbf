#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif

#include "doctest.h"
#include "Robot.hpp"
#include "Swarm.hpp"
#include "bridge/ReserveTaskHomotopy.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <memory>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <type_traits>

static_assert(std::is_same_v<
    decltype(&OptimiserBase::addLinearConstraint),
    void (OptimiserBase::*)(const Eigen::VectorXd &, double)
>);
static_assert(std::is_same_v<
    decltype(&OptimiserBase::setObjective),
    void (OptimiserBase::*)(const Eigen::VectorXd &)
>);

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

std::string selectRobotTestOptimiser() {
    auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "OSQP") != available.end()) {
        return "OSQP";
    }
    REQUIRE(!available.empty());
    return available.front();
}

json makeSingleRobotNoCbfConfig(const std::string &optimiser_name) {
    return {
        {"world", {
            {"boundary", {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}},
            {"charge", json::array()},
            {"spacing", 1.0}
        }},
        {"num", 1},
        {"all", {1}},
        {"formation", {
            {"parts", 1},
            {"bases-id", {json::array()}}
        }},
        {"bases", json::array()},
        {"initial", {
            {"position", {
                {"method", "specified"},
                {"positions", {{5.0, 5.0}}}
            }},
            {"battery", {
                {"min", 4100.0},
                {"max", 4100.0}
            }},
            {"yawDeg", 0.0}
        }},
        {"model", "SingleIntegrate2D"},
        {"model-params", {
            {"discharge-rate", 0.0}
        }},
        {"optimiser", optimiser_name},
        {"cbfs", {
            {"objective-function", {
                {"k_delta", 10.0}
            }},
            {"with-slack", {
                {"cvt", {{"on", false}}},
                {"cvt-yaw", {{"on", false}}},
                {"target-yaw", {{"on", false}}}
            }},
            {"without-slack", {
                {"method", "all"},
                {"energy", {{"on", false}}},
                {"safety", {{"on", false}}},
                {"comm-fixed", {{"on", false}}},
                {"comm-auto", {{"on", false}}}
            }}
        }},
        {"debug", {
            {"opt-cbc", false}
        }}
    };
}

json makeFullRowSingleLadderSwarmConfig(
        const std::string &optimiserName,
        const std::filesystem::path &outputPath) {
    return {
        {"world", {
            {"boundary", {{0.0, 0.0}, {3000.0, 0.0},
                          {3000.0, 2600.0}, {0.0, 2600.0}}},
            {"charge", json::array()},
            {"spacing", 100.0}
        }},
        {"num", 4},
        {"dim", 4},
        {"formation", {
            {"parts", 1},
            {"bases-id", {{0, 1}}}
        }},
        {"bases", {{200.0, 1200.0}, {200.0, 2049.0}}},
        {"initial", {
            {"position", {
                {"method", "specified"},
                {"positions", {
                    {935.255567813, 1624.5},
                    {935.255567813, 775.5},
                    {1670.511135626, 1200.0},
                    {1670.511135626, 351.0}
                }}
            }},
            {"velocity", {{"values", {
                {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}
            }}}},
            {"battery", {{"min", 4100.0}, {"max", 4100.0}}},
            {"yawDeg", 0.0}
        }},
        {"model", "DoubleIntegrate2D"},
        {"model-params", {{"discharge-rate", 0.0}}},
        {"position_covariance", {{"enable", false}}},
        {"searching", {
            {"method", "downward"},
            {"downward", {{"radius", 150.0}}}
        }},
        {"optimiser", optimiserName},
        {"cbfs", {
            {"objective-function", {{"k_delta", 10.0}}},
            {"high-order", {
                {"enabled", true},
                {"lambda1", 1.0},
                {"lambda2", 1.0},
                {"acceleration-bound", 4.0},
                {"sampled-data-reserve", 0.0},
                {"feasibility-slack", {{"enabled", false}}}
            }},
            {"with-slack", {
                {"cvt", {{"on", false}}},
                {"cvt-yaw", {{"on", false}}},
                {"target-yaw", {{"on", false}}}
            }},
            {"without-slack", {
                {"method", "all"},
                {"energy", {{"on", false}}},
                {"safety", {
                    {"on", true},
                    {"safe-distance", 10.0},
                    {"safe-distance-tightening-margin", 0.0},
                    {"consider-uncertainty", false},
                    {"pair-state-reserve", {
                        {"enabled", false},
                        {"velocity-gain", 0.0},
                        {"sample-time", 0.1},
                        {"acceleration-gain", 0.0},
                        {"neighbor-acceleration-bound", 0.0},
                        {"max-reserve", 0.0}
                    }}
                }},
                {"comm-fixed", {
                    {"on", true},
                    {"max-range", 850.0},
                    {"range-tightening-margin", 0.0},
                    {"k", 1.0},
                    {"compensate-velocity", true},
                    {"consider-uncertainty", false},
                    {"state-dependent-reserve", {
                        {"enabled", false},
                        {"velocity-gain", 0.0},
                        {"sample-time", 0.1},
                        {"acceleration-gain", 0.0},
                        {"neighbor-acceleration-bound", 0.0},
                        {"max-reserve", 0.0}
                    }}
                }},
                {"comm-auto", {{"on", false}}}
            }}
        }},
        {"bridge", {
            {"enabled", true},
            {"row", "test"},
            {"search-policy", "coverage"},
            {"topology-policy", "fixed"},
            {"safety-filter", "second-order-hocbf"},
            {"topology", {
                {"max-range", 850.0},
                {"uncertainty-multiplier", 0.0},
                {"certified-margin", 0.0},
                {"certified-only", true},
                {"fail-safe-hold", true},
                {"fixed-references", {
                    {"1", {{"anchor-ids", json::array()}, {"base-ids", {0, 1}}}},
                    {"2", {{"anchor-ids", {1}}, {"base-ids", {0}}}},
                    {"3", {{"anchor-ids", {2, 1}}, {"base-ids", json::array()}}},
                    {"4", {{"anchor-ids", {3, 2}}, {"base-ids", json::array()}}}
                }}
            }},
            {"nominal", {
                {"max-speed", 8.0},
                {"max-acceleration", 4.0},
                {"max-yaw-rate", 0.35},
                {"guard", {
                    {"enabled", true},
                    {"mode", "hocbf-feasible-projection"},
                    {"tolerance", 1.0e-9}
                }},
                {"gamma-star-feedback", {
                    {"enabled", true},
                    {"mode", "reserve-task-homotopy"},
                    {"analysis-role", "main"},
                    {"homotopy-intervals", 8},
                    {"lookahead-steps", 1},
                    {"predictive-gate", 0.0}
                }}
            }},
            {"target", {{"x", 1800.0}, {"y", 1800.0}, {"radius", 50.0}}}
        }},
        {"execute", {
            {"execution-mode", "distributed"},
            {"time-total", 1.0},
            {"time-step", 0.5},
            {"random-seed", 20261501},
            {"check-constraint-violation", false}
        }},
        {"debug", {{"opt-cbc", false}}},
        {"output_path", outputPath.string()},
        {"run_suffix", "_full_row_swarm_test"}
    };
}

struct ScopedTestDirectory {
    explicit ScopedTestDirectory(std::filesystem::path pathIn)
            : path(std::move(pathIn)) {
        std::filesystem::create_directories(path);
    }
    ~ScopedTestDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    std::filesystem::path path;
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

TEST_CASE("SlackConstraintCoefficientsAppendSingleActiveSlack") {
    Eigen::VectorXd uCoe(3);
    uCoe << 1.0, -2.0, 3.0;

    Eigen::VectorXd coe = makeSlackConstraintCoefficients(uCoe, 4, 2);

    REQUIRE(coe.size() == 7);
    CHECK(coe(0) == doctest::Approx(1.0));
    CHECK(coe(1) == doctest::Approx(-2.0));
    CHECK(coe(2) == doctest::Approx(3.0));
    CHECK(coe(3) == doctest::Approx(0.0));
    CHECK(coe(4) == doctest::Approx(0.0));
    CHECK(coe(5) == doctest::Approx(1.0));
    CHECK(coe(6) == doctest::Approx(0.0));
}

TEST_CASE("Gurobi preserves an explicit infeasible status without reading a solution") {
#ifdef ENABLE_GUROBI
    json settings = {{"k_delta", 10.0}};
    Gurobi optimiser(settings);
    optimiser.start(1, 1);
    optimiser.setObjective(Eigen::VectorXd::Zero(1));
    Eigen::VectorXd positive(1);
    positive << 1.0;
    Eigen::VectorXd negative(1);
    negative << -1.0;
    optimiser.addLinearConstraint(positive, 1.0);
    optimiser.addLinearConstraint(negative, 1.0);

    const Eigen::VectorXd result = optimiser.solve();
    const json status = optimiser.getStatus();

    CHECK(result.norm() == doctest::Approx(0.0));
    CHECK(status.at("status") == "infeasible");
    CHECK(status.at("objective_value").is_null());
    CHECK(status.at("constraints_count") == 2);
#else
    MESSAGE("Gurobi is unavailable; explicit infeasible status was not tested");
#endif
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

TEST_CASE("RobotOptimiseUsesZeroNominalControlWhenNoNominalPolicyConfigured") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);

    Robot robot(1, settings);
    robot.optimise();

    Eigen::VectorXd control = robot.model->getControlInput();
    REQUIRE(control.size() == 3);
    CHECK(control[0] == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(control[1] == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(control[2] == doctest::Approx(0.0).epsilon(1e-12));

    CHECK(robot.opt.at("nominal").at("vx").get<double>() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(robot.opt.at("nominal").at("vy").get<double>() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(robot.opt.at("nominal").at("yawRateRad").get<double>() == doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("RobotCachesSearchConfigurationAtConstruction") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["searching"] = {
        {"method", "front-sector"},
        {"front-sector", {
            {"inner-radius", 0.0},
            {"outer-radius", 400.0},
            {"half-angle-deg", 60.0}
        }}
    };

    Robot robot(1, settings);

    CHECK(robot.searchMethod == "front-sector");
    CHECK(robot.searchParams.at("outer-radius").get<double>() == doctest::Approx(400.0));
    CHECK(robot.searchParams.at("half-angle-deg").get<double>() == doctest::Approx(60.0));
}

TEST_CASE("RobotGetStateEvaluatesCbfLogsFromSingleStateSnapshot") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);

    Robot robot(1, settings);

    CBF mutatingCBF;
    mutatingCBF.name = "mutatingCBF";
    mutatingCBF.h = [&](VectorXd x, double) {
        robot.model->setStateVariable("x", 9.0);
        return x[0];
    };
    robot.cbfNoSlack.cbfs[mutatingCBF.name] = mutatingCBF;

    CBF snapshotCBF;
    snapshotCBF.name = "snapshotCBF";
    snapshotCBF.h = [&](VectorXd x, double) {
        return x[0];
    };
    robot.cbfSlack[snapshotCBF.name] = snapshotCBF;

    json state = robot.getState();

    CHECK(state.at("state").at("x").get<double>() == doctest::Approx(5.0));
    CHECK(state.at("cbfNoSlack").at("mutatingCBF").get<double>() == doctest::Approx(5.0));
    CHECK(state.at("cbfSlack").at("snapshotCBF").get<double>() == doctest::Approx(5.0));
}

TEST_CASE("RobotBuildsSecondOrderSafetyCbfForDoubleIntegrator") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["model"] = "DoubleIntegrate2D";
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["cbfs"]["without-slack"]["safety"] = {
        {"on", true},
        {"safe-distance", 1.0},
        {"consider-uncertainty", false}
    };
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"lambda1", 1.0},
        {"lambda2", 1.0}
    };

    Robot robot(1, settings);
    robot.model->setStateVariable("vx", 0.5);
    robot.model->setStateVariable("vy", 0.0);

    VectorXd otherVelocity(2);
    otherVelocity << 0.0, 0.0;
    VectorXd otherAcceleration(2);
    otherAcceleration << 0.1, 0.0;

    robot.comm->receivePosition2D(2, Point(8.0, 5.0));
    robot.comm->receiveVelocity2D(2, otherVelocity);
    robot.comm->receiveAcceleration2D(2, otherAcceleration);

    robot.postsetCBF();

    REQUIRE(robot.secondOrderCbfNoSlack.count("secondOrderSafetyCBF(#2)") == 1);
    VectorXd x = robot.model->getX();
    auto evaluation = robot.secondOrderCbfNoSlack.at("secondOrderSafetyCBF(#2)").evaluateConstraint(x, 0.0);

    CHECK(evaluation.h == doctest::Approx(2.0));
    CHECK(evaluation.hdot == doctest::Approx(-0.5));
    CHECK(evaluation.psi1 == doctest::Approx(1.5));
    CHECK(evaluation.uCoe.size() == 3);
    CHECK(evaluation.uCoe(0) == doctest::Approx(-1.0));
    CHECK(evaluation.uCoe(1) == doctest::Approx(0.0));
    CHECK(evaluation.uCoe(2) == doctest::Approx(0.0));
    CHECK(evaluation.constTerm == doctest::Approx(1.1));
}

TEST_CASE("RobotAppliesTighteningMarginToFirstOrderSafetyCbf") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["cbfs"]["without-slack"]["safety"] = {
        {"on", true},
        {"safe-distance", 1.0},
        {"safe-distance-tightening-margin", 0.5},
        {"k", 1.0},
        {"consider-uncertainty", false}
    };

    Robot robot(1, settings);
    robot.comm->receivePosition2D(2, Point(8.0, 5.0));

    robot.postsetCBF();

    REQUIRE(robot.cbfNoSlack.cbfs.count("safetyCBF") == 1);
    VectorXd x = robot.model->getX();
    auto evaluation = robot.cbfNoSlack.cbfs.at("safetyCBF").evaluateConstraint(
        robot.model->f(), robot.model->g(), x, 0.0);

    CHECK(evaluation.h == doctest::Approx(1.5));
}

TEST_CASE("RobotInitializesDoubleIntegratorVelocityFromConfig") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["model"] = "DoubleIntegrate2D";
    settings["initial"]["velocity"] = {
        {"values", {{1.25, -0.5}}}
    };

    Robot robot(1, settings);

    CHECK(robot.model->getStateVariable("vx") == doctest::Approx(1.25));
    CHECK(robot.model->getStateVariable("vy") == doctest::Approx(-0.5));
}

TEST_CASE("RobotLogsSolvedSecondOrderHocbfValue") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["model"] = "DoubleIntegrate2D";

    Robot robot(1, settings);

    SecondOrderCBF cbf;
    cbf.name = "manualSecondOrderCBF";
    cbf.k0 = 0.0;
    cbf.k1 = 0.0;
    cbf.lambda1 = 1.0;
    cbf.h = [](const VectorXd&, double) { return 0.0; };
    cbf.hdot = [](const VectorXd&, double) { return 0.0; };
    cbf.hddotConst = [](const VectorXd&, double) { return -1.0; };
    cbf.uCoe = [](const VectorXd&, double) {
        VectorXd coe = VectorXd::Zero(3);
        coe(0) = 1.0;
        return coe;
    };
    robot.secondOrderCbfNoSlack[cbf.name] = cbf;

    robot.optimise();

    CHECK(robot.model->getControlInput()(0) == doctest::Approx(1.0).epsilon(1e-5));
    REQUIRE(robot.opt.at("hocbfNoSlack").size() == 1);
    CHECK(robot.opt.at("hocbfNoSlack").at(0).at("hocbf").get<double>() == doctest::Approx(0.0).epsilon(1e-5));
}

TEST_CASE("RobotUsesBridgeNominalControlOverride") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["cbfs"]["without-slack"]["safety"]["on"] = false;
    settings["cbfs"]["without-slack"]["comm-fixed"]["on"] = false;
    settings["cbfs"]["with-slack"]["cvt"]["on"] = false;
    settings["cbfs"]["with-slack"]["cvt-yaw"]["on"] = false;
    settings["initial"]["position"]["positions"] = {{1.0, 1.0}};

    Robot robot(1, settings);
    Eigen::VectorXd nominal(3);
    nominal << 0.4, -0.2, 0.1;
    robot.setNominalControlOverride(nominal);
    robot.optimise();

    CHECK(robot.opt.at("nominal").at("vx").get<double>() == doctest::Approx(0.4));
    CHECK(robot.opt.at("nominal").at("vy").get<double>() == doctest::Approx(-0.2));
    CHECK(robot.opt.at("nominal").at("yawRateRad").get<double>() == doctest::Approx(0.1));
    CHECK(robot.opt.at("result").at("vx").get<double>() == doctest::Approx(0.4).epsilon(1e-4));
}

TEST_CASE("RobotProjectsNominalAccelerationThroughHocbfGuardAndPreservesYawRate") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["model"] = "DoubleIntegrate2D";
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"acceleration-bound", 2.0}
    };

    Robot robot(1, settings);
    Eigen::VectorXd nominal(3);
    nominal << -1.0, 0.5, 0.25;
    robot.setNominalControlOverride(nominal);

    SecondOrderCBF cbf;
    cbf.name = "manualGuardCBF";
    cbf.k0 = 0.0;
    cbf.k1 = 0.0;
    cbf.lambda1 = 1.0;
    cbf.h = [](const VectorXd&, double) { return 0.0; };
    cbf.hdot = [](const VectorXd&, double) { return 0.0; };
    cbf.hddotConst = [](const VectorXd&, double) { return 0.0; };
    cbf.uCoe = [](const VectorXd&, double) {
        VectorXd coe = VectorXd::Zero(3);
        coe(0) = 1.0;
        return coe;
    };
    robot.secondOrderCbfNoSlack[cbf.name] = cbf;

    robot.applySecondOrderNominalFeasibilityGuard(1.0e-9);
    robot.optimise();

    CHECK(robot.opt.at("nominal").at("ax").get<double>() == doctest::Approx(0.0));
    CHECK(robot.opt.at("nominal").at("ay").get<double>() == doctest::Approx(0.5));
    CHECK(robot.opt.at("nominal").at("yawRateRad").get<double>() == doctest::Approx(0.25));
    REQUIRE(robot.opt.contains("nominalGuard"));
    CHECK(robot.opt.at("nominalGuard").at("enabled").get<bool>());
    CHECK(robot.opt.at("nominalGuard").at("active").get<bool>());
    CHECK(robot.opt.at("nominalGuard").at("feasible").get<bool>());
    CHECK(robot.opt.at("nominalGuard").at("margin_before").get<double>() == doctest::Approx(-1.0));
    CHECK(robot.opt.at("nominalGuard").at("margin_after").get<double>() == doctest::Approx(0.0));
}

TEST_CASE("HomotopyCandidateExecutesWithinFrozenReproductionTolerance") {
    const std::string optimiserName = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiserName);
    settings["model"] = "DoubleIntegrate2D";
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"acceleration-bound", 2.0}
    };

    const std::vector<BridgeHocbfHalfspace2D> constraints = {
            {1.0, 0.0, 0.0},
            {-1.0, 0.0, -1.0},
    };
    const auto legacy = projectBridgeHocbfNominalAcceleration(
            -1.0, 0.0, 2.0, constraints, 1.0e-12);
    REQUIRE(legacy.feasible);
    CHECK(legacy.projected_ax == doctest::Approx(0.0).epsilon(1.0e-12));

    const auto reserve = solveExactBridgeGammaStar2D({
            bridgeGammaStarResidualFromAffineMargin(1.0, 0.0, 0.0),
            bridgeGammaStarResidualFromAffineMargin(-1.0, 0.0, 1.0),
    }, 2.0);
    REQUIRE(reserve.valid);
    CHECK(reserve.accelX == doctest::Approx(0.5).epsilon(1.0e-12));

    auto candidates = buildReserveTaskHomotopy(
            legacy.projected_ax, legacy.projected_ay,
            reserve.accelX, reserve.accelY, 4);
    for (auto &candidate : candidates) {
        candidate.predictedBudget = candidate.alpha >= 0.5 ? 0.1 : -0.1;
    }
    const auto selected = selectReserveTaskHomotopy(
            candidates,
            legacy.projected_ax, legacy.projected_ay,
            0.0);
    REQUIRE(selected.selected);
    CHECK(selected.alpha == doctest::Approx(0.5));
    CHECK(bridgeHocbfMinMargin(
            constraints, selected.accelX, selected.accelY) >= -1.0e-12);

    Robot robot(1, settings);
    SecondOrderCBF lower;
    lower.name = "homotopyLower";
    lower.k0 = 0.0;
    lower.k1 = 0.0;
    lower.lambda1 = 1.0;
    lower.h = [](const VectorXd &, double) { return 0.0; };
    lower.hdot = [](const VectorXd &, double) { return 0.0; };
    lower.hddotConst = [](const VectorXd &, double) { return 0.0; };
    lower.uCoe = [](const VectorXd &, double) {
        VectorXd coefficients = VectorXd::Zero(3);
        coefficients(0) = 1.0;
        return coefficients;
    };
    SecondOrderCBF upper = lower;
    upper.name = "homotopyUpper";
    upper.hddotConst = [](const VectorXd &, double) { return 1.0; };
    upper.uCoe = [](const VectorXd &, double) {
        VectorXd coefficients = VectorXd::Zero(3);
        coefficients(0) = -1.0;
        return coefficients;
    };
    robot.secondOrderCbfNoSlack[lower.name] = lower;
    robot.secondOrderCbfNoSlack[upper.name] = upper;

    VectorXd nominal = VectorXd::Zero(3);
    nominal(0) = selected.accelX;
    nominal(1) = selected.accelY;
    robot.setNominalControlOverride(nominal);
    robot.applySecondOrderNominalFeasibilityGuard(1.0e-12);
    CHECK(robot.nominalGuardDiagnostic.at("projection_norm").get<double>()
          <= 1.0e-12);
    robot.optimise();

    const auto executed = robot.model->getAcceleration();
    CHECK((executed - Eigen::Vector2d(selected.accelX, selected.accelY)).norm()
          <= BRIDGE_FULL_ROW_QP_REPRODUCTION_TOLERANCE);
}

TEST_CASE("Finite-precision endpoint repair moves only toward the certified witness") {
    const std::vector<BridgeHocbfHalfspace2D> constraints = {
            {1.0, 0.0, 0.0},
            {-1.0, 0.0, -1.0},
    };
    const auto repair = certifyBridgeHocbfEndpointTowardWitness(
            -5.0e-13, 0.25,
            0.5, 0.25,
            2.0,
            constraints);

    REQUIRE(repair.valid);
    CHECK(repair.repaired);
    CHECK(repair.repairMix > 0.0);
    CHECK(repair.repairMix < 1.0e-9);
    CHECK(repair.repairNorm < 1.0e-9);
    CHECK(repair.marginBefore < 0.0);
    CHECK(repair.marginAfter >= 0.0);
    CHECK(bridgeHocbfMinMargin(
            constraints, repair.accelX, repair.accelY) >= 0.0);

    const auto unchanged = certifyBridgeHocbfEndpointTowardWitness(
            0.25, 0.25,
            0.5, 0.25,
            2.0,
            constraints);
    REQUIRE(unchanged.valid);
    CHECK_FALSE(unchanged.repaired);
    CHECK(unchanged.accelX == doctest::Approx(0.25));
    CHECK(unchanged.accelY == doctest::Approx(0.25));

    const double boundaryCandidate = 0.3665856765818431;
    const std::vector<BridgeHocbfHalfspace2D> ulpConstraints = {{
            1.0,
            0.0,
            std::nextafter(
                    boundaryCandidate,
                    std::numeric_limits<double>::infinity()),
    }};
    const auto ulpRepair = certifyBridgeHocbfEndpointTowardWitness(
            boundaryCandidate, 1.3650549828132403,
            0.5, 0.0,
            2.0,
            ulpConstraints);
    REQUIRE(ulpRepair.valid);
    CHECK(ulpRepair.repaired);
    CHECK(ulpRepair.repairNorm <= BRIDGE_FULL_ROW_NUMERICAL_REPAIR_TOLERANCE);
    CHECK(ulpRepair.marginBefore < 0.0);
    CHECK(ulpRepair.marginAfter >= 0.0);
}

TEST_CASE("Full-row single-ladder Swarm step preserves rows execution and prediction audit") {
    const auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "Gurobi")
            == available.end()) {
        WARN("Gurobi is unavailable; full-row Swarm integration was not run");
        return;
    }

    const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
    ScopedTestDirectory output(
            std::filesystem::temp_directory_path()
            / ("cbf-full-row-swarm-" + uniqueSuffix));
    auto settings = makeFullRowSingleLadderSwarmConfig(
            "Gurobi", output.path);
    Swarm swarm(settings);
    swarm.run();

    REQUIRE(swarm.data.at("state").size() == 2);
    bool exercisedActiveSelectedRow = false;
    bool exercisedPositiveIntervention = false;
    std::set<int> auditedOriginZeroHorizons;
    const std::map<int, std::vector<std::string>> expectedRows = {
            {1, {
                    "secondOrderFixedCommCBF(base-0)",
                    "secondOrderFixedCommCBF(base-1)",
                    "secondOrderSafetyCBF(#2)",
                    "secondOrderSafetyCBF(#3)",
                    "secondOrderSafetyCBF(#4)"}},
            {2, {
                    "secondOrderFixedCommCBF(#1)",
                    "secondOrderFixedCommCBF(base-0)",
                    "secondOrderSafetyCBF(#1)",
                    "secondOrderSafetyCBF(#3)",
                    "secondOrderSafetyCBF(#4)"}},
            {3, {
                    "secondOrderFixedCommCBF(#1)",
                    "secondOrderFixedCommCBF(#2)",
                    "secondOrderSafetyCBF(#1)",
                    "secondOrderSafetyCBF(#2)",
                    "secondOrderSafetyCBF(#4)"}},
            {4, {
                    "secondOrderFixedCommCBF(#2)",
                    "secondOrderFixedCommCBF(#3)",
                    "secondOrderSafetyCBF(#1)",
                    "secondOrderSafetyCBF(#2)",
                    "secondOrderSafetyCBF(#3)"}},
    };
    for (const auto &step : swarm.data.at("state")) {
        const auto &topology = step.at("bridge").at("topology");
        CHECK(topology.at("fixed") == true);
        CHECK(topology.at("certified") == true);
        CHECK(topology.at("geometry").size() == 8);

        const auto &feedback = step.at("bridge").at("nominal")
                .at("gamma_star_feedback");
        const auto &budgetAudit = step.at("bridge").at("nominal")
                .at("full_row_budget_audit");
        CHECK(budgetAudit.at("valid") == true);
        CHECK(budgetAudit.at("robots").size() == 4);
        CHECK(budgetAudit.at("minimum_gamma_star").is_number());
        const auto &taskGoals = step.at("bridge").at("nominal")
                .at("task_goals");
        CHECK(taskGoals.size() == 4);
        for (const auto &taskGoal : taskGoals) {
            CHECK(taskGoal.at("robot").is_number_integer());
            CHECK(taskGoal.at("x").is_number());
            CHECK(taskGoal.at("y").is_number());
            CHECK(taskGoal.at("changed").is_boolean());
        }
        REQUIRE(feedback.at("links").size() == 4);
        for (const auto &link : feedback.at("links")) {
            INFO(link.dump(2));
            const int robotId = link.at("robot").get<int>();
            CHECK(link.at("selected") == true);
            if (!link.at("selected").get<bool>()) {
                continue;
            }
            CHECK(link.at("candidate_current_feasible") == true);
            CHECK(link.at("pre_score_constraint_ledger_consistent") == true);
            CHECK(link.at("pre_score_row_identity_consistent") == true);
            CHECK(link.at("pre_score_row_class_consistent") == true);
            CHECK(link.at("pre_score_row_execution_class") == "hard");
            CHECK(link.at("row_identity_consistent") == true);
            CHECK(link.at("execution_consistent") == true);
            CHECK(link.at("row_execution_class") == "hard");
            CHECK(link.at("neighbor_prediction_mode")
                  == "synchronous-task-reference");
            CHECK(link.at("selected_candidate_index").is_number_unsigned());
            CHECK(link.at("maximum_homotopy_candidate_index")
                  .is_number_unsigned());
            CHECK(link.at("maximum_homotopy_predicted_budget").is_number());
            if (link.at("fallback_used").get<bool>()) {
                CHECK(link.at("fallback_reason")
                      == "predictive-gate-unattainable");
                CHECK(link.at("selected_is_homotopy_argmax") == true);
                CHECK(link.at("fallback_positive_recoverability")
                      .is_boolean());
            } else {
                CHECK(link.at("fallback_reason").is_null());
                CHECK(link.at("fallback_positive_recoverability").is_null());
            }
            CHECK(link.at("installed_hard_row_identities")
                  == expectedRows.at(robotId));
            CHECK(link.at("guard_candidate_difference").get<double>()
                  <= BRIDGE_FULL_ROW_GUARD_REPRODUCTION_TOLERANCE);
            CHECK(link.at("qp_candidate_difference").get<double>()
                  <= BRIDGE_FULL_ROW_QP_REPRODUCTION_TOLERANCE);
            CHECK(link.at("solver_optimal") == true);
            CHECK(link.at("qp_minimum_hard_margin").get<double>()
                  >= -BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
            CHECK(link.at("qp_acceleration_box_violation").get<double>()
                  <= BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
            exercisedPositiveIntervention = exercisedPositiveIntervention
                    || link.at("selected_alpha").get<double>() > 0.0;
            exercisedActiveSelectedRow = exercisedActiveSelectedRow
                    || link.at("current_candidate_minimum_margin").get<double>()
                            <= 1.0e-8;
            if (link.at("current_candidate_minimum_margin").get<double>()
                    <= 1.0e-8) {
                double minimumMargin = std::numeric_limits<double>::infinity();
                std::string minimumIdentity;
                for (const auto &row : link.at("current_rows")) {
                    const double margin = row.at("const_term").get<double>()
                            + row.at("u_coe").at(0).get<double>()
                                    * link.at("selected_accel_x").get<double>()
                            + row.at("u_coe").at(1).get<double>()
                                    * link.at("selected_accel_y").get<double>();
                    if (margin < minimumMargin) {
                        minimumMargin = margin;
                        minimumIdentity = row.at("identity").get<std::string>();
                    }
                }
                CHECK(minimumMargin
                      >= -BRIDGE_FULL_ROW_NUMERICAL_REPAIR_TOLERANCE);
                CHECK(minimumIdentity.find("secondOrderFixedCommCBF") == 0);
            }
        }
        const auto &audit = feedback.at("prediction_audit");
        CHECK(audit.at("invalid_count") == 0);
        for (const auto &resolved : audit.at("resolved")) {
            if (resolved.at("origin_step") == 0) {
                auditedOriginZeroHorizons.insert(
                        resolved.at("horizon_step").get<int>());
            }
        }
    }
    CHECK(exercisedActiveSelectedRow);
    CHECK(exercisedPositiveIntervention);
    CHECK(auditedOriginZeroHorizons == std::set<int>({1}));

    const auto &audit0 = swarm.data.at("state").at(0)
            .at("bridge").at("nominal").at("gamma_star_feedback")
            .at("prediction_audit");
    CHECK(audit0.at("resolved_count") == 0);
    CHECK(audit0.at("pending_count") == 4);

    const auto &audit1 = swarm.data.at("state").at(1)
            .at("bridge").at("nominal").at("gamma_star_feedback")
            .at("prediction_audit");
    CHECK(audit1.at("resolved_count") == 4);
    CHECK(audit1.at("invalid_count") == 0);
    CHECK(audit1.at("pending_count") == 4);
    REQUIRE(audit1.at("resolved").size() == 4);
    for (const auto &audit : audit1.at("resolved")) {
        CHECK(audit.at("valid") == true);
        CHECK(audit.at("origin_step") == 0);
        CHECK(audit.at("due_step") == 1);
        CHECK(audit.at("observed_step") == 1);
        CHECK(audit.at("horizon_step") == 1);
        CHECK(audit.at("states").size() == 4);
        CHECK(audit.at("actual_budget").is_number());
        CHECK(audit.at("budget_error_actual_minus_predicted").is_number());
    }
    CHECK(swarm.data.at("bridge")
          .at("prediction_audit_unresolved_at_horizon") == 4);
    CHECK(swarm.data.at("terminal").at("runtime")
          == doctest::Approx(1.0));
    CHECK(swarm.data.at("termination").at("status") == "horizon");
    CHECK(swarm.data.at("termination").at("runtime")
          == doctest::Approx(1.0));
    REQUIRE(swarm.data.at("terminal").at("robots").size() == 4);
    for (const auto &robot : swarm.data.at("terminal").at("robots")) {
        CHECK(robot.at("state").at("x").is_number());
        CHECK(robot.at("state").at("y").is_number());
        CHECK(robot.at("state").at("vx").is_number());
        CHECK(robot.at("state").at("vy").is_number());
    }
}

TEST_CASE("Full-row predictive-soft comparator reports its actual row class") {
    const auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "Gurobi")
            == available.end()) {
        WARN("Gurobi is unavailable; predictive-soft integration was not run");
        return;
    }

    const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
    ScopedTestDirectory output(
            std::filesystem::temp_directory_path()
            / ("cbf-full-row-soft-swarm-" + uniqueSuffix));
    auto settings = makeFullRowSingleLadderSwarmConfig(
            "Gurobi", output.path);
    auto &feedback = settings["bridge"]["nominal"]["gamma-star-feedback"];
    feedback["analysis-role"] = "comparator";
    feedback["constraint-execution"] = "soft";
    settings["cbfs"]["high-order"]["feasibility-slack"]["enabled"] = true;
    settings["cbfs"]["objective-function"]["k_delta"] = 1000.0;
    settings["execute"]["time-total"] = 0.1;

    Swarm swarm(settings);
    swarm.run();

    const auto &links = swarm.data.at("state").at(0)
            .at("bridge").at("nominal").at("gamma_star_feedback")
            .at("links");
    REQUIRE(links.size() == 4);
    for (const auto &link : links) {
        INFO(link.dump(2));
        CHECK(link.at("pre_score_constraint_ledger_consistent") == true);
        CHECK(link.at("pre_score_row_identity_consistent") == true);
        CHECK(link.at("pre_score_row_class_consistent") == true);
        CHECK(link.at("pre_score_row_execution_class") == "soft");
        CHECK(link.at("row_execution_class") == "soft");
        CHECK(link.at("installed_soft_row_identities").size() == 5);
        CHECK(link.at("row_identity_consistent") == true);
        CHECK(link.at("selected") == true);
        if (!link.at("selected").get<bool>()) {
            continue;
        }
        CHECK(link.at("qp_minimum_relaxed_row_margin").get<double>()
              >= -BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
        CHECK(link.at("qp_maximum_slack").get<double>() >= 0.0);
        CHECK(link.at("qp_minimum_slack").get<double>()
              >= -BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
        CHECK(link.at("qp_slack_lower_bound_violation").get<double>()
              <= BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
        CHECK(link.at("qp_slack_count") == 5);
        CHECK(link.at("qp_minimum_hard_margin").is_null());
        CHECK(link.at("execution_consistent") == true);
    }
}

TEST_CASE("Predictive-soft audits current-infeasible execution as an explicit fallback") {
    const auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "Gurobi")
            == available.end()) {
        WARN("Gurobi is unavailable; predictive-soft fallback was not run");
        return;
    }

    const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
    ScopedTestDirectory output(
            std::filesystem::temp_directory_path()
            / ("cbf-full-row-soft-fallback-" + uniqueSuffix));
    auto settings = makeFullRowSingleLadderSwarmConfig(
            "Gurobi", output.path);
    auto &feedback = settings["bridge"]["nominal"]["gamma-star-feedback"];
    feedback["analysis-role"] = "comparator";
    feedback["constraint-execution"] = "soft";
    settings["cbfs"]["high-order"]["feasibility-slack"]["enabled"] = true;
    settings["cbfs"]["objective-function"]["k_delta"] = 1000.0;
    settings["bases"] = {{200.0, 1200.0}, {200.0, 2049.9}};
    settings["initial"]["position"]["positions"] = {
            {936.0349906763944, 1624.95},
            {936.0349906763944, 775.05},
            {1672.0699813527888, 1200.0},
            {1672.0699813527888, 350.1}
    };
    settings["initial"]["velocity"]["values"][3] = {0.0, -5.0};
    settings["execute"]["time-total"] = 0.1;

    Swarm swarm(settings);
    swarm.run();

    const auto &step = swarm.data.at("state").at(0);
    CHECK(step.at("bridge").at("topology").at("certified") == true);
    const auto &links = step.at("bridge").at("nominal")
            .at("gamma_star_feedback").at("links");
    int fallbackCount = 0;
    for (const auto &link : links) {
        if (!link.at("soft_current_infeasible_fallback").get<bool>()) {
            continue;
        }
        ++fallbackCount;
        CHECK(link.at("selected") == false);
        CHECK(link.at("current_infeasible") == true);
        CHECK(link.at("certified_negative_current_hard_set") == true);
        CHECK(link.at("current_gamma_star").get<double>() < 0.0);
        CHECK(link.at("soft_fallback_task_guard_difference").get<double>()
              <= BRIDGE_FULL_ROW_GUARD_REPRODUCTION_TOLERANCE);
        CHECK(link.at("pre_score_constraint_ledger_consistent") == true);
        CHECK(link.at("row_identity_consistent") == true);
        CHECK(link.at("row_execution_class") == "soft");
        CHECK(link.at("solver_optimal") == true);
        CHECK(link.at("qp_minimum_relaxed_row_margin").get<double>()
              >= -BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
        CHECK(link.at("qp_slack_lower_bound_violation").get<double>()
              <= BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE);
        CHECK(link.at("execution_consistent") == true);
    }
    CHECK(fallbackCount >= 1);
}

TEST_CASE("Full-row budget audit matches installed tightened communication rows") {
    const auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "Gurobi")
            == available.end()) {
        WARN("Gurobi is unavailable; tightened full-row audit was not run");
        return;
    }

    const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
    ScopedTestDirectory output(
            std::filesystem::temp_directory_path()
            / ("cbf-tightened-full-row-audit-" + uniqueSuffix));
    auto settings = makeFullRowSingleLadderSwarmConfig(
            "Gurobi", output.path);
    settings["bridge"]["nominal"].erase("gamma-star-feedback");
    settings["cbfs"]["without-slack"]["comm-fixed"]
            ["range-tightening-margin"] = 1.0;
    settings["execute"]["time-total"] = 0.5;

    Swarm swarm(settings);
    swarm.run();

    REQUIRE(swarm.data.at("state").size() == 1);
    const auto &step = swarm.data.at("state").at(0);
    const auto &audits = step.at("bridge").at("nominal")
            .at("full_row_budget_audit").at("robots");
    REQUIRE(audits.size() == 4);
    for (const auto &robot : step.at("robots")) {
        const int robotId = robot.at("id").get<int>();
        std::vector<BridgeGammaStarResidual2D> residuals;
        for (const auto &row : robot.at("opt").at("hocbfNoSlack")) {
            residuals.push_back(bridgeGammaStarResidualFromAffineMargin(
                    row.at("coe").at("ax").get<double>(),
                    row.at("coe").at("ay").get<double>(),
                    row.at("const").get<double>()));
        }
        const auto exact = solveExactBridgeGammaStar2D(residuals, 4.0);
        REQUIRE(exact.valid);
        const auto audit = std::find_if(
                audits.begin(), audits.end(), [robotId](const json &candidate) {
                    return candidate.at("robot").get<int>() == robotId;
                });
        REQUIRE(audit != audits.end());
        CHECK(audit->at("current_gamma_star").get<double>()
              == doctest::Approx(exact.gamma).epsilon(1.0e-12));
    }
}

TEST_CASE("Single-ladder search stops when the target is detected") {
    const auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "Gurobi")
            == available.end()) {
        WARN("Gurobi is unavailable; task-completion termination was not run");
        return;
    }

    const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
    ScopedTestDirectory output(
            std::filesystem::temp_directory_path()
            / ("cbf-task-complete-" + uniqueSuffix));
    auto settings = makeFullRowSingleLadderSwarmConfig(
            "Gurobi", output.path);
    settings["bridge"]["search"] = {{"stop-on-detection", true}};
    settings["bridge"]["target"] = {
            {"x", settings["initial"]["position"]["positions"][0][0]},
            {"y", settings["initial"]["position"]["positions"][0][1]},
            {"radius", 1.0},
    };

    Swarm swarm(settings);
    swarm.run();

    REQUIRE(swarm.data.at("state").size() == 1);
    CHECK(swarm.data.at("state").at(0).at("bridge").at("search")
          .at("detected") == true);
    CHECK(swarm.data.at("termination").at("status") == "task-complete");
    CHECK(swarm.data.at("termination").at("runtime").get<double>()
          == doctest::Approx(0.0));
    CHECK(swarm.data.at("terminal").at("runtime").get<double>()
          == doctest::Approx(0.0));
}

TEST_CASE("Full-row prediction clock advances while fixed topology is in fail-safe") {
    const auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "Gurobi")
            == available.end()) {
        WARN("Gurobi is unavailable; fail-safe prediction clock was not run");
        return;
    }

    const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
    ScopedTestDirectory output(
            std::filesystem::temp_directory_path()
            / ("cbf-full-row-fail-safe-" + uniqueSuffix));
    auto settings = makeFullRowSingleLadderSwarmConfig(
            "Gurobi", output.path);
    settings["bases"] = {{200.0, 1200.0}, {200.0, 2049.9}};
    settings["initial"]["position"]["positions"] = {
            {936.0349906763944, 1624.95},
            {936.0349906763944, 775.05},
            {1672.0699813527888, 1200.0},
            {1672.0699813527888, 350.1}
    };
    settings["initial"]["velocity"]["values"][3] = {0.0, -0.8};
    settings["execute"]["time-total"] = 1.0;

    Swarm swarm(settings);
    swarm.run();

    REQUIRE(swarm.data.at("state").size() == 2);
    const auto &certifiedStep = swarm.data.at("state").at(0);
    CHECK(certifiedStep.at("bridge").at("topology").at("certified") == true);
    CHECK(certifiedStep.at("bridge").at("nominal").at("fail_safe")
          .at("active") == false);
    const auto &certifiedFeedback = certifiedStep.at("bridge").at("nominal")
            .at("gamma_star_feedback");
    CHECK(certifiedFeedback.at("links").size() == 4);
    CHECK(certifiedFeedback.at("prediction_audit").at("step") == 0);
    CHECK(certifiedFeedback.at("prediction_audit").at("resolved_count") == 0);
    CHECK(certifiedFeedback.at("prediction_audit").at("pending_count") == 4);

    const auto &failSafeStep = swarm.data.at("state").at(1);
    CHECK(failSafeStep.at("bridge").at("topology").at("certified") == false);
    CHECK(failSafeStep.at("bridge").at("nominal").at("fail_safe")
          .at("active") == true);
    const auto &failSafeFeedback = failSafeStep.at("bridge").at("nominal")
            .at("gamma_star_feedback");
    CHECK(failSafeFeedback.at("control_skipped_due_to_fail_safe") == true);
    CHECK(failSafeFeedback.at("links").empty());
    const auto &audit = failSafeFeedback.at("prediction_audit");
    CHECK(audit.at("step") == 1);
    CHECK(audit.at("resolved_count") == 4);
    CHECK(audit.at("invalid_count") == 0);
    CHECK(audit.at("pending_count") == 0);
    REQUIRE(audit.at("resolved").size() == 4);
    for (const auto &resolved : audit.at("resolved")) {
        CHECK(resolved.at("valid") == true);
        CHECK(resolved.at("origin_step") == 0);
        CHECK(resolved.at("due_step") == 1);
        CHECK(resolved.at("observed_step") == 1);
        CHECK(resolved.at("origin_time_s") == doctest::Approx(0.0));
        CHECK(resolved.at("predicted_time_s") == doctest::Approx(0.5));
        CHECK(resolved.at("observed_time_s") == doctest::Approx(0.5));
    }
    const auto &failSafeBudget = failSafeStep.at("bridge").at("nominal")
            .at("full_row_budget_audit");
    CHECK(failSafeBudget.at("valid") == true);
    CHECK(failSafeBudget.at("minimum_gamma_star").is_number());
    CHECK(swarm.data.at("bridge")
          .at("prediction_audit_unresolved_at_horizon") == 0);
}

TEST_CASE("RobotHocbfGuardReportsEmptyFeasibleSetWithoutChangingYawRate") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["model"] = "DoubleIntegrate2D";
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"acceleration-bound", 2.0}
    };

    Robot robot(1, settings);
    Eigen::VectorXd nominal(3);
    nominal << 0.0, 0.0, -0.2;
    robot.setNominalControlOverride(nominal);

    SecondOrderCBF lower;
    lower.name = "guardLower";
    lower.k0 = 0.0;
    lower.k1 = 0.0;
    lower.lambda1 = 1.0;
    lower.h = [](const VectorXd&, double) { return 0.0; };
    lower.hdot = [](const VectorXd&, double) { return 0.0; };
    lower.hddotConst = [](const VectorXd&, double) { return -1.0; };
    lower.uCoe = [](const VectorXd&, double) {
        VectorXd coe = VectorXd::Zero(3);
        coe(0) = 1.0;
        return coe;
    };
    SecondOrderCBF upper = lower;
    upper.name = "guardUpper";
    upper.hddotConst = [](const VectorXd&, double) { return -1.0; };
    upper.uCoe = [](const VectorXd&, double) {
        VectorXd coe = VectorXd::Zero(3);
        coe(0) = -1.0;
        return coe;
    };
    robot.secondOrderCbfNoSlack[lower.name] = lower;
    robot.secondOrderCbfNoSlack[upper.name] = upper;

    robot.applySecondOrderNominalFeasibilityGuard(1.0e-9);

    CHECK(robot.nominalControlOverride(2) == doctest::Approx(-0.2));
    REQUIRE(robot.nominalGuardDiagnostic.contains("enabled"));
    CHECK(robot.nominalGuardDiagnostic.at("enabled").get<bool>());
    CHECK_FALSE(robot.nominalGuardDiagnostic.at("feasible").get<bool>());
    CHECK_FALSE(robot.nominalGuardDiagnostic.at("active").get<bool>());
}

TEST_CASE("RobotAppliesSecondOrderAccelerationBounds") {
    auto available = getAvailableOptimisers();
    if (std::find(available.begin(), available.end(), "OSQP") == available.end()) {
        return;
    }

    json settings = makeSingleRobotNoCbfConfig("OSQP");
    settings["model"] = "DoubleIntegrate2D";
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"acceleration-bound", 0.25}
    };

    Robot robot(1, settings);

    SecondOrderCBF cbf;
    cbf.name = "manualInfeasibleSecondOrderCBF";
    cbf.k0 = 0.0;
    cbf.k1 = 0.0;
    cbf.lambda1 = 1.0;
    cbf.h = [](const VectorXd&, double) { return 0.0; };
    cbf.hdot = [](const VectorXd&, double) { return 0.0; };
    cbf.hddotConst = [](const VectorXd&, double) { return -1.0; };
    cbf.uCoe = [](const VectorXd&, double) {
        VectorXd coe = VectorXd::Zero(3);
        coe(0) = 1.0;
        return coe;
    };
    robot.secondOrderCbfNoSlack[cbf.name] = cbf;

    CHECK_THROWS_AS(robot.optimise(), std::runtime_error);
}

TEST_CASE("RobotBuildsSecondOrderFixedCommunicationCbfForDoubleIntegrator") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["model"] = "DoubleIntegrate2D";
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["cbfs"]["without-slack"]["comm-fixed"] = {
        {"on", true},
        {"max-range", 10.0},
        {"k", 1.0},
        {"min-neighbour-id-offset", -1},
        {"max-neighbour-id-offset", 0},
        {"compensate-velocity", true},
        {"consider-uncertainty", false}
    };
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"lambda1", 1.0},
        {"lambda2", 1.0}
    };

    Robot robot(2, settings);
    robot.model->setStateVariable("vx", 0.5);
    robot.model->setStateVariable("vy", 0.0);

    VectorXd otherVelocity(2);
    otherVelocity << 0.0, 0.0;
    VectorXd otherAcceleration(2);
    otherAcceleration << 0.1, 0.0;

    robot.comm->receivePosition2D(1, Point(5.0, 5.0));
    robot.comm->receiveVelocity2D(1, otherVelocity);
    robot.comm->receiveAcceleration2D(1, otherAcceleration);

    robot.postsetCBF();

    REQUIRE(robot.secondOrderCbfNoSlack.count("secondOrderFixedCommCBF(#1)") == 1);
    VectorXd x = robot.model->getX();
    auto evaluation = robot.secondOrderCbfNoSlack.at("secondOrderFixedCommCBF(#1)").evaluateConstraint(x, 0.0);

    CHECK(evaluation.h == doctest::Approx(7.0));
    CHECK(evaluation.hdot == doctest::Approx(-0.5));
    CHECK(evaluation.psi1 == doctest::Approx(6.5));
    CHECK(evaluation.uCoe.size() == 3);
    CHECK(evaluation.uCoe(0) == doctest::Approx(-1.0));
    CHECK(evaluation.uCoe(1) == doctest::Approx(0.0));
    CHECK(evaluation.uCoe(2) == doctest::Approx(0.0));
    CHECK(evaluation.constTerm == doctest::Approx(6.1));
}

TEST_CASE("SecondOrderCbfCombinesConstantAndStateDependentReserve") {
    SecondOrderCBF cbf;
    cbf.k0 = 1.0;
    cbf.k1 = 2.0;
    cbf.lambda1 = 1.0;
    cbf.sampledDataReserve = 1.0;
    cbf.h = [](const VectorXd&, double) { return 4.0; };
    cbf.hdot = [](const VectorXd&, double) { return -0.5; };
    cbf.hddotConst = [](const VectorXd&, double) { return 3.0; };
    cbf.uCoe = [](const VectorXd& x, double) { return VectorXd::Zero(x.size()); };
    cbf.stateDependentReserve = [](const VectorXd&, double) { return 2.5; };

    VectorXd x = VectorXd::Zero(3);
    auto evaluation = cbf.evaluateConstraint(x, 0.0);

    CHECK(evaluation.sampledDataReserve == doctest::Approx(1.0));
    CHECK(evaluation.stateDependentReserve == doctest::Approx(2.5));
    CHECK(evaluation.totalReserve == doctest::Approx(3.5));
    CHECK(evaluation.constTerm == doctest::Approx(2.5));
}

TEST_CASE("RobotAppliesRangeTighteningMarginToFirstOrderCommunicationCbf") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["cbfs"]["without-slack"]["comm-fixed"] = {
        {"on", true},
        {"max-range", 10.0},
        {"range-tightening-margin", 2.0},
        {"k", 1.0},
        {"min-neighbour-id-offset", -1},
        {"max-neighbour-id-offset", 0},
        {"compensate-velocity", true},
        {"consider-uncertainty", false}
    };

    Robot robot(2, settings);
    VectorXd otherVelocity(2);
    otherVelocity << 0.0, 0.0;
    robot.comm->receivePosition2D(1, Point(5.0, 5.0));
    robot.comm->receiveVelocity2D(1, otherVelocity);

    robot.postsetCBF();

    REQUIRE(robot.cbfNoSlack.cbfs.count("fixedCommCBF(#1)") == 1);
    VectorXd x = robot.model->getX();
    auto evaluation = robot.cbfNoSlack.cbfs.at("fixedCommCBF(#1)").evaluateConstraint(
        robot.model->f(), robot.model->g(), x, 0.0);

    CHECK(evaluation.h == doctest::Approx(5.0));
    CHECK(evaluation.constWithTime == doctest::Approx(0.5));
}

TEST_CASE("RobotClearsStaleFixedCommunicationCbfsWhenBridgeAnchorsChange") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 3;
    settings["all"] = {1, 2, 3};
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}, {11.0, 5.0}};
    settings["cbfs"]["without-slack"]["comm-fixed"] = {
        {"on", true},
        {"max-range", 10.0},
        {"range-tightening-margin", 0.0},
        {"k", 1.0},
        {"min-neighbour-id-offset", -1},
        {"max-neighbour-id-offset", 0},
        {"compensate-velocity", true},
        {"consider-uncertainty", false}
    };

    Robot robot(3, settings);
    VectorXd zeroVelocity(2);
    zeroVelocity << 0.0, 0.0;
    robot.comm->receivePosition2D(1, Point(5.0, 5.0));
    robot.comm->receivePosition2D(2, Point(8.0, 5.0));
    robot.comm->receiveVelocity2D(1, zeroVelocity);
    robot.comm->receiveVelocity2D(2, zeroVelocity);

    robot.setBridgeFormationOverride({1}, {});
    robot.postsetCBF();
    REQUIRE(robot.cbfNoSlack.cbfs.count("fixedCommCBF(#1)") == 1);

    robot.setBridgeFormationOverride({2}, {});
    robot.postsetCBF();

    CHECK(robot.cbfNoSlack.cbfs.count("fixedCommCBF(#1)") == 0);
    CHECK(robot.cbfNoSlack.cbfs.count("fixedCommCBF(#2)") == 1);
}

TEST_CASE("RobotAppliesRangeTighteningMarginToSecondOrderCommunicationCbf") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["model"] = "DoubleIntegrate2D";
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["cbfs"]["without-slack"]["comm-fixed"] = {
        {"on", true},
        {"max-range", 10.0},
        {"range-tightening-margin", 2.0},
        {"k", 1.0},
        {"min-neighbour-id-offset", -1},
        {"max-neighbour-id-offset", 0},
        {"compensate-velocity", true},
        {"consider-uncertainty", false}
    };
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"lambda1", 1.0},
        {"lambda2", 1.0}
    };

    Robot robot(2, settings);
    robot.model->setStateVariable("vx", 0.5);
    robot.model->setStateVariable("vy", 0.0);

    VectorXd otherVelocity(2);
    otherVelocity << 0.0, 0.0;
    VectorXd otherAcceleration(2);
    otherAcceleration << 0.1, 0.0;

    robot.comm->receivePosition2D(1, Point(5.0, 5.0));
    robot.comm->receiveVelocity2D(1, otherVelocity);
    robot.comm->receiveAcceleration2D(1, otherAcceleration);

    robot.postsetCBF();

    REQUIRE(robot.secondOrderCbfNoSlack.count("secondOrderFixedCommCBF(#1)") == 1);
    VectorXd x = robot.model->getX();
    auto evaluation = robot.secondOrderCbfNoSlack.at("secondOrderFixedCommCBF(#1)").evaluateConstraint(x, 0.0);

    CHECK(evaluation.h == doctest::Approx(5.0));
    CHECK(evaluation.hdot == doctest::Approx(-0.5));
    CHECK(evaluation.psi1 == doctest::Approx(4.5));
    CHECK(evaluation.constTerm == doctest::Approx(4.1));
}

TEST_CASE("RobotAppliesSampledDataReserveToSecondOrderCommunicationCbf") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["model"] = "DoubleIntegrate2D";
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["cbfs"]["without-slack"]["comm-fixed"] = {
        {"on", true},
        {"max-range", 10.0},
        {"range-tightening-margin", 2.0},
        {"k", 1.0},
        {"min-neighbour-id-offset", -1},
        {"max-neighbour-id-offset", 0},
        {"compensate-velocity", true},
        {"consider-uncertainty", false}
    };
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"lambda1", 1.0},
        {"lambda2", 1.0},
        {"sampled-data-reserve", 1.25}
    };

    Robot robot(2, settings);
    robot.model->setStateVariable("vx", 0.5);
    robot.model->setStateVariable("vy", 0.0);

    VectorXd otherVelocity(2);
    otherVelocity << 0.0, 0.0;
    VectorXd otherAcceleration(2);
    otherAcceleration << 0.1, 0.0;

    robot.comm->receivePosition2D(1, Point(5.0, 5.0));
    robot.comm->receiveVelocity2D(1, otherVelocity);
    robot.comm->receiveAcceleration2D(1, otherAcceleration);

    robot.postsetCBF();

    REQUIRE(robot.secondOrderCbfNoSlack.count("secondOrderFixedCommCBF(#1)") == 1);
    VectorXd x = robot.model->getX();
    auto evaluation = robot.secondOrderCbfNoSlack.at("secondOrderFixedCommCBF(#1)").evaluateConstraint(x, 0.0);

    CHECK(evaluation.sampledDataReserve == doctest::Approx(1.25));
    CHECK(evaluation.constTerm == doctest::Approx(2.85));

    robot.optimise();

    REQUIRE(robot.opt.at("hocbfNoSlack").size() == 1);
    CHECK(robot.opt.at("hocbfNoSlack").at(0).at("sampledDataReserve").get<double>() == doctest::Approx(1.25));
}

TEST_CASE("RobotAppliesStateDependentReserveToSecondOrderCommunicationCbf") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["model"] = "DoubleIntegrate2D";
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["cbfs"]["without-slack"]["comm-fixed"] = {
        {"on", true},
        {"max-range", 10.0},
        {"range-tightening-margin", 2.0},
        {"k", 1.0},
        {"min-neighbour-id-offset", -1},
        {"max-neighbour-id-offset", 0},
        {"compensate-velocity", true},
        {"consider-uncertainty", false},
        {"state-dependent-reserve", {
            {"enabled", true},
            {"velocity-gain", 2.0},
            {"sample-time", 0.5},
            {"acceleration-gain", 1.0},
            {"neighbor-acceleration-bound", 4.0},
            {"max-reserve", 20.0}
        }}
    };
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"lambda1", 1.0},
        {"lambda2", 1.0},
        {"sampled-data-reserve", 1.25}
    };

    Robot robot(2, settings);
    robot.model->setStateVariable("vx", 0.5);
    robot.model->setStateVariable("vy", 0.0);

    VectorXd otherVelocity(2);
    otherVelocity << 0.0, 0.0;
    VectorXd otherAcceleration(2);
    otherAcceleration << 0.1, 0.0;

    robot.comm->receivePosition2D(1, Point(5.0, 5.0));
    robot.comm->receiveVelocity2D(1, otherVelocity);
    robot.comm->receiveAcceleration2D(1, otherAcceleration);

    robot.postsetCBF();

    REQUIRE(robot.secondOrderCbfNoSlack.count("secondOrderFixedCommCBF(#1)") == 1);
    VectorXd x = robot.model->getX();
    auto evaluation = robot.secondOrderCbfNoSlack.at("secondOrderFixedCommCBF(#1)").evaluateConstraint(x, 0.0);

    CHECK(evaluation.sampledDataReserve == doctest::Approx(1.25));
    CHECK(evaluation.stateDependentReserve == doctest::Approx(1.0));
    CHECK(evaluation.totalReserve == doctest::Approx(2.25));
    CHECK(evaluation.constTerm == doctest::Approx(1.85));

    const auto sharedRow = buildPairwiseSecondOrderRow(
        {Point(8.0, 5.0), Eigen::Vector2d(0.5, 0.0), Eigen::Vector2d::Zero()},
        {Point(5.0, 5.0), Eigen::Vector2d::Zero(), Eigen::Vector2d(0.1, 0.0)},
        {PairwiseSecondOrderBarrierKind::CommunicationUpper,
         8.0, 0.0, 1.0, 1.0, 1.0, 2.25});
    CHECK(evaluation.h == doctest::Approx(sharedRow.h).epsilon(1.0e-12));
    CHECK(evaluation.hdot == doctest::Approx(sharedRow.hdot).epsilon(1.0e-12));
    CHECK(evaluation.psi1 == doctest::Approx(sharedRow.psi1).epsilon(1.0e-12));
    CHECK(evaluation.uCoe(0) == doctest::Approx(sharedRow.uCoe(0)).epsilon(1.0e-12));
    CHECK(evaluation.uCoe(1) == doctest::Approx(sharedRow.uCoe(1)).epsilon(1.0e-12));
    CHECK(evaluation.constTerm == doctest::Approx(sharedRow.constTerm).epsilon(1.0e-12));
}

TEST_CASE("SwarmLogOnceEvaluatesCentralizedCbfsFromSingleStateSnapshot") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["execute"] = {
        {"execution-mode", "centralized"},
        {"time-step", 0.5},
        {"time-total", 0.5}
    };

    Swarm swarm(settings);
    swarm.centralizedModel->updateConcatenatedStates();

    CBF mutatingCBF;
    mutatingCBF.name = "centralMutatingCBF";
    mutatingCBF.h = [&](VectorXd x, double) {
        Eigen::VectorXd robotState = swarm.centralizedModel->getRobotState(0);
        robotState[0] = 9.0;
        swarm.centralizedModel->setRobotStates(0, robotState);
        return x[0];
    };
    swarm.cbfNoSlack.cbfs[mutatingCBF.name] = mutatingCBF;

    CBF snapshotCBF;
    snapshotCBF.name = "centralSnapshotCBF";
    snapshotCBF.h = [&](VectorXd x, double) {
        return x[0];
    };
    swarm.cbfSlack[snapshotCBF.name] = snapshotCBF;

    swarm.logOnce();

    const json &cbfs = swarm.data.at("state").at(0).at("centralized").at("cbfs");
    CHECK(cbfs.at("centralMutatingCBF").get<double>() == doctest::Approx(5.0));
    CHECK(cbfs.at("centralSnapshotCBF").get<double>() == doctest::Approx(5.0));
}

TEST_CASE("SwarmCentralizedRunLogsSecondOrderSafetyCbfForDoubleIntegrator") {
    const std::string optimiser_name = selectRobotTestOptimiser();
    json settings = makeSingleRobotNoCbfConfig(optimiser_name);
    settings["num"] = 2;
    settings["all"] = {1, 2};
    settings["model"] = "DoubleIntegrate2D";
    settings["output_path"] = "/private/tmp/cbf-second-order-test";
    settings["initial"]["position"]["positions"] = {{5.0, 5.0}, {8.0, 5.0}};
    settings["searching"] = {
        {"method", "downward"},
        {"downward", {
            {"radius", 1.0}
        }}
    };
    settings["execute"] = {
        {"execution-mode", "centralized"},
        {"time-step", 0.5},
        {"time-total", 0.5},
        {"check-constraint-violation", false}
    };
    settings["cbfs"]["without-slack"]["safety"] = {
        {"on", true},
        {"safe-distance", 1.0},
        {"consider-uncertainty", false}
    };
    settings["cbfs"]["high-order"] = {
        {"enabled", true},
        {"lambda1", 1.0},
        {"lambda2", 1.0}
    };

    Swarm swarm(settings);
    swarm.robots[0]->model->setStateVariable("vx", 0.5);
    swarm.robots[0]->model->setStateVariable("vy", 0.0);
    swarm.run();

    const json &centralized = swarm.data.at("state").at(0).at("centralized");
    REQUIRE(centralized.contains("hocbfNoSlack"));
    REQUIRE(centralized.at("hocbfNoSlack").size() >= 1);
    const json &item = centralized.at("hocbfNoSlack").at(0);
    CHECK(item.at("h").get<double>() == doctest::Approx(2.0));
    CHECK(item.at("psi1").get<double>() == doctest::Approx(1.5));
    CHECK(item.contains("hocbf"));
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
