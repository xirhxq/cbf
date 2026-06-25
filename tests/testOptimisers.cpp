#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif

#include "doctest.h"
#include "Robot.hpp"
#include "Swarm.hpp"
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
