#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "Robot.hpp"

#include <algorithm>

namespace {

class FakeInfeasibleOptimiser : public OptimiserBase {
public:
    explicit FakeInfeasibleOptimiser(json& settings) : OptimiserBase(settings) {}

    void clear() override {}
    void start(int, int) override {}
    void setObjective(Eigen::VectorXd&) override {}
    void addLinearConstraint(Eigen::VectorXd, double) override {}

    Eigen::VectorXd solve() override {
        return (Eigen::Vector3d() << 2.0, -3.0, 0.5).finished();
    }

    void write(std::string) override {}
    double getObjectiveValue() const override { return 0.0; }
    json getStatus() const override { return {{"status", "infeasible"}}; }
};

class FakeThrowingStatusOptimiser : public FakeInfeasibleOptimiser {
public:
    explicit FakeThrowingStatusOptimiser(json& settings) : FakeInfeasibleOptimiser(settings) {}

    json getStatus() const override {
        throw std::logic_error("status unavailable");
    }
};

json makeSingleRobotNoCbfConfig(const std::string& optimiser) {
    return {
        {"world", {
            {"boundary", json::array({
                json::array({-2.0, -2.0}),
                json::array({2.0, -2.0}),
                json::array({2.0, 2.0}),
                json::array({-2.0, 2.0})
            })},
            {"charge", json::array()},
            {"spacing", 1.0}
        }},
        {"num", 1},
        {"all", json::array({1})},
        {"dim", 2},
        {"formation", {{"parts", 1}, {"bases-id", json::array({json::array({0})})}}},
        {"bases", json::array()},
        {"initial", {
            {"position", {{"method", "specified"}, {"positions", json::array({json::array({1.0, 1.0})})}}},
            {"battery", {{"min", 4000.0}, {"max", 4000.0}}},
            {"yawDeg", 0.0}
        }},
        {"model", "SingleIntegrate2D"},
        {"optimiser", optimiser},
        {"cbfs", {
            {"objective-function", {{"k_delta", 1.0}}},
            {"with-slack", {
                {"cvt", {{"on", false}}},
                {"cvt-yaw", {{"on", false}}},
                {"target-yaw", {{"on", false}}}
            }},
            {"without-slack", {
                {"method", "all"},
                {"comm-fixed", {{"on", false}}}
            }}
        }},
        {"debug", {{"opt-cbc", false}}},
        {"execute", {{"time-step", 0.1}}}
    };
}

}  // namespace

TEST_CASE("RobotUsesZeroNominalWhenNoPolicyIsConfigured") {
    const auto available = getAvailableOptimisers();
    REQUIRE(std::find(available.begin(), available.end(), "Gurobi") != available.end());

    json settings = makeSingleRobotNoCbfConfig("Gurobi");
    Robot robot(1, settings);
    robot.optimise();
    Eigen::VectorXd control = robot.model->getControlInput();
    REQUIRE(control.size() == 3);
    CHECK(control[0] == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(control[1] == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(control[2] == doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("RobotDoesNotApplyControlFromNonOptimalSolve") {
    json settings = makeSingleRobotNoCbfConfig("Gurobi");
    Robot robot(1, settings);
    robot.model->setControlInput((Eigen::Vector3d() << 0.25, -0.5, 0.1).finished());
    robot.optimiser = std::make_unique<FakeInfeasibleOptimiser>(settings["cbfs"]["objective-function"]);

    CHECK_THROWS_AS(robot.optimise(), std::runtime_error);
    Eigen::VectorXd control = robot.model->getControlInput();
    CHECK(control[0] == doctest::Approx(0.25));
    CHECK(control[1] == doctest::Approx(-0.5));
    CHECK(control[2] == doctest::Approx(0.1));
    CHECK(robot.opt.at("status") == "failed");
}

TEST_CASE("RobotRecordsFailureWhenSolverStatusQueryThrows") {
    json settings = makeSingleRobotNoCbfConfig("Gurobi");
    Robot robot(1, settings);
    robot.model->setControlInput((Eigen::Vector3d() << 0.4, -0.2, 0.3).finished());
    robot.optimiser = std::make_unique<FakeThrowingStatusOptimiser>(
        settings["cbfs"]["objective-function"]);

    CHECK_THROWS_AS(robot.optimise(), std::runtime_error);
    Eigen::VectorXd control = robot.model->getControlInput();
    CHECK(control[0] == doctest::Approx(0.4));
    CHECK(control[1] == doctest::Approx(-0.2));
    CHECK(control[2] == doctest::Approx(0.3));
    CHECK(robot.opt.value("status", "missing") == "failed");
    CHECK(robot.opt.value("error", "missing") == "Status check failed");
    CHECK(robot.opt.value("solver_info", json::object()).value("status", "missing")
          == "status-check-error");
}
