#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif
#include "doctest/doctest.h"
#include "Swarm.hpp"

#include <memory>
#include <stdexcept>
#include <vector>

namespace {

json makeDiagnosticSettings() {
    return {
        {"world", {
            {"boundary", json::array({
                json::array({-100.0, -100.0}),
                json::array({100.0, -100.0}),
                json::array({100.0, 100.0}),
                json::array({-100.0, 100.0})
            })},
            {"charge", json::array()},
            {"spacing", 1.0}
        }},
        {"num", 2},
        {"all", json::array({1, 2})},
        {"dim", 2},
        {"formation", {
            {"parts", 1},
            {"bases-id", json::array({json::array({0, 1})})}
        }},
        {"bases", json::array({json::array({-20.0, 0.0}), json::array({0.0, -20.0})})},
        {"initial", {
            {"position", {
                {"method", "specified"},
                {"positions", json::array({json::array({0.0, 0.0}), json::array({2.0, 0.0})})}
            }},
            {"battery", {{"min", 4000.0}, {"max", 4000.0}}},
            {"yawDeg", 0.0}
        }},
        {"model", "SingleIntegrate2D"},
        {"optimiser", "Gurobi"},
        {"position_covariance", {
            {"enable", true},
            {"ranging_sigma", 1.0},
            {"uncertainty-type", "std_avg"}
        }},
        {"cbfs", {
            {"objective-function", {{"k_delta", 1.0}}},
            {"with-slack", {
                {"cvt", {{"on", false}}},
                {"cvt-yaw", {{"on", false}}},
                {"target-yaw", {{"on", false}}}
            }},
            {"without-slack", {
                {"method", "all"},
                {"comm-fixed", {
                    {"on", true},
                    {"max-range", 100.0},
                    {"k", 1.0},
                    {"compensate-velocity", false},
                    {"consider-uncertainty", true},
                    {"min-neighbour-id-offset", -2},
                    {"max-neighbour-id-offset", 0},
                    {"alpha", {{"coe", 0.1}, {"pow", 1}}}
                }},
                {"comm-auto", {{"on", false}}},
                {"safety", {{"on", false}}}
            }}
        }},
        {"debug", {{"opt-cbc", false}}},
        {"execute", {{"time-step", 0.5}}}
    };
}

Robot makeDiagnosticRobot(int id = 1) {
    json settings = makeDiagnosticSettings();
    return Robot(id, settings);
}

std::vector<std::unique_ptr<Robot>> makeTwoDiagnosticRobots() {
    json firstSettings = makeDiagnosticSettings();
    json secondSettings = makeDiagnosticSettings();
    std::vector<std::unique_ptr<Robot>> robots;
    robots.emplace_back(std::make_unique<Robot>(1, firstSettings));
    robots.emplace_back(std::make_unique<Robot>(2, secondSettings));
    return robots;
}

void exchangeDiagnosticData(std::vector<std::unique_ptr<Robot>>& robots) {
    for (const auto& robot : robots) {
        const Point position = robot->model->xy();
        const VectorXd velocity = robot->model->getVelocity();
        const double yaw = robot->model->getStateVariable("yawRad");
        const double battery = robot->model->getStateVariable("battery");

        for (const auto& receiver : robots) {
            receiver->comm->receivePosition2D(robot->id, position);
            receiver->comm->receiveVelocity2D(robot->id, velocity);
            receiver->comm->receiveYawRad(robot->id, yaw);
            receiver->comm->receiveBatteryLevel(robot->id, battery);
            receiver->comm->receivePositionCovariance(robot->id, robot->positionCovariance);
            receiver->comm->receiveUncertaintyRate(robot->id, robot->uncertaintyRate);
        }
    }
}

void refreshDiagnosticUncertaintySnapshot(std::vector<std::unique_ptr<Robot>>& robots, double dt) {
    for (const auto& robot : robots) {
        robot->updateCovarianceAndRate(dt);
    }
    exchangeDiagnosticData(robots);
}

}  // namespace

TEST_CASE("positive backward uncertainty rate ignores decreases") {
    Robot robot = makeDiagnosticRobot();
    robot.hasUncertaintyHistory = true;
    robot.previousUncertainty = 10.0;
    CHECK(robot.positiveBackwardUncertaintyRate(12.0, 0.5) == doctest::Approx(4.0));
    CHECK(robot.positiveBackwardUncertaintyRate(8.0, 0.5) == doctest::Approx(0.0));
    CHECK_THROWS_AS(robot.positiveBackwardUncertaintyRate(12.0, 0.0), std::invalid_argument);
}

TEST_CASE("first covariance sample has zero uncertainty rate") {
    Robot robot = makeDiagnosticRobot();
    robot.hasUncertaintyHistory = false;
    robot.updateUncertaintyHistory(7.0, 0.5);
    CHECK(robot.currentUncertainty == doctest::Approx(7.0));
    CHECK(robot.uncertaintyRate == doctest::Approx(0.0));
}

TEST_CASE("communicator exchanges the current uncertainty rate") {
    json settings = makeDiagnosticSettings();
    settings["id"] = 1;
    CommunicatorCentral communicator(settings);
    communicator.receiveUncertaintyRate(3, 1.25);
    CHECK(communicator._othersUncertaintyRate.at(3) == doctest::Approx(1.25));
}

TEST_CASE("two-phase refresh publishes current covariance and rate before CBF construction") {
    auto robots = makeTwoDiagnosticRobots();
    exchangeDiagnosticData(robots);
    refreshDiagnosticUncertaintySnapshot(robots, 0.5);
    CHECK(robots[0]->comm->_othersPositionCovariance.at(2).isApprox(
        robots[1]->positionCovariance
    ));
    CHECK(robots[0]->comm->_othersUncertaintyRate.at(2)
          == doctest::Approx(robots[1]->uncertaintyRate));

    robots[0]->setupFormation();
    const auto covarianceBefore = robots[0]->positionCovariance;
    const auto rateBefore = robots[0]->uncertaintyRate;
    robots[0]->setFixedCommCBF(
        robots[0]->settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    CHECK(robots[0]->positionCovariance.isApprox(covarianceBefore));
    CHECK(robots[0]->uncertaintyRate == doctest::Approx(rateBefore));
}

TEST_CASE("covariance refresh is independent of comm-fixed enforcement") {
    auto robots = makeTwoDiagnosticRobots();
    robots[0]->settings["cbfs"]["without-slack"]["comm-fixed"]["on"] = false;
    exchangeDiagnosticData(robots);
    refreshDiagnosticUncertaintySnapshot(robots, 0.5);
    CHECK(robots[0]->hasUncertaintyHistory);
}

TEST_CASE("centralized startup refreshes snapshots before fixed CBF construction") {
    json settings = makeDiagnosticSettings();
    settings["execute"]["execution-mode"] = "centralized";
    settings["execute"]["time-total"] = 0.0;
    settings["output_path"] = "build-diagnostic";

    Swarm swarm(settings);
    swarm.run();

    for (const auto& robot : swarm.robots) {
        CHECK(robot->hasUncertaintyHistory);
        CHECK(robot->currentUncertainty > 0.0);
    }
    CHECK(swarm.robots[0]->comm->_othersPositionCovariance.at(2).isApprox(
        swarm.robots[1]->positionCovariance
    ));
    REQUIRE(swarm.robots[0]->cbfNoSlack.cbfs.count("fixedCommCBF(base-0)") == 1);
    const CBF& initialFixedCbf =
        swarm.robots[0]->cbfNoSlack.cbfs.at("fixedCommCBF(base-0)");
    CHECK(initialFixedCbf.h(swarm.robots[0]->model->getX(), 0.0)
          == doctest::Approx(79.0));
}
