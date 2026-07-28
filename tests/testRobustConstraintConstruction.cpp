#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif
#include "doctest/doctest.h"
#include "Swarm.hpp"

#include <cmath>
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
                {"positions", json::array({json::array({0.0, 0.0}), json::array({2.0, 1.0})})}
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
            {"uncertainty-rate", {{"mode", "backward-difference-positive"}}},
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
                {"safety", {
                    {"on", false},
                    {"mode", "minimum"},
                    {"safe-distance", 1.0},
                    {"k", 1.0},
                    {"consider-uncertainty", true},
                    {"alpha", {{"coe", 0.1}, {"pow", 1}}}
                }}
            }}
        }},
        {"debug", {{"opt-cbc", false}}},
        {"execute", {{"time-step", 0.5}}}
    };
}

json makeThreeRobotChainSettings() {
    json settings = makeDiagnosticSettings();
    settings["num"] = 3;
    settings["all"] = json::array({1, 2, 3});
    settings["initial"]["position"]["positions"] = json::array({
        json::array({0.0, 0.0}),
        json::array({2.0, 1.0}),
        json::array({4.0, 3.0})
    });
    settings["cbfs"]["without-slack"]["energy"] = {{"on", false}};
    settings["searching"] = {
        {"method", "downward"},
        {"downward", {{"radius", 5.0}}}
    };
    settings["execute"]["check-constraint-violation"] = false;
    settings["output_path"] = "build-diagnostic";
    return settings;
}

json makeFourRobotFixedGraphSettings() {
    json settings = makeThreeRobotChainSettings();
    settings["num"] = 4;
    settings["all"] = json::array({1, 2, 3, 4});
    settings["world"]["boundary"] = json::array({
        json::array({-2000.0, -2000.0}),
        json::array({2000.0, -2000.0}),
        json::array({2000.0, 2000.0}),
        json::array({-2000.0, 2000.0})
    });
    settings["initial"]["position"]["positions"] = json::array({
        json::array({0.0, 0.0}),
        json::array({2.0, 1.0}),
        json::array({4.0, 3.0}),
        json::array({6.0, 6.0})
    });
    settings["bases"] = json::array({
        json::array({-20.0, 0.0}),
        json::array({0.0, -20.0}),
        json::array({10.0, 10.0})
    });
    settings["formation"]["bases-id"] =
        json::array({json::array({0, 1, 2})});
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 100.0;
    return settings;
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

std::vector<std::unique_ptr<Robot>> makeFourFixedGraphRobots() {
    json settings = makeFourRobotFixedGraphSettings();
    std::vector<std::unique_ptr<Robot>> robots;
    for (int id = 1; id <= 4; ++id) {
        json robotSettings = settings;
        robots.emplace_back(std::make_unique<Robot>(id, robotSettings));
    }
    exchangeDiagnosticData(robots);
    return robots;
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

TEST_CASE("rate-aware communication row uses the hand-derived neighbor projection") {
    json settings = makeDiagnosticSettings();
    settings["initial"]["position"]["positions"] =
        json::array({json::array({0.0, 0.0}), json::array({3.0, 4.0})});
    settings["cbfs"]["without-slack"]["comm-fixed"]["compensate-velocity"] = true;
    Robot robot(2, settings);
    robot.currentUncertainty = 0.5;
    robot.uncertaintyRate = 0.4;

    VectorXd neighborVelocity(2);
    neighborVelocity << 2.0, -1.0;
    robot.comm->receivePosition2D(1, Point(0.0, 0.0));
    robot.comm->receiveVelocity2D(1, neighborVelocity);
    robot.comm->receivePositionCovariance(1, Eigen::Matrix2d::Zero());
    robot.comm->receiveUncertaintyRate(1, 0.3);
    robot.setupFormation();
    robot.setFixedCommCBF(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );

    CBF& localizationCBF =
        robot.cbfNoSlack.cbfs.at("fixedCommCBF(#1)");
    CHECK(localizationCBF.dhdt(robot.model->getX(), 0.0)
          == doctest::Approx(-0.3));
    CHECK(localizationCBF.getAlphaCoefficient() == doctest::Approx(0.1));
    CHECK(localizationCBF.getAlphaPower() == 1);
}

TEST_CASE("rate-aware collision row matches the hand-derived closing derivative") {
    json settings = makeDiagnosticSettings();
    settings["initial"]["position"]["positions"] =
        json::array({json::array({0.0, 0.0}), json::array({3.0, 4.0})});
    Robot robot(2, settings);
    robot.currentUncertainty = 0.5;
    robot.uncertaintyRate = 0.4;

    VectorXd neighborVelocity(2);
    neighborVelocity << 2.0, -1.0;
    robot.comm->receivePosition2D(1, Point(0.0, 0.0));
    robot.comm->receiveVelocity2D(1, neighborVelocity);
    robot.comm->receivePositionCovariance(1, Eigen::Matrix2d::Zero());
    robot.comm->receiveUncertaintyRate(1, 0.3);
    robot.setSafetyCBF(robot.settings["cbfs"]["without-slack"]["safety"]);

    REQUIRE(robot.cbfNoSlack.cbfs.count("safetyCBF(#1)") == 1);
    CBF& collisionCBF =
        robot.cbfNoSlack.cbfs.at("safetyCBF(#1)");
    CHECK(collisionCBF.dhdt(robot.model->getX(), 0.0)
          == doctest::Approx(-1.1));
    CHECK(collisionCBF.getAlphaCoefficient() == doctest::Approx(0.1));
    CHECK(collisionCBF.getAlphaPower() == 1);
}

TEST_CASE("pairwise safety mode creates two stable named rows for three UAVs") {
    json settings = makeDiagnosticSettings();
    settings["num"] = 3;
    settings["all"] = json::array({1, 2, 3});
    settings["initial"]["position"]["positions"] = json::array({
        json::array({3.0, 4.0}),
        json::array({0.0, 0.0}),
        json::array({9.0, 12.0})
    });
    settings["cbfs"]["without-slack"]["safety"]["mode"] = "pairwise";
    Robot robot(1, settings);

    VectorXd stopped = VectorXd::Zero(2);
    for (int otherId : {2, 3}) {
        const Point otherPosition =
            otherId == 2 ? Point(0.0, 0.0) : Point(9.0, 12.0);
        robot.comm->receivePosition2D(otherId, otherPosition);
        robot.comm->receiveVelocity2D(otherId, stopped);
        robot.comm->receivePositionCovariance(
            otherId, Eigen::Matrix2d::Zero()
        );
        robot.comm->receiveUncertaintyRate(otherId, 0.0);
    }
    robot.setSafetyCBF(robot.settings["cbfs"]["without-slack"]["safety"]);

    CHECK(robot.cbfNoSlack.cbfs.size() == 2);
    CHECK(robot.cbfNoSlack.cbfs.count("safetyCBF(#2)") == 1);
    CHECK(robot.cbfNoSlack.cbfs.count("safetyCBF(#3)") == 1);
}

TEST_CASE("minimum safety mode keeps only the smallest current robust pair") {
    json settings = makeDiagnosticSettings();
    settings["num"] = 3;
    settings["all"] = json::array({1, 2, 3});
    settings["initial"]["position"]["positions"] = json::array({
        json::array({3.0, 4.0}),
        json::array({0.0, 0.0}),
        json::array({30.0, 40.0})
    });
    Robot robot(1, settings);

    VectorXd stopped = VectorXd::Zero(2);
    robot.comm->receivePosition2D(2, Point(0.0, 0.0));
    robot.comm->receivePosition2D(3, Point(30.0, 40.0));
    for (int otherId : {2, 3}) {
        robot.comm->receiveVelocity2D(otherId, stopped);
        robot.comm->receivePositionCovariance(
            otherId, Eigen::Matrix2d::Zero()
        );
        robot.comm->receiveUncertaintyRate(otherId, 0.0);
    }
    robot.setSafetyCBF(robot.settings["cbfs"]["without-slack"]["safety"]);

    CHECK(robot.cbfNoSlack.cbfs.size() == 1);
    CHECK(robot.cbfNoSlack.cbfs.count("safetyCBF(#2)") == 1);
}

TEST_CASE("disabling uncertainty also disables rate tightening") {
    json settings = makeDiagnosticSettings();
    settings["initial"]["position"]["positions"] =
        json::array({json::array({0.0, 0.0}), json::array({3.0, 4.0})});
    settings["cbfs"]["without-slack"]["comm-fixed"]["compensate-velocity"] = true;
    settings["cbfs"]["without-slack"]["comm-fixed"]["consider-uncertainty"] = false;
    settings["cbfs"]["without-slack"]["safety"]["consider-uncertainty"] = false;
    Robot robot(2, settings);
    robot.currentUncertainty = 2.0;
    robot.uncertaintyRate = 0.4;

    VectorXd neighborVelocity(2);
    neighborVelocity << 2.0, -1.0;
    robot.comm->receivePosition2D(1, Point(0.0, 0.0));
    robot.comm->receiveVelocity2D(1, neighborVelocity);
    robot.comm->receivePositionCovariance(
        1, 9.0 * Eigen::Matrix2d::Identity()
    );
    robot.comm->receiveUncertaintyRate(1, 0.3);
    robot.setupFormation();
    robot.setFixedCommCBF(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    robot.setSafetyCBF(robot.settings["cbfs"]["without-slack"]["safety"]);

    const VectorXd x = robot.model->getX();
    CBF& localizationCBF =
        robot.cbfNoSlack.cbfs.at("fixedCommCBF(#1)");
    REQUIRE(robot.cbfNoSlack.cbfs.count("safetyCBF(#1)") == 1);
    CBF& collisionCBF =
        robot.cbfNoSlack.cbfs.at("safetyCBF(#1)");
    CHECK(localizationCBF.h(x, 0.0) == doctest::Approx(95.0));
    CHECK(localizationCBF.dhdt(x, 0.0) == doctest::Approx(0.4));
    CHECK(collisionCBF.h(x, 0.0) == doctest::Approx(4.0));
    CHECK(collisionCBF.dhdt(x, 0.0) == doctest::Approx(-0.4));
}

TEST_CASE("base communication row treats anchor velocity and rate as zero") {
    json settings = makeDiagnosticSettings();
    settings["initial"]["position"]["positions"] =
        json::array({json::array({0.0, 0.0}), json::array({3.0, 4.0})});
    Robot robot(2, settings);
    robot.currentUncertainty = 0.5;
    robot.uncertaintyRate = 0.4;
    robot.myFormation = {
        {"baseIds", json::array({0})},
        {"anchorIds", json::array()}
    };
    robot.setFixedCommCBF(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );

    CBF& baseCBF =
        robot.cbfNoSlack.cbfs.at("fixedCommCBF(base-0)");
    CHECK(baseCBF.dhdt(robot.model->getX(), 0.0)
          == doctest::Approx(-0.4));
}

TEST_CASE("hard input rows bound a strong planar soft task inside the QP") {
    json settings = makeDiagnosticSettings();
    settings["cbfs"]["objective-function"]["k_delta"] = 100.0;
    settings["cbfs"]["input-limits"] = {
        {"on", true},
        {"planar-component-max", 25.0},
        {"yaw-rate-max", 0.35}
    };
    Robot robot(1, settings);

    CBF strongDistanceTask;
    strongDistanceTask.name = "strong-distance-task";
    strongDistanceTask.h = [](VectorXd x, double) {
        return x[0] + x[1] - 200.0;
    };
    strongDistanceTask.dhdx_analytical = [](VectorXd, double) {
        VectorXd gradient(4);
        gradient << 1.0, 1.0, 0.0, 0.0;
        return gradient;
    };
    strongDistanceTask.setAlphaClassK(1.0, 1);
    robot.cbfSlack[strongDistanceTask.name] = strongDistanceTask;

    robot.optimise();
    const VectorXd control = robot.model->getControlInput();
    CAPTURE(control.transpose());
    CHECK(std::abs(control[0]) <= 25.0 + 1e-7);
    CHECK(std::abs(control[1]) <= 25.0 + 1e-7);
    CHECK(std::abs(control[2]) <= 0.35 + 1e-7);
    CHECK(robot.opt["input_limits"]["bound_row_count"] == 6);
    CHECK(robot.opt["input_limits"]["saturation_tolerance"]
          == doctest::Approx(1e-7));
    CHECK(robot.opt["input_limits"]["saturated"]["vx"]);
    CHECK(robot.opt["input_limits"]["saturated"]["vy"]);
    CHECK(robot.opt["input_limits"]["saturated"]["any"]);
}

TEST_CASE("state logging exposes the uncertainty snapshot consumed by the controller") {
    Robot robot = makeDiagnosticRobot();
    robot.currentUncertainty = 7.25;
    robot.uncertaintyRate = 1.5;

    const json state = robot.getState();
    CHECK(state.at("uncertainty").get<double>()
          == doctest::Approx(7.25));
    CHECK(state.at("uncertainty_rate").get<double>()
          == doctest::Approx(1.5));
}

TEST_CASE("startup bootstraps a self-consistent lower-index covariance chain") {
    json settings = makeThreeRobotChainSettings();
    settings["execute"]["execution-mode"] = "centralized";
    settings["execute"]["time-total"] = 0.0;
    settings["run_suffix"] = "-covariance-bootstrap";

    Swarm swarm(settings);
    swarm.run();

    Robot& highestIndexRobot = *swarm.robots.at(2);
    const Eigen::Matrix2d startupCovariance =
        highestIndexRobot.positionCovariance;
    highestIndexRobot.getCovariance();
    CHECK(highestIndexRobot.positionCovariance.isApprox(
        startupCovariance, 1e-12
    ));
}

TEST_CASE("covariance FIM excludes in-range references outside the fixed localization graph") {
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(3);

    robot.getCovariance();

    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array({2, 3}));
    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array());
}

TEST_CASE("covariance FIM retains declared base references beyond max range") {
    json settings = makeDiagnosticSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 5.0;
    Robot robot(1, settings);

    CHECK_NOTHROW(robot.getCovariance());
    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array());
    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array({0, 1}));
    CHECK(robot.positionCovariance.isApprox(
        Eigen::Matrix2d::Identity(), 1e-12
    ));
}

TEST_CASE("covariance information set is invariant across optional-anchor range crossing") {
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(3);
    robot.getCovariance();
    const json before = robot.myCovarianceFormation;

    robots.at(0)->model->setPosition2D(Point(1500.0, 1500.0));
    exchangeDiagnosticData(robots);
    robot.getCovariance();

    CHECK(robot.myCovarianceFormation == before);
}

TEST_CASE("distributed frame zero logs only first-sample zero uncertainty rates") {
    json settings = makeThreeRobotChainSettings();
    settings["execute"]["execution-mode"] = "distributed";
    settings["execute"]["time-total"] = 0.5;
    settings["cbfs"]["without-slack"]["comm-fixed"]["on"] = false;
    settings["run_suffix"] = "-frame-zero-rate";

    Swarm swarm(settings);
    swarm.run();

    REQUIRE(swarm.data.at("state").size() == 1);
    const json& frameZeroRobots = swarm.data.at("state").at(0).at("robots");
    REQUIRE(frameZeroRobots.size() == 3);
    for (const json& robot : frameZeroRobots) {
        CHECK(robot.at("uncertainty_rate").get<double>()
              == doctest::Approx(0.0));
    }
}
