#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif
#include "doctest/doctest.h"
#include "Swarm.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
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

json makeTheoremSettings() {
    json settings = makeDiagnosticSettings();
    settings["cbfs"]["uncertainty-rate"]["mode"] =
        "analytic-topological";
    settings["cbfs"]["input-limits"] = {
        {"on", true},
        {"planar-component-max", 25.0},
        {"yaw-rate-max", 0.35}
    };
    return settings;
}

Robot paperRobotWithCommittedHardProblem() {
    json settings = makeTheoremSettings();
    settings["cbfs"]["hard-interior-selection"] = {
        {"mode", "planar-chebyshev-fraction-cap-v1"},
        {"fraction", 0.1},
        {"cap-mps", 0.1},
        {"feasibility-tolerance-mps", 1e-9}
    };
    Robot robot(1, settings);
    cbf2026::HardConstraintProblem problem;
    problem.owner = robot.id;
    problem.controlSize = 3;
    problem.planarComponentMax = 25.0;
    problem.snapshotVersion = 8;
    problem.allocationVersion = 1;
    problem.yawRateMax = 0.35;
    problem.bounds = cbf2026::theoremInputBounds();

    cbf2026::HardConstraintRow lower;
    lower.owner = robot.id;
    lower.name = "local-planar-lower";
    lower.coefficients = Eigen::Vector3d(1.0, 0.0, 0.0);
    lower.constant = 0.0;
    lower.snapshotVersion = 8;
    lower.allocationVersion = 1;
    cbf2026::HardConstraintRow upper = lower;
    upper.name = "local-planar-upper";
    upper.coefficients = Eigen::Vector3d(-1.0, 0.0, 0.0);
    upper.constant = 1.0;
    problem.rows = {lower, upper};

    cbf2026::CommittedCertificateState state;
    state.version = 8;
    state.nodeVersions.emplace(robot.id, 8);
    state.valid = true;
    state.hardProblems.emplace(robot.id, problem);
    robot.committedCertificateState = state;
    return robot;
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

json makePaperTopologySettings() {
    json settings = makeTheoremSettings();
    settings["num"] = 14;
    settings["all"] = json::array();
    settings["formation"]["parts"] = 2;
    settings["formation"]["bases-id"] = json::array({
        json::array({1, 0}),
        json::array({1, 2})
    });
    settings["bases"] = json::array({
        json::array({-20.0, -80.0}),
        json::array({-20.0, 0.0}),
        json::array({-20.0, 80.0})
    });
    settings["initial"]["position"]["positions"] = json::array();
    for (int id = 1; id <= 14; ++id) {
        settings["all"].push_back(id);
        const int localIndex = (id - 1) % 7;
        const double y = id <= 7 ? -50.0 : 50.0;
        settings["initial"]["position"]["positions"].push_back(
            json::array({10.0 * localIndex, y + localIndex})
        );
    }
    settings["world"]["boundary"] = json::array({
        json::array({-200.0, -200.0}),
        json::array({200.0, -200.0}),
        json::array({200.0, 200.0}),
        json::array({-200.0, 200.0})
    });
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] =
        1000.0;
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

cbf2026::NodeRateCertificate makeAxisRateCertificate(
    int robotId,
    std::uint64_t snapshotVersion,
    double componentMax = 25.0
) {
    return cbf2026::computeNodeRateCertificate({
        robotId,
        1,
        snapshotVersion,
        componentMax,
        {
            {
                {cbf2026::canonicalBaseReferenceId(0), 0,
                 snapshotVersion, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {cbf2026::canonicalBaseReferenceId(1), 0,
                 snapshotVersion, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            }
        }
    });
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

TEST_CASE("communicator atomically exchanges the complete endpoint certificate") {
    json settings = makeDiagnosticSettings();
    settings["id"] = 1;
    CommunicatorCentral communicator(settings);
    const auto certificate = cbf2026::computeNodeRateCertificate({
        3,
        1,
        7,
        25.0,
        {
            {
                {cbf2026::canonicalBaseReferenceId(0), 0, 7, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {cbf2026::canonicalBaseReferenceId(1), 0, 7, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            }
        }
    });
    const auto snapshot = cbf2026::makeEndpointCertificateSnapshot(
        certificate,
        Eigen::Vector2d(4.0, 5.0),
        11
    );

    communicator.receiveEndpointCertificateSnapshot(3, snapshot);

    REQUIRE(communicator._othersEndpointCertificateSnapshots.count(3) == 1);
    const auto& received =
        communicator._othersEndpointCertificateSnapshots.at(3);
    CHECK(received.robotId == 3);
    CHECK(received.estimate == Eigen::Vector2d(4.0, 5.0));
    CHECK(received.covariance == certificate.covariance);
    CHECK(received.covarianceRateBound
          == certificate.covarianceRateBound);
    CHECK(received.epsilon == certificate.epsilon);
    CHECK(received.barNu == certificate.epsilonRateBound);
    CHECK(received.snapshotVersion == 7);
    CHECK(received.allocationVersion == 11);
    CHECK(communicator._othersEpsilon.at(3) == received.epsilon);
    CHECK(communicator._othersBarNu.at(3) == received.barNu);
    CHECK(communicator._othersCovarianceRateBound.at(3)
          == received.covarianceRateBound);
    CHECK(communicator._othersCertificateSnapshotVersion.at(3) == 7);
    CHECK(communicator._othersAllocationVersion.at(3) == 11);
}

TEST_CASE("atomic endpoint transport rejects wrong identity and invalid fields") {
    json settings = makeDiagnosticSettings();
    settings["id"] = 1;
    CommunicatorCentral communicator(settings);
    const auto valid = cbf2026::makeEndpointCertificateSnapshot(
        makeAxisRateCertificate(3, 7),
        Eigen::Vector2d(4.0, 5.0),
        11
    );

    auto invalid = valid;
    invalid.robotId = 4;
    CHECK_THROWS_AS(
        communicator.receiveEndpointCertificateSnapshot(3, invalid),
        std::invalid_argument
    );
    CHECK(communicator._othersEndpointCertificateSnapshots.empty());

    invalid = valid;
    invalid.covarianceRateBound = -1.0;
    CHECK_THROWS_AS(
        communicator.receiveEndpointCertificateSnapshot(3, invalid),
        std::invalid_argument
    );
    CHECK(communicator._othersEndpointCertificateSnapshots.empty());

    invalid = valid;
    invalid.estimate[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(
        communicator.receiveEndpointCertificateSnapshot(3, invalid),
        std::invalid_argument
    );
    CHECK(communicator._othersEndpointCertificateSnapshots.empty());
}

TEST_CASE("topological recursion consumes transported predecessor covariance rate") {
    json settings = makeTheoremSettings();
    Robot robot(2, settings);
    robot.certificateSnapshotVersion = 7;
    robot.certificateAllocationVersion = 11;

    const auto slower = cbf2026::makeEndpointCertificateSnapshot(
        makeAxisRateCertificate(1, 7, 5.0),
        Eigen::Vector2d(0.0, 0.0),
        11
    );
    const auto faster = cbf2026::makeEndpointCertificateSnapshot(
        makeAxisRateCertificate(1, 7, 25.0),
        Eigen::Vector2d(0.0, 0.0),
        11
    );
    REQUIRE(slower.covariance == faster.covariance);
    REQUIRE(slower.covarianceRateBound
            < faster.covarianceRateBound);

    robot.comm->receiveEndpointCertificateSnapshot(1, slower);
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    const Eigen::Matrix2d slowerCovariance = robot.positionCovariance;
    const double slowerBound = robot.rateCertificate.covarianceRateBound;

    robot.comm->receiveEndpointCertificateSnapshot(1, faster);
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );

    CHECK(robot.positionCovariance == slowerCovariance);
    CHECK(robot.rateCertificate.covarianceRateBound > slowerBound);
}

TEST_CASE("local endpoint certificate freezes the estimate at certificate computation") {
    json settings = makeTheoremSettings();
    Robot robot(1, settings);
    robot.certificateSnapshotVersion = 7;
    robot.certificateAllocationVersion = 11;
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    REQUIRE(robot.certificateAvailable);

    robot.model->setPosition2D(Point(30.0, 40.0));
    const auto snapshot = robot.localEndpointCertificateSnapshot();

    CHECK(snapshot.estimate == Eigen::Vector2d(0.0, 0.0));
    CHECK(snapshot.snapshotVersion == 7);
    CHECK(snapshot.allocationVersion == 11);
    CHECK(snapshot.covariance == robot.rateCertificate.covariance);
    CHECK(snapshot.covarianceRateBound
          == robot.rateCertificate.covarianceRateBound);
    CHECK(snapshot.epsilon == robot.rateCertificate.epsilon);
    CHECK(snapshot.barNu == robot.rateCertificate.epsilonRateBound);
}

TEST_CASE("failed certificate refresh clears the frozen local endpoint snapshot") {
    json settings = makeTheoremSettings();
    Robot robot(1, settings);
    robot.certificateSnapshotVersion = 7;
    robot.certificateAllocationVersion = 11;
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    REQUIRE(robot.certificateAvailable);

    robot.settings["cbfs"]["input-limits"]["planar-component-max"] = 24.0;
    CHECK_THROWS_AS(
        robot.getCovariance(
            robot.settings["cbfs"]["without-slack"]["comm-fixed"]
        ),
        std::invalid_argument
    );
    REQUIRE_FALSE(robot.certificateAvailable);

    robot.certificateAvailable = true;
    CHECK_THROWS_AS(
        robot.localEndpointCertificateSnapshot(),
        std::invalid_argument
    );
    CHECK_FALSE(robot.certificateAvailable);
}

TEST_CASE("theorem endpoint rows are byte-independent of neighbor commands") {
    json settings = makeTheoremSettings();
    settings["cbfs"]["without-slack"]["safety"]["mode"] = "pairwise";
    Robot robot(2, settings);
    robot.certificateSnapshotVersion = 7;
    robot.certificateAllocationVersion = 11;
    const auto predecessor = cbf2026::makeEndpointCertificateSnapshot(
        makeAxisRateCertificate(1, 7),
        Eigen::Vector2d(0.0, 0.0),
        11
    );
    robot.comm->receiveEndpointCertificateSnapshot(1, predecessor);
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    REQUIRE(robot.certificateAvailable);

    VectorXd firstVelocity(2);
    firstVelocity << 24.0, -17.0;
    robot.comm->receiveVelocity2D(1, firstVelocity);
    robot.setupFormation();
    robot.setFixedCommCBF(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    robot.setSafetyCBF(
        robot.settings["cbfs"]["without-slack"]["safety"]
    );
    const VectorXd x = robot.model->getX();
    const auto firstLocalizationCoefficient =
        robot.cbfNoSlack.cbfs.at("fixedCommCBF(#1)")
            .constraintUCoe(robot.model->f(), robot.model->g(), x, 0.0);
    const double firstLocalizationConstant =
        robot.cbfNoSlack.cbfs.at("fixedCommCBF(#1)")
            .constraintConstWithTime(
                robot.model->f(), robot.model->g(), x, 0.0
            );
    const auto firstCollisionCoefficient =
        robot.cbfNoSlack.cbfs.at("safetyCBF(#1)")
            .constraintUCoe(robot.model->f(), robot.model->g(), x, 0.0);
    const double firstCollisionConstant =
        robot.cbfNoSlack.cbfs.at("safetyCBF(#1)")
            .constraintConstWithTime(
                robot.model->f(), robot.model->g(), x, 0.0
            );

    VectorXd secondVelocity(2);
    secondVelocity << -25.0, 25.0;
    robot.comm->receiveVelocity2D(1, secondVelocity);
    robot.setFixedCommCBF(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    robot.setSafetyCBF(
        robot.settings["cbfs"]["without-slack"]["safety"]
    );
    const auto secondLocalizationCoefficient =
        robot.cbfNoSlack.cbfs.at("fixedCommCBF(#1)")
            .constraintUCoe(robot.model->f(), robot.model->g(), x, 0.0);
    const double secondLocalizationConstant =
        robot.cbfNoSlack.cbfs.at("fixedCommCBF(#1)")
            .constraintConstWithTime(
                robot.model->f(), robot.model->g(), x, 0.0
            );
    const auto secondCollisionCoefficient =
        robot.cbfNoSlack.cbfs.at("safetyCBF(#1)")
            .constraintUCoe(robot.model->f(), robot.model->g(), x, 0.0);
    const double secondCollisionConstant =
        robot.cbfNoSlack.cbfs.at("safetyCBF(#1)")
            .constraintConstWithTime(
                robot.model->f(), robot.model->g(), x, 0.0
            );

    CHECK((firstLocalizationCoefficient.array()
           == secondLocalizationCoefficient.array()).all());
    CHECK(firstLocalizationConstant == secondLocalizationConstant);
    CHECK((firstCollisionCoefficient.array()
           == secondCollisionCoefficient.array()).all());
    CHECK(firstCollisionConstant == secondCollisionConstant);
    REQUIRE(secondLocalizationCoefficient.size() == 3);
    REQUIRE(secondCollisionCoefficient.size() == 3);
    CHECK(secondLocalizationCoefficient[2] == 0.0);
    CHECK(secondCollisionCoefficient[2] == 0.0);

    const double separation = std::sqrt(5.0);
    const Eigen::Vector2d canonicalNormal(
        -2.0 / separation,
        -1.0 / separation
    );
    CHECK(secondLocalizationCoefficient.head<2>().isApprox(
        canonicalNormal, 1e-12
    ));
    CHECK(secondCollisionCoefficient.head<2>().isApprox(
        -canonicalNormal, 1e-12
    ));
    const double localizationBarrier =
        100.0 - separation
        - robot.rateCertificate.epsilon
        - predecessor.epsilon;
    const double collisionBarrier =
        separation - 1.0
        - robot.rateCertificate.epsilon
        - predecessor.epsilon;
    CHECK(secondLocalizationConstant == doctest::Approx(
        -robot.rateCertificate.epsilonRateBound
        + 0.5 * 0.1 * localizationBarrier
    ).epsilon(1e-12));
    CHECK(secondCollisionConstant == doctest::Approx(
        -robot.rateCertificate.epsilonRateBound
        + 0.5 * 0.1 * collisionBarrier
    ).epsilon(1e-12));
}

TEST_CASE("theorem hard rows fail closed on missing or mismatched snapshots") {
    json settings = makeTheoremSettings();
    settings["cbfs"]["without-slack"]["safety"]["mode"] = "pairwise";
    Robot robot(2, settings);
    robot.certificateSnapshotVersion = 7;
    robot.certificateAllocationVersion = 11;
    const auto predecessor = cbf2026::makeEndpointCertificateSnapshot(
        makeAxisRateCertificate(1, 7),
        Eigen::Vector2d(0.0, 0.0),
        11
    );
    robot.comm->receiveEndpointCertificateSnapshot(1, predecessor);
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    robot.setupFormation();

    robot.comm->_othersEndpointCertificateSnapshots.erase(1);
    CHECK_THROWS_AS(
        robot.setFixedCommCBF(
            robot.settings["cbfs"]["without-slack"]["comm-fixed"]
        ),
        std::invalid_argument
    );
    CHECK_FALSE(robot.certificateAvailable);

    robot.certificateAvailable = true;
    robot.comm->_othersEndpointCertificateSnapshots[1] = predecessor;
    robot.comm->_othersEndpointCertificateSnapshots[1].allocationVersion = 12;
    CHECK_THROWS_AS(
        robot.setSafetyCBF(
            robot.settings["cbfs"]["without-slack"]["safety"]
        ),
        std::invalid_argument
    );
    CHECK_FALSE(robot.certificateAvailable);

    robot.certificateAvailable = true;
    robot.comm->_othersEndpointCertificateSnapshots[1] = predecessor;
    robot.comm->_othersEndpointCertificateSnapshots[1].robotId = 2;
    CHECK_THROWS_AS(
        robot.setSafetyCBF(
            robot.settings["cbfs"]["without-slack"]["safety"]
        ),
        std::invalid_argument
    );
    CHECK_FALSE(robot.certificateAvailable);
}

TEST_CASE("theorem Robot uses fixed localization edges and every collision pair") {
    json settings = makeFourRobotFixedGraphSettings();
    settings["cbfs"]["uncertainty-rate"]["mode"] =
        "analytic-topological";
    settings["cbfs"]["input-limits"] = {
        {"on", true},
        {"planar-component-max", 25.0},
        {"yaw-rate-max", 0.35}
    };
    settings["cbfs"]["without-slack"]["safety"]["mode"] = "minimum";
    Robot robot(4, settings);
    robot.certificateSnapshotVersion = 7;
    robot.certificateAllocationVersion = 11;
    const std::array<Eigen::Vector2d, 3> estimates = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 1.0),
        Eigen::Vector2d(4.0, 3.0)
    };
    for (int otherId = 1; otherId <= 3; ++otherId) {
        robot.comm->receiveEndpointCertificateSnapshot(
            otherId,
            cbf2026::makeEndpointCertificateSnapshot(
                makeAxisRateCertificate(otherId, 7),
                estimates[otherId - 1],
                11
            )
        );
    }
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    REQUIRE(robot.myCovarianceFormation.at("anchorIds")
            == json::array({1, 2, 3}));

    robot.setupFormation();
    robot.setFixedCommCBF(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    CHECK(robot.cbfNoSlack.cbfs.count("fixedCommCBF(#1)") == 0);
    CHECK(robot.cbfNoSlack.cbfs.count("fixedCommCBF(#2)") == 1);
    CHECK(robot.cbfNoSlack.cbfs.count("fixedCommCBF(#3)") == 1);

    robot.setSafetyCBF(
        robot.settings["cbfs"]["without-slack"]["safety"]
    );
    CHECK(robot.cbfNoSlack.cbfs.count("safetyCBF(#1)") == 1);
    CHECK(robot.cbfNoSlack.cbfs.count("safetyCBF(#2)") == 1);
    CHECK(robot.cbfNoSlack.cbfs.count("safetyCBF(#3)") == 1);
}

TEST_CASE("optional FIM-only additions leave the fixed edge registry unchanged") {
    json settings = makeFourRobotFixedGraphSettings();
    settings["cbfs"]["uncertainty-rate"]["mode"] =
        "analytic-topological";
    settings["cbfs"]["input-limits"] = {
        {"on", true},
        {"planar-component-max", 25.0},
        {"yaw-rate-max", 0.35}
    };
    Robot robot(4, settings);
    robot.certificateSnapshotVersion = 7;
    robot.certificateAllocationVersion = 11;
    robot.comm->receiveEndpointCertificateSnapshot(
        2,
        cbf2026::makeEndpointCertificateSnapshot(
            makeAxisRateCertificate(2, 7),
            Eigen::Vector2d(2.0, 1.0),
            11
        )
    );
    robot.comm->receiveEndpointCertificateSnapshot(
        3,
        cbf2026::makeEndpointCertificateSnapshot(
            makeAxisRateCertificate(3, 7),
            Eigen::Vector2d(4.0, 3.0),
            11
        )
    );
    const auto before = robot.fixedBarrierEdgeRegistry();

    robot.comm->receiveEndpointCertificateSnapshot(
        1,
        cbf2026::makeEndpointCertificateSnapshot(
            makeAxisRateCertificate(1, 7),
            Eigen::Vector2d(0.0, 0.0),
            11
        )
    );
    const json active = robot.activeLocalizationReferences(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    const auto after = robot.fixedBarrierEdgeRegistry();

    CHECK(std::find(
        active.at("anchorIds").begin(),
        active.at("anchorIds").end(),
        json(1)
    ) != active.at("anchorIds").end());
    CHECK(before.fixedLocalizationEdges()
          == after.fixedLocalizationEdges());
    CHECK(before.collisionEdges() == after.collisionEdges());
    const auto fimOnly = cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Localization, 1, 4
    );
    CHECK(std::find(
        after.fixedLocalizationEdges().begin(),
        after.fixedLocalizationEdges().end(),
        fimOnly
    ) == after.fixedLocalizationEdges().end());
}

TEST_CASE("paper-topology Robots install all fifty reverse-incident localization rows") {
    json settings = makePaperTopologySettings();
    std::vector<std::unique_ptr<Robot>> robots;
    for (int id = 1; id <= 14; ++id) {
        json robotSettings = settings;
        robots.emplace_back(std::make_unique<Robot>(id, robotSettings));
        robots.back()->certificateSnapshotVersion = 7;
        robots.back()->certificateAllocationVersion = 11;
    }

    std::array<cbf2026::EndpointCertificateSnapshot, 14> snapshots;
    for (int id = 1; id <= 14; ++id) {
        Robot& robot = *robots[id - 1];
        for (int otherId = 1; otherId < id; ++otherId) {
            robot.comm->receiveEndpointCertificateSnapshot(
                otherId, snapshots[otherId - 1]
            );
        }
        robot.getCovariance(
            robot.settings["cbfs"]["without-slack"]["comm-fixed"]
        );
        REQUIRE(robot.certificateAvailable);
        snapshots[id - 1] = robot.localEndpointCertificateSnapshot();
    }

    for (int id = 1; id <= 14; ++id) {
        Robot& robot = *robots[id - 1];
        for (int otherId = 1; otherId <= 14; ++otherId) {
            if (otherId == id) {
                continue;
            }
            robot.comm->receiveEndpointCertificateSnapshot(
                otherId, snapshots[otherId - 1]
            );
        }
        robot.setupFormation();
        robot.setFixedCommCBF(
            robot.settings["cbfs"]["without-slack"]["comm-fixed"]
        );
    }

    std::array<int, 14> ownerRows{};
    int totalRows = 0;
    for (int id = 1; id <= 14; ++id) {
        for (const auto& [name, cbf] : robots[id - 1]->cbfNoSlack.cbfs) {
            if (name.rfind("fixedCommCBF(", 0) != 0) {
                continue;
            }
            ++ownerRows[id - 1];
            ++totalRows;
        }
    }
    const std::array<int, 14> expectedOwnerRows = {
        4, 4, 4, 4, 4, 3, 2,
        4, 4, 4, 4, 4, 3, 2
    };
    CHECK(totalRows == 50);
    CHECK(ownerRows == expectedOwnerRows);
    CHECK(robots[6]->myCovarianceFormation.at("anchorIds").size() == 6);
    CHECK(ownerRows[6] == 2);
    CHECK(robots[0]->myCovarianceFormation.at("baseIds").size() == 3);
    CHECK(robots[0]->cbfNoSlack.cbfs.count("fixedCommCBF(base-2)") == 0);
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

TEST_CASE("theorem task QP enforces the frozen local interior floor") {
    Robot robot = paperRobotWithCommittedHardProblem();
    const std::string committedHash = cbf2026::canonicalHardConstraintProblemHash(
        robot.committedCertificateState.hardProblems.at(robot.id)
    );
    robot.optimise();

    REQUIRE(robot.opt.contains("hard_interior_selection"));
    const auto& policy = robot.opt.at("hard_interior_selection");
    CHECK(policy.at("mode") == "planar-chebyshev-fraction-cap-v1");
    CHECK(policy.at("fraction") == doctest::Approx(0.10));
    CHECK(policy.at("cap_mps") == doctest::Approx(0.10));
    CHECK(policy.at("feasibility_tolerance_mps") == doctest::Approx(1e-9));
    const double floor = policy.at("enforced_floor_mps");
    CHECK(floor == doctest::Approx(0.05));
    CHECK(floor <= policy.at("planar_chebyshev_radius_mps"));
    REQUIRE(robot.lastConsumedHardConstraintProblem.has_value());
    CHECK(cbf2026::canonicalHardConstraintProblemHash(
        *robot.lastConsumedHardConstraintProblem
    ) == committedHash);
    for (const auto& row : robot.lastConsumedHardConstraintProblem->rows) {
        CHECK(row.constant + row.coefficients.dot(robot.model->getControlInput())
              >= floor - 1e-7);
    }
}

TEST_CASE("interior selection consumes no other UAV control variable") {
    Robot robot = paperRobotWithCommittedHardProblem();
    robot.optimise();

    REQUIRE(robot.lastConsumedHardConstraintProblem.has_value());
    CHECK(robot.lastConsumedHardConstraintProblem->owner == robot.id);
    CHECK(robot.model->getControlInput().size() == 3);
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
    highestIndexRobot.getCovariance(
        highestIndexRobot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    CHECK(highestIndexRobot.positionCovariance.isApprox(
        startupCovariance, 1e-12
    ));
}

TEST_CASE("covariance FIM includes eligible in-range references outside the fixed CBF graph") {
    json settings = makeFourRobotFixedGraphSettings();
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(3);

    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);

    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array({2, 3, 1}));
    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array({0, 1, 2}));
}

TEST_CASE("covariance FIM rejects collinear fixed lower-index references") {
    auto robots = makeFourFixedGraphRobots();
    robots.at(1)->model->setPosition2D(Point(2.0, 0.0));
    robots.at(2)->model->setPosition2D(Point(4.0, 0.0));
    robots.at(3)->model->setPosition2D(Point(6.0, 0.0));
    exchangeDiagnosticData(robots);

    Robot& robot = *robots.at(3);
    robot.settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 1.0;

    bool threw = false;
    try {
        robot.getCovariance(
            robot.settings["cbfs"]["without-slack"]["comm-fixed"]
        );
    } catch (const std::invalid_argument& error) {
        threw = true;
        const std::string message = error.what();
        CHECK(message.find("#4") != std::string::npos);
        CHECK(message.find("FIM") != std::string::npos);
    }
    CHECK(threw);
}

TEST_CASE("covariance FIM rejects a non-positive configured ranging variance") {
    json settings = makeDiagnosticSettings();
    settings["position_covariance"]["ranging_sigma"] = 0.0;
    Robot robot(1, settings);

    bool threw = false;
    try {
        robot.getCovariance(
            robot.settings["cbfs"]["without-slack"]["comm-fixed"]
        );
    } catch (const std::invalid_argument& error) {
        threw = true;
        const std::string message = error.what();
        CHECK(message.find("#1") != std::string::npos);
        CHECK(message.find("total variance") != std::string::npos);
    }
    CHECK(threw);
}

TEST_CASE("covariance FIM retains the non-collinear hand calculation beyond max range") {
    json settings = makeDiagnosticSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 5.0;
    Robot robot(1, settings);

    CHECK_NOTHROW(robot.getCovariance(
        settings["cbfs"]["without-slack"]["comm-fixed"]
    ));
    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array());
    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array({0, 1}));
    CHECK(robot.positionCovariance.isApprox(
        Eigen::Matrix2d::Identity(), 1e-12
    ));
}

TEST_CASE("covariance FIM removes optional anchors that leave range") {
    json settings = makeFourRobotFixedGraphSettings();
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(3);

    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);
    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array({2, 3, 1}));

    robots.at(0)->model->setPosition2D(Point(1500.0, 1500.0));
    exchangeDiagnosticData(robots);
    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);

    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array({2, 3}));
}

TEST_CASE("covariance FIM keeps assigned bases beyond range and gates unassigned bases") {
    json settings = makeFourRobotFixedGraphSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 5.0;
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(1);

    robot.bases.at(0) = Point(-20.0, 0.0);
    robot.bases.at(1) = Point(2.0, 0.0);
    robot.bases.at(2) = Point(1000.0, 1000.0);

    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);
    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array({0, 1}));

    robot.bases.at(1) = Point(1000.0, 1000.0);
    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);

    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array({0}));
}

TEST_CASE("active covariance references deduplicate duplicated assigned bases") {
    json settings = makeDiagnosticSettings();
    settings["formation"]["bases-id"] = json::array({json::array({0, 0, 1})});
    Robot robot(1, settings);

    CHECK(robot.fixedLocalizationReferences().at("baseIds")
          == json::array({0, 0}));

    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);

    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array({0, 1}));
}

TEST_CASE("optional anchors change the covariance FIM while in range") {
    json settings = makeFourRobotFixedGraphSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 3.0;
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(3);

    robots.at(0)->model->setPosition2D(Point(2.0, 2.0));
    robots.at(1)->model->setPosition2D(Point(0.0, 2.0));
    robots.at(2)->model->setPosition2D(Point(2.0, 0.0));
    robots.at(3)->model->setPosition2D(Point(0.0, 0.0));
    robot.bases.at(0) = Point(1000.0, 1000.0);
    robot.bases.at(1) = Point(1000.0, 1001.0);
    robot.bases.at(2) = Point(1001.0, 1000.0);
    exchangeDiagnosticData(robots);
    for (int otherId : {1, 2, 3}) {
        robot.comm->receivePositionCovariance(
            otherId, Eigen::Matrix2d::Zero()
        );
    }

    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);

    const Eigen::Matrix2d dynamicExpected =
        (Eigen::Matrix2d() << 0.75, -0.25, -0.25, 0.75).finished();
    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array({2, 3, 1}));
    CHECK(robot.positionCovariance.isApprox(dynamicExpected, 1e-12));

    robots.at(0)->model->setPosition2D(Point(10.0, 10.0));
    exchangeDiagnosticData(robots);
    for (int otherId : {1, 2, 3}) {
        robot.comm->receivePositionCovariance(
            otherId, Eigen::Matrix2d::Zero()
        );
    }
    robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);

    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array({2, 3}));
    CHECK(robot.positionCovariance.isApprox(Eigen::Matrix2d::Identity(), 1e-12));
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
