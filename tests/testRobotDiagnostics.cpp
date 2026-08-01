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

json makeTheoremRateConfig() {
    json settings = makeSingleRobotNoCbfConfig("Gurobi");
    settings["world"]["boundary"] = json::array({
        json::array({-50.0, -50.0}),
        json::array({50.0, -50.0}),
        json::array({50.0, 50.0}),
        json::array({-50.0, 50.0})
    });
    settings["initial"]["position"]["positions"] =
        json::array({json::array({0.0, 0.0})});
    settings["bases"] = json::array({
        json::array({-10.0, 0.0}),
        json::array({0.0, -10.0})
    });
    settings["formation"]["bases-id"] =
        json::array({json::array({0, 1})});
    settings["position_covariance"] = {
        {"enable", true},
        {"ranging_sigma", 2.0},
        {"uncertainty-type", "std_avg"}
    };
    settings["cbfs"]["uncertainty-rate"] = {
        {"mode", "analytic-topological"}
    };
    settings["cbfs"]["input-limits"] = {
        {"on", true},
        {"planar-component-max", 25.0},
        {"yaw-rate-max", 0.35}
    };
    settings["cbfs"]["without-slack"]["comm-fixed"] = {
        {"on", true},
        {"max-range", 100.0},
        {"k", 1.0},
        {"compensate-velocity", false},
        {"consider-uncertainty", true},
        {"min-neighbour-id-offset", -2},
        {"max-neighbour-id-offset", 0},
        {"alpha", {{"coe", 0.1}, {"pow", 1}}}
    };
    return settings;
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

TEST_CASE("theorem covariance entry point installs the exact rate certificate state") {
    json settings = makeTheoremRateConfig();
    Robot robot(1, settings);
    robot.certificateSnapshotVersion = 7;
    const auto independentlyComputed =
        cbf2026::computeNodeRateCertificate({
            robot.id,
            robot.idInMyPart,
            7,
            25.0,
            {
                {
                    {cbf2026::canonicalBaseReferenceId(0), 0, 7, true,
                     Eigen::Matrix2d::Zero(), 0.0, 0.0},
                    10.0,
                    Eigen::Vector2d::UnitX(),
                    4.0
                },
                {
                    {cbf2026::canonicalBaseReferenceId(1), 0, 7, true,
                     Eigen::Matrix2d::Zero(), 0.0, 0.0},
                    10.0,
                    Eigen::Vector2d::UnitY(),
                    4.0
                }
            }
        });

    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );

    CHECK(robot.certificateAvailable);
    CHECK(robot.rateCertificate.robotId == robot.id);
    CHECK(robot.rateCertificate.snapshotVersion == 7);
    CHECK((robot.rateCertificate.information.array()
           == independentlyComputed.information.array()).all());
    CHECK((robot.positionCovariance.array()
           == independentlyComputed.covariance.array()).all());
    CHECK(robot.currentUncertainty == independentlyComputed.epsilon);

    robot.positionCovariance.setConstant(99.0);
    robot.currentUncertainty = -1.0;
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );

    CHECK((robot.positionCovariance.array()
           == independentlyComputed.covariance.array()).all());
    CHECK(robot.currentUncertainty == independentlyComputed.epsilon);

    robot.bases[1] = robot.bases[0];
    CHECK_THROWS_AS(
        robot.getCovariance(
            robot.settings["cbfs"]["without-slack"]["comm-fixed"]
        ),
        std::invalid_argument
    );
    CHECK_FALSE(robot.certificateAvailable);
}

TEST_CASE("theorem hard row uses analytic epsilon rate not backward history") {
    json settings = makeTheoremRateConfig();
    Robot robot(1, settings);
    robot.certificateSnapshotVersion = 7;
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    REQUIRE(robot.certificateAvailable);
    robot.setupFormation();

    auto& commConfig =
        robot.settings["cbfs"]["without-slack"]["comm-fixed"];
    robot.previousUncertainty = -1000.0;
    robot.uncertaintyRate = 1000.0;
    robot.setFixedCommCBF(commConfig);
    const Eigen::VectorXd state = robot.model->getX();
    const double first = robot.cbfNoSlack.cbfs.at(
        "fixedCommCBF(base-0)"
    ).dhdt(state, 0.0);

    robot.previousUncertainty = 1000.0;
    robot.uncertaintyRate = 0.0;
    robot.setFixedCommCBF(commConfig);
    const double second = robot.cbfNoSlack.cbfs.at(
        "fixedCommCBF(base-0)"
    ).dhdt(state, 0.0);

    CHECK(first == doctest::Approx(
        -robot.rateCertificate.epsilonRateBound
    ).epsilon(1e-12));
    CHECK(second == doctest::Approx(first).epsilon(1e-12));
}

TEST_CASE("theorem certificate requires the frozen componentwise QP bound") {
    json settings = makeTheoremRateConfig();

    SUBCASE("input limits are disabled") {
        settings["cbfs"]["input-limits"]["on"] = false;
        Robot robot(1, settings);
        CHECK_THROWS_AS(
            robot.getCovariance(
                robot.settings["cbfs"]["without-slack"]["comm-fixed"]
            ),
            std::invalid_argument
        );
        CHECK_FALSE(robot.certificateAvailable);
    }

    SUBCASE("planar component limit differs from 25") {
        settings["cbfs"]["input-limits"]["planar-component-max"] = 24.0;
        Robot robot(1, settings);
        CHECK_THROWS_AS(
            robot.getCovariance(
                robot.settings["cbfs"]["without-slack"]["comm-fixed"]
            ),
            std::invalid_argument
        );
        CHECK_FALSE(robot.certificateAvailable);
    }
}

TEST_CASE("legacy covariance ignores a disabled zero component limit") {
    json settings = makeTheoremRateConfig();
    settings["cbfs"]["uncertainty-rate"]["mode"] = "off";
    settings["cbfs"]["input-limits"]["on"] = false;
    settings["cbfs"]["input-limits"]["planar-component-max"] = 0.0;
    Robot robot(1, settings);
    robot.currentUncertainty = 13.0;

    CHECK_NOTHROW(robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    ));
    CHECK(robot.positionCovariance.isApprox(
        4.0 * Eigen::Matrix2d::Identity(),
        1e-12
    ));
    CHECK(robot.currentUncertainty == doctest::Approx(13.0));
    CHECK_FALSE(robot.certificateAvailable);
}

TEST_CASE("theorem refresh preserves descriptive backward uncertainty history") {
    json settings = makeTheoremRateConfig();
    Robot robot(1, settings);
    robot.certificateSnapshotVersion = 7;
    robot.hasUncertaintyHistory = true;
    robot.previousUncertainty = 1.0;
    robot.currentUncertainty = 1.0;
    robot.uncertaintyRate = 99.0;

    robot.updateCovarianceAndRate(0.5);

    CHECK(robot.previousUncertainty == doctest::Approx(1.0));
    CHECK(robot.currentUncertainty == doctest::Approx(6.0));
    CHECK(robot.uncertaintyRate == doctest::Approx(10.0));
    CHECK(robot.rateCertificate.epsilonRateBound
          > robot.uncertaintyRate);
}

TEST_CASE("theorem recursion consumes predecessor covariance rate certificate") {
    json settings = makeTheoremRateConfig();
    settings["num"] = 2;
    settings["all"] = json::array({1, 2});
    settings["initial"]["position"]["positions"] = json::array({
        json::array({0.0, 0.0}),
        json::array({5.0, 5.0})
    });
    Robot robot(2, settings);
    robot.certificateSnapshotVersion = 7;
    robot.comm->receivePosition2D(1, Point(0.0, 0.0));

    auto makePredecessor = [](double componentMax) {
        return cbf2026::computeNodeRateCertificate({
            1,
            1,
            7,
            componentMax,
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
    };
    const auto slowerPredecessor = makePredecessor(5.0);
    const auto fasterPredecessor = makePredecessor(25.0);
    REQUIRE((slowerPredecessor.covariance.array()
             == fasterPredecessor.covariance.array()).all());
    REQUIRE(slowerPredecessor.covarianceRateBound
            < fasterPredecessor.covarianceRateBound);

    robot.comm->receiveEndpointCertificateSnapshot(
        1,
        cbf2026::makeEndpointCertificateSnapshot(
            slowerPredecessor,
            Eigen::Vector2d(0.0, 0.0),
            robot.certificateAllocationVersion
        )
    );
    robot.uncertaintyRate = 10000.0;
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    const Eigen::Matrix2d slowerCovariance = robot.positionCovariance;
    const double slowerBound = robot.rateCertificate.covarianceRateBound;

    robot.comm->receiveEndpointCertificateSnapshot(
        1,
        cbf2026::makeEndpointCertificateSnapshot(
            fasterPredecessor,
            Eigen::Vector2d(0.0, 0.0),
            robot.certificateAllocationVersion
        )
    );
    robot.uncertaintyRate = 0.0;
    robot.getCovariance(
        robot.settings["cbfs"]["without-slack"]["comm-fixed"]
    );

    CHECK(robot.positionCovariance.isApprox(slowerCovariance, 1e-12));
    CHECK(robot.rateCertificate.covarianceRateBound > slowerBound);
}
