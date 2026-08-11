#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "cbf/CBF.hpp"
#include "cbf/PairwiseSecondOrderCBF.hpp"
#include "cbf/SecondOrderCBF.hpp"

TEST_CASE("CBFConstraintEvaluationReusesDerivativeTerms") {
    CBF cbf;
    int h_calls = 0;
    int dhdx_calls = 0;
    int dhdt_calls = 0;

    cbf.setAlphaClassK(2.0, 1);
    cbf.h = [&](VectorXd x, double t) {
        ++h_calls;
        return x.sum() + t;
    };
    cbf.dhdx_analytical = [&](VectorXd, double) {
        ++dhdx_calls;
        VectorXd gradient(2);
        gradient << 2.0, -1.0;
        return gradient;
    };
    cbf.dhdt_analytical = [&](VectorXd, double) {
        ++dhdt_calls;
        return 0.5;
    };

    VectorXd f(2);
    f << 3.0, 4.0;
    MatrixXd g(2, 2);
    g << 1.0, 2.0,
         0.0, 1.0;
    VectorXd x(2);
    x << 1.0, 5.0;

    auto evaluation = cbf.evaluateConstraint(f, g, x, 0.25);

    CHECK(h_calls == 1);
    CHECK(dhdx_calls == 1);
    CHECK(dhdt_calls == 1);
    CHECK(evaluation.h == doctest::Approx(6.25));
    CHECK(evaluation.dhdt == doctest::Approx(0.5));
    CHECK(evaluation.drift == doctest::Approx(2.0));
    CHECK(evaluation.uCoe[0] == doctest::Approx(2.0));
    CHECK(evaluation.uCoe[1] == doctest::Approx(3.0));
    CHECK(evaluation.constWithTime == doctest::Approx(15.0));
    CHECK(evaluation.constWithoutTime == doctest::Approx(14.5));

    VectorXd u(2);
    u << 0.5, -1.0;
    CHECK(evaluation.hdot(u) == doctest::Approx(0.5));
}

TEST_CASE("SecondOrderCBFEvaluatesAffineHOCBFConstraint") {
    SecondOrderCBF cbf;
    cbf.name = "hocbf";
    cbf.k0 = 6.0;
    cbf.k1 = 5.0;
    cbf.lambda1 = 2.0;
    cbf.h = [](const VectorXd&, double) { return 3.0; };
    cbf.hdot = [](const VectorXd&, double) { return -1.0; };
    cbf.hddotConst = [](const VectorXd&, double) { return 4.0; };
    cbf.uCoe = [](const VectorXd&, double) {
        VectorXd coe(2);
        coe << 2.0, -3.0;
        return coe;
    };

    VectorXd x(2);
    x << 0.0, 0.0;
    auto evaluation = cbf.evaluateConstraint(x, 0.0);

    VectorXd u(2);
    u << 1.0, 1.0;

    CHECK(evaluation.h == doctest::Approx(3.0));
    CHECK(evaluation.hdot == doctest::Approx(-1.0));
    CHECK(evaluation.hddotConst == doctest::Approx(4.0));
    CHECK(evaluation.constTerm == doctest::Approx(17.0));
    CHECK(evaluation.psi1 == doctest::Approx(5.0));
    CHECK(evaluation.value(u) == doctest::Approx(16.0));
}

TEST_CASE("SecondOrderCBFAppliesSampledDataReserveToAffineHOCBFConstraint") {
    SecondOrderCBF cbf;
    cbf.name = "reservedHocbf";
    cbf.k0 = 6.0;
    cbf.k1 = 5.0;
    cbf.lambda1 = 2.0;
    cbf.sampledDataReserve = 1.25;
    cbf.h = [](const VectorXd&, double) { return 3.0; };
    cbf.hdot = [](const VectorXd&, double) { return -1.0; };
    cbf.hddotConst = [](const VectorXd&, double) { return 4.0; };
    cbf.uCoe = [](const VectorXd&, double) {
        VectorXd coe(2);
        coe << 2.0, -3.0;
        return coe;
    };

    VectorXd x(2);
    x << 0.0, 0.0;
    auto evaluation = cbf.evaluateConstraint(x, 0.0);

    CHECK(evaluation.sampledDataReserve == doctest::Approx(1.25));
    CHECK(evaluation.constTerm == doctest::Approx(15.75));
}

TEST_CASE("PairwiseDistanceKinematicsIncludesRadialVelocityAndCurvature") {
    Point pi(3.0, 4.0);
    Point pj(0.0, 0.0);
    VectorXd vi(2);
    vi << 1.0, 2.0;
    VectorXd vj(2);
    vj << 0.0, 0.0;

    auto terms = computePairwiseDistanceKinematics(pi, pj, vi, vj);

    CHECK(terms.distance == doctest::Approx(5.0));
    CHECK(terms.normal.x() == doctest::Approx(0.6));
    CHECK(terms.normal.y() == doctest::Approx(0.8));
    CHECK(terms.radialVelocity == doctest::Approx(2.2));
    CHECK(terms.curvature == doctest::Approx(0.032));
}

TEST_CASE("Shared pairwise second-order rows use exact collision and communication signs") {
    PairwiseSecondOrderState2D self{
        Point(3.0, 4.0),
        Eigen::Vector2d(2.0, 1.0),
        Eigen::Vector2d::Zero()};
    PairwiseSecondOrderState2D reference{
        Point(0.0, 0.0),
        Eigen::Vector2d(-1.0, 0.5),
        Eigen::Vector2d(0.4, -0.2)};

    constexpr double k = 1.5;
    constexpr double lambda1 = 1.0;
    constexpr double lambda2 = 2.0;
    constexpr double uncertainty = 0.25;
    constexpr double reserve = 0.75;
    const auto kinematics = computePairwiseDistanceKinematics(
        self.position,
        reference.position,
        self.velocity,
        reference.velocity);
    const double neighbourRadialAcceleration =
        kinematics.normal.dot(reference.acceleration);

    const auto collision = buildPairwiseSecondOrderRow(
        self,
        reference,
        {PairwiseSecondOrderBarrierKind::CollisionLower,
         10.0, uncertainty, k, lambda1, lambda2, reserve});
    CHECK(collision.uCoe(0) == doctest::Approx(k * kinematics.normal(0)));
    CHECK(collision.uCoe(1) == doctest::Approx(k * kinematics.normal(1)));
    CHECK(collision.h == doctest::Approx(
        k * (kinematics.distance - 10.0 - uncertainty)));
    CHECK(collision.hdot == doctest::Approx(k * kinematics.radialVelocity));
    CHECK(collision.hddotConst == doctest::Approx(
        k * (kinematics.curvature - neighbourRadialAcceleration)));
    CHECK(collision.psi1 == doctest::Approx(
        collision.hdot + lambda1 * collision.h));
    CHECK(collision.constTerm == doctest::Approx(
        collision.hddotConst
        + (lambda1 + lambda2) * collision.hdot
        + lambda1 * lambda2 * collision.h
        - reserve));

    const auto communication = buildPairwiseSecondOrderRow(
        self,
        reference,
        {PairwiseSecondOrderBarrierKind::CommunicationUpper,
         850.0, uncertainty, k, lambda1, lambda2, reserve});
    CHECK(communication.uCoe(0) == doctest::Approx(-k * kinematics.normal(0)));
    CHECK(communication.uCoe(1) == doctest::Approx(-k * kinematics.normal(1)));
    CHECK(communication.h == doctest::Approx(
        k * (850.0 - kinematics.distance - uncertainty)));
    CHECK(communication.hdot == doctest::Approx(-k * kinematics.radialVelocity));
    CHECK(communication.hddotConst == doctest::Approx(
        k * (neighbourRadialAcceleration - kinematics.curvature)));
    CHECK(communication.psi1 == doctest::Approx(
        communication.hdot + lambda1 * communication.h));
    CHECK(communication.constTerm == doctest::Approx(
        communication.hddotConst
        + (lambda1 + lambda2) * communication.hdot
        + lambda1 * lambda2 * communication.h
        - reserve));
}

TEST_CASE("Shared pairwise second-order rows reject coincident positions") {
    PairwiseSecondOrderState2D self{
        Point(1.0, 1.0), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    PairwiseSecondOrderState2D reference{
        Point(1.0, 1.0), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};

    CHECK_THROWS_AS(
        buildPairwiseSecondOrderRow(
            self,
            reference,
            {PairwiseSecondOrderBarrierKind::CollisionLower,
             10.0, 0.0, 1.0, 1.0, 1.0, 0.0}),
        std::invalid_argument);
}
