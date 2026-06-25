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
