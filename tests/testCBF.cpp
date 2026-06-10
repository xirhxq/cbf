#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "cbf/CBF.hpp"

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
