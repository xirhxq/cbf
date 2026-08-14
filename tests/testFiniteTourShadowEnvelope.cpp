#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/FiniteTourShadowEnvelope.hpp"

#include <Eigen/Dense>

#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

TEST_CASE("Unbounded accepted innovation cannot produce a deterministic shadow enclosure") {
    const gf::ShadowStateBox prior{
        Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)};
    const gf::ScalarUpdateBound unbounded{
        Eigen::VectorXd::Ones(1),
        std::numeric_limits<double>::infinity()};
    CHECK_THROWS_WITH_AS(
        gf::accumulateScalarUpdateBatch(prior, {unbounded}),
        "accepted innovation bound must be finite and non-negative",
        std::invalid_argument);
}

TEST_CASE("Prediction and one accepted-or-dropout update enclose the analytic one-dimensional example") {
    const gf::ShadowStateBox prior{
        Eigen::VectorXd::Constant(1, -0.2),
        Eigen::VectorXd::Constant(1, 0.2)};
    const gf::ScalarUpdateBound update{
        Eigen::VectorXd::Constant(1, 0.5), 0.1};
    const auto corrected = gf::accumulateScalarUpdateBatch(prior, {update});
    CHECK(corrected.lower(0) <= -0.25);
    CHECK(corrected.upper(0) >= 0.25);

    const auto predicted = gf::propagateShadowPrediction(
        corrected,
        Eigen::MatrixXd::Identity(1, 1),
        Eigen::MatrixXd::Identity(1, 1),
        Eigen::VectorXd::Constant(1, 0.02));
    CHECK(predicted.lower(0) <= -0.27);
    CHECK(predicted.upper(0) >= 0.27);
}

TEST_CASE("Sequential scalar corrections accumulate linearly without enumerating words") {
    const gf::ShadowStateBox prior{
        Eigen::VectorXd::Constant(2, -0.1),
        Eigen::VectorXd::Constant(2, 0.1)};
    const std::vector<gf::ScalarUpdateBound> updates = {
        {Eigen::Vector2d{0.2, 0.4}, 0.1},
        {Eigen::Vector2d{0.3, 0.1}, 0.2},
        {Eigen::Vector2d{0.1, 0.5}, 0.2}};
    const auto result = gf::accumulateScalarUpdateBatch(prior, updates);

    CHECK(result.lower(0) <= -0.20);
    CHECK(result.upper(0) >= 0.20);
    CHECK(result.lower(1) <= -0.26);
    CHECK(result.upper(1) >= 0.26);
}

TEST_CASE("Direction support covers mobile-mobile and mobile-fixed relative position errors") {
    Eigen::VectorXd lower(8);
    Eigen::VectorXd upper(8);
    lower << -0.2, -0.1, -0.3, -0.3,
             -0.4, -0.5, -0.2, -0.2;
    upper <<  0.3,  0.2,  0.3,  0.3,
              0.1,  0.6,  0.2,  0.2;
    const gf::ShadowStateBox box{lower, upper};

    CHECK(gf::relativePositionSupport(
        box, 0, std::optional<std::size_t>{1}, Eigen::Vector2d{1.0, 0.0}) ==
        doctest::Approx(0.7));
    CHECK(gf::relativePositionSupport(
        box, 0, std::nullopt, Eigen::Vector2d{0.0, -1.0}) ==
        doctest::Approx(0.1));
}

TEST_CASE("Gain norm bound uses propagation covariance Jacobian and positive range variance") {
    CHECK(gf::branchIndependentGainNormBound(2.0, std::sqrt(2.0), 0.5) ==
          doctest::Approx(4.0 * std::sqrt(2.0)));
    CHECK_THROWS_AS(
        gf::branchIndependentGainNormBound(2.0, 1.0, 0.0),
        std::invalid_argument);

    Eigen::Matrix2d covariance;
    covariance << 2.0, 0.0, 0.0, 0.5;
    const Eigen::VectorXd component =
        gf::branchIndependentComponentGainBounds(
            covariance, std::sqrt(2.0), 0.5);
    REQUIRE(component.size() == 2);
    CHECK(component(0) == doctest::Approx(4.0 * std::sqrt(2.0)));
    CHECK(component(1) == doctest::Approx(2.0 * std::sqrt(2.0)));

    Eigen::Matrix2d loewner_upper = Eigen::Matrix2d::Zero();
    loewner_upper.diagonal() << 1.0, 0.1;
    Eigen::Matrix2d admissible;
    admissible << 0.5, 0.5 * std::sqrt(0.1),
                  0.5 * std::sqrt(0.1), 0.05;
    CHECK(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
        loewner_upper - admissible).eigenvalues().minCoeff() >= -1.0e-12);
    CHECK(admissible.row(1).norm() > loewner_upper.row(1).norm());
    const auto sound_component = gf::branchIndependentComponentGainBounds(
        loewner_upper, 1.0, 1.0);
    CHECK(sound_component(1) >= admissible.row(1).norm());
}
