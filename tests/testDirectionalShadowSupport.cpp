#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/DirectionalShadowSupport.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr std::size_t kMobileCount = 14;
constexpr std::size_t kStateDimension = 56;
constexpr std::size_t kSlotsPerCycle = 133;
constexpr double kDt = 0.1;

gf::ShadowStateBox initialBox() {
    Eigen::VectorXd lower(kStateDimension), upper(kStateDimension);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        lower.segment<4>(4 * mobile) << -0.01, -0.01, -0.002, -0.002;
        upper.segment<4>(4 * mobile) << 0.01, 0.01, 0.002, 0.002;
    }
    return {lower, upper};
}

Eigen::MatrixXd transition() {
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(
        kStateDimension, kStateDimension);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        result(4 * mobile, 4 * mobile + 2) = kDt;
        result(4 * mobile + 1, 4 * mobile + 3) = kDt;
    }
    return result;
}

Eigen::MatrixXd disturbanceInput() {
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(kStateDimension, 28);
    Eigen::Matrix<double, 4, 2> block;
    block << 0.5 * kDt * kDt, 0.0,
             0.0, 0.5 * kDt * kDt,
             kDt, 0.0,
             0.0, kDt;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        result.block<4, 2>(4 * mobile, 2 * mobile) = block;
    return result;
}

std::vector<gf::DirectionalShadowStep> frozenSteps(std::size_t count) {
    std::vector<gf::DirectionalShadowStep> result;
    Eigen::MatrixXd covariance =
        1.0e-4 * Eigen::MatrixXd::Identity(kStateDimension, kStateDimension);
    const Eigen::MatrixXd F = transition();
    for (std::size_t cycle = 0; cycle < count; ++cycle) {
        covariance = F * covariance * F.transpose();
        gf::DirectionalShadowStep step;
        step.transition = F;
        step.disturbance_input = disturbanceInput();
        step.absolute_disturbance_bound =
            Eigen::VectorXd::Constant(28, 1.0e-4);
        step.covariance_upper_after_prediction = covariance;
        step.scalar_slots.assign(kSlotsPerCycle,
            gf::DirectionalScalarSlotBound{std::sqrt(2.0), 1.0, 0.05});
        result.push_back(std::move(step));
    }
    return result;
}

}  // namespace

TEST_CASE("One scalar correction remains one joint generator in a queried direction") {
    const Eigen::Vector2d gain{1.0, 1.0};
    const Eigen::Vector2d common_mode_rejecting{1.0, -1.0};
    CHECK(gf::sharedScalarCorrectionSupport(
              common_mode_rejecting, gain, 0.2) == doctest::Approx(0.0));
    CHECK(gf::axisBoxCorrectionSupport(
              common_mode_rejecting, gain.cwiseAbs(), 0.2) ==
          doctest::Approx(0.4));
}

TEST_CASE("Accepted and dropout words are enclosed without branching") {
    const Eigen::Vector2d gain{0.4, -0.2};
    const Eigen::Vector2d direction{0.6, 0.8};
    const double bound = gf::sharedScalarCorrectionSupport(
        direction, gain, 0.05);
    for (double innovation : {-0.05, -0.02, 0.0, 0.03, 0.05}) {
        const double accepted = direction.dot(-gain * innovation);
        CHECK(accepted <= bound + 1.0e-15);
        CHECK(-accepted <= bound + 1.0e-15);
    }
    CHECK(0.0 <= bound);  // dropout
    CHECK(gf::sharedScalarCorrectionSupport(
              -direction, gain, 0.05) == doctest::Approx(bound));
}

TEST_CASE("Frozen 14 plus 3 direction support reproduces the offline budget") {
    const Eigen::Vector2d diagonal =
        Eigen::Vector2d::Constant(1.0 / std::sqrt(2.0));
    const auto absolute_position = gf::mobileStateDirection(
        kMobileCount, 0, gf::MobileStatePlane::Position, diagonal);
    const auto absolute_velocity = gf::mobileStateDirection(
        kMobileCount, 0, gf::MobileStatePlane::Velocity, diagonal);
    const auto relative_position = gf::relativeMobileStateDirection(
        kMobileCount, 0, 1, gf::MobileStatePlane::Position, diagonal);
    const auto relative_velocity = gf::relativeMobileStateDirection(
        kMobileCount, 0, 1, gf::MobileStatePlane::Velocity, diagonal);

    const auto steps41 = frozenSteps(41);
    CHECK(gf::finiteHorizonDirectionalSupport(
              initialBox(), steps41, absolute_position) ==
          doctest::Approx(0.44409840436029324).epsilon(1.0e-11));
    CHECK(gf::finiteHorizonDirectionalSupport(
              initialBox(), steps41, relative_position) ==
          doctest::Approx(0.64382365314330747).epsilon(1.0e-11));

    const auto steps42 = frozenSteps(42);
    CHECK(gf::finiteHorizonDirectionalSupport(
              initialBox(), steps42, absolute_position) ==
          doctest::Approx(0.4720206740224786).epsilon(1.0e-11));

    const auto steps82 = frozenSteps(82);
    CHECK(gf::finiteHorizonDirectionalSupport(
              initialBox(), steps82, absolute_position) ==
          doctest::Approx(2.8853201331533249).epsilon(1.0e-11));
    CHECK(gf::finiteHorizonDirectionalSupport(
              initialBox(), steps82, absolute_velocity) ==
          doctest::Approx(0.3481735511806151).epsilon(1.0e-11));
    CHECK(gf::finiteHorizonDirectionalSupport(
              initialBox(), steps82, relative_position) ==
          doctest::Approx(4.1051145121802746).epsilon(1.0e-11));
    CHECK(gf::finiteHorizonDirectionalSupport(
              initialBox(), steps82, relative_velocity) ==
          doctest::Approx(0.49472792263101284).epsilon(1.0e-11));
}

TEST_CASE("Frozen box contract uses the diagonal support that dominates cardinal directions") {
    const auto steps = frozenSteps(42);
    const Eigen::Vector2d diagonal =
        Eigen::Vector2d::Constant(1.0 / std::sqrt(2.0));
    const auto absolute = [&](const Eigen::Vector2d& direction) {
        return gf::finiteHorizonDirectionalSupport(
            initialBox(), steps,
            gf::mobileStateDirection(
                kMobileCount, 7, gf::MobileStatePlane::Position,
                direction));
    };
    const auto relative = [&](const Eigen::Vector2d& direction) {
        return gf::finiteHorizonDirectionalSupport(
            initialBox(), steps,
            gf::relativeMobileStateDirection(
                kMobileCount, 7, 12, gf::MobileStatePlane::Position,
                direction));
    };
    CHECK(absolute(Eigen::Vector2d::UnitX()) ==
          doctest::Approx(absolute(Eigen::Vector2d::UnitY())).epsilon(1.0e-12));
    CHECK(absolute(diagonal) > absolute(Eigen::Vector2d::UnitX()));
    CHECK(absolute(diagonal) ==
          doctest::Approx(absolute(-diagonal)).epsilon(1.0e-12));
    CHECK(relative(Eigen::Vector2d::UnitX()) ==
          doctest::Approx(relative(Eigen::Vector2d::UnitY())).epsilon(1.0e-12));
    CHECK(relative(diagonal) > relative(Eigen::Vector2d::UnitX()));
    CHECK(relative(diagonal) ==
          doctest::Approx(relative(-diagonal)).epsilon(1.0e-12));
}

TEST_CASE("Directional support rejects malformed finite-horizon contracts") {
    const gf::ShadowStateBox one_dimensional{
        Eigen::VectorXd::Constant(1, -0.1),
        Eigen::VectorXd::Constant(1, 0.1)};
    gf::DirectionalShadowStep malformed;
    malformed.transition = Eigen::MatrixXd::Identity(1, 1);
    malformed.disturbance_input = Eigen::MatrixXd::Identity(1, 1);
    malformed.absolute_disturbance_bound = Eigen::VectorXd::Zero(1);
    malformed.covariance_upper_after_prediction =
        Eigen::MatrixXd::Identity(2, 2);
    CHECK_THROWS_AS(gf::finiteHorizonDirectionalSupport(
        one_dimensional, {malformed}, Eigen::VectorXd::Ones(1)),
        std::invalid_argument);
    CHECK_THROWS_AS(gf::sharedScalarCorrectionSupport(
        Eigen::VectorXd::Ones(1), Eigen::VectorXd::Ones(2), 0.1),
        std::invalid_argument);
    CHECK_THROWS_AS(gf::sharedScalarCorrectionSupport(
        Eigen::VectorXd::Ones(1), Eigen::VectorXd::Ones(1),
        std::numeric_limits<double>::infinity()), std::invalid_argument);
}
