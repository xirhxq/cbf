#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/SnapshotRobustPairRow.hpp"

#include <array>
#include <cmath>

namespace {

PairwiseSecondOrderState2D state(
    double px, double py, double vx, double vy) {
    return {Point(px, py), Eigen::Vector2d(vx, vy), Eigen::Vector2d::Zero()};
}

double exactCentralResidual(
    PairwiseSecondOrderState2D first,
    PairwiseSecondOrderState2D second,
    const PairwiseSecondOrderRowSpec& spec,
    const Eigen::Vector2d& first_control,
    const Eigen::Vector2d& second_control) {
    first.acceleration = first_control;
    second.acceleration = second_control;
    const auto row = buildPairwiseSecondOrderRow(first, second, spec);
    return row.uCoe.dot(first_control) + row.constTerm;
}

std::array<Eigen::Vector2d, 4> corners(double half_width) {
    return {{{-half_width, -half_width}, {-half_width, half_width},
             {half_width, -half_width}, {half_width, half_width}}};
}

PairwiseSecondOrderRowSpec spec(
    PairwiseSecondOrderBarrierKind kind, double limit) {
    return {kind, limit, 0.0, 1.0, 1.0, 1.0, 0.0};
}

}  // namespace

TEST_CASE("Snapshot robust fixed rows lower-bound every box vertex residual") {
    const auto self = state(100.0, 20.0, 1.2, -0.4);
    const auto anchor = state(0.0, 0.0, 0.0, 0.0);
    const double position_half_width = 0.35;
    const double velocity_half_width = 0.18;
    const double input_half_box = 2.0;
    const gf::PairwiseSnapshotTube tube{
        std::sqrt(2.0) * position_half_width,
        std::sqrt(2.0) * velocity_half_width,
        gf::SnapshotTubeProvenance::ExternallyCertified};

    for (const auto barrier : {
             PairwiseSecondOrderBarrierKind::CollisionLower,
             PairwiseSecondOrderBarrierKind::CommunicationUpper}) {
        const auto row_spec = spec(
            barrier,
            barrier == PairwiseSecondOrderBarrierKind::CollisionLower
                ? 20.0 : 140.0);
        const auto robust = gf::buildSnapshotRobustPairRow(
            self, anchor, row_spec, tube, input_half_box);
        for (const auto& position_error : corners(position_half_width)) {
            for (const auto& velocity_error : corners(velocity_half_width)) {
                for (const auto& control : corners(input_half_box)) {
                    auto actual = self;
                    actual.position.x += position_error.x();
                    actual.position.y += position_error.y();
                    actual.velocity += velocity_error;
                    const double exact = exactCentralResidual(
                        actual, anchor, row_spec, control,
                        Eigen::Vector2d::Zero());
                    CHECK(robust.fixedMargin(control) <= exact + 1e-10);
                }
            }
        }
    }
}

TEST_CASE("Robust mobile half rows compose below the exact coupled residual") {
    const auto first = state(120.0, 30.0, 0.8, -0.3);
    const auto second = state(0.0, 0.0, -0.2, 0.1);
    const double position_half_width = 0.25;
    const double velocity_half_width = 0.12;
    const double input_half_box = 2.0;
    const gf::PairwiseSnapshotTube tube{
        2.0 * std::sqrt(2.0) * position_half_width,
        2.0 * std::sqrt(2.0) * velocity_half_width,
        gf::SnapshotTubeProvenance::ExternallyCertified};
    const auto row_spec = spec(
        PairwiseSecondOrderBarrierKind::CommunicationUpper, 160.0);
    const auto robust = gf::buildSnapshotRobustPairRow(
        first, second, row_spec, tube, input_half_box);

    for (const auto& first_position_error : corners(position_half_width)) {
        for (const auto& second_position_error : corners(position_half_width)) {
            for (const auto& first_velocity_error : corners(velocity_half_width)) {
                for (const auto& second_velocity_error : corners(velocity_half_width)) {
                    auto actual_first = first;
                    auto actual_second = second;
                    actual_first.position.x += first_position_error.x();
                    actual_first.position.y += first_position_error.y();
                    actual_second.position.x += second_position_error.x();
                    actual_second.position.y += second_position_error.y();
                    actual_first.velocity += first_velocity_error;
                    actual_second.velocity += second_velocity_error;
                    for (const auto& first_control : corners(input_half_box)) {
                        for (const auto& second_control : corners(input_half_box)) {
                            const double local_sum =
                                robust.firstHalfMargin(first_control) +
                                robust.secondHalfMargin(second_control);
                            const double exact = exactCentralResidual(
                                actual_first, actual_second, row_spec,
                                first_control, second_control);
                            CHECK(local_sum <= exact + 1e-10);
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("Position-only tightening misses an allowed adverse velocity") {
    const auto self = state(10.0, 0.0, 0.0, 0.0);
    const auto anchor = state(0.0, 0.0, 0.0, 0.0);
    const auto row_spec = spec(
        PairwiseSecondOrderBarrierKind::CollisionLower, 9.0);
    PairwiseSecondOrderRowSpec legacy_spec = row_spec;
    const auto legacy = buildPairwiseSecondOrderRow(self, anchor, legacy_spec);
    CHECK(legacy.constTerm > 0.0);

    auto adverse = self;
    adverse.velocity.x() = -1.0;
    const double exact = exactCentralResidual(
        adverse, anchor, row_spec, Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Zero());
    CHECK(exact < 0.0);

    const gf::PairwiseSnapshotTube tube{
        0.0, 1.0, gf::SnapshotTubeProvenance::ExternallyCertified};
    const auto robust = gf::buildSnapshotRobustPairRow(
        self, anchor, row_spec, tube, 2.0);
    CHECK(robust.fixedMargin(Eigen::Vector2d::Zero()) <= exact);
}

TEST_CASE("Shrinking h once does not cover radial-direction uncertainty") {
    const auto self = state(10.0, 0.0, 0.0, -200.0);
    const auto anchor = state(0.0, 0.0, 0.0, 0.0);
    const auto truth_spec = spec(
        PairwiseSecondOrderBarrierKind::CollisionLower, 9.0);
    auto legacy_spec = truth_spec;
    legacy_spec.uncertainty = 0.1;
    const auto legacy = buildPairwiseSecondOrderRow(self, anchor, legacy_spec);
    CHECK(legacy.psi1 > 0.0);

    auto adverse = self;
    adverse.position.y += 0.1;
    const auto exact = buildPairwiseSecondOrderRow(adverse, anchor, truth_spec);
    CHECK(exact.psi1 < 0.0);

    const auto robust = gf::buildSnapshotRobustPairRow(
        self, anchor, truth_spec,
        {0.1, 0.0, gf::SnapshotTubeProvenance::ExternallyCertified}, 2.0);
    CHECK(robust.barrier_psi1_lower <= exact.psi1);
}

TEST_CASE("Robust row lower bound also covers sampled ball directions") {
    const auto self = state(80.0, 25.0, 0.7, -0.2);
    const auto anchor = state(0.0, 0.0, 0.0, 0.0);
    const auto row_spec = spec(
        PairwiseSecondOrderBarrierKind::CommunicationUpper, 120.0);
    const double position_radius = 0.4;
    const double velocity_radius = 0.2;
    const double input_half_box = 2.0;
    const double pi = std::acos(-1.0);
    const auto robust = gf::buildSnapshotRobustPairRow(
        self, anchor, row_spec,
        {position_radius, velocity_radius,
         gf::SnapshotTubeProvenance::ExternallyCertified},
        input_half_box);
    for (int position_index = 0; position_index < 32; ++position_index) {
        const double position_angle =
            2.0 * pi * static_cast<double>(position_index) / 32.0;
        const Eigen::Vector2d position_error = position_radius *
            Eigen::Vector2d(std::cos(position_angle), std::sin(position_angle));
        for (int velocity_index = 0; velocity_index < 16; ++velocity_index) {
            const double velocity_angle =
                2.0 * pi * static_cast<double>(velocity_index) / 16.0;
            const Eigen::Vector2d velocity_error = velocity_radius *
                Eigen::Vector2d(std::cos(velocity_angle), std::sin(velocity_angle));
            auto actual = self;
            actual.position.x += position_error.x();
            actual.position.y += position_error.y();
            actual.velocity += velocity_error;
            for (const auto& control : corners(input_half_box)) {
                CHECK(robust.fixedMargin(control) <= exactCentralResidual(
                    actual, anchor, row_spec, control,
                    Eigen::Vector2d::Zero()) + 1e-10);
            }
        }
    }
}

TEST_CASE("Nominal coefficient without input-box reserve can overstate safety") {
    const auto self = state(10.0, 0.0, 0.0, 0.0);
    const auto anchor = state(0.0, 0.0, 0.0, 0.0);
    const auto row_spec = spec(
        PairwiseSecondOrderBarrierKind::CollisionLower, 1.0);
    const gf::PairwiseSnapshotTube tube{
        1.0, 0.0, gf::SnapshotTubeProvenance::ExternallyCertified};
    const auto robust = gf::buildSnapshotRobustPairRow(
        self, anchor, row_spec, tube, 2.0);
    REQUIRE(robust.coefficient_reserve > 0.0);
    CHECK(robust.fixedMargin({-2.0, 2.0}) == doctest::Approx(
        robust.nominal_control_coefficient.dot(Eigen::Vector2d(-2.0, 2.0)) +
        robust.central_constant_lower - robust.coefficient_reserve));
    CHECK(robust.fixedMargin({-2.0, 2.0}) <
          robust.nominal_control_coefficient.dot(Eigen::Vector2d(-2.0, 2.0)) +
          robust.central_constant_lower);
}

TEST_CASE("Snapshot robust rows reject singular or invalid positive-gain contracts") {
    const auto anchor = state(0.0, 0.0, 0.0, 0.0);
    auto row_spec = spec(
        PairwiseSecondOrderBarrierKind::CommunicationUpper, 20.0);
    CHECK_THROWS_AS(
        gf::buildSnapshotRobustPairRow(
            state(1.0, 0.0, 0.0, 0.0), anchor, row_spec,
            {1.0, 0.0, gf::SnapshotTubeProvenance::ExternallyCertified}, 2.0),
        std::invalid_argument);
    row_spec.lambda2 = 0.0;
    CHECK_THROWS_AS(
        gf::buildSnapshotRobustPairRow(
            state(10.0, 0.0, 0.0, 0.0), anchor, row_spec,
            {0.1, 0.1, gf::SnapshotTubeProvenance::ExternallyCertified}, 2.0),
        std::invalid_argument);
    row_spec.lambda2 = 1.0;
    row_spec.lambda1 = -0.5;
    CHECK_THROWS_AS(
        gf::buildSnapshotRobustPairRow(
            state(10.0, 0.0, 0.0, 0.0), anchor, row_spec,
            {0.1, 0.1, gf::SnapshotTubeProvenance::ExternallyCertified}, 2.0),
        std::invalid_argument);
}

TEST_CASE("Formal ten metre collision reserve is outside the position tube") {
    const auto self=state(15.0,0.0,0.0,0.0);
    const auto anchor=state(0.0,0.0,0.0,0.0);
    const auto row_spec=spec(
        PairwiseSecondOrderBarrierKind::CollisionLower,10.0);
    const auto robust=gf::buildSnapshotRobustPairRow(
        self,anchor,row_spec,
        {1.0,0.25,gf::SnapshotTubeProvenance::ExternallyCertified},4.0);
    CHECK(robust.barrier_h_lower==doctest::Approx(4.0));
    CHECK(robust.position_uncertainty_reserve_m==doctest::Approx(1.0));
    CHECK(robust.velocity_uncertainty_reserve_mps==doctest::Approx(0.25));
    CHECK(robust.barrier_h_lower<5.0);
}
