#pragma once

#include "cbf/PairwiseSecondOrderCBF.hpp"

#include <cmath>
#include <stdexcept>

namespace gf {

enum class SnapshotTubeProvenance {
    ExternallyCertified,
    CovarianceSigmaDevelopment
};

struct PairwiseSnapshotTube {
    double position_radius_m = 0.0;
    double velocity_radius_mps = 0.0;
    SnapshotTubeProvenance provenance =
        SnapshotTubeProvenance::CovarianceSigmaDevelopment;
};

struct SnapshotRobustPairRow {
    Eigen::Vector2d nominal_normal = Eigen::Vector2d::Zero();
    Eigen::Vector2d nominal_control_coefficient = Eigen::Vector2d::Zero();
    double distance_lower = 0.0;
    double distance_upper = 0.0;
    double direction_radius = 0.0;
    double barrier_h_lower = 0.0;
    double barrier_hdot_lower = 0.0;
    double barrier_psi1_lower = 0.0;
    double central_constant_lower = 0.0;
    double coefficient_reserve = 0.0;
    double position_uncertainty_reserve_m = 0.0;
    double velocity_uncertainty_reserve_mps = 0.0;
    SnapshotTubeProvenance provenance =
        SnapshotTubeProvenance::CovarianceSigmaDevelopment;

    double fixedMargin(const Eigen::Vector2d& control) const {
        return nominal_control_coefficient.dot(control) +
               central_constant_lower - coefficient_reserve;
    }

    double firstHalfMargin(const Eigen::Vector2d& control) const {
        return nominal_control_coefficient.dot(control) +
               0.5 * central_constant_lower - coefficient_reserve;
    }

    double secondHalfMargin(const Eigen::Vector2d& control) const {
        return -nominal_control_coefficient.dot(control) +
               0.5 * central_constant_lower - coefficient_reserve;
    }
};

inline SnapshotRobustPairRow buildSnapshotRobustPairRow(
    const PairwiseSecondOrderState2D& first,
    const PairwiseSecondOrderState2D& second,
    const PairwiseSecondOrderRowSpec& spec,
    const PairwiseSnapshotTube& tube,
    double acceleration_half_box) {
    const bool finite_states =
        std::isfinite(first.position.x) && std::isfinite(first.position.y) &&
        std::isfinite(second.position.x) && std::isfinite(second.position.y) &&
        first.velocity.allFinite() && second.velocity.allFinite();
    if (!finite_states || !std::isfinite(tube.position_radius_m) ||
        tube.position_radius_m < 0.0 ||
        !std::isfinite(tube.velocity_radius_mps) ||
        tube.velocity_radius_mps < 0.0 ||
        !std::isfinite(acceleration_half_box) ||
        acceleration_half_box <= 0.0 ||
        !std::isfinite(spec.distanceLimit) || spec.distanceLimit <= 0.0 ||
        !std::isfinite(spec.uncertainty) || spec.uncertainty < 0.0 ||
        !std::isfinite(spec.totalReserve) || spec.totalReserve < 0.0 ||
        spec.k != 1.0 || spec.lambda1 != 1.0 || spec.lambda2 != 1.0) {
        throw std::invalid_argument("invalid snapshot robust pair-row contract");
    }

    const Eigen::Vector2d relative_position(
        first.position.x - second.position.x,
        first.position.y - second.position.y);
    const Eigen::Vector2d relative_velocity =
        first.velocity - second.velocity;
    const double nominal_distance = relative_position.norm();
    const double position_radius = spec.uncertainty + tube.position_radius_m;
    if (!std::isfinite(nominal_distance) || nominal_distance <= position_radius ||
        nominal_distance < 1e-9) {
        throw std::invalid_argument(
            "snapshot position tube contains the radial singularity");
    }

    SnapshotRobustPairRow row;
    row.position_uncertainty_reserve_m = position_radius;
    row.velocity_uncertainty_reserve_mps = tube.velocity_radius_mps;
    row.nominal_normal = relative_position / nominal_distance;
    row.distance_lower = nominal_distance - position_radius;
    row.distance_upper = nominal_distance + position_radius;
    const double angle = std::asin(position_radius / nominal_distance);
    row.direction_radius = 2.0 * std::sin(0.5 * angle);
    const double nominal_radial_velocity =
        row.nominal_normal.dot(relative_velocity);
    const double radial_velocity_uncertainty =
        row.direction_radius * relative_velocity.norm() +
        tube.velocity_radius_mps;
    const double relative_speed_upper =
        relative_velocity.norm() + tube.velocity_radius_mps;
    row.coefficient_reserve = acceleration_half_box * std::sqrt(2.0) *
                              row.direction_radius;
    row.provenance = tube.provenance;

    if (spec.kind == PairwiseSecondOrderBarrierKind::CollisionLower) {
        row.nominal_control_coefficient = row.nominal_normal;
        row.barrier_h_lower = row.distance_lower - spec.distanceLimit;
        row.barrier_hdot_lower =
            nominal_radial_velocity - radial_velocity_uncertainty;
        row.central_constant_lower =
            row.barrier_h_lower + 2.0 * row.barrier_hdot_lower -
            spec.totalReserve;
    } else {
        row.nominal_control_coefficient = -row.nominal_normal;
        row.barrier_h_lower = spec.distanceLimit - row.distance_upper;
        row.barrier_hdot_lower =
            -nominal_radial_velocity - radial_velocity_uncertainty;
        row.central_constant_lower =
            row.barrier_h_lower + 2.0 * row.barrier_hdot_lower -
            relative_speed_upper * relative_speed_upper / row.distance_lower -
            spec.totalReserve;
    }
    row.barrier_psi1_lower =
        row.barrier_h_lower + row.barrier_hdot_lower;
    return row;
}

}  // namespace gf
