#pragma once

#include "bridge/HocbfFeasibilityGuard.hpp"
#include "grand_finale/CanonicalHardRows.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct ProgressCompatibilityConfig {
    double max_projection_norm = 0.0;
    double min_direction_ratio = 0.0;
    double comparison_tolerance = 1.0e-10;
    bool zero_nominal_is_compatible = true;
};

struct ProgressCompatibilityResult {
    bool polytope_nonempty = false;
    bool compatible = false;
    std::string reason;
    Eigen::Vector2d projection = Eigen::Vector2d::Zero();
    double projection_norm = std::numeric_limits<double>::infinity();
    double direction_inner_product =
        -std::numeric_limits<double>::infinity();
    double required_direction_inner_product = 0.0;
    double minimum_hard_residual =
        -std::numeric_limits<double>::infinity();
};

inline std::vector<BridgeHocbfHalfspace2D> canonicalOwnerHalfspaces(
    const std::vector<CanonicalHardRow>& rows,
    NodeId owner) {
    std::vector<BridgeHocbfHalfspace2D> result;
    for (const CanonicalHardRow& row : rows) {
        if (row.owner != owner || row.kind == CanonicalHardRowKind::InputBox) {
            continue;
        }
        result.push_back(BridgeHocbfHalfspace2D{
            row.control_coefficient.x(), row.control_coefficient.y(),
            -row.constant});
    }
    return result;
}

inline double minimumCanonicalOwnerResidual(
    const std::vector<CanonicalHardRow>& rows,
    NodeId owner,
    const Eigen::Vector2d& control) {
    double minimum = std::numeric_limits<double>::infinity();
    bool found = false;
    for (const CanonicalHardRow& row : rows) {
        if (row.owner != owner) continue;
        minimum = std::min(minimum, row.margin(control));
        found = true;
    }
    return found ? minimum : std::numeric_limits<double>::infinity();
}

inline ProgressCompatibilityResult evaluateProgressCompatibility(
    const std::vector<CanonicalHardRow>& rows,
    NodeId owner,
    const Eigen::Vector2d& nominal,
    double acceleration_half_box,
    const ProgressCompatibilityConfig& config) {
    if (!nominal.allFinite() || !std::isfinite(acceleration_half_box) ||
        acceleration_half_box <= 0.0 ||
        !std::isfinite(config.max_projection_norm) ||
        config.max_projection_norm < 0.0 ||
        !std::isfinite(config.min_direction_ratio) ||
        config.min_direction_ratio < 0.0 ||
        config.min_direction_ratio > 1.0 ||
        !std::isfinite(config.comparison_tolerance) ||
        config.comparison_tolerance < 0.0) {
        throw std::invalid_argument("invalid progress compatibility input");
    }

    ProgressCompatibilityResult result;
    const auto projection = projectBridgeHocbfNominalAcceleration(
        nominal.x(), nominal.y(), acceleration_half_box,
        canonicalOwnerHalfspaces(rows, owner),
        config.comparison_tolerance);
    if (!projection.feasible) {
        result.reason = "hard_polytope_empty";
        return result;
    }
    result.polytope_nonempty = true;
    result.projection = {projection.projected_ax, projection.projected_ay};
    result.projection_norm = projection.projection_norm;
    result.minimum_hard_residual = minimumCanonicalOwnerResidual(
        rows, owner, result.projection);
    if (result.projection_norm >
        config.max_projection_norm + config.comparison_tolerance) {
        result.reason = "projection_norm";
        return result;
    }

    const double nominal_squared = nominal.squaredNorm();
    result.direction_inner_product = nominal.dot(result.projection);
    result.required_direction_inner_product =
        config.min_direction_ratio * nominal_squared;
    if (nominal_squared <=
        config.comparison_tolerance * config.comparison_tolerance) {
        result.compatible = config.zero_nominal_is_compatible;
        result.reason = result.compatible ? "accepted" : "zero_nominal";
        return result;
    }
    if (result.direction_inner_product + config.comparison_tolerance <
        result.required_direction_inner_product) {
        result.reason = "direction";
        return result;
    }
    result.compatible = true;
    result.reason = "accepted";
    return result;
}

}  // namespace gf
