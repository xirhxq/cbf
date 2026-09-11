#pragma once

#include "grand_finale/ReferenceGeometry.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace gf {

struct RobustReferenceFimResult {
    bool valid = false;
    std::string reason;
    double lower_eigenvalue = 0.0;
    std::string first_reference_edge_id;
    std::string second_reference_edge_id;
};

namespace robust_reference_fim_detail {

struct DirectionCone {
    DirectedEdge edge;
    Eigen::Vector2d nominal = Eigen::Vector2d::Zero();
    double half_angle = 0.0;
    double information_weight_lower = 0.0;
    bool unbounded = false;
};

inline double pairLowerEigenvalue(
    const DirectionCone& first,
    const DirectionCone& second) {
    if (first.unbounded || second.unbounded) return 0.0;
    const double pi = std::acos(-1.0);
    const double nominal_angle = std::acos(std::clamp(
        first.nominal.dot(second.nominal), -1.0, 1.0));
    const double low = std::max(
        0.0, nominal_angle - first.half_angle - second.half_angle);
    const double high = std::min(
        pi, nominal_angle + first.half_angle + second.half_angle);
    if (low <= 0.0 || high >= pi) return 0.0;
    const double minimum_sine_squared = std::min(
        std::pow(std::sin(low), 2),
        std::pow(std::sin(high), 2));
    const double first_weight = first.information_weight_lower;
    const double second_weight = second.information_weight_lower;
    const double discriminant =
        (first_weight - second_weight) *
            (first_weight - second_weight) +
        4.0 * first_weight * second_weight *
            (1.0 - minimum_sine_squared);
    return std::max(
        0.0,
        0.5 * (first_weight + second_weight -
               std::sqrt(std::max(0.0, discriminant))));
}

}  // namespace robust_reference_fim_detail

inline RobustReferenceFimResult robustReferenceFimLowerBound(
    NodeId owner,
    const std::vector<DirectedEdge>& selected_edges,
    const JointEstimateSnapshot& estimate,
    const std::map<std::string, double>& range_variances_m2,
    const std::map<std::string, double>& direction_support_m) {
    using namespace robust_reference_fim_detail;
    RobustReferenceFimResult result;
    std::vector<DirectionCone> cones;
    const Eigen::Vector2d owner_position =
        detail::nodePosition(estimate, owner);
    for (const DirectedEdge& edge : selected_edges) {
        if (edge.owner != owner) {
            result.reason = "wrong_owner";
            return result;
        }
        const auto support = direction_support_m.find(edge.id());
        if (support == direction_support_m.end()) {
            result.reason = "direction_support_missing";
            return result;
        }
        if (!std::isfinite(support->second) || support->second < 0.0) {
            result.reason = "direction_support_invalid";
            return result;
        }
        const Eigen::Vector2d delta =
            owner_position - detail::nodePosition(estimate, edge.reference);
        const double distance = delta.norm();
        if (!std::isfinite(distance) || distance <= 1.0e-12) {
            result.reason = "direction_singularity";
            return result;
        }
        const std::string range_id = UndirectedEdge::canonical(
            owner, edge.reference).id();
        const auto variance = range_variances_m2.find(range_id);
        if (variance == range_variances_m2.end() ||
            !std::isfinite(variance->second) || variance->second <= 0.0) {
            result.reason = "range_variance_missing";
            return result;
        }
        const double reference_covariance_upper = std::max(
            0.0,
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                detail::positionCovariance(estimate, edge.reference))
                .eigenvalues().maxCoeff());
        DirectionCone cone{edge};
        cone.nominal = delta / distance;
        cone.unbounded = support->second >= distance;
        cone.half_angle = cone.unbounded
            ? 0.5 * std::acos(-1.0)
            : std::asin(std::clamp(
                  support->second / distance, 0.0, 1.0));
        cone.information_weight_lower =
            1.0 / (variance->second + reference_covariance_upper);
        cones.push_back(std::move(cone));
    }
    if (cones.size() < 2) {
        result.reason = "insufficient_references";
        return result;
    }

    double best = -1.0;
    for (std::size_t first = 0; first < cones.size(); ++first) {
        for (std::size_t second = first + 1;
             second < cones.size(); ++second) {
            const double lower = pairLowerEigenvalue(
                cones[first], cones[second]);
            if (lower > best) {
                best = lower;
                result.first_reference_edge_id = cones[first].edge.id();
                result.second_reference_edge_id = cones[second].edge.id();
            }
        }
    }
    result.valid = true;
    result.lower_eigenvalue = std::max(0.0, best);
    result.reason = result.lower_eigenvalue > 0.0
        ? "certified"
        : "direction_cone_unbounded";
    return result;
}

}  // namespace gf

