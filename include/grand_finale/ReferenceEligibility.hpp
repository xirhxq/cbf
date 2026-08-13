#pragma once

#include "grand_finale/CentralizedEkfOracle.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct RangeLinkState {
    double age_s;
    double quality;
};

struct EligibilityThresholds {
    double add_distance_m;
    double keep_distance_m;
    double max_aoi_s;
    double max_reference_position_eigenvalue_m2;
    double min_range_quality;
    double uncertainty_sigma;
};

struct ReferenceCandidate {
    DirectedEdge edge;
    bool eligible;
    double robust_distance_m;
    double age_s;
    std::string reason;
};

struct EligibilitySnapshot {
    std::vector<ReferenceCandidate> candidates;

    std::vector<DirectedEdge> eligibleEdges() const {
        std::vector<DirectedEdge> edges;
        for (const ReferenceCandidate& candidate : candidates) {
            if (candidate.eligible) {
                edges.push_back(candidate.edge);
            }
        }
        return edges;
    }
};

namespace detail {

inline std::size_t mobileIndex(
    const JointEstimateSnapshot& estimate,
    NodeId id) {
    const auto iterator = std::lower_bound(
        estimate.mobile_ids.begin(), estimate.mobile_ids.end(), id);
    if (iterator == estimate.mobile_ids.end() || *iterator != id) {
        throw std::invalid_argument("unknown mobile node");
    }
    return static_cast<std::size_t>(
        std::distance(estimate.mobile_ids.begin(), iterator));
}

inline Eigen::Vector2d nodePosition(
    const JointEstimateSnapshot& estimate,
    NodeId id) {
    const auto mobile = std::lower_bound(
        estimate.mobile_ids.begin(), estimate.mobile_ids.end(), id);
    if (mobile != estimate.mobile_ids.end() && *mobile == id) {
        const Eigen::Index offset = static_cast<Eigen::Index>(
            4 * std::distance(estimate.mobile_ids.begin(), mobile));
        return estimate.mean.segment<2>(offset);
    }
    const auto fixed = estimate.fixed_positions.find(id);
    if (fixed == estimate.fixed_positions.end()) {
        throw std::invalid_argument("unknown reference node");
    }
    return fixed->second;
}

inline Eigen::Matrix2d positionCovariance(
    const JointEstimateSnapshot& estimate,
    NodeId id) {
    const auto mobile = std::lower_bound(
        estimate.mobile_ids.begin(), estimate.mobile_ids.end(), id);
    if (mobile != estimate.mobile_ids.end() && *mobile == id) {
        const Eigen::Index offset = static_cast<Eigen::Index>(
            4 * std::distance(estimate.mobile_ids.begin(), mobile));
        return estimate.covariance.block<2, 2>(offset, offset);
    }
    if (estimate.fixed_positions.count(id) != 0) {
        return Eigen::Matrix2d::Zero();
    }
    throw std::invalid_argument("unknown reference node");
}

inline double maximumPositionEigenvalue(
    const JointEstimateSnapshot& estimate,
    NodeId id) {
    return Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
               positionCovariance(estimate, id))
        .eigenvalues()
        .maxCoeff();
}

}  // namespace detail

inline EligibilitySnapshot buildEligibility(
    const JointEstimateSnapshot& estimate,
    const std::map<std::string, RangeLinkState>& range_links,
    const EligibilityThresholds& thresholds,
    const std::vector<DirectedEdge>& current_edges) {
    if (!(thresholds.add_distance_m > 0.0) ||
        !std::isfinite(thresholds.add_distance_m) ||
        !std::isfinite(thresholds.keep_distance_m) ||
        thresholds.keep_distance_m < thresholds.add_distance_m ||
        !std::isfinite(thresholds.max_aoi_s) ||
        thresholds.max_aoi_s < 0.0 ||
        !std::isfinite(thresholds.max_reference_position_eigenvalue_m2) ||
        thresholds.max_reference_position_eigenvalue_m2 < 0.0 ||
        !std::isfinite(thresholds.min_range_quality) ||
        thresholds.min_range_quality < 0.0 ||
        !std::isfinite(thresholds.uncertainty_sigma) ||
        thresholds.uncertainty_sigma < 0.0) {
        throw std::invalid_argument("invalid eligibility thresholds");
    }

    std::set<std::string> current_ids;
    for (const DirectedEdge& edge : current_edges) {
        current_ids.insert(edge.id());
    }

    std::vector<NodeId> references = estimate.mobile_ids;
    for (const auto& [id, position] : estimate.fixed_positions) {
        (void)position;
        references.push_back(id);
    }
    std::sort(references.begin(), references.end());

    EligibilitySnapshot result;
    for (NodeId owner : estimate.mobile_ids) {
        const double owner_eigenvalue =
            detail::maximumPositionEigenvalue(estimate, owner);
        for (NodeId reference : references) {
            if (reference == owner) {
                continue;
            }

            const DirectedEdge edge{reference, owner};
            const double reference_eigenvalue =
                detail::maximumPositionEigenvalue(estimate, reference);
            const double robust_distance =
                (detail::nodePosition(estimate, owner) -
                 detail::nodePosition(estimate, reference)).norm() +
                thresholds.uncertainty_sigma *
                    (std::sqrt(std::max(0.0, owner_eigenvalue)) +
                     std::sqrt(std::max(0.0, reference_eigenvalue)));

            bool eligible = true;
            std::string reason;
            double age_s = std::numeric_limits<double>::infinity();
            if (!std::isfinite(reference_eigenvalue) ||
                reference_eigenvalue >
                    thresholds.max_reference_position_eigenvalue_m2) {
                eligible = false;
                reason = "reference_covariance";
            }

            const std::string range_id =
                UndirectedEdge::canonical(reference, owner).id();
            const auto link = range_links.find(range_id);
            if (eligible && link == range_links.end()) {
                eligible = false;
                reason = "range_missing";
            } else if (link != range_links.end()) {
                age_s = link->second.age_s;
                if (eligible &&
                    (!std::isfinite(age_s) || age_s > thresholds.max_aoi_s)) {
                    eligible = false;
                    reason = "range_aoi";
                }
                if (eligible &&
                    (!std::isfinite(link->second.quality) ||
                     link->second.quality < thresholds.min_range_quality)) {
                    eligible = false;
                    reason = "range_quality";
                }
            }

            const double distance_limit = current_ids.count(edge.id()) != 0
                ? thresholds.keep_distance_m
                : thresholds.add_distance_m;
            if (eligible &&
                (!std::isfinite(robust_distance) ||
                 robust_distance > distance_limit)) {
                eligible = false;
                reason = "robust_distance";
            }

            result.candidates.push_back(ReferenceCandidate{
                edge, eligible, robust_distance, age_s, reason});
        }
    }
    return result;
}

}  // namespace gf
