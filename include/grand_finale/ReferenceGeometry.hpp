#pragma once

#include "grand_finale/ReferenceEligibility.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

inline Eigen::Matrix2d referenceFim(
    NodeId owner,
    const std::vector<DirectedEdge>& selected_edges,
    const JointEstimateSnapshot& estimate,
    const std::map<std::string, double>& range_variances_m2) {
    Eigen::Matrix2d fim = Eigen::Matrix2d::Zero();
    const Eigen::Vector2d owner_position =
        detail::nodePosition(estimate, owner);

    for (const DirectedEdge& edge : selected_edges) {
        if (edge.owner != owner) {
            throw std::invalid_argument(
                "reference FIM edge has the wrong owner");
        }
        const Eigen::Vector2d delta =
            owner_position - detail::nodePosition(estimate, edge.reference);
        const double distance = delta.norm();
        if (!std::isfinite(distance) || distance <= 1e-12) {
            throw std::invalid_argument(
                "reference FIM is undefined at coincident positions");
        }
        const Eigen::Vector2d direction = delta / distance;
        const std::string range_id =
            UndirectedEdge::canonical(owner, edge.reference).id();
        const auto variance = range_variances_m2.find(range_id);
        if (variance == range_variances_m2.end() ||
            !std::isfinite(variance->second) || variance->second <= 0.0) {
            throw std::invalid_argument(
                "reference FIM requires positive finite range variance");
        }
        const Eigen::Matrix2d reference_covariance =
            detail::positionCovariance(estimate, edge.reference);
        const double effective_variance =
            variance->second +
            (direction.transpose() * reference_covariance * direction)(0, 0);
        if (!std::isfinite(effective_variance) || effective_variance <= 0.0) {
            throw std::invalid_argument(
                "reference FIM effective variance must be positive and finite");
        }
        fim += direction * direction.transpose() / effective_variance;
    }
    return 0.5 * (fim + fim.transpose());
}

inline bool referenceCountsValid(
    const std::vector<NodeId>& mobile_ids,
    const std::vector<DirectedEdge>& selected_edges,
    std::size_t r_max) {
    if (r_max < 2) {
        return false;
    }
    std::map<NodeId, std::size_t> counts;
    for (NodeId id : mobile_ids) {
        counts[id] = 0;
    }
    std::set<std::string> unique_edges;
    for (const DirectedEdge& edge : selected_edges) {
        const auto owner = counts.find(edge.owner);
        if (owner == counts.end() || !unique_edges.insert(edge.id()).second) {
            return false;
        }
        ++owner->second;
    }
    for (const auto& [id, count] : counts) {
        (void)id;
        if (count < 2 || count > r_max) {
            return false;
        }
    }
    return true;
}

inline Eigen::Matrix2d marginalPositionCovariance(
    const JointEstimateSnapshot& estimate,
    NodeId owner) {
    const std::size_t index = detail::mobileIndex(estimate, owner);
    const Eigen::Index offset = static_cast<Eigen::Index>(4 * index);
    return estimate.covariance.block<2, 2>(offset, offset);
}

inline Eigen::Matrix2d schurEffectivePositionInformation(
    const JointEstimateSnapshot& estimate,
    NodeId owner) {
    if (estimate.covariance.rows() != estimate.covariance.cols() ||
        estimate.covariance.rows() != estimate.mean.size()) {
        throw std::invalid_argument("joint estimate dimensions are inconsistent");
    }
    const Eigen::LDLT<Eigen::MatrixXd> covariance_ldlt(estimate.covariance);
    if (covariance_ldlt.info() != Eigen::Success || !covariance_ldlt.isPositive()) {
        throw std::invalid_argument(
            "joint covariance must be positive definite for Schur information");
    }
    const Eigen::MatrixXd information = covariance_ldlt.solve(
        Eigen::MatrixXd::Identity(
            estimate.covariance.rows(), estimate.covariance.cols()));

    const std::size_t mobile_index = detail::mobileIndex(estimate, owner);
    const Eigen::Index first_position =
        static_cast<Eigen::Index>(4 * mobile_index);
    std::vector<Eigen::Index> complement;
    for (Eigen::Index index = 0; index < information.rows(); ++index) {
        if (index != first_position && index != first_position + 1) {
            complement.push_back(index);
        }
    }

    Eigen::MatrixXd cross(2, complement.size());
    Eigen::MatrixXd complement_information(
        complement.size(), complement.size());
    for (std::size_t column = 0; column < complement.size(); ++column) {
        cross(0, static_cast<Eigen::Index>(column)) =
            information(first_position, complement[column]);
        cross(1, static_cast<Eigen::Index>(column)) =
            information(first_position + 1, complement[column]);
        for (std::size_t row = 0; row < complement.size(); ++row) {
            complement_information(
                static_cast<Eigen::Index>(row),
                static_cast<Eigen::Index>(column)) =
                information(complement[row], complement[column]);
        }
    }

    const Eigen::Matrix2d direct =
        information.block<2, 2>(first_position, first_position);
    const Eigen::Matrix2d effective =
        direct - cross * complement_information.ldlt().solve(cross.transpose());
    return 0.5 * (effective + effective.transpose());
}

inline bool posteriorPositionHealthy(
    const JointEstimateSnapshot& estimate,
    NodeId owner,
    double maximum_eigenvalue_m2) {
    if (!std::isfinite(maximum_eigenvalue_m2) || maximum_eigenvalue_m2 < 0.0) {
        throw std::invalid_argument(
            "posterior eigenvalue threshold must be non-negative and finite");
    }
    const double maximum =
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
            marginalPositionCovariance(estimate, owner))
            .eigenvalues()
            .maxCoeff();
    return std::isfinite(maximum) && maximum <= maximum_eigenvalue_m2;
}

}  // namespace gf
