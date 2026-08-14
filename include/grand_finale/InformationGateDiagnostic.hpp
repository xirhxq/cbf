#pragma once

#include "grand_finale/ReferenceGeometry.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <map>

namespace gf {

enum class InformationFailureCause {
    None,
    ScientificRejection,
    InsufficientInitialization,
    CandidateGenerationDefect,
    SolverFailure,
    DataFlowError
};

struct InformationGateEdgeInput {
    DirectedEdge edge;
    bool keep_distance_valid = false;
    bool add_distance_valid = false;
    bool range_present = false;
    double aoi_margin_s = 0.0;
    double quality_margin = 0.0;
    bool hocbf_initial_set_valid = false;
};

struct InformationGateOwnerInput {
    NodeId owner = 0;
    bool initialized = false;
    double posterior_margin_m2 = 0.0;
    double nominal_fim_eigenvalue = 0.0;
    double posterior_fim_proxy_eigenvalue = 0.0;
    double robust_cone_fim_lower_bound = 0.0;
    double minimum_fim_eigenvalue = 0.0;
    bool candidate_requested = false;
    bool candidate_generated = false;
    bool solver_succeeded = false;
    std::vector<InformationGateEdgeInput> edges;
};

struct InformationGateCertificate {
    NodeId owner = 0;
    bool accepted = false;
    InformationFailureCause cause = InformationFailureCause::None;
    std::string first_failed_gate;
    std::size_t effective_reference_count = 0;
    double posterior_margin_m2 = 0.0;
    double nominal_fim_eigenvalue = 0.0;
    double posterior_fim_proxy_eigenvalue = 0.0;
    double robust_cone_fim_lower_bound = 0.0;
    std::vector<InformationGateEdgeInput> edges;
};

inline double robustReferenceFimConeLowerBound(
    NodeId owner,
    const std::vector<DirectedEdge>& edges,
    const JointEstimateSnapshot& estimate,
    const std::map<std::string, double>& range_variances_m2,
    double uncertainty_sigma) {
    if (!std::isfinite(uncertainty_sigma) || uncertainty_sigma < 0.0)
        throw std::invalid_argument("invalid robust FIM cone sigma");
    const Eigen::Matrix2d nominal = referenceFim(
        owner, edges, estimate, range_variances_m2);
    double projector_reserve = 0.0;
    const Eigen::Vector2d owner_position = detail::nodePosition(estimate, owner);
    const double owner_sigma = uncertainty_sigma * std::sqrt(std::max(
        0.0, detail::maximumPositionEigenvalue(estimate, owner)));
    for (const auto& edge : edges) {
        const Eigen::Vector2d delta = owner_position -
            detail::nodePosition(estimate, edge.reference);
        const double distance = delta.norm();
        const double reference_sigma = uncertainty_sigma * std::sqrt(std::max(
            0.0, detail::maximumPositionEigenvalue(
                estimate, edge.reference)));
        const double radius = owner_sigma + reference_sigma;
        if (!std::isfinite(distance) || distance <= radius || distance <= 1e-12)
            return -std::numeric_limits<double>::infinity();
        const double angle = std::asin(std::min(1.0, radius / distance));
        const std::string id = UndirectedEdge::canonical(
            owner, edge.reference).id();
        const auto variance = range_variances_m2.find(id);
        if (variance == range_variances_m2.end() || variance->second <= 0.0)
            return -std::numeric_limits<double>::infinity();
        const Eigen::Vector2d direction = delta / distance;
        const double effective_variance = variance->second +
            (direction.transpose() * detail::positionCovariance(
                estimate, edge.reference) * direction)(0, 0);
        projector_reserve += std::sin(angle) / effective_variance;
    }
    const double nominal_min = Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
        nominal).eigenvalues().minCoeff();
    return nominal_min - projector_reserve;
}

inline InformationGateCertificate diagnoseInformationGate(
    const InformationGateOwnerInput& input) {
    InformationGateCertificate result;
    result.owner = input.owner;
    result.posterior_margin_m2 = input.posterior_margin_m2;
    result.nominal_fim_eigenvalue = input.nominal_fim_eigenvalue;
    result.posterior_fim_proxy_eigenvalue =
        input.posterior_fim_proxy_eigenvalue;
    result.robust_cone_fim_lower_bound = input.robust_cone_fim_lower_bound;
    result.edges = input.edges;
    const auto fail = [&](InformationFailureCause cause,
                          const std::string& gate) {
        result.cause = cause;
        result.first_failed_gate = gate;
        return result;
    };
    if (input.owner <= 0 || !input.initialized)
        return fail(InformationFailureCause::InsufficientInitialization,
                    "initialization");
    for (const auto& edge : input.edges) {
        if (!edge.range_present)
            return fail(InformationFailureCause::DataFlowError,
                        "range_missing:" + edge.edge.id());
        if (edge.aoi_margin_s < 0.0)
            return fail(InformationFailureCause::ScientificRejection,
                        "aoi:" + edge.edge.id());
        if (edge.quality_margin < 0.0)
            return fail(InformationFailureCause::ScientificRejection,
                        "range_quality:" + edge.edge.id());
        if (!edge.keep_distance_valid)
            return fail(InformationFailureCause::ScientificRejection,
                        "keep_distance:" + edge.edge.id());
        if (!edge.hocbf_initial_set_valid)
            return fail(InformationFailureCause::ScientificRejection,
                        "hocbf_initial_set:" + edge.edge.id());
        ++result.effective_reference_count;
    }
    if (!std::isfinite(input.posterior_margin_m2) ||
        input.posterior_margin_m2 < 0.0)
        return fail(InformationFailureCause::ScientificRejection, "posterior");
    if (result.effective_reference_count < 2)
        return fail(InformationFailureCause::ScientificRejection,
                    "double_reference");
    if (!std::isfinite(input.posterior_fim_proxy_eigenvalue) ||
        input.posterior_fim_proxy_eigenvalue + 1.0e-12 <
            input.minimum_fim_eigenvalue)
        return fail(InformationFailureCause::ScientificRejection,
                    "posterior_fim_proxy");
    if (!std::isfinite(input.robust_cone_fim_lower_bound) ||
        input.robust_cone_fim_lower_bound + 1.0e-12 <
            input.minimum_fim_eigenvalue)
        return fail(InformationFailureCause::ScientificRejection,
                    "robust_cone_fim");
    if (input.candidate_requested && !input.candidate_generated)
        return fail(InformationFailureCause::CandidateGenerationDefect,
                    "candidate_generation");
    if (input.candidate_requested && !input.solver_succeeded)
        return fail(InformationFailureCause::SolverFailure,
                    "topology_solver");
    result.accepted = true;
    return result;
}

}  // namespace gf
