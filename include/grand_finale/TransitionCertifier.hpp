#pragma once

#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/ReferenceGeometry.hpp"
#include "grand_finale/ProgressCompatibility.hpp"
#include "grand_finale/TopologyModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace gf {

struct CertifiedEdgeGate {
    bool add_valid = false;
    bool keep_valid = false;
    double robust_distance_m = std::numeric_limits<double>::infinity();
};

struct TransitionCertificationContext {
    std::uint64_t topology_version = 0;
    std::uint64_t estimator_version = 0;
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::size_t r_max = 0;
    double max_reference_distance_m = 0.0;
    double min_fim_eigenvalue = 0.0;
    double max_posterior_eigenvalue_m2 = 0.0;
    double gamma_accept = 0.0;
    JointEstimateSnapshot estimate;
    std::map<std::string, double> range_variances_m2;
    std::map<std::string, CertifiedEdgeGate> edge_gates;
    CanonicalHardRowRequest hard_row_request;
    std::map<NodeId, Eigen::Vector2d> nominal_controls;
    ProgressCompatibilityConfig progress_compatibility;
};

struct TransitionProposal {
    std::vector<DirectedEdge> old_edges;
    DirectedEdge new_edge;
    DirectedEdge old_edge;
    std::uint64_t expected_topology_version;
    std::uint64_t expected_estimator_version;
};

struct CertifiedTopologyState {
    bool valid = false;
    std::string reason;
    double minimum_gamma = std::numeric_limits<double>::infinity();
};

struct TransitionCertificate {
    bool valid = false;
    bool forward_valid = false;
    bool reverse_valid = false;
    std::string reason;
    std::uint64_t topology_version = 0;
    std::uint64_t estimator_version = 0;
    std::vector<DirectedEdge> old_edges;
    std::optional<DirectedEdge> new_edge;
    std::optional<DirectedEdge> old_edge;
    std::vector<DirectedEdge> union_edges;
    std::vector<DirectedEdge> successor_edges;
    CertifiedTopologyState old_state;
    CertifiedTopologyState union_state;
    CertifiedTopologyState successor_state;
    double minimum_gamma = -std::numeric_limits<double>::infinity();
};

namespace transition_certifier_detail {

inline std::vector<DirectedEdge> canonicalEdges(
    std::vector<DirectedEdge> edges) {
    std::sort(edges.begin(), edges.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.id() < rhs.id();
    });
    edges.erase(std::unique(edges.begin(), edges.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.id() == rhs.id();
    }), edges.end());
    return edges;
}

inline bool contains(
    const std::vector<DirectedEdge>& edges,
    const DirectedEdge& target) {
    return std::any_of(edges.begin(), edges.end(), [&](const auto& edge) {
        return edge.id() == target.id();
    });
}

inline std::vector<DirectedEdge> without(
    const std::vector<DirectedEdge>& edges,
    const DirectedEdge& target) {
    std::vector<DirectedEdge> result;
    for (const auto& edge : edges) {
        if (edge.id() != target.id()) result.push_back(edge);
    }
    return result;
}

inline CertifiedTopologyState evaluateState(
    const std::vector<DirectedEdge>& edges,
    const TransitionCertificationContext& context,
    std::size_t maximum_indegree,
    const std::set<std::string>& add_ids) {
    CertifiedTopologyState result;
    if (!referenceCountsValid(context.mobile_ids, edges, maximum_indegree)) {
        result.reason = "indegree";
        return result;
    }
    TopologyRequest topology_request{
        context.mobile_ids, context.fixed_ids, edges, edges,
        2, maximum_indegree, {}, {}, {}};
    const TopologyEvaluation topology = TopologyModel(topology_request).evaluate(edges);
    if (!topology.valid) {
        result.reason = topology.reason == "cycle" ? "dag" : topology.reason;
        return result;
    }
    for (const DirectedEdge& edge : edges) {
        const auto gate = context.edge_gates.find(edge.id());
        const bool add = add_ids.count(edge.id()) != 0;
        if (gate == context.edge_gates.end() ||
            (add ? !gate->second.add_valid : !gate->second.keep_valid)) {
            result.reason = add ? "add_eligibility" : "keep_eligibility";
            return result;
        }
        if (!std::isfinite(gate->second.robust_distance_m) ||
            gate->second.robust_distance_m > context.max_reference_distance_m) {
            result.reason = "reference_distance";
            return result;
        }
    }
    for (NodeId owner : context.mobile_ids) {
        if (!posteriorPositionHealthy(
                context.estimate, owner,
                context.max_posterior_eigenvalue_m2)) {
            result.reason = "posterior";
            return result;
        }
        std::vector<DirectedEdge> owner_edges;
        for (const DirectedEdge& edge : edges)
            if (edge.owner == owner) owner_edges.push_back(edge);
        const Eigen::Matrix2d fim = referenceFim(
            owner, owner_edges, context.estimate,
            context.range_variances_m2);
        const double minimum_fim =
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(fim)
                .eigenvalues().minCoeff();
        if (!std::isfinite(minimum_fim) ||
            minimum_fim < context.min_fim_eigenvalue) {
            result.reason = "fim";
            return result;
        }
    }
    CanonicalHardRowRequest hard_row_request = context.hard_row_request;
    hard_row_request.reference_edges = edges;
    const std::vector<CanonicalHardRow> hard_rows =
        buildCanonicalHardRows(std::move(hard_row_request));
    if (!context.nominal_controls.empty()) {
        for (NodeId owner : context.mobile_ids) {
            const auto nominal = context.nominal_controls.find(owner);
            if (nominal == context.nominal_controls.end()) {
                result.reason = "progress_nominal_missing";
                return result;
            }
            const auto progress = evaluateProgressCompatibility(
                hard_rows, owner, nominal->second,
                context.hard_row_request.acceleration_half_box,
                context.progress_compatibility);
            if (!progress.compatible) {
                result.reason = "progress_" + progress.reason;
                return result;
            }
        }
    }
    for (const CanonicalHardRow& row : hard_rows) {
        if (row.kind != CanonicalHardRowKind::ReferenceDistance &&
            row.kind != CanonicalHardRowKind::Collision) {
            continue;
        }
        if (!std::isfinite(row.barrier_h) ||
            !std::isfinite(row.barrier_psi1) ||
            row.barrier_h < -1e-12 || row.barrier_psi1 < -1e-12) {
            result.reason = row.kind == CanonicalHardRowKind::Collision
                ? "collision_initial_set"
                : "reference_initial_set";
            return result;
        }
    }
    result.minimum_gamma = std::numeric_limits<double>::infinity();
    for (NodeId owner : context.mobile_ids) {
        const auto gamma = solveCanonicalGammaStar(
            hard_rows, owner,
            context.hard_row_request.acceleration_half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma)) {
            result.reason = "gamma_invalid";
            return result;
        }
        result.minimum_gamma = std::min(result.minimum_gamma, gamma.gamma);
    }
    if (result.minimum_gamma < context.gamma_accept) {
        result.reason = "gamma_accept";
        return result;
    }
    result.valid = true;
    return result;
}

}  // namespace transition_certifier_detail

class TransitionCertifier {
public:
    TransitionCertificate certify(
        const TransitionProposal& proposal,
        const TransitionCertificationContext& context,
        bool require_reverse) const {
        using namespace transition_certifier_detail;
        TransitionCertificate certificate;
        certificate.topology_version = context.topology_version;
        certificate.estimator_version = context.estimator_version;
        if (proposal.expected_topology_version != context.topology_version ||
            proposal.expected_estimator_version != context.estimator_version) {
            certificate.reason = "stale_snapshot";
            return certificate;
        }
        if (!std::isfinite(context.max_reference_distance_m) ||
            context.max_reference_distance_m <= 0.0 ||
            !std::isfinite(context.min_fim_eigenvalue) ||
            context.min_fim_eigenvalue < 0.0 ||
            !std::isfinite(context.gamma_accept) ||
            context.gamma_accept <= 0.0 || context.r_max < 2) {
            certificate.reason = "invalid_context";
            return certificate;
        }
        const std::vector<DirectedEdge> old_edges = canonicalEdges(proposal.old_edges);
        if (old_edges.size() != proposal.old_edges.size() ||
            !contains(old_edges, proposal.old_edge) ||
            contains(old_edges, proposal.new_edge) ||
            proposal.new_edge.owner != proposal.old_edge.owner) {
            certificate.reason = "invalid_replacement";
            return certificate;
        }
        certificate.old_edges = old_edges;
        certificate.new_edge = proposal.new_edge;
        certificate.old_edge = proposal.old_edge;
        certificate.union_edges = old_edges;
        certificate.union_edges.push_back(proposal.new_edge);
        certificate.union_edges = canonicalEdges(std::move(certificate.union_edges));
        certificate.successor_edges = canonicalEdges(
            without(certificate.union_edges, proposal.old_edge));

        certificate.old_state = evaluateState(
            old_edges, context, context.r_max, {});
        if (!certificate.old_state.valid) {
            certificate.reason = "old_state";
            return certificate;
        }
        certificate.union_state = evaluateState(
            certificate.union_edges, context, context.r_max + 1,
            {proposal.new_edge.id()});
        if (!certificate.union_state.valid) {
            certificate.reason = "union_state";
            return certificate;
        }
        certificate.successor_state = evaluateState(
            certificate.successor_edges, context, context.r_max,
            {proposal.new_edge.id()});
        if (!certificate.successor_state.valid) {
            certificate.reason = "successor_state";
            return certificate;
        }
        certificate.forward_valid = true;
        certificate.minimum_gamma = std::min({
            certificate.old_state.minimum_gamma,
            certificate.union_state.minimum_gamma,
            certificate.successor_state.minimum_gamma});

        certificate.reverse_valid = true;
        if (require_reverse) {
            const auto old_gate = context.edge_gates.find(proposal.old_edge.id());
            if (old_gate == context.edge_gates.end() ||
                !old_gate->second.add_valid) {
                certificate.reverse_valid = false;
            } else {
                const CertifiedTopologyState reverse_union = evaluateState(
                    certificate.union_edges, context, context.r_max + 1,
                    {proposal.old_edge.id()});
                const CertifiedTopologyState reverse_successor = evaluateState(
                    old_edges, context, context.r_max,
                    {proposal.old_edge.id()});
                certificate.reverse_valid =
                    reverse_union.valid && reverse_successor.valid;
            }
            if (!certificate.reverse_valid) {
                certificate.reason = "reverse_certificate";
                return certificate;
            }
        }
        certificate.valid = true;
        certificate.reason = "certified";
        return certificate;
    }
};

}  // namespace gf
