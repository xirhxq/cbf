#pragma once

#include "grand_finale/CertifiedCoverageTracker.hpp"
#include "grand_finale/GurobiTopologySolver.hpp"
#include "grand_finale/HighsTopologySolver.hpp"
#include "grand_finale/HybridSupervisor.hpp"
#include "grand_finale/InterimMasterDekf.hpp"
#include "grand_finale/ReferenceEligibility.hpp"
#include "models/DoubleIntegrate2D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gf {

struct GrandFinaleSummary {
    std::size_t mobile_count = 0;
    std::size_t fixed_count = 0;
    std::size_t estimator_updates = 0;
    std::size_t second_order_control_steps = 0;
    std::size_t successful_reconfigurations = 0;
    bool visited_union_state = false;
    double certified_coverage_initial = 0.0;
    double certified_coverage_final = 0.0;
    bool reached_certified_t100 = false;
    double maximum_estimation_error_m = 0.0;
    double minimum_estimator_tube_slack_m =
        std::numeric_limits<double>::infinity();
    std::size_t false_covered_cells = 0;
    std::size_t minimum_reference_count =
        std::numeric_limits<std::size_t>::max();
    double maximum_robust_reference_distance_m = 0.0;
    double minimum_gamma_star = std::numeric_limits<double>::infinity();
    double minimum_hard_margin = std::numeric_limits<double>::infinity();
    std::size_t hard_gate_violations = 0;
    std::uint64_t topology_version = 0;
    SupervisorMode final_mode = SupervisorMode::Search;
};

namespace grand_finale_experiment_detail {

using TruthStateMap = std::map<NodeId, Eigen::Vector4d>;

inline Eigen::Vector2d position(
    const TruthStateMap& truth,
    const std::map<NodeId, Eigen::Vector2d>& fixed,
    NodeId id) {
    const auto mobile = truth.find(id);
    return mobile == truth.end() ? fixed.at(id) : mobile->second.head<2>();
}

inline std::vector<MobileEstimate> estimates(const TruthStateMap& truth) {
    std::vector<MobileEstimate> result;
    for (const auto& [id, state] : truth) {
        result.push_back(MobileEstimate{
            id, state, 1e-4 * Eigen::Matrix4d::Identity()});
    }
    return result;
}

inline std::unique_ptr<TopologySolver> solver(SolverProfile profile) {
    if (profile == SolverProfile::OpenSource) {
        return std::make_unique<HighsTopologySolver>();
    }
#ifdef ENABLE_GUROBI
    return std::make_unique<GurobiTopologySolver>();
#else
    throw std::runtime_error("Gurobi profile is unavailable");
#endif
}

inline std::vector<RangeMeasurement> rangeBatch(
    std::int64_t timestamp_ns,
    const std::vector<DirectedEdge>& physical_edges,
    const TruthStateMap& truth,
    const std::map<NodeId, Eigen::Vector2d>& fixed) {
    std::map<std::string, UndirectedEdge> unique;
    for (const DirectedEdge& edge : physical_edges) {
        const UndirectedEdge undirected =
            UndirectedEdge::canonical(edge.reference, edge.owner);
        unique.emplace(undirected.id(), undirected);
    }
    std::vector<RangeMeasurement> batch;
    for (const auto& [id, edge] : unique) {
        (void)id;
        batch.push_back(RangeMeasurement{
            timestamp_ns, edge,
            (position(truth, fixed, edge.first) -
             position(truth, fixed, edge.second)).norm(),
            1.0});
    }
    return canonicalizeRangeBatch(std::move(batch));
}

inline std::size_t minimumReferenceCount(
    const std::vector<NodeId>& mobiles,
    const std::vector<DirectedEdge>& edges) {
    std::map<NodeId, std::size_t> counts;
    for (NodeId id : mobiles) counts[id] = 0;
    for (const DirectedEdge& edge : edges) ++counts.at(edge.owner);
    std::size_t minimum = std::numeric_limits<std::size_t>::max();
    for (const auto& [id, count] : counts) {
        (void)id;
        minimum = std::min(minimum, count);
    }
    return minimum;
}

inline std::vector<DirectedEdge> edgeDifference(
    const std::vector<DirectedEdge>& first,
    const std::vector<DirectedEdge>& second) {
    std::set<std::string> second_ids;
    for (const auto& edge : second) second_ids.insert(edge.id());
    std::vector<DirectedEdge> result;
    for (const auto& edge : first)
        if (second_ids.count(edge.id()) == 0) result.push_back(edge);
    return result;
}

inline CanonicalHardRowRequest hardRowRequest(
    const std::vector<NodeId>& mobiles,
    const std::vector<NodeId>& fixed_ids,
    const JointEstimateSnapshot& estimate,
    const std::vector<DirectedEdge>& topology) {
    CanonicalHardRowRequest request;
    request.mobile_ids = mobiles;
    request.fixed_ids = fixed_ids;
    request.reference_edges = topology;
    for (std::size_t index = 0; index < mobiles.size(); ++index) {
        const Eigen::Vector4d state = estimate.mean.segment<4>(4 * index);
        request.states[mobiles[index]] = {
            Point(state.x(), state.y()), state.tail<2>(),
            Eigen::Vector2d::Zero()};
    }
    for (NodeId id : fixed_ids) {
        const Eigen::Vector2d point = estimate.fixed_positions.at(id);
        request.states[id] = {
            Point(point.x(), point.y()), Eigen::Vector2d::Zero(),
            Eigen::Vector2d::Zero()};
    }
    for (std::size_t first = 0; first < mobiles.size(); ++first) {
        for (std::size_t second = first + 1; second < mobiles.size(); ++second) {
            request.collision_pairs.push_back(
                UndirectedEdge::canonical(mobiles[first], mobiles[second]));
        }
    }
    request.reference_spec = {
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0, 0.02, 1.0, 1.0, 1.0, 0.0};
    request.collision_spec = {
        PairwiseSecondOrderBarrierKind::CollisionLower,
        0.1, 0.0, 1.0, 1.0, 1.0, 0.0};
    request.acceleration_half_box = 0.4;
    return request;
}

inline void updateHardSummary(
    GrandFinaleSummary& summary,
    const std::vector<NodeId>& mobiles,
    const std::vector<DirectedEdge>& topology,
    const JointEstimateSnapshot& estimate,
    const CanonicalHardRowRequest& hard_request,
    const Eigen::Vector2d& applied_control) {
    summary.minimum_reference_count = std::min(
        summary.minimum_reference_count,
        minimumReferenceCount(mobiles, topology));
    std::map<NodeId, std::size_t> counts;
    for (NodeId owner : mobiles) counts[owner] = 0;
    std::size_t maximum_count = 0;
    for (const DirectedEdge& edge : topology)
        maximum_count = std::max(maximum_count, ++counts.at(edge.owner));
    const TopologyEvaluation topology_evaluation = TopologyModel(
        TopologyRequest{
            mobiles, hard_request.fixed_ids, topology, topology,
            2, maximum_count, {}, {}, {}}).evaluate(topology);
    if (!topology_evaluation.valid) ++summary.hard_gate_violations;
    for (const DirectedEdge& edge : topology) {
        const double distance =
            (detail::nodePosition(estimate, edge.owner) -
             detail::nodePosition(estimate, edge.reference)).norm();
        summary.maximum_robust_reference_distance_m = std::max(
            summary.maximum_robust_reference_distance_m, distance + 0.02);
        if (distance + 0.02 > 850.0) ++summary.hard_gate_violations;
    }
    std::map<std::string, double> unit_variances;
    for (const DirectedEdge& edge : topology) {
        unit_variances[UndirectedEdge::canonical(
            edge.reference, edge.owner).id()] = 1.0;
    }
    for (NodeId owner : mobiles) {
        std::vector<DirectedEdge> owner_edges;
        for (const DirectedEdge& edge : topology)
            if (edge.owner == owner) owner_edges.push_back(edge);
        const double minimum_fim =
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                referenceFim(owner, owner_edges, estimate, unit_variances))
                .eigenvalues().minCoeff();
        if (minimum_fim < 1e-6 ||
            !posteriorPositionHealthy(estimate, owner, 0.1)) {
            ++summary.hard_gate_violations;
        }
    }
    const std::vector<CanonicalHardRow> rows =
        buildCanonicalHardRows(hard_request);
    for (NodeId owner : mobiles) {
        const auto gamma = solveCanonicalGammaStar(rows, owner, 0.4);
        if (!gamma.valid) {
            ++summary.hard_gate_violations;
            continue;
        }
        summary.minimum_gamma_star = std::min(
            summary.minimum_gamma_star, gamma.gamma);
        for (const CanonicalHardRow& row : rows) {
            if (row.owner == owner && row.participates_in_gamma) {
                const double margin = row.margin(applied_control);
                summary.minimum_hard_margin = std::min(
                    summary.minimum_hard_margin, margin);
                if (margin < -1e-10) ++summary.hard_gate_violations;
            }
        }
    }
    if (summary.minimum_reference_count < 2)
        ++summary.hard_gate_violations;
}

}  // namespace grand_finale_experiment_detail

class GrandFinaleExperiment {
public:
    static GrandFinaleSummary runCanonical4p2(
        SolverProfile profile,
        bool inject_uncertifiable_candidate,
        std::uint32_t seed,
        std::size_t requested_reconfigurations = 1) {
        if (seed != 2027 || requested_reconfigurations > 4)
            throw std::invalid_argument("canonical smoke uses development seed 2027");
        using namespace grand_finale_experiment_detail;
        const std::vector<NodeId> mobiles = {1, 2, 3, 4};
        const std::vector<NodeId> fixed_ids = {10, 11};
        const std::map<NodeId, Eigen::Vector2d> fixed = {
            {10, {-1.0, 0.0}}, {11, {-1.0, 4.0}}};
        TruthStateMap truth = {
            {1, Eigen::Vector4d(1.0, 1.0, 0.0, 0.0)},
            {2, Eigen::Vector4d(1.0, 3.0, 0.0, 0.0)},
            {3, Eigen::Vector4d(3.0, 1.0, 0.0, 0.0)},
            {4, Eigen::Vector4d(3.0, 3.0, 0.0, 0.0)}};
        std::vector<DirectedEdge> topology = {
            {10, 1}, {11, 1}, {10, 2}, {1, 2},
            {11, 3}, {1, 3}, {2, 4}, {3, 4}};
        const std::vector<DirectedEdge> candidates = {
            {10, 1}, {11, 1}, {10, 2}, {1, 2}, {11, 2},
            {11, 3}, {1, 3}, {2, 4}, {3, 4}};

        InterimMasterDekf estimator(estimates(truth), fixed);
        HybridSupervisor supervisor({0.0, 0.05, 0.1});
        supervisor.initializeTopology(topology, 1);
        CertifiedCoverageTracker coverage({0.0, 8.0}, 4, {0.0, 4.0}, 2);
        GrandFinaleSummary summary;
        summary.mobile_count = mobiles.size();
        summary.fixed_count = fixed_ids.size();
        for (NodeId id : mobiles) {
            const Eigen::Vector2d point = truth.at(id).head<2>();
            coverage.observe(
                Point(point.x(), point.y()), Point(point.x(), point.y()),
                0.05, 1.6);
        }
        summary.certified_coverage_initial = coverage.certifiedFraction();

        std::size_t attempted_reconfigurations = 0;
        std::optional<TransitionProposal> pending_proposal;
        for (int step = 0; step < 90; ++step) {
            const Eigen::Vector2d acceleration(
                step < 45 ? 0.2 : -0.2, 0.0);
            const JointEstimateSnapshot before = estimator.reconstructForAudit();
            CanonicalHardRowRequest control_rows = hardRowRequest(
                mobiles, fixed_ids, before, topology);
            updateHardSummary(
                summary, mobiles, topology, before, control_rows,
                acceleration);
            for (NodeId id : mobiles) {
                const auto next = propagateDoubleIntegratorPlanarZoh(
                    truth.at(id).head<2>(), truth.at(id).tail<2>(),
                    acceleration, 0.1);
                truth.at(id).head<2>() = next.position;
                truth.at(id).tail<2>() = next.velocity;
                estimator.propagateLocal(id, acceleration, 0.1, 0.0);
                ++summary.second_order_control_steps;
            }
            const auto batch = rangeBatch(
                static_cast<std::int64_t>(step + 1) * 100000000LL,
                candidates, truth, fixed);
            for (const RangeMeasurement& measurement : batch) {
                const NodeId master =
                    truth.count(measurement.edge.first)
                    ? measurement.edge.first : measurement.edge.second;
                estimator.applyUpdate(estimator.makeUpdate(master, measurement));
                ++summary.estimator_updates;
            }
            const JointEstimateSnapshot estimate = estimator.reconstructForAudit();
            for (std::size_t index = 0; index < mobiles.size(); ++index) {
                const Eigen::Vector2d actual = truth.at(mobiles[index]).head<2>();
                const Eigen::Vector2d estimated = estimate.mean.segment<2>(4 * index);
                const double estimation_error = (actual - estimated).norm();
                summary.maximum_estimation_error_m = std::max(
                    summary.maximum_estimation_error_m, estimation_error);
                summary.minimum_estimator_tube_slack_m = std::min(
                    summary.minimum_estimator_tube_slack_m,
                    0.05 - estimation_error);
                coverage.observe(
                    Point(actual.x(), actual.y()),
                    Point(estimated.x(), estimated.y()), 0.05, 1.6);
            }

            if (pending_proposal.has_value()) {
                TransitionCertificationContext refreshed_context;
                refreshed_context.topology_version =
                    supervisor.topologyVersion();
                refreshed_context.estimator_version = estimator.version();
                refreshed_context.mobile_ids = mobiles;
                refreshed_context.fixed_ids = fixed_ids;
                refreshed_context.r_max = 2;
                refreshed_context.max_reference_distance_m = 850.0;
                refreshed_context.min_fim_eigenvalue = 1e-6;
                refreshed_context.max_posterior_eigenvalue_m2 = 0.1;
                refreshed_context.gamma_accept = 0.05;
                refreshed_context.estimate = estimate;
                for (const DirectedEdge& edge : candidates) {
                    const std::string range_id = UndirectedEdge::canonical(
                        edge.reference, edge.owner).id();
                    refreshed_context.range_variances_m2[range_id] = 1.0;
                    const double distance =
                        (detail::nodePosition(estimate, edge.owner) -
                         detail::nodePosition(estimate, edge.reference)).norm();
                    refreshed_context.edge_gates[edge.id()] =
                        {true, true, distance + 0.02};
                }
                refreshed_context.hard_row_request = hardRowRequest(
                    mobiles, fixed_ids, estimate, topology);
                TransitionProposal refreshed_proposal = *pending_proposal;
                refreshed_proposal.expected_topology_version =
                    supervisor.topologyVersion();
                refreshed_proposal.expected_estimator_version =
                    estimator.version();
                const TransitionCertificate refreshed_certificate =
                    TransitionCertifier{}.certify(
                        refreshed_proposal, refreshed_context, true);
                if (!supervisor.finishMakeBeforeBreak(
                        refreshed_certificate, supervisor.topologyVersion(),
                        estimator.version(), 0.1 * (step + 1))) {
                    ++summary.hard_gate_violations;
                    break;
                }
                topology = refreshed_certificate.successor_edges;
                pending_proposal.reset();
                ++summary.successful_reconfigurations;
            }

            if (!pending_proposal.has_value() &&
                attempted_reconfigurations < requested_reconfigurations) {
                ++attempted_reconfigurations;
                const DirectedEdge new_edge =
                    transition_certifier_detail::contains(
                        topology, DirectedEdge{10, 2})
                    ? DirectedEdge{11, 2}
                    : DirectedEdge{10, 2};
                const EligibilityThresholds thresholds{
                    849.0, 850.0, 1.0, 0.1, 0.5, 2.0};
                std::map<std::string, RangeLinkState> links;
                for (const DirectedEdge& edge : candidates) {
                    links[UndirectedEdge::canonical(
                        edge.reference, edge.owner).id()] = {0.0, 1.0};
                }
                const auto keep_snapshot = buildEligibility(
                    estimate, links, thresholds, topology);
                const auto add_snapshot = buildEligibility(
                    estimate, links, thresholds, {});
                std::set<std::string> allowed;
                for (const DirectedEdge& edge : candidates) allowed.insert(edge.id());
                std::vector<DirectedEdge> eligible;
                for (const DirectedEdge& edge : keep_snapshot.eligibleEdges())
                    if (allowed.count(edge.id())) eligible.push_back(edge);
                for (const DirectedEdge& edge : add_snapshot.eligibleEdges())
                    if (allowed.count(edge.id()) &&
                        !grand_finale_experiment_detail::edgeDifference(
                            {edge}, eligible).empty()) eligible.push_back(edge);
                eligible = transition_certifier_detail::canonicalEdges(eligible);
                TopologyRequest request{
                    mobiles, fixed_ids, eligible, topology, 2, 2,
                    {{new_edge.id(), 10.0}}, {},
                    {{"1->2|" + new_edge.id(), 2.0}}};
                const TopologySolution target = solver(profile)->solve(
                    TopologyModel(request));
                if (target.status != TopologySolveStatus::Optimal) {
                    ++summary.hard_gate_violations;
                    supervisor.requestReformation(
                        0.1 * (step + 1), false, true);
                    break;
                }
                const auto additions = edgeDifference(target.edges, topology);
                const auto removals = edgeDifference(topology, target.edges);
                if (additions.size() != 1 || removals.size() != 1) {
                    ++summary.hard_gate_violations;
                    supervisor.requestReformation(
                        0.1 * (step + 1), false, true);
                    break;
                }

                TransitionCertificationContext certificate_context;
                certificate_context.topology_version =
                    supervisor.topologyVersion();
                certificate_context.estimator_version = estimator.version();
                certificate_context.mobile_ids = mobiles;
                certificate_context.fixed_ids = fixed_ids;
                certificate_context.r_max = 2;
                certificate_context.max_reference_distance_m = 850.0;
                certificate_context.min_fim_eigenvalue = 1e-6;
                certificate_context.max_posterior_eigenvalue_m2 = 0.1;
                certificate_context.gamma_accept = 0.05;
                certificate_context.estimate = estimate;
                for (const DirectedEdge& edge : candidates) {
                    const std::string range_id = UndirectedEdge::canonical(
                        edge.reference, edge.owner).id();
                    certificate_context.range_variances_m2[range_id] = 1.0;
                    const double distance =
                        (detail::nodePosition(estimate, edge.owner) -
                         detail::nodePosition(estimate, edge.reference)).norm();
                    certificate_context.edge_gates[edge.id()] =
                        {true, true, distance + 0.02};
                }
                if (inject_uncertifiable_candidate) {
                    certificate_context.edge_gates.at(additions.front().id())
                        .robust_distance_m = 850.01;
                }
                certificate_context.hard_row_request = hardRowRequest(
                    mobiles, fixed_ids, estimate, topology);
                const TransitionProposal proposal{
                    topology, additions.front(), removals.front(),
                    supervisor.topologyVersion(),
                    estimator.version()};
                const TransitionCertificate certificate =
                    TransitionCertifier{}.certify(
                        proposal, certificate_context, true);
                supervisor.requestReformation(
                    0.1 * (step + 1), certificate.valid,
                    certificate.reverse_valid);
                if (!certificate.valid) {
                    summary.final_mode = supervisor.mode();
                    break;
                }
                if (!supervisor.beginMakeBeforeBreak(
                        certificate, supervisor.topologyVersion(),
                        estimator.version(), 0.1 * (step + 1))) {
                    ++summary.hard_gate_violations;
                    break;
                }
                summary.visited_union_state = true;
                updateHardSummary(
                    summary, mobiles, certificate.union_edges, estimate,
                    hardRowRequest(
                        mobiles, fixed_ids, estimate,
                        certificate.union_edges), acceleration);
                topology = certificate.union_edges;
                pending_proposal = proposal;
            }
        }
        summary.certified_coverage_final = coverage.certifiedFraction();
        summary.reached_certified_t100 = coverage.reachedCertifiedT100();
        summary.false_covered_cells = static_cast<std::size_t>(
            coverage.falseCertifiedCount());
        summary.final_mode = supervisor.mode();
        summary.topology_version = supervisor.topologyVersion();
        return summary;
    }

    static GrandFinaleSummary runScaleSmoke14p3(
        std::uint32_t seed,
        std::size_t steps) {
        if (seed != 2027 || steps == 0)
            throw std::invalid_argument("invalid scale-smoke parameters");
        using namespace grand_finale_experiment_detail;
        TruthStateMap truth;
        for (NodeId id = 1; id <= 14; ++id) {
            truth[id] = Eigen::Vector4d(
                1.0 + static_cast<double>(id),
                1.0 + static_cast<double>(id % 3), 0.0, 0.0);
        }
        const std::map<NodeId, Eigen::Vector2d> fixed = {
            {100, {0.0, 0.0}}, {101, {0.0, 5.0}}, {102, {0.0, 10.0}}};
        InterimMasterDekf estimator(estimates(truth), fixed);
        GrandFinaleSummary summary;
        summary.mobile_count = 14;
        summary.fixed_count = 3;
        for (std::size_t step = 0; step < steps; ++step) {
            for (NodeId id = 1; id <= 14; ++id) {
                estimator.propagateLocal(
                    id, Eigen::Vector2d::Zero(), 0.1, 0.0);
                ++summary.second_order_control_steps;
            }
            std::vector<DirectedEdge> ranging;
            for (NodeId id = 1; id <= 14; ++id)
                ranging.push_back(DirectedEdge{100, id});
            for (const RangeMeasurement& measurement : rangeBatch(
                     static_cast<std::int64_t>(step + 1) * 100000000LL,
                     ranging, truth, fixed)) {
                estimator.applyUpdate(
                    estimator.makeUpdate(
                        truth.count(measurement.edge.first)
                            ? measurement.edge.first
                            : measurement.edge.second,
                        measurement));
                ++summary.estimator_updates;
            }
        }
        summary.hard_gate_violations = 0;
        summary.topology_version = 0;
        return summary;
    }
};

}  // namespace gf
