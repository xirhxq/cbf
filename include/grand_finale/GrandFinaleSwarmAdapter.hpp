#pragma once

#include "Swarm.hpp"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/CertifiedCoverageTracker.hpp"
#include "grand_finale/InterimMasterDekf.hpp"
#include "grand_finale/GurobiTopologySolver.hpp"
#include "grand_finale/HighsTopologySolver.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

namespace gf {

inline bool referenceEdgeInitialSetAudited(
    const std::vector<CanonicalHardRow>& rows,
    const DirectedEdge& edge,
    const std::vector<NodeId>& mobile_ids,
    double tolerance = 1.0e-12) {
    if (!std::isfinite(tolerance) || tolerance < 0.0)
        throw std::invalid_argument("invalid initial-set audit tolerance");
    std::vector<NodeId> required_owners{edge.owner};
    if (std::find(mobile_ids.begin(), mobile_ids.end(), edge.reference) !=
        mobile_ids.end()) {
        required_owners.push_back(edge.reference);
    }
    for (NodeId owner : required_owners) {
        const std::string id =
            "reference:" + edge.id() + ":owner:" +
            std::to_string(owner);
        const auto row = std::find_if(
            rows.begin(), rows.end(), [&](const CanonicalHardRow& candidate) {
                return candidate.id == id &&
                       candidate.kind ==
                           CanonicalHardRowKind::ReferenceDistance;
            });
        if (row == rows.end() || !std::isfinite(row->barrier_h) ||
            !std::isfinite(row->barrier_psi1) ||
            row->barrier_h < -tolerance ||
            row->barrier_psi1 < -tolerance) {
            return false;
        }
    }
    return true;
}

inline double posteriorPairPositionTube(
    const JointEstimateSnapshot& snapshot,
    const DirectedEdge& edge,
    double uncertainty_sigma) {
    if (!std::isfinite(uncertainty_sigma) || uncertainty_sigma < 0.0)
        throw std::invalid_argument("uncertainty sigma must be non-negative");
    const double owner = std::sqrt(std::max(
        0.0, detail::maximumPositionEigenvalue(snapshot, edge.owner)));
    const double reference = std::sqrt(std::max(
        0.0, detail::maximumPositionEigenvalue(snapshot, edge.reference)));
    return uncertainty_sigma * (owner + reference);
}

struct GrandFinaleSwarmAdapterConfig {
    SolverProfile solver_profile = SolverProfile::OpenSource;
    double dt_s = 0.1;
    double minimum_dwell_s = 0.1;
    double acceleration_half_box = 0.4;
    double position_gain = 0.4;
    double velocity_gain = 0.8;
    double estimator_acceleration_variance = 0.0;
    double initial_position_variance_m2 = 1.0e-4;
    double initial_velocity_variance_m2 = 1.0e-4;
    double certified_error_bound_m = 0.05;
    double certified_shadow_single_position_support_m = 0.0;
    double certified_shadow_relative_position_support_m = 0.0;
    double certified_shadow_relative_velocity_support_mps = 0.0;
    double maximum_accepted_range_innovation_m = 1.0;
    unsigned int range_random_seed = 2027;
    double range_noise_std_m = 0.0;
    double range_dropout_probability = 0.0;
    double sensor_radius_m = 1.6;
    double reference_distance_m = 850.0;
    double add_reference_distance_m = 849.0;
    double reference_uncertainty_m = 0.02;
    double uncertainty_sigma = 3.0;
    double maximum_reference_position_eigenvalue_m2 = 0.1;
    double maximum_posterior_eigenvalue_m2 = 0.1;
    double maximum_range_aoi_s = 1.0;
    double minimum_range_quality = 0.5;
    double collision_distance_m = 0.1;
    double residual_tolerance = 1.0e-7;
    double qp_oracle_tolerance = 1.0e-5;
    ProgressCompatibilityConfig progress_compatibility{
        0.8, 0.0, 1.0e-9, true};
};

struct GrandFinaleSwarmStep {
    bool advanced = false;
    std::string reason;
    SupervisorMode mode = SupervisorMode::Search;
    std::uint64_t estimator_version_before = 0;
    std::uint64_t estimator_version_after = 0;
    std::uint64_t topology_version = 0;
    double minimum_hard_residual =
        std::numeric_limits<double>::infinity();
    std::size_t certified_control_count = 0;
    std::size_t updated_truth_cells = 0;
    double truth_coverage = 0.0;
    double certified_coverage = 0.0;
    double qp_wall_s = 0.0;
    std::map<NodeId, Eigen::Vector2d> applied_controls;
};

struct GrandFinaleProposalResult {
    bool transition_started = false;
    std::size_t no_good_rejections = 0;
    std::string reason;
    std::vector<DirectedEdge> selected_topology;
    std::vector<std::string> rejection_reasons;
};

struct CurrentReferenceAudit {
    std::size_t minimum_effective_reference_count = 0;
    double minimum_fim_eigenvalue =
        std::numeric_limits<double>::infinity();
    double maximum_posterior_eigenvalue = 0.0;
};

enum class FreshnessRelation {
    NoPending,
    PendingCurrent,
    UnionRequiresFreshBreak
};

struct RuntimeRangeLinkState {
    double age_s = 0.0;
    double quality = 0.0;
    double variance_m2 = 0.0;
};

struct GrandFinaleRuntimeSnapshot {
    double runtime_s = 0.0;
    JointEstimateSnapshot estimate;
    DekfInternalAudit dekf;
    std::vector<DirectedEdge> topology;
    SupervisorMode mode = SupervisorMode::Search;
    std::uint64_t estimator_token = 0;
    std::uint64_t topology_token = 0;
    FreshnessRelation freshness = FreshnessRelation::NoPending;
    double timer_since_supervisor_transition_s = 0.0;
    bool supervisor_transition_pending = false;
    bool adapter_transition_pending = false;
    bool pending_is_retreat = false;
    std::size_t transition_stack_size = 0;
    std::size_t union_control_cycles = 0;
    std::map<std::string, RuntimeRangeLinkState> range_links;
};

class GrandFinaleSwarmAdapter {
public:
    GrandFinaleSwarmAdapter(
        Swarm& swarm,
        std::vector<NodeId> mobile_ids,
        std::map<NodeId, Eigen::Vector2d> fixed_positions,
        std::vector<DirectedEdge> initial_topology,
        GrandFinaleSwarmAdapterConfig config)
        : swarm_(swarm),
          mobile_ids_(std::move(mobile_ids)),
          fixed_positions_(std::move(fixed_positions)),
          config_(config),
          estimator_(initialEstimates(), fixed_positions_),
          coverage_(
              swarm.gridWorldGroundTruth.xLim,
              swarm.gridWorldGroundTruth.xNum,
              swarm.gridWorldGroundTruth.yLim,
              swarm.gridWorldGroundTruth.yNum),
          supervisor_({config_.minimum_dwell_s, 0.05, 0.1}),
          range_rng_(config_.range_random_seed) {
        if (mobile_ids_.size() != swarm_.robots.size())
            throw std::invalid_argument("mobile ids must match Swarm robots");
        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            if (mobile_ids_[index] != swarm_.robots[index]->id) {
                throw std::invalid_argument(
                    "mobile id order must match Swarm robot order");
            }
        }
        if (!std::isfinite(config_.dt_s) || config_.dt_s <= 0.0)
            throw std::invalid_argument("adapter dt must be positive");
        if (!std::isfinite(config_.minimum_dwell_s) ||
            config_.minimum_dwell_s < config_.dt_s) {
            throw std::invalid_argument(
                "supervisor dwell must span at least one control period");
        }
        if (!std::isfinite(config_.acceleration_half_box) ||
            config_.acceleration_half_box <= 0.0 ||
            !std::isfinite(config_.residual_tolerance) ||
            config_.residual_tolerance < 0.0 ||
            !std::isfinite(config_.qp_oracle_tolerance) ||
            config_.qp_oracle_tolerance < 0.0 ||
            !std::isfinite(config_.uncertainty_sigma) ||
            config_.uncertainty_sigma < 0.0 ||
            !std::isfinite(config_.add_reference_distance_m) ||
            config_.add_reference_distance_m <= 0.0 ||
            !std::isfinite(config_.reference_distance_m) ||
            config_.reference_distance_m < config_.add_reference_distance_m ||
            !std::isfinite(config_.initial_position_variance_m2) ||
            config_.initial_position_variance_m2 < 0.0 ||
            !std::isfinite(config_.initial_velocity_variance_m2) ||
            config_.initial_velocity_variance_m2 < 0.0 ||
            !std::isfinite(config_.maximum_range_aoi_s) ||
            config_.maximum_range_aoi_s < 0.0 ||
            !std::isfinite(
                config_.maximum_reference_position_eigenvalue_m2) ||
            config_.maximum_reference_position_eigenvalue_m2 < 0.0 ||
            !std::isfinite(config_.maximum_posterior_eigenvalue_m2) ||
            config_.maximum_posterior_eigenvalue_m2 < 0.0 ||
            !std::isfinite(config_.minimum_range_quality) ||
            config_.minimum_range_quality < 0.0 ||
            !std::isfinite(config_.estimator_acceleration_variance) ||
            config_.estimator_acceleration_variance < 0.0 ||
            !std::isfinite(config_.sensor_radius_m) ||
            config_.sensor_radius_m <= 0.0 ||
            !std::isfinite(config_.certified_error_bound_m) ||
            config_.certified_error_bound_m < 0.0 ||
            !std::isfinite(
                config_.certified_shadow_single_position_support_m) ||
            config_.certified_shadow_single_position_support_m < 0.0 ||
            !std::isfinite(
                config_.certified_shadow_relative_position_support_m) ||
            config_.certified_shadow_relative_position_support_m < 0.0 ||
            !std::isfinite(
                config_.certified_shadow_relative_velocity_support_mps) ||
            config_.certified_shadow_relative_velocity_support_mps < 0.0 ||
            !std::isfinite(
                config_.maximum_accepted_range_innovation_m) ||
            config_.maximum_accepted_range_innovation_m <= 0.0 ||
            !std::isfinite(config_.range_noise_std_m) ||
            config_.range_noise_std_m < 0.0 ||
            !std::isfinite(config_.range_dropout_probability) ||
            config_.range_dropout_probability < 0.0 ||
            config_.range_dropout_probability > 1.0 ||
            !std::isfinite(config_.position_gain) ||
            config_.position_gain < 0.0 ||
            !std::isfinite(config_.velocity_gain) ||
            config_.velocity_gain < 0.0) {
            throw std::invalid_argument("invalid GrandFinale safety configuration");
        }
        supervisor_.initializeTopology(std::move(initial_topology), 1);
    }

    GrandFinaleSwarmStep step() {
        GrandFinaleSwarmStep metrics;
        metrics.mode = supervisor_.mode();
        metrics.topology_version = supervisor_.topologyVersion();
        metrics.estimator_version_before = estimator_.version();
        swarm_.prepareCertifiedControlStep();
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        const std::uint64_t snapshot_version = estimator_.version();
        const auto nominal = nominal_override_.has_value()
            ? *nominal_override_
            : nominalControls(metrics.mode);
        std::vector<CanonicalHardRow> rows;
        try {
            rows = buildCanonicalHardRows(
                hardRowRequest(snapshot, supervisor_.topology()));
        } catch (const std::exception&) {
            metrics.reason = "snapshot_robust_hard_row_invalid";
            return metrics;
        }
        std::map<int, Eigen::Vector2d> applied;

        const auto build_controls = [&]()
            -> std::optional<Swarm::CertifiedControlBatch> {
                for (NodeId owner : mobile_ids_) {
                    const auto qp_start = std::chrono::steady_clock::now();
                    const CanonicalQpResult result = controller_.solve(
                        CanonicalQpRequest{
                            config_.solver_profile, owner,
                            snapshot_version, supervisor_.topologyVersion(),
                            supervisor_.mode(), nominal.at(owner),
                            config_.acceleration_half_box, rows,
                            config_.residual_tolerance});
                    metrics.qp_wall_s += std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - qp_start).count();
                    if (!controlMayBeApplied(
                            result, snapshot_version,
                            supervisor_.topologyVersion(), supervisor_.mode())) {
                        metrics.reason = result.failure_reason;
                        return std::nullopt;
                    }
                    if (result.exact_oracle_error >
                        config_.qp_oracle_tolerance) {
                        metrics.reason = "exact_qp_oracle_mismatch";
                        return std::nullopt;
                    }
                    applied[static_cast<int>(owner)] = result.control;
                    metrics.minimum_hard_residual = std::min(
                        metrics.minimum_hard_residual,
                        result.minimum_hard_residual);
                }
                return applied;
            };
        const auto physical = swarm_.applyCertifiedControlsAndAdvance(
            config_.dt_s, build_controls());
        metrics.advanced = physical.advanced;
        metrics.updated_truth_cells = physical.updated_truth_cells;
        if (!physical.advanced) {
            if (metrics.reason.empty()) metrics.reason = physical.reason;
            return metrics;
        }

        for (NodeId owner : mobile_ids_) {
            estimator_.propagateLocal(
                owner, applied.at(static_cast<int>(owner)), config_.dt_s,
                config_.estimator_acceleration_variance);
        }
        applyDeterministicRangeBatch();
        const JointEstimateSnapshot after = estimator_.reconstructForAudit();
        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            const Point truth = swarm_.robots.at(index)->model->xy();
            const Eigen::Vector2d estimate = after.mean.segment<2>(4 * index);
            coverage_.observe(
                truth, Point(estimate.x(), estimate.y()),
                std::max(
                    config_.certified_shadow_single_position_support_m,
                    std::max(
                        config_.certified_error_bound_m,
                        config_.uncertainty_sigma * std::sqrt(std::max(
                            0.0, detail::maximumPositionEigenvalue(
                                after, mobile_ids_[index]))))),
                config_.sensor_radius_m);
        }
        metrics.certified_control_count = applied.size();
        for (const auto& [id, control] : applied)
            metrics.applied_controls[static_cast<NodeId>(id)] = control;
        metrics.estimator_version_after = estimator_.version();
        metrics.truth_coverage = coverage_.truthFraction();
        metrics.certified_coverage = coverage_.certifiedFraction();
        metrics.reason = "advanced";
        if (metrics.mode == SupervisorMode::Union &&
            pending_proposal_.has_value()) {
            ++union_control_cycles_;
        }
        return metrics;
    }

    GrandFinaleSwarmStep stepWithNominal(
        const std::map<NodeId, Eigen::Vector2d>& nominal) {
        GrandFinaleSwarmStep rejected;
        rejected.mode = supervisor_.mode();
        rejected.topology_version = supervisor_.topologyVersion();
        rejected.estimator_version_before = estimator_.version();
        if (nominal.size() != mobile_ids_.size()) {
            rejected.reason = "primitive_nominal_incomplete";
            return rejected;
        }
        for (const auto& [id, control] : nominal) {
            if (std::find(mobile_ids_.begin(), mobile_ids_.end(), id) ==
                    mobile_ids_.end() ||
                !control.allFinite() ||
                control.cwiseAbs().maxCoeff() >
                    config_.acceleration_half_box + 1.0e-12) {
                rejected.reason = "primitive_nominal_invalid";
                return rejected;
            }
        }
        nominal_override_ = nominal;
        try {
            GrandFinaleSwarmStep result = step();
            nominal_override_.reset();
            return result;
        } catch (...) {
            nominal_override_.reset();
            throw;
        }
    }

    bool beginReplacement(
        const DirectedEdge& new_edge,
        const DirectedEdge& old_edge,
        bool update_supervisor_on_rejection = true) {
        if (pending_proposal_.has_value()) return false;
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        TransitionProposal proposal{
            supervisor_.topology(), new_edge, old_edge,
            supervisor_.topologyVersion(), estimator_.version()};
        const TransitionCertificate certificate = TransitionCertifier{}.certify(
            proposal, certificationContext(snapshot, {new_edge}), true);
        last_certification_reason_ = certificate.reason;
        if (!certificate.old_state.valid)
            last_certification_reason_ += ":old:" + certificate.old_state.reason;
        else if (!certificate.union_state.valid)
            last_certification_reason_ += ":union:" + certificate.union_state.reason;
        else if (!certificate.successor_state.valid)
            last_certification_reason_ +=
                ":successor:" + certificate.successor_state.reason;
        if (!certificate.valid) {
            if (update_supervisor_on_rejection) {
                supervisor_.requestReformation(
                    swarm_.robots.front()->runtime, false,
                    certificate.reverse_valid);
            }
            return false;
        }
        supervisor_.requestReformation(
            swarm_.robots.front()->runtime, true,
            certificate.reverse_valid);
        if (!supervisor_.beginMakeBeforeBreak(
                certificate, supervisor_.topologyVersion(),
                estimator_.version(), swarm_.robots.front()->runtime)) {
            last_certification_reason_ =
                "supervisor_begin_rejected:mode=" +
                std::to_string(static_cast<int>(supervisor_.mode())) +
                ":cert_topology=" +
                std::to_string(certificate.topology_version) +
                ":current_topology=" +
                std::to_string(supervisor_.topologyVersion()) +
                ":cert_estimator=" +
                std::to_string(certificate.estimator_version) +
                ":current_estimator=" +
                std::to_string(estimator_.version());
            return false;
        }
        pending_proposal_ = proposal;
        pending_certificate_ = certificate;
        union_control_cycles_ = 0;
        return true;
    }

    GrandFinaleProposalResult proposeAndBegin(TopologyRequest request) {
        GrandFinaleProposalResult result;
        request.old_edges = supervisor_.topology();
        const std::size_t attempt_limit =
            std::size_t{1} << std::min<std::size_t>(
                request.eligible_edges.size(), 20);
        for (std::size_t attempt = 0; attempt < attempt_limit; ++attempt) {
            std::unique_ptr<TopologySolver> solver;
            if (config_.solver_profile == SolverProfile::Gurobi) {
#ifdef ENABLE_GUROBI
                solver = std::make_unique<GurobiTopologySolver>();
#else
                result.reason = "gurobi_unavailable";
                return result;
#endif
            } else {
                solver = std::make_unique<HighsTopologySolver>();
            }
            const TopologySolution proposal = solver->solve(TopologyModel(request));
            if (proposal.status != TopologySolveStatus::Optimal) {
                if (result.no_good_rejections == 0) {
                    result.reason = proposal.status == TopologySolveStatus::Infeasible
                        ? "candidate_exhausted:" + proposal.detail
                        : "topology_solver_failure:" + proposal.detail;
                }
                supervisor_.requestReformation(
                    swarm_.robots.front()->runtime, false,
                    !transition_stack_.empty());
                return result;
            }
            result.selected_topology = proposal.edges;
            const auto additions = edgeDifference(
                proposal.edges, supervisor_.topology());
            const auto removals = edgeDifference(
                supervisor_.topology(), proposal.edges);
            if (additions.size() == 1 && removals.size() == 1 &&
                beginReplacement(
                    additions.front(), removals.front(), false)) {
                result.transition_started = true;
                result.reason = "certified";
                return result;
            }
            if (additions.size() != 1 || removals.size() != 1) {
                last_certification_reason_ =
                    "not_single_replacement:additions=" +
                    std::to_string(additions.size()) +
                    ":removals=" + std::to_string(removals.size());
            }
            request.forbidden_topologies.push_back(proposal.edges);
            ++result.no_good_rejections;
            result.reason = "certifier_rejected:" + last_certification_reason_;
            result.rejection_reasons.push_back(last_certification_reason_);
        }
        if (result.reason.empty()) result.reason = "candidate_exhausted";
        supervisor_.requestReformation(
            swarm_.robots.front()->runtime, false,
            !transition_stack_.empty());
        return result;
    }

    bool finishReplacementAfterFreshCycle() {
        if (!pending_proposal_.has_value() || union_control_cycles_ < 1 ||
            supervisor_.mode() != SupervisorMode::Union) {
            return false;
        }
        TransitionProposal refreshed = *pending_proposal_;
        refreshed.expected_topology_version = supervisor_.topologyVersion();
        refreshed.expected_estimator_version = estimator_.version();
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        const TransitionCertificate certificate = TransitionCertifier{}.certify(
            refreshed, certificationContext(snapshot, {refreshed.new_edge}),
            true);
        last_certification_reason_ = certificate.reason;
        if (!supervisor_.finishMakeBeforeBreak(
                certificate, supervisor_.topologyVersion(),
                estimator_.version(), swarm_.robots.front()->runtime)) {
            return false;
        }
        if (pending_is_retreat_) {
            if (!transition_stack_.empty()) transition_stack_.pop_back();
        } else {
            transition_stack_.push_back(*pending_certificate_);
        }
        pending_proposal_.reset();
        pending_certificate_.reset();
        union_control_cycles_ = 0;
        pending_is_retreat_ = false;
        return true;
    }

    bool requestCertifiedRetreat() {
        const bool available = !transition_stack_.empty() &&
            !pending_proposal_.has_value();
        supervisor_.requestRetreat(
            swarm_.robots.front()->runtime, available);
        return available;
    }

    bool beginRetreatReplacement() {
        if (supervisor_.mode() != SupervisorMode::Retreat ||
            transition_stack_.empty() || pending_proposal_.has_value()) {
            return false;
        }
        const TransitionCertificate& previous = transition_stack_.back();
        if (!previous.new_edge.has_value() || !previous.old_edge.has_value())
            return false;
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        TransitionProposal reverse{
            supervisor_.topology(), *previous.old_edge, *previous.new_edge,
            supervisor_.topologyVersion(), estimator_.version()};
        const TransitionCertificate certificate = TransitionCertifier{}.certify(
            reverse, certificationContext(snapshot, {reverse.new_edge}), true);
        if (!certificate.valid ||
            !supervisor_.beginMakeBeforeBreak(
                certificate, supervisor_.topologyVersion(),
                estimator_.version(), swarm_.robots.front()->runtime)) {
            return false;
        }
        pending_proposal_ = reverse;
        pending_certificate_ = certificate;
        pending_is_retreat_ = true;
        union_control_cycles_ = 0;
        return true;
    }

    std::size_t unionControlCycles() const { return union_control_cycles_; }
    std::size_t transitionStackSize() const { return transition_stack_.size(); }

    HybridSupervisor& supervisor() { return supervisor_; }
    const HybridSupervisor& supervisor() const { return supervisor_; }
    InterimMasterDekf& estimator() { return estimator_; }
    const InterimMasterDekf& estimator() const { return estimator_; }
    const CertifiedCoverageTracker& coverage() const { return coverage_; }
    const GrandFinaleSwarmAdapterConfig& config() const { return config_; }
    std::map<NodeId, Eigen::Vector2d> currentNominalControls() {
        return nominalControls(supervisor_.mode());
    }
    const std::string& lastCertificationReason() const {
        return last_certification_reason_;
    }
    GrandFinaleRuntimeSnapshot runtimeSnapshot() const {
        GrandFinaleRuntimeSnapshot snapshot;
        snapshot.runtime_s = swarm_.robots.front()->runtime;
        snapshot.estimate = estimator_.reconstructForAudit();
        snapshot.dekf = estimator_.internalAudit();
        snapshot.topology = supervisor_.topology();
        snapshot.mode = supervisor_.mode();
        snapshot.estimator_token = estimator_.version();
        snapshot.topology_token = supervisor_.topologyVersion();
        snapshot.timer_since_supervisor_transition_s = std::max(
            0.0, snapshot.runtime_s - supervisor_.lastTransitionS());
        snapshot.supervisor_transition_pending =
            supervisor_.transitionPending();
        snapshot.adapter_transition_pending = pending_proposal_.has_value();
        snapshot.pending_is_retreat = pending_is_retreat_;
        snapshot.transition_stack_size = transition_stack_.size();
        snapshot.union_control_cycles = union_control_cycles_;
        if (pending_proposal_.has_value()) {
            snapshot.freshness = supervisor_.mode() == SupervisorMode::Union
                ? FreshnessRelation::UnionRequiresFreshBreak
                : FreshnessRelation::PendingCurrent;
        }
        for (const auto& [id, observed_s] : range_last_observation_s_) {
            const auto quality = range_quality_.find(id);
            const auto variance = range_variance_m2_.find(id);
            if (quality == range_quality_.end() ||
                variance == range_variance_m2_.end())
                continue;
            snapshot.range_links[id] = {
                std::max(0.0, snapshot.runtime_s - observed_s),
                quality->second, variance->second};
        }
        return snapshot;
    }
    std::vector<CanonicalHardRow> currentSnapshotHardRows(
        const std::vector<DirectedEdge>& topology) const {
        return buildCanonicalHardRows(hardRowRequest(
            estimator_.reconstructForAudit(), topology));
    }
    CurrentReferenceAudit currentReferenceAudit() {
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        const auto topology = supervisor_.topology();
        const TransitionCertificationContext context =
            certificationContext(snapshot, {});
        const auto hard_rows = buildCanonicalHardRows(
            hardRowRequest(snapshot, topology));
        CurrentReferenceAudit audit;
        audit.minimum_effective_reference_count =
            std::numeric_limits<std::size_t>::max();
        for (NodeId owner : mobile_ids_) {
            std::vector<DirectedEdge> effective_edges;
            for (const DirectedEdge& edge : topology) {
                if (edge.owner != owner) continue;
                const auto gate = context.edge_gates.find(edge.id());
                const bool hocbf_audited = referenceEdgeInitialSetAudited(
                    hard_rows, edge, mobile_ids_);
                if (gate != context.edge_gates.end() &&
                    gate->second.keep_valid && hocbf_audited) {
                    effective_edges.push_back(edge);
                }
            }
            audit.minimum_effective_reference_count = std::min(
                audit.minimum_effective_reference_count,
                effective_edges.size());
            const double fim = effective_edges.size() < 2
                ? -std::numeric_limits<double>::infinity()
                : Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    referenceFim(
                        owner, effective_edges, snapshot,
                        context.range_variances_m2))
                    .eigenvalues().minCoeff();
            audit.minimum_fim_eigenvalue = std::min(
                audit.minimum_fim_eigenvalue, fim);
            const double posterior =
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    marginalPositionCovariance(snapshot, owner))
                    .eigenvalues().maxCoeff();
            audit.maximum_posterior_eigenvalue = std::max(
                audit.maximum_posterior_eigenvalue, posterior);
        }
        return audit;
    }

private:
    std::vector<MobileEstimate> initialEstimates() const {
        std::vector<MobileEstimate> estimates;
        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            const auto& robot = swarm_.robots.at(index);
            Eigen::Vector4d mean;
            mean << robot->model->xy().x, robot->model->xy().y,
                    robot->model->getVelocity().x(),
                    robot->model->getVelocity().y();
            estimates.push_back(MobileEstimate{
                mobile_ids_[index], mean,
                (Eigen::Vector4d(
                    config_.initial_position_variance_m2,
                    config_.initial_position_variance_m2,
                    config_.initial_velocity_variance_m2,
                    config_.initial_velocity_variance_m2)).asDiagonal()});
        }
        return estimates;
    }

    std::map<NodeId, Eigen::Vector2d> nominalControls(
        SupervisorMode mode) {
        std::map<NodeId, Eigen::Vector2d> result;
        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            const auto& robot = swarm_.robots.at(index);
            if (mode == SupervisorMode::Hold ||
                mode == SupervisorMode::Retreat) {
                Eigen::Vector2d braking =
                    -config_.velocity_gain * robot->model->getVelocity().head<2>();
                braking.x() = std::max(-config_.acceleration_half_box,
                    std::min(config_.acceleration_half_box, braking.x()));
                braking.y() = std::max(-config_.acceleration_half_box,
                    std::min(config_.acceleration_half_box, braking.y()));
                result[mobile_ids_[index]] = braking;
            } else {
                result[mobile_ids_[index]] = robot->coverageNominalAcceleration(
                    config_.position_gain, config_.velocity_gain,
                    config_.acceleration_half_box);
            }
        }
        return result;
    }

    CanonicalHardRowRequest hardRowRequest(
        const JointEstimateSnapshot& snapshot,
        const std::vector<DirectedEdge>& topology) const {
        CanonicalHardRowRequest request;
        request.mobile_ids = mobile_ids_;
        for (const auto& [id, point] : fixed_positions_) {
            request.fixed_ids.push_back(id);
            request.states[id] = {Point(point.x(), point.y()),
                                  Eigen::Vector2d::Zero(),
                                  Eigen::Vector2d::Zero()};
        }
        request.reference_edges = topology;
        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            const Eigen::Vector4d state = snapshot.mean.segment<4>(4 * index);
            request.states[mobile_ids_[index]] = {
                Point(state.x(), state.y()), state.tail<2>(),
                Eigen::Vector2d::Zero()};
            for (std::size_t second = index + 1;
                 second < mobile_ids_.size(); ++second) {
                request.collision_pairs.push_back(UndirectedEdge::canonical(
                    mobile_ids_[index], mobile_ids_[second]));
            }
            for (const auto& [fixed, point] : fixed_positions_) {
                (void)point;
                request.collision_pairs.push_back(UndirectedEdge::canonical(
                    mobile_ids_[index], fixed));
            }
        }
        request.reference_spec = {
            PairwiseSecondOrderBarrierKind::CommunicationUpper,
            config_.reference_distance_m,
            config_.reference_uncertainty_m,
            1.0, 1.0, 1.0, 0.0};
        request.collision_spec = {
            PairwiseSecondOrderBarrierKind::CollisionLower,
            config_.collision_distance_m,
            0.0,
            1.0, 1.0, 1.0, 0.0};
        request.acceleration_half_box = config_.acceleration_half_box;
        request.require_snapshot_robust_rows = true;
        for (const DirectedEdge& edge : request.reference_edges) {
            request.reference_snapshot_tubes.emplace(
                edge.id(), snapshotPairTube(snapshot, edge.owner, edge.reference));
        }
        for (const UndirectedEdge& edge : request.collision_pairs) {
            request.collision_snapshot_tubes.emplace(
                edge.id(), snapshotPairTube(snapshot, edge.first, edge.second));
        }
        return request;
    }

    TransitionCertificationContext certificationContext(
        const JointEstimateSnapshot& snapshot,
        const std::vector<DirectedEdge>& additions) {
        TransitionCertificationContext context;
        context.topology_version = supervisor_.topologyVersion();
        context.estimator_version = estimator_.version();
        context.mobile_ids = mobile_ids_;
        for (const auto& [id, point] : fixed_positions_) {
            (void)point;
            context.fixed_ids.push_back(id);
        }
        context.r_max = 2;
        context.max_reference_distance_m = config_.reference_distance_m;
        context.min_fim_eigenvalue = 1.0e-6;
        context.max_posterior_eigenvalue_m2 =
            config_.maximum_posterior_eigenvalue_m2;
        context.gamma_accept = 0.05;
        context.estimate = snapshot;
        std::vector<DirectedEdge> edges = supervisor_.topology();
        edges.insert(edges.end(), additions.begin(), additions.end());
        edges = transition_certifier_detail::canonicalEdges(std::move(edges));
        std::map<std::string, RangeLinkState> links;
        const double now_s = swarm_.robots.front()->runtime;
        for (const auto& [id, observed_s] : range_last_observation_s_) {
            links[id] = {
                std::max(0.0, now_s - observed_s),
                range_quality_.at(id)};
        }
        const EligibilityThresholds thresholds{
            config_.add_reference_distance_m,
            config_.reference_distance_m,
            config_.maximum_range_aoi_s,
            config_.maximum_reference_position_eigenvalue_m2,
            config_.minimum_range_quality,
            config_.uncertainty_sigma};
        const EligibilitySnapshot keep_eligibility = buildEligibility(
            snapshot, links, thresholds, supervisor_.topology());
        const EligibilitySnapshot add_eligibility = buildEligibility(
            snapshot, links, thresholds, {});
        std::map<std::string, ReferenceCandidate> keep_candidates;
        std::map<std::string, ReferenceCandidate> add_candidates;
        for (const ReferenceCandidate& candidate : keep_eligibility.candidates)
            keep_candidates.emplace(candidate.edge.id(), candidate);
        for (const ReferenceCandidate& candidate : add_eligibility.candidates)
            add_candidates.emplace(candidate.edge.id(), candidate);
        for (const DirectedEdge& edge : edges) {
            const auto keep_candidate = keep_candidates.find(edge.id());
            const auto add_candidate = add_candidates.find(edge.id());
            const bool keep_valid = keep_candidate != keep_candidates.end() &&
                                    keep_candidate->second.eligible;
            const bool add_valid = add_candidate != add_candidates.end() &&
                                   add_candidate->second.eligible;
            context.edge_gates[edge.id()] = {
                add_valid, keep_valid,
                keep_candidate == keep_candidates.end()
                    ? std::numeric_limits<double>::infinity()
                    : keep_candidate->second.robust_distance_m};
            const std::string range_id = UndirectedEdge::canonical(
                edge.reference, edge.owner).id();
            const auto variance = range_variance_m2_.find(range_id);
            if (variance != range_variance_m2_.end())
                context.range_variances_m2[range_id] = variance->second;
        }
        context.hard_row_request = hardRowRequest(snapshot, edges);
        context.nominal_controls = nominalControls(supervisor_.mode());
        context.progress_compatibility = config_.progress_compatibility;
        return context;
    }

    Eigen::Vector2d truthPosition(NodeId id) const {
        const auto mobile = std::find(
            mobile_ids_.begin(), mobile_ids_.end(), id);
        if (mobile != mobile_ids_.end()) {
            const Point point = swarm_.robots.at(
                static_cast<std::size_t>(mobile - mobile_ids_.begin()))->model->xy();
            return {point.x, point.y};
        }
        return fixed_positions_.at(id);
    }

    double maximumPairPositionTube(
        const JointEstimateSnapshot& snapshot,
        const std::vector<DirectedEdge>& edges) const {
        double maximum = 0.0;
        for (const DirectedEdge& edge : edges) {
            maximum = std::max(
                maximum, posteriorPairPositionTube(
                    snapshot, edge, config_.uncertainty_sigma));
        }
        return maximum;
    }

    PairwiseSnapshotTube snapshotPairTube(
        const JointEstimateSnapshot& snapshot,
        NodeId first,
        NodeId second) const {
        const double first_position = std::sqrt(std::max(
            0.0, detail::maximumPositionEigenvalue(snapshot, first)));
        const double second_position = std::sqrt(std::max(
            0.0, detail::maximumPositionEigenvalue(snapshot, second)));
        const double first_velocity = std::sqrt(std::max(
            0.0, detail::maximumVelocityEigenvalue(snapshot, first)));
        const double second_velocity = std::sqrt(std::max(
            0.0, detail::maximumVelocityEigenvalue(snapshot, second)));
        return PairwiseSnapshotTube{
            config_.uncertainty_sigma * (first_position + second_position) +
                config_.certified_shadow_relative_position_support_m,
            config_.uncertainty_sigma * (first_velocity + second_velocity) +
                config_.certified_shadow_relative_velocity_support_mps,
            SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    }

    double maximumCollisionPositionTube(
        const JointEstimateSnapshot& snapshot) const {
        double maximum = 0.0;
        for (std::size_t first = 0; first < mobile_ids_.size(); ++first) {
            const double first_sigma = std::sqrt(std::max(
                0.0, detail::maximumPositionEigenvalue(
                    snapshot, mobile_ids_[first])));
            for (std::size_t second = first + 1;
                 second < mobile_ids_.size(); ++second) {
                const double second_sigma = std::sqrt(std::max(
                    0.0, detail::maximumPositionEigenvalue(
                        snapshot, mobile_ids_[second])));
                maximum = std::max(
                    maximum,
                    config_.uncertainty_sigma *
                        (first_sigma + second_sigma));
            }
            maximum = std::max(
                maximum, config_.uncertainty_sigma * first_sigma);
        }
        return maximum;
    }

    static std::vector<DirectedEdge> edgeDifference(
        const std::vector<DirectedEdge>& first,
        const std::vector<DirectedEdge>& second) {
        std::set<std::string> second_ids;
        for (const DirectedEdge& edge : second) second_ids.insert(edge.id());
        std::vector<DirectedEdge> result;
        for (const DirectedEdge& edge : first)
            if (second_ids.count(edge.id()) == 0) result.push_back(edge);
        return result;
    }

    void applyDeterministicRangeBatch() {
        std::vector<RangeMeasurement> measurements;
        const std::int64_t timestamp = static_cast<std::int64_t>(
            std::llround(swarm_.robots.front()->runtime * 1.0e9));
        for (std::size_t first = 0; first < mobile_ids_.size(); ++first) {
            for (std::size_t second = first + 1;
                 second < mobile_ids_.size(); ++second) {
                const auto edge = UndirectedEdge::canonical(
                    mobile_ids_[first], mobile_ids_[second]);
                measurements.push_back({
                    timestamp, edge,
                    (truthPosition(edge.first) - truthPosition(edge.second)).norm(),
                    1.0});
            }
            for (const auto& [fixed, point] : fixed_positions_) {
                (void)point;
                const auto edge = UndirectedEdge::canonical(
                    mobile_ids_[first], fixed);
                measurements.push_back({
                    timestamp, edge,
                    (truthPosition(edge.first) - truthPosition(edge.second)).norm(),
                    1.0});
            }
        }
        for (const RangeMeasurement& measurement :
             canonicalizeRangeBatch(std::move(measurements))) {
            if (range_batch_count_ > 0 && range_uniform_(range_rng_) <
                config_.range_dropout_probability) {
                continue;
            }
            RangeMeasurement accepted = measurement;
            if (config_.range_noise_std_m > 0.0) {
                accepted.range_m += config_.range_noise_std_m *
                    range_normal_(range_rng_);
                accepted.variance_m2 = std::max(
                    accepted.variance_m2,
                    config_.range_noise_std_m * config_.range_noise_std_m);
            }
            const std::string range_id = measurement.edge.id();
            const NodeId master = std::find(
                mobile_ids_.begin(), mobile_ids_.end(),
                measurement.edge.first) != mobile_ids_.end()
                ? measurement.edge.first : measurement.edge.second;
            const InterimMasterMessage update =
                estimator_.makeUpdate(master, accepted);
            if (std::abs(update.innovation) >
                config_.maximum_accepted_range_innovation_m) {
                continue;
            }
            range_last_observation_s_[range_id] =
                swarm_.robots.front()->runtime;
            range_quality_[range_id] = 1.0;
            range_variance_m2_[range_id] = accepted.variance_m2;
            estimator_.applyUpdate(update);
        }
        ++range_batch_count_;
    }

    Swarm& swarm_;
    std::vector<NodeId> mobile_ids_;
    std::map<NodeId, Eigen::Vector2d> fixed_positions_;
    GrandFinaleSwarmAdapterConfig config_;
    InterimMasterDekf estimator_;
    CertifiedCoverageTracker coverage_;
    HybridSupervisor supervisor_;
    CanonicalHocbfQpController controller_;
    std::optional<TransitionProposal> pending_proposal_;
    std::optional<TransitionCertificate> pending_certificate_;
    std::size_t union_control_cycles_ = 0;
    std::vector<TransitionCertificate> transition_stack_;
    bool pending_is_retreat_ = false;
    std::string last_certification_reason_;
    std::map<std::string, double> range_last_observation_s_;
    std::map<std::string, double> range_quality_;
    std::map<std::string, double> range_variance_m2_;
    std::optional<std::map<NodeId, Eigen::Vector2d>> nominal_override_;
    std::mt19937 range_rng_;
    std::uniform_real_distribution<double> range_uniform_{0.0, 1.0};
    std::normal_distribution<double> range_normal_{0.0, 1.0};
    std::size_t range_batch_count_ = 0;
};

}  // namespace gf
