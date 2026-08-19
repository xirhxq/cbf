#pragma once

#include "Swarm.hpp"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/CanonicalGammaStarFeedback.hpp"
#include "grand_finale/DynamicPairCertifiedControl.hpp"
#include "grand_finale/BoundaryPolicy.hpp"
#include "grand_finale/BoundarySoftNominalSelector.hpp"
#include "grand_finale/CertifiedCoverageTracker.hpp"
#include "grand_finale/InterimMasterDekf.hpp"
#include "grand_finale/GurobiTopologySolver.hpp"
#include "grand_finale/HighsTopologySolver.hpp"
#include "grand_finale/InformationGateDiagnostic.hpp"

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
    // A non-positive value disables the plant-speed domain. Formal V1 uses
    // the 64-facet exact-ZOH applied-control path; speed_cbf_gain remains only
    // for explicitly constructed historical SpeedLimit fixtures.
    double speed_limit_mps = 0.0;
    double speed_cbf_gain = 1.0;
    std::size_t plant_speed_facet_count = 64;
    double maximum_yaw_rate_radps = 0.0;
    double position_gain = 0.4;
    double velocity_gain = 0.8;
    double estimator_acceleration_variance = 0.0;
    double initial_position_variance_m2 = 1.0e-4;
    double initial_velocity_variance_m2 = 1.0e-4;
    double certified_error_bound_m = 0.05;
    double certified_shadow_single_position_support_m = 0.0;
    double certified_shadow_single_velocity_support_mps = 0.0;
    double certified_shadow_relative_position_support_m = 0.0;
    double certified_shadow_relative_velocity_support_mps = 0.0;
    double maximum_accepted_range_innovation_m = 1.0;
    unsigned int range_random_seed = 2027;
    double range_noise_std_m = 0.0;
    double range_dropout_probability = 0.0;
    double sensor_radius_m = 1.6;
    CoverageFootprintKind coverage_footprint_kind =
        CoverageFootprintKind::Circular;
    double coverage_inner_radius_m = 0.0;
    double coverage_half_angle_rad = M_PI;
    double reference_distance_m = 850.0;
    double add_reference_distance_m = 849.0;
    double reference_uncertainty_m = 0.02;
    double uncertainty_sigma = 3.0;
    double maximum_reference_position_eigenvalue_m2 = 0.1;
    double maximum_posterior_eigenvalue_m2 = 0.1;
    double maximum_range_aoi_s = 1.0;
    double minimum_range_quality = 0.5;
    // Formal callers must state the operational reference-point separation.
    // Historical toy fixtures may still opt in to their explicit old value.
    double collision_distance_m =
        std::numeric_limits<double>::quiet_NaN();
    double collision_lambda1 = 1.0;
    double collision_lambda2 = 1.0;
    double residual_tolerance = 1.0e-7;
    double qp_oracle_tolerance = 1.0e-5;
    BoundaryPolicyConfig boundary;
    GammaFeedbackSelectionMode gamma_feedback_selection =
        GammaFeedbackSelectionMode::DiagnosticsOnly;
    std::optional<double> predictive_gamma_tau_mps2;
    std::size_t gamma_feedback_homotopy_segments = 8;
    double gamma_feedback_tolerance = 1.0e-10;
    ProgressCompatibilityConfig progress_compatibility{
        0.8, 0.0, 1.0e-9, true};
};

struct GrandFinaleGammaFeedbackDiagnostic {
    double current_gamma = -std::numeric_limits<double>::infinity();
    double nominal_predicted_gamma =
        -std::numeric_limits<double>::infinity();
    double maximum_margin_candidate_predicted_gamma =
        -std::numeric_limits<double>::infinity();
    double selected_predicted_gamma =
        -std::numeric_limits<double>::infinity();
    double controllable_predicted_gamma_range = 0.0;
    Eigen::Vector2d current_hard_projection = Eigen::Vector2d::Zero();
    Eigen::Vector2d maximum_margin_control = Eigen::Vector2d::Zero();
    Eigen::Vector2d selected_nominal = Eigen::Vector2d::Zero();
    bool intervened = false;
    std::string dominant_row;
    std::string fallback_reason;
};

struct GrandFinaleDynamicPairDiagnostic {
    bool attempted = false;
    bool applied = false;
    std::string reason;
    std::string pair_base_id;
    NodeId first_owner = 0;
    NodeId second_owner = 0;
    double transfer_interval_lower_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double transfer_interval_upper_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double selected_transfer_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double minimum_local_residual_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    NodeId limiting_owner = 0;
    std::string limiting_row_id;
    double dynamic_pair_local_sum_residual_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double once_reserve_full_pair_residual_mps2 =
        std::numeric_limits<double>::quiet_NaN();
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
    double minimum_plant_speed_applied_control_residual =
        std::numeric_limits<double>::infinity();
    double minimum_collision_h=std::numeric_limits<double>::infinity();
    double minimum_collision_psi1=std::numeric_limits<double>::infinity();
    double minimum_collision_residual=
        std::numeric_limits<double>::infinity();
    double minimum_reference_h=std::numeric_limits<double>::infinity();
    double minimum_reference_psi1=std::numeric_limits<double>::infinity();
    double minimum_reference_residual=
        std::numeric_limits<double>::infinity();
    std::size_t certified_control_count = 0;
    std::size_t updated_truth_cells = 0;
    std::size_t outside_observer_new_truth_cells = 0;
    double truth_coverage = 0.0;
    double certified_coverage = 0.0;
    double qp_wall_s = 0.0;
    CanonicalGammaFeedbackWork gamma_policy_work;
    std::map<NodeId, Eigen::Vector2d> applied_controls;
    std::map<NodeId,BoundarySoftQpResult> boundary_soft_nominal;
    std::map<NodeId, double> applied_yaw_rates_radps;
    std::map<NodeId, GrandFinaleGammaFeedbackDiagnostic> gamma_feedback;
    GrandFinaleDynamicPairDiagnostic dynamic_pair;
    Task10p11ComputeProfile compute_profile;
};

struct GrandFinaleStageZero {
    bool initialized = false;
    std::string reason;
    std::uint64_t estimator_version = 0;
    double truth_coverage = 0.0;
    double certified_coverage = 0.0;
};

// Unsafe-by-design causal diagnostic.  Only input and plant-speed applied-
// control rows
// authorize the applied acceleration; collision/reference/workspace rows are
// rebuilt and audited but intentionally do not constrain the control.  This
// type cannot be substituted for GrandFinaleSwarmStep in the formal path.
struct GrandFinaleNominalOnlyDiagnosticStep {
    bool advanced=false;
    bool safety_authorized=false;
    std::string reason;
    std::size_t restricted_row_count=0;
    std::size_t full_hard_row_count=0;
    double minimum_restricted_residual=
        std::numeric_limits<double>::infinity();
    double minimum_plant_speed_applied_control_residual=
        std::numeric_limits<double>::infinity();
    double minimum_full_hard_residual=
        std::numeric_limits<double>::infinity();
    double truth_coverage=0.0;
    double certified_coverage=0.0;
    std::map<NodeId,Eigen::Vector2d> applied_controls;
    std::map<NodeId,double> current_gamma;
    std::map<NodeId,double> local_maximum_predicted_gamma;
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
    NodeId minimum_effective_reference_owner = 0;
    // FIM fields below use the qualified accepted ranging-information graph,
    // not the directed reference-control DAG.  The reference-only values are
    // retained explicitly as diagnostics so the two graph semantics cannot be
    // silently conflated again.
    std::size_t minimum_information_edge_count = 0;
    NodeId minimum_information_edge_owner = 0;
    double minimum_fim_eigenvalue =
        std::numeric_limits<double>::infinity();
    NodeId minimum_fim_owner = 0;
    double maximum_posterior_eigenvalue = 0.0;
    NodeId maximum_posterior_owner = 0;
    double minimum_robust_fim_cone_lower_bound =
        std::numeric_limits<double>::infinity();
    NodeId minimum_robust_fim_owner = 0;
    double minimum_reference_only_fim_eigenvalue =
        std::numeric_limits<double>::infinity();
    NodeId minimum_reference_only_fim_owner = 0;
    double minimum_reference_only_robust_fim_cone_lower_bound =
        std::numeric_limits<double>::infinity();
    NodeId minimum_reference_only_robust_fim_owner = 0;
    double minimum_range_aoi_margin_s =
        std::numeric_limits<double>::infinity();
    NodeId minimum_range_aoi_owner = 0;
    std::string minimum_range_aoi_edge;
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

struct AcceptedRangeUpdateAudit {
    RangeMeasurement measurement;
    NodeId master=0;
    double innovation=0.0;
    double innovation_variance=0.0;
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
            !std::isfinite(config_.speed_limit_mps) ||
            config_.speed_limit_mps < 0.0 ||
            !std::isfinite(config_.speed_cbf_gain) ||
            config_.speed_cbf_gain <= 0.0 ||
            (config_.speed_limit_mps>0.0 &&
             config_.plant_speed_facet_count!=64) ||
            !std::isfinite(config_.maximum_yaw_rate_radps) ||
            config_.maximum_yaw_rate_radps < 0.0 ||
            !std::isfinite(config_.collision_distance_m) ||
            config_.collision_distance_m <= 0.0 ||
            !std::isfinite(config_.collision_lambda1) ||
            config_.collision_lambda1 <= 0.0 ||
            !std::isfinite(config_.collision_lambda2) ||
            config_.collision_lambda2 <= 0.0 ||
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
            !std::isfinite(config_.coverage_inner_radius_m) ||
            config_.coverage_inner_radius_m < 0.0 ||
            config_.coverage_inner_radius_m >= config_.sensor_radius_m ||
            !std::isfinite(config_.coverage_half_angle_rad) ||
            config_.coverage_half_angle_rad <= 0.0 ||
            config_.coverage_half_angle_rad > M_PI ||
            !std::isfinite(config_.certified_error_bound_m) ||
            config_.certified_error_bound_m < 0.0 ||
            !std::isfinite(
                config_.certified_shadow_single_position_support_m) ||
            config_.certified_shadow_single_position_support_m < 0.0 ||
            !std::isfinite(
                config_.certified_shadow_single_velocity_support_mps) ||
            config_.certified_shadow_single_velocity_support_mps < 0.0 ||
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
            config_.velocity_gain < 0.0 ||
            config_.gamma_feedback_homotopy_segments == 0 ||
            !std::isfinite(config_.gamma_feedback_tolerance) ||
            config_.gamma_feedback_tolerance < 0.0 ||
            (config_.predictive_gamma_tau_mps2.has_value() &&
             (!std::isfinite(*config_.predictive_gamma_tau_mps2) ||
              *config_.predictive_gamma_tau_mps2 < 0.0)) ||
            (config_.gamma_feedback_selection ==
                 GammaFeedbackSelectionMode::LeastIntervention &&
             !config_.predictive_gamma_tau_mps2.has_value())) {
            throw std::invalid_argument("invalid GrandFinale safety configuration");
        }
        supervisor_.initializeTopology(std::move(initial_topology), 1);
    }

    GrandFinaleStageZero initializeStageZero() {
        GrandFinaleStageZero result;
        if (stage_zero_initialized_) {
            result.reason = "stage_zero_already_initialized";
            return result;
        }
        swarm_.prepareCertifiedControlStep();
        applyDeterministicRangeBatch();
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        observeCoverageSnapshot(snapshot);
        swarm_.observeGridWorldAtCurrentState();
        stage_zero_initialized_ = true;
        result.initialized = true;
        result.reason = "initialized";
        result.estimator_version = estimator_.version();
        result.truth_coverage = coverage_.truthFraction();
        result.certified_coverage = coverage_.certifiedFraction();
        return result;
    }

    GrandFinaleSwarmStep step() {
        GrandFinaleSwarmStep metrics;
        metrics.mode = supervisor_.mode();
        metrics.topology_version = supervisor_.topologyVersion();
        metrics.estimator_version_before = estimator_.version();
        swarm_.prepareCertifiedControlStep();
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        const std::uint64_t snapshot_version = estimator_.version();
        auto task_nominal = nominal_override_.has_value()
            ? *nominal_override_
            : nominalControls(metrics.mode, snapshot);
        if (config_.boundary.policy==BoundaryPolicy::SoftSearchRetention) {
            const auto soft_rows=boundarySoftRows(snapshot);
            for (NodeId owner : mobile_ids_) {
                const auto selected=boundary_soft_selector_.solve({
                    config_.solver_profile,owner,task_nominal.at(owner),
                    config_.acceleration_half_box,
                    config_.boundary.soft_slack_weight,soft_rows,
                    config_.residual_tolerance});
                metrics.boundary_soft_nominal.emplace(owner,selected);
                if (selected.nominal_available)
                    task_nominal[owner]=selected.selected_nominal;
            }
        }
        const CanonicalGammaFeedbackConfig feedback_config{
            config_.acceleration_half_box,
            config_.gamma_feedback_homotopy_segments,
            config_.gamma_feedback_selection,
            config_.predictive_gamma_tau_mps2,
            config_.gamma_feedback_tolerance};
        CanonicalGammaFeedbackEvaluationContext feedback_context;
        const auto feedback_batch=evaluateCanonicalGammaFeedbackBatch(
            snapshot,task_nominal,feedback_config,config_.dt_s,
            config_.estimator_acceleration_variance,
            [&](const JointEstimateSnapshot& value) {
                return hardRowRequest(value,supervisor_.topology());
            },feedback_context);
        metrics.gamma_policy_work=feedback_batch.work;
        metrics.compute_profile.merge(feedback_batch.compute_profile);
        if (!feedback_batch.valid) {
            metrics.reason=feedback_batch.reason==
                    "current_canonical_row_rebuild_failed"
                ? "snapshot_robust_hard_row_invalid"
                : feedback_batch.reason;
            return metrics;
        }
        const auto& rows=feedback_batch.current_rows;
        const auto& selected_nominal=feedback_batch.selected_controls;
        for (NodeId owner : mobile_ids_) {
            const auto& stage=feedback_batch.stages.at(owner);
            const auto& selected=feedback_batch.selections.at(owner);
            metrics.gamma_feedback.emplace(owner,
                GrandFinaleGammaFeedbackDiagnostic{
                    stage.current_gamma,
                    selected.nominal_predicted_gamma,
                    selected.maximum_margin_candidate_predicted_gamma,
                    selected.selected_predicted_gamma,
                    selected.controllable_predicted_gamma_range,
                    stage.task_projection,
                    stage.maximum_margin_control,
                    selected.selected_control,
                    selected.intervened,
                    selected.dominant_row,
                    selected.fallback_reason});
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
                            supervisor_.mode(), selected_nominal.at(owner),
                            config_.acceleration_half_box, rows,
                            config_.residual_tolerance});
                    metrics.qp_wall_s += std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - qp_start).count();
                    const bool steady=result.solver_instance_reused;
                    metrics.compute_profile.record(
                        Task10p11ComputePhase::FinalQp,
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now()-qp_start).count(),
                        steady);
                    metrics.compute_profile.record(
                        Task10p11ComputePhase::ExactHardProjection,
                        result.exact_projection_wall_s,steady);
                    metrics.compute_profile.record(
                        Task10p11ComputePhase::SolverInitialization,
                        result.solver_initialization_wall_s,steady);
                    metrics.compute_profile.record(
                        Task10p11ComputePhase::SolverModelUpdate,
                        result.solver_model_update_wall_s,steady);
                    metrics.compute_profile.record(
                        Task10p11ComputePhase::RobustQpSolve,
                        result.solver_solve_wall_s,steady);
                    metrics.compute_profile.record(
                        Task10p11ComputePhase::ResidualTokenAudit,
                        result.residual_token_audit_wall_s,steady);
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
                    for (const auto& row:rows) {
                        if (row.owner!=owner) continue;
                        if (row.kind==CanonicalHardRowKind::
                                PlantSpeedAppliedControl) {
                            metrics.minimum_plant_speed_applied_control_residual=
                                std::min(
                                    metrics.minimum_plant_speed_applied_control_residual,
                                    row.margin(result.control));
                        } else if (row.kind==CanonicalHardRowKind::Collision) {
                            metrics.minimum_collision_h=std::min(
                                metrics.minimum_collision_h,row.barrier_h);
                            metrics.minimum_collision_psi1=std::min(
                                metrics.minimum_collision_psi1,row.barrier_psi1);
                            metrics.minimum_collision_residual=std::min(
                                metrics.minimum_collision_residual,
                                row.margin(result.control));
                        } else if (row.kind==
                                CanonicalHardRowKind::ReferenceDistance) {
                            metrics.minimum_reference_h=std::min(
                                metrics.minimum_reference_h,row.barrier_h);
                            metrics.minimum_reference_psi1=std::min(
                                metrics.minimum_reference_psi1,row.barrier_psi1);
                            metrics.minimum_reference_residual=std::min(
                                metrics.minimum_reference_residual,
                                row.margin(result.control));
                        }
                    }
                }
                return applied;
            };
        const auto certified_controls=build_controls();
        const auto physical_started=std::chrono::steady_clock::now();
        const auto physical = swarm_.applyCertifiedControlsAndAdvance(
            config_.dt_s, certified_controls, yaw_rate_override_,
            config_.speed_limit_mps>0.0
                ?std::optional<double>(config_.speed_limit_mps)
                :std::nullopt);
        metrics.compute_profile.record(
            Task10p11ComputePhase::PlantPreflightZoh,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()-physical_started).count(),true);
        metrics.advanced = physical.advanced;
        metrics.updated_truth_cells = physical.updated_truth_cells;
        if (!physical.advanced) {
            if (metrics.reason.empty()) metrics.reason = physical.reason;
            return metrics;
        }

        const auto estimator_started=std::chrono::steady_clock::now();
        for (NodeId owner : mobile_ids_) {
            estimator_.propagateLocal(
                owner, applied.at(static_cast<int>(owner)), config_.dt_s,
                config_.estimator_acceleration_variance);
        }
        applyDeterministicRangeBatch();
        metrics.compute_profile.record(
            Task10p11ComputePhase::OnlineEstimator,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()-estimator_started).count(),true);
        const JointEstimateSnapshot after = estimator_.reconstructForAudit();
        metrics.outside_observer_new_truth_cells=observeCoverageSnapshot(after);
        metrics.certified_control_count = applied.size();
        for (const auto& [id, control] : applied)
            metrics.applied_controls[static_cast<NodeId>(id)] = control;
        if (yaw_rate_override_.has_value()) {
            for (const auto& [id,rate] : *yaw_rate_override_)
                metrics.applied_yaw_rates_radps[static_cast<NodeId>(id)]=rate;
        }
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

    GrandFinaleNominalOnlyDiagnosticStep stepNominalOnlyCausalDiagnostic(
        const std::map<NodeId,Eigen::Vector2d>& nominal,
        const std::map<NodeId,double>& yaw_rates) {
        GrandFinaleNominalOnlyDiagnosticStep result;
        if (!stage_zero_initialized_) {
            result.reason="stage_zero_not_initialized";
            return result;
        }
        if (nominal.size()!=mobile_ids_.size() ||
            yaw_rates.size()!=mobile_ids_.size()) {
            result.reason="nominal_only_incomplete";
            return result;
        }
        for (NodeId owner:mobile_ids_) {
            const auto control=nominal.find(owner);
            const auto yaw=yaw_rates.find(owner);
            if (control==nominal.end() || yaw==yaw_rates.end() ||
                !control->second.allFinite() || !std::isfinite(yaw->second) ||
                std::abs(yaw->second)>config_.maximum_yaw_rate_radps+1e-12) {
                result.reason="nominal_only_invalid";
                return result;
            }
        }

        swarm_.prepareCertifiedControlStep();
        const JointEstimateSnapshot snapshot=estimator_.reconstructForAudit();
        const std::uint64_t snapshot_version=estimator_.version();
        std::vector<CanonicalHardRow> full_rows;
        try {
            full_rows=buildCanonicalHardRows(
                hardRowRequest(snapshot,supervisor_.topology()));
        } catch (...) {
            result.reason="snapshot_robust_hard_row_invalid";
            return result;
        }
        result.full_hard_row_count=full_rows.size();
        std::vector<CanonicalHardRow> restricted_rows;
        for (const auto& row:full_rows)
            if (row.kind==CanonicalHardRowKind::PlantSpeedAppliedControl ||
                row.kind==CanonicalHardRowKind::InputBox)
                restricted_rows.push_back(row);
        result.restricted_row_count=restricted_rows.size();

        for (NodeId owner:mobile_ids_) {
            const auto gamma=solveCanonicalGammaStar(
                full_rows,owner,config_.acceleration_half_box);
            result.current_gamma[owner]=gamma.valid
                ?gamma.gamma:-std::numeric_limits<double>::infinity();
        }
        const CanonicalGammaFeedbackConfig maximum_config{
            config_.acceleration_half_box,
            config_.gamma_feedback_homotopy_segments,
            GammaFeedbackSelectionMode::MaximumPredictedMargin,
            std::nullopt,config_.gamma_feedback_tolerance};
        CanonicalGammaFeedbackEvaluationContext feedback_context;
        const auto predicted=evaluateCanonicalGammaFeedbackBatch(
            snapshot,nominal,maximum_config,config_.dt_s,
            config_.estimator_acceleration_variance,
            [&](const JointEstimateSnapshot& value) {
                return hardRowRequest(value,supervisor_.topology());
            },feedback_context);
        for (NodeId owner:mobile_ids_) {
            const auto found=predicted.selections.find(owner);
            result.local_maximum_predicted_gamma[owner]=
                predicted.valid && found!=predicted.selections.end()
                ?found->second.selected_predicted_gamma
                :std::numeric_limits<double>::quiet_NaN();
        }

        Swarm::CertifiedControlBatch applied;
        for (NodeId owner:mobile_ids_) {
            const auto qp=controller_.solve({
                config_.solver_profile,owner,snapshot_version,
                supervisor_.topologyVersion(),supervisor_.mode(),
                nominal.at(owner),config_.acceleration_half_box,
                restricted_rows,config_.residual_tolerance});
            if (!controlMayBeApplied(qp,snapshot_version,
                    supervisor_.topologyVersion(),supervisor_.mode()) ||
                qp.exact_oracle_error>config_.qp_oracle_tolerance) {
                result.reason=qp.failure_reason.empty()
                    ?"nominal_only_restricted_qp_failed":qp.failure_reason;
                return result;
            }
            applied[static_cast<int>(owner)]=qp.control;
            result.applied_controls[owner]=qp.control;
            result.minimum_restricted_residual=std::min(
                result.minimum_restricted_residual,
                minimumCanonicalOwnerResidual(
                    restricted_rows,owner,qp.control));
            for (const auto& row:restricted_rows)
                if (row.owner==owner &&
                    row.kind==CanonicalHardRowKind::PlantSpeedAppliedControl)
                    result.minimum_plant_speed_applied_control_residual=std::min(
                        result.minimum_plant_speed_applied_control_residual,
                        row.margin(qp.control));
            result.minimum_full_hard_residual=std::min(
                result.minimum_full_hard_residual,
                minimumCanonicalOwnerResidual(full_rows,owner,qp.control));
        }
        std::map<int,double> yaw_batch;
        for (const auto& [owner,rate]:yaw_rates)
            yaw_batch[static_cast<int>(owner)]=rate;
        const auto physical=swarm_.applyCertifiedControlsAndAdvance(
            config_.dt_s,applied,yaw_batch,
            config_.speed_limit_mps>0.0
                ?std::optional<double>(config_.speed_limit_mps)
                :std::nullopt);
        if (!physical.advanced) {
            result.reason=physical.reason;
            return result;
        }
        for (NodeId owner:mobile_ids_)
            estimator_.propagateLocal(owner,
                applied.at(static_cast<int>(owner)),config_.dt_s,
                config_.estimator_acceleration_variance);
        applyDeterministicRangeBatch();
        const auto after=estimator_.reconstructForAudit();
        observeCoverageSnapshot(after);
        result.truth_coverage=coverage_.truthFraction();
        result.certified_coverage=coverage_.certifiedFraction();
        result.advanced=true;
        result.reason="nominal_only_causal_advanced";
        return result;
    }

    GrandFinaleSwarmStep stepWithNominalAndYawRates(
        const std::map<NodeId, Eigen::Vector2d>& nominal,
        const std::map<NodeId, double>& yaw_rates) {
        GrandFinaleSwarmStep rejected;
        rejected.mode = supervisor_.mode();
        rejected.topology_version = supervisor_.topologyVersion();
        rejected.estimator_version_before = estimator_.version();
        if (yaw_rates.size()!=mobile_ids_.size()) {
            rejected.reason="yaw_rate_batch_incomplete";
            return rejected;
        }
        for (const auto& [id,rate] : yaw_rates) {
            if (std::find(mobile_ids_.begin(),mobile_ids_.end(),id)==
                    mobile_ids_.end() || !std::isfinite(rate) ||
                std::abs(rate)>config_.maximum_yaw_rate_radps+1.0e-12) {
                rejected.reason="yaw_rate_batch_invalid";
                return rejected;
            }
        }
        yaw_rate_override_=std::map<int,double>{};
        for (const auto& [id,rate] : yaw_rates)
            (*yaw_rate_override_)[static_cast<int>(id)]=rate;
        try {
            GrandFinaleSwarmStep result=stepWithNominal(nominal);
            yaw_rate_override_.reset();
            return result;
        } catch (...) {
            yaw_rate_override_.reset();
            throw;
        }
    }

    GrandFinaleSwarmStep stepWithDynamicPairCertifiedControls(
        const DynamicPairCertifiedControlRequest& request,
        const std::map<NodeId, double>& yaw_rates,
        std::uint64_t expected_estimator_version,
        std::uint64_t expected_topology_version) {
        GrandFinaleSwarmStep metrics;
        metrics.mode = supervisor_.mode();
        metrics.topology_version = supervisor_.topologyVersion();
        metrics.estimator_version_before = estimator_.version();
        metrics.dynamic_pair.attempted = true;
        metrics.dynamic_pair.pair_base_id = request.pair_base_id;
        metrics.dynamic_pair.first_owner = request.first_owner;
        metrics.dynamic_pair.second_owner = request.second_owner;
        metrics.dynamic_pair.transfer_interval_lower_mps2 =
            request.transfer_interval_lower_mps2;
        metrics.dynamic_pair.transfer_interval_upper_mps2 =
            request.transfer_interval_upper_mps2;
        metrics.dynamic_pair.selected_transfer_mps2 = request.transfer_mps2;
        if (yaw_rates.size() != mobile_ids_.size()) {
            metrics.reason = "yaw_rate_batch_incomplete";
            metrics.dynamic_pair.reason = metrics.reason;
            return metrics;
        }
        for (const auto& [id, rate] : yaw_rates) {
            if (std::find(mobile_ids_.begin(), mobile_ids_.end(), id) ==
                    mobile_ids_.end() || !std::isfinite(rate) ||
                std::abs(rate) > config_.maximum_yaw_rate_radps + 1e-12) {
                metrics.reason = "yaw_rate_batch_invalid";
                metrics.dynamic_pair.reason = metrics.reason;
                return metrics;
            }
        }

        swarm_.prepareCertifiedControlStep();
        const JointEstimateSnapshot snapshot = estimator_.reconstructForAudit();
        const std::uint64_t snapshot_version = estimator_.version();
        if (snapshot_version != expected_estimator_version ||
            supervisor_.topologyVersion() != expected_topology_version ||
            supervisor_.mode() != SupervisorMode::Search ||
            pending_proposal_.has_value() || supervisor_.transitionPending()) {
            metrics.reason = "dynamic_pair_control_epoch_stale";
            metrics.dynamic_pair.reason = metrics.reason;
            return metrics;
        }
        std::vector<CanonicalHardRow> rows;
        try {
            rows = buildCanonicalHardRows(
                hardRowRequest(snapshot, supervisor_.topology()));
        } catch (...) {
            metrics.reason = "snapshot_robust_hard_row_invalid";
            metrics.dynamic_pair.reason = metrics.reason;
            return metrics;
        }
        const auto audit = auditDynamicPairCertifiedControls(
            rows, mobile_ids_, config_.acceleration_half_box,
            config_.residual_tolerance, request);
        metrics.dynamic_pair.reason = audit.reason;
        metrics.dynamic_pair.minimum_local_residual_mps2 =
            audit.minimum_local_residual_mps2;
        metrics.dynamic_pair.limiting_owner = audit.limiting_owner;
        metrics.dynamic_pair.limiting_row_id = audit.limiting_row_id;
        metrics.dynamic_pair.dynamic_pair_local_sum_residual_mps2 =
            audit.dynamic_pair_local_sum_residual_mps2;
        metrics.dynamic_pair.once_reserve_full_pair_residual_mps2 =
            audit.once_reserve_full_pair_residual_mps2;
        if (!audit.valid) {
            metrics.reason = audit.reason;
            return metrics;
        }

        Swarm::CertifiedControlBatch applied;
        for (const auto& [owner, control] : request.controls)
            applied[static_cast<int>(owner)] = control;
        metrics.minimum_hard_residual = audit.minimum_local_residual_mps2;
        std::vector<const CanonicalHardRow*> selected_pair_rows;
        for (const CanonicalHardRow& row : rows) {
            if (dynamicPairBaseId(row.id) == request.pair_base_id)
                selected_pair_rows.push_back(&row);
        }
        std::sort(selected_pair_rows.begin(), selected_pair_rows.end(),
            [](const auto* left, const auto* right) {
                return left->owner < right->owner;
            });
        for (const CanonicalHardRow& row : rows) {
            const auto control = request.controls.find(row.owner);
            double residual = row.margin(control->second);
            if (selected_pair_rows.size() == 2 &&
                row.id == selected_pair_rows[0]->id)
                residual += request.transfer_mps2;
            if (selected_pair_rows.size() == 2 &&
                row.id == selected_pair_rows[1]->id)
                residual -= request.transfer_mps2;
            if (row.kind == CanonicalHardRowKind::PlantSpeedAppliedControl) {
                metrics.minimum_plant_speed_applied_control_residual = std::min(
                    metrics.minimum_plant_speed_applied_control_residual,
                    residual);
            } else if (row.kind == CanonicalHardRowKind::Collision) {
                metrics.minimum_collision_h = std::min(
                    metrics.minimum_collision_h, row.barrier_h);
                metrics.minimum_collision_psi1 = std::min(
                    metrics.minimum_collision_psi1, row.barrier_psi1);
                metrics.minimum_collision_residual = std::min(
                    metrics.minimum_collision_residual, residual);
            } else if (row.kind == CanonicalHardRowKind::ReferenceDistance) {
                metrics.minimum_reference_h = std::min(
                    metrics.minimum_reference_h, row.barrier_h);
                metrics.minimum_reference_psi1 = std::min(
                    metrics.minimum_reference_psi1, row.barrier_psi1);
                metrics.minimum_reference_residual = std::min(
                    metrics.minimum_reference_residual, residual);
            }
        }
        std::map<int, double> yaw_batch;
        for (const auto& [owner, rate] : yaw_rates)
            yaw_batch[static_cast<int>(owner)] = rate;
        const auto physical_started = std::chrono::steady_clock::now();
        const auto physical = swarm_.applyCertifiedControlsAndAdvance(
            config_.dt_s, applied, yaw_batch,
            config_.speed_limit_mps > 0.0
                ? std::optional<double>(config_.speed_limit_mps)
                : std::nullopt);
        metrics.compute_profile.record(
            Task10p11ComputePhase::PlantPreflightZoh,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - physical_started).count(),
            true);
        metrics.advanced = physical.advanced;
        metrics.updated_truth_cells = physical.updated_truth_cells;
        if (!physical.advanced) {
            metrics.reason = physical.reason;
            metrics.dynamic_pair.reason = physical.reason;
            return metrics;
        }

        const auto estimator_started = std::chrono::steady_clock::now();
        for (NodeId owner : mobile_ids_) {
            estimator_.propagateLocal(
                owner, request.controls.at(owner), config_.dt_s,
                config_.estimator_acceleration_variance);
        }
        applyDeterministicRangeBatch();
        metrics.compute_profile.record(
            Task10p11ComputePhase::OnlineEstimator,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - estimator_started).count(),
            true);
        const JointEstimateSnapshot after = estimator_.reconstructForAudit();
        metrics.outside_observer_new_truth_cells = observeCoverageSnapshot(after);
        metrics.certified_control_count = request.controls.size();
        metrics.applied_controls = request.controls;
        metrics.applied_yaw_rates_radps = yaw_rates;
        metrics.estimator_version_after = estimator_.version();
        metrics.truth_coverage = coverage_.truthFraction();
        metrics.certified_coverage = coverage_.certifiedFraction();
        metrics.reason = "advanced";
        metrics.dynamic_pair.applied = true;
        metrics.dynamic_pair.reason = "dynamic_pair_control_batch_applied";
        return metrics;
    }

    bool beginReplacement(
        const DirectedEdge& new_edge,
        const DirectedEdge& old_edge,
        bool update_supervisor_on_rejection = true) {
        if (pending_proposal_.has_value()) return false;
        const TransitionCertificate certificate=auditReplacement(
            new_edge,old_edge);
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
        pending_proposal_ = TransitionProposal{
            certificate.old_edges,new_edge,old_edge,
            certificate.topology_version,certificate.estimator_version};
        pending_certificate_ = certificate;
        union_control_cycles_ = 0;
        return true;
    }

    TransitionCertificate auditReplacement(
        const DirectedEdge& new_edge,const DirectedEdge& old_edge) {
        const JointEstimateSnapshot snapshot=estimator_.reconstructForAudit();
        const TransitionProposal proposal{
            supervisor_.topology(),new_edge,old_edge,
            supervisor_.topologyVersion(),estimator_.version()};
        return TransitionCertifier{}.certify(
            proposal,certificationContext(snapshot,{new_edge}),true);
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
    std::vector<AcceptedRangeUpdateAudit> lastAcceptedRangeBatchAudit() const {
        return last_accepted_range_batch_audit_;
    }
    std::vector<Eigen::Vector2d> searchPolygonVertices() const {
        return searchPolygon();
    }
    std::map<NodeId, Eigen::Vector2d> currentNominalControls() {
        return nominalControls(
            supervisor_.mode(), estimator_.reconstructForAudit());
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
    CanonicalHardRowRequest currentSnapshotHardRowRequest(
        const std::vector<DirectedEdge>& topology) const {
        return hardRowRequest(estimator_.reconstructForAudit(),topology);
    }
    CanonicalHardRowRequest snapshotHardRowRequest(
        const JointEstimateSnapshot& snapshot,
        const std::vector<DirectedEdge>& topology) const {
        return hardRowRequest(snapshot,topology);
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
        audit.minimum_information_edge_count =
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
            if (effective_edges.size()<
                audit.minimum_effective_reference_count) {
                audit.minimum_effective_reference_count=
                    effective_edges.size();
                audit.minimum_effective_reference_owner=owner;
            }
            const double reference_only_fim = effective_edges.size() < 2
                ? -std::numeric_limits<double>::infinity()
                : Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    referenceFim(
                        owner, effective_edges, snapshot,
                        context.range_variances_m2))
                    .eigenvalues().minCoeff();
            if (reference_only_fim<
                audit.minimum_reference_only_fim_eigenvalue) {
                audit.minimum_reference_only_fim_eigenvalue=reference_only_fim;
                audit.minimum_reference_only_fim_owner=owner;
            }
            const double reference_only_robust_fim = effective_edges.size()<2
                ?-std::numeric_limits<double>::infinity()
                :robustReferenceFimConeLowerBound(owner,effective_edges,snapshot,
                    context.range_variances_m2,config_.uncertainty_sigma);
            if (reference_only_robust_fim<
                audit.minimum_reference_only_robust_fim_cone_lower_bound) {
                audit.minimum_reference_only_robust_fim_cone_lower_bound=
                    reference_only_robust_fim;
                audit.minimum_reference_only_robust_fim_owner=owner;
            }

            std::vector<DirectedEdge> information_edges;
            std::map<std::string,double> information_variances;
            std::vector<NodeId> information_references=mobile_ids_;
            for (const auto& [id,position]:fixed_positions_) {
                (void)position;
                information_references.push_back(id);
            }
            const double now_s=swarm_.robots.front()->runtime;
            const Eigen::Vector2d owner_position=
                detail::nodePosition(snapshot,owner);
            for (const NodeId reference:information_references) {
                if (reference==owner) continue;
                const std::string id=UndirectedEdge::canonical(
                    owner,reference).id();
                const auto observed=range_last_observation_s_.find(id);
                const auto quality=range_quality_.find(id);
                const auto variance=range_variance_m2_.find(id);
                if (observed==range_last_observation_s_.end() ||
                    quality==range_quality_.end() ||
                    variance==range_variance_m2_.end() ||
                    !std::isfinite(observed->second) ||
                    !std::isfinite(quality->second) ||
                    !std::isfinite(variance->second) || variance->second<=0.0 ||
                    std::max(0.0,now_s-observed->second)>
                        config_.maximum_range_aoi_s+1.0e-12 ||
                    quality->second+1.0e-12<config_.minimum_range_quality)
                    continue;
                const double distance=(owner_position-
                    detail::nodePosition(snapshot,reference)).norm();
                if (!std::isfinite(distance) || distance<=1.0e-12) continue;
                information_edges.emplace_back(reference,owner);
                information_variances.emplace(id,variance->second);
            }
            if (information_edges.size()<audit.minimum_information_edge_count) {
                audit.minimum_information_edge_count=information_edges.size();
                audit.minimum_information_edge_owner=owner;
            }
            const double fim = information_edges.size() < 2
                ? -std::numeric_limits<double>::infinity()
                : Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    referenceFim(owner,information_edges,snapshot,
                        information_variances)).eigenvalues().minCoeff();
            if (fim<audit.minimum_fim_eigenvalue) {
                audit.minimum_fim_eigenvalue=fim;
                audit.minimum_fim_owner=owner;
            }
            const double robust_fim = information_edges.size() < 2
                ? -std::numeric_limits<double>::infinity()
                : robustReferenceFimConeLowerBound(
                    owner,information_edges,snapshot,
                    information_variances,config_.uncertainty_sigma);
            if (robust_fim<audit.minimum_robust_fim_cone_lower_bound) {
                audit.minimum_robust_fim_cone_lower_bound=robust_fim;
                audit.minimum_robust_fim_owner=owner;
            }
            for (const auto& edge : effective_edges) {
                const std::string id=UndirectedEdge::canonical(
                    edge.owner,edge.reference).id();
                const auto observed=range_last_observation_s_.find(id);
                if (observed==range_last_observation_s_.end()) {
                    audit.minimum_range_aoi_margin_s=
                        -std::numeric_limits<double>::infinity();
                    audit.minimum_range_aoi_owner=owner;
                    audit.minimum_range_aoi_edge=id;
                    continue;
                }
                const double margin=config_.maximum_range_aoi_s-
                    std::max(0.0,now_s-observed->second);
                if (margin<audit.minimum_range_aoi_margin_s) {
                    audit.minimum_range_aoi_margin_s=margin;
                    audit.minimum_range_aoi_owner=owner;
                    audit.minimum_range_aoi_edge=id;
                }
            }
            const double posterior =
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    marginalPositionCovariance(snapshot, owner))
                    .eigenvalues().maxCoeff();
            if (posterior>audit.maximum_posterior_eigenvalue) {
                audit.maximum_posterior_eigenvalue=posterior;
                audit.maximum_posterior_owner=owner;
            }
        }
        return audit;
    }

private:
    std::size_t observeCoverageSnapshot(const JointEstimateSnapshot& snapshot) {
        std::size_t outside_observer_new_truth_cells=0;
        const auto polygon=searchPolygon();
        for (std::size_t index=0; index<mobile_ids_.size(); ++index) {
            const Point truth = swarm_.robots.at(index)->model->xy();
            const bool outside=distanceOutsidePolygon(
                Eigen::Vector2d(truth.x,truth.y),polygon)>1e-12;
            const int before=coverage_.truthCoveredCount();
            const Eigen::Vector2d estimate = snapshot.mean.segment<2>(4*index);
            const double coverage_error = std::max(
                config_.certified_shadow_single_position_support_m,
                std::max(config_.certified_error_bound_m,
                    config_.uncertainty_sigma*std::sqrt(std::max(
                        0.0,detail::maximumPositionEigenvalue(
                            snapshot,mobile_ids_[index])))));
            if (config_.coverage_footprint_kind ==
                CoverageFootprintKind::ForwardSector) {
                coverage_.observeSector(
                    truth,Point(estimate.x(),estimate.y()),coverage_error,
                    config_.coverage_inner_radius_m,config_.sensor_radius_m,
                    config_.coverage_half_angle_rad,
                    swarm_.robots.at(index)->model->getStateVariable("yawRad"));
            } else {
                coverage_.observe(
                    truth,Point(estimate.x(),estimate.y()),coverage_error,
                    config_.sensor_radius_m);
            }
            if (outside)
                outside_observer_new_truth_cells+=static_cast<std::size_t>(
                    coverage_.truthCoveredCount()-before);
        }
        return outside_observer_new_truth_cells;
    }

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
        SupervisorMode mode,const JointEstimateSnapshot& snapshot) {
        std::map<NodeId, Eigen::Vector2d> result;
        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            const auto& robot = swarm_.robots.at(index);
            const Eigen::Vector4d estimated =
                snapshot.mean.segment<4>(4*index);
            if (mode == SupervisorMode::Hold ||
                mode == SupervisorMode::Retreat) {
                Eigen::Vector2d braking =
                    -config_.velocity_gain * estimated.tail<2>();
                braking.x() = std::max(-config_.acceleration_half_box,
                    std::min(config_.acceleration_half_box, braking.x()));
                braking.y() = std::max(-config_.acceleration_half_box,
                    std::min(config_.acceleration_half_box, braking.y()));
                result[mobile_ids_[index]] = braking;
            } else {
                const Point estimate(estimated.x(),estimated.y());
                result[mobile_ids_[index]] =
                    robot->coverageNominalAcceleration(
                        estimate,estimated.tail<2>(),config_.position_gain,
                        config_.velocity_gain,config_.acceleration_half_box);
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
            1.0, config_.collision_lambda1,
            config_.collision_lambda2, 0.0};
        request.acceleration_half_box = config_.acceleration_half_box;
        request.speed_limit_mps = config_.speed_limit_mps;
        request.speed_cbf_gain = config_.speed_cbf_gain;
        request.plant_speed_facet_count =
            config_.speed_limit_mps>0.0?config_.plant_speed_facet_count:0;
        request.plant_speed_dt_s = config_.dt_s;
        request.require_snapshot_robust_rows = true;
        if (config_.speed_limit_mps > 0.0) {
            for (NodeId id : mobile_ids_) {
                request.plant_speed_snapshot_tubes[id] = {
                    0.0,
                    config_.uncertainty_sigma*std::sqrt(std::max(
                        0.0,detail::maximumVelocityEigenvalue(snapshot,id)))+
                        config_.certified_shadow_single_velocity_support_mps,
                    SnapshotTubeProvenance::CovarianceSigmaDevelopment};
            }
        }
        const BoundaryBlueprint boundary=buildBoundaryBlueprint(
            config_.boundary,searchPolygon(),
            config_.boundary.explicit_flight_polygon);
        if (!boundary.hard_facets.empty()) {
            request.workspace_facets=boundary.hard_facets;
            for (NodeId id : mobile_ids_) {
                request.workspace_snapshot_tubes[id] = {
                    config_.uncertainty_sigma*std::sqrt(std::max(
                        0.0,detail::maximumPositionEigenvalue(snapshot,id)))+
                        config_.certified_shadow_single_position_support_m,
                    config_.uncertainty_sigma*std::sqrt(std::max(
                        0.0,detail::maximumVelocityEigenvalue(snapshot,id))),
                    SnapshotTubeProvenance::CovarianceSigmaDevelopment};
            }
        }
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

    std::vector<Eigen::Vector2d> searchPolygon() const {
        const auto& boundary=swarm_.config.at("world").at("boundary");
        if (!boundary.is_array())
            throw std::invalid_argument("search boundary is not polygonal");
        std::vector<Eigen::Vector2d> points;
        for (const auto& raw : boundary)
            points.emplace_back(
                raw.at(0).get<double>(),raw.at(1).get<double>());
        return points;
    }

    std::vector<BoundarySoftRow> boundarySoftRows(
        const JointEstimateSnapshot& snapshot) const {
        const BoundaryBlueprint blueprint=buildBoundaryBlueprint(
            config_.boundary,searchPolygon(),
            config_.boundary.explicit_flight_polygon);
        if (blueprint.soft_facets.empty()) return {};
        const auto canonical=hardRowRequest(snapshot,supervisor_.topology());
        BoundarySoftRowRequest request;
        request.mobile_ids=mobile_ids_;
        request.states=canonical.states;
        request.facets=blueprint.soft_facets;
        for (NodeId id : mobile_ids_)
            request.snapshot_tubes[id]={
                config_.uncertainty_sigma*std::sqrt(std::max(
                    0.0,detail::maximumPositionEigenvalue(snapshot,id)))+
                    config_.certified_shadow_single_position_support_m,
                config_.uncertainty_sigma*std::sqrt(std::max(
                    0.0,detail::maximumVelocityEigenvalue(snapshot,id)))+
                    config_.certified_shadow_single_velocity_support_mps,
                SnapshotTubeProvenance::CovarianceSigmaDevelopment};
        return buildBoundarySoftRows(request);
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
        std::vector<NodeId> information_references=mobile_ids_;
        for (const auto& [id,position]:fixed_positions_) {
            (void)position;
            information_references.push_back(id);
        }
        const double now_s=swarm_.robots.front()->runtime;
        for (const NodeId owner:mobile_ids_) {
            const Eigen::Vector2d owner_position=
                detail::nodePosition(snapshot,owner);
            for (const NodeId reference:information_references) {
                if (reference==owner) continue;
                const std::string id=UndirectedEdge::canonical(
                    owner,reference).id();
                const auto observed=range_last_observation_s_.find(id);
                const auto quality=range_quality_.find(id);
                const auto variance=range_variance_m2_.find(id);
                if (observed==range_last_observation_s_.end() ||
                    quality==range_quality_.end() ||
                    variance==range_variance_m2_.end() ||
                    !std::isfinite(observed->second) ||
                    !std::isfinite(quality->second) ||
                    !std::isfinite(variance->second) || variance->second<=0.0 ||
                    std::max(0.0,now_s-observed->second)>
                        config_.maximum_range_aoi_s+1.0e-12 ||
                    quality->second+1.0e-12<config_.minimum_range_quality)
                    continue;
                const double distance=(owner_position-
                    detail::nodePosition(snapshot,reference)).norm();
                if (!std::isfinite(distance) || distance<=1.0e-12) continue;
                context.information_edges.emplace_back(reference,owner);
                context.information_range_variances_m2.emplace(
                    id,variance->second);
            }
        }
        std::vector<DirectedEdge> edges = supervisor_.topology();
        edges.insert(edges.end(), additions.begin(), additions.end());
        edges = transition_certifier_detail::canonicalEdges(std::move(edges));
        std::map<std::string, RangeLinkState> links;
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
        context.nominal_controls = nominalControls(supervisor_.mode(), snapshot);
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
        last_accepted_range_batch_audit_.clear();
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
            last_accepted_range_batch_audit_.push_back({
                accepted,master,update.innovation,update.innovation_variance});
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
    BoundarySoftNominalSelector boundary_soft_selector_;
    std::optional<TransitionProposal> pending_proposal_;
    std::optional<TransitionCertificate> pending_certificate_;
    std::size_t union_control_cycles_ = 0;
    bool stage_zero_initialized_ = false;
    std::vector<TransitionCertificate> transition_stack_;
    bool pending_is_retreat_ = false;
    std::string last_certification_reason_;
    std::map<std::string, double> range_last_observation_s_;
    std::map<std::string, double> range_quality_;
    std::map<std::string, double> range_variance_m2_;
    std::vector<AcceptedRangeUpdateAudit> last_accepted_range_batch_audit_;
    std::optional<std::map<NodeId, Eigen::Vector2d>> nominal_override_;
    std::optional<Swarm::CertifiedYawRateBatch> yaw_rate_override_;
    std::mt19937 range_rng_;
    std::uniform_real_distribution<double> range_uniform_{0.0, 1.0};
    std::normal_distribution<double> range_normal_{0.0, 1.0};
    std::size_t range_batch_count_ = 0;
};

}  // namespace gf
