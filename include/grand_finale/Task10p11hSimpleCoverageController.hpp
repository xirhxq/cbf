#pragma once

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/Task10p11gFrozenModel.hpp"
#include "grand_finale/Task10p11hLeaderCoveragePolicy.hpp"
#include "grand_finale/Task10p11hSimpleCoveragePolicy.hpp"
#include "grand_finale/Task10p11tDynamicPairResponsibility.hpp"
#include "grand_finale/Task13UnifiedCoveragePolicy.hpp"
#include "grand_finale/Task14TargetGovernor.hpp"
#include "grand_finale/Task15ForwardCoveragePolicy.hpp"
#include "grand_finale/Task15ReferenceGovernor.hpp"
#include "grand_finale/TargetLiftTransitionPrototype.hpp"

#include <functional>

namespace gf {

enum class SimpleCoveragePhase {
    CoverageSearch
};

struct NaturalSettlingConfig {
    double position_tolerance_m=5.0;
    double speed_tolerance_mps=0.5;
    double yaw_tolerance_rad=5.0*M_PI/180.0;
    std::size_t dwell_cycles=20;
};

struct NaturalSettlingAudit {
    bool snapshot_settled=false;
    double maximum_position_error_m=0.0;
    double maximum_speed_mps=0.0;
    double maximum_yaw_error_rad=0.0;
    double minimum_hard_residual=std::numeric_limits<double>::infinity();
    std::size_t minimum_effective_reference_count=0;
    double minimum_robust_fim=std::numeric_limits<double>::infinity();
    double maximum_posterior=0.0;
    double minimum_aoi_margin_s=std::numeric_limits<double>::infinity();
    bool no_pending_transition=false;
    CurrentReferenceAudit information;
};

struct BoundaryExcursionAudit {
    double maximum_outside_distance_m=0.0;
    double any_outside_duration_s=0.0;
    std::map<NodeId,double> owner_outside_duration_s;
    std::size_t maximum_simultaneous_outside=0;
    double maximum_position_norm_m=0.0;
    Eigen::Vector2d maximum_position=Eigen::Vector2d::Zero();
    std::size_t outside_observer_new_truth_cells=0;
};

struct SimpleCoverageControlStep {
    bool allocation_evaluated=false;
    LeaderCoverageResult allocation;
    bool unified_allocation_evaluated=false;
    Task13UnifiedCoverageResult unified_allocation;
    bool task15_allocation_evaluated=false;
    Task15ForwardCoverageResult task15_allocation;
    GrandFinaleSwarmStep step;
    std::string reason;
    std::map<NodeId,FrontierCell> committed_targets;
    std::map<NodeId,Eigen::Vector2d> applied_target_centers;
    std::size_t target_epoch=0;
    std::size_t consecutive_certification_failures=0;
    SimpleCoveragePhase phase=SimpleCoveragePhase::CoverageSearch;
    std::optional<double> t100_coverage_s;
    NaturalSettlingAudit settling_audit;
    std::size_t settling_dwell_cycles=0;
    bool t100_event_latched=false;
    BoundaryExcursionAudit boundary_excursion;
    std::uint64_t applied_target_digest=0;
    Task10p11ComputeProfile compute_profile;
    bool target_governor_evaluated=false;
    double target_governor_common_fraction=1.0;
    double target_governor_minimum_stopping_margin_m=
        std::numeric_limits<double>::infinity();
    bool target_governor_reselect_required=false;
    std::size_t target_governor_feasibility_evaluations=0;
};

struct SimpleCoverageControllerRestartState {
    std::map<NodeId,FrontierCell> targets;
    std::size_t target_epoch=0;
    std::size_t consecutive_failures=0;
    std::size_t successful_control_cycles=0;
    std::size_t control_boundaries=0;
    SimpleCoveragePhase phase=SimpleCoveragePhase::CoverageSearch;
    std::optional<double> t100_coverage_s;
    std::size_t settling_dwell_cycles=0;
    std::map<NodeId,Eigen::Vector2d> last_nominal_controls;
    BoundaryExcursionAudit boundary_excursion;
};

class Task10p11hSimpleCoverageController {
public:
    using DevelopmentControlOverride = std::function<GrandFinaleSwarmStep(
        const GrandFinaleRuntimeSnapshot&,
        const std::map<NodeId,Eigen::Vector2d>&,
        const std::map<NodeId,double>&)>;

    Task10p11hSimpleCoverageController(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter,
        SimpleCoveragePolicyConfig config={},NaturalSettlingConfig settling={},
        std::vector<LeaderCoverageBranchSpec> branches=
            task10p11hLeaderCoverageSpec())
        : swarm_(swarm),adapter_(adapter),config_(config),settling_(settling),
          branches_(std::move(branches)) {
        if (config_.allocation_epoch_cycles==0 ||
            config_.maximum_certification_failures==0 ||
            !std::isfinite(settling_.position_tolerance_m) ||
            settling_.position_tolerance_m<0.0 ||
            !std::isfinite(settling_.speed_tolerance_mps) ||
            settling_.speed_tolerance_mps<0.0 ||
            !std::isfinite(settling_.yaw_tolerance_rad) ||
            settling_.yaw_tolerance_rad<0.0 || settling_.dwell_cycles==0)
            throw std::invalid_argument("invalid simple coverage controller config");
        freezeDenominatorFromGridWorld();
    }

    SimpleCoverageControlStep advance() {
        SimpleCoverageControlStep result;
        result.t100_event_latched=observeT100Boundary();
        const auto runtime=adapter_.runtimeSnapshot();
        if (runtime.mode!=SupervisorMode::Search) {
            result.step=adapter_.step();
            ++control_boundaries_;
            if (result.step.advanced) {
                const bool t100_event=observeT100Boundary();
                result.t100_event_latched=
                    result.t100_event_latched || t100_event;
                updateBoundaryExcursion(result.step);
                if (t100_coverage_s_.has_value())
                    result.settling_audit=evaluateNaturalSettlingAudit(
                        result.step);
            }
            result.reason=result.step.reason;
            result.committed_targets=targets_;
            result.target_epoch=target_epoch_;
            result.phase=phase_;
            result.t100_coverage_s=t100_coverage_s_;
            result.boundary_excursion=boundary_excursion_;
            return result;
        }

        const auto target_started=std::chrono::steady_clock::now();

        const bool policy_v2=adapter_.config().target_policy_v2;
        const bool policy_v3=adapter_.config().target_policy_v3;
        const bool policy_h2=adapter_.config().target_policy_unified_h2;
        const bool policy_task15=
            adapter_.config().target_policy_task15_forward;
        if (policy_task15) {
            advanceTask15Targets(runtime,result);
            if (result.task15_allocation_evaluated &&
                !result.task15_allocation.valid) {
                result.step.reason=result.reason;
                return result;
            }
        } else if (policy_h2) {
            advanceUnifiedH2Targets(runtime,result);
            if (result.unified_allocation_evaluated &&
                !result.unified_allocation.valid) return result;
        } else if (policy_v2) {
            // Task 13 Phase B0-a: demand field (low-frequency/event-driven
            // recompute) + reachability projection + target-lock contract.
            advanceV2Targets(runtime,result);
        } else if (policy_v3) {
            // Task 13 Phase B0-a v3: CBF2026 leader-CVT design restored
            // (centroid-primary leader targets) with the target-lock
            // contract and low-frequency/event-driven partition recompute.
            advanceV3Targets(runtime,result);
        } else {
        const auto cells=currentUncoveredCells();
        std::set<std::string> uncovered;
        for (const auto& cell : cells) uncovered.insert(cell.id());
        bool target_completed=false;
        bool cvt_target_present=false;
        const bool policy_v6=adapter_.config().target_policy_v6;
        for (const auto& target_entry : targets_) {
            const NodeId owner=target_entry.first;
            const FrontierCell& target=target_entry.second;
            const bool leader=std::any_of(
                branches_.begin(),branches_.end(),[&](const auto& branch) {
                    return branch.leader==owner;
                });
            if (!policy_v6 && !leader) continue;
            if (!denominator_ids_.count(target.id())) {
                cvt_target_present=true;
            } else if (!uncovered.count(target.id())) {
                target_completed=true;
            }
        }
        bool target_invalid=targets_.size()!=runtime.estimate.mobile_ids.size();
        for (const auto& [owner,target] : targets_) {
            (void)owner;
            target_invalid=target_invalid ||
                (target.x_index>=0 && !denominator_ids_.count(target.id()));
        }
        const bool failure_reallocation=
            consecutive_failures_>=config_.maximum_certification_failures;
        const bool logical_tick=
            control_boundaries_%config_.allocation_epoch_cycles==0;
        if (targets_.empty() ||
            (logical_tick && (cvt_target_present || target_completed ||
                              target_invalid)) ||
            failure_reallocation) {
            result.allocation_evaluated=true;
            LeaderCoverageRequest request;
            request.uncovered_cells=cells;
            request.domain_cells=denominator_cells_;
            request.branches=branches_;
            for (std::size_t index=0;
                 index<runtime.estimate.mobile_ids.size();++index)
                request.agents.push_back({runtime.estimate.mobile_ids[index],
                    runtime.estimate.mean.segment<2>(4*index),
                    runtime.estimate.mean.segment<2>(4*index+2)});
            result.allocation=allocateLeaderCoverageTargets(request);
            if (!result.allocation.valid) {
                result.reason=result.allocation.reason;
                return result;
            }
            if (config_.target_projection==
                TargetProjection::LegacySearchPolygonClippingAblation) {
                const GridWorld& grid=swarm_.robots.front()->gridWorld;
                result.allocation=
                    legacyProjectLeaderCoverageTargetsToSearchPolygon(
                        result.allocation,
                        Eigen::Vector2d(grid.xLim.first,grid.yLim.first),
                        Eigen::Vector2d(grid.xLim.second,grid.yLim.second));
            }
            if (policy_v6)
                v6ChaseOverride(result.allocation.targets,cells,runtime);
            targets_=result.allocation.targets;
            ++target_epoch_;
            consecutive_failures_=0;
        }
        }

        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
        std::map<NodeId,Eigen::Vector2d> nominal_targets;
        for (const auto& [owner,target]:targets_)
            nominal_targets[owner]=target.center;
        if (policy_task15 &&
            nominal_targets.size()==runtime.estimate.mobile_ids.size()) {
            Task15ForwardCoverageConfig task15_config;
            task15_config.shortlist_capacity=
                adapter_.config().task15_forward_shortlist_capacity;
            const std::size_t attempt_limit=
                task15ReselectionAttemptLimit(task15_config);
            for (std::size_t attempt=0;attempt<attempt_limit;++attempt) {
                advanceTask15ReferenceGovernor(
                    runtime,nominal_targets,result);
                if (!result.target_governor_reselect_required) break;
                advanceTask15Targets(runtime,result);
                if (!result.task15_allocation.valid) {
                    result.step.reason=result.reason;
                    return result;
                }
                nominal_targets.clear();
                for (const auto& [owner,target]:targets_)
                    nominal_targets[owner]=target.center;
            }
            if (result.target_governor_reselect_required) {
                result.reason="task15_allocator_no_reference_safe_direction";
                result.step.reason=result.reason;
                return result;
            }
            nominal_targets=governed_targets_;
        } else if (adapter_.config().target_homotopy_enabled &&
            nominal_targets.size()==runtime.estimate.mobile_ids.size()) {
            if (governed_targets_.size()!=nominal_targets.size()) {
                governed_targets_.clear();
                for (std::size_t index=0;
                     index<runtime.estimate.mobile_ids.size();++index)
                    governed_targets_[runtime.estimate.mobile_ids[index]]=
                        runtime.estimate.mean.segment<2>(4*index);
            }
            std::map<NodeId,Task14TargetGovernorState> states;
            for (std::size_t index=0;
                 index<runtime.estimate.mobile_ids.size();++index) {
                const NodeId owner=runtime.estimate.mobile_ids[index];
                states[owner]={runtime.estimate.mean.segment<2>(4*index),
                    runtime.estimate.mean.segment<2>(4*index+2)};
            }
            const auto boundary=buildBoundaryBlueprint(
                adapter_.config().boundary,
                adapter_.config().boundary.explicit_flight_polygon,
                adapter_.config().boundary.explicit_flight_polygon);
            const auto governed=task14AdvanceTargetGovernor(
                states,governed_targets_,nominal_targets,
                boundary.hard_facets,{0.1,
                    adapter_.config().
                        target_homotopy_braking_acceleration_mps2,
                    adapter_.config().target_homotopy_rate_gain,
                    adapter_.config().target_homotopy_workspace_guard_m});
            result.target_governor_evaluated=true;
            if (!governed.valid) {
                result.reason=governed.reason;
                return result;
            }
            governed_targets_=governed.targets;
            nominal_targets=governed.targets;
            result.target_governor_common_fraction=governed.common_fraction;
            result.target_governor_minimum_stopping_margin_m=
                governed.minimum_stopping_margin_m;
        }
        result.applied_target_centers=nominal_targets;
        for (std::size_t index=0;
             index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            const Eigen::Vector4d state=runtime.estimate.mean.segment<4>(4*index);
            if (policy_v2) {
                nominal[owner]=v2SpeedTrackingNominal(owner,state,runtime);
                yaw_rates[owner]=0.0;
                continue;
            }
            auto model=task10p11gFrozenModel();
            model.acceleration_half_box_mps2=adapter_.config().acceleration_half_box;
            model.position_gain=adapter_.config().position_gain;
            model.velocity_gain=adapter_.config().velocity_gain;
            model.maximum_yaw_rate_radps=adapter_.config().maximum_yaw_rate_radps;
            const auto found=nominal_targets.find(owner);
            std::optional<Eigen::Vector2d> soft_target=
                found==nominal_targets.end()?std::optional<Eigen::Vector2d>{}:
                    std::optional<Eigen::Vector2d>{found->second};
            const auto soft=task10p11gSoftTask({state.head<2>(),state.tail<2>(),
                currentYaw(owner),soft_target,
                runtime.mode},model);
            nominal[owner]=soft.acceleration;
            yaw_rates[owner]=soft.yaw_rate_radps;
        }
        last_nominal_controls_=nominal;
        result.compute_profile.record(
            Task10p11ComputePhase::GridWorldTarget,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()-target_started).count(),true);
        if (development_control_override_.has_value()) {
            result.step=(*development_control_override_)(
                runtime,nominal,yaw_rates);
        } else if (dynamic_pair_override_.has_value()) {
            const auto pair_started = std::chrono::steady_clock::now();
            std::vector<CanonicalHardRow> rows;
            try {
                rows = adapter_.currentSnapshotHardRows(runtime.topology);
            } catch (...) {
                result.step.reason = "snapshot_robust_hard_row_invalid";
                result.step.dynamic_pair.attempted = true;
                result.step.dynamic_pair.pair_base_id = *dynamic_pair_override_;
                result.step.dynamic_pair.reason = result.step.reason;
            }
            if (result.step.reason.empty()) {
                const auto coordinated = solveTask10p11tDistributedLocalStep(
                    rows, runtime.estimate.mobile_ids, nominal,
                    adapter_.config().acceleration_half_box,
                    *dynamic_pair_override_);
                if (!coordinated.feasible) {
                    result.step.reason = coordinated.reason;
                    result.step.dynamic_pair.attempted = true;
                    result.step.dynamic_pair.pair_base_id =
                        *dynamic_pair_override_;
                    result.step.dynamic_pair.reason = coordinated.reason;
                    result.step.dynamic_pair.first_owner =
                        coordinated.pair.pair.first.owner;
                    result.step.dynamic_pair.second_owner =
                        coordinated.pair.pair.second.owner;
                } else {
                    DynamicPairCertifiedControlRequest request;
                    request.pair_base_id = *dynamic_pair_override_;
                    request.first_owner = coordinated.pair.pair.first.owner;
                    request.second_owner = coordinated.pair.pair.second.owner;
                    request.transfer_interval_lower_mps2 =
                        coordinated.pair.shared_interval.lower;
                    request.transfer_interval_upper_mps2 =
                        coordinated.pair.shared_interval.upper;
                    request.transfer_mps2 =
                        coordinated.pair.selected_transfer_mps2;
                    request.controls = coordinated.controls;
                    result.step = adapter_.stepWithDynamicPairCertifiedControls(
                        request, yaw_rates, runtime.estimator_token,
                        runtime.topology_token);
                }
            }
            result.step.qp_wall_s += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - pair_started).count();
        } else {
            result.step=adapter_.stepWithNominalAndYawRates(nominal,yaw_rates);
        }
        result.compute_profile.merge(result.step.compute_profile);
        if (!result.step.advanced) ++consecutive_failures_;
        else {
            consecutive_failures_=0;
            ++successful_control_cycles_;
        }
        result.reason=result.step.reason;
        result.committed_targets=targets_;
        result.target_epoch=target_epoch_;
        result.consecutive_certification_failures=consecutive_failures_;
        ++control_boundaries_;
        if (result.step.advanced) {
            const bool t100_event=observeT100Boundary();
            result.t100_event_latched=
                result.t100_event_latched || t100_event;
            updateBoundaryExcursion(result.step);
            if (t100_coverage_s_.has_value())
                result.settling_audit=evaluateNaturalSettlingAudit(result.step);
        }
        result.phase=phase_;
        result.t100_coverage_s=t100_coverage_s_;
        result.settling_dwell_cycles=settling_dwell_cycles_;
        result.boundary_excursion=boundary_excursion_;
        return result;
    }

    // Consume one immutable lifted ledger during an already-certified UNION
    // cycle.  The planner-owned raw ledger and target epoch are not mutated.
    SimpleCoverageControlStep advanceWithFrozenTargetLift(
        const FrozenTargetLiftToken& token) {
        SimpleCoverageControlStep result;
        const auto runtime=adapter_.runtimeSnapshot();
        if (!token.valid || runtime.mode!=SupervisorMode::Union ||
            canonicalTargetEdges(runtime.topology)!=token.union_edges ||
            canonicalTargetLedgerDigest([&]() {
                std::map<NodeId,Eigen::Vector2d> raw;
                for (const auto& [owner,target]:targets_)
                    raw[owner]=target.center;
                return raw;
            }())!=token.raw_ledger_digest ||
            canonicalLiftedTargetDigest(
                token.r_plan_m,token.lifted_targets)!=token.lifted_digest) {
            result.reason="frozen_target_lift_stale";
            return result;
        }

        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
        auto model=task10p11gFrozenModel();
        model.acceleration_half_box_mps2=adapter_.config().acceleration_half_box;
        model.position_gain=adapter_.config().position_gain;
        model.velocity_gain=adapter_.config().velocity_gain;
        model.maximum_yaw_rate_radps=adapter_.config().maximum_yaw_rate_radps;
        for (std::size_t index=0;
             index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            const auto target=token.lifted_targets.find(owner);
            if (target==token.lifted_targets.end()) {
                result.reason="lifted_target_missing";
                return result;
            }
            const Eigen::Vector4d state=
                runtime.estimate.mean.segment<4>(4*index);
            const auto soft=task10p11gSoftTask({state.head<2>(),
                state.tail<2>(),currentYaw(owner),target->second,runtime.mode},
                model);
            nominal[owner]=soft.acceleration;
            yaw_rates[owner]=soft.yaw_rate_radps;
        }
        result.step=adapter_.stepWithNominalAndYawRates(nominal,yaw_rates);
        result.reason=result.step.reason;
        result.applied_target_digest=canonicalLiftedTargetDigest(
            token.r_plan_m,token.lifted_targets);
        result.committed_targets=targets_;
        result.target_epoch=target_epoch_;
        ++control_boundaries_;
        if (result.step.advanced) {
            ++successful_control_cycles_;
            const bool t100_event=observeT100Boundary();
            result.t100_event_latched=t100_event;
            updateBoundaryExcursion(result.step);
        } else {
            ++consecutive_failures_;
        }
        result.consecutive_certification_failures=consecutive_failures_;
        result.phase=phase_;
        result.t100_coverage_s=t100_coverage_s_;
        result.boundary_excursion=boundary_excursion_;
        return result;
    }

    void freezeDenominatorFromGridWorld() {
        denominator_ids_.clear();
        denominator_cells_.clear();
        GridWorld& grid=swarm_.robots.front()->gridWorld;
        for (int x=0;x<grid.xNum;++x) for (int y=0;y<grid.yNum;++y) {
            denominator_ids_.insert(std::to_string(x)+":"+std::to_string(y));
            denominator_cells_.push_back({x,y,{
                grid.getCellCenterX(x),grid.getCellCenterY(y)}});
        }
    }

    const std::map<NodeId,FrontierCell>& committedTargets() const {
        return targets_;
    }
    std::size_t targetEpoch() const { return target_epoch_; }
    std::size_t successfulControlCycles() const {
        return successful_control_cycles_;
    }
    SimpleCoveragePhase phase() const { return phase_; }
    const std::optional<double>& t100CoverageS() const {
        return t100_coverage_s_;
    }
    bool settled() const {
        return settling_dwell_cycles_>=settling_.dwell_cycles;
    }
    std::size_t settlingDwellCycles() const { return settling_dwell_cycles_; }
    const std::map<NodeId,Eigen::Vector2d>& lastNominalControls() const {
        return last_nominal_controls_;
    }
    SimpleCoverageControllerRestartState restartState() const {
        if (dynamic_pair_override_.has_value() ||
            development_control_override_.has_value())
            throw std::logic_error(
                "cannot checkpoint during control override call");
        return {targets_,target_epoch_,consecutive_failures_,
            successful_control_cycles_,control_boundaries_,phase_,
            t100_coverage_s_,settling_dwell_cycles_,last_nominal_controls_,
            boundary_excursion_};
    }
    void restoreRestartState(
        const SimpleCoverageControllerRestartState& state) {
        if (dynamic_pair_override_.has_value() ||
            (state.targets.size()!=0 && state.targets.size()!=14) ||
            (state.last_nominal_controls.size()!=0 &&
             state.last_nominal_controls.size()!=14))
            throw std::invalid_argument("invalid coverage-controller restart");
        for (const auto& [owner,target]:state.targets) {
            if (owner<1 || owner>14 ||
                (target.x_index>=0 && !denominator_ids_.count(target.id())) ||
                !target.center.allFinite())
                throw std::invalid_argument("invalid restart target ledger");
        }
        for (const auto& [owner,control]:state.last_nominal_controls)
            if (owner<1 || owner>14 || !control.allFinite())
                throw std::invalid_argument("invalid restart nominal control");
        targets_=state.targets;
        target_epoch_=state.target_epoch;
        consecutive_failures_=state.consecutive_failures;
        successful_control_cycles_=state.successful_control_cycles;
        control_boundaries_=state.control_boundaries;
        phase_=state.phase;
        t100_coverage_s_=state.t100_coverage_s;
        settling_dwell_cycles_=state.settling_dwell_cycles;
        last_nominal_controls_=state.last_nominal_controls;
        boundary_excursion_=state.boundary_excursion;
        unified_h2_retained_.clear();
        unified_h2_active_.clear();
        governed_targets_.clear();
        task15_retained_.clear();
        task15_blocked_task_ids_.clear();
        task15_force_reselect_=false;
        task15_last_update_cycle_=0;
        task15_last_certified_count_=-1;
    }
    SimpleCoverageControlStep advanceWithDynamicPairResponsibility(
        const std::string& pair_base_id) {
        if (pair_base_id.empty())
            throw std::invalid_argument("dynamic pair base id is empty");
        if (dynamic_pair_override_.has_value())
            throw std::logic_error("dynamic pair override already active");
        dynamic_pair_override_=pair_base_id;
        try {
            auto result=advance();
            dynamic_pair_override_.reset();
            return result;
        } catch (...) {
            dynamic_pair_override_.reset();
            throw;
        }
    }

    SimpleCoverageControlStep advanceWithDevelopmentControlOverride(
        DevelopmentControlOverride control_override) {
        if (!control_override)
            throw std::invalid_argument("development control override empty");
        if (development_control_override_.has_value())
            throw std::logic_error("development control override active");
        development_control_override_=std::move(control_override);
        try {
            auto result=advance();
            development_control_override_.reset();
            return result;
        } catch (...) {
            development_control_override_.reset();
            throw;
        }
    }

private:
    // ---- Task 15: forward endpoint formation + certified union utility ----
    void advanceTask15Targets(
        const GrandFinaleRuntimeSnapshot& runtime,
        SimpleCoverageControlStep& result) {
        if (task15_retained_.empty() && targets_.size()==14) {
            for (const auto& squad:task13UnifiedCoverageSquads()) {
                const auto first=targets_.find(squad.members.front());
                if (first==targets_.end()) continue;
                Task15ForwardCoverageCandidate retained;
                retained.squad=squad.name;
                retained.task=first->second;
                retained.endpoint=first->second;
                retained.level_a=true;
                for (NodeId member:squad.members) {
                    const auto target=targets_.find(member);
                    if (target==targets_.end() ||
                        target->second.id()!=retained.task.id()) {
                        retained.targets.clear();
                        break;
                    }
                    retained.targets[member]=target->second.center;
                    retained.target_ids[member]=retained.task.id();
                }
                if (retained.targets.size()==7)
                    task15_retained_[squad.name]=std::move(retained);
            }
        }
        const int certified_count=adapter_.coverage().certifiedCoveredCount();
        if (task15_last_certified_count_>=0 &&
            certified_count!=task15_last_certified_count_)
            task15_blocked_task_ids_.clear();
        bool task_completed=false;
        for (const auto& [squad,candidate]:task15_retained_) {
            (void)squad;
            if (!certifiedCellUncovered(candidate.task)) task_completed=true;
        }
        const bool periodic=control_boundaries_>=task15_last_update_cycle_+
            adapter_.config().task15_forward_update_period_cycles;
        const bool event=targets_.empty() || task15_retained_.size()!=2 ||
            task15_force_reselect_ || task_completed || periodic;
        if (!event) return;

        Task15ForwardCoverageRequest request;
        request.domain_cells=denominator_cells_;
        request.fixed_positions=runtime.estimate.fixed_positions;
        request.retained=task15_retained_;
        request.config.shortlist_capacity=
            adapter_.config().task15_forward_shortlist_capacity;
        request.config.endpoint_standoff_m=
            adapter_.config().task15_forward_endpoint_standoff_m;
        request.config.sensor_inner_radius_m=
            adapter_.config().coverage_inner_radius_m;
        request.config.sensor_outer_radius_m=adapter_.config().sensor_radius_m;
        request.config.sensor_half_angle_rad=
            adapter_.config().coverage_half_angle_rad;
        request.config.reference_limit_m=
            adapter_.config().reference_distance_m;
        request.config.target_separation_m=
            adapter_.config().collision_distance_m;
        request.config.nominal_speed_mps=adapter_.config().speed_limit_mps;
        request.config.braking_acceleration_mps2=
            adapter_.config().acceleration_half_box;
        for (const auto& cell:currentCertifiedUncoveredCells())
            if (!task15_blocked_task_ids_.count(cell.id()))
                request.uncovered_cells.push_back(cell);
        for (std::size_t index=0;
             index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            const double error=std::max(
                adapter_.config().certified_shadow_single_position_support_m,
                std::max(adapter_.config().certified_error_bound_m,
                    adapter_.config().uncertainty_sigma*std::sqrt(std::max(
                        0.0,detail::maximumPositionEigenvalue(
                            runtime.estimate,owner)))));
            request.agents.push_back({owner,
                runtime.estimate.mean.segment<2>(4*index),
                runtime.estimate.mean.segment<2>(4*index+2),
                currentYaw(owner),error});
        }
        result.task15_allocation_evaluated=true;
        result.task15_allocation=allocateTask15ForwardCoverage(
            std::move(request));
        if (!result.task15_allocation.valid) {
            result.reason=result.task15_allocation.reason;
            return;
        }
        targets_=result.task15_allocation.targets;
        task15_retained_.clear();
        for (const auto& [squad,assignment]:
             result.task15_allocation.assignments)
            task15_retained_[squad]=assignment.candidate;
        task15_force_reselect_=false;
        task15_last_update_cycle_=control_boundaries_;
        task15_last_certified_count_=certified_count;
        ++target_epoch_;
        consecutive_failures_=0;
    }

    void advanceTask15ReferenceGovernor(
        const GrandFinaleRuntimeSnapshot& runtime,
        const std::map<NodeId,Eigen::Vector2d>& nominal_targets,
        SimpleCoverageControlStep& result) {
        std::map<NodeId,Eigen::Vector2d> current_position_ledger;
        for (std::size_t index=0;
             index<runtime.estimate.mobile_ids.size();++index)
            current_position_ledger[runtime.estimate.mobile_ids[index]]=
                runtime.estimate.mean.segment<2>(4*index);
        const auto feasible=[&](
            const std::map<NodeId,Eigen::Vector2d>& candidate,double) {
            if (!(task15TargetLedgerMinimumSeparation(candidate,
                    runtime.estimate.fixed_positions)>
                  adapter_.config().collision_distance_m+1e-9))
                return false;
            std::map<NodeId,Eigen::Vector2d> task_nominals;
            auto model=task10p11gFrozenModel();
            model.acceleration_half_box_mps2=
                adapter_.config().acceleration_half_box;
            model.position_gain=adapter_.config().position_gain;
            model.velocity_gain=adapter_.config().velocity_gain;
            model.maximum_yaw_rate_radps=
                adapter_.config().maximum_yaw_rate_radps;
            for (std::size_t index=0;
                 index<runtime.estimate.mobile_ids.size();++index) {
                const NodeId owner=runtime.estimate.mobile_ids[index];
                const Eigen::Vector4d state=
                    runtime.estimate.mean.segment<4>(4*index);
                task_nominals[owner]=task10p11gSoftTask({state.head<2>(),
                    state.tail<2>(),currentYaw(owner),candidate.at(owner),
                    runtime.mode},model).acceleration;
            }
            const auto& config=adapter_.config();
            const CanonicalGammaFeedbackConfig feedback_config{
                config.acceleration_half_box,
                config.gamma_feedback_homotopy_segments,
                config.gamma_feedback_selection,
                config.predictive_gamma_tau_mps2,
                config.gamma_feedback_tolerance,
                config.tau_margin_gate_enabled,1.0,nullptr,
                config.tau_analytic_first_order,
                config.nominal_throttle_enabled,
                config.throttle_gamma_th_mps2,
                config.throttle_gamma_floor_mps2,
                config.nominal_speed_saturation_mps,
                config.throttle_v2_enabled,
                config.throttle_v2_endpoint_family};
            CanonicalGammaFeedbackEvaluationContext context;
            const auto batch=evaluateCanonicalGammaFeedbackBatch(
                runtime.estimate,task_nominals,feedback_config,config.dt_s,
                config.estimator_acceleration_variance,
                [&](const JointEstimateSnapshot& snapshot) {
                    return adapter_.snapshotHardRowRequest(
                        snapshot,runtime.topology);
                },context);
            if (!batch.valid) return false;
            std::map<NodeId,Eigen::Vector2d> projected_controls;
            for (NodeId owner:runtime.estimate.mobile_ids) {
                const auto stage=batch.stages.find(owner);
                const auto selected=batch.selections.find(owner);
                if (stage==batch.stages.end() ||
                    selected==batch.selections.end() ||
                    stage->second.current_gamma<-
                        config.gamma_feedback_tolerance ||
                    selected->second.selected_predicted_gamma<-
                        config.gamma_feedback_tolerance) return false;
                const auto qp=task15_governor_qp_.solve({
                    config.solver_profile,owner,runtime.estimator_token,
                    runtime.topology_token,runtime.mode,
                    batch.selected_controls.at(owner),
                    config.acceleration_half_box,batch.current_rows,
                    config.residual_tolerance,
                    config.speed_initial_set_truth_gate});
                if (!controlMayBeApplied(qp,runtime.estimator_token,
                        runtime.topology_token,runtime.mode) ||
                    qp.exact_oracle_error>config.qp_oracle_tolerance)
                    return false;
                projected_controls[owner]=qp.control;
            }
            const auto successor=predictNoMeasurementSnapshot(
                runtime.estimate,projected_controls,config.dt_s,
                config.estimator_acceleration_variance);
            for (std::size_t index=0;
                 index<successor.mobile_ids.size();++index)
                if (successor.mean.segment<2>(4*index+2).norm()>
                    config.speed_limit_mps+1e-12) return false;
            const auto successor_rows=buildCanonicalHardRows(
                adapter_.snapshotHardRowRequest(successor,runtime.topology));
            for (NodeId owner:runtime.estimate.mobile_ids) {
                const auto gamma=solveCanonicalGammaStar(
                    successor_rows,owner,config.acceleration_half_box);
                if (!gamma.valid || gamma.gamma<-
                    config.gamma_feedback_tolerance) return false;
            }
            return true;
        };
        const auto governed=task15AdvanceReferenceGovernor(
            current_position_ledger,nominal_targets,feasible);
        result.target_governor_evaluated=true;
        result.target_governor_common_fraction=governed.common_fraction;
        result.target_governor_reselect_required=governed.reselect_required;
        result.target_governor_feasibility_evaluations=
            governed.feasibility_evaluations;
        if (governed.valid) {
            governed_targets_=governed.targets;
            return;
        }
        task15_force_reselect_=true;
        for (const auto& [squad,candidate]:task15_retained_) {
            (void)squad;
            task15_blocked_task_ids_.insert(candidate.task.id());
        }
        // The caller consumes this event in the same sample and exhausts the
        // bounded real-task shortlist before failing closed.
    }

    // ---- Task 13 unified H2: certified-event-driven global pool -------
    void advanceUnifiedH2Targets(
        const GrandFinaleRuntimeSnapshot& runtime,
        SimpleCoverageControlStep& result) {
        if (unified_h2_retained_.empty() && !targets_.empty())
            reconstructUnifiedH2Retained(runtime);
        bool event=targets_.empty() || unified_h2_retained_.size()!=2 ||
            unified_h2_active_.size()!=2;
        bool any_active=false;
        for (const auto& [squad,witness]:unified_h2_retained_) {
            const bool active=unified_h2_active_[squad];
            any_active=any_active||active;
            if (active&&!certifiedCellUncovered(witness.cell)) event=true;
        }
        if (!any_active&&!adapter_.coverage().reachedCertifiedT100())
            event=true;
        if (!event) return;

        Task13UnifiedCoverageRequest request;
        request.uncovered_cells=currentCertifiedUncoveredCells();
        request.fixed_positions=runtime.estimate.fixed_positions;
        request.retained=unified_h2_retained_;
        request.config.minimum_half_width_m=
            adapter_.config().unified_h2_minimum_half_width_m;
        request.config.fan_ratio=adapter_.config().unified_h2_fan_ratio;
        request.config.reference_limit_m=adapter_.config().reference_distance_m;
        request.config.separation_limit_m=adapter_.config().collision_distance_m;
        request.config.shortlist_per_squad=
            adapter_.config().unified_h2_shortlist_per_squad;
        request.config.certified_service_standoff_m=
            adapter_.config().unified_h2_service_standoff_m;
        request.config.cbf2026_wide_virtual_formation=
            adapter_.config().unified_h2_cbf2026_wide_virtual_formation;
        request.config.compute_nominal_fim_proxy=false;
        for (std::size_t index=0;
             index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            request.agents.push_back({owner,
                runtime.estimate.mean.segment<2>(4*index),currentYaw(owner)});
        }
        result.unified_allocation_evaluated=true;
        result.unified_allocation=allocateTask13UnifiedCoverage(
            std::move(request));
        if (!result.unified_allocation.valid) {
            result.reason=result.unified_allocation.reason;
            return;
        }
        targets_=result.unified_allocation.targets;
        unified_h2_retained_.clear();
        unified_h2_active_.clear();
        for (const auto& [squad,assignment]:
             result.unified_allocation.assignments) {
            unified_h2_retained_[squad]=assignment.witness;
            unified_h2_active_[squad]=assignment.active;
        }
        ++target_epoch_;
        consecutive_failures_=0;
    }

    void reconstructUnifiedH2Retained(
        const GrandFinaleRuntimeSnapshot& runtime) {
        if (targets_.size()!=14) return;
        const auto squads=task13UnifiedCoverageSquads();
        Task13UnifiedCoverageConfig config;
        config.minimum_half_width_m=
            adapter_.config().unified_h2_minimum_half_width_m;
        config.fan_ratio=adapter_.config().unified_h2_fan_ratio;
        config.reference_limit_m=adapter_.config().reference_distance_m;
        config.separation_limit_m=adapter_.config().collision_distance_m;
        config.certified_service_standoff_m=
            adapter_.config().unified_h2_service_standoff_m;
        config.cbf2026_wide_virtual_formation=
            adapter_.config().unified_h2_cbf2026_wide_virtual_formation;
        std::map<std::string,Task13UnifiedCoverageWitness> rebuilt;
        for (const auto& squad:squads) {
            const auto first=targets_.find(squad.members.front());
            if (first==targets_.end()) return;
            const auto cell_it=std::find_if(denominator_cells_.begin(),
                denominator_cells_.end(),[&](const FrontierCell& value) {
                    return value.id()==first->second.id();
                });
            if (cell_it==denominator_cells_.end()) return;
            bool found=false;
            for (NodeId member:squad.members) {
                const auto candidate=task13UnifiedWitness(
                    squad,member,*cell_it,runtime.estimate.fixed_positions,
                    config);
                if (!candidate.has_value()) continue;
                bool equal=true;
                for (NodeId owner:squad.members) {
                    const auto target=targets_.find(owner);
                    equal=equal&&target!=targets_.end()&&
                        target->second.id()==cell_it->id()&&
                        (target->second.center-
                         candidate->targets.at(owner)).norm()<=1e-8;
                }
                if (equal) {
                    rebuilt[squad.name]=*candidate;
                    found=true;
                    break;
                }
            }
            if (!found) return;
        }
        unified_h2_retained_=std::move(rebuilt);
        unified_h2_active_.clear();
        for (const auto& [squad,witness]:unified_h2_retained_)
            unified_h2_active_[squad]=certifiedCellUncovered(witness.cell);
    }

    // ---- Task 13 Phase B0-a v6: per-drone chase-cell targeting --------
    // CBF2026 restoration: EVERY drone (leader and follower) targets the
    // nearest uncovered cell within its neighborhood radius around its
    // ladder position, restricted to its branch share (the leader-CVT
    // two-share partition is unchanged).  Covered targets auto-switch at
    // the next allocation epoch via the all-owner completion check.
    void v6ChaseOverride(
        std::map<NodeId,FrontierCell>& allocation_targets,
        const std::vector<FrontierCell>& uncovered_cells,
        const GrandFinaleRuntimeSnapshot& runtime) {
        const auto& config=adapter_.config();
        const double radius=config.v6_neighborhood_radius_m;
        std::map<NodeId,Eigen::Vector2d> leader_positions;
        for (const auto& branch:branches_)
            leader_positions[branch.leader]=
                v2Position(runtime,branch.leader);
        for (auto& [owner,ladder_point] : allocation_targets) {
            const auto branch_it=std::find_if(
                branches_.begin(),branches_.end(),
                [&](const auto& branch) {
                    return std::find(branch.members.begin(),
                        branch.members.end(),owner)!=branch.members.end();
                });
            if (branch_it==branches_.end()) continue;
            const NodeId share_leader=branch_it->leader;
            const FrontierCell* best=nullptr;
            double best_distance=std::numeric_limits<double>::infinity();
            for (const auto& cell : uncovered_cells) {
                NodeId nearest=share_leader;
                double nearest_distance=std::numeric_limits<
                    double>::infinity();
                for (const auto& [leader,position]:leader_positions) {
                    const double d=(cell.center-position).norm();
                    if (d<nearest_distance) {nearest_distance=d;nearest=leader;}
                }
                if (nearest!=share_leader) continue;
                const double distance=
                    (cell.center-ladder_point.center).norm();
                if (distance<=radius && distance<best_distance) {
                    best_distance=distance;
                    best=&cell;
                }
            }
            if (best!=nullptr)
                allocation_targets[owner]=*best;
            // Neighborhood exhausted: the ladder point remains the
            // structure target; the next epoch re-chases as coverage
            // grows.  No synthetic targets are emitted under v6.
        }
    }

    // ---- Task 13 Phase B0-a v3: leader-CVT centroid-primary targeting
    // with the lock contract and low-frequency/event-driven recompute.
    void advanceV3Targets(
        const GrandFinaleRuntimeSnapshot& runtime,
        SimpleCoverageControlStep& result) {
        const auto& config=adapter_.config();
        const double now=runtime.runtime_s;
        bool event=targets_.empty();
        for (NodeId owner:runtime.estimate.mobile_ids) {
            const auto found=targets_.find(owner);
            if (found==targets_.end()) { event=true; continue; }
            const double distance=
                (v2Position(runtime,owner)-found->second.center).norm();
            if (distance<=config.target_lock_epsilon_m) { event=true; break; }
            const std::size_t dwell=++v2_dwell_cycles_[owner];
            if (dwell==1) v2_dwell_start_distance_[owner]=distance;
            else if (dwell>=config.target_lock_dwell_cycles) {
                const double progress=
                    v2_dwell_start_distance_[owner]-distance;
                if (progress>=config.target_lock_progress_epsilon_m) {
                    v2_dwell_cycles_[owner]=0;
                    v2_dwell_start_distance_[owner]=distance;
                } else {
                    event=true;
                    break;
                }
            }
        }
        const bool interval=now-last_demand_recompute_s_>=
            config.demand_recompute_interval_s;
        if (!event && !interval) {
            result.target_epoch=target_epoch_;
            return;
        }
        // Low-frequency/event-driven partition recompute (runs outside
        // lock periods only).
        const auto cells=currentUncoveredCells();
        LeaderCoverageRequest request;
        request.uncovered_cells=cells;
        request.domain_cells=denominator_cells_;
        request.branches=branches_;
        request.leader_centroid_primary=true;
        if (config.leader_reachability_filter) {
            for (const auto& branch:branches_) {
                std::vector<std::pair<Eigen::Vector2d,double>> disks;
                std::set<NodeId> seen;
                for (NodeId member:branch.members)
                    for (const auto& edge:runtime.topology)
                        if (edge.owner==member &&
                            seen.insert(edge.reference).second)
                            disks.emplace_back(
                                v2Position(runtime,edge.reference),
                                v2ReachableRadius(edge.reference,runtime));
                request.leader_reference_disks[branch.leader]=
                    std::move(disks);
            }
            request.leader_reachability_filter=true;
        } else {
            request.leader_tie_break_tolerance_m=
                config.leader_tie_break_tolerance_m;
        }
        for (std::size_t index=0;
             index<runtime.estimate.mobile_ids.size();++index)
            request.agents.push_back({runtime.estimate.mobile_ids[index],
                runtime.estimate.mean.segment<2>(4*index),
                runtime.estimate.mean.segment<2>(4*index+2)});
        result.allocation_evaluated=true;
        result.allocation=allocateLeaderCoverageTargets(request);
        if (!result.allocation.valid) {
            result.reason=result.allocation.reason;
            return;
        }
        targets_=result.allocation.targets;
        ++target_epoch_;
        last_demand_recompute_s_=now;
        consecutive_failures_=0;
        v2_dwell_cycles_.clear();
        v2_dwell_start_distance_.clear();
        result.target_epoch=target_epoch_;
    }

    // ---- Task 13 Phase B0-a: target policy v2 -------------------------
    void advanceV2Targets(
        const GrandFinaleRuntimeSnapshot& runtime,
        SimpleCoverageControlStep& result) {
        const auto& config=adapter_.config();
        const double now=runtime.runtime_s;
        const bool interval=now-last_demand_recompute_s_>=
            config.demand_recompute_interval_s;
        if (targets_.empty()||interval||v2_demand_cells_.empty()) {
            v2_demand_cells_=currentUncoveredCells();
            last_demand_recompute_s_=now;
            for (NodeId owner:runtime.estimate.mobile_ids)
                if (!targets_.count(owner))
                    v2RecomputeOwnerDemand(owner,runtime);
            ++target_epoch_;
        }
        for (NodeId owner:runtime.estimate.mobile_ids) {
            const auto found=targets_.find(owner);
            if (found==targets_.end()) {
                v2RecomputeOwnerDemand(owner,runtime);
                continue;
            }
            const Eigen::Vector2d position=v2Position(runtime,owner);
            const double distance=(position-found->second.center).norm();
            const bool arrived=distance<=config.target_lock_epsilon_m;
            const bool covered=std::none_of(
                v2_demand_cells_.begin(),v2_demand_cells_.end(),
                [&](const FrontierCell& cell){
                    return cell.id()==found->second.id();});
            const std::size_t dwell=++v2_dwell_cycles_[owner];
            if (dwell==1) v2_dwell_start_distance_[owner]=distance;
            bool stalled=false;
            if (dwell>=config.target_lock_dwell_cycles) {
                const double progress=
                    v2_dwell_start_distance_[owner]-distance;
                if (progress>=config.target_lock_progress_epsilon_m) {
                    v2_dwell_cycles_[owner]=0;
                    v2_dwell_start_distance_[owner]=distance;
                } else {
                    stalled=true;
                }
            }
            if (arrived||covered||stalled) {
                v2RecomputeOwnerDemand(owner,runtime);
                continue;
            }
            // Out-of-feasible escape: an empty projected feasible set
            // triggers an event-driven demand recompute (B1 will reuse
            // this signal as the topology-health escape trigger).
            if (!v2Project(owner,found->second.center,runtime).has_value()) {
                ++v2_escape_count_;
                v2RecomputeOwnerDemand(owner,runtime);
            }
        }
        result.target_epoch=target_epoch_;
    }

    void v2RecomputeOwnerDemand(
        NodeId owner,const GrandFinaleRuntimeSnapshot& runtime) {
        if (v2_demand_cells_.empty()) return;
        const Eigen::Vector2d position=v2Position(runtime,owner);
        Eigen::Vector2d sum=Eigen::Vector2d::Zero();
        std::size_t count=0;
        for (const auto& cell:v2_demand_cells_) {
            NodeId nearest=owner;
            double best=std::numeric_limits<double>::infinity();
            for (NodeId other:runtime.estimate.mobile_ids) {
                const double d=(cell.center-v2Position(runtime,other)).norm();
                if (d<best) {best=d;nearest=other;}
            }
            if (nearest==owner) {sum+=cell.center;++count;}
        }
        if (count==0) return;
        targets_[owner]=FrontierCell{-1,-1,sum/count};
        v2_dwell_cycles_[owner]=0;
        v2_dwell_start_distance_[owner]=
            (position-targets_[owner].center).norm();
        ++target_epoch_;
    }

    std::optional<Eigen::Vector2d> v2Project(
        NodeId owner,const Eigen::Vector2d& demand,
        const GrandFinaleRuntimeSnapshot& runtime) const {
        const auto& config=adapter_.config();
        std::vector<std::pair<Eigen::Vector2d,double>> disks;
        disks.emplace_back(v2Position(runtime,owner),
            v2ReachableRadius(owner,runtime));
        for (const auto& edge:runtime.topology)
            if (edge.owner==owner)
                disks.emplace_back(v2Position(runtime,edge.reference),
                    v2ReachableRadius(edge.reference,runtime));
        Eigen::Vector2d q=demand;
        for (int pass=0;pass<config.projection_passes;++pass) {
            bool feasible=true;
            for (const auto& [center,radius]:disks) {
                const Eigen::Vector2d delta=q-center;
                const double d=delta.norm();
                if (d>radius) {
                    feasible=false;
                    q=d>1e-9?center+delta*(radius/d):center;
                }
            }
            if (feasible) return q;
        }
        for (const auto& [center,radius]:disks)
            if ((q-center).norm()>radius+1e-9) return std::nullopt;
        return q;
    }

    double v2ReachableRadius(
        NodeId node,const GrandFinaleRuntimeSnapshot& runtime) const {
        const auto& config=adapter_.config();
        const Eigen::Index off=4*static_cast<Eigen::Index>(
            std::distance(runtime.estimate.mobile_ids.begin(),
                std::find(runtime.estimate.mobile_ids.begin(),
                    runtime.estimate.mobile_ids.end(),node)));
        const double speed=runtime.estimate.mean.segment<2>(off+2).norm();
        const double tube=config.uncertainty_sigma*std::sqrt(std::max(
            0.0,detail::maximumPositionEigenvalue(runtime.estimate,node)))+
            config.certified_shadow_single_position_support_m;
        return config.reference_distance_m-
            speed*speed/(2.0*config.acceleration_half_box)-tube-
            config.reachability_hysteresis_m;
    }

    Eigen::Vector2d v2SpeedTrackingNominal(
        NodeId owner,const Eigen::Vector4d& state,
        const GrandFinaleRuntimeSnapshot& runtime) const {
        const auto& config=adapter_.config();
        const auto found=targets_.find(owner);
        if (found==targets_.end()) return Eigen::Vector2d::Zero();
        const auto projected=v2Project(owner,found->second.center,runtime);
        const Eigen::Vector2d aim=projected.value_or(found->second.center);
        const Eigen::Vector2d delta=aim-state.head<2>();
        const double distance=delta.norm();
        const double cap=config.speed_row_nominal
            ?config.speed_row_nominal_limit_mps:config.speed_limit_mps;
        const double desired_speed=
            cap*distance/(distance+config.speed_tracking_blend_m);
        const Eigen::Vector2d v_des=distance>1e-9?
            delta*(desired_speed/distance):
            Eigen::Vector2d(Eigen::Vector2d::Zero());
        Eigen::Vector2d acceleration=
            config.speed_tracking_gain*(v_des-state.tail<2>());
        const double bound=config.acceleration_half_box;
        for (int axis=0;axis<2;++axis)
            acceleration(axis)=std::clamp(acceleration(axis),-bound,bound);
        return acceleration;
    }

    static Eigen::Vector2d v2Position(
        const GrandFinaleRuntimeSnapshot& runtime,NodeId node) {
        const Eigen::Index off=4*static_cast<Eigen::Index>(
            std::distance(runtime.estimate.mobile_ids.begin(),
                std::find(runtime.estimate.mobile_ids.begin(),
                    runtime.estimate.mobile_ids.end(),node)));
        return runtime.estimate.mean.segment<2>(off);
    }

    bool observeT100Boundary() {
        const bool complete=(adapter_.config().target_policy_unified_h2 ||
            adapter_.config().target_policy_task15_forward)
            ?adapter_.coverage().reachedCertifiedT100()
            :adapter_.coverage().truthFraction()>=1.0-1e-12;
        if (!t100_coverage_s_.has_value() && complete) {
            t100_coverage_s_=swarm_.robots.front()->runtime;
            settling_dwell_cycles_=0;
            return true;
        }
        return false;
    }

    void updateBoundaryExcursion(const GrandFinaleSwarmStep& step) {
        const auto polygon=adapter_.searchPolygonVertices();
        std::size_t simultaneous=0;
        for (const auto& robot : swarm_.robots) {
            const Point raw=robot->model->xy();
            const Eigen::Vector2d position(raw.x,raw.y);
            const double outside=distanceOutsidePolygon(position,polygon);
            if (outside>1e-12) {
                ++simultaneous;
                boundary_excursion_.owner_outside_duration_s[robot->id]+=
                    adapter_.config().dt_s;
                boundary_excursion_.maximum_outside_distance_m=std::max(
                    boundary_excursion_.maximum_outside_distance_m,outside);
            }
            if (position.norm()>boundary_excursion_.maximum_position_norm_m) {
                boundary_excursion_.maximum_position_norm_m=position.norm();
                boundary_excursion_.maximum_position=position;
            }
        }
        if (simultaneous>0)
            boundary_excursion_.any_outside_duration_s+=adapter_.config().dt_s;
        boundary_excursion_.maximum_simultaneous_outside=std::max(
            boundary_excursion_.maximum_simultaneous_outside,simultaneous);
        boundary_excursion_.outside_observer_new_truth_cells+=
            step.outside_observer_new_truth_cells;
    }

    NaturalSettlingAudit evaluateNaturalSettlingAudit(
        const GrandFinaleSwarmStep& step) {
        NaturalSettlingAudit audit;
        audit.minimum_hard_residual=step.minimum_hard_residual;
        const auto runtime=adapter_.runtimeSnapshot();
        for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            const Eigen::Vector4d state=runtime.estimate.mean.segment<4>(4*index);
            const auto target=targets_.find(owner);
            if (target==targets_.end()) {
                audit.maximum_position_error_m=
                    std::numeric_limits<double>::infinity();
                audit.maximum_yaw_error_rad=
                    std::numeric_limits<double>::infinity();
                continue;
            }
            const Eigen::Vector2d displacement=target->second.center-state.head<2>();
            audit.maximum_position_error_m=std::max(
                audit.maximum_position_error_m,displacement.norm());
            audit.maximum_speed_mps=std::max(
                audit.maximum_speed_mps,state.tail<2>().norm());
            const double yaw_error=
                displacement.norm()<=settling_.position_tolerance_m?0.0:
                std::abs(wrapYawRad(std::atan2(
                    displacement.y(),displacement.x())-currentYaw(owner)));
            audit.maximum_yaw_error_rad=std::max(
                audit.maximum_yaw_error_rad,yaw_error);
        }
        audit.information=adapter_.currentReferenceAudit();
        audit.minimum_effective_reference_count=
            audit.information.minimum_effective_reference_count;
        audit.minimum_robust_fim=
            audit.information.minimum_robust_fim_cone_lower_bound;
        audit.maximum_posterior=audit.information.maximum_posterior_eigenvalue;
        audit.minimum_aoi_margin_s=audit.information.minimum_range_aoi_margin_s;
        audit.no_pending_transition=
            runtime.mode==SupervisorMode::Search &&
            !runtime.supervisor_transition_pending &&
            !runtime.adapter_transition_pending;
        audit.snapshot_settled=
            audit.maximum_position_error_m<=settling_.position_tolerance_m &&
            audit.maximum_speed_mps<=settling_.speed_tolerance_mps &&
            audit.maximum_yaw_error_rad<=settling_.yaw_tolerance_rad &&
            audit.minimum_hard_residual>=-adapter_.config().residual_tolerance &&
            audit.minimum_effective_reference_count>=2 &&
            audit.minimum_robust_fim>=1e-6 &&
            audit.maximum_posterior<=
                adapter_.config().maximum_posterior_eigenvalue_m2 &&
            audit.minimum_aoi_margin_s>=-1e-12 &&
            audit.no_pending_transition;
        settling_dwell_cycles_=audit.snapshot_settled
            ? settling_dwell_cycles_+1 : 0;
        return audit;
    }

    std::vector<FrontierCell> currentUncoveredCells() const {
        std::vector<FrontierCell> result;
        for (const auto& cell :
             swarm_.robots.front()->gridWorld.getUnexploredCellCenters())
            result.push_back({cell.x_index,cell.y_index,
                {cell.center.x,cell.center.y}});
        return result;
    }
    std::vector<FrontierCell> currentCertifiedUncoveredCells() const {
        std::vector<FrontierCell> result;
        const GridWorld& certified=adapter_.coverage().certifiedGrid();
        for (const auto& cell:denominator_cells_) {
            const std::size_t index=static_cast<std::size_t>(
                certified.getIndex(cell.x_index,cell.y_index));
            if (certified.valid[index]&&!certified.vis[index])
                result.push_back(cell);
        }
        return result;
    }
    bool certifiedCellUncovered(const FrontierCell& cell) const {
        if (cell.x_index<0||cell.y_index<0) return false;
        const GridWorld& certified=adapter_.coverage().certifiedGrid();
        const std::size_t index=static_cast<std::size_t>(
            certified.getIndex(cell.x_index,cell.y_index));
        return index<certified.vis.size()&&certified.valid[index]&&
            !certified.vis[index];
    }
    double currentYaw(NodeId owner) const {
        for (const auto& robot : swarm_.robots)
            if (robot->id==owner)
                return robot->model->getStateVariable("yawRad");
        throw std::invalid_argument("missing yaw state");
    }

    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    SimpleCoveragePolicyConfig config_;
    NaturalSettlingConfig settling_;
    std::vector<LeaderCoverageBranchSpec> branches_;
    std::set<std::string> denominator_ids_;
    std::vector<FrontierCell> denominator_cells_;
    std::map<NodeId,FrontierCell> targets_;
    std::map<NodeId,Eigen::Vector2d> governed_targets_;
    std::size_t target_epoch_=0;
    std::size_t consecutive_failures_=0;
    std::size_t successful_control_cycles_=0;
    std::size_t control_boundaries_=0;
    SimpleCoveragePhase phase_=SimpleCoveragePhase::CoverageSearch;
    std::optional<double> t100_coverage_s_;
    std::size_t settling_dwell_cycles_=0;
    std::map<NodeId,Eigen::Vector2d> last_nominal_controls_;
    std::map<std::string,Task13UnifiedCoverageWitness> unified_h2_retained_;
    std::map<std::string,bool> unified_h2_active_;
    std::map<std::string,Task15ForwardCoverageCandidate> task15_retained_;
    std::set<std::string> task15_blocked_task_ids_;
    bool task15_force_reselect_=false;
    std::size_t task15_last_update_cycle_=0;
    int task15_last_certified_count_=-1;
    CanonicalHocbfQpController task15_governor_qp_;
    // Task 13 Phase B0-a (target policy v2) state.
    double last_demand_recompute_s_=-1.0e9;
    std::vector<FrontierCell> v2_demand_cells_;
    std::map<NodeId,std::size_t> v2_dwell_cycles_;
    std::map<NodeId,double> v2_dwell_start_distance_;
    std::size_t v2_escape_count_=0;
    std::optional<std::string> dynamic_pair_override_;
    std::optional<DevelopmentControlOverride> development_control_override_;
    BoundaryExcursionAudit boundary_excursion_;
};

}  // namespace gf
