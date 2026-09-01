#pragma once

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/Task10p11gFrozenModel.hpp"
#include "grand_finale/Task10p11hLeaderCoveragePolicy.hpp"
#include "grand_finale/Task10p11hSimpleCoveragePolicy.hpp"
#include "grand_finale/Task10p11tDynamicPairResponsibility.hpp"
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
    GrandFinaleSwarmStep step;
    std::string reason;
    std::map<NodeId,FrontierCell> committed_targets;
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
        if (policy_v2) {
            // Task 13 Phase B0-a: demand field (low-frequency/event-driven
            // recompute) + reachability projection + target-lock contract.
            advanceV2Targets(runtime,result);
        } else {
        const auto cells=currentUncoveredCells();
        std::set<std::string> uncovered;
        for (const auto& cell : cells) uncovered.insert(cell.id());
        bool target_completed=false;
        bool cvt_target_present=false;
        for (const auto& target_entry : targets_) {
            const NodeId owner=target_entry.first;
            const FrontierCell& target=target_entry.second;
            const bool leader=std::any_of(
                branches_.begin(),branches_.end(),[&](const auto& branch) {
                    return branch.leader==owner;
                });
            if (!leader) continue;
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
            targets_=result.allocation.targets;
            ++target_epoch_;
            consecutive_failures_=0;
        }
        }

        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
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
            const auto found=targets_.find(owner);
            std::optional<Eigen::Vector2d> soft_target=
                found==targets_.end()?std::optional<Eigen::Vector2d>{}:
                    std::optional<Eigen::Vector2d>{found->second.center};
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
        if (!t100_coverage_s_.has_value() &&
            adapter_.coverage().truthFraction()>=1.0-1e-12) {
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
    std::size_t target_epoch_=0;
    std::size_t consecutive_failures_=0;
    std::size_t successful_control_cycles_=0;
    std::size_t control_boundaries_=0;
    SimpleCoveragePhase phase_=SimpleCoveragePhase::CoverageSearch;
    std::optional<double> t100_coverage_s_;
    std::size_t settling_dwell_cycles_=0;
    std::map<NodeId,Eigen::Vector2d> last_nominal_controls_;
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
