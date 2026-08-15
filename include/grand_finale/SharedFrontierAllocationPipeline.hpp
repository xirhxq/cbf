#pragma once

#include "grand_finale/FrontierViabilityGate.hpp"
#include "grand_finale/FrontierViabilityRollout.hpp"

#include <cstddef>
#include <chrono>
#include <numeric>
#include <string>
#include <vector>

namespace gf {

struct SharedFrontierPipelineRequest {
    FrontierAllocationRequest allocation;
    CanonicalHardRowRequest hard_row_request;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    SolverProfile profile = SolverProfile::OpenSource;
    double dt_s = 0.0;
    JointEstimateSnapshot estimator_snapshot;
    double estimator_acceleration_variance = 0.0;
    CanonicalGammaFeedbackConfig gamma_feedback_config;
    std::function<CanonicalHardRowRequest(
        const JointEstimateSnapshot&)> canonical_request_builder;
};

struct SharedFrontierPipelineResult {
    bool accepted = false;
    std::string reason;
    AllocatorExhaustion exhaustion = AllocatorExhaustion::None;
    bool physical_deadlock_claimed = false;
    bool search_budget_truncated = false;
    std::size_t candidate_bundles = 0;
    std::size_t fast_rejections = 0;
    std::size_t exact_rejections = 0;
    std::size_t rollout_rejections = 0;
    std::size_t rollout_attempts = 0;
    std::string last_candidate_failure_reason;
    FrontierAllocationResult allocation;
    FrontierRolloutResult rollout;
    std::vector<FrontierRolloutResult> rejected_rollouts;
    CanonicalGammaFeedbackWork gamma_policy_work;
    std::size_t qp_solves = 0;
    std::size_t rollout_cycles = 0;
    double policy_wall_s = 0.0;
    double total_wall_s = 0.0;
    std::vector<double> policy_cycle_wall_s;
    Task10p11ComputeProfile compute_profile;
};

class SharedFrontierAllocationPipeline {
public:
    explicit SharedFrontierAllocationPipeline(
        SharedFrontierAllocatorConfig config)
        : config_(config),allocator_(config) {}

    SharedFrontierPipelineResult choose(
        SharedFrontierPipelineRequest request) {
        const auto started=std::chrono::steady_clock::now();
        SharedFrontierPipelineResult result;
        const auto finish=[&]() {
            result.total_wall_s=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-started).count();
            return result;
        };
        const auto rows=buildCanonicalHardRows(request.hard_row_request);
        request.allocation.hard_rows=rows;
        AllocatorExhaustion last_rejection=AllocatorExhaustion::None;
        for (std::size_t inspected=0;inspected<config_.bundle_cap;++inspected) {
            const auto candidate_started=std::chrono::steady_clock::now();
            const auto allocation=allocator_.allocate(request.allocation);
            result.compute_profile.record(
                Task10p11ComputePhase::CandidateBundleConstruction,
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-candidate_started).count(),true);
            if (!allocation.accepted) {
                result.reason="allocator_search_exhausted";
                result.exhaustion=last_rejection==AllocatorExhaustion::None
                    ? allocation.exhaustion:last_rejection;
                result.search_budget_truncated=
                    allocation.exhaustion==AllocatorExhaustion::SearchBudgetTruncated;
                return finish();
            }
            ++result.candidate_bundles;
            BrakingGateRequest braking;
            braking.fixed_positions=request.fixed_positions;
            braking.acceleration_half_box=
                request.allocation.acceleration_half_box;
            braking.collision_distance_m=
                request.allocation.collision_distance_m;
            braking.position_reserve_m=
                request.allocation.position_reserve_m;
            braking.velocity_reserve_mps=
                request.allocation.velocity_reserve_mps;
            for (const auto& agent : request.allocation.agents) {
                braking.agents.push_back({
                    agent,allocation.targets.at(agent.id).center});
            }
            const auto fast=evaluateBrakingGate(braking);
            if (!fast.accepted) {
                ++result.fast_rejections;
                last_rejection=AllocatorExhaustion::FastGateRejected;
                allocator_.rejectCurrentBundle(request.allocation);
                continue;
            }
            const auto exact_started=std::chrono::steady_clock::now();
            const auto exact=evaluateCurrentBundleGate(
                rows,allocation.nominal_controls,
                request.allocation.acceleration_half_box,
                {config_.maximum_projection_norm,
                 config_.minimum_direction_ratio,
                 config_.comparison_tolerance,true});
            result.compute_profile.record(
                Task10p11ComputePhase::ExactHardProjection,
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-exact_started).count(),true);
            if (!exact.accepted) {
                ++result.exact_rejections;
                last_rejection=AllocatorExhaustion::ExactGateRejected;
                allocator_.rejectCurrentBundle(request.allocation);
                continue;
            }
            if (result.rollout_attempts>=config_.rollout_cap) {
                result.reason="allocator_search_exhausted";
                result.exhaustion=AllocatorExhaustion::SearchBudgetTruncated;
                result.search_budget_truncated=true;
                return finish();
            }
            ++result.rollout_attempts;
            FrontierRolloutRequest rollout_request;
            rollout_request.profile=request.profile;
            rollout_request.hard_row_request=request.hard_row_request;
            for (const auto& [id,cell] : allocation.targets)
                rollout_request.targets[id]=cell.center;
            rollout_request.position_gain=request.allocation.position_gain;
            rollout_request.velocity_gain=request.allocation.velocity_gain;
            rollout_request.dt_s=request.dt_s;
            rollout_request.cycles=config_.rollout_cycles;
            rollout_request.estimator_snapshot=request.estimator_snapshot;
            rollout_request.estimator_acceleration_variance=
                request.estimator_acceleration_variance;
            rollout_request.gamma_feedback_config=
                request.gamma_feedback_config;
            rollout_request.canonical_request_builder=
                request.canonical_request_builder;
            const auto rollout=evaluateFrontierRollout(rollout_request);
            result.compute_profile.merge(rollout.compute_profile);
            result.gamma_policy_work.policy_evaluations+=
                rollout.gamma_policy_work.policy_evaluations;
            result.gamma_policy_work.canonical_row_rebuilds+=
                rollout.gamma_policy_work.canonical_row_rebuilds;
            result.gamma_policy_work.exact_gamma_solves+=
                rollout.gamma_policy_work.exact_gamma_solves;
            result.qp_solves+=rollout.qp_solves;
            result.rollout_cycles+=rollout.policy_trace.size();
            result.policy_wall_s+=std::accumulate(
                rollout.policy_cycle_wall_s.begin(),
                rollout.policy_cycle_wall_s.end(),0.0);
            result.policy_cycle_wall_s.insert(
                result.policy_cycle_wall_s.end(),
                rollout.policy_cycle_wall_s.begin(),
                rollout.policy_cycle_wall_s.end());
            if (!rollout.accepted) {
                ++result.rollout_rejections;
                last_rejection=AllocatorExhaustion::RolloutRejected;
                result.rollout=rollout;
                result.last_candidate_failure_reason=rollout.reason;
                result.rejected_rollouts.push_back(rollout);
                allocator_.rejectCurrentBundle(request.allocation);
                continue;
            }
            result.accepted=true;
            result.reason="accepted";
            result.allocation=allocation;
            result.rollout=rollout;
            return finish();
        }
        result.reason="allocator_search_exhausted";
        result.exhaustion=last_rejection==AllocatorExhaustion::None
            ? AllocatorExhaustion::SearchBudgetTruncated:last_rejection;
        result.search_budget_truncated=true;
        return finish();
    }

    std::size_t rejectionAge(const std::string& cell_id) const {
        return allocator_.rejectionAge(cell_id);
    }

private:
    SharedFrontierAllocatorConfig config_;
    SharedCollisionViableFrontierAllocator allocator_;
};

}  // namespace gf
