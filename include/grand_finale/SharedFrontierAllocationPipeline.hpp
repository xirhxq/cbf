#pragma once

#include "grand_finale/FrontierViabilityGate.hpp"
#include "grand_finale/FrontierViabilityRollout.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace gf {

struct SharedFrontierPipelineRequest {
    FrontierAllocationRequest allocation;
    CanonicalHardRowRequest hard_row_request;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    SolverProfile profile = SolverProfile::OpenSource;
    double dt_s = 0.0;
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
};

class SharedFrontierAllocationPipeline {
public:
    explicit SharedFrontierAllocationPipeline(
        SharedFrontierAllocatorConfig config)
        : config_(config),allocator_(config) {}

    SharedFrontierPipelineResult choose(
        SharedFrontierPipelineRequest request) {
        SharedFrontierPipelineResult result;
        const auto rows=buildCanonicalHardRows(request.hard_row_request);
        request.allocation.hard_rows=rows;
        AllocatorExhaustion last_rejection=AllocatorExhaustion::None;
        for (std::size_t inspected=0;inspected<config_.bundle_cap;++inspected) {
            const auto allocation=allocator_.allocate(request.allocation);
            if (!allocation.accepted) {
                result.reason="allocator_search_exhausted";
                result.exhaustion=last_rejection==AllocatorExhaustion::None
                    ? allocation.exhaustion:last_rejection;
                result.search_budget_truncated=
                    allocation.exhaustion==AllocatorExhaustion::SearchBudgetTruncated;
                return result;
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
            const auto exact=evaluateCurrentBundleGate(
                rows,allocation.nominal_controls,
                request.allocation.acceleration_half_box,
                {config_.maximum_projection_norm,
                 config_.minimum_direction_ratio,
                 config_.comparison_tolerance,true});
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
                return result;
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
            const auto rollout=evaluateFrontierRollout(rollout_request);
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
            return result;
        }
        result.reason="allocator_search_exhausted";
        result.exhaustion=last_rejection==AllocatorExhaustion::None
            ? AllocatorExhaustion::SearchBudgetTruncated:last_rejection;
        result.search_budget_truncated=true;
        return result;
    }

    std::size_t rejectionAge(const std::string& cell_id) const {
        return allocator_.rejectionAge(cell_id);
    }

private:
    SharedFrontierAllocatorConfig config_;
    SharedCollisionViableFrontierAllocator allocator_;
};

}  // namespace gf
