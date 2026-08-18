#pragma once

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/TargetLiftTransitionPrototype.hpp"
#include "grand_finale/Task10p11gFrozenModel.hpp"

namespace gf {

struct TargetLiftUnionExecutionStep {
    GrandFinaleSwarmStep step;
    std::uint64_t applied_target_digest=0;
    FrozenUnionCycleEvidence evidence;
};

// Fixture-only coordinator for the Task 10.11i mechanism proof.  It owns no
// alternate safety controller: all physical control is still applied by the
// formal adapter's canonical gamma/QP/residual path.
class TargetLiftUnionExecutionPrototype {
public:
    TargetLiftUnionExecutionPrototype(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter)
        : swarm_(swarm),adapter_(adapter) {}

    bool begin(
        const DirectedEdge& addition,const DirectedEdge& removal,
        FrozenTargetLiftToken token,
        TargetGeometryGateRequest geometry_request) {
        if (active_.has_value()) return false;
        const auto before=adapter_.runtimeSnapshot();
        const auto expected_additions=targetEdgeDifference(
            token.successor_edges,token.old_edges);
        const auto expected_removals=targetEdgeDifference(
            token.old_edges,token.successor_edges);
        if (!token.valid || token.lifted_digest!=canonicalLiftedTargetDigest(
                token.r_plan_m,token.lifted_targets) ||
            before.mode!=SupervisorMode::Search ||
            expected_additions.size()!=1 || expected_removals.size()!=1 ||
            expected_additions.front().id()!=addition.id() ||
            expected_removals.front().id()!=removal.id() ||
            canonicalTargetEdges(before.topology)!=token.old_edges ||
            before.topology_token!=token.topology_version ||
            before.estimator_token!=token.estimator_version ||
            geometry_request.reference_edges!=token.union_edges ||
            canonicalTargetLedgerDigest(geometry_request.raw_targets)!=
                token.raw_ledger_digest ||
            canonicalTargetLedgerDigest(geometry_request.targets)!=
                canonicalTargetLedgerDigest(token.lifted_targets))
            return false;
        const auto geometry=evaluateTargetGeometryGates(geometry_request);
        if (!geometry.valid) return false;
        if (!adapter_.beginReplacement(addition,removal)) return false;
        const auto after=adapter_.runtimeSnapshot();
        if (after.mode!=SupervisorMode::Union ||
            !after.adapter_transition_pending ||
            canonicalTargetEdges(after.topology)!=token.union_edges)
            return false;
        active_=Active{std::move(token),std::move(geometry_request)};
        return true;
    }

    TargetLiftUnionExecutionStep advanceUnion() {
        TargetLiftUnionExecutionStep result;
        if (!active_.has_value()) {
            result.step.reason="target_lift_union_not_active";
            return result;
        }
        const auto before=adapter_.runtimeSnapshot();
        if (before.mode!=SupervisorMode::Union ||
            canonicalTargetEdges(before.topology)!=active_->token.union_edges) {
            result.step.reason="target_lift_union_state_mismatch";
            return result;
        }
        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
        auto model=task10p11gFrozenModel();
        model.acceleration_half_box_mps2=adapter_.config().acceleration_half_box;
        model.position_gain=adapter_.config().position_gain;
        model.velocity_gain=adapter_.config().velocity_gain;
        model.maximum_yaw_rate_radps=adapter_.config().maximum_yaw_rate_radps;
        for (std::size_t index=0;index<before.estimate.mobile_ids.size();++index) {
            const NodeId owner=before.estimate.mobile_ids[index];
            const auto target=active_->token.lifted_targets.find(owner);
            if (target==active_->token.lifted_targets.end()) {
                result.step.reason="lifted_target_missing";
                return result;
            }
            const Eigen::Vector4d state=
                before.estimate.mean.segment<4>(4*index);
            const auto task=task10p11gSoftTask({
                state.head<2>(),state.tail<2>(),currentYaw(owner),
                target->second,before.mode},model);
            nominal[owner]=task.acceleration;
            yaw_rates[owner]=task.yaw_rate_radps;
        }
        result.applied_target_digest=canonicalLiftedTargetDigest(
            active_->token.r_plan_m,active_->token.lifted_targets);
        result.step=adapter_.stepWithNominalAndYawRates(nominal,yaw_rates);
        const auto after=adapter_.runtimeSnapshot();
        result.evidence={result.step.mode,after.topology,
            result.applied_target_digest,active_->token.raw_ledger_version,
            active_->token.raw_ledger_digest,active_->token.config_version,
            active_->token.config_digest,result.step.topology_version,
            result.step.estimator_version_before,
            result.step.estimator_version_after,result.step.advanced,
            result.step.advanced,result.step.minimum_hard_residual,
            adapter_.config().residual_tolerance};
        active_->token=recordFreshUnionControlCycle(
            active_->token,result.evidence);
        return result;
    }

    bool finishAfterFreshCycle() {
        if (!active_.has_value()) return false;
        const auto runtime=adapter_.runtimeSnapshot();
        if (!freshBreakTargetReady(active_->token,runtime.topology_token,
                runtime.estimator_token))
            return false;
        if (!adapter_.finishReplacementAfterFreshCycle()) return false;
        active_.reset();
        return true;
    }

    const FrozenTargetLiftToken* token() const {
        return active_.has_value()?&active_->token:nullptr;
    }

private:
    struct Active {
        FrozenTargetLiftToken token;
        TargetGeometryGateRequest geometry;
    };
    double currentYaw(NodeId owner) const {
        for (const auto& robot : swarm_.robots)
            if (robot->id==owner)
                return robot->model->getStateVariable("yawRad");
        throw std::invalid_argument("missing yaw state");
    }
    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    std::optional<Active> active_;
};

}  // namespace gf
