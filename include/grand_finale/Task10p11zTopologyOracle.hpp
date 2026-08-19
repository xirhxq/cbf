#pragma once

#include "grand_finale/Task10p11xRecoveryCampaign.hpp"
#include "grand_finale/Task10p11yEvidence.hpp"

#include <filesystem>
#include <optional>

namespace gf {

struct Task10p11zTopologyPhase {
    bool performed=false;
    bool current_feasible=false;
    bool successor_feasible=false;
    std::size_t current_row_count=0;
    std::size_t successor_row_count=0;
    double current_minimum_residual_mps2=0.0;
    double successor_minimum_residual_mps2=0.0;
};

struct Task10p11zTopologyGate {
    bool valid=false;
    std::string reason;
    std::string r4_status="offline_rejected";
    std::string rule_addition;
    std::string rule_removal;
    double snapshot_time_s=0.0;
    bool replacement_certificate_valid=false;
    bool fresh_post_union_information_epoch_available=false;
    bool temporal_chain_reached_new=false;
    bool offline_gate_passed=false;
    bool trajectory_advanced=false;
    Task10p11zTopologyPhase old_phase;
    Task10p11zTopologyPhase union_phase;
    Task10p11zTopologyPhase new_phase;
};

namespace task10p11z_topology_detail {

struct PhaseEvaluation {
    Task10p11zTopologyPhase phase;
    std::optional<JointEstimateSnapshot> predicted;
};

inline PhaseEvaluation evaluatePhase(const nlohmann::json& snapshot,
    const JointEstimateSnapshot& start,
    const std::vector<DirectedEdge>& topology,
    const std::map<NodeId,Eigen::Vector2d>& nominal) {
    PhaseEvaluation result;
    const auto request=task10p11x_detail::requestAtEstimate(
        snapshot,start,std::nullopt,topology);
    const auto rows=buildCanonicalHardRows(request);
    const auto problem=buildTask10p11sRows28d(
        rows,request.mobile_ids,true);
    result.phase.performed=true;
    result.phase.current_row_count=problem.rows.size();
    const auto current=solveTask10p11sQp(problem,
        task10p11sOrderedControls(request.mobile_ids,nominal));
    result.phase.current_feasible=current.feasible &&
        current.minimum_residual>=-1.0e-8;
    result.phase.current_minimum_residual_mps2=current.feasible
        ?current.minimum_residual:0.0;
    if (!result.phase.current_feasible) return result;

    const auto controls=task10p11sControlMap(
        request.mobile_ids,current.controls);
    const auto& parameters=snapshot.at("successor_parameters");
    result.predicted=predictNoMeasurementSnapshot(start,controls,
        parameters.at("dt_s").get<double>(),
        parameters.at("estimator_acceleration_variance").get<double>());
    const auto successor_request=task10p11x_detail::requestAtEstimate(
        snapshot,*result.predicted,std::nullopt,topology);
    const auto successor_rows=buildCanonicalHardRows(successor_request);
    const auto successor_problem=buildTask10p11sRows28d(
        successor_rows,successor_request.mobile_ids,true);
    result.phase.successor_row_count=successor_problem.rows.size();
    const auto successor=solveTask10p11sQp(
        successor_problem,current.controls);
    result.phase.successor_feasible=successor.feasible &&
        successor.minimum_residual>=-1.0e-8;
    result.phase.successor_minimum_residual_mps2=successor.feasible
        ?successor.minimum_residual:0.0;
    return result;
}

}  // namespace task10p11z_topology_detail

inline Task10p11zTopologyGate runTask10p11zEarlyTopologyGate(
    const std::filesystem::path& sparse_checkpoint) {
    Task10p11zTopologyGate gate;
    const auto protocol=task10p11xPreregisteredParameters();
    gate.rule_addition=protocol.topology_addition.id();
    gate.rule_removal=protocol.topology_removal.id();
    try {
        const auto sparse=readTask10p11vJson(sparse_checkpoint);
        auto fixture=makeTask10p11rFixedBaselineFixture(
            GammaFeedbackSelectionMode::LeastIntervention,14.0);
        if (!fixture->adapter.initializeStageZero().initialized) {
            gate.reason="stage_zero_initialization_failed";
            return gate;
        }
        restoreTask10p11vSparseRestartCheckpoint(*fixture,sparse);
        gate.snapshot_time_s=fixture->swarm.robots.front()->runtime;
        std::optional<nlohmann::json> snapshot;
        const auto capture=fixture->controller.advanceWithDevelopmentControlOverride(
            [&](const GrandFinaleRuntimeSnapshot& runtime,
                const std::map<NodeId,Eigen::Vector2d>& nominal,
                const std::map<NodeId,double>&) {
                snapshot=makeTask10p11sSnapshot(runtime,
                    fixture->adapter.snapshotHardRowRequest(
                        runtime.estimate,runtime.topology),nominal,
                    fixture->adapter.config());
                GrandFinaleSwarmStep rejected;
                rejected.reason="offline_topology_capture_no_advance";
                return rejected;
            });
        if (capture.step.advanced || !snapshot.has_value()) {
            gate.reason="offline_snapshot_capture_advanced_or_missing";
            return gate;
        }
        const auto current=task10p11s_capture_detail::estimateFromJson(
            snapshot->at("estimator"));
        const auto old=task10p11s_capture_detail::requestFromJson(
            snapshot->at("canonical_request")).reference_edges;
        const auto union_edges=task10p11x_detail::replacedTopology(
            old,protocol.topology_addition,protocol.topology_removal,true);
        const auto new_edges=task10p11x_detail::replacedTopology(
            old,protocol.topology_addition,protocol.topology_removal,false);
        const auto nominal=task10p11s_capture_detail::nominalFromJson(
            snapshot->at("nominal_controls"));
        const auto old_phase=task10p11z_topology_detail::evaluatePhase(
            *snapshot,current,old,nominal);
        const auto union_phase=task10p11z_topology_detail::evaluatePhase(
            *snapshot,current,union_edges,nominal);
        gate.old_phase=old_phase.phase;
        gate.union_phase=union_phase.phase;
        if (union_phase.predicted.has_value()) {
            const auto union_controls_request=
                task10p11x_detail::requestAtEstimate(
                    *snapshot,current,std::nullopt,union_edges);
            const auto union_problem=buildTask10p11sRows28d(
                buildCanonicalHardRows(union_controls_request),
                union_controls_request.mobile_ids,true);
            const auto union_controls=solveTask10p11sQp(
                union_problem,task10p11sOrderedControls(
                    union_controls_request.mobile_ids,nominal));
            const auto new_nominal=task10p11sControlMap(
                union_controls_request.mobile_ids,union_controls.controls);
            gate.new_phase=task10p11z_topology_detail::evaluatePhase(
                *snapshot,*union_phase.predicted,new_edges,new_nominal).phase;
            gate.temporal_chain_reached_new=gate.new_phase.performed;
        } else {
            gate.new_phase=task10p11z_topology_detail::evaluatePhase(
                *snapshot,current,new_edges,nominal).phase;
        }
        const auto certificate=fixture->adapter.auditReplacement(
            protocol.topology_addition,protocol.topology_removal);
        gate.replacement_certificate_valid=certificate.valid;
        gate.fresh_post_union_information_epoch_available=false;
        gate.trajectory_advanced=false;
        gate.offline_gate_passed=false;
        gate.r4_status="offline_rejected";
        gate.valid=gate.old_phase.performed && gate.union_phase.performed &&
            gate.new_phase.performed;
        if (!gate.valid)
            gate.reason="old_union_new_control_audit_incomplete";
        else if (!gate.old_phase.current_feasible)
            gate.reason="old_current_infeasible";
        else if (!gate.old_phase.successor_feasible)
            gate.reason="old_successor_infeasible";
        else if (!gate.union_phase.current_feasible)
            gate.reason="union_current_infeasible";
        else if (!gate.union_phase.successor_feasible)
            gate.reason="union_successor_infeasible";
        else gate.reason=
            "fresh_post_union_information_epoch_not_available_offline";
    } catch (const std::exception& error) {
        gate.reason=error.what();
    }
    return gate;
}

inline nlohmann::json task10p11zTopologyPhaseJson(
    const Task10p11zTopologyPhase& phase) {
    return {{"performed",phase.performed},
        {"current_feasible",phase.current_feasible},
        {"successor_feasible",phase.successor_feasible},
        {"current_row_count",phase.current_row_count},
        {"successor_row_count",phase.successor_row_count},
        {"current_minimum_residual_mps2",
            phase.current_minimum_residual_mps2},
        {"successor_minimum_residual_mps2",
            phase.successor_minimum_residual_mps2}};
}

inline nlohmann::json task10p11zTopologyGateJson(
    const Task10p11zTopologyGate& gate) {
    return {{"protocol","task10p11z-early-topology-gate-v1"},
        {"valid",gate.valid},{"reason",gate.reason},
        {"r4_status",gate.r4_status},
        {"rule_addition",gate.rule_addition},
        {"rule_removal",gate.rule_removal},
        {"snapshot_time_s",gate.snapshot_time_s},
        {"replacement_certificate_valid",
            gate.replacement_certificate_valid},
        {"fresh_post_union_information_epoch_available",
            gate.fresh_post_union_information_epoch_available},
        {"temporal_chain_reached_new",gate.temporal_chain_reached_new},
        {"offline_gate_passed",gate.offline_gate_passed},
        {"trajectory_advanced",gate.trajectory_advanced},
        {"old",task10p11zTopologyPhaseJson(gate.old_phase)},
        {"union",task10p11zTopologyPhaseJson(gate.union_phase)},
        {"new",task10p11zTopologyPhaseJson(gate.new_phase)},
        {"claim_boundary",{{"offline_exact_zoh_only",true},
            {"fresh_information_epoch_claimed",false},
            {"recursive_feasibility_claimed",false}}}};
}

}  // namespace gf
