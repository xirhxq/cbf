#pragma once

#include "grand_finale/Task10p11agComponentRecovery.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

namespace gf {

struct Task10p11ahFrozenProtocol {
    double tau_mps2=22.0;
    double dt_s=0.1;
    std::set<NodeId> component{2,9};
    std::size_t horizon_steps=2;
    double feasibility_tolerance_mps2=1.0e-8;
    bool fixed_topology=true;
    bool recursive_feasibility_claimed=false;
};

inline Task10p11ahFrozenProtocol task10p11ahFrozenProtocol() { return {}; }

enum class Task10p11ahTerminalClassification {
    Invalid,
    DistributedRecovered,
    ComponentStillRequired,
    FiniteHorizonDeadEnd
};

inline std::string task10p11ahTerminalClassificationName(
    Task10p11ahTerminalClassification value) {
    switch (value) {
    case Task10p11ahTerminalClassification::DistributedRecovered:
        return "distributed_recovered";
    case Task10p11ahTerminalClassification::ComponentStillRequired:
        return "component_still_required";
    case Task10p11ahTerminalClassification::FiniteHorizonDeadEnd:
        return "finite_horizon_dead_end";
    default: return "invalid";
    }
}

namespace task10p11ah_detail {

inline std::map<NodeId,Eigen::Vector2d> controlsFromJson(
    const nlohmann::json& value,const std::vector<NodeId>& owners) {
    std::map<NodeId,Eigen::Vector2d> result;
    for (NodeId owner:owners) {
        const auto encoded=value.at(std::to_string(owner)).get<std::vector<double>>();
        if (encoded.size()!=2 || !std::isfinite(encoded[0]) ||
            !std::isfinite(encoded[1]))
            throw std::runtime_error("invalid encoded H2 control");
        result.emplace(owner,Eigen::Vector2d(encoded[0],encoded[1]));
    }
    return result;
}

inline bool fullRowFeasible(const Task10p11sRows28d& problem,
                            const Eigen::VectorXd& controls,
                            double* minimum=nullptr) {
    const auto audit=task10p11w_detail::minimumResidual(problem,controls);
    if (minimum!=nullptr) *minimum=audit.first;
    return audit.first>=-task10p11ag_detail::kTolerance;
}

inline nlohmann::json snapshotAtEstimate(
    nlohmann::json snapshot,const JointEstimateSnapshot& estimate,
    const CanonicalHardRowRequest& request,
    const std::map<NodeId,Eigen::Vector2d>& nominal,double runtime_s) {
    snapshot["estimator"]=task10p11s_capture_detail::estimateJson(estimate);
    snapshot["canonical_request"]=task10p11s_capture_detail::requestJson(request);
    snapshot["nominal_controls"]=nlohmann::json::object();
    for (const auto& [owner,control]:nominal)
        snapshot["nominal_controls"][std::to_string(owner)]={control.x(),control.y()};
    const auto rows=buildCanonicalHardRows(request);
    snapshot["actual_rows"]=nlohmann::json::array();
    snapshot["owner_row_counts"]=nlohmann::json::object();
    for (NodeId owner:request.mobile_ids)
        snapshot["owner_row_counts"][std::to_string(owner)]=0;
    for (const auto& row:rows) {
        snapshot["actual_rows"].push_back(
            task10p11s_capture_detail::rowJson(row));
        snapshot["owner_row_counts"][std::to_string(row.owner)]=
            snapshot["owner_row_counts"][std::to_string(row.owner)]
                .get<std::size_t>()+1;
    }
    snapshot["objective_28d"]=
        task10p11s_capture_detail::orderedObjective(request.mobile_ids,nominal);
    snapshot["runtime"]["runtime_s"]=runtime_s;
    snapshot["runtime"]["fixed_topology"]=
        snapshot["canonical_request"]["reference_edges"];
    if (!validateTask10p11sSnapshot(snapshot).complete)
        throw std::runtime_error("derived estimate snapshot is incomplete");
    return snapshot;
}

struct ContinuationWitness {
    bool h1=false;
    bool h2=false;
    double h1_margin=-std::numeric_limits<double>::infinity();
    double h2_margin=-std::numeric_limits<double>::infinity();
};

inline ContinuationWitness continuationWitness(
    const nlohmann::json& snapshot,const JointEstimateSnapshot& state,
    const CanonicalHardRowRequest& request,const Eigen::VectorXd& current) {
    ContinuationWitness result;
    const std::set<NodeId> all(request.mobile_ids.begin(),request.mobile_ids.end());
    const auto controls=task10p11sControlMap(request.mobile_ids,current);
    const auto next=task10p11aa_detail::predictEstimate(snapshot,state,controls);
    const auto next_request=task10p11x_detail::requestAtEstimate(snapshot,next);
    const auto next_problem=buildTask10p11sRows28d(
        buildCanonicalHardRows(next_request),next_request.mobile_ids,true);
    const auto one=task10p11w_detail::solveRestricted(
        next_problem,current,current,all,true);
    if (!one.feasible || one.controls.size()!=current.size()) return result;
    result.h1_margin=one.margin;
    result.h1=one.margin>=-task10p11ag_detail::kTolerance &&
        one.minimum_residual>=-task10p11ag_detail::kTolerance;
    if (!result.h1) return result;
    const auto controls1=task10p11sControlMap(next_request.mobile_ids,one.controls);
    const auto next2=task10p11aa_detail::predictEstimate(
        snapshot,next,controls1);
    const auto request2=task10p11x_detail::requestAtEstimate(snapshot,next2);
    const auto problem2=buildTask10p11sRows28d(
        buildCanonicalHardRows(request2),request2.mobile_ids,true);
    const auto two=task10p11w_detail::solveRestricted(
        problem2,one.controls,one.controls,all,true);
    if (!two.feasible) return result;
    result.h2_margin=two.margin;
    result.h2=two.margin>=-task10p11ag_detail::kTolerance &&
        two.minimum_residual>=-task10p11ag_detail::kTolerance;
    return result;
}

}  // namespace task10p11ah_detail

struct Task10p11ahExistingH2Audit {
    bool valid=false;
    std::string reason;
    std::size_t x0_row_count=0,x1_row_count=0,x2_row_count=0;
    double x0_minimum_residual_mps2=-std::numeric_limits<double>::infinity();
    double x1_minimum_residual_mps2=-std::numeric_limits<double>::infinity();
    double x2_minimum_residual_mps2=-std::numeric_limits<double>::infinity();
    double terminal_owner_local_gamma_mps2=-std::numeric_limits<double>::infinity();
    bool terminal_signed_transfer_feasible=false;
    bool terminal_h1_witness=false;
    bool terminal_h2_witness=false;
    bool terminal_h1_strictly_infeasible=false;
    bool terminal_h2_strictly_infeasible=false;
    double terminal_h1_margin_mps2=-std::numeric_limits<double>::infinity();
    double terminal_h2_margin_mps2=-std::numeric_limits<double>::infinity();
    bool native_distributed_terminal_audited=false;
    Task10p11ahTerminalClassification terminal_classification=
        Task10p11ahTerminalClassification::Invalid;
};

inline Task10p11ahExistingH2Audit auditTask10p11ahExistingH2Witness(
    const nlohmann::json& snapshot,const nlohmann::json& witness) {
    Task10p11ahExistingH2Audit result;
    try {
        const auto request0=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto u0_map=task10p11ah_detail::controlsFromJson(
            witness.at("U0"),request0.mobile_ids);
        const auto u0=task10p11sOrderedControls(request0.mobile_ids,u0_map);
        const auto problem0=buildTask10p11sRows28d(
            buildCanonicalHardRows(request0),request0.mobile_ids,true);
        result.x0_row_count=problem0.rows.size();
        if (!task10p11ah_detail::fullRowFeasible(
                problem0,u0,&result.x0_minimum_residual_mps2))
            throw std::runtime_error("x0 full rows infeasible");
        const auto x0=task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        const auto x1=task10p11aa_detail::predictEstimate(snapshot,x0,u0_map);
        if (task10p11s_capture_detail::estimateJson(x1)!=witness.at("x1"))
            throw std::runtime_error("x1 exact-ZOH identity mismatch");
        const auto request1=task10p11x_detail::requestAtEstimate(snapshot,x1);
        const auto u1_map=task10p11ah_detail::controlsFromJson(
            witness.at("U1"),request1.mobile_ids);
        const auto u1=task10p11sOrderedControls(request1.mobile_ids,u1_map);
        const auto problem1=buildTask10p11sRows28d(
            buildCanonicalHardRows(request1),request1.mobile_ids,true);
        result.x1_row_count=problem1.rows.size();
        if (!task10p11ah_detail::fullRowFeasible(
                problem1,u1,&result.x1_minimum_residual_mps2))
            throw std::runtime_error("x1 full rows infeasible");
        const auto x2=task10p11aa_detail::predictEstimate(snapshot,x1,u1_map);
        if (task10p11s_capture_detail::estimateJson(x2)!=witness.at("x2"))
            throw std::runtime_error("x2 exact-ZOH identity mismatch");
        const auto request2=task10p11x_detail::requestAtEstimate(snapshot,x2);
        const auto u2_map=task10p11ah_detail::controlsFromJson(
            witness.at("U2"),request2.mobile_ids);
        const auto u2=task10p11sOrderedControls(request2.mobile_ids,u2_map);
        const auto rows2=buildCanonicalHardRows(request2);
        const auto problem2=buildTask10p11sRows28d(
            rows2,request2.mobile_ids,true);
        result.x2_row_count=problem2.rows.size();
        if (!task10p11ah_detail::fullRowFeasible(
                problem2,u2,&result.x2_minimum_residual_mps2))
            throw std::runtime_error("x2 full rows infeasible");
        result.terminal_owner_local_gamma_mps2=
            task10p11af_detail::minimumOwnerLocalGamma(rows2,request2);
        result.terminal_signed_transfer_feasible=solveTask10p11tDynamicPair(
            rows2,request2.mobile_ids,u2_map,request2.acceleration_half_box,
            "collision:2--9").feasible;
        const auto continuation=task10p11ah_detail::continuationWitness(
            snapshot,x2,request2,u2);
        result.terminal_h1_witness=continuation.h1;
        result.terminal_h2_witness=continuation.h2;
        result.terminal_h1_margin_mps2=continuation.h1_margin;
        result.terminal_h2_margin_mps2=continuation.h2_margin;
        const bool local=result.terminal_owner_local_gamma_mps2>=
            -task10p11ag_detail::kTolerance;
        if (result.terminal_h1_strictly_infeasible)
            result.terminal_classification=
                Task10p11ahTerminalClassification::FiniteHorizonDeadEnd;
        else if (local && result.terminal_signed_transfer_feasible &&
                 continuation.h2 && result.native_distributed_terminal_audited)
            result.terminal_classification=
                Task10p11ahTerminalClassification::DistributedRecovered;
        else
            result.terminal_classification=
                Task10p11ahTerminalClassification::ComponentStillRequired;
        result.valid=true;
        result.reason="existing_full_domain_H2_witness_rebuilt";
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    return result;
}

inline bool task10p11ahShouldEnter(bool current_h2_feasible,
                                   bool normal_next_h2_feasible) {
    return current_h2_feasible && !normal_next_h2_feasible;
}

inline bool task10p11ahOnlyComponentDiffers(
    const std::map<NodeId,Eigen::Vector2d>& distributed,
    const std::map<NodeId,Eigen::Vector2d>& selected,
    const std::set<NodeId>& component,double tolerance) {
    if (distributed.size()!=selected.size()) return false;
    for (const auto& [owner,control]:distributed) {
        const auto found=selected.find(owner);
        if (found==selected.end()) return false;
        if (component.count(owner)==0 &&
            (control-found->second).cwiseAbs().maxCoeff()>tolerance)
            return false;
    }
    return true;
}

struct Task10p11ahDecision {
    bool valid=false;
    bool triggered=false;
    bool component_applied=false;
    bool fail_closed=false;
    std::string reason;
    double current_h2_margin_mps2=-std::numeric_limits<double>::infinity();
    double normal_next_h2_margin_mps2=-std::numeric_limits<double>::infinity();
    std::map<NodeId,Eigen::Vector2d> selected_controls;
};

inline Task10p11ahDecision task10p11ahDecideEarlyH2(
    const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& distributed,bool enabled=true) {
    Task10p11ahDecision result;
    const auto validation=validateTask10p11sSnapshot(snapshot);
    if (!validation.complete) {
        result.fail_closed=true;
        result.reason="packed_snapshot_incomplete:"+validation.reason;
        return result;
    }
    if (!enabled) {
        result.valid=true;
        result.reason="early_H2_disabled_distributed_command_unchanged";
        result.selected_controls=distributed;
        return result;
    }
    result.reason="enabled_H2_planner_not_evaluated";
    return result;
}

inline nlohmann::json task10p11ahDecisionJson(
    const Task10p11ahDecision& value) {
    return {{"valid",value.valid},{"triggered",value.triggered},
        {"component_applied",value.component_applied},
        {"fail_closed",value.fail_closed},{"reason",value.reason},
        {"current_h2_margin_mps2",task10p11w_detail::number(
            value.current_h2_margin_mps2)},
        {"normal_next_h2_margin_mps2",task10p11w_detail::number(
            value.normal_next_h2_margin_mps2)},
        {"recursive_feasibility_claimed",false}};
}

}  // namespace gf
