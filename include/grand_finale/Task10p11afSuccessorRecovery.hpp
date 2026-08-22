#pragma once

#include "grand_finale/Task10p11aaGraphOracle.hpp"
#include "grand_finale/Task10p11aaMarginAudit.hpp"
#include "grand_finale/Task10p11acCampaign.hpp"

#include <filesystem>
#include <limits>
#include <optional>
#include <string>

namespace gf {

struct Task10p11afFrozenProtocol {
    double tau_mps2=22.0;
    std::size_t candidate_count=9;
    double dt_s=0.1;
    std::string initialization="P3";
    std::string monitored_pair="collision:2--9";
    bool fixed_topology=true;
    bool development_oracle=true;
    bool recursive_feasibility_claimed=false;
};

inline Task10p11afFrozenProtocol task10p11afFrozenProtocol() { return {}; }

inline std::map<NodeId,Eigen::Vector2d>
task10p11afAppliedControlsFromCheckpoint(const nlohmann::json& checkpoint) {
    const nlohmann::json* decision=nullptr;
    if (checkpoint.contains("task10p11ac") &&
        checkpoint.at("task10p11ac").contains("decision"))
        decision=&checkpoint.at("task10p11ac").at("decision");
    else if (checkpoint.contains("task10p11aa") &&
             checkpoint.at("task10p11aa").contains("decision"))
        decision=&checkpoint.at("task10p11aa").at("decision");
    if (decision==nullptr)
        throw std::runtime_error("checkpoint decision absent");
    if (decision->contains("applied_controls")) {
        std::map<NodeId,Eigen::Vector2d> controls;
        for (const auto& item:decision->at("applied_controls").items()) {
            const auto value=item.value().get<std::vector<double>>();
            if (value.size()!=2 || !std::isfinite(value[0]) ||
                !std::isfinite(value[1]))
                throw std::runtime_error("invalid encoded applied control");
            controls.emplace(static_cast<NodeId>(std::stoull(item.key())),
                Eigen::Vector2d(value[0],value[1]));
        }
        if (controls.size()!=14)
            throw std::runtime_error("checkpoint applied controls incomplete");
        return controls;
    }
    std::map<NodeId,Eigen::Vector2d> controls;
    for (const auto& owner:decision->at("owner_decisions")) {
        if (!owner.contains("final_applied_control"))
            throw std::runtime_error("owner final applied control absent");
        const auto value=owner.at("final_applied_control").get<std::vector<double>>();
        if (value.size()!=2 || !std::isfinite(value[0]) ||
            !std::isfinite(value[1]))
            throw std::runtime_error("invalid owner final applied control");
        controls.emplace(owner.at("owner").get<NodeId>(),
            Eigen::Vector2d(value[0],value[1]));
    }
    if (controls.size()!=14)
        throw std::runtime_error("checkpoint owner controls incomplete");
    return controls;
}

namespace task10p11af_detail {

constexpr double kTolerance=1.0e-8;

inline double independentMinimumResidual(
    const Task10p11sRows28d& problem,const Eigen::VectorXd& controls,
    std::string* limiting=nullptr) {
    double minimum=std::numeric_limits<double>::infinity();
    std::string row_id;
    for (const auto& row:problem.rows) {
        const double residual=row.residual(controls);
        if (residual<minimum) { minimum=residual; row_id=row.id; }
    }
    if (limiting!=nullptr) *limiting=row_id;
    return minimum;
}

inline double minimumOwnerLocalGamma(
    const std::vector<CanonicalHardRow>& rows,
    const CanonicalHardRowRequest& request) {
    double minimum=std::numeric_limits<double>::infinity();
    for (NodeId owner:request.mobile_ids) {
        const auto result=solveCanonicalGammaStar(
            rows,owner,request.acceleration_half_box);
        if (!result.valid || !std::isfinite(result.gamma))
            throw std::runtime_error("owner local gamma unavailable:"+
                                     std::to_string(owner));
        minimum=std::min(minimum,result.gamma);
    }
    return minimum;
}

inline task10p11aa_detail::FullStateAudit successorFullPair(
    const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& controls) {
    const auto estimate0=task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    const auto estimate1=task10p11aa_detail::predictEstimate(
        snapshot,estimate0,controls);
    const auto request1=task10p11x_detail::requestAtEstimate(
        snapshot,estimate1);
    return task10p11aa_detail::fullStateAudit(request1,
        task10p11sOrderedControls(request1.mobile_ids,controls));
}

}  // namespace task10p11af_detail

struct Task10p11afTakeoverDecision {
    bool valid=false;
    bool current_distributed_feasible=false;
    bool successor_distributed_feasible=false;
    bool takeover_required=false;
    bool takeover_applied=false;
    bool fail_closed=false;
    std::string reason;
    double current_distributed_residual_mps2=
        -std::numeric_limits<double>::infinity();
    double successor_distributed_gamma_mps2=
        -std::numeric_limits<double>::infinity();
    double successor_distributed_residual_mps2=
        -std::numeric_limits<double>::infinity();
    Task10p11aaGraphDecision oracle;
    std::map<NodeId,Eigen::Vector2d> selected_controls;
};

inline Task10p11afTakeoverDecision task10p11afDecideTakeover(
    const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& distributed_controls,
    Task10p11aaGraphDepth depth,bool enabled=true) {
    using namespace task10p11af_detail;
    Task10p11afTakeoverDecision result;
    try {
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        if (distributed_controls.size()!=request.mobile_ids.size())
            throw std::runtime_error("distributed control map incomplete");
        const auto rows=buildCanonicalHardRows(request);
        const auto problem=buildTask10p11sRows28d(
            rows,request.mobile_ids,true);
        const auto ordered=task10p11sOrderedControls(
            request.mobile_ids,distributed_controls);
        result.current_distributed_residual_mps2=
            independentMinimumResidual(problem,ordered);
        result.current_distributed_feasible=
            result.current_distributed_residual_mps2>=-kTolerance;
        if (!result.current_distributed_feasible) {
            result.fail_closed=true;
            result.reason="distributed_current_full_rows_infeasible";
            return result;
        }
        const auto successor=successorFullPair(snapshot,distributed_controls);
        result.successor_distributed_gamma_mps2=successor.recomputed_gamma;
        result.successor_distributed_residual_mps2=successor.minimum_residual;
        result.successor_distributed_feasible=successor.feasible;
        result.takeover_required=!result.successor_distributed_feasible;
        if (!enabled || !result.takeover_required) {
            result.valid=true;
            result.reason=!enabled?"takeover_disabled_distributed_command_committed":
                "distributed_current_and_successor_full_pair_feasible";
            result.selected_controls=distributed_controls;
            return result;
        }
        result.oracle=task10p11aaDecideGraphOracle(
            snapshot,distributed_controls,depth);
        if (!result.oracle.valid) {
            result.fail_closed=true;
            result.reason="oracle_fail_closed:"+result.oracle.reason;
            return result;
        }
        result.valid=true;
        result.takeover_applied=result.oracle.selected_index!=0;
        result.reason=result.takeover_applied
            ?"fixed_topology_full_pair_successor_recovery_applied"
            :"successor_recovered_by_distributed_endpoint";
        result.selected_controls=result.oracle.selected_controls;
    } catch (const std::exception& error) {
        result.fail_closed=true;
        result.reason=error.what();
    }
    return result;
}

struct Task10p11afLayerAudit {
    bool valid=false;
    std::string reason;
    double time_s=0.0;
    double owner_local_gamma_mps2=std::numeric_limits<double>::quiet_NaN();
    Task10p11tDynamicPairResult signed_transfer;
    task10p11w_detail::RestrictedResult component_margin;
    task10p11w_detail::RestrictedResult current_full_pair_margin;
    double applied_current_full_row_residual_mps2=
        std::numeric_limits<double>::quiet_NaN();
    task10p11aa_detail::FullStateAudit applied_successor_full_pair;
    Task10p11afTakeoverDecision g1;
    Task10p11afTakeoverDecision g2;
    std::size_t current_full_row_count=0;
};

inline Task10p11afLayerAudit task10p11afAuditPackedCheckpoint(
    const std::filesystem::path& path,
    const std::string& pair_id="collision:2--9") {
    using namespace task10p11af_detail;
    Task10p11afLayerAudit result;
    try {
        const auto snapshot=readTask10p11vJson(path);
        const auto validation=validateTask10p11sSnapshot(snapshot);
        if (!validation.complete)
            throw std::runtime_error("packed snapshot incomplete");
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto rows=buildCanonicalHardRows(request);
        const auto problem=buildTask10p11sRows28d(
            rows,request.mobile_ids,true);
        const auto nominal_map=task10p11s_capture_detail::nominalFromJson(
            snapshot.at("nominal_controls"));
        const auto applied=task10p11afAppliedControlsFromCheckpoint(snapshot);
        const auto ordered=task10p11sOrderedControls(request.mobile_ids,applied);
        result.owner_local_gamma_mps2=minimumOwnerLocalGamma(rows,request);
        result.signed_transfer=solveTask10p11tDynamicPair(
            rows,request.mobile_ids,nominal_map,
            request.acceleration_half_box,pair_id);
        const auto nominal=task10p11sOrderedControls(
            request.mobile_ids,nominal_map);
        const std::set<NodeId> component{
            result.signed_transfer.pair.first.owner,
            result.signed_transfer.pair.second.owner};
        result.component_margin=task10p11w_detail::solveRestricted(
            problem,nominal,ordered,component,true);
        const std::set<NodeId> all(request.mobile_ids.begin(),
                                   request.mobile_ids.end());
        result.current_full_pair_margin=task10p11w_detail::solveRestricted(
            problem,nominal,nominal,all,true);
        result.applied_current_full_row_residual_mps2=
            independentMinimumResidual(problem,ordered);
        result.applied_successor_full_pair=successorFullPair(snapshot,applied);
        result.g1=task10p11afDecideTakeover(
            snapshot,applied,Task10p11aaGraphDepth::H1,true);
        result.g2=task10p11afDecideTakeover(
            snapshot,applied,Task10p11aaGraphDepth::H2FiniteWitness,true);
        result.current_full_row_count=problem.rows.size();
        result.time_s=snapshot.at("runtime").at("runtime_s").get<double>();
        result.valid=true;
        result.reason="all_margin_layers_reconstructed";
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    return result;
}

inline nlohmann::json task10p11afTakeoverJson(
    const Task10p11afTakeoverDecision& value) {
    return {{"valid",value.valid},{"reason",value.reason},
        {"current_distributed_feasible",value.current_distributed_feasible},
        {"successor_distributed_feasible",value.successor_distributed_feasible},
        {"takeover_required",value.takeover_required},
        {"takeover_applied",value.takeover_applied},
        {"fail_closed",value.fail_closed},
        {"current_distributed_full_row_residual_mps2",
            task10p11w_detail::number(value.current_distributed_residual_mps2)},
        {"successor_distributed_global_gamma_mps2",
            task10p11w_detail::number(value.successor_distributed_gamma_mps2)},
        {"successor_distributed_full_row_residual_mps2",
            task10p11w_detail::number(value.successor_distributed_residual_mps2)},
        {"oracle",task10p11aaGraphDecisionJson(value.oracle)}};
}

inline nlohmann::json task10p11afLayerAuditJson(
    const Task10p11afLayerAudit& value) {
    const auto margin=[](const task10p11w_detail::RestrictedResult& item) {
        return nlohmann::json{{"status",item.status},{"feasible",item.feasible},
            {"gamma_mps2",task10p11w_detail::number(item.margin)},
            {"minimum_full_row_residual_mps2",
                task10p11w_detail::number(item.minimum_residual)},
            {"limiting_row_id",item.limiting_row}};
    };
    return {{"valid",value.valid},{"reason",value.reason},
        {"time_s",value.time_s},
        {"owner_local_gamma_mps2",
            task10p11w_detail::number(value.owner_local_gamma_mps2)},
        {"signed_transfer",task10p11w_detail::localIntervalJson(
            value.signed_transfer)},
        {"component_margin",margin(value.component_margin)},
        {"current_full_pair_global",margin(value.current_full_pair_margin)},
        {"applied_current_full_row_residual_mps2",
            task10p11w_detail::number(
                value.applied_current_full_row_residual_mps2)},
        {"applied_successor_full_pair_global",margin(
            value.applied_successor_full_pair.margin)},
        {"current_full_row_count",value.current_full_row_count},
        {"g1",task10p11afTakeoverJson(value.g1)},
        {"g2",task10p11afTakeoverJson(value.g2)},
        {"margin_layers_are_interchangeable",false},
        {"full_pair_model","fixed_topology_once_reserve_28d_1113_rows"},
        {"recursive_feasibility_claimed",false}};
}

}  // namespace gf
