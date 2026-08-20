#pragma once

#include "grand_finale/Task10p11vRestartCheckpoint.hpp"
#include "grand_finale/Task10p11wConflictComponentOracle.hpp"

#include <filesystem>
#include <map>

namespace gf {

inline Task10p11tPairRows task10p11aaPairRowsById(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids,const std::string& pair_id) {
    std::vector<const CanonicalHardRow*> halves;
    for (const auto& row:rows) {
        if (!row.peer.has_value() ||
            std::find(mobile_ids.begin(),mobile_ids.end(),*row.peer)==
                mobile_ids.end()) continue;
        const bool pair_kind=row.kind==CanonicalHardRowKind::ReferenceDistance ||
            row.kind==CanonicalHardRowKind::Collision;
        if (pair_kind && task10p11sPairBaseId(row.id)==pair_id)
            halves.push_back(&row);
    }
    if (halves.size()!=2)
        throw std::runtime_error("monitored pair absent:"+pair_id);
    return makeTask10p11tPairRows(*halves[0],*halves[1]);
}

struct Task10p11aaMarginAuditFrame {
    bool valid=false;
    std::string reason;
    double time_s=0.0;
    std::map<NodeId,double> owner_local_gamma_mps2;
    double minimum_owner_local_gamma_mps2=
        std::numeric_limits<double>::quiet_NaN();
    Task10p11tDynamicPairResult signed_transfer;
    std::string monitored_pair;
    std::set<NodeId> diagnostic_component;
    task10p11w_detail::RestrictedResult pair_component_margin;
    task10p11w_detail::RestrictedResult triplet_component_margin;
    task10p11w_detail::RestrictedResult full_pair_margin;
    std::vector<NodeId> canonical_frozen_local_infeasible_owners;
    std::size_t full_row_count=0;
    bool margin_layers_are_interchangeable=false;
    bool oracle_successor_performed=false;
    std::string oracle_successor_reason;
    double oracle_successor_minimum_owner_local_gamma_mps2=
        std::numeric_limits<double>::quiet_NaN();
    Task10p11tDynamicPairResult oracle_successor_signed_transfer;
    task10p11w_detail::RestrictedResult oracle_successor_component_margin;
    task10p11w_detail::RestrictedResult oracle_successor_full_pair_margin;
    std::size_t oracle_successor_full_row_count=0;
};

inline Task10p11aaMarginAuditFrame task10p11aaAuditPackedCheckpoint(
    const std::filesystem::path& path) {
    Task10p11aaMarginAuditFrame result;
    try {
        const auto snapshot=readTask10p11vJson(path);
        const auto validation=validateTask10p11sSnapshot(snapshot);
        if (!validation.complete) {
            result.reason="packed_snapshot_incomplete";
            return result;
        }
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto rows=buildCanonicalHardRows(request);
        const auto nominal_map=task10p11s_capture_detail::nominalFromJson(
            snapshot.at("nominal_controls"));
        const auto nominal=task10p11sOrderedControls(
            request.mobile_ids,nominal_map);
        const auto problem=buildTask10p11sRows28d(
            rows,request.mobile_ids,true);
        const std::string profile=snapshot.at("task10p11z")
            .at("profile").get<std::string>();
        result.monitored_pair=profile=="R2"?"collision:4--11":
                                              task10p11w_detail::kPair;
        result.diagnostic_component=profile=="R2"
            ?std::set<NodeId>{4,11}:std::set<NodeId>{2,4};
        const auto pair_rows=task10p11aaPairRowsById(
            rows,request.mobile_ids,result.monitored_pair);
        std::map<NodeId,Eigen::Vector2d> frozen_map;
        for (NodeId owner:request.mobile_ids) {
            const auto replay=task10p11t_detail::localReplay(
                rows,pair_rows,owner,0.0,request.acceleration_half_box,
                nominal_map.at(owner));
            if (!replay.feasible)
                result.canonical_frozen_local_infeasible_owners.push_back(owner);
            frozen_map.emplace(owner,replay.feasible?replay.control:
                                                nominal_map.at(owner));
        }
        const auto frozen=task10p11sOrderedControls(
            request.mobile_ids,frozen_map);
        result.minimum_owner_local_gamma_mps2=
            std::numeric_limits<double>::infinity();
        for (NodeId owner:request.mobile_ids) {
            const auto gamma=solveCanonicalGammaStar(
                rows,owner,request.acceleration_half_box);
            if (!gamma.valid || !std::isfinite(gamma.gamma))
                throw std::runtime_error("owner_local_gamma_invalid:"+
                    std::to_string(owner));
            result.owner_local_gamma_mps2[owner]=gamma.gamma;
            result.minimum_owner_local_gamma_mps2=std::min(
                result.minimum_owner_local_gamma_mps2,gamma.gamma);
        }
        result.signed_transfer=solveTask10p11tDynamicPair(
            rows,request.mobile_ids,nominal_map,
            request.acceleration_half_box,result.monitored_pair);
        result.pair_component_margin=task10p11w_detail::solveRestricted(
            problem,nominal,frozen,result.diagnostic_component,true);
        if (profile=="R3")
            result.triplet_component_margin=task10p11w_detail::solveRestricted(
                problem,nominal,frozen,{2,4,6},true);
        else
            result.triplet_component_margin.status=
                "not_applicable_no_registered_expanded_component";
        const std::set<NodeId> all(request.mobile_ids.begin(),
                                   request.mobile_ids.end());
        result.full_pair_margin=task10p11w_detail::solveRestricted(
            problem,nominal,nominal,all,true);
        result.full_row_count=problem.rows.size();
        const bool current_global_hard_feasible=
            result.full_pair_margin.feasible &&
            result.full_pair_margin.margin>=-1.0e-8 &&
            result.full_pair_margin.minimum_residual>=-1.0e-8;
        if (current_global_hard_feasible) {
            const auto control_map=task10p11sControlMap(
                request.mobile_ids,result.full_pair_margin.controls);
            const auto successor_request=rebuildTask10p11sSuccessorRequest(
                snapshot,control_map);
            const auto successor_rows=buildCanonicalHardRows(successor_request);
            const auto successor_problem=buildTask10p11sRows28d(
                successor_rows,successor_request.mobile_ids,true);
            result.oracle_successor_minimum_owner_local_gamma_mps2=
                std::numeric_limits<double>::infinity();
            for (NodeId owner:successor_request.mobile_ids) {
                const auto gamma=solveCanonicalGammaStar(successor_rows,owner,
                    successor_request.acceleration_half_box);
                if (!gamma.valid || !std::isfinite(gamma.gamma))
                    throw std::runtime_error("successor owner gamma invalid");
                result.oracle_successor_minimum_owner_local_gamma_mps2=
                    std::min(result.oracle_successor_minimum_owner_local_gamma_mps2,
                             gamma.gamma);
            }
            result.oracle_successor_signed_transfer=solveTask10p11tDynamicPair(
                successor_rows,successor_request.mobile_ids,control_map,
                successor_request.acceleration_half_box,result.monitored_pair);
            std::map<NodeId,Eigen::Vector2d> successor_frozen_map;
            const auto successor_pair=task10p11aaPairRowsById(
                successor_rows,successor_request.mobile_ids,
                result.monitored_pair);
            for (NodeId owner:successor_request.mobile_ids) {
                const auto replay=task10p11t_detail::localReplay(
                    successor_rows,successor_pair,owner,0.0,
                    successor_request.acceleration_half_box,control_map.at(owner));
                successor_frozen_map.emplace(owner,replay.feasible?replay.control:
                    control_map.at(owner));
            }
            const auto successor_frozen=task10p11sOrderedControls(
                successor_request.mobile_ids,successor_frozen_map);
            const auto successor_nominal=task10p11sOrderedControls(
                successor_request.mobile_ids,control_map);
            result.oracle_successor_component_margin=
                task10p11w_detail::solveRestricted(successor_problem,
                    successor_nominal,successor_frozen,
                    result.diagnostic_component,true);
            const std::set<NodeId> successor_all(
                successor_request.mobile_ids.begin(),
                successor_request.mobile_ids.end());
            result.oracle_successor_full_pair_margin=
                task10p11w_detail::solveRestricted(successor_problem,
                    successor_nominal,successor_nominal,successor_all,true);
            result.oracle_successor_full_row_count=successor_problem.rows.size();
            result.oracle_successor_performed=true;
            result.oracle_successor_reason=
                "advanced_current_global_maximum_margin_endpoint_exact_zoh";
        } else {
            result.oracle_successor_reason=
                "current_full_pair_global_hard_infeasible";
        }
        result.time_s=snapshot.at("runtime").at("runtime_s").get<double>();
        result.valid=true;
        result.reason="evaluated_distinct_margin_layers";
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    return result;
}

inline nlohmann::json task10p11aaMarginAuditJson(
    const Task10p11aaMarginAuditFrame& value) {
    const auto margin=[](const task10p11w_detail::RestrictedResult& item) {
        const bool hard_feasible=item.feasible && std::isfinite(item.margin) &&
            item.margin>=-1.0e-8 && item.minimum_residual>=-1.0e-8;
        return nlohmann::json{{"status",item.status},
            {"optimizer_solution_available",item.feasible},
            {"hard_feasible_at_gamma_zero",hard_feasible},
            {"margin_mps2",task10p11w_detail::number(item.margin)},
            {"minimum_full_row_residual_mps2",
                task10p11w_detail::number(item.minimum_residual)},
            {"limiting_row_id",item.limiting_row}};
    };
    nlohmann::json local=nlohmann::json::object();
    for (const auto& [owner,gamma]:value.owner_local_gamma_mps2)
        local[std::to_string(owner)]={{"gamma_mps2",gamma},
            {"feasible",gamma>=0.0}};
    return {{"valid",value.valid},{"reason",value.reason},
        {"time_s",value.time_s},{"owner_local",local},
        {"minimum_owner_local_gamma_mps2",
            task10p11w_detail::number(value.minimum_owner_local_gamma_mps2)},
        {"signed_transfer",task10p11w_detail::localIntervalJson(
            value.signed_transfer)},
        {"monitored_pair",value.monitored_pair},
        {"diagnostic_component",task10p11w_detail::idsJson(
            value.diagnostic_component)},
        {"component_seed",margin(value.pair_component_margin)},
        {"component_triplet_2_4_6",margin(value.triplet_component_margin)},
        {"full_pair_global",margin(value.full_pair_margin)},
        {"full_row_count",value.full_row_count},
        {"margin_layers_are_interchangeable",false},
        {"component_freeze_semantics",
            "same_frame_canonical_local_controls_outside_component_with_nominal_placeholder_for_already_infeasible_owner"},
        {"canonical_frozen_local_infeasible_owners",
            value.canonical_frozen_local_infeasible_owners},
        {"oracle_successor_from_current_global_maximum_margin",{
            {"performed",value.oracle_successor_performed},
            {"reason",value.oracle_successor_reason},
            {"minimum_owner_local_gamma_mps2",task10p11w_detail::number(
                value.oracle_successor_minimum_owner_local_gamma_mps2)},
            {"signed_transfer",task10p11w_detail::localIntervalJson(
                value.oracle_successor_signed_transfer)},
            {"component_seed",margin(
                value.oracle_successor_component_margin)},
            {"full_pair_global",margin(
                value.oracle_successor_full_pair_margin)},
            {"full_row_count",value.oracle_successor_full_row_count},
            {"command_semantics","current_full_pair_global_maximum_margin_endpoint"},
            {"recursive_feasibility_claimed",false}}}};
}

}  // namespace gf
