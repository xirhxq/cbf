#pragma once

#include "grand_finale/Task10p11agFullDomainPredecessor.hpp"
#include "grand_finale/Task10p11zComponentOracle.hpp"

#include <functional>

namespace gf {

namespace task10p11ag_component_detail {

constexpr double kTolerance=1.0e-8;
constexpr const char* kSeedPair="collision:2--9";

inline Task10p11tPairRows pairRows(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids,const std::string& pair_id) {
    std::vector<const CanonicalHardRow*> halves;
    for (const auto& row:rows) {
        if (!row.peer.has_value() ||
            std::find(mobile_ids.begin(),mobile_ids.end(),*row.peer)==
                mobile_ids.end()) continue;
        const bool physical=row.kind==CanonicalHardRowKind::ReferenceDistance ||
            row.kind==CanonicalHardRowKind::Collision;
        if (physical && task10p11sPairBaseId(row.id)==pair_id)
            halves.push_back(&row);
    }
    if (halves.size()!=2)
        throw std::runtime_error("component seed pair absent:"+pair_id);
    return makeTask10p11tPairRows(*halves[0],*halves[1]);
}

inline Eigen::VectorXd successorOutsideLocalControls(
    const std::vector<CanonicalHardRow>& rows,
    const CanonicalHardRowRequest& request,
    const std::map<NodeId,Eigen::Vector2d>& nominal,
    const std::string& pair_id) {
    const auto pair=pairRows(rows,request.mobile_ids,pair_id);
    std::map<NodeId,Eigen::Vector2d> controls;
    for (NodeId owner:request.mobile_ids) {
        const auto replay=task10p11t_detail::localReplay(
            rows,pair,owner,0.0,request.acceleration_half_box,
            nominal.at(owner));
        if (!replay.feasible)
            throw std::runtime_error("successor canonical local QP infeasible:"+
                                     std::to_string(owner));
        controls.emplace(owner,replay.control);
    }
    return task10p11sOrderedControls(request.mobile_ids,controls);
}

inline std::vector<std::set<NodeId>> connectedComponentsAtSize(
    const std::vector<NodeId>& owners,std::size_t size,
    const std::map<NodeId,std::set<NodeId>>& graph,
    const std::set<NodeId>& seed) {
    std::vector<NodeId> optional;
    for (NodeId owner:owners)
        if (seed.count(owner)==0) optional.push_back(owner);
    std::vector<std::set<NodeId>> output;
    if (size<seed.size() || size-seed.size()>optional.size()) return output;
    std::vector<NodeId> chosen;
    std::function<void(std::size_t,std::size_t)> enumerate=
        [&](std::size_t offset,std::size_t remaining) {
            if (remaining==0) {
                auto component=seed;
                component.insert(chosen.begin(),chosen.end());
                if (task10p11z_component_detail::connected(component,graph))
                    output.push_back(std::move(component));
                return;
            }
            if (optional.size()-offset<remaining) return;
            for (std::size_t index=offset;
                 index+remaining<=optional.size();++index) {
                chosen.push_back(optional[index]);
                enumerate(index+1,remaining-1);
                chosen.pop_back();
            }
        };
    enumerate(0,size-seed.size());
    return output;
}

struct Attempt {
    std::set<NodeId> component;
    task10p11w_detail::RestrictedResult current;
    task10p11w_detail::RestrictedResult successor;
    task10p11w_detail::RestrictedResult successor_full_margin;
    bool current_feasible=false;
    bool successor_feasible=false;
    std::size_t current_row_count=0;
    std::size_t successor_row_count=0;
    double coverage_deviation=std::numeric_limits<double>::infinity();
};

inline Attempt evaluateComponent(const nlohmann::json& snapshot,
    const Eigen::VectorXd& distributed,const std::set<NodeId>& component) {
    Attempt result;
    result.component=component;
    const auto request=task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto rows=buildCanonicalHardRows(request);
    const auto problem=buildTask10p11sRows28d(rows,request.mobile_ids,true);
    result.current_row_count=problem.rows.size();
    result.current=task10p11w_detail::solveRestricted(
        problem,distributed,distributed,component,false);
    result.current_feasible=result.current.feasible &&
        result.current.minimum_residual>=-kTolerance;
    if (!result.current_feasible) return result;
    result.coverage_deviation=(result.current.controls-distributed).norm();
    const auto controls=task10p11sControlMap(
        request.mobile_ids,result.current.controls);
    const auto estimate0=task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    const auto estimate1=task10p11aa_detail::predictEstimate(
        snapshot,estimate0,controls);
    const auto request1=task10p11x_detail::requestAtEstimate(
        snapshot,estimate1);
    const auto rows1=buildCanonicalHardRows(request1);
    const auto problem1=buildTask10p11sRows28d(
        rows1,request1.mobile_ids,true);
    result.successor_row_count=problem1.rows.size();
    const std::set<NodeId> all(request1.mobile_ids.begin(),
                               request1.mobile_ids.end());
    result.successor_full_margin=task10p11w_detail::solveRestricted(
        problem1,result.current.controls,result.current.controls,all,true);
    if (!result.successor_full_margin.feasible ||
        result.successor_full_margin.margin< -kTolerance)
        return result;
    const auto frozen=successorOutsideLocalControls(
        rows1,request1,controls,kSeedPair);
    result.successor=task10p11w_detail::solveRestricted(
        problem1,result.current.controls,frozen,component,false);
    result.successor_feasible=result.successor.feasible &&
        result.successor.minimum_residual>=-kTolerance &&
        result.successor_full_margin.feasible &&
        result.successor_full_margin.margin>=-kTolerance;
    return result;
}

}  // namespace task10p11ag_component_detail

struct Task10p11agComponentGate {
    bool valid=false;
    std::string reason;
    std::set<NodeId> selected_component;
    std::size_t frozen_maximum_component_size=0;
    std::map<NodeId,std::set<NodeId>> constraint_graph;
    std::vector<task10p11ag_component_detail::Attempt> attempts;
};

inline Task10p11agComponentGate runTask10p11agComponentGate(
    const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& distributed_controls,
    std::size_t size_limit=13) {
    using namespace task10p11ag_component_detail;
    Task10p11agComponentGate gate;
    try {
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto problem=buildTask10p11sRows28d(
            buildCanonicalHardRows(request),request.mobile_ids,true);
        gate.constraint_graph=task10p11z_component_detail::coupledGraph(problem);
        const Eigen::VectorXd distributed=task10p11sOrderedControls(
            request.mobile_ids,distributed_controls);
        const std::set<NodeId> seed{2,9};
        if (!task10p11z_component_detail::connected(seed,
                gate.constraint_graph))
            throw std::runtime_error("collision_2_9_seed_is_not_connected");
        for (std::size_t size=seed.size();size<=size_limit;++size) {
            std::optional<std::size_t> best;
            for (const auto& component:connectedComponentsAtSize(
                    request.mobile_ids,size,gate.constraint_graph,seed)) {
                gate.attempts.push_back(evaluateComponent(
                    snapshot,distributed,component));
                const std::size_t index=gate.attempts.size()-1;
                if (gate.attempts[index].successor_feasible &&
                    (!best.has_value() ||
                     gate.attempts[index].coverage_deviation<
                        gate.attempts[*best].coverage_deviation))
                    best=index;
            }
            if (best.has_value()) {
                gate.selected_component=gate.attempts[*best].component;
                gate.frozen_maximum_component_size=
                    gate.selected_component.size();
                gate.valid=true;
                gate.reason="protocol_minimum_connected_component_found";
                return gate;
            }
        }
        gate.reason="no_successor_feasible_component_within_registered_limit";
    } catch (const std::exception& error) {
        gate.reason=error.what();
    }
    return gate;
}

inline nlohmann::json task10p11agComponentGateJson(
    const Task10p11agComponentGate& gate) {
    nlohmann::json attempts=nlohmann::json::array();
    for (const auto& attempt:gate.attempts)
        attempts.push_back({{"component",task10p11w_detail::idsJson(
                attempt.component)},
            {"current_feasible",attempt.current_feasible},
            {"successor_feasible",attempt.successor_feasible},
            {"coverage_deviation_l2_mps2",task10p11w_detail::number(
                attempt.coverage_deviation)},
            {"current",task10p11zRestrictedSummaryJson(attempt.current)},
            {"successor_component",task10p11zRestrictedSummaryJson(
                attempt.successor)},
            {"successor_full_pair_margin",task10p11zRestrictedSummaryJson(
                attempt.successor_full_margin)},
            {"current_row_count",attempt.current_row_count},
            {"successor_row_count",attempt.successor_row_count}});
    nlohmann::json graph=nlohmann::json::object();
    for (const auto& [owner,neighbors]:gate.constraint_graph)
        graph[std::to_string(owner)]=std::vector<NodeId>(
            neighbors.begin(),neighbors.end());
    return {{"protocol","task10p11ag-component-gate-v1"},
        {"valid",gate.valid},{"reason",gate.reason},
        {"seed_pair","collision:2--9"},
        {"protocol_minimum_connected_component",
            task10p11w_detail::idsJson(gate.selected_component)},
        {"frozen_maximum_component_size",
            gate.frozen_maximum_component_size},
        {"constraint_graph",std::move(graph)},
        {"attempts",std::move(attempts)},
        {"outside_owner_policy",
            "same_frame_canonical_distributed_then_successor_local_qp"},
        {"claim_boundary",{{"protocol_minimum_not_graph_theoretic_unique",true},
            {"full_14_owner_production_controller",false},
            {"recursive_feasibility_claimed",false}}}};
}

}  // namespace gf
