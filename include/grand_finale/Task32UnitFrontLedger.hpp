#pragma once
#include "grand_finale/Task20CoveragePolicy.hpp"
#include <optional>

namespace gf {
struct Task32UnitFrontLedger {
    FrontierCell task;
    Eigen::Vector2d applied_front=Eigen::Vector2d::Zero();
    bool active=false;
};
struct Task32UnitFrontResult {
    bool valid=false;
    std::string reason;
    std::map<std::string,Task32UnitFrontLedger> units;
    std::map<NodeId,FrontierCell> targets;
};

// Pure task/motion bookkeeping, not a safety or information qualification.
// No initialized reference is fabricated for a never-assigned inactive unit.
inline Task32UnitFrontResult task32AdvanceUnitFrontLedger(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& anchors,
    const std::map<std::string,Task20CoverageAssignment>& assignments,
    const std::set<std::string>& certified_uncovered_ids,
    const std::map<std::string,Task32UnitFrontLedger>& previous,
    bool allocation_evaluated,std::optional<double> maximum_front_step_m) {
    Task32UnitFrontResult out;
    const auto fail=[&](const std::string& reason) {
        Task32UnitFrontResult result;result.reason=reason;return result;
    };
    if(!contract.valid || (maximum_front_step_m &&
       (!std::isfinite(*maximum_front_step_m)||*maximum_front_step_m<0)))
        return fail("invalid_front_ledger_request");
    std::set<std::string> unit_ids,assigned_ids;
    for(const auto& u:contract.coverage_units)unit_ids.insert(u.id);
    for(const auto& [id,state]:previous)
        if(!unit_ids.count(id)||state.task.x_index<0||state.task.y_index<0||
            !state.task.center.allFinite()||!state.applied_front.allFinite())
            return fail("invalid_previous_front_ledger");
    if(!allocation_evaluated&&!assignments.empty())return fail("assignment_without_allocation");
    for(const auto& [id,a]:assignments) {
        if(!unit_ids.count(id)||a.coverage_unit!=id||a.task.x_index<0||a.task.y_index<0||
           !a.front.allFinite()||!a.task.center.allFinite()||
           (a.front-a.task.center).norm()!=0||!certified_uncovered_ids.count(a.task.id())||
           !assigned_ids.insert(a.task.id()).second)
            return fail("nonreal_covered_duplicate_or_mismatched_assignment");
    }
    out.units=previous;
    std::map<std::string,Eigen::Vector2d> fronts;
    for(const auto& u:contract.coverage_units) {
        const auto a=assignments.find(u.id);
        if(a!=assignments.end()) {
            if(!out.units.count(u.id))out.units[u.id]={a->second.task,a->second.front,true};
            else {out.units.at(u.id).task=a->second.task;out.units.at(u.id).active=true;}
        } else if(!out.units.count(u.id))return fail("missing_initialized_front_ledger");
        else if(allocation_evaluated)out.units.at(u.id).active=false;
        auto& state=out.units.at(u.id);
        state.active=state.active&&certified_uncovered_ids.count(state.task.id());
        // C changes reference progression only: a covered historical task
        // ceases to be active, but its inherited motion is not event-paused.
        if(state.active || maximum_front_step_m.has_value()) {
            const Eigen::Vector2d delta=state.task.center-state.applied_front;
            const double length=delta.norm();
            if(!maximum_front_step_m || length<=*maximum_front_step_m)state.applied_front=state.task.center;
            else if(length>0)state.applied_front+=delta*(*maximum_front_step_m/length);
        }
        fronts[u.id]=state.applied_front;
    }
    const auto lifted=task20LiftTargets(contract,anchors,fronts);
    if(!lifted.valid)return fail(lifted.reason);
    for(const auto& u:contract.coverage_units)for(const auto id:u.members) {
        auto target=out.units.at(u.id).task;target.center=lifted.targets.at(id);out.targets[id]=target;
    }
    out.valid=true;out.reason="real_task_and_shared_motion_ledger";return out;
}
} // namespace gf
