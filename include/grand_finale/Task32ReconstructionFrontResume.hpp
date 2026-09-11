#pragma once
#include "grand_finale/Task32UnitFrontLedger.hpp"

namespace gf {
// Exact bookkeeping at the external-reference -> search boundary. This is
// neither an actual-state fit nor a new motion/qualification controller.
inline Task32UnitFrontResult task32ResumeFrontLedger(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& anchors,
    const std::map<NodeId,Eigen::Vector2d>& reference,
    const std::map<std::string,FrontierCell>& historical_tasks) {
    Task32UnitFrontResult out;
    auto fail=[](const std::string& reason) {Task32UnitFrontResult x;x.reason=reason;return x;};
    if(!contract.valid||reference.size()!=contract.member_roles.size()||
       historical_tasks.size()!=contract.coverage_units.size())return fail("incomplete_resume_contract");
    for(const auto& [id,role]:contract.member_roles)
        if(!reference.count(id)||!reference.at(id).allFinite())return fail("invalid_resume_reference");
    std::map<std::string,Eigen::Vector2d> fronts;
    for(const auto& u:contract.coverage_units) {
        if(u.members.empty()||!historical_tasks.count(u.id))return fail("missing_resume_unit");
        const auto& task=historical_tasks.at(u.id);
        if(task.x_index<0||task.y_index<0||!task.center.allFinite())return fail("invalid_resume_task");
        // Deterministic labelled role; validate all other members below.
        const NodeId id=*std::min_element(u.members.begin(),u.members.end());
        const auto inverse=task20FrontForMemberPose(contract,anchors,id,reference.at(id));
        if(!inverse.valid)return fail("noninvertible_resume_role");
        fronts[u.id]=inverse.front;
        out.units[u.id]={task,inverse.front,false};
    }
    const auto lifted=task20LiftTargets(contract,anchors,fronts);
    if(!lifted.valid)return fail("invalid_resume_lifting");
    for(const auto& [id,p]:reference) {
        // Numerical consistency only; not a relaxed physical shape threshold.
        if((lifted.targets.at(id)-p).norm()>1e-7)return fail("reference_outside_final_lifting");
        const auto& unit=contract.member_roles.at(id).coverage_unit;
        auto task=historical_tasks.at(unit);task.center=p;out.targets[id]=task;
    }
    out.valid=true;return out;
}
} // namespace gf
