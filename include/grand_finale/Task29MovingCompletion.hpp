#pragma once

#include "grand_finale/Task20DagLatticeContract.hpp"

namespace gf {

struct Task29MotionState {
    Eigen::Vector2d position=Eigen::Vector2d::Zero(),velocity=Eigen::Vector2d::Zero();
    double position_support=0,velocity_support=0;
};

inline Eigen::Matrix2d task29RoleMatrix(const Task20MemberRole& role) {
    const double a=role.axial_fraction+.5*std::abs(role.triangular_fraction);
    const double b=.86602540378443864676*role.triangular_fraction;
    Eigen::Matrix2d m;m<<a,-b,b,a;return m;
}

struct Task29CompletionAudit {
    bool valid=false,moving_instant_ready=false,legacy_instant_ready=false;
    std::string reason;
    double rms_position_bound=0,maximum_position_bound=0,maximum_absolute_speed_bound=0;
    double maximum_coordinated_speed_bound=0;
    std::map<std::string,Eigen::Vector2d> front_positions,front_velocities;
    std::map<NodeId,double> coordinated_speed_bounds,coordinated_speed_residuals;
};

// Observe only the front members already fixed by the coverage contract.
// No per-member fitted velocity, no truth input, no task/plant mutation.
inline Task29CompletionAudit task29MovingCompletion(
    const Task20DagLatticeContract& contract,const std::map<NodeId,Eigen::Vector2d>& fixed,
    const std::map<NodeId,Eigen::Vector2d>& reference,
    const std::map<NodeId,Task29MotionState>& states,bool graph_handed_off,bool information_ready) {
    Task29CompletionAudit out;
    auto invalid=[&](const std::string& reason) {out.reason=reason;return out;};
    if (!contract.valid||contract.member_roles.empty()||states.size()!=contract.member_roles.size()||
        reference.size()!=states.size()) return invalid("missing_contract_or_members");
    for (const auto& [id,role]:contract.member_roles) {
        if (role.member!=id||!states.count(id)||!reference.count(id)) return invalid("role_state_mismatch");
        const auto& s=states.at(id);
        if (!s.position.allFinite()||!s.velocity.allFinite()||!reference.at(id).allFinite()||
            !std::isfinite(s.position_support)||s.position_support<0||
            !std::isfinite(s.velocity_support)||s.velocity_support<0||
            !task29RoleMatrix(role).allFinite()) return invalid("nonfinite_or_negative_support");
        const double e=(s.position-reference.at(id)).norm()+s.position_support;
        out.rms_position_bound+=e*e;out.maximum_position_bound=std::max(out.maximum_position_bound,e);
        out.maximum_absolute_speed_bound=std::max(out.maximum_absolute_speed_bound,s.velocity.norm()+s.velocity_support);
    }
    out.rms_position_bound=std::sqrt(out.rms_position_bound/states.size());
    std::set<NodeId> accounted;
    for (const auto& unit:contract.coverage_units) {
        const auto front=unit.front_members.empty()?std::vector<NodeId>{unit.leader}:unit.front_members;
        const std::set<NodeId> front_set(front.begin(),front.end());
        if (front.empty()||front_set.size()!=front.size()||unit.members.empty()||unit.base_anchors.empty()||
            out.front_positions.count(unit.id)) return invalid("invalid_unit_support");
        Eigen::Matrix2d mean_matrix=Eigen::Matrix2d::Zero();
        Eigen::Vector2d mean_p=Eigen::Vector2d::Zero(),mean_v=Eigen::Vector2d::Zero(),base=Eigen::Vector2d::Zero();
        for (auto id:unit.base_anchors) {
            if (!fixed.count(id)||!fixed.at(id).allFinite()) return invalid("missing_fixed_anchor");
            base+=fixed.at(id);
        }
        base/=unit.base_anchors.size();
        for (auto id:front) {
            if (!states.count(id)||contract.member_roles.at(id).coverage_unit!=unit.id||
                std::find(unit.members.begin(),unit.members.end(),id)==unit.members.end()) return invalid("invalid_front_member");
            mean_matrix+=task29RoleMatrix(contract.member_roles.at(id));
            mean_p+=states.at(id).position;mean_v+=states.at(id).velocity;
        }
        mean_matrix/=front.size();mean_p/=front.size();mean_v/=front.size();
        if (!mean_matrix.allFinite()||std::abs(mean_matrix.determinant())<1e-12) return invalid("singular_front_mapping");
        const Eigen::Matrix2d inverse=mean_matrix.inverse();
        const Eigen::Vector2d w=inverse*mean_v;
        out.front_positions[unit.id]=base+inverse*(mean_p-base);out.front_velocities[unit.id]=w;
        for (auto id:unit.members) {
            if (!states.count(id)||contract.member_roles.at(id).coverage_unit!=unit.id||!accounted.insert(id).second)
                return invalid("unit_membership_mismatch");
            const auto& s=states.at(id);const Eigen::Matrix2d m=task29RoleMatrix(contract.member_roles.at(id));
            const double residual=(s.velocity-m*w).norm();double support=front_set.count(id)?0:s.velocity_support;
            for (auto j:front) {
                Eigen::Matrix2d block=-m*inverse/double(front.size());
                if (j==id) block+=Eigen::Matrix2d::Identity();
                // Every block is aI+bJ; either column norm is its exact
                // operator 2-norm. Triangle bound needs no independence.
                support+=block.col(0).norm()*states.at(j).velocity_support;
            }
            out.coordinated_speed_residuals[id]=residual;out.coordinated_speed_bounds[id]=residual+support;
            out.maximum_coordinated_speed_bound=std::max(out.maximum_coordinated_speed_bound,residual+support);
        }
    }
    if (accounted.size()!=states.size()) return invalid("unassigned_member");
    const bool shape=out.rms_position_bound<=100&&out.maximum_position_bound<=180;
    out.valid=true;
    out.moving_instant_ready=graph_handed_off&&information_ready&&shape&&out.maximum_coordinated_speed_bound<=3;
    out.legacy_instant_ready=graph_handed_off&&information_ready&&shape&&out.maximum_absolute_speed_bound<=3;
    out.reason=out.moving_instant_ready?"coordinated_shape_ready":"moving_completion_not_ready";
    return out;
}

} // namespace gf
