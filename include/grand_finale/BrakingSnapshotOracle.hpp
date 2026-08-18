#pragma once

#include "grand_finale/ProgressCompatibility.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gf {

struct BrakingSnapshotMetric {
    std::string source_id;
    CanonicalHardRowKind kind = CanonicalHardRowKind::Auxiliary;
    std::vector<NodeId> responsibility_owners;
    double robust_distance_m = 0.0;
    double robust_outward_speed_mps = 0.0;
    double available_inward_acceleration_mps2 = 0.0;
    double stop_distance_m = 0.0;
    double stop_time_s = 0.0;
    double braking_slack_m = 0.0;
};

struct BrakingSnapshotOwnerAudit {
    NodeId owner = 0;
    bool hard_polytope_nonempty = false;
};

struct BrakingSnapshotResult {
    bool hard_polytope_nonempty = false;
    bool snapshot_braking_admissible = false;
    bool horizon_sufficient = false;
    std::string reason;
    std::string first_negative_source;
    double minimum_braking_slack_m =
        std::numeric_limits<double>::infinity();
    double maximum_stop_time_s = 0.0;
    std::vector<BrakingSnapshotMetric> metrics;
    std::vector<BrakingSnapshotOwnerAudit> owners;
};

struct MobilePairBrakingAudit {
    bool valid = false;
    std::string reason;
    NodeId first_owner = 0;
    NodeId second_owner = 0;
    bool snapshot_coherent = false;
    bool local_state_constants_coherent = false;
    bool independent_central_verified = false;
    double robust_separation_margin_m = 0.0;
    double robust_radial_closing_speed_mps = 0.0;
    double input_box_relative_separation_support_mps2 = 0.0;
    double full_hard_relative_separation_support_mps2 = 0.0;
    double stop_distance_m = 0.0;
    double stop_time_s = 0.0;
    double braking_slack_m = 0.0;
    double local_coefficient_reserve_sum_mps2 = 0.0;
    Eigen::Vector2d first_coefficient = Eigen::Vector2d::Zero();
    Eigen::Vector2d second_coefficient = Eigen::Vector2d::Zero();
    double first_constant = 0.0;
    double second_constant = 0.0;
    Eigen::Vector2d independent_central_first_coefficient =
        Eigen::Vector2d::Zero();
    Eigen::Vector2d independent_central_second_coefficient =
        Eigen::Vector2d::Zero();
    double independent_central_constant = 0.0;

    double firstLocalResidual(const Eigen::Vector2d& control) const {
        return first_coefficient.dot(control)+first_constant;
    }
    double secondLocalResidual(const Eigen::Vector2d& control) const {
        return second_coefficient.dot(control)+second_constant;
    }
    double centralizedResidual(
        const Eigen::Vector2d& first_control,
        const Eigen::Vector2d& second_control) const {
        return independent_central_first_coefficient.dot(first_control)+
            independent_central_second_coefficient.dot(second_control)+
            independent_central_constant;
    }
};

namespace braking_snapshot_detail {

struct DirectionSupport {
    bool feasible = false;
    double value = -std::numeric_limits<double>::infinity();
};

inline bool satisfies(
    const std::vector<const CanonicalHardRow*>& rows,
    const Eigen::Vector2d& control,double tolerance) {
    for (const auto* row : rows) {
        if (row->margin(control) < -tolerance) return false;
    }
    return true;
}

inline DirectionSupport exactDirectionSupport(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,
    const std::set<std::string>& excluded,
    const Eigen::Vector2d& direction,double tolerance) {
    std::vector<const CanonicalHardRow*> active;
    for (const auto& row : rows) {
        if (row.owner==owner && excluded.count(row.id)==0)
            active.push_back(&row);
    }
    DirectionSupport result;
    const auto consider=[&](const Eigen::Vector2d& candidate) {
        if (!candidate.allFinite() ||
            !satisfies(active,candidate,tolerance)) return;
        const double value=direction.dot(candidate);
        if (!result.feasible || value>result.value) {
            result.feasible=true;
            result.value=value;
        }
    };
    consider(Eigen::Vector2d::Zero());
    for (std::size_t first=0;first<active.size();++first) {
        for (std::size_t second=first+1;second<active.size();++second) {
            const Eigen::Vector2d a=active[first]->control_coefficient;
            const Eigen::Vector2d b=active[second]->control_coefficient;
            const double determinant=a.x()*b.y()-a.y()*b.x();
            const double scale=std::max(1.0,a.norm()*b.norm());
            if (std::abs(determinant)<=64.0*
                    std::numeric_limits<double>::epsilon()*scale) continue;
            const double first_rhs=-active[first]->constant;
            const double second_rhs=-active[second]->constant;
            consider({
                (first_rhs*b.y()-a.y()*second_rhs)/determinant,
                (a.x()*second_rhs-first_rhs*b.x())/determinant});
        }
    }
    return result;
}

inline std::string pairPrefix(const CanonicalHardRow& row) {
    const auto marker=row.id.rfind(":owner:");
    if (marker==std::string::npos)
        throw std::invalid_argument("pair hard row lacks owner suffix");
    return row.id.substr(0,marker);
}

struct MetricGroup {
    std::string source_id;
    CanonicalHardRowKind kind = CanonicalHardRowKind::Auxiliary;
    std::vector<const CanonicalHardRow*> rows;
};

inline BrakingSnapshotMetric buildMetric(
    const MetricGroup& group,const std::vector<CanonicalHardRow>& rows,
    double tolerance) {
    if (group.rows.empty())
        throw std::invalid_argument("empty braking metric group");
    const double distance=group.rows.front()->barrier_h;
    const double hdot=group.rows.front()->barrier_hdot;
    if (!std::isfinite(distance) || !std::isfinite(hdot))
        throw std::invalid_argument("braking oracle requires robust h and hdot");
    std::set<std::string> excluded;
    for (const auto* row : group.rows) excluded.insert(row->id);
    double available=0.0;
    std::vector<NodeId> owners;
    for (const auto* row : group.rows) {
        const auto support=exactDirectionSupport(
            rows,row->owner,excluded,row->control_coefficient,tolerance);
        if (!support.feasible) {
            available=-std::numeric_limits<double>::infinity();
            break;
        }
        available+=support.value-row->coefficient_uncertainty_reserve;
        owners.push_back(row->owner);
    }
    const double outward=std::max(0.0,-hdot);
    double stop_distance=0.0;
    double stop_time=0.0;
    if (outward>tolerance) {
        if (available<=tolerance) {
            stop_distance=std::numeric_limits<double>::infinity();
            stop_time=std::numeric_limits<double>::infinity();
        } else {
            stop_distance=outward*outward/(2.0*available);
            stop_time=outward/available;
        }
    }
    return {group.source_id,group.kind,std::move(owners),distance,outward,
        available,stop_distance,stop_time,distance-stop_distance};
}

inline bool exactOwnerFeasible(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,
    double input_half_box,double tolerance) {
    return evaluateProgressCompatibility(
        rows,owner,Eigen::Vector2d::Zero(),input_half_box,
        {std::numeric_limits<double>::max(),0.0,tolerance,true})
        .polytope_nonempty;
}

}  // namespace braking_snapshot_detail

inline MobilePairBrakingAudit evaluateMobilePairBrakingRequestRows(
    const CanonicalHardRowRequest& request,
    const std::vector<CanonicalHardRow>& rows,
    const UndirectedEdge& raw_edge,
    double tolerance=1e-10) {
    const double input_half_box=request.acceleration_half_box;
    if (!std::isfinite(input_half_box) || input_half_box<=0.0 ||
        !std::isfinite(tolerance) || tolerance<0.0) {
        throw std::invalid_argument("invalid mobile-pair braking audit config");
    }
    using namespace braking_snapshot_detail;
    const UndirectedEdge edge=UndirectedEdge::canonical(
        raw_edge.first,raw_edge.second);
    const std::string prefix="collision:"+edge.id();
    const std::string first_id=prefix+":owner:"+
        std::to_string(edge.first);
    const std::string second_id=prefix+":owner:"+
        std::to_string(edge.second);
    const auto find_row=[&](const std::string& id)
        -> const CanonicalHardRow* {
        const auto found=std::find_if(rows.begin(),rows.end(),
            [&](const auto& row) { return row.id==id; });
        return found==rows.end()?nullptr:&*found;
    };
    const CanonicalHardRow* first=find_row(first_id);
    const CanonicalHardRow* second=find_row(second_id);
    MobilePairBrakingAudit result;
    result.first_owner=edge.first;
    result.second_owner=edge.second;
    if (first==nullptr || second==nullptr ||
        first->kind!=CanonicalHardRowKind::Collision ||
        second->kind!=CanonicalHardRowKind::Collision ||
        first->peer!=std::optional<NodeId>{edge.second} ||
        second->peer!=std::optional<NodeId>{edge.first} ||
        std::abs(first->responsibility-0.5)>tolerance ||
        std::abs(second->responsibility-0.5)>tolerance) {
        result.reason="collision_half_rows_missing_or_wrong_responsibility";
        return result;
    }
    const auto close=[&](double lhs,double rhs) {
        return std::abs(lhs-rhs)<=tolerance*
            std::max({1.0,std::abs(lhs),std::abs(rhs)});
    };
    const bool provenance_same=first->tube_provenance==second->tube_provenance;
    result.snapshot_coherent=
        first->control_coefficient.isApprox(
            -second->control_coefficient,tolerance) &&
        first->normal.isApprox(-second->normal,tolerance) &&
        close(first->barrier_h,second->barrier_h) &&
        close(first->barrier_hdot,second->barrier_hdot) &&
        close(first->barrier_psi1,second->barrier_psi1) &&
        close(first->position_uncertainty_reserve_m,
              second->position_uncertainty_reserve_m) &&
        close(first->velocity_uncertainty_reserve_mps,
              second->velocity_uncertainty_reserve_mps) &&
        close(first->coefficient_uncertainty_reserve,
              second->coefficient_uncertainty_reserve) && provenance_same;
    if (!result.snapshot_coherent) {
        result.reason="collision_half_row_snapshot_incoherent";
        return result;
    }
    result.local_state_constants_coherent=close(
        first->constant+first->coefficient_uncertainty_reserve,
        second->constant+second->coefficient_uncertainty_reserve);
    if (!result.local_state_constants_coherent) {
        result.reason="collision_half_row_state_constant_incoherent";
        return result;
    }
    const auto first_state=request.states.find(edge.first);
    const auto second_state=request.states.find(edge.second);
    const auto tube=request.collision_snapshot_tubes.find(edge.id());
    if (first_state==request.states.end() ||
        second_state==request.states.end() ||
        tube==request.collision_snapshot_tubes.end()) {
        result.reason="collision_independent_central_inputs_missing";
        return result;
    }
    if (request.collision_spec.kind!=
            PairwiseSecondOrderBarrierKind::CollisionLower ||
        request.collision_spec.k!=1.0 ||
        !std::isfinite(request.collision_spec.lambda1) ||
        request.collision_spec.lambda1<=0.0 ||
        !std::isfinite(request.collision_spec.lambda2) ||
        request.collision_spec.lambda2<=0.0) {
        result.reason="collision_independent_central_spec_unsupported";
        return result;
    }
    // Deliberately reconstruct the collision central row from its closed form
    // instead of calling the production pair-row builder.  This keeps the
    // diagnostic independent from produced rows and from their helper.
    const Eigen::Vector2d relative_position{
        first_state->second.position.x-second_state->second.position.x,
        first_state->second.position.y-second_state->second.position.y};
    const Eigen::Vector2d relative_velocity=
        first_state->second.velocity-second_state->second.velocity;
    const double nominal_distance=relative_position.norm();
    const double position_radius=request.collision_spec.uncertainty+
        tube->second.position_radius_m;
    if (!std::isfinite(nominal_distance) ||
        nominal_distance<=position_radius || nominal_distance<1e-9) {
        result.reason="collision_independent_central_radial_singularity";
        return result;
    }
    const Eigen::Vector2d independent_coefficient=
        relative_position/nominal_distance;
    const double angle=std::asin(position_radius/nominal_distance);
    const double direction_radius=2.0*std::sin(0.5*angle);
    const double radial_velocity_uncertainty=
        direction_radius*relative_velocity.norm()+
        tube->second.velocity_radius_mps;
    const double independent_h=nominal_distance-position_radius-
        request.collision_spec.distanceLimit;
    const double independent_hdot=
        independent_coefficient.dot(relative_velocity)-
        radial_velocity_uncertainty;
    const double independent_psi1=
        request.collision_spec.lambda1*independent_h+independent_hdot;
    const double independent_central_constant_lower=
        request.collision_spec.lambda1*request.collision_spec.lambda2*
            independent_h+
        (request.collision_spec.lambda1+request.collision_spec.lambda2)*
            independent_hdot-
        request.collision_spec.totalReserve;
    const double independent_reserve=input_half_box*std::sqrt(2.0)*
        direction_radius;
    const double expected_local_constant=
        0.5*independent_central_constant_lower-independent_reserve;
    const double expected_central_constant=
        independent_central_constant_lower-2.0*independent_reserve;
    result.independent_central_verified=
        first->control_coefficient.isApprox(
            independent_coefficient,tolerance) &&
        second->control_coefficient.isApprox(
            -independent_coefficient,tolerance) &&
        close(first->constant,expected_local_constant) &&
        close(second->constant,expected_local_constant) &&
        close(first->barrier_h,independent_h) &&
        close(first->barrier_hdot,independent_hdot) &&
        close(first->barrier_psi1,independent_psi1) &&
        close(first->coefficient_uncertainty_reserve,
              independent_reserve);
    if (!result.independent_central_verified) {
        result.reason="collision_half_row_independent_central_mismatch";
        return result;
    }
    result.independent_central_first_coefficient=
        independent_coefficient;
    result.independent_central_second_coefficient=
        -independent_coefficient;
    result.independent_central_constant=expected_central_constant;
    result.first_coefficient=first->control_coefficient;
    result.second_coefficient=second->control_coefficient;
    result.first_constant=first->constant;
    result.second_constant=second->constant;
    result.robust_separation_margin_m=first->barrier_h;
    result.robust_radial_closing_speed_mps=std::max(
        0.0,-first->barrier_hdot);
    result.local_coefficient_reserve_sum_mps2=
        first->coefficient_uncertainty_reserve+
        second->coefficient_uncertainty_reserve;
    result.input_box_relative_separation_support_mps2=
        input_half_box*(first->control_coefficient.cwiseAbs().sum()+
                        second->control_coefficient.cwiseAbs().sum());
    const std::set<std::string> excluded{first_id,second_id};
    const auto first_support=exactDirectionSupport(
        rows,edge.first,excluded,first->control_coefficient,tolerance);
    const auto second_support=exactDirectionSupport(
        rows,edge.second,excluded,second->control_coefficient,tolerance);
    if (!first_support.feasible || !second_support.feasible) {
        result.reason="other_hard_rows_empty";
        return result;
    }
    result.full_hard_relative_separation_support_mps2=
        first_support.value+second_support.value-
        result.local_coefficient_reserve_sum_mps2;
    const double speed=result.robust_radial_closing_speed_mps;
    const double acceleration=result.full_hard_relative_separation_support_mps2;
    if (speed<=tolerance) {
        result.stop_distance_m=0.0;
        result.stop_time_s=0.0;
    } else if (acceleration<=tolerance) {
        result.stop_distance_m=std::numeric_limits<double>::infinity();
        result.stop_time_s=std::numeric_limits<double>::infinity();
    } else {
        result.stop_distance_m=speed*speed/(2.0*acceleration);
        result.stop_time_s=speed/acceleration;
    }
    result.braking_slack_m=result.robust_separation_margin_m-
        result.stop_distance_m;
    result.valid=true;
    result.reason="audited";
    return result;
}

inline MobilePairBrakingAudit evaluateMobilePairBraking(
    const CanonicalHardRowRequest& request,
    const UndirectedEdge& raw_edge,double tolerance=1e-10) {
    return evaluateMobilePairBrakingRequestRows(
        request,buildCanonicalHardRows(request),raw_edge,tolerance);
}

inline BrakingSnapshotResult evaluateBrakingSnapshot(
    const CanonicalHardRowRequest& request,
    double available_horizon_s=std::numeric_limits<double>::infinity(),
    double tolerance=1e-10) {
    if (!request.require_snapshot_robust_rows) {
        throw std::invalid_argument(
            "braking snapshot oracle requires an explicit robust tube contract");
    }
    if ((!std::isfinite(available_horizon_s) &&
            available_horizon_s!=std::numeric_limits<double>::infinity()) ||
        available_horizon_s<0.0 || !std::isfinite(tolerance) ||
        tolerance<0.0) {
        throw std::invalid_argument("invalid braking snapshot audit config");
    }
    using namespace braking_snapshot_detail;
    BrakingSnapshotResult result;
    std::vector<CanonicalHardRow> rows=buildCanonicalHardRows(request);
    std::map<std::string,MetricGroup> groups;
    for (const auto& row : rows) {
        if (row.kind==CanonicalHardRowKind::Workspace) {
            groups[row.id]={row.id,row.kind,{&row}};
        } else if (row.kind==CanonicalHardRowKind::Collision ||
                   row.kind==CanonicalHardRowKind::ReferenceDistance) {
            const std::string prefix=pairPrefix(row);
            auto& group=groups[prefix];
            group.source_id=prefix;
            group.kind=row.kind;
            group.rows.push_back(&row);
        }
    }
    for (const auto& [id,group] : groups) {
        result.metrics.push_back(buildMetric(group,rows,tolerance));
        result.minimum_braking_slack_m=std::min(
            result.minimum_braking_slack_m,
            result.metrics.back().braking_slack_m);
        result.maximum_stop_time_s=std::max(
            result.maximum_stop_time_s,result.metrics.back().stop_time_s);
        if (result.first_negative_source.empty() &&
            result.metrics.back().braking_slack_m < -tolerance)
            result.first_negative_source=id;
    }

    result.hard_polytope_nonempty=true;
    for (NodeId owner : request.mobile_ids) {
        BrakingSnapshotOwnerAudit audit;
        audit.owner=owner;
        audit.hard_polytope_nonempty=exactOwnerFeasible(
            rows,owner,request.acceleration_half_box,tolerance);
        result.hard_polytope_nonempty &= audit.hard_polytope_nonempty;
        result.owners.push_back(audit);
    }
    result.snapshot_braking_admissible=result.hard_polytope_nonempty &&
        result.first_negative_source.empty();
    result.horizon_sufficient=result.maximum_stop_time_s<=
        available_horizon_s+tolerance;
    if (!result.hard_polytope_nonempty)
        result.reason="hard_polytope_empty";
    else if (!result.snapshot_braking_admissible)
        result.reason="snapshot_braking_inadmissible";
    else if (!result.horizon_sufficient)
        result.reason="braking_horizon_insufficient";
    else
        result.reason="accepted";
    return result;
}

}  // namespace gf
