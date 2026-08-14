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
