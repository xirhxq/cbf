#pragma once

#include "grand_finale/Types.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace gf {

struct TargetDisc2D {
    Eigen::Vector2d center=Eigen::Vector2d::Zero();
    double radius_m=0.0;
    std::string id;
};

struct TargetLensProjection {
    bool valid=false;
    std::string reason;
    Eigen::Vector2d point=Eigen::Vector2d::Zero();
    double squared_deformation_m2=std::numeric_limits<double>::infinity();
    double maximum_constraint_violation_m=
        std::numeric_limits<double>::infinity();
};

namespace certified_target_lifting_detail {

inline bool lexicographicallyLess(
    const Eigen::Vector2d& lhs,const Eigen::Vector2d& rhs,double tolerance) {
    if (lhs.x()<rhs.x()-tolerance) return true;
    if (rhs.x()<lhs.x()-tolerance) return false;
    return lhs.y()<rhs.y()-tolerance;
}

inline double maximumViolation(
    const Eigen::Vector2d& point,const std::vector<TargetDisc2D>& discs) {
    double maximum=-std::numeric_limits<double>::infinity();
    for (const auto& disc : discs)
        maximum=std::max(maximum,(point-disc.center).norm()-disc.radius_m);
    return maximum;
}

inline std::vector<Eigen::Vector2d> circleIntersections(
    const TargetDisc2D& first,const TargetDisc2D& second,double tolerance) {
    std::vector<Eigen::Vector2d> result;
    const Eigen::Vector2d delta=second.center-first.center;
    const double distance=delta.norm();
    if (distance<=tolerance) return result;
    if (distance>first.radius_m+second.radius_m+tolerance ||
        distance<std::abs(first.radius_m-second.radius_m)-tolerance)
        return result;
    const double along=(first.radius_m*first.radius_m-
        second.radius_m*second.radius_m+distance*distance)/(2.0*distance);
    const double height_squared=std::max(
        0.0,first.radius_m*first.radius_m-along*along);
    const Eigen::Vector2d axis=delta/distance;
    const Eigen::Vector2d normal(-axis.y(),axis.x());
    const Eigen::Vector2d base=first.center+along*axis;
    const double height=std::sqrt(height_squared);
    result.push_back(base+height*normal);
    if (height>tolerance) result.push_back(base-height*normal);
    return result;
}

inline std::uint64_t hashText(const std::string& text) {
    std::uint64_t hash=1469598103934665603ULL;
    for (const unsigned char byte : text) {
        hash^=byte;
        hash*=1099511628211ULL;
    }
    return hash==0?1:hash;
}

}  // namespace certified_target_lifting_detail

inline TargetLensProjection projectOntoTargetLens(
    const Eigen::Vector2d& raw,const std::vector<TargetDisc2D>& discs,
    double feasibility_tolerance_m=1e-10,
    double objective_tolerance_m2=1e-12) {
    TargetLensProjection result;
    if (!raw.allFinite() || discs.empty() ||
        !std::isfinite(feasibility_tolerance_m) ||
        feasibility_tolerance_m<0.0 ||
        !std::isfinite(objective_tolerance_m2) ||
        objective_tolerance_m2<0.0) {
        result.reason="invalid_target_lens_request";
        return result;
    }
    for (const auto& disc : discs)
        if (!disc.center.allFinite() || !std::isfinite(disc.radius_m) ||
            disc.radius_m<=0.0 || disc.id.empty()) {
            result.reason="invalid_target_disc";
            return result;
        }

    std::vector<Eigen::Vector2d> candidates{raw};
    for (const auto& disc : discs) {
        const Eigen::Vector2d delta=raw-disc.center;
        if (delta.norm()>feasibility_tolerance_m)
            candidates.push_back(
                disc.center+disc.radius_m*delta.normalized());
    }
    for (std::size_t first=0;first<discs.size();++first)
        for (std::size_t second=first+1;second<discs.size();++second) {
            const auto intersections=
                certified_target_lifting_detail::circleIntersections(
                    discs[first],discs[second],feasibility_tolerance_m);
            candidates.insert(
                candidates.end(),intersections.begin(),intersections.end());
        }

    for (const auto& candidate : candidates) {
        const double violation=
            certified_target_lifting_detail::maximumViolation(candidate,discs);
        if (!std::isfinite(violation) ||
            violation>feasibility_tolerance_m) continue;
        const double deformation=(candidate-raw).squaredNorm();
        if (!result.valid ||
            deformation<result.squared_deformation_m2-objective_tolerance_m2 ||
            (std::abs(deformation-result.squared_deformation_m2)<=
                 objective_tolerance_m2 &&
             certified_target_lifting_detail::lexicographicallyLess(
                 candidate,result.point,feasibility_tolerance_m))) {
            result.valid=true;
            result.point=candidate;
            result.squared_deformation_m2=deformation;
            result.maximum_constraint_violation_m=violation;
        }
    }
    result.reason=result.valid?"target_lens_projected":"lens_empty";
    return result;
}

struct TargetLiftRequest {
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::vector<DirectedEdge> reference_edges;
    std::map<NodeId,Eigen::Vector2d> raw_targets;
    std::map<NodeId,Eigen::Vector2d> fixed_targets;
    double add_distance_m=849.0;
    double target_margin_m=1.0;
    double feasibility_tolerance_m=1e-10;
    double objective_tolerance_m2=1e-12;
    std::size_t minimum_reference_count=2;
};

struct TargetLiftResult {
    bool valid=false;
    std::string reason;
    double r_plan_m=0.0;
    double maximum_edge_distance_m=0.0;
    std::map<NodeId,Eigen::Vector2d> targets;
    std::vector<NodeId> topological_order;
    std::uint64_t canonical_digest=0;
};

inline std::uint64_t canonicalLiftedTargetDigest(
    double r_plan_m,const std::map<NodeId,Eigen::Vector2d>& targets) {
    std::ostringstream canonical;
    canonical.precision(17);
    canonical<<"r="<<r_plan_m<<';';
    for (const auto& [node,target] : targets)
        canonical<<node<<':'<<target.x()<<','<<target.y()<<';';
    return certified_target_lifting_detail::hashText(canonical.str());
}

inline TargetLiftResult liftReferenceFeasibleTargets(TargetLiftRequest request) {
    TargetLiftResult result;
    std::sort(request.mobile_ids.begin(),request.mobile_ids.end());
    std::sort(request.fixed_ids.begin(),request.fixed_ids.end());
    std::sort(request.reference_edges.begin(),request.reference_edges.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
    result.r_plan_m=request.add_distance_m-request.target_margin_m;
    if (request.mobile_ids.empty() || request.fixed_ids.empty() ||
        request.minimum_reference_count<2 ||
        !std::isfinite(request.add_distance_m) ||
        !std::isfinite(request.target_margin_m) ||
        request.target_margin_m<=request.feasibility_tolerance_m ||
        result.r_plan_m<=0.0 ||
        !std::isfinite(request.feasibility_tolerance_m) ||
        request.feasibility_tolerance_m<0.0 ||
        !std::isfinite(request.objective_tolerance_m2) ||
        request.objective_tolerance_m2<0.0) {
        result.reason="invalid_target_lift_request";
        return result;
    }
    const std::set<NodeId> mobiles(
        request.mobile_ids.begin(),request.mobile_ids.end());
    std::set<NodeId> nodes=mobiles;
    for (NodeId fixed : request.fixed_ids)
        if (!nodes.insert(fixed).second) {
            result.reason="target_node_sets_overlap";
            return result;
        }
    std::map<NodeId,std::size_t> indegree;
    std::map<NodeId,std::vector<NodeId>> successors;
    std::map<NodeId,std::vector<DirectedEdge>> owner_edges;
    for (NodeId node : nodes) indegree[node]=0;
    std::set<std::string> edge_ids;
    for (const auto& edge : request.reference_edges) {
        if (!mobiles.count(edge.owner) || !nodes.count(edge.reference) ||
            !edge_ids.insert(edge.id()).second) {
            result.reason="invalid_target_reference_edge";
            return result;
        }
        ++indegree[edge.owner];
        successors[edge.reference].push_back(edge.owner);
        owner_edges[edge.owner].push_back(edge);
    }
    for (NodeId owner : request.mobile_ids)
        if (owner_edges[owner].size()<request.minimum_reference_count) {
            result.reason="target_reference_count";
            return result;
        }

    std::priority_queue<NodeId,std::vector<NodeId>,std::greater<NodeId>> ready;
    for (const auto& [node,degree] : indegree)
        if (degree==0) ready.push(node);
    while (!ready.empty()) {
        const NodeId node=ready.top();
        ready.pop();
        result.topological_order.push_back(node);
        auto& next=successors[node];
        std::sort(next.begin(),next.end());
        for (NodeId successor : next)
            if (--indegree[successor]==0) ready.push(successor);
    }
    if (result.topological_order.size()!=nodes.size()) {
        result.reason="target_graph_cycle";
        return result;
    }

    for (NodeId fixed : request.fixed_ids) {
        const auto target=request.fixed_targets.find(fixed);
        if (target==request.fixed_targets.end() || !target->second.allFinite()) {
            result.reason="fixed_target_missing";
            return result;
        }
        result.targets[fixed]=target->second;
    }
    for (NodeId node : result.topological_order) {
        if (!mobiles.count(node)) continue;
        const auto raw=request.raw_targets.find(node);
        if (raw==request.raw_targets.end() || !raw->second.allFinite()) {
            result.reason="raw_target_missing:owner="+std::to_string(node);
            return result;
        }
        std::vector<TargetDisc2D> discs;
        for (const auto& edge : owner_edges[node]) {
            const auto reference=result.targets.find(edge.reference);
            if (reference==result.targets.end()) {
                result.reason="target_reference_not_processed";
                return result;
            }
            discs.push_back({reference->second,result.r_plan_m,edge.id()});
        }
        const auto projection=projectOntoTargetLens(
            raw->second,discs,request.feasibility_tolerance_m,
            request.objective_tolerance_m2);
        if (!projection.valid) {
            result.reason=projection.reason+":owner="+std::to_string(node);
            return result;
        }
        result.targets[node]=projection.point;
    }
    for (const auto& edge : request.reference_edges) {
        const double distance=(result.targets.at(edge.owner)-
                               result.targets.at(edge.reference)).norm();
        if (!std::isfinite(distance) ||
            distance>result.r_plan_m+request.feasibility_tolerance_m ||
            distance>request.add_distance_m) {
            result.reason="target_edge_audit_failed:"+edge.id();
            return result;
        }
        result.maximum_edge_distance_m=std::max(
            result.maximum_edge_distance_m,distance);
    }
    result.canonical_digest=canonicalLiftedTargetDigest(
        result.r_plan_m,result.targets);
    result.valid=true;
    result.reason="certified_target_lift";
    return result;
}

}  // namespace gf
