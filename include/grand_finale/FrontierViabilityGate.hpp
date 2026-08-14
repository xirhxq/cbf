#pragma once

#include "grand_finale/SharedCollisionViableFrontierAllocator.hpp"

#include <map>
#include <string>
#include <vector>

namespace gf {

struct BrakingAgentTarget {
    FrontierAgentState state;
    Eigen::Vector2d target = Eigen::Vector2d::Zero();
};

struct BrakingGateRequest {
    std::vector<BrakingAgentTarget> agents;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    double acceleration_half_box = 0.0;
    double collision_distance_m = 0.0;
    double position_reserve_m = 0.0;
    double velocity_reserve_mps = 0.0;
};

struct BrakingGateResult {
    bool accepted = false;
    std::string reason;
    double minimum_braking_margin = std::numeric_limits<double>::infinity();
};

namespace frontier_viability_detail {
inline double cross(const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    return a.x()*b.y()-a.y()*b.x();
}
inline bool segmentsIntersect(
    const Eigen::Vector2d& a,const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,const Eigen::Vector2d& d) {
    const double o1=cross(b-a,c-a),o2=cross(b-a,d-a);
    const double o3=cross(d-c,a-c),o4=cross(d-c,b-c);
    return o1*o2 <= 1e-12 && o3*o4 <= 1e-12;
}
inline double pointSegmentDistance(
    const Eigen::Vector2d& p,const Eigen::Vector2d& a,
    const Eigen::Vector2d& b) {
    const Eigen::Vector2d delta=b-a;
    if (delta.squaredNorm() <= 1e-18) return (p-a).norm();
    const double t=std::clamp((p-a).dot(delta)/delta.squaredNorm(),0.0,1.0);
    return (p-(a+t*delta)).norm();
}
}  // namespace frontier_viability_detail

inline BrakingGateResult evaluateBrakingGate(const BrakingGateRequest& request) {
    if (request.agents.empty() ||
        !std::isfinite(request.acceleration_half_box) ||
        request.acceleration_half_box <= 0.0 ||
        !std::isfinite(request.collision_distance_m) ||
        request.collision_distance_m < 0.0 ||
        !std::isfinite(request.position_reserve_m) ||
        request.position_reserve_m < 0.0 ||
        !std::isfinite(request.velocity_reserve_mps) ||
        request.velocity_reserve_mps < 0.0) {
        throw std::invalid_argument("invalid braking gate request");
    }
    BrakingGateResult result;
    for (std::size_t i=0;i<request.agents.size();++i) {
        for (std::size_t j=i+1;j<request.agents.size();++j) {
            const auto& first=request.agents[i];
            const auto& second=request.agents[j];
            if ((first.target-second.target).norm() <= 1e-9) {
                result.reason="duplicate_target";
                return result;
            }
            if (frontier_viability_detail::segmentsIntersect(
                    first.state.position,first.target,
                    second.state.position,second.target)) {
                result.reason="segment_crossing";
                return result;
            }
            const Eigen::Vector2d relative=
                first.state.position-second.state.position;
            const double distance=relative.norm();
            if (distance <= 1e-12) {
                result.reason="braking_distance";
                return result;
            }
            const Eigen::Vector2d normal=relative/distance;
            const double closing=std::max(0.0,
                -normal.dot(first.state.velocity-second.state.velocity)+
                request.velocity_reserve_mps);
            const double clearance=distance-request.collision_distance_m-
                request.position_reserve_m;
            const double stop=closing*closing/
                (4.0*request.acceleration_half_box);
            result.minimum_braking_margin=std::min(
                result.minimum_braking_margin,clearance-stop);
        }
        for (const auto& [id,fixed] : request.fixed_positions) {
            (void)id;
            const auto& agent=request.agents[i];
            const Eigen::Vector2d relative=agent.state.position-fixed;
            const double distance=relative.norm();
            if (distance <= 1e-12) {
                result.reason="braking_distance";
                return result;
            }
            const Eigen::Vector2d normal=relative/distance;
            const double closing=std::max(0.0,
                -normal.dot(agent.state.velocity)+request.velocity_reserve_mps);
            const double clearance=distance-request.collision_distance_m-
                request.position_reserve_m;
            const double stop=closing*closing/
                (2.0*request.acceleration_half_box);
            result.minimum_braking_margin=std::min(
                result.minimum_braking_margin,clearance-stop);
            if (frontier_viability_detail::pointSegmentDistance(
                    fixed,agent.state.position,agent.target) <=
                request.collision_distance_m+request.position_reserve_m) {
                result.reason="fixed_segment";
                return result;
            }
        }
    }
    if (result.minimum_braking_margin < -1e-12) {
        result.reason="braking_distance";
        return result;
    }
    result.accepted=true;
    result.reason="accepted";
    return result;
}

struct CurrentBundleGateResult {
    bool accepted = false;
    std::string reason;
    double minimum_residual = std::numeric_limits<double>::infinity();
};

inline CurrentBundleGateResult evaluateCurrentBundleGate(
    const std::vector<CanonicalHardRow>& rows,
    const std::map<NodeId,Eigen::Vector2d>& nominal,
    double acceleration_half_box,
    const ProgressCompatibilityConfig& config) {
    CurrentBundleGateResult result;
    for (const auto& [owner,control] : nominal) {
        const auto exact=evaluateProgressCompatibility(
            rows,owner,control,acceleration_half_box,config);
        result.minimum_residual=std::min(
            result.minimum_residual,exact.minimum_hard_residual);
        if (!exact.polytope_nonempty || !exact.compatible) {
            result.reason=exact.reason;
            return result;
        }
    }
    result.accepted=true;
    result.reason="accepted";
    return result;
}

}  // namespace gf
