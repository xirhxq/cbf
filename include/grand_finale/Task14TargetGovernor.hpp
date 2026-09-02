#pragma once

#include "grand_finale/BoundaryPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace gf {

struct Task14TargetGovernorState {
    Eigen::Vector2d position=Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity=Eigen::Vector2d::Zero();
};

struct Task14TargetGovernorConfig {
    double control_period_s=0.1;
    double braking_acceleration_mps2=4.0;
    double rate_gain=1.0;
    double workspace_guard_m=1.5;
};

struct Task14TargetGovernorResult {
    bool valid=false;
    std::string reason;
    double common_fraction=0.0;
    double minimum_stopping_margin_m=
        std::numeric_limits<double>::infinity();
    std::map<NodeId,Eigen::Vector2d> targets;
};

// A fixed-form viability-aware reference governor.  Every mobile target uses
// the same interpolation fraction, so a formation ledger is not re-shaped by
// the governor.  For each outward workspace component, the transition speed
// is capped by the remaining stopping-distance margin
//
//   m = h - max(n^T v,0)^2/(2 a_b),
//   |Delta r|/dt <= k sqrt(2 a_b max(m,0)).
//
// This is a nominal-reference rule only.  It does not replace the runtime
// robust HOCBF certificate and never changes the committed real cell IDs.
inline Task14TargetGovernorResult task14AdvanceTargetGovernor(
    const std::map<NodeId,Task14TargetGovernorState>& states,
    const std::map<NodeId,Eigen::Vector2d>& current,
    const std::map<NodeId,Eigen::Vector2d>& desired,
    const std::vector<WorkspaceFacet2D>& facets,
    const Task14TargetGovernorConfig& config={}) {
    Task14TargetGovernorResult result;
    if (states.empty() || states.size()!=current.size() ||
        states.size()!=desired.size() || facets.empty() ||
        !(config.control_period_s>0.0) ||
        !(config.braking_acceleration_mps2>0.0) ||
        !(config.rate_gain>0.0) || config.workspace_guard_m<0.0 ||
        !std::isfinite(config.control_period_s) ||
        !std::isfinite(config.braking_acceleration_mps2) ||
        !std::isfinite(config.rate_gain) ||
        !std::isfinite(config.workspace_guard_m)) {
        result.reason="invalid_target_governor_request";
        return result;
    }
    double fraction=1.0;
    for (const auto& [owner,state]:states) {
        const auto old_it=current.find(owner);
        const auto desired_it=desired.find(owner);
        if (old_it==current.end() || desired_it==desired.end() ||
            !state.position.allFinite() || !state.velocity.allFinite() ||
            !old_it->second.allFinite() || !desired_it->second.allFinite()) {
            result.reason="invalid_target_governor_state";
            return result;
        }
        const Eigen::Vector2d delta=desired_it->second-old_it->second;
        for (const auto& facet:facets) {
            const double normal_norm=facet.outward_normal.norm();
            if (!facet.outward_normal.allFinite() ||
                std::abs(normal_norm-1.0)>1e-9 ||
                !std::isfinite(facet.offset_m)) {
                result.reason="invalid_target_governor_facet";
                return result;
            }
            const double h=facet.offset_m-config.workspace_guard_m-
                facet.outward_normal.dot(state.position);
            const double outward_speed=std::max(
                0.0,facet.outward_normal.dot(state.velocity));
            const double stopping_margin=h-outward_speed*outward_speed/
                (2.0*config.braking_acceleration_mps2);
            result.minimum_stopping_margin_m=std::min(
                result.minimum_stopping_margin_m,stopping_margin);
            const double outward_delta=facet.outward_normal.dot(delta);
            if (outward_delta<=0.0) continue;
            const double safe_rate=config.rate_gain*std::sqrt(
                2.0*config.braking_acceleration_mps2*
                std::max(0.0,stopping_margin));
            fraction=std::min(fraction,
                config.control_period_s*safe_rate/outward_delta);
        }
    }
    result.common_fraction=std::clamp(fraction,0.0,1.0);
    for (const auto& [owner,target]:desired)
        result.targets[owner]=current.at(owner)+result.common_fraction*
            (target-current.at(owner));
    result.valid=true;
    result.reason="viability_aware_common_homotopy";
    return result;
}

}  // namespace gf
