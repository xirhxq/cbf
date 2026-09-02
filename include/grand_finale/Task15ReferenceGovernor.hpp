#pragma once

#include "grand_finale/Types.hpp"

#include <Eigen/Dense>
#include <array>
#include <map>
#include <string>

namespace gf {

struct Task15ReferenceGovernorResult {
    bool valid=false;
    bool reselect_required=false;
    std::string reason;
    double common_fraction=0.0;
    std::map<NodeId,Eigen::Vector2d> targets;
    std::size_t feasibility_evaluations=0;
};

template<class FeasibleFn>
Task15ReferenceGovernorResult task15AdvanceReferenceGovernor(
    const std::map<NodeId,Eigen::Vector2d>& current,
    const std::map<NodeId,Eigen::Vector2d>& nominal,FeasibleFn&& feasible) {
    Task15ReferenceGovernorResult result;
    if (current.size()!=14 || nominal.size()!=14) {
        result.reason="incomplete_task15_governor_ledger";
        return result;
    }
    static constexpr std::array<double,7> fractions{
        1.0,0.5,0.25,0.125,0.0625,0.03125,0.015625};
    for (const double fraction:fractions) {
        std::map<NodeId,Eigen::Vector2d> candidate;
        bool finite=true;
        for (const auto& [owner,target]:nominal) {
            const auto found=current.find(owner);
            if (found==current.end()) { finite=false; break; }
            candidate[owner]=found->second+fraction*(target-found->second);
            finite=finite&&candidate[owner].allFinite();
        }
        ++result.feasibility_evaluations;
        if (!finite || !feasible(candidate,fraction)) continue;
        result.valid=true;
        result.reason="maximum_registered_reference_safe_step";
        result.common_fraction=fraction;
        result.targets=std::move(candidate);
        return result;
    }
    result.reselect_required=true;
    result.reason="no_positive_reference_safe_step";
    return result;
}

}  // namespace gf
