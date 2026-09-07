#pragma once

#include "grand_finale/Task20DagLatticeContract.hpp"

namespace gf {

// Scale only temporary common fronts. Anchors stay fixed. This operation is
// a similarity of all mobile targets only when the labelled lifting commutes
// with scaling about base101; no target-level flight certificate is implied.
inline std::map<std::string,Eigen::Vector2d> task30SimilarityBridgeFronts(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed,
    const std::map<std::string,Eigen::Vector2d>& fronts,double kappa) {
    if(!contract.valid||!std::isfinite(kappa)||kappa<1||!fixed.count(101)||
        !fixed.at(101).allFinite()||fronts.size()!=contract.coverage_units.size())
        throw std::invalid_argument("invalid similarity bridge input");
    const Eigen::Vector2d center=fixed.at(101);
    auto result=fronts;
    for(const auto& unit:contract.coverage_units) {
        if(unit.base_anchors.empty()||!fronts.count(unit.id)||!fronts.at(unit.id).allFinite())
            throw std::invalid_argument("invalid similarity bridge unit");
        Eigen::Vector2d base=Eigen::Vector2d::Zero();
        for(auto id:unit.base_anchors) {
            if(!fixed.count(id)||!fixed.at(id).allFinite())
                throw std::invalid_argument("missing similarity bridge anchor");
            base+=fixed.at(id);
        }
        base/=unit.base_anchors.size();
        if(!base.allFinite()||!(base-center).allFinite())
            throw std::invalid_argument("nonfinite similarity bridge base");
        for(auto id:unit.members) {
            const auto found=contract.member_roles.find(id);
            if(found==contract.member_roles.end())throw std::invalid_argument("missing similarity role");
            const auto& role=found->second;
            const double a=role.axial_fraction+std::abs(role.triangular_fraction)/2;
            const double b=0.86602540378443864676*role.triangular_fraction;
            Eigen::Matrix2d m;m<<a,-b,b,a;
            const Eigen::Vector2d residual=(Eigen::Matrix2d::Identity()-m)*(base-center);
            const double error=residual.norm();
            if(!m.allFinite()||!residual.allFinite()||!std::isfinite(error)||error>1e-9)
                throw std::invalid_argument("lifting is not a base101 similarity");
        }
        // Identity must preserve the original floating-point path exactly.
        if(kappa!=1)result.at(unit.id)=center+kappa*(fronts.at(unit.id)-center);
        if(!result.at(unit.id).allFinite())throw std::invalid_argument("nonfinite similarity bridge front");
    }
    return result;
}

} // namespace gf
