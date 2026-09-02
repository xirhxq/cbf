#pragma once

#include "grand_finale/Task16Cbf2026CoveragePolicy.hpp"

#include <array>

namespace gf {

struct Task16FormationGovernorResult {
    bool valid=false;
    std::string reason;
    std::map<std::string,double> common_fraction;
    std::map<NodeId,Eigen::Vector2d> targets;
    std::size_t stalled_squads=0;
    std::size_t feasibility_evaluations=0;
};

inline double task16AnalyticReferenceSpeedMps(
    double speed_limit_mps,double acceleration_half_box_mps2,
    double velocity_gain,double damping_reserve_multiples=1.0) {
    if (!std::isfinite(speed_limit_mps)||speed_limit_mps<=0.0||
        !std::isfinite(acceleration_half_box_mps2)||
        acceleration_half_box_mps2<=0.0||!std::isfinite(velocity_gain)||
        velocity_gain<=0.0||!std::isfinite(damping_reserve_multiples)||
        damping_reserve_multiples<=0.0)
        throw std::invalid_argument("invalid Task 16 reference-rate inputs");
    return std::max(1.0,
        speed_limit_mps-damping_reserve_multiples*
            acceleration_half_box_mps2/velocity_gain);
}

inline double task16AnalyticTrackingEnvelopeM(
    double reference_speed_mps,double velocity_gain,double position_gain,
    double speed_limit_mps,double braking_acceleration_mps2) {
    if (!std::isfinite(reference_speed_mps)||reference_speed_mps<=0.0||
        !std::isfinite(velocity_gain)||velocity_gain<=0.0||
        !std::isfinite(position_gain)||position_gain<=0.0||
        !std::isfinite(speed_limit_mps)||speed_limit_mps<=0.0||
        !std::isfinite(braking_acceleration_mps2)||
        braking_acceleration_mps2<=0.0)
        throw std::invalid_argument("invalid Task 16 tracking envelope");
    return velocity_gain*reference_speed_mps/position_gain+
        speed_limit_mps*speed_limit_mps/
            (2.0*braking_acceleration_mps2);
}

template<class FeasibleFn>
Task16FormationGovernorResult task16AdvanceFormationGovernor(
    const std::map<NodeId,Eigen::Vector2d>& previous,
    const std::map<NodeId,Eigen::Vector2d>& nominal,FeasibleFn&& feasible,
    double maximum_member_step_m=
        std::numeric_limits<double>::infinity()) {
    Task16FormationGovernorResult result;
    if (previous.size()!=14 || nominal.size()!=14) {
        result.reason="incomplete_task16_governor_ledger";
        return result;
    }
    if (!(maximum_member_step_m>0.0)) {
        result.reason="invalid_task16_governor_rate_cap";
        return result;
    }
    const auto squads=task13UnifiedCoverageSquads();
    std::map<std::string,double> cap;
    for (const auto& squad:squads) {
        double maximum_displacement=0.0;
        for (NodeId owner:squad.members) {
            const auto old=previous.find(owner);
            const auto target=nominal.find(owner);
            if (old==previous.end()||target==nominal.end()) {
                result.reason="incomplete_task16_governor_ledger";
                return result;
            }
            maximum_displacement=std::max(maximum_displacement,
                (target->second-old->second).norm());
        }
        cap[squad.name]=maximum_displacement<=1e-15?1.0:
            std::min(1.0,maximum_member_step_m/maximum_displacement);
    }
    static constexpr std::array<double,8> scales{
        1.0,0.5,0.25,0.125,0.0625,0.03125,0.015625,0.0};
    struct Pair { double a=0.0; double b=0.0; };
    std::vector<Pair> pairs;
    for (const double a:scales) for (const double b:scales)
        pairs.push_back({a*cap.at("A"),b*cap.at("B")});
    std::sort(pairs.begin(),pairs.end(),[](const Pair& lhs,const Pair& rhs) {
        const int lhs_active=(lhs.a>0.0)+(lhs.b>0.0);
        const int rhs_active=(rhs.a>0.0)+(rhs.b>0.0);
        if (lhs_active!=rhs_active) return lhs_active>rhs_active;
        const double lhs_min=lhs_active==2?std::min(lhs.a,lhs.b):
            std::max(lhs.a,lhs.b);
        const double rhs_min=rhs_active==2?std::min(rhs.a,rhs.b):
            std::max(rhs.a,rhs.b);
        if (lhs_min!=rhs_min) return lhs_min>rhs_min;
        if (lhs.a+lhs.b!=rhs.a+rhs.b)
            return lhs.a+lhs.b>rhs.a+rhs.b;
        if (lhs.a!=rhs.a) return lhs.a>rhs.a;
        return lhs.b>rhs.b;
    });
    for (const auto pair:pairs) {
        std::map<std::string,double> lambda{{"A",pair.a},{"B",pair.b}};
        std::map<NodeId,Eigen::Vector2d> candidate;
        bool finite=true;
        for (const auto& squad:squads) {
            const double fraction=lambda.at(squad.name);
            for (NodeId owner:squad.members) {
                const auto old=previous.find(owner);
                const auto target=nominal.find(owner);
                if (old==previous.end() || target==nominal.end()) {
                    finite=false;
                    break;
                }
                candidate[owner]=old->second+
                    fraction*(target->second-old->second);
                finite=finite&&candidate[owner].allFinite();
            }
            if (!finite) break;
        }
        ++result.feasibility_evaluations;
        if (!finite || !feasible(candidate,lambda)) continue;
        result.valid=true;
        result.reason=(pair.a==0.0&&pair.b==0.0)
            ?"task16_safe_zero_step":"task16_safe_common_homotopy";
        result.common_fraction=std::move(lambda);
        result.targets=std::move(candidate);
        result.stalled_squads=(pair.a==0.0)+(pair.b==0.0);
        return result;
    }
    result.reason="task16_no_safe_homotopy_pair";
    return result;
}

}  // namespace gf
