#pragma once

#include "grand_finale/SnapshotRobustPairRow.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct PairSuccessorUpperRequest {
    PairwiseSecondOrderState2D first;
    PairwiseSecondOrderState2D second;
    PairwiseSecondOrderRowSpec collision_spec;
    PairwiseSnapshotTube successor_tube;
    double acceleration_half_box = 0.0;
    double dt_s = 0.0;
    double branch_tolerance_mps2 = 1.0e-4;
    std::size_t maximum_boxes = 250000;
};

struct PairSuccessorUpperResult {
    bool valid = false;
    std::string reason;
    double sampled_lower_gamma_mps2 =
        -std::numeric_limits<double>::infinity();
    double certified_upper_gamma_mps2 =
        std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_relative_acceleration = Eigen::Vector2d::Zero();
    std::size_t boxes_examined = 0;
};

namespace task10p11o_detail {

struct RelativeControlBox {
    Eigen::Vector2d lower = Eigen::Vector2d::Zero();
    Eigen::Vector2d upper = Eigen::Vector2d::Zero();
    double gamma_upper = std::numeric_limits<double>::infinity();
};

struct LargestUpperFirst {
    bool operator()(const RelativeControlBox& lhs,
                    const RelativeControlBox& rhs) const {
        return lhs.gamma_upper < rhs.gamma_upper;
    }
};

inline double directionRadius(double position_radius,double distance) {
    if (!(distance>position_radius) || position_radius<0.0)
        throw std::invalid_argument("invalid pair successor direction radius");
    return 2.0*std::sin(0.5*std::asin(position_radius/distance));
}

inline double pairOnlySuccessorGamma(
    const PairSuccessorUpperRequest& request,
    const Eigen::Vector2d& relative_acceleration) {
    const Eigen::Vector2d position(
        request.first.position.x-request.second.position.x,
        request.first.position.y-request.second.position.y);
    const Eigen::Vector2d velocity=
        request.first.velocity-request.second.velocity;
    const double dt=request.dt_s;
    const Eigen::Vector2d successor_position=
        position+dt*velocity+0.5*dt*dt*relative_acceleration;
    const Eigen::Vector2d successor_velocity=
        velocity+dt*relative_acceleration;
    const double distance=successor_position.norm();
    const double position_radius=request.collision_spec.uncertainty+
        request.successor_tube.position_radius_m;
    if (!(distance>position_radius) || distance<1.0e-12)
        return -std::numeric_limits<double>::infinity();
    const Eigen::Vector2d normal=successor_position/distance;
    const double direction_radius=directionRadius(position_radius,distance);
    const double h=distance-position_radius-
        request.collision_spec.distanceLimit;
    const double hdot=normal.dot(successor_velocity)-
        direction_radius*successor_velocity.norm()-
        request.successor_tube.velocity_radius_mps;
    const double central_constant=
        request.collision_spec.lambda1*request.collision_spec.lambda2*h+
        (request.collision_spec.lambda1+request.collision_spec.lambda2)*hdot-
        request.collision_spec.totalReserve;
    const double coefficient_reserve=request.acceleration_half_box*
        std::sqrt(2.0)*direction_radius;
    return request.acceleration_half_box*normal.lpNorm<1>()+
        0.5*central_constant-coefficient_reserve;
}

inline double pairOnlySuccessorGammaUpper(
    const PairSuccessorUpperRequest& request,
    const Eigen::Vector2d& lower,const Eigen::Vector2d& upper) {
    const Eigen::Vector2d centre=0.5*(lower+upper);
    const double control_radius=0.5*(upper-lower).norm();
    const double dt=request.dt_s;
    const double position_perturbation=0.5*dt*dt*control_radius;
    const double velocity_perturbation=dt*control_radius;
    const Eigen::Vector2d current_position(
        request.first.position.x-request.second.position.x,
        request.first.position.y-request.second.position.y);
    const Eigen::Vector2d current_velocity=
        request.first.velocity-request.second.velocity;
    const Eigen::Vector2d centre_position=
        current_position+dt*current_velocity+0.5*dt*dt*centre;
    const Eigen::Vector2d centre_velocity=current_velocity+dt*centre;
    const double centre_distance=centre_position.norm();
    const double distance_lower=centre_distance-position_perturbation;
    const double distance_upper=centre_distance+position_perturbation;
    const double position_radius=request.collision_spec.uncertainty+
        request.successor_tube.position_radius_m;
    if (!(distance_lower>position_radius) || distance_lower<1.0e-12)
        return std::numeric_limits<double>::infinity();
    const Eigen::Vector2d centre_normal=centre_position/centre_distance;
    const double normal_radius=2.0*std::sin(
        0.5*std::asin(std::min(1.0,position_perturbation/distance_lower)));
    const double normal_l1_upper=std::min(
        std::sqrt(2.0),centre_normal.lpNorm<1>()+
        std::sqrt(2.0)*normal_radius);
    const double radial_velocity_upper=centre_normal.dot(centre_velocity)+
        normal_radius*centre_velocity.norm()+velocity_perturbation;
    const double speed_lower=std::max(
        0.0,centre_velocity.norm()-velocity_perturbation);
    const double uncertainty_direction_lower=
        directionRadius(position_radius,distance_upper);
    const double h_upper=distance_upper-position_radius-
        request.collision_spec.distanceLimit;
    const double hdot_upper=radial_velocity_upper-
        uncertainty_direction_lower*speed_lower-
        request.successor_tube.velocity_radius_mps;
    const double central_upper=
        request.collision_spec.lambda1*request.collision_spec.lambda2*h_upper+
        (request.collision_spec.lambda1+request.collision_spec.lambda2)*
            hdot_upper-
        request.collision_spec.totalReserve;
    const double reserve_lower=request.acceleration_half_box*
        std::sqrt(2.0)*uncertainty_direction_lower;
    return request.acceleration_half_box*normal_l1_upper+
        0.5*central_upper-reserve_lower;
}

inline void validate(const PairSuccessorUpperRequest& request) {
    const bool finite_states=
        std::isfinite(request.first.position.x) &&
        std::isfinite(request.first.position.y) &&
        std::isfinite(request.second.position.x) &&
        std::isfinite(request.second.position.y) &&
        request.first.velocity.allFinite() &&
        request.second.velocity.allFinite();
    if (!finite_states ||
        request.collision_spec.kind!=
            PairwiseSecondOrderBarrierKind::CollisionLower ||
        request.collision_spec.k!=1.0 ||
        !std::isfinite(request.collision_spec.distanceLimit) ||
        request.collision_spec.distanceLimit<=0.0 ||
        !std::isfinite(request.collision_spec.uncertainty) ||
        request.collision_spec.uncertainty<0.0 ||
        !std::isfinite(request.collision_spec.totalReserve) ||
        request.collision_spec.totalReserve<0.0 ||
        !std::isfinite(request.collision_spec.lambda1) ||
        request.collision_spec.lambda1<=0.0 ||
        !std::isfinite(request.collision_spec.lambda2) ||
        request.collision_spec.lambda2<=0.0 ||
        !std::isfinite(request.acceleration_half_box) ||
        request.acceleration_half_box<=0.0 ||
        !std::isfinite(request.dt_s) || request.dt_s<=0.0 ||
        !std::isfinite(request.branch_tolerance_mps2) ||
        request.branch_tolerance_mps2<=0.0 || request.maximum_boxes==0 ||
        !std::isfinite(request.successor_tube.position_radius_m) ||
        request.successor_tube.position_radius_m<0.0 ||
        !std::isfinite(request.successor_tube.velocity_radius_mps) ||
        request.successor_tube.velocity_radius_mps<0.0) {
        throw std::invalid_argument("invalid pair successor upper request");
    }
}

}  // namespace task10p11o_detail

// This oracle deliberately relaxes every current and successor hard row except
// the selected mobile-mobile collision pair and the two input boxes.  Its
// result is therefore an upper bound on any conflict-component or swarm-wide
// one-step successor margin.  A negative result certifies impossibility; a
// positive result does not certify that the full joint problem is feasible.
inline PairSuccessorUpperResult certifyRelaxedPairSuccessorGammaUpper(
    const PairSuccessorUpperRequest& request) {
    using namespace task10p11o_detail;
    validate(request);
    PairSuccessorUpperResult result;
    const double relative_half_box=2.0*request.acceleration_half_box;
    const Eigen::Vector2d root_lower=
        Eigen::Vector2d::Constant(-relative_half_box);
    const Eigen::Vector2d root_upper=
        Eigen::Vector2d::Constant(relative_half_box);
    std::priority_queue<RelativeControlBox,std::vector<RelativeControlBox>,
                        LargestUpperFirst> pending;
    pending.push({root_lower,root_upper,
        pairOnlySuccessorGammaUpper(request,root_lower,root_upper)});
    const std::vector<Eigen::Vector2d> initial_samples{
        root_lower,root_upper,
        {root_lower.x(),root_upper.y()},
        {root_upper.x(),root_lower.y()},
        Eigen::Vector2d::Zero()};
    for (const auto& sample:initial_samples) {
        const double gamma=pairOnlySuccessorGamma(request,sample);
        if (gamma>result.sampled_lower_gamma_mps2) {
            result.sampled_lower_gamma_mps2=gamma;
            result.best_relative_acceleration=sample;
        }
    }
    while (!pending.empty() && result.boxes_examined<request.maximum_boxes) {
        const auto box=pending.top();
        pending.pop();
        ++result.boxes_examined;
        if (box.gamma_upper<=result.sampled_lower_gamma_mps2+
                              request.branch_tolerance_mps2) {
            result.certified_upper_gamma_mps2=std::max(
                box.gamma_upper,result.sampled_lower_gamma_mps2);
            result.valid=true;
            result.reason="global_gap_certified";
            return result;
        }
        const Eigen::Vector2d widths=box.upper-box.lower;
        Eigen::Index axis=0;
        widths.maxCoeff(&axis);
        const double split=0.5*(box.lower(axis)+box.upper(axis));
        RelativeControlBox left{box.lower,box.upper,0.0};
        RelativeControlBox right{box.lower,box.upper,0.0};
        left.upper(axis)=split;
        right.lower(axis)=split;
        for (auto* child:{&left,&right}) {
            const Eigen::Vector2d centre=0.5*(child->lower+child->upper);
            const double gamma=pairOnlySuccessorGamma(request,centre);
            if (gamma>result.sampled_lower_gamma_mps2) {
                result.sampled_lower_gamma_mps2=gamma;
                result.best_relative_acceleration=centre;
            }
            child->gamma_upper=pairOnlySuccessorGammaUpper(
                request,child->lower,child->upper);
            pending.push(*child);
        }
    }
    result.reason="branch_budget_exhausted";
    if (!pending.empty())
        result.certified_upper_gamma_mps2=pending.top().gamma_upper;
    return result;
}

}  // namespace gf
