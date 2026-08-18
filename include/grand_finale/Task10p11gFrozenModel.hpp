#pragma once

#include "grand_finale/CanonicalGammaStarFeedback.hpp"
#include "grand_finale/HybridSupervisor.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace gf {

struct Task10p11gFrozenModel {
    double speed_limit_mps = 30.0;
    double acceleration_half_box_mps2 = 4.0;
    double maximum_acceleration_norm_mps2 = 4.0*std::sqrt(2.0);
    double collision_distance_m = 10.0;
    double control_period_s = 0.1;
    double planner_period_s = 1.0;
    double position_gain = 0.4;
    double velocity_gain = 0.8;
    double initial_yaw_rad = M_PI/2.0;
    // Development yaw-input bound.  It is a reproducible model parameter,
    // not a hardware flight-envelope claim inherited from CBF2026.
    double maximum_yaw_rate_radps = 1.0;
    double yaw_gain_per_s = 2.0;
    double coincident_target_tolerance_m = 1.0e-9;
    GammaFeedbackSelectionMode gamma_selection =
        GammaFeedbackSelectionMode::LeastIntervention;
    double predictive_tau_mps2 = 0.0;
};

inline Task10p11gFrozenModel task10p11gFrozenModel() {
    return {};
}

inline double wrapYawRad(double angle) {
    if (!std::isfinite(angle))
        throw std::invalid_argument("yaw angle must be finite");
    double wrapped=std::fmod(angle+M_PI,2.0*M_PI);
    if (wrapped<0.0) wrapped+=2.0*M_PI;
    return wrapped-M_PI;
}

inline double propagateYawExactZoh(
    double yaw_rad,double yaw_rate_radps,double dt_s) {
    if (!std::isfinite(yaw_rate_radps) || !std::isfinite(dt_s) || dt_s<0.0)
        throw std::invalid_argument("yaw ZOH inputs must be finite and dt nonnegative");
    return wrapYawRad(yaw_rad+yaw_rate_radps*dt_s);
}

struct Task10p11gSoftTaskRequest {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double yaw_rad = 0.0;
    std::optional<Eigen::Vector2d> committed_target;
    SupervisorMode mode = SupervisorMode::Hold;
};

struct Task10p11gSoftTaskResult {
    Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
    double yaw_rate_radps = 0.0;
    double desired_yaw_rad = 0.0;
    double yaw_error_rad = 0.0;
    bool target_tracking_active = false;
};

inline Task10p11gSoftTaskResult task10p11gSoftTask(
    const Task10p11gSoftTaskRequest& request,
    const Task10p11gFrozenModel& model=task10p11gFrozenModel()) {
    if (!request.position.allFinite() || !request.velocity.allFinite() ||
        !std::isfinite(request.yaw_rad) ||
        !std::isfinite(model.acceleration_half_box_mps2) ||
        model.acceleration_half_box_mps2<=0.0 ||
        !std::isfinite(model.maximum_yaw_rate_radps) ||
        model.maximum_yaw_rate_radps<=0.0 ||
        !std::isfinite(model.yaw_gain_per_s) || model.yaw_gain_per_s<0.0) {
        throw std::invalid_argument("invalid Task 10.11g soft-task request");
    }
    const bool tracking_mode=request.mode==SupervisorMode::Search;
    Task10p11gSoftTaskResult result;
    result.desired_yaw_rad=wrapYawRad(request.yaw_rad);
    const bool valid_target=request.committed_target.has_value() &&
        request.committed_target->allFinite() &&
        (*request.committed_target-request.position).norm()>
            model.coincident_target_tolerance_m;
    if (tracking_mode && valid_target) {
        const Eigen::Vector2d displacement=
            *request.committed_target-request.position;
        result.acceleration=model.position_gain*displacement-
            model.velocity_gain*request.velocity;
        result.desired_yaw_rad=std::atan2(displacement.y(),displacement.x());
        result.yaw_error_rad=wrapYawRad(
            result.desired_yaw_rad-request.yaw_rad);
        result.yaw_rate_radps=std::clamp(
            model.yaw_gain_per_s*result.yaw_error_rad,
            -model.maximum_yaw_rate_radps,model.maximum_yaw_rate_radps);
        result.target_tracking_active=true;
    } else {
        result.acceleration=-model.velocity_gain*request.velocity;
    }
    for (int axis=0;axis<2;++axis)
        result.acceleration(axis)=std::clamp(result.acceleration(axis),
            -model.acceleration_half_box_mps2,
             model.acceleration_half_box_mps2);
    return result;
}

}  // namespace gf
