#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace gf {
struct Task32SourceCvtYawResult {
    bool valid = true;
    bool active = true;
    double rate = 0.;
    double slack = 0.;
    double bearing_drift = 0.;
    double coefficient = 0.;
    double rhs = 0.;
    double h = 0.;
};

inline Task32SourceCvtYawResult task32SourceCvtYaw(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    double yaw, const Eigen::Vector2d& target, double maximum_rate) {
    Task32SourceCvtYawResult out;
    if(!position.allFinite()||!velocity.allFinite()||!target.allFinite()||
       !std::isfinite(yaw)||!std::isfinite(maximum_rate)||maximum_rate<=0.) {
        out.valid=false;out.active=false;return out;
    }
    const auto delta = (target-position).eval();
    if(!delta.allFinite()||!std::isfinite(delta.squaredNorm())) {
        out.valid=false;out.active=false;return out;
    }
    if(delta.squaredNorm()==0.) {out.active=false;return out;}
    const double error = std::atan2(delta.y(),delta.x())-yaw;
    out.h=-.5*(1.-std::cos(error));
    out.coefficient=.5*std::sin(error);
    out.bearing_drift=(delta.y()*velocity.x()-delta.x()*velocity.y())/delta.squaredNorm();
    out.rhs=out.coefficient*out.bearing_drift-out.h;
    if(!std::isfinite(out.rhs)||!std::isfinite(out.bearing_drift)) {
        out.valid=false;out.active=false;return out;
    }
    const auto objective=[&](double rate) {
        return rate*rate+10.*std::max(0.,out.rhs-out.coefficient*rate);
    };
    const auto consider=[&](double rate) {
        rate=std::clamp(rate,-maximum_rate,maximum_rate);
        if(objective(rate)<objective(out.rate))out.rate=rate;
    };
    // Strictly convex scalar objective after eliminating nonnegative linear slack.
    // Its minimum is at a smooth stationary point or the sole hinge/boundary.
    consider(5.*out.coefficient);
    if(out.coefficient!=0.)consider(out.rhs/out.coefficient);
    consider(-maximum_rate);consider(maximum_rate);
    out.slack=std::max(0.,out.rhs-out.coefficient*out.rate);
    return out;
}
}
