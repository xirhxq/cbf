#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

namespace gf {

struct LinearHocbfGains {
    double lambda1_per_s=1.0;
    double lambda2_per_s=1.0;

    bool operator==(const LinearHocbfGains& other) const {
        return lambda1_per_s==other.lambda1_per_s &&
               lambda2_per_s==other.lambda2_per_s;
    }
};

inline std::vector<LinearHocbfGains> task10p11nGainCandidates() {
    constexpr std::array<double,4> lambda1{0.125,0.25,0.5,1.0};
    constexpr std::array<double,4> lambda2{0.5,1.0,1.5,2.0};
    std::vector<LinearHocbfGains> result;
    result.reserve(lambda1.size()*lambda2.size());
    for (double first:lambda1)
        for (double second:lambda2)
            result.push_back({first,second});
    return result;
}

struct IsolatedPairGainAudit {
    bool valid=false;
    double initial_central_margin_mps2=0.0;
    double minimum_central_margin_mps2=0.0;
    double minimum_time_s=0.0;
    double minimum_endpoint_margin_mps2=0.0;
    double minimum_endpoint_time_s=0.0;
    double stop_time_s=0.0;
    bool complete_braking_nonnegative=false;
};

struct ExactZohPairIntervalAudit {
    bool valid=false;
    double minimum_time_s=0.0;
    double minimum_nominal_distance_m=0.0;
    double minimum_robust_clearance_m=0.0;
};

namespace task10p11n_detail {

inline std::vector<double> realPolynomialRoots(
    double cubic,double quadratic,double linear,double constant) {
    constexpr double epsilon=1.0e-12;
    constexpr double pi=3.141592653589793238462643383279502884;
    std::vector<double> roots;
    if (std::abs(cubic)<=epsilon) {
        if (std::abs(quadratic)<=epsilon) {
            if (std::abs(linear)>epsilon) roots.push_back(-constant/linear);
            return roots;
        }
        const double discriminant=linear*linear-4.0*quadratic*constant;
        if (discriminant<-epsilon) return roots;
        const double square_root=std::sqrt(std::max(0.0,discriminant));
        roots.push_back((-linear-square_root)/(2.0*quadratic));
        if (square_root>epsilon)
            roots.push_back((-linear+square_root)/(2.0*quadratic));
        return roots;
    }
    const double a=quadratic/cubic;
    const double b=linear/cubic;
    const double c=constant/cubic;
    const double p=b-a*a/3.0;
    const double q=2.0*a*a*a/27.0-a*b/3.0+c;
    const double discriminant=q*q/4.0+p*p*p/27.0;
    if (discriminant>epsilon) {
        const double square_root=std::sqrt(discriminant);
        roots.push_back(std::cbrt(-q/2.0+square_root)+
                        std::cbrt(-q/2.0-square_root)-a/3.0);
    } else if (discriminant>=-epsilon) {
        const double value=std::cbrt(-q/2.0);
        roots.push_back(2.0*value-a/3.0);
        roots.push_back(-value-a/3.0);
    } else {
        const double radius=2.0*std::sqrt(-p/3.0);
        const double angle=std::acos(
            std::clamp((3.0*q/(2.0*p))*std::sqrt(-3.0/p),-1.0,1.0))/3.0;
        for (int index=0;index<3;++index)
            roots.push_back(radius*std::cos(angle-2.0*pi*index/3.0)-a/3.0);
    }
    return roots;
}

}  // namespace task10p11n_detail

inline ExactZohPairIntervalAudit auditExactZohPairInterval(
    const Eigen::Vector2d& relative_position,
    const Eigen::Vector2d& relative_velocity,
    const Eigen::Vector2d& relative_acceleration,double duration_s,
    double position_tube_radius_m,double separation_distance_m) {
    if (!relative_position.allFinite() || !relative_velocity.allFinite() ||
        !relative_acceleration.allFinite() || !std::isfinite(duration_s) ||
        duration_s<=0.0 || !std::isfinite(position_tube_radius_m) ||
        position_tube_radius_m<0.0 || !std::isfinite(separation_distance_m) ||
        separation_distance_m<=0.0)
        throw std::invalid_argument("invalid exact-ZOH interval audit");
    const double cubic=relative_acceleration.squaredNorm();
    const double quadratic=3.0*relative_velocity.dot(relative_acceleration);
    const double linear=2.0*(relative_velocity.squaredNorm()+
        relative_position.dot(relative_acceleration));
    const double constant=2.0*relative_position.dot(relative_velocity);
    std::vector<double> times{0.0,duration_s};
    for (double root:task10p11n_detail::realPolynomialRoots(
            cubic,quadratic,linear,constant)) {
        if (std::isfinite(root) && root>0.0 && root<duration_s)
            times.push_back(root);
    }
    double minimum=std::numeric_limits<double>::infinity();
    double minimum_time=0.0;
    for (double time_s:times) {
        const Eigen::Vector2d position=relative_position+
            relative_velocity*time_s+
            0.5*relative_acceleration*time_s*time_s;
        const double distance=position.norm();
        if (distance<minimum) {
            minimum=distance;
            minimum_time=time_s;
        }
    }
    return {true,minimum_time,minimum,
        minimum-position_tube_radius_m-separation_distance_m};
}

inline IsolatedPairGainAudit auditIsolatedPairGain(
    const LinearHocbfGains& gains,double robust_clearance_m,
    double radial_closing_speed_mps,double relative_separation_acceleration_mps2,
    double control_period_s,double tolerance=1.0e-12) {
    if (!std::isfinite(gains.lambda1_per_s) ||
        gains.lambda1_per_s<=0.0 ||
        !std::isfinite(gains.lambda2_per_s) ||
        gains.lambda2_per_s<=0.0 ||
        !std::isfinite(robust_clearance_m) || robust_clearance_m<0.0 ||
        !std::isfinite(radial_closing_speed_mps) ||
        radial_closing_speed_mps<0.0 ||
        !std::isfinite(relative_separation_acceleration_mps2) ||
        relative_separation_acceleration_mps2<=0.0 ||
        !std::isfinite(control_period_s) || control_period_s<=0.0 ||
        !std::isfinite(tolerance) || tolerance<0.0) {
        throw std::invalid_argument("invalid isolated gain-audit contract");
    }
    const double first=gains.lambda1_per_s;
    const double second=gains.lambda2_per_s;
    const double acceleration=relative_separation_acceleration_mps2;
    const double closing=radial_closing_speed_mps;
    const double stop_time=closing/acceleration;
    const double quadratic=0.5*first*second*acceleration;
    const double linear=(first+second)*acceleration-
        first*second*closing;
    const double constant=acceleration-(first+second)*closing+
        first*second*robust_clearance_m;
    const auto margin=[&](double time_s) {
        return quadratic*time_s*time_s+linear*time_s+constant;
    };
    double minimum=margin(0.0);
    double minimum_time=0.0;
    const auto consider=[&](double time_s,double& value,double& time) {
        if (time_s<0.0 || time_s>stop_time) return;
        const double candidate=margin(time_s);
        if (candidate<value) {
            value=candidate;
            time=time_s;
        }
    };
    consider(stop_time,minimum,minimum_time);
    if (quadratic>0.0)
        consider(-linear/(2.0*quadratic),minimum,minimum_time);
    double endpoint_minimum=margin(0.0);
    double endpoint_time=0.0;
    const std::size_t endpoint_count=static_cast<std::size_t>(
        std::ceil(stop_time/control_period_s));
    for (std::size_t index=1;index<=endpoint_count;++index) {
        const double time_s=std::min(
            stop_time,static_cast<double>(index)*control_period_s);
        const double candidate=margin(time_s);
        if (candidate<endpoint_minimum) {
            endpoint_minimum=candidate;
            endpoint_time=time_s;
        }
    }

    IsolatedPairGainAudit result;
    result.valid=true;
    result.initial_central_margin_mps2=constant;
    result.minimum_central_margin_mps2=minimum;
    result.minimum_time_s=minimum_time;
    result.minimum_endpoint_margin_mps2=endpoint_minimum;
    result.minimum_endpoint_time_s=endpoint_time;
    result.stop_time_s=stop_time;
    result.complete_braking_nonnegative=minimum>=-tolerance;
    return result;
}

}  // namespace gf
