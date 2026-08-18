#pragma once

#include "grand_finale/Types.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct PlantSpeedAppliedControlRequest {
    NodeId owner=0;
    Eigen::Vector2d estimated_velocity_mps=Eigen::Vector2d::Zero();
    double velocity_support_mps=0.0;
    double speed_limit_mps=0.0;
    double dt_s=0.0;
    std::size_t facet_count=64;
    double low_speed_orientation_epsilon_mps=1.0e-9;
};

struct PlantSpeedAppliedControlRow {
    std::string id;
    NodeId owner=0;
    Eigen::Vector2d facet_normal=Eigen::Vector2d::Zero();
    Eigen::Vector2d control_coefficient=Eigen::Vector2d::Zero();
    double constant=0.0;
    double direction_velocity_support_mps=0.0;
    double polygon_apothem_mps=0.0;

    double margin(const Eigen::Vector2d& control) const {
        return control_coefficient.dot(control)+constant;
    }
};

struct PlantSpeedExactZohAudit {
    bool valid=false;
    double current_speed_mps=std::numeric_limits<double>::infinity();
    double endpoint_speed_mps=std::numeric_limits<double>::infinity();
    double maximum_interval_speed_mps=std::numeric_limits<double>::infinity();
    double speed_margin_mps=-std::numeric_limits<double>::infinity();
};

inline std::vector<PlantSpeedAppliedControlRow>
buildPlantSpeedAppliedControlRows(
    const PlantSpeedAppliedControlRequest& request) {
    if (!request.estimated_velocity_mps.allFinite() ||
        !std::isfinite(request.velocity_support_mps) ||
        request.velocity_support_mps<0.0 ||
        !std::isfinite(request.speed_limit_mps) ||
        request.speed_limit_mps<=0.0 ||
        !std::isfinite(request.dt_s) || request.dt_s<=0.0 ||
        request.facet_count<4 || request.facet_count%2!=0 ||
        !std::isfinite(request.low_speed_orientation_epsilon_mps) ||
        request.low_speed_orientation_epsilon_mps<0.0) {
        throw std::invalid_argument("invalid plant-speed applied-control request");
    }

    const double pi=std::acos(-1.0);
    const double orientation=
        request.estimated_velocity_mps.norm()>
            request.low_speed_orientation_epsilon_mps
        ?std::atan2(request.estimated_velocity_mps.y(),
                    request.estimated_velocity_mps.x())
        :0.0;
    const double half_vertex_angle=pi/static_cast<double>(request.facet_count);
    const double apothem=request.speed_limit_mps*std::cos(half_vertex_angle);
    std::vector<PlantSpeedAppliedControlRow> rows;
    rows.reserve(request.facet_count);
    for (std::size_t index=0;index<request.facet_count;++index) {
        const double normal_angle=orientation+
            (2.0*static_cast<double>(index)+1.0)*half_vertex_angle;
        const Eigen::Vector2d normal(
            std::cos(normal_angle),std::sin(normal_angle));
        rows.push_back({
            "plant_speed_applied_control:"+std::to_string(request.owner)+
                ":facet:"+std::to_string(index),
            request.owner,normal,-normal,
            (apothem-normal.dot(request.estimated_velocity_mps)-
                request.velocity_support_mps)/request.dt_s,
            request.velocity_support_mps,apothem});
    }
    return rows;
}

inline PlantSpeedExactZohAudit auditPlantSpeedExactZoh(
    const Eigen::Vector2d& velocity_mps,
    const Eigen::Vector2d& applied_acceleration_mps2,
    double dt_s,double speed_limit_mps,double tolerance_mps) {
    PlantSpeedExactZohAudit result;
    if (!velocity_mps.allFinite() ||
        !applied_acceleration_mps2.allFinite() ||
        !std::isfinite(dt_s) || dt_s<=0.0 ||
        !std::isfinite(speed_limit_mps) || speed_limit_mps<=0.0 ||
        !std::isfinite(tolerance_mps) || tolerance_mps<0.0) {
        return result;
    }
    result.current_speed_mps=velocity_mps.norm();
    result.endpoint_speed_mps=
        (velocity_mps+dt_s*applied_acceleration_mps2).norm();
    // The norm of an affine function is convex, hence its maximum over the
    // closed ZOH interval is attained at one of the two endpoints.
    result.maximum_interval_speed_mps=std::max(
        result.current_speed_mps,result.endpoint_speed_mps);
    result.speed_margin_mps=
        speed_limit_mps-result.maximum_interval_speed_mps;
    result.valid=result.maximum_interval_speed_mps<=
        speed_limit_mps+tolerance_mps;
    return result;
}

} // namespace gf
