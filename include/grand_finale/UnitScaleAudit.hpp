#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace gf {

enum class ScaleClassification {
    InheritedToyGeometry,
    ValidButReferenceInactive,
    ReferenceActive
};

struct UnitScaleInputs {
    double map_width_m = 0.0;
    double map_height_m = 0.0;
    double grid_spacing_m = 0.0;
    double sensor_radius_m = 0.0;
    double reference_keep_m = 0.0;
    double reference_add_m = 0.0;
    double collision_distance_m = 0.0;
    double acceleration_half_box_mps2 = 0.0;
    double dt_s = 0.0;
    double observed_speed_mps = 0.0;
    bool inherited_from_d0 = false;
    double position_scale_to_metres = 1.0;
};

struct UnitScaleReport {
    ScaleClassification classification =
        ScaleClassification::ValidButReferenceInactive;
    double map_width_over_reference = 0.0;
    double map_height_over_reference = 0.0;
    double sensor_radius_over_grid = 0.0;
    double acceleration_step_over_grid = 0.0;
    double speed_step_over_grid = 0.0;
    std::string position_units = "m";
    std::string velocity_units = "m/s";
    std::string acceleration_units = "m/s^2";
    std::string canonical_row_units = "m/s^2";
};

inline UnitScaleReport auditUnitScale(const UnitScaleInputs& value) {
    const double numbers[] = {
        value.map_width_m, value.map_height_m, value.grid_spacing_m,
        value.sensor_radius_m, value.reference_keep_m,
        value.reference_add_m, value.collision_distance_m,
        value.acceleration_half_box_mps2, value.dt_s,
        value.observed_speed_mps, value.position_scale_to_metres};
    for (double number : numbers) {
        if (!std::isfinite(number))
            throw std::invalid_argument("unit scale values must be finite");
    }
    if (std::abs(value.position_scale_to_metres - 1.0) > 1.0e-12)
        throw std::invalid_argument(
            "unit conversion changes approved distance semantics");
    if (value.map_width_m <= 0.0 || value.map_height_m <= 0.0 ||
        value.grid_spacing_m <= 0.0 || value.sensor_radius_m <= 0.0 ||
        value.reference_keep_m <= 0.0 || value.reference_add_m <= 0.0 ||
        value.reference_add_m >= value.reference_keep_m ||
        value.collision_distance_m <= 0.0 ||
        value.acceleration_half_box_mps2 <= 0.0 || value.dt_s <= 0.0 ||
        value.observed_speed_mps < 0.0) {
        throw std::invalid_argument("scientifically incompatible scale values");
    }
    UnitScaleReport report;
    report.map_width_over_reference =
        value.map_width_m / value.reference_keep_m;
    report.map_height_over_reference =
        value.map_height_m / value.reference_keep_m;
    report.sensor_radius_over_grid =
        value.sensor_radius_m / value.grid_spacing_m;
    report.acceleration_step_over_grid =
        value.acceleration_half_box_mps2 * value.dt_s * value.dt_s /
        value.grid_spacing_m;
    report.speed_step_over_grid =
        value.observed_speed_mps * value.dt_s / value.grid_spacing_m;
    const bool inactive =
        std::hypot(value.map_width_m, value.map_height_m) <
        value.reference_add_m;
    report.classification = inactive
        ? (value.inherited_from_d0
            ? ScaleClassification::InheritedToyGeometry
            : ScaleClassification::ValidButReferenceInactive)
        : ScaleClassification::ReferenceActive;
    return report;
}

}  // namespace gf
