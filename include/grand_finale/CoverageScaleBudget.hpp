#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace gf {

struct CoverageGridSpec {
    double width_m = 0.0;
    double height_m = 0.0;
    double spacing_m = 0.0;
};

struct SectorFootprintSpec {
    double inner_radius_m = 0.0;
    double outer_radius_m = 0.0;
    double half_angle_rad = 0.0;
};

struct SectorFootprintStage {
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double heading_rad = 0.0;
};

struct CoverageScaleBudget {
    double map_area_m2 = 0.0;
    std::size_t valid_cell_count = 0;
    std::size_t initial_covered_cells = 0;
    std::size_t reachable_union_cells = 0;
    double initial_fraction = 0.0;
    double reachable_union_fraction = 0.0;
    bool initial_below_target = false;
};

enum class MotionProfile { Triangular, Trapezoidal };

struct RestToRestTime {
    double seconds = 0.0;
    MotionProfile profile = MotionProfile::Triangular;
    std::optional<double> speed_limit_mps;
};

inline bool sectorContains(
    const SectorFootprintStage& stage,
    const SectorFootprintSpec& footprint,
    const Eigen::Vector2d& point) {
    const Eigen::Vector2d relative = point - stage.center;
    const double distance = relative.norm();
    if (distance < footprint.inner_radius_m - 1e-12 ||
        distance > footprint.outer_radius_m + 1e-12) return false;
    if (distance <= 1e-12) return true;
    const double bearing = std::atan2(relative.y(),relative.x());
    const double error = std::atan2(
        std::sin(bearing-stage.heading_rad),
        std::cos(bearing-stage.heading_rad));
    return std::abs(error) <= footprint.half_angle_rad + 1e-12;
}

inline CoverageScaleBudget evaluateCoverageScaleBudget(
    const CoverageGridSpec& grid,
    const SectorFootprintSpec& footprint,
    const std::vector<SectorFootprintStage>& initialized,
    const std::vector<SectorFootprintStage>& reachable,
    double completion_fraction) {
    const bool finite = std::isfinite(grid.width_m) &&
        std::isfinite(grid.height_m) && std::isfinite(grid.spacing_m) &&
        std::isfinite(footprint.inner_radius_m) &&
        std::isfinite(footprint.outer_radius_m) &&
        std::isfinite(footprint.half_angle_rad) &&
        std::isfinite(completion_fraction);
    if (!finite || grid.width_m <= 0.0 || grid.height_m <= 0.0 ||
        grid.spacing_m <= 0.0 || footprint.inner_radius_m < 0.0 ||
        footprint.outer_radius_m <= footprint.inner_radius_m ||
        footprint.half_angle_rad <= 0.0 || footprint.half_angle_rad > M_PI ||
        completion_fraction <= 0.0 || completion_fraction > 1.0) {
        throw std::invalid_argument("invalid coverage scale budget request");
    }
    const double x_count = grid.width_m/grid.spacing_m;
    const double y_count = grid.height_m/grid.spacing_m;
    if (std::abs(x_count-std::round(x_count)) > 1e-9 ||
        std::abs(y_count-std::round(y_count)) > 1e-9) {
        throw std::invalid_argument("grid dimensions must be spacing multiples");
    }
    const std::size_t nx = static_cast<std::size_t>(std::llround(x_count));
    const std::size_t ny = static_cast<std::size_t>(std::llround(y_count));
    std::vector<bool> initial_union(nx*ny,false);
    std::vector<bool> reachable_union(nx*ny,false);
    auto add_stages = [&](const std::vector<SectorFootprintStage>& stages,
                          std::vector<bool>& cells) {
        for (const auto& stage : stages) {
            if (!stage.center.allFinite() || !std::isfinite(stage.heading_rad))
                throw std::invalid_argument("non-finite sector stage");
            for (std::size_t ix=0; ix<nx; ++ix) {
                for (std::size_t iy=0; iy<ny; ++iy) {
                    const Eigen::Vector2d point(
                        (static_cast<double>(ix)+0.5)*grid.spacing_m,
                        (static_cast<double>(iy)+0.5)*grid.spacing_m);
                    if (sectorContains(stage,footprint,point))
                        cells[ix*ny+iy] = true;
                }
            }
        }
    };
    add_stages(initialized,initial_union);
    reachable_union = initial_union;
    add_stages(reachable,reachable_union);
    CoverageScaleBudget result;
    result.map_area_m2 = grid.width_m*grid.height_m;
    result.valid_cell_count = nx*ny;
    for (bool value : initial_union)
        result.initial_covered_cells += value ? 1U : 0U;
    for (bool value : reachable_union)
        result.reachable_union_cells += value ? 1U : 0U;
    result.initial_fraction = static_cast<double>(result.initial_covered_cells)/
        static_cast<double>(result.valid_cell_count);
    result.reachable_union_fraction =
        static_cast<double>(result.reachable_union_cells)/
        static_cast<double>(result.valid_cell_count);
    result.initial_below_target = result.initial_fraction < completion_fraction;
    return result;
}

inline RestToRestTime minimumRestToRestTime(
    double distance_m,
    double acceleration_mps2,
    std::optional<double> speed_limit_mps) {
    if (!std::isfinite(distance_m) || distance_m < 0.0 ||
        !std::isfinite(acceleration_mps2) || acceleration_mps2 <= 0.0 ||
        (speed_limit_mps.has_value() &&
         (!std::isfinite(*speed_limit_mps) || *speed_limit_mps <= 0.0))) {
        throw std::invalid_argument("invalid rest-to-rest motion budget");
    }
    if (distance_m == 0.0)
        return {0.0,MotionProfile::Triangular,speed_limit_mps};
    const double triangular_time =
        2.0*std::sqrt(distance_m/acceleration_mps2);
    if (!speed_limit_mps.has_value())
        return {triangular_time,MotionProfile::Triangular,std::nullopt};
    const double critical_distance =
        (*speed_limit_mps)*(*speed_limit_mps)/acceleration_mps2;
    if (distance_m <= critical_distance + 1e-12)
        return {triangular_time,MotionProfile::Triangular,speed_limit_mps};
    const double seconds = 2.0*(*speed_limit_mps)/acceleration_mps2 +
        (distance_m-critical_distance)/(*speed_limit_mps);
    return {seconds,MotionProfile::Trapezoidal,speed_limit_mps};
}

}  // namespace gf
