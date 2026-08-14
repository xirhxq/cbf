#pragma once

#include "world/world"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace gf {

enum class CoverageFootprintKind { Circular, ForwardSector };

class CertifiedCoverageTracker {
public:
    CertifiedCoverageTracker(
        std::pair<double, double> x_limits,
        int x_cells,
        std::pair<double, double> y_limits,
        int y_cells)
        : truth_(x_limits, x_cells, y_limits, y_cells),
          certified_(x_limits, x_cells, y_limits, y_cells) {
        if (!(x_limits.second > x_limits.first) ||
            !(y_limits.second > y_limits.first) ||
            x_cells <= 0 || y_cells <= 0) {
            throw std::invalid_argument("coverage grid must be finite and nonempty");
        }
        const double cell_width =
            (x_limits.second - x_limits.first) / x_cells;
        const double cell_height =
            (y_limits.second - y_limits.first) / y_cells;
        cell_half_diagonal_ = 0.5 * std::hypot(cell_width, cell_height);
    }

    void observe(
        const Point& truth_position,
        const Point& estimated_position,
        double error_bound_m,
        double sensor_radius_m) {
        if (!std::isfinite(truth_position.x) ||
            !std::isfinite(truth_position.y) ||
            !std::isfinite(estimated_position.x) ||
            !std::isfinite(estimated_position.y) ||
            !std::isfinite(error_bound_m) || error_bound_m < 0.0 ||
            !std::isfinite(sensor_radius_m) || sensor_radius_m <= 0.0) {
            throw std::invalid_argument("invalid coverage observation");
        }
        for (int x = 0; x < truth_.xNum; ++x) {
            for (int y = 0; y < truth_.yNum; ++y) {
                const Point cell(
                    truth_.getCellCenterX(x), truth_.getCellCenterY(y));
                const double truth_distance = std::hypot(
                    cell.x - truth_position.x, cell.y - truth_position.y);
                if (truth_distance <= sensor_radius_m)
                    truth_.setValue(x, y, true);
                const double estimated_distance = std::hypot(
                    cell.x - estimated_position.x,
                    cell.y - estimated_position.y);
                if (estimated_distance + error_bound_m + cell_half_diagonal_
                    <= sensor_radius_m) {
                    certified_.setValue(x, y, true);
                }
            }
        }
    }

    void observeSector(
        const Point& truth_position,
        const Point& estimated_position,
        double error_bound_m,
        double inner_radius_m,
        double outer_radius_m,
        double half_angle_rad,
        double heading_rad) {
        if (!std::isfinite(truth_position.x) ||
            !std::isfinite(truth_position.y) ||
            !std::isfinite(estimated_position.x) ||
            !std::isfinite(estimated_position.y) ||
            !std::isfinite(error_bound_m) || error_bound_m < 0.0 ||
            !std::isfinite(inner_radius_m) || inner_radius_m < 0.0 ||
            !std::isfinite(outer_radius_m) ||
            outer_radius_m <= inner_radius_m ||
            !std::isfinite(half_angle_rad) || half_angle_rad <= 0.0 ||
            half_angle_rad > M_PI || !std::isfinite(heading_rad)) {
            throw std::invalid_argument("invalid sector coverage observation");
        }
        const double reserve = error_bound_m + cell_half_diagonal_;
        const auto angular_error = [heading_rad](double dy, double dx) {
            const double bearing = std::atan2(dy,dx);
            return std::abs(std::atan2(
                std::sin(bearing-heading_rad),
                std::cos(bearing-heading_rad)));
        };
        for (int x=0; x<truth_.xNum; ++x) {
            for (int y=0; y<truth_.yNum; ++y) {
                const Point cell(
                    truth_.getCellCenterX(x),truth_.getCellCenterY(y));
                const double truth_dx = cell.x-truth_position.x;
                const double truth_dy = cell.y-truth_position.y;
                const double truth_distance = std::hypot(truth_dx,truth_dy);
                const bool truth_angle = truth_distance <= 1e-12 ||
                    angular_error(truth_dy,truth_dx) <= half_angle_rad+1e-12;
                if (truth_distance >= inner_radius_m-1e-12 &&
                    truth_distance <= outer_radius_m+1e-12 && truth_angle) {
                    truth_.setValue(x,y,true);
                }

                const double estimated_dx = cell.x-estimated_position.x;
                const double estimated_dy = cell.y-estimated_position.y;
                const double estimated_distance =
                    std::hypot(estimated_dx,estimated_dy);
                bool angular_containment = half_angle_rad >= M_PI-1e-12;
                if (!angular_containment && estimated_distance > reserve) {
                    const double angular_reserve = std::asin(std::min(
                        1.0,reserve/estimated_distance));
                    angular_containment =
                        angular_error(estimated_dy,estimated_dx)+angular_reserve
                        <= half_angle_rad+1e-12;
                }
                const bool radial_containment =
                    estimated_distance+reserve <= outer_radius_m+1e-12 &&
                    (inner_radius_m <= 1e-12 ||
                     estimated_distance-reserve >= inner_radius_m-1e-12);
                if (radial_containment && angular_containment)
                    certified_.setValue(x,y,true);
            }
        }
    }

    int truthCoveredCount() const { return coveredCount(truth_); }
    int certifiedCoveredCount() const { return coveredCount(certified_); }
    double truthFraction() const {
        return truth_.validCount == 0
            ? 0.0
            : static_cast<double>(truthCoveredCount()) / truth_.validCount;
    }
    double certifiedFraction() const {
        return certified_.validCount == 0
            ? 0.0
            : static_cast<double>(certifiedCoveredCount()) /
                certified_.validCount;
    }
    bool reachedCertifiedT100() const {
        return certified_.validCount > 0 &&
            certifiedCoveredCount() == certified_.validCount;
    }
    int falseCertifiedCount() const {
        int count = 0;
        for (std::size_t index = 0; index < certified_.vis.size(); ++index) {
            if (certified_.valid[index] && certified_.vis[index] &&
                !truth_.vis[index]) {
                ++count;
            }
        }
        return count;
    }
    bool certifiedSubsetOfTruth() const {
        return falseCertifiedCount() == 0;
    }
    bool truthCovered(int x, int y) { return truth_.getValue(x, y); }
    bool certifiedCovered(int x, int y) {
        return certified_.getValue(x, y);
    }

private:
    static int coveredCount(const GridWorld& grid) {
        int count = 0;
        for (std::size_t index = 0; index < grid.vis.size(); ++index)
            if (grid.valid[index] && grid.vis[index]) ++count;
        return count;
    }

    GridWorld truth_;
    GridWorld certified_;
    double cell_half_diagonal_ = 0.0;
};

}  // namespace gf
