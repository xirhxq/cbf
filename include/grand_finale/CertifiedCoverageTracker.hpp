#pragma once

#include "world/world"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace gf {

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
