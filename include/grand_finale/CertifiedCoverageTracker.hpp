#pragma once

#include "world/world"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace gf {

enum class CoverageFootprintKind { Circular, ForwardSector };

struct CertifiedCoverageRestartState {
    std::vector<bool> truth;
    std::vector<bool> certified;
};

class CertifiedCoverageTracker {
public:
    // The task-domain mask is fixed before observations. History is not copied:
    // certified and truth history still comes from the frozen sensing rules.
    explicit CertifiedCoverageTracker(const GridWorld& domain)
        : CertifiedCoverageTracker(domain.xLim,domain.xNum,
                                   domain.yLim,domain.yNum) {
        if (domain.valid.size()!=truth_.valid.size() || domain.validCount<=0 ||
            domain.validCount!=std::count(domain.valid.begin(),domain.valid.end(),true))
            throw std::invalid_argument("invalid coverage task-domain mask");
        truth_.valid=domain.valid;certified_.valid=domain.valid;
        truth_.validCount=domain.validCount;certified_.validCount=domain.validCount;
        recomputeCoveredCounts();
    }

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
        truth_covered_count_ = 0;
        certified_covered_count_ = 0;
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
        const double r_bound = sensor_radius_m + 1.0e-6;
        const double x_min_box = std::min(truth_position.x, estimated_position.x) - r_bound;
        const double x_max_box = std::max(truth_position.x, estimated_position.x) + r_bound;
        const double y_min_box = std::min(truth_position.y, estimated_position.y) - r_bound;
        const double y_max_box = std::max(truth_position.y, estimated_position.y) + r_bound;
        const auto [ix_min, ix_max, iy_min, iy_max] =
            clampedIndexBox(x_min_box, x_max_box, y_min_box, y_max_box);
        const double r_bound_2 = (sensor_radius_m + 1.0e-12) * (sensor_radius_m + 1.0e-12);
        for (int x = ix_min; x <= ix_max; ++x) {
            const double cx = truth_.getCellCenterX(x);
            for (int y = iy_min; y <= iy_max; ++y) {
                const int idx = truth_.getIndex(x, y);
                if (truth_.vis[idx] && certified_.vis[idx]) continue;
                const double cy = truth_.getCellCenterY(y);
                const double tdx = cx - truth_position.x;
                const double tdy = cy - truth_position.y;
                if (!truth_.vis[idx] && tdx * tdx + tdy * tdy <= r_bound_2) {
                    const double truth_distance = std::hypot(tdx, tdy);
                    if (truth_distance <= sensor_radius_m) {
                        truth_.vis[idx] = true;
                        if (truth_.valid[idx]) ++truth_covered_count_;
                    }
                }
                const double edx = cx - estimated_position.x;
                const double edy = cy - estimated_position.y;
                if (!certified_.vis[idx] && edx * edx + edy * edy <= r_bound_2) {
                    const double estimated_distance = std::hypot(edx, edy);
                    if (estimated_distance + error_bound_m + cell_half_diagonal_
                        <= sensor_radius_m) {
                        certified_.vis[idx] = true;
                        if (certified_.valid[idx]) ++certified_covered_count_;
                    }
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
        const double r_bound = outer_radius_m + 1.0e-6;
        const double x_min_box = std::min(truth_position.x, estimated_position.x) - r_bound;
        const double x_max_box = std::max(truth_position.x, estimated_position.x) + r_bound;
        const double y_min_box = std::min(truth_position.y, estimated_position.y) - r_bound;
        const double y_max_box = std::max(truth_position.y, estimated_position.y) + r_bound;
        const auto [ix_min, ix_max, iy_min, iy_max] =
            clampedIndexBox(x_min_box, x_max_box, y_min_box, y_max_box);
        const double max_truth_r = outer_radius_m + 1e-12;
        const double max_truth_r2 = max_truth_r * max_truth_r;
        const double max_est = outer_radius_m + 1e-12 - reserve;
        const double max_est_2 = max_est >= 0.0 ? max_est * max_est : -1.0;

        for (int x=ix_min; x<=ix_max; ++x) {
            const double cx = truth_.getCellCenterX(x);
            if (cx < x_min_box || cx > x_max_box) continue;
            for (int y=iy_min; y<=iy_max; ++y) {
                const int idx = truth_.getIndex(x, y);
                if (!truth_.valid[idx] || (truth_.vis[idx] && certified_.vis[idx])) continue;
                const double cy = truth_.getCellCenterY(y);
                if (cy < y_min_box || cy > y_max_box) continue;

                const double truth_dx = cx-truth_position.x;
                const double truth_dy = cy-truth_position.y;
                const double truth_d2 = truth_dx*truth_dx + truth_dy*truth_dy;
                const bool in_truth_disk = !truth_.vis[idx] && (truth_d2 <= max_truth_r2);

                const double estimated_dx = cx-estimated_position.x;
                const double estimated_dy = cy-estimated_position.y;
                const bool in_est_disk = !certified_.vis[idx] && (max_est >= 0.0) &&
                    (estimated_dx*estimated_dx + estimated_dy*estimated_dy <= max_est_2);

                if (!in_truth_disk && !in_est_disk) continue;

                if (in_truth_disk) {
                    const double truth_distance = std::sqrt(truth_d2);
                    if (truth_distance >= inner_radius_m-1e-12 &&
                        truth_distance <= outer_radius_m+1e-12) {
                        const bool truth_angle = truth_distance <= 1e-12 ||
                            angular_error(truth_dy,truth_dx) <= half_angle_rad+1e-12;
                        if (truth_angle) {
                            truth_.vis[idx] = true;
                            ++truth_covered_count_;
                        }
                    }
                }

                if (in_est_disk) {
                    const double estimated_distance = std::hypot(estimated_dx,estimated_dy);
                    const bool radial_containment =
                        estimated_distance+reserve <= outer_radius_m+1e-12 &&
                        (inner_radius_m <= 1e-12 ||
                         estimated_distance-reserve >= inner_radius_m-1e-12);
                    if (radial_containment) {
                        bool angular_containment = half_angle_rad >= M_PI-1e-12;
                        if (!angular_containment && estimated_distance > reserve) {
                            const double angular_reserve = std::asin(std::min(
                                1.0,reserve/estimated_distance));
                            angular_containment =
                                angular_error(estimated_dy,estimated_dx)+angular_reserve
                                <= half_angle_rad+1e-12;
                        }
                        if (angular_containment) {
                            certified_.vis[idx] = true;
                            ++certified_covered_count_;
                        }
                    }
                }
            }
        }
    }

    int truthCoveredCount() const { return truth_covered_count_; }
    int certifiedCoveredCount() const { return certified_covered_count_; }
    double truthFraction() const {
        return truth_.validCount == 0
            ? 0.0
            : static_cast<double>(truth_covered_count_) / truth_.validCount;
    }
    double certifiedFraction() const {
        return certified_.validCount == 0
            ? 0.0
            : static_cast<double>(certified_covered_count_) /
                certified_.validCount;
    }
    bool reachedCertifiedT100() const {
        return certified_.validCount > 0 &&
            certified_covered_count_ == certified_.validCount;
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
    const GridWorld& truthGrid() const { return truth_; }
    const GridWorld& certifiedGrid() const { return certified_; }
    CertifiedCoverageRestartState restartState() const {
        return {truth_.vis,certified_.vis};
    }
    void restoreRestartState(const CertifiedCoverageRestartState& state) {
        if (state.truth.size()!=truth_.vis.size() ||
            state.certified.size()!=certified_.vis.size())
            throw std::invalid_argument("coverage restart dimension mismatch");
        truth_.vis=state.truth;
        certified_.vis=state.certified;
        recomputeCoveredCounts();
    }

private:
    struct IndexBox {
        int ix_min = 0;
        int ix_max = -1;
        int iy_min = 0;
        int iy_max = -1;
    };

    IndexBox clampedIndexBox(
        double x_min_box,
        double x_max_box,
        double y_min_box,
        double y_max_box) const {
        const double cell_dx =
            (truth_.xLim.second - truth_.xLim.first) / static_cast<double>(truth_.xNum);
        const double cell_dy =
            (truth_.yLim.second - truth_.yLim.first) / static_cast<double>(truth_.yNum);
        const int ix_min = std::max(
            0, static_cast<int>(std::floor((x_min_box - truth_.xLim.first) / cell_dx - 0.5)) - 1);
        const int ix_max = std::min(
            truth_.xNum - 1,
            static_cast<int>(std::ceil((x_max_box - truth_.xLim.first) / cell_dx - 0.5)) + 1);
        const int iy_min = std::max(
            0, static_cast<int>(std::floor((y_min_box - truth_.yLim.first) / cell_dy - 0.5)) - 1);
        const int iy_max = std::min(
            truth_.yNum - 1,
            static_cast<int>(std::ceil((y_max_box - truth_.yLim.first) / cell_dy - 0.5)) + 1);
        return {ix_min, ix_max, iy_min, iy_max};
    }

    void recomputeCoveredCounts() {
        truth_covered_count_ = coveredCount(truth_);
        certified_covered_count_ = coveredCount(certified_);
    }

    static int coveredCount(const GridWorld& grid) {
        int count = 0;
        for (std::size_t index = 0; index < grid.vis.size(); ++index)
            if (grid.valid[index] && grid.vis[index]) ++count;
        return count;
    }

    GridWorld truth_;
    GridWorld certified_;
    double cell_half_diagonal_ = 0.0;
    int truth_covered_count_ = 0;
    int certified_covered_count_ = 0;
};

}  // namespace gf
