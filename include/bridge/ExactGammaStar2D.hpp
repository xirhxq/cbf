#ifndef CBF_EXACT_GAMMA_STAR_2D_HPP
#define CBF_EXACT_GAMMA_STAR_2D_HPP

#include <array>
#include <cmath>
#include <limits>
#include <vector>

struct BridgeGammaStarResidual2D {
    double ax = 0.0;
    double ay = 0.0;
    double constant = 0.0;
};

namespace bridge_gamma_star_detail {

struct EpigraphHalfspace3D {
    double ax = 0.0;
    double ay = 0.0;
    double t = 0.0;
    double rhs = 0.0;
};

inline double determinant3(
        double a11, double a12, double a13,
        double a21, double a22, double a23,
        double a31, double a32, double a33) {
    return a11 * (a22 * a33 - a23 * a32)
           - a12 * (a21 * a33 - a23 * a31)
           + a13 * (a21 * a32 - a22 * a31);
}

inline bool finite(const EpigraphHalfspace3D &halfspace) {
    return std::isfinite(halfspace.ax)
           && std::isfinite(halfspace.ay)
           && std::isfinite(halfspace.t)
           && std::isfinite(halfspace.rhs);
}

inline bool solveIntersection(
        const std::array<EpigraphHalfspace3D, 3> &planes,
        std::array<double, 3> *solution) {
    const auto &a = planes[0];
    const auto &b = planes[1];
    const auto &c = planes[2];
    double determinant = determinant3(
            a.ax, a.ay, a.t,
            b.ax, b.ay, b.t,
            c.ax, c.ay, c.t);
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1.0e-12) {
        return false;
    }

    (*solution)[0] = determinant3(
            a.rhs, a.ay, a.t,
            b.rhs, b.ay, b.t,
            c.rhs, c.ay, c.t) / determinant;
    (*solution)[1] = determinant3(
            a.ax, a.rhs, a.t,
            b.ax, b.rhs, b.t,
            c.ax, c.rhs, c.t) / determinant;
    (*solution)[2] = determinant3(
            a.ax, a.ay, a.rhs,
            b.ax, b.ay, b.rhs,
            c.ax, c.ay, c.rhs) / determinant;
    return std::isfinite((*solution)[0])
           && std::isfinite((*solution)[1])
           && std::isfinite((*solution)[2]);
}

}  // namespace bridge_gamma_star_detail

inline double exactBridgeGammaStar2D(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double accelerationHalfBox,
        double tolerance = 1.0e-9) {
    if (residuals.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (!std::isfinite(accelerationHalfBox) || accelerationHalfBox < 0.0
        || !std::isfinite(tolerance) || tolerance < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    using bridge_gamma_star_detail::EpigraphHalfspace3D;
    std::vector<EpigraphHalfspace3D> halfspaces;
    halfspaces.reserve(residuals.size() + 4);
    for (const auto &residual : residuals) {
        EpigraphHalfspace3D plane{residual.ax, residual.ay, 1.0, residual.constant};
        if (!bridge_gamma_star_detail::finite(plane)) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        halfspaces.push_back(plane);
    }
    halfspaces.push_back({1.0, 0.0, 0.0, accelerationHalfBox});
    halfspaces.push_back({-1.0, 0.0, 0.0, accelerationHalfBox});
    halfspaces.push_back({0.0, 1.0, 0.0, accelerationHalfBox});
    halfspaces.push_back({0.0, -1.0, 0.0, accelerationHalfBox});

    double best = -std::numeric_limits<double>::infinity();
    for (size_t first = 0; first + 2 < halfspaces.size(); ++first) {
        for (size_t second = first + 1; second + 1 < halfspaces.size(); ++second) {
            for (size_t third = second + 1; third < halfspaces.size(); ++third) {
                std::array<double, 3> candidate{};
                if (!bridge_gamma_star_detail::solveIntersection(
                            {halfspaces[first], halfspaces[second], halfspaces[third]},
                            &candidate)) {
                    continue;
                }

                bool feasible = true;
                for (const auto &halfspace : halfspaces) {
                    double value = halfspace.ax * candidate[0]
                                   + halfspace.ay * candidate[1]
                                   + halfspace.t * candidate[2];
                    if (value > halfspace.rhs + tolerance) {
                        feasible = false;
                        break;
                    }
                }
                if (feasible && candidate[2] > best) {
                    best = candidate[2];
                }
            }
        }
    }
    return best;
}

#endif
