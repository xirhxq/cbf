#ifndef CBF_EXACT_GAMMA_STAR_2D_HPP
#define CBF_EXACT_GAMMA_STAR_2D_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

struct BridgeGammaStarResidual2D {
    double ax = 0.0;
    double ay = 0.0;
    double constant = 0.0;
};

struct BridgeGammaStarSolution2D {
    bool valid = false;
    double gamma = std::numeric_limits<double>::quiet_NaN();
    double accelX = 0.0;
    double accelY = 0.0;
};

struct BridgeGammaStarWork2D {
    std::size_t intersection_vertices = 0;
    std::size_t residual_evaluations = 0;
    std::size_t pruned_vertices = 0;
};

inline BridgeGammaStarResidual2D bridgeGammaStarResidualFromAffineMargin(
        double controlAx,
        double controlAy,
        double constant) {
    return {-controlAx, -controlAy, constant};
}

namespace bridge_gamma_star_detail {

struct EpigraphHalfspace3D {
    double ax = 0.0;
    double ay = 0.0;
    double t = 0.0;
    double rhs = 0.0;
};

struct Halfspace2D {
    double x = 0.0;
    double y = 0.0;
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
    const double rowNormProduct =
            std::hypot(a.ax, a.ay, a.t)
            * std::hypot(b.ax, b.ay, b.t)
            * std::hypot(c.ax, c.ay, c.t);
    const double singularityBound =
            64.0 * std::numeric_limits<double>::epsilon() * rowNormProduct;
    if (!std::isfinite(determinant)
        || !std::isfinite(singularityBound)
        || std::abs(determinant) <= singularityBound) {
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

inline double roundoffAllowance(
        double firstTerm,
        double secondTerm,
        double thirdTerm,
        double rhs) {
    const double scale = std::abs(firstTerm)
                         + std::abs(secondTerm)
                         + std::abs(thirdTerm)
                         + std::abs(rhs);
    return 64.0 * std::numeric_limits<double>::epsilon() * scale
           + std::numeric_limits<double>::denorm_min();
}

inline bool feasible3D(
        const std::vector<EpigraphHalfspace3D> &halfspaces,
        double x,
        double y,
        double t) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(t)) {
        return false;
    }
    for (const auto &halfspace : halfspaces) {
        const double firstTerm = halfspace.ax * x;
        const double secondTerm = halfspace.ay * y;
        const double thirdTerm = halfspace.t * t;
        const double value = firstTerm + secondTerm + thirdTerm;
        if (!std::isfinite(value)
            || value - halfspace.rhs > roundoffAllowance(
                    firstTerm, secondTerm, thirdTerm, halfspace.rhs)) {
            return false;
        }
    }
    return true;
}

inline bool feasible2D(
        const std::vector<Halfspace2D> &halfspaces,
        double x,
        double y) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }
    for (const auto &halfspace : halfspaces) {
        const double firstTerm = halfspace.x * x;
        const double secondTerm = halfspace.y * y;
        const double value = firstTerm + secondTerm;
        if (!std::isfinite(value)
            || value - halfspace.rhs > roundoffAllowance(
                    firstTerm, secondTerm, 0.0, halfspace.rhs)) {
            return false;
        }
    }
    return true;
}

inline bool betterMinimumNormWitness(
        double candidateX,
        double candidateY,
        double bestX,
        double bestY) {
    const double candidateNormSquared =
            candidateX * candidateX + candidateY * candidateY;
    const double bestNormSquared = bestX * bestX + bestY * bestY;
    if (candidateNormSquared < bestNormSquared) {
        return true;
    }
    if (candidateNormSquared != bestNormSquared) {
        return false;
    }
    if (candidateX < bestX) {
        return true;
    }
    return candidateX == bestX && candidateY < bestY;
}

}  // namespace bridge_gamma_star_detail

inline BridgeGammaStarSolution2D solveExactBridgeGammaStar2DImpl(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double accelerationHalfBox,
        BridgeGammaStarWork2D *work,
        bool enableEarlyPruning) {
    if (residuals.empty()) {
        return {
                true,
                std::numeric_limits<double>::infinity(),
                0.0,
                0.0,
        };
    }
    if (!std::isfinite(accelerationHalfBox) || accelerationHalfBox < 0.0) {
        return {};
    }

    using bridge_gamma_star_detail::EpigraphHalfspace3D;
    std::vector<EpigraphHalfspace3D> halfspaces;
    halfspaces.reserve(residuals.size() + 4);
    for (const auto &residual : residuals) {
        EpigraphHalfspace3D plane{residual.ax, residual.ay, 1.0, residual.constant};
        if (!bridge_gamma_star_detail::finite(plane)) {
            return {};
        }
        halfspaces.push_back(plane);
    }
    halfspaces.push_back({1.0, 0.0, 0.0, accelerationHalfBox});
    halfspaces.push_back({-1.0, 0.0, 0.0, accelerationHalfBox});
    halfspaces.push_back({0.0, 1.0, 0.0, accelerationHalfBox});
    halfspaces.push_back({0.0, -1.0, 0.0, accelerationHalfBox});

    double bestGamma = -std::numeric_limits<double>::infinity();
    double fallbackX = 0.0;
    double fallbackY = 0.0;
    bool foundEpigraphVertex = false;
    for (size_t first = 0; first + 2 < halfspaces.size(); ++first) {
        for (size_t second = first + 1; second + 1 < halfspaces.size(); ++second) {
            for (size_t third = second + 1; third < halfspaces.size(); ++third) {
                std::array<double, 3> candidate{};
                if (!bridge_gamma_star_detail::solveIntersection(
                            {halfspaces[first], halfspaces[second], halfspaces[third]},
                            &candidate)) {
                    continue;
                }
                if (work != nullptr) ++work->intersection_vertices;

                const auto isDefining = [&](size_t index) {
                    return first == index || second == index || third == index;
                };
                const size_t boxOffset = residuals.size();
                if (isDefining(boxOffset)) {
                    candidate[0] = accelerationHalfBox;
                } else if (isDefining(boxOffset + 1)) {
                    candidate[0] = -accelerationHalfBox;
                } else {
                    candidate[0] = std::max(
                            -accelerationHalfBox,
                            std::min(accelerationHalfBox, candidate[0]));
                }
                if (isDefining(boxOffset + 2)) {
                    candidate[1] = accelerationHalfBox;
                } else if (isDefining(boxOffset + 3)) {
                    candidate[1] = -accelerationHalfBox;
                } else {
                    candidate[1] = std::max(
                            -accelerationHalfBox,
                            std::min(accelerationHalfBox, candidate[1]));
                }
                double achievedGamma = std::numeric_limits<double>::infinity();
                bool pruned = false;
                for (const auto &residual : residuals) {
                    if (work != nullptr) ++work->residual_evaluations;
                    achievedGamma = std::min(
                            achievedGamma,
                            residual.constant
                                    - residual.ax * candidate[0]
                                    - residual.ay * candidate[1]);
                    if (enableEarlyPruning && foundEpigraphVertex &&
                        achievedGamma < bestGamma) {
                        pruned = true;
                        if (work != nullptr) ++work->pruned_vertices;
                        break;
                    }
                }
                if (pruned) continue;
                if (!std::isfinite(achievedGamma)) {
                    continue;
                }
                candidate[2] = achievedGamma;
                if (!bridge_gamma_star_detail::feasible3D(
                            halfspaces,
                            candidate[0], candidate[1], candidate[2])) {
                    continue;
                }
                if (!foundEpigraphVertex || achievedGamma > bestGamma) {
                    bestGamma = achievedGamma;
                    fallbackX = candidate[0];
                    fallbackY = candidate[1];
                    foundEpigraphVertex = true;
                } else if (achievedGamma == bestGamma
                           && bridge_gamma_star_detail::betterMinimumNormWitness(
                                   candidate[0], candidate[1],
                                   fallbackX, fallbackY)) {
                    fallbackX = candidate[0];
                    fallbackY = candidate[1];
                }
            }
        }
    }
    if (!foundEpigraphVertex || !std::isfinite(bestGamma)) {
        return {};
    }

    using bridge_gamma_star_detail::Halfspace2D;
    std::vector<Halfspace2D> optimalFace;
    optimalFace.reserve(residuals.size() + 4);
    for (const auto &residual : residuals) {
        optimalFace.push_back({
                residual.ax,
                residual.ay,
                residual.constant - bestGamma,
        });
    }
    optimalFace.push_back({1.0, 0.0, accelerationHalfBox});
    optimalFace.push_back({-1.0, 0.0, accelerationHalfBox});
    optimalFace.push_back({0.0, 1.0, accelerationHalfBox});
    optimalFace.push_back({0.0, -1.0, accelerationHalfBox});

    double witnessX = fallbackX;
    double witnessY = fallbackY;
    // The fallback was already box-projected, its gamma was recomputed as the
    // exact minimum of all residuals, and the full epigraph point passed the
    // global feasibility check.  It is therefore a constructive optimal-face
    // witness even if re-evaluating an equivalent 2-D inequality orders the
    // floating-point operations differently by a few ulps.
    bool foundWitness = true;
    const auto considerWitness = [&](double x, double y) {
        x = std::max(-accelerationHalfBox,
                     std::min(accelerationHalfBox, x));
        y = std::max(-accelerationHalfBox,
                     std::min(accelerationHalfBox, y));
        if (!bridge_gamma_star_detail::feasible2D(
                    optimalFace, x, y)) {
            return;
        }
        if (!foundWitness
            || bridge_gamma_star_detail::betterMinimumNormWitness(
                    x, y, witnessX, witnessY)) {
            witnessX = x;
            witnessY = y;
            foundWitness = true;
        }
    };

    considerWitness(0.0, 0.0);
    for (const auto &halfspace : optimalFace) {
        const double normalSquared =
                halfspace.x * halfspace.x + halfspace.y * halfspace.y;
        if (normalSquared > 0.0) {
            considerWitness(
                    halfspace.rhs * halfspace.x / normalSquared,
                    halfspace.rhs * halfspace.y / normalSquared);
        }
    }
    for (size_t first = 0; first + 1 < optimalFace.size(); ++first) {
        for (size_t second = first + 1; second < optimalFace.size(); ++second) {
            const auto &a = optimalFace[first];
            const auto &b = optimalFace[second];
            const double determinant = a.x * b.y - a.y * b.x;
            const double rowNormProduct =
                    std::hypot(a.x, a.y) * std::hypot(b.x, b.y);
            const double singularityBound =
                    64.0 * std::numeric_limits<double>::epsilon()
                    * rowNormProduct;
            if (!std::isfinite(determinant)
                || !std::isfinite(singularityBound)
                || std::abs(determinant) <= singularityBound) {
                continue;
            }
            considerWitness(
                    (a.rhs * b.y - a.y * b.rhs) / determinant,
                    (a.x * b.rhs - a.rhs * b.x) / determinant);
        }
    }
    if (!foundWitness) {
        return {};
    }

    double achievedGamma = std::numeric_limits<double>::infinity();
    for (const auto &residual : residuals) {
        achievedGamma = std::min(
                achievedGamma,
                residual.constant
                        - residual.ax * witnessX
                        - residual.ay * witnessY);
    }
    return {true, achievedGamma, witnessX, witnessY};
}

inline BridgeGammaStarSolution2D solveExactBridgeGammaStar2D(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double accelerationHalfBox,
        BridgeGammaStarWork2D *work = nullptr) {
    return solveExactBridgeGammaStar2DImpl(
            residuals, accelerationHalfBox, work, true);
}

inline BridgeGammaStarSolution2D solveExactBridgeGammaStar2DReference(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double accelerationHalfBox) {
    return solveExactBridgeGammaStar2DImpl(
            residuals, accelerationHalfBox, nullptr, false);
}

inline double exactBridgeGammaStar2D(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double accelerationHalfBox) {
    return solveExactBridgeGammaStar2D(
            residuals, accelerationHalfBox).gamma;
}

#endif
