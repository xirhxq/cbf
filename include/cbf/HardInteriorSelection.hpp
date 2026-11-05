#ifndef CBF_HARD_INTERIOR_SELECTION_HPP
#define CBF_HARD_INTERIOR_SELECTION_HPP

#include "cbf/HybridCertificateGuard.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace cbf2026 {

struct PlanarChebyshevResult {
    double radius = 0.0;
    Eigen::Vector2d witness = Eigen::Vector2d::Zero();
    std::vector<std::size_t> tightHardRows;
};

namespace detail {

struct PlanarChebyshevPlane {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double constant = 0.0;
};

inline void validatePlanarHardRowProblem(
    const HardConstraintProblem& problem,
    double feasibilityTolerance
) {
    if (problem.controlSize != 3
        || !std::isfinite(problem.planarComponentMax)
        || problem.planarComponentMax <= 0.0
        || !std::isfinite(problem.yawRateMax)
        || problem.yawRateMax <= 0.0
        || !std::isfinite(feasibilityTolerance)
        || feasibilityTolerance < 0.0) {
        throw std::invalid_argument("planar hard-row problem is malformed");
    }

    const std::array<HardInputBound, 6> requiredBounds = {{
        {0, 1.0, problem.planarComponentMax},
        {0, -1.0, problem.planarComponentMax},
        {1, 1.0, problem.planarComponentMax},
        {1, -1.0, problem.planarComponentMax},
        {2, 1.0, problem.yawRateMax},
        {2, -1.0, problem.yawRateMax}
    }};
    std::array<bool, requiredBounds.size()> found = {};
    if (problem.bounds.size() != requiredBounds.size()) {
        throw std::invalid_argument("hard input bounds are incomplete");
    }
    for (const auto& bound : problem.bounds) {
        if (!std::isfinite(bound.coefficient) || !std::isfinite(bound.limit)) {
            throw std::invalid_argument("hard input bounds must be finite");
        }
        bool matched = false;
        for (std::size_t index = 0; index < requiredBounds.size(); ++index) {
            const auto& required = requiredBounds[index];
            if (bound.controlIndex == required.controlIndex
                && bound.coefficient == required.coefficient
                && bound.limit == required.limit) {
                if (found[index]) {
                    throw std::invalid_argument("hard input bounds are duplicated");
                }
                found[index] = true;
                matched = true;
                break;
            }
        }
        if (!matched) {
            throw std::invalid_argument("hard input bound is unsupported");
        }
    }
    for (const bool present : found) {
        if (!present) {
            throw std::invalid_argument("hard input bounds are incomplete");
        }
    }

    for (const auto& row : problem.rows) {
        if (row.coefficients.size() != 3
            || !row.coefficients.allFinite()
            || !std::isfinite(row.constant)
            || row.coefficients[2] != 0.0) {
            throw std::invalid_argument("hard row is not planar and finite");
        }
    }
}

inline bool lexicographicallyBefore(
    const Eigen::Vector2d& lhs,
    const Eigen::Vector2d& rhs
) {
    return lhs.x() < rhs.x()
        || (lhs.x() == rhs.x() && lhs.y() < rhs.y());
}

} // namespace detail

inline PlanarChebyshevResult solvePlanarHardRowChebyshev(
    const HardConstraintProblem& problem,
    double feasibilityTolerance = 1e-9
) {
    detail::validatePlanarHardRowProblem(problem, feasibilityTolerance);

    std::vector<detail::PlanarChebyshevPlane> planes;
    planes.reserve(problem.rows.size() + 4);
    for (const auto& row : problem.rows) {
        planes.push_back({
            Eigen::Vector3d(-row.coefficients[0], -row.coefficients[1], 1.0),
            row.constant
        });
    }
    const double componentMax = problem.planarComponentMax;
    planes.push_back({Eigen::Vector3d(1.0, 0.0, 0.0), componentMax});
    planes.push_back({Eigen::Vector3d(-1.0, 0.0, 0.0), componentMax});
    planes.push_back({Eigen::Vector3d(0.0, 1.0, 0.0), componentMax});
    planes.push_back({Eigen::Vector3d(0.0, -1.0, 0.0), componentMax});

    std::vector<Eigen::Vector3d> feasibleCandidates;
    double maximumRadius = -std::numeric_limits<double>::infinity();
    for (std::size_t first = 0; first < planes.size(); ++first) {
        for (std::size_t second = first + 1; second < planes.size(); ++second) {
            for (std::size_t third = second + 1; third < planes.size(); ++third) {
                Eigen::Matrix3d matrix;
                matrix.row(0) = planes[first].normal.transpose();
                matrix.row(1) = planes[second].normal.transpose();
                matrix.row(2) = planes[third].normal.transpose();
                const Eigen::FullPivLU<Eigen::Matrix3d> decomposition(matrix);
                if (!decomposition.isInvertible()) {
                    continue;
                }
                const Eigen::Vector3d candidate = decomposition.solve(
                    Eigen::Vector3d(
                        planes[first].constant,
                        planes[second].constant,
                        planes[third].constant
                    )
                );
                if (!candidate.allFinite()) {
                    continue;
                }
                bool feasible = true;
                for (const auto& plane : planes) {
                    const double residual = plane.normal.dot(candidate)
                        - plane.constant;
                    if (!std::isfinite(residual)
                        || residual > feasibilityTolerance) {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible) {
                    continue;
                }
                feasibleCandidates.push_back(candidate);
                maximumRadius = std::max(maximumRadius, candidate[2]);
            }
        }
    }
    if (feasibleCandidates.empty()) {
        throw std::runtime_error("planar Chebyshev problem has no finite vertex");
    }

    bool selectedCandidate = false;
    PlanarChebyshevResult result;
    for (const auto& candidate : feasibleCandidates) {
        if (maximumRadius - candidate[2] > feasibilityTolerance) {
            continue;
        }
        const Eigen::Vector2d witness = candidate.head<2>();
        if (!selectedCandidate
            || detail::lexicographicallyBefore(witness, result.witness)
            || (witness == result.witness && candidate[2] < result.radius)) {
            selectedCandidate = true;
            result.radius = candidate[2];
            result.witness = witness;
        }
    }

    for (std::size_t index = 0; index < problem.rows.size(); ++index) {
        const auto& row = problem.rows[index];
        const double slack = row.coefficients.head<2>().dot(result.witness)
            + row.constant - result.radius;
        if (std::abs(slack) <= feasibilityTolerance) {
            result.tightHardRows.push_back(index);
        }
    }
    return result;
}

inline double frozenInteriorFloor(
    double radius,
    double fraction = 0.10,
    double capMps = 0.10,
    double feasibilityTolerance = 1e-9
) {
    if (!std::isfinite(radius)
        || !std::isfinite(fraction)
        || fraction < 0.0
        || !std::isfinite(capMps)
        || capMps < 0.0
        || !std::isfinite(feasibilityTolerance)
        || feasibilityTolerance < 0.0) {
        throw std::invalid_argument("frozen interior floor arguments are invalid");
    }
    if (radius <= feasibilityTolerance) {
        return 0.0;
    }
    return std::min(
        capMps, fraction * std::max(0.0, radius - feasibilityTolerance)
    );
}

} // namespace cbf2026

#endif
