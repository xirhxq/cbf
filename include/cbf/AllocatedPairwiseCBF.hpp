#ifndef CBF_ALLOCATED_PAIRWISE_CBF_HPP
#define CBF_ALLOCATED_PAIRWISE_CBF_HPP

#include "cbf/BarrierEdgeRegistry.hpp"
#include "cbf/FimRateCertificate.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace cbf2026 {

inline constexpr double kBarrierEdgeSeparationTolerance = 1e-8;
inline constexpr double kAllocationSumTolerance = 1e-12;

struct EdgeSnapshot {
    EdgeId edge;
    Eigen::Vector2d normal;
    double separation;
    double nuI;
    double nuJ;
    double alpha;
    std::uint64_t snapshotVersion;
    std::uint64_t allocationVersion;
};

struct EndpointCertificateSnapshot {
    int robotId;
    Eigen::Vector2d estimate;
    Eigen::Matrix2d covariance;
    double covarianceRateBound;
    double epsilon;
    double barNu;
    std::uint64_t snapshotVersion;
    std::uint64_t allocationVersion;
};

inline EndpointCertificateSnapshot makeEndpointCertificateSnapshot(
    const NodeRateCertificate& certificate,
    const Eigen::Vector2d& estimate,
    std::uint64_t allocationVersion
) {
    return {
        certificate.robotId,
        estimate,
        certificate.covariance,
        certificate.covarianceRateBound,
        certificate.epsilon,
        certificate.epsilonRateBound,
        certificate.snapshotVersion,
        allocationVersion
    };
}

inline void validateEndpointCertificateSnapshot(
    int mapKey,
    const EndpointCertificateSnapshot& snapshot
) {
    if (mapKey <= 0 || snapshot.robotId != mapKey) {
        throw std::invalid_argument(
            "endpoint snapshot map key does not match embedded robot ID"
        );
    }
    if (!snapshot.estimate.allFinite()
        || !snapshot.covariance.allFinite()
        || !std::isfinite(snapshot.covarianceRateBound)
        || !std::isfinite(snapshot.epsilon)
        || !std::isfinite(snapshot.barNu)
        || snapshot.covarianceRateBound < 0.0
        || snapshot.epsilon < 0.0
        || snapshot.barNu < 0.0) {
        throw std::invalid_argument(
            "endpoint certificate snapshot contains invalid numeric data"
        );
    }
    const double covarianceScale =
        snapshot.covariance.cwiseAbs().maxCoeff();
    if ((snapshot.covariance - snapshot.covariance.transpose())
            .cwiseAbs().maxCoeff()
        > 1e-12 * covarianceScale) {
        throw std::invalid_argument(
            "endpoint certificate covariance is not symmetric"
        );
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(
        snapshot.covariance
    );
    if (solver.info() != Eigen::Success
        || !solver.eigenvalues().allFinite()
        || solver.eigenvalues().minCoeff() <= 0.0) {
        throw std::invalid_argument(
            "endpoint certificate covariance is not positive definite"
        );
    }
    const double maximumEigenvalue = solver.eigenvalues().maxCoeff();
    const double expectedEpsilon = 3.0 * std::sqrt(maximumEigenvalue);
    const double expectedBarNu =
        3.0 * snapshot.covarianceRateBound
        / (2.0 * std::sqrt(maximumEigenvalue));
    const auto matches = [](double actual, double expected) {
        return std::abs(actual - expected)
               <= 1e-12 * std::max(1.0, std::abs(expected));
    };
    if (!matches(snapshot.epsilon, expectedEpsilon)
        || !matches(snapshot.barNu, expectedBarNu)) {
        throw std::invalid_argument(
            "endpoint certificate radii do not match covariance and rate"
        );
    }
}

struct EndpointRow {
    EdgeId edge;
    int owner;
    Eigen::Vector2d coefficient;
    double constant;
    double allocation;
    std::uint64_t snapshotVersion;
    std::uint64_t allocationVersion;
};

struct FullRow {
    EdgeId edge;
    Eigen::Vector2d coefficientI;
    Eigen::Vector2d coefficientJ;
    double constant;
    std::uint64_t snapshotVersion;
    std::uint64_t allocationVersion;
};

inline EdgeSnapshot makeEdgeSnapshot(
    const EdgeId& edge,
    const Eigen::Vector2d& positionI,
    const Eigen::Vector2d& positionJ,
    double nuI,
    double nuJ,
    double alpha,
    std::uint64_t snapshotVersion,
    std::uint64_t allocationVersion
) {
    validateCanonicalEdge(edge);
    if (!positionI.allFinite() || !positionJ.allFinite()) {
        throw std::invalid_argument("edge positions must be finite");
    }
    const Eigen::Vector2d displacement = positionI - positionJ;
    const double separation = displacement.norm();
    if (!std::isfinite(separation)
        || separation <= kBarrierEdgeSeparationTolerance) {
        throw std::invalid_argument("edge separation is singular");
    }
    return {
        edge,
        displacement / separation,
        separation,
        nuI,
        nuJ,
        alpha,
        snapshotVersion,
        allocationVersion
    };
}

inline void validateEdgeSnapshot(
    const EdgeSnapshot& snapshot,
    EdgeKind expectedKind
) {
    validateCanonicalEdge(snapshot.edge);
    if (snapshot.edge.kind != expectedKind) {
        throw std::invalid_argument("edge kind does not match row formula");
    }
    if (!snapshot.normal.allFinite()
        || std::abs(snapshot.normal.norm() - 1.0) > 1e-12
        || !std::isfinite(snapshot.separation)
        || snapshot.separation <= kBarrierEdgeSeparationTolerance) {
        throw std::invalid_argument(
            "edge geometry must contain a finite unit normal and separation"
        );
    }
    if (!std::isfinite(snapshot.nuI)
        || !std::isfinite(snapshot.nuJ)
        || snapshot.nuI < 0.0
        || snapshot.nuJ < 0.0
        || !std::isfinite(snapshot.alpha)) {
        throw std::invalid_argument(
            "edge rates and class-K value are invalid"
        );
    }
    if (snapshot.edge.baseId >= 0 && snapshot.nuJ != 0.0) {
        throw std::invalid_argument("hovering base rate must be zero");
    }
}

inline void validateAllocation(
    const EdgeSnapshot& snapshot,
    double allocationI,
    double allocationJ
) {
    validateCanonicalEdge(snapshot.edge);
    if (!std::isfinite(allocationI)
        || !std::isfinite(allocationJ)
        || allocationI < 0.0
        || allocationJ < 0.0
        || std::abs(allocationI + allocationJ - 1.0)
           > kAllocationSumTolerance) {
        throw std::invalid_argument(
            "allocations must be finite, nonnegative, and sum to one"
        );
    }
    if (snapshot.edge.baseId >= 0
        && (std::abs(allocationI - 1.0) > kAllocationSumTolerance
            || std::abs(allocationJ) > kAllocationSumTolerance)) {
        throw std::invalid_argument(
            "hovering base edge requires complete UAV allocation"
        );
    }
}

inline std::vector<EndpointRow> allocatedLocalizationRows(
    const EdgeSnapshot& snapshot,
    double allocationI,
    double allocationJ
) {
    validateEdgeSnapshot(snapshot, EdgeKind::Localization);
    validateAllocation(snapshot, allocationI, allocationJ);
    if (snapshot.edge.baseId >= 0) {
        return {
            {
                snapshot.edge,
                snapshot.edge.low,
                -snapshot.normal,
                -snapshot.nuI + allocationI * snapshot.alpha,
                allocationI,
                snapshot.snapshotVersion,
                snapshot.allocationVersion
            }
        };
    }
    return {
        {
            snapshot.edge,
            snapshot.edge.low,
            -snapshot.normal,
            -snapshot.nuI + allocationI * snapshot.alpha,
            allocationI,
            snapshot.snapshotVersion,
            snapshot.allocationVersion
        },
        {
            snapshot.edge,
            snapshot.edge.high,
            snapshot.normal,
            -snapshot.nuJ + allocationJ * snapshot.alpha,
            allocationJ,
            snapshot.snapshotVersion,
            snapshot.allocationVersion
        }
    };
}

inline std::vector<EndpointRow> allocatedCollisionRows(
    const EdgeSnapshot& snapshot,
    double allocationI,
    double allocationJ
) {
    validateEdgeSnapshot(snapshot, EdgeKind::Collision);
    validateAllocation(snapshot, allocationI, allocationJ);
    return {
        {
            snapshot.edge,
            snapshot.edge.low,
            snapshot.normal,
            -snapshot.nuI + allocationI * snapshot.alpha,
            allocationI,
            snapshot.snapshotVersion,
            snapshot.allocationVersion
        },
        {
            snapshot.edge,
            snapshot.edge.high,
            -snapshot.normal,
            -snapshot.nuJ + allocationJ * snapshot.alpha,
            allocationJ,
            snapshot.snapshotVersion,
            snapshot.allocationVersion
        }
    };
}

inline std::vector<EndpointRow> allocatedRows(
    const EdgeSnapshot& snapshot,
    double allocationI,
    double allocationJ
) {
    switch (snapshot.edge.kind) {
        case EdgeKind::Localization:
            return allocatedLocalizationRows(
                snapshot, allocationI, allocationJ
            );
        case EdgeKind::Collision:
            return allocatedCollisionRows(
                snapshot, allocationI, allocationJ
            );
    }
    throw std::invalid_argument("edge kind is unsupported");
}

inline void validateEndpointRowNumeric(const EndpointRow& row) {
    if (!row.coefficient.allFinite()
        || !std::isfinite(row.constant)
        || !std::isfinite(row.allocation)
        || row.allocation < 0.0) {
        throw std::invalid_argument(
            "endpoint row contains invalid numeric data"
        );
    }
}

inline FullRow reconstructFullRow(const std::vector<EndpointRow>& rows) {
    if (rows.size() == 1) {
        const auto& row = rows.front();
        validateEndpointRowNumeric(row);
        validateCanonicalEdge(row.edge);
        if (row.edge.baseId < 0
            || row.owner != row.edge.low
            || !std::isfinite(row.allocation)
            || std::abs(row.allocation - 1.0)
               > kAllocationSumTolerance) {
            throw std::invalid_argument(
                "single endpoint row is not a full-allocation base edge"
            );
        }
        return {
            row.edge,
            row.coefficient,
            Eigen::Vector2d::Zero(),
            row.constant,
            row.snapshotVersion,
            row.allocationVersion
        };
    }
    if (rows.size() != 2) {
        throw std::invalid_argument(
            "paired full-row reconstruction requires two endpoint rows"
        );
    }
    const auto& first = rows[0];
    const auto& second = rows[1];
    validateEndpointRowNumeric(first);
    validateEndpointRowNumeric(second);
    validateCanonicalEdge(first.edge);
    if (first.edge.baseId >= 0
        || first.edge != second.edge
        || first.snapshotVersion != second.snapshotVersion
        || first.allocationVersion != second.allocationVersion
        || first.owner == second.owner
        || std::abs(first.allocation + second.allocation - 1.0)
           > kAllocationSumTolerance) {
        throw std::invalid_argument(
            "endpoint rows do not share one canonical allocation snapshot"
        );
    }
    const EndpointRow* lowRow = nullptr;
    const EndpointRow* highRow = nullptr;
    for (const auto& row : rows) {
        if (row.owner == first.edge.low) {
            lowRow = &row;
        } else if (row.owner == first.edge.high) {
            highRow = &row;
        } else {
            throw std::invalid_argument(
                "endpoint row owner is not incident to its edge"
            );
        }
    }
    if (lowRow == nullptr || highRow == nullptr) {
        throw std::invalid_argument("endpoint row owner is duplicated");
    }
    return {
        first.edge,
        lowRow->coefficient,
        highRow->coefficient,
        lowRow->constant + highRow->constant,
        first.snapshotVersion,
        first.allocationVersion
    };
}

inline Eigen::VectorXd endpointRowToModelControl(
    const EndpointRow& row,
    int controlSize
) {
    if (controlSize < 2) {
        throw std::invalid_argument(
            "endpoint row requires at least two model controls"
        );
    }
    if (!row.coefficient.allFinite()) {
        throw std::invalid_argument(
            "endpoint coefficient must be finite"
        );
    }
    Eigen::VectorXd control = Eigen::VectorXd::Zero(controlSize);
    control.head<2>() = row.coefficient;
    return control;
}

inline std::string allocatedFeasibilityLabel(
    bool coupledFeasible,
    const std::vector<bool>& endpointFeasibility
) {
    if (endpointFeasibility.empty()) {
        return "missing_data";
    }
    const bool everyEndpointFeasible = std::all_of(
        endpointFeasibility.begin(),
        endpointFeasibility.end(),
        [](bool feasible) { return feasible; }
    );
    if (everyEndpointFeasible) {
        return "feasible";
    }
    if (coupledFeasible) {
        return "allocation_conservatism";
    }
    return "coupled_row_infeasible";
}

}

#endif
