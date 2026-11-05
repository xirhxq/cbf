#ifndef CBF_FIM_RATE_CERTIFICATE_HPP
#define CBF_FIM_RATE_CERTIFICATE_HPP

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cbf2026 {

inline constexpr double kFimRateSingularDistanceTolerance = 1e-8;
inline constexpr double kFimRateRelativeSpectralThreshold = 1e-12;
inline constexpr double kFimRateUnitDirectionTolerance = 1e-12;

// Frozen reference IDs use disjoint namespaces: UAV IDs stay nonnegative,
// while base b is encoded as -(b + 1).
inline int canonicalBaseReferenceId(int baseId) {
    if (baseId < 0) {
        throw std::invalid_argument("base reference ID must be nonnegative");
    }
    return static_cast<int>(-static_cast<std::int64_t>(baseId) - 1);
}

inline int canonicalUavReferenceId(int robotId) {
    if (robotId < 0) {
        throw std::invalid_argument("UAV reference ID must be nonnegative");
    }
    return robotId;
}

inline double spectralNormSymmetric(const Eigen::Matrix2d& matrix) {
    if (!matrix.allFinite()) {
        throw std::invalid_argument("symmetric matrix contains nonfinite data");
    }
    const double scale = matrix.cwiseAbs().maxCoeff();
    if ((matrix - matrix.transpose()).cwiseAbs().maxCoeff()
        > kFimRateRelativeSpectralThreshold * scale) {
        throw std::invalid_argument("matrix is not symmetric");
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(matrix);
    if (solver.info() != Eigen::Success
        || !solver.eigenvalues().allFinite()) {
        throw std::invalid_argument("symmetric eigendecomposition failed");
    }
    return solver.eigenvalues().cwiseAbs().maxCoeff();
}

struct PredecessorRateSnapshot {
    int referenceId;
    int localIndex;
    std::uint64_t snapshotVersion;
    bool hoveringBase;
    Eigen::Matrix2d covariance;
    double covarianceRateBound;
    double speedBound;
};

struct ReferenceRateInput {
    PredecessorRateSnapshot predecessor;
    double distance;
    Eigen::Vector2d direction;
    double rangingVariance;
};

struct NodeRateInput {
    int robotId;
    int localIndex;
    std::uint64_t snapshotVersion;
    double planarComponentMax;
    std::vector<ReferenceRateInput> references;
};

struct FrozenReferenceTerm {
    int referenceId;
    Eigen::Vector2d direction;
    double effectiveVariance;
};

struct ReferenceRealizedDerivative {
    int referenceId;
    double dotEffectiveVariance;
    Eigen::Vector2d dotDirection;
};

struct NodeRateCertificate {
    int robotId;
    std::uint64_t snapshotVersion;
    Eigen::Matrix2d information;
    Eigen::Matrix2d covariance;
    double epsilon;
    double informationRateBound;
    double covarianceRateBound;
    double epsilonRateBound;
    std::vector<FrozenReferenceTerm> frozenReferences;
};

inline NodeRateCertificate baseRateCertificate(
    int baseId,
    std::uint64_t snapshotVersion
) {
    if (baseId < 0) {
        throw std::invalid_argument("base ID must be nonnegative");
    }
    return {
        baseId,
        snapshotVersion,
        Eigen::Matrix2d::Zero(),
        Eigen::Matrix2d::Zero(),
        0.0,
        0.0,
        0.0,
        0.0,
        {}
    };
}

inline NodeRateCertificate computeNodeRateCertificate(
    const NodeRateInput& input
) {
    if (input.localIndex <= 0) {
        throw std::invalid_argument("node local index must be positive");
    }
    if (!std::isfinite(input.planarComponentMax)
        || input.planarComponentMax <= 0.0) {
        throw std::invalid_argument(
            "planar component bound must be finite and positive"
        );
    }

    Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
    double informationRateBound = 0.0;
    std::vector<FrozenReferenceTerm> frozenReferences;
    frozenReferences.reserve(input.references.size());
    std::unordered_set<int> referenceIds;
    bool sawUavReference = false;
    int previousBaseReferenceId = 0;
    int previousUavLocalIndex = 0;

    for (const auto& reference : input.references) {
        if (reference.predecessor.hoveringBase) {
            if (sawUavReference
                || reference.predecessor.referenceId
                   >= previousBaseReferenceId) {
                throw std::invalid_argument(
                    "references are not in canonical bases-first order"
                );
            }
            previousBaseReferenceId =
                reference.predecessor.referenceId;
        } else {
            sawUavReference = true;
            if (reference.predecessor.localIndex
                <= previousUavLocalIndex) {
                throw std::invalid_argument(
                    "UAV references are not in increasing local-index order"
                );
            }
            previousUavLocalIndex =
                reference.predecessor.localIndex;
        }
        if (reference.predecessor.snapshotVersion != input.snapshotVersion
            || (reference.predecessor.hoveringBase
                && reference.predecessor.localIndex != 0)
            || (!reference.predecessor.hoveringBase
                && (reference.predecessor.localIndex <= 0
                    || reference.predecessor.localIndex >= input.localIndex))) {
            throw std::invalid_argument(
                "reference is not a same-version predecessor"
            );
        }
        if (reference.predecessor.hoveringBase
            != (reference.predecessor.referenceId < 0)) {
            throw std::invalid_argument(
                "reference identity does not match its base/UAV kind"
            );
        }
        if (!referenceIds.insert(reference.predecessor.referenceId).second) {
            throw std::invalid_argument("reference identity is duplicated");
        }
        if (!std::isfinite(reference.distance)
            || reference.distance <= kFimRateSingularDistanceTolerance) {
            throw std::invalid_argument(
                "reference distance is singular or nonfinite"
            );
        }
        if (!reference.direction.allFinite()
            || std::abs(reference.direction.norm() - 1.0)
               > kFimRateUnitDirectionTolerance) {
            throw std::invalid_argument(
                "reference direction must be a finite unit vector"
            );
        }
        if (!std::isfinite(reference.rangingVariance)
            || reference.rangingVariance <= 0.0) {
            throw std::invalid_argument(
                "ranging variance must be finite and positive"
            );
        }
        if (!std::isfinite(reference.predecessor.covarianceRateBound)
            || reference.predecessor.covarianceRateBound < 0.0
            || !std::isfinite(reference.predecessor.speedBound)
            || reference.predecessor.speedBound < 0.0) {
            throw std::invalid_argument(
                "predecessor rate bounds must be finite and nonnegative"
            );
        }
        const double predecessorNorm =
            spectralNormSymmetric(reference.predecessor.covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> predecessorSolver(
            reference.predecessor.covariance
        );
        const double predecessorScale =
            reference.predecessor.covariance.cwiseAbs().maxCoeff();
        if (predecessorSolver.eigenvalues().minCoeff()
            < -kFimRateRelativeSpectralThreshold * predecessorScale) {
            throw std::invalid_argument(
                "predecessor covariance is not positive semidefinite"
            );
        }
        if (reference.predecessor.hoveringBase
            && (predecessorNorm != 0.0
                || reference.predecessor.covarianceRateBound != 0.0
                || reference.predecessor.speedBound != 0.0)) {
            throw std::invalid_argument(
                "hovering base predecessor must have zero covariance and rates"
            );
        }
        const double effectiveVariance = (
            reference.direction.transpose()
            * reference.predecessor.covariance
            * reference.direction
        )(0, 0) + reference.rangingVariance;
        const double beta = (
            std::sqrt(2.0) * input.planarComponentMax
            + reference.predecessor.speedBound
        ) / reference.distance;
        const double effectiveVarianceRateBound =
            2.0 * beta * predecessorNorm
            + reference.predecessor.covarianceRateBound;

        information.noalias() += (
            reference.direction * reference.direction.transpose()
        ) / effectiveVariance;
        informationRateBound +=
            effectiveVarianceRateBound / std::pow(effectiveVariance, 2)
            + 2.0 * beta / effectiveVariance;
        frozenReferences.push_back({
            reference.predecessor.referenceId,
            reference.direction,
            effectiveVariance
        });
    }

    if (!information.allFinite() || !std::isfinite(informationRateBound)) {
        throw std::invalid_argument(
            "information matrix or rate bound contains nonfinite data"
        );
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> informationSolver(
        information
    );
    if (informationSolver.info() != Eigen::Success
        || !informationSolver.eigenvalues().allFinite()) {
        throw std::invalid_argument("information eigendecomposition failed");
    }
    const double minimumInformationEigenvalue =
        informationSolver.eigenvalues().minCoeff();
    const double maximumInformationEigenvalue =
        informationSolver.eigenvalues().maxCoeff();
    if (maximumInformationEigenvalue <= 0.0
        || minimumInformationEigenvalue
           <= kFimRateRelativeSpectralThreshold
              * maximumInformationEigenvalue) {
        throw std::invalid_argument(
            "information matrix is not positive definite or exceeds the condition limit"
        );
    }

    Eigen::Matrix2d covariance = information.inverse();
    covariance = 0.5 * (covariance + covariance.transpose());
    if (!covariance.allFinite()) {
        throw std::invalid_argument("covariance contains nonfinite data");
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> covarianceSolver(covariance);
    if (covarianceSolver.info() != Eigen::Success
        || !covarianceSolver.eigenvalues().allFinite()
        || covarianceSolver.eigenvalues().minCoeff() <= 0.0) {
        throw std::invalid_argument("covariance is not positive definite");
    }
    const double maximumCovarianceEigenvalue =
        covarianceSolver.eigenvalues().maxCoeff();
    const double covarianceNorm = spectralNormSymmetric(covariance);
    const double covarianceRateBound =
        std::pow(covarianceNorm, 2) * informationRateBound;
    const double epsilon = 3.0 * std::sqrt(maximumCovarianceEigenvalue);
    const double epsilonRateBound =
        3.0 * covarianceRateBound
        / (2.0 * std::sqrt(maximumCovarianceEigenvalue));
    if (!std::isfinite(epsilon)
        || !std::isfinite(covarianceRateBound)
        || !std::isfinite(epsilonRateBound)) {
        throw std::invalid_argument("computed rate certificate is nonfinite");
    }

    return {
        input.robotId,
        input.snapshotVersion,
        information,
        covariance,
        epsilon,
        informationRateBound,
        covarianceRateBound,
        epsilonRateBound,
        std::move(frozenReferences)
    };
}

inline double realizedEpsilonRate(
    const NodeRateCertificate& certificate,
    const std::vector<ReferenceRealizedDerivative>& derivatives
) {
    if (derivatives.size() != certificate.frozenReferences.size()) {
        throw std::invalid_argument(
            "realized derivative set does not match frozen references"
        );
    }
    if (certificate.frozenReferences.empty()) {
        if (certificate.information.allFinite()
            && certificate.covariance.allFinite()
            && certificate.information.cwiseAbs().maxCoeff() == 0.0
            && certificate.covariance.cwiseAbs().maxCoeff() == 0.0
            && certificate.epsilon == 0.0
            && certificate.informationRateBound == 0.0
            && certificate.covarianceRateBound == 0.0
            && certificate.epsilonRateBound == 0.0) {
            return 0.0;
        }
        throw std::invalid_argument(
            "empty realized derivative set requires a zero base certificate"
        );
    }
    std::unordered_set<int> derivativeIds;
    for (const auto& derivative : derivatives) {
        if (!derivativeIds.insert(derivative.referenceId).second) {
            throw std::invalid_argument("realized derivative is duplicated");
        }
        if (!std::isfinite(derivative.dotEffectiveVariance)
            || !derivative.dotDirection.allFinite()) {
            throw std::invalid_argument(
                "realized derivative contains nonfinite data"
            );
        }
    }

    Eigen::Matrix2d dotInformation = Eigen::Matrix2d::Zero();
    for (const auto& frozen : certificate.frozenReferences) {
        const auto derivative = std::find_if(
            derivatives.begin(),
            derivatives.end(),
            [&frozen](const ReferenceRealizedDerivative& candidate) {
                return candidate.referenceId == frozen.referenceId;
            }
        );
        if (derivative == derivatives.end()) {
            throw std::invalid_argument(
                "realized derivative is missing for a frozen reference"
            );
        }
        dotInformation.noalias() +=
            -derivative->dotEffectiveVariance
             / std::pow(frozen.effectiveVariance, 2)
             * (frozen.direction * frozen.direction.transpose())
            + (
                derivative->dotDirection * frozen.direction.transpose()
                + frozen.direction * derivative->dotDirection.transpose()
            ) / frozen.effectiveVariance;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> covarianceSolver(
        certificate.covariance
    );
    const double maximumCovarianceEigenvalue =
        covarianceSolver.eigenvalues().maxCoeff();
    const double covarianceNorm =
        spectralNormSymmetric(certificate.covariance);
    const double realizedRate =
        3.0 * std::pow(covarianceNorm, 2)
        * spectralNormSymmetric(dotInformation)
        / (2.0 * std::sqrt(maximumCovarianceEigenvalue));
    return realizedRate;
}

}  // namespace cbf2026

#endif
