#ifndef CBF_PAIRWISE_SECOND_ORDER_CBF_HPP
#define CBF_PAIRWISE_SECOND_ORDER_CBF_HPP

#include "utils.h"
#include <cmath>
#include <stdexcept>

struct PairwiseDistanceKinematics {
    double distance = 0.0;
    Eigen::Vector2d normal = Eigen::Vector2d::Zero();
    Eigen::Vector2d relativeVelocity = Eigen::Vector2d::Zero();
    double radialVelocity = 0.0;
    double curvature = 0.0;
};

enum class PairwiseSecondOrderBarrierKind {
    CollisionLower,
    CommunicationUpper
};

struct PairwiseSecondOrderState2D {
    Point position;
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
};

struct PairwiseSecondOrderRowSpec {
    PairwiseSecondOrderBarrierKind kind = PairwiseSecondOrderBarrierKind::CollisionLower;
    double distanceLimit = 0.0;
    double uncertainty = 0.0;
    double k = 1.0;
    double lambda1 = 1.0;
    double lambda2 = 1.0;
    double totalReserve = 0.0;
};

struct PairwiseSecondOrderRow {
    Eigen::Vector2d uCoe = Eigen::Vector2d::Zero();
    double h = 0.0;
    double hdot = 0.0;
    double hddotConst = 0.0;
    double psi1 = 0.0;
    double constTerm = 0.0;
    double totalReserve = 0.0;
};

inline PairwiseDistanceKinematics computePairwiseDistanceKinematics(
        const Point &pi,
        const Point &pj,
        const VectorXd &vi,
        const VectorXd &vj) {
    if (vi.size() < 2 || vj.size() < 2) {
        throw std::invalid_argument("Pairwise velocities must have at least two entries");
    }

    Eigen::Vector2d r(pi.x - pj.x, pi.y - pj.y);
    double distance = r.norm();
    if (distance < 1e-9) {
        throw std::invalid_argument("Pairwise distance is too small for second-order CBF evaluation");
    }

    PairwiseDistanceKinematics terms;
    terms.distance = distance;
    terms.normal = r / distance;
    terms.relativeVelocity << vi(0) - vj(0), vi(1) - vj(1);
    terms.radialVelocity = terms.normal.dot(terms.relativeVelocity);
    double speedSquared = terms.relativeVelocity.squaredNorm();
    terms.curvature = (speedSquared - terms.radialVelocity * terms.radialVelocity) / distance;
    return terms;
}

inline PairwiseSecondOrderRow buildPairwiseSecondOrderRow(
        const PairwiseSecondOrderState2D &self,
        const PairwiseSecondOrderState2D &reference,
        const PairwiseSecondOrderRowSpec &spec) {
    const bool statesFinite = std::isfinite(self.position.x)
                              && std::isfinite(self.position.y)
                              && std::isfinite(reference.position.x)
                              && std::isfinite(reference.position.y)
                              && self.velocity.allFinite()
                              && self.acceleration.allFinite()
                              && reference.velocity.allFinite()
                              && reference.acceleration.allFinite();
    if (!statesFinite) {
        throw std::invalid_argument("pairwise second-order states must be finite");
    }
    if (!std::isfinite(spec.distanceLimit) || spec.distanceLimit <= 0.0
        || !std::isfinite(spec.uncertainty) || spec.uncertainty < 0.0
        || !std::isfinite(spec.k) || spec.k <= 0.0
        || !std::isfinite(spec.lambda1) || spec.lambda1 <= 0.0
        || !std::isfinite(spec.lambda2) || spec.lambda2 <= 0.0
        || !std::isfinite(spec.totalReserve) || spec.totalReserve < 0.0) {
        throw std::invalid_argument("pairwise second-order row parameters are invalid");
    }

    VectorXd selfVelocity = self.velocity;
    VectorXd referenceVelocity = reference.velocity;
    const auto terms = computePairwiseDistanceKinematics(
        self.position,
        reference.position,
        selfVelocity,
        referenceVelocity);
    const double neighbourRadialAcceleration =
        terms.normal.dot(reference.acceleration);

    PairwiseSecondOrderRow row;
    row.totalReserve = spec.totalReserve;
    if (spec.kind == PairwiseSecondOrderBarrierKind::CollisionLower) {
        row.uCoe = spec.k * terms.normal;
        row.h = spec.k * (terms.distance - spec.distanceLimit - spec.uncertainty);
        row.hdot = spec.k * terms.radialVelocity;
        row.hddotConst = spec.k * (terms.curvature - neighbourRadialAcceleration);
    } else {
        row.uCoe = -spec.k * terms.normal;
        row.h = spec.k * (spec.distanceLimit - terms.distance - spec.uncertainty);
        row.hdot = -spec.k * terms.radialVelocity;
        row.hddotConst = spec.k * (neighbourRadialAcceleration - terms.curvature);
    }

    const double k1 = spec.lambda1 + spec.lambda2;
    const double k0 = spec.lambda1 * spec.lambda2;
    row.psi1 = row.hdot + spec.lambda1 * row.h;
    row.constTerm = row.hddotConst + k1 * row.hdot + k0 * row.h - spec.totalReserve;
    return row;
}

#endif
