#ifndef CBF_PAIRWISE_SECOND_ORDER_CBF_HPP
#define CBF_PAIRWISE_SECOND_ORDER_CBF_HPP

#include "utils.h"
#include <stdexcept>

struct PairwiseDistanceKinematics {
    double distance = 0.0;
    Eigen::Vector2d normal = Eigen::Vector2d::Zero();
    Eigen::Vector2d relativeVelocity = Eigen::Vector2d::Zero();
    double radialVelocity = 0.0;
    double curvature = 0.0;
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

#endif
