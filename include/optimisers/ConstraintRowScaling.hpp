#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

inline double exactEquivalentConstraintRowScale(
    const Eigen::VectorXd& coefficient,double rhs) {
    return 1.0/std::max({1.0,std::abs(rhs),
        coefficient.lpNorm<Eigen::Infinity>()});
}
