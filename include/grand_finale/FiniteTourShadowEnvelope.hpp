#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace gf {

inline std::size_t completeRangeSlotCount(
    std::size_t mobile_count,
    std::size_t fixed_count) {
    if (mobile_count == 0)
        throw std::invalid_argument("mobile count must be positive");
    return mobile_count * (mobile_count - 1) / 2 +
           mobile_count * fixed_count;
}

struct ShadowStateBox {
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};

struct ScalarUpdateBound {
    Eigen::VectorXd absolute_gain_bound;
    double accepted_innovation_bound = 0.0;
};

namespace finite_tour_shadow_detail {

inline double downward(double value) {
    return std::nextafter(value, -std::numeric_limits<double>::infinity());
}

inline double upward(double value) {
    return std::nextafter(value, std::numeric_limits<double>::infinity());
}

inline void requireBox(const ShadowStateBox& box) {
    if (box.lower.size() == 0 || box.lower.size() != box.upper.size() ||
        !box.lower.allFinite() || !box.upper.allFinite() ||
        (box.lower.array() > box.upper.array()).any())
        throw std::invalid_argument("invalid shadow state box");
}

inline std::pair<double, double> scaledInterval(
    double coefficient,
    double lower,
    double upper) {
    if (coefficient >= 0.0)
        return {coefficient * lower, coefficient * upper};
    return {coefficient * upper, coefficient * lower};
}

}  // namespace finite_tour_shadow_detail

inline ShadowStateBox accumulateScalarUpdateBatch(
    const ShadowStateBox& prior,
    const std::vector<ScalarUpdateBound>& updates) {
    using namespace finite_tour_shadow_detail;
    requireBox(prior);
    ShadowStateBox result = prior;
    for (const ScalarUpdateBound& update : updates) {
        if (!std::isfinite(update.accepted_innovation_bound) ||
            update.accepted_innovation_bound < 0.0)
            throw std::invalid_argument(
                "accepted innovation bound must be finite and non-negative");
        if (update.absolute_gain_bound.size() != prior.lower.size() ||
            !update.absolute_gain_bound.allFinite() ||
            (update.absolute_gain_bound.array() < 0.0).any())
            throw std::invalid_argument(
                "absolute gain bound must match the shadow dimension");
        for (Eigen::Index index = 0; index < result.lower.size(); ++index) {
            const double radius = update.absolute_gain_bound(index) *
                                  update.accepted_innovation_bound;
            result.lower(index) = downward(result.lower(index) - radius);
            result.upper(index) = upward(result.upper(index) + radius);
        }
    }
    return result;
}

inline ShadowStateBox propagateShadowPrediction(
    const ShadowStateBox& prior,
    const Eigen::MatrixXd& transition,
    const Eigen::MatrixXd& disturbance_input,
    const Eigen::VectorXd& absolute_disturbance_bound) {
    using namespace finite_tour_shadow_detail;
    requireBox(prior);
    if (transition.rows() != prior.lower.size() ||
        transition.cols() != prior.lower.size() ||
        disturbance_input.rows() != prior.lower.size() ||
        disturbance_input.cols() != absolute_disturbance_bound.size() ||
        !transition.allFinite() || !disturbance_input.allFinite() ||
        !absolute_disturbance_bound.allFinite() ||
        (absolute_disturbance_bound.array() < 0.0).any())
        throw std::invalid_argument("invalid shadow prediction input");

    ShadowStateBox result{
        Eigen::VectorXd::Zero(prior.lower.size()),
        Eigen::VectorXd::Zero(prior.lower.size())};
    for (Eigen::Index row = 0; row < transition.rows(); ++row) {
        double lower = 0.0;
        double upper = 0.0;
        for (Eigen::Index column = 0; column < transition.cols(); ++column) {
            const auto interval = scaledInterval(
                transition(row, column),
                prior.lower(column), prior.upper(column));
            lower += interval.first;
            upper += interval.second;
        }
        double disturbance_radius = 0.0;
        for (Eigen::Index column = 0;
             column < disturbance_input.cols(); ++column) {
            disturbance_radius +=
                std::abs(disturbance_input(row, column)) *
                absolute_disturbance_bound(column);
        }
        result.lower(row) = downward(lower - disturbance_radius);
        result.upper(row) = upward(upper + disturbance_radius);
    }
    return result;
}

inline double relativePositionSupport(
    const ShadowStateBox& box,
    std::size_t first_mobile_index,
    std::optional<std::size_t> second_mobile_index,
    const Eigen::Vector2d& direction) {
    using namespace finite_tour_shadow_detail;
    requireBox(box);
    if (box.lower.size() % 4 != 0 || !direction.allFinite())
        throw std::invalid_argument("invalid relative support input");
    const std::size_t mobile_count =
        static_cast<std::size_t>(box.lower.size() / 4);
    if (first_mobile_index >= mobile_count ||
        (second_mobile_index.has_value() &&
         *second_mobile_index >= mobile_count))
        throw std::invalid_argument("relative support mobile index out of range");

    double support = 0.0;
    const auto addPosition = [&](std::size_t mobile, double sign) {
        for (Eigen::Index axis = 0; axis < 2; ++axis) {
            const Eigen::Index state_index =
                static_cast<Eigen::Index>(4 * mobile) + axis;
            const double coefficient = sign * direction(axis);
            support += coefficient >= 0.0
                ? coefficient * box.upper(state_index)
                : coefficient * box.lower(state_index);
        }
    };
    addPosition(first_mobile_index, 1.0);
    if (second_mobile_index.has_value())
        addPosition(*second_mobile_index, -1.0);
    return upward(support);
}

inline double branchIndependentGainNormBound(
    double covariance_norm_upper,
    double jacobian_norm_upper,
    double measurement_variance_lower) {
    if (!std::isfinite(covariance_norm_upper) ||
        covariance_norm_upper < 0.0 ||
        !std::isfinite(jacobian_norm_upper) ||
        jacobian_norm_upper < 0.0 ||
        !std::isfinite(measurement_variance_lower) ||
        measurement_variance_lower <= 0.0)
        throw std::invalid_argument("invalid branch-independent gain bound");
    return finite_tour_shadow_detail::upward(
        covariance_norm_upper * jacobian_norm_upper /
        measurement_variance_lower);
}

inline Eigen::VectorXd branchIndependentComponentGainBounds(
    const Eigen::MatrixXd& covariance_upper,
    double jacobian_norm_upper,
    double measurement_variance_lower) {
    if (covariance_upper.rows() == 0 ||
        covariance_upper.rows() != covariance_upper.cols() ||
        !covariance_upper.allFinite() ||
        !covariance_upper.isApprox(covariance_upper.transpose(), 1.0e-12) ||
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(covariance_upper)
                .eigenvalues().minCoeff() < -1.0e-12 ||
        !std::isfinite(jacobian_norm_upper) ||
        jacobian_norm_upper < 0.0 ||
        !std::isfinite(measurement_variance_lower) ||
        measurement_variance_lower <= 0.0)
        throw std::invalid_argument(
            "invalid branch-independent component gain bound");
    const double spectral_upper =
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(covariance_upper)
            .eigenvalues().maxCoeff();
    Eigen::VectorXd result(covariance_upper.rows());
    for (Eigen::Index row = 0; row < covariance_upper.rows(); ++row) {
        result(row) = finite_tour_shadow_detail::upward(
            std::sqrt(std::max(0.0, covariance_upper(row, row)) *
                      std::max(0.0, spectral_upper)) *
            jacobian_norm_upper / measurement_variance_lower);
    }
    return result;
}

}  // namespace gf
