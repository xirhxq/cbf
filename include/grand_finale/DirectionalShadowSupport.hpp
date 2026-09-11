#pragma once

#include "grand_finale/FiniteTourShadowEnvelope.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gf {

struct DirectionalScalarSlotBound {
    double jacobian_norm_upper = 0.0;
    double measurement_variance_lower = 0.0;
    double accepted_innovation_bound = 0.0;
};

struct DirectionalShadowStep {
    Eigen::MatrixXd transition;
    Eigen::MatrixXd disturbance_input;
    Eigen::VectorXd absolute_disturbance_bound;
    Eigen::MatrixXd covariance_upper_after_prediction;
    std::vector<DirectionalScalarSlotBound> scalar_slots;
};

enum class MobileStatePlane { Position, Velocity };

namespace directional_shadow_detail {

inline double upward(double value) {
    return std::nextafter(value, std::numeric_limits<double>::infinity());
}

inline void requireFiniteVector(
    const Eigen::VectorXd& vector,
    Eigen::Index expected,
    const char* message) {
    if (vector.size() != expected || !vector.allFinite())
        throw std::invalid_argument(message);
}

inline double boxSupport(
    const ShadowStateBox& box,
    const Eigen::VectorXd& direction) {
    finite_tour_shadow_detail::requireBox(box);
    requireFiniteVector(
        direction, box.lower.size(), "shadow direction dimension mismatch");
    double result = 0.0;
    for (Eigen::Index index = 0; index < direction.size(); ++index) {
        result += direction(index) >= 0.0
            ? direction(index) * box.upper(index)
            : direction(index) * box.lower(index);
    }
    return upward(result);
}

inline void requireStep(
    const DirectionalShadowStep& step,
    Eigen::Index dimension) {
    if (step.transition.rows() != dimension ||
        step.transition.cols() != dimension ||
        step.disturbance_input.rows() != dimension ||
        step.disturbance_input.cols() !=
            step.absolute_disturbance_bound.size() ||
        step.covariance_upper_after_prediction.rows() != dimension ||
        step.covariance_upper_after_prediction.cols() != dimension ||
        !step.transition.allFinite() ||
        !step.disturbance_input.allFinite() ||
        !step.absolute_disturbance_bound.allFinite() ||
        (step.absolute_disturbance_bound.array() < 0.0).any() ||
        !step.covariance_upper_after_prediction.allFinite() ||
        !step.covariance_upper_after_prediction.isApprox(
            step.covariance_upper_after_prediction.transpose(), 1.0e-12)) {
        throw std::invalid_argument("invalid directional shadow step");
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(
        step.covariance_upper_after_prediction);
    if (eigen.info() != Eigen::Success ||
        eigen.eigenvalues().minCoeff() < -1.0e-12) {
        throw std::invalid_argument(
            "directional covariance upper bound must be PSD");
    }
    for (const auto& slot : step.scalar_slots) {
        if (!std::isfinite(slot.jacobian_norm_upper) ||
            slot.jacobian_norm_upper < 0.0 ||
            !std::isfinite(slot.measurement_variance_lower) ||
            slot.measurement_variance_lower <= 0.0 ||
            !std::isfinite(slot.accepted_innovation_bound) ||
            slot.accepted_innovation_bound < 0.0) {
            throw std::invalid_argument(
                "invalid directional scalar slot bound");
        }
    }
}

}  // namespace directional_shadow_detail

inline double sharedScalarCorrectionSupport(
    const Eigen::VectorXd& direction,
    const Eigen::VectorXd& gain,
    double accepted_innovation_bound) {
    if (direction.size() == 0 || direction.size() != gain.size() ||
        !direction.allFinite() || !gain.allFinite() ||
        !std::isfinite(accepted_innovation_bound) ||
        accepted_innovation_bound < 0.0) {
        throw std::invalid_argument("invalid shared scalar correction");
    }
    return directional_shadow_detail::upward(
        std::abs(direction.dot(gain)) * accepted_innovation_bound);
}

inline double axisBoxCorrectionSupport(
    const Eigen::VectorXd& direction,
    const Eigen::VectorXd& absolute_gain_bound,
    double accepted_innovation_bound) {
    if (direction.size() == 0 ||
        direction.size() != absolute_gain_bound.size() ||
        !direction.allFinite() || !absolute_gain_bound.allFinite() ||
        (absolute_gain_bound.array() < 0.0).any() ||
        !std::isfinite(accepted_innovation_bound) ||
        accepted_innovation_bound < 0.0) {
        throw std::invalid_argument("invalid axis box correction");
    }
    return directional_shadow_detail::upward(
        direction.cwiseAbs().dot(absolute_gain_bound) *
        accepted_innovation_bound);
}

inline double finiteHorizonDirectionalSupport(
    const ShadowStateBox& initial,
    const std::vector<DirectionalShadowStep>& steps,
    const Eigen::VectorXd& final_direction) {
    using namespace directional_shadow_detail;
    finite_tour_shadow_detail::requireBox(initial);
    requireFiniteVector(
        final_direction, initial.lower.size(),
        "shadow direction dimension mismatch");
    Eigen::VectorXd direction = final_direction;
    double additive_support = 0.0;
    for (auto iterator = steps.rbegin(); iterator != steps.rend(); ++iterator) {
        const DirectionalShadowStep& step = *iterator;
        requireStep(step, initial.lower.size());

        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(
            step.covariance_upper_after_prediction);
        const double spectral_upper = std::max(
            0.0, eigen.eigenvalues().maxCoeff());
        const double directional_variance = std::max(
            0.0,
            (direction.transpose() *
             step.covariance_upper_after_prediction * direction)(0, 0));
        for (const auto& slot : step.scalar_slots) {
            const double gain_support =
                std::sqrt(directional_variance * spectral_upper) *
                slot.jacobian_norm_upper /
                slot.measurement_variance_lower;
            additive_support = upward(
                additive_support +
                gain_support * slot.accepted_innovation_bound);
        }

        const Eigen::VectorXd disturbance_direction =
            step.disturbance_input.transpose() * direction;
        additive_support = upward(
            additive_support +
            disturbance_direction.cwiseAbs().dot(
                step.absolute_disturbance_bound));
        direction = step.transition.transpose() * direction;
    }
    return upward(boxSupport(initial, direction) + additive_support);
}

inline Eigen::VectorXd mobileStateDirection(
    std::size_t mobile_count,
    std::size_t mobile_index,
    MobileStatePlane plane,
    const Eigen::Vector2d& direction) {
    if (mobile_count == 0 || mobile_index >= mobile_count ||
        !direction.allFinite()) {
        throw std::invalid_argument("invalid mobile state direction");
    }
    Eigen::VectorXd result = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(4 * mobile_count));
    const Eigen::Index offset = static_cast<Eigen::Index>(4 * mobile_index) +
        (plane == MobileStatePlane::Position ? 0 : 2);
    result.segment<2>(offset) = direction;
    return result;
}

inline Eigen::VectorXd relativeMobileStateDirection(
    std::size_t mobile_count,
    std::size_t first_mobile_index,
    std::size_t second_mobile_index,
    MobileStatePlane plane,
    const Eigen::Vector2d& direction) {
    if (first_mobile_index == second_mobile_index)
        throw std::invalid_argument("relative direction endpoints must differ");
    Eigen::VectorXd result = mobileStateDirection(
        mobile_count, first_mobile_index, plane, direction);
    result -= mobileStateDirection(
        mobile_count, second_mobile_index, plane, direction);
    return result;
}

}  // namespace gf

