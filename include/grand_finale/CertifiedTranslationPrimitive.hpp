#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace gf {

struct Interval {
    double lower = 0.0;
    double upper = 0.0;
};

struct Interval2D {
    Interval x;
    Interval y;
};

struct SecondOrderBox2D {
    Interval2D position;
    Interval2D velocity;
};

struct TranslationPhase {
    Interval2D acceleration;
};

struct TranslationPrimitive {
    std::vector<TranslationPhase> phases;
    double dt_s = 0.0;
    double duration_s = 0.0;
    Eigen::Vector2d displacement = Eigen::Vector2d::Zero();
};

namespace certified_translation_detail {

inline Interval outward(double lower, double upper) {
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
        throw std::invalid_argument("invalid interval");
    return {
        std::nextafter(lower, -std::numeric_limits<double>::infinity()),
        std::nextafter(upper, std::numeric_limits<double>::infinity())};
}

inline Interval add(const Interval& lhs, const Interval& rhs) {
    return outward(lhs.lower + rhs.lower, lhs.upper + rhs.upper);
}

inline Interval scale(const Interval& value, double scalar) {
    if (!std::isfinite(scalar))
        throw std::invalid_argument("interval scale must be finite");
    const double first = value.lower * scalar;
    const double second = value.upper * scalar;
    return outward(std::min(first, second), std::max(first, second));
}

inline Interval inflate(const Interval& value, double radius) {
    if (!std::isfinite(radius) || radius < 0.0)
        throw std::invalid_argument("interval inflation must be non-negative");
    return outward(value.lower - radius, value.upper + radius);
}

inline bool valid(const Interval& value) {
    return std::isfinite(value.lower) && std::isfinite(value.upper) &&
        value.lower <= value.upper;
}

}  // namespace certified_translation_detail

inline SecondOrderBox2D propagateExactZoh(
    const SecondOrderBox2D& state,
    const Interval2D& commanded_acceleration,
    double dt_s,
    double control_error_bound) {
    using namespace certified_translation_detail;
    if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
        !std::isfinite(control_error_bound) || control_error_bound < 0.0 ||
        !valid(state.position.x) || !valid(state.position.y) ||
        !valid(state.velocity.x) || !valid(state.velocity.y) ||
        !valid(commanded_acceleration.x) ||
        !valid(commanded_acceleration.y)) {
        throw std::invalid_argument("invalid exact-ZOH enclosure input");
    }
    const Interval ax = inflate(commanded_acceleration.x, control_error_bound);
    const Interval ay = inflate(commanded_acceleration.y, control_error_bound);
    const auto axis = [&](const Interval& position, const Interval& velocity,
                          const Interval& acceleration) {
        const Interval next_position = add(
            add(position, scale(velocity, dt_s)),
            scale(acceleration, 0.5 * dt_s * dt_s));
        const Interval next_velocity = add(
            velocity, scale(acceleration, dt_s));
        return std::pair<Interval, Interval>{next_position, next_velocity};
    };
    const auto x = axis(state.position.x, state.velocity.x, ax);
    const auto y = axis(state.position.y, state.velocity.y, ay);
    return {{x.first, y.first}, {x.second, y.second}};
}

inline std::optional<TranslationPrimitive> makeRestToRestTranslation(
    Eigen::Vector2d direction,
    double acceleration,
    double dt_s,
    std::size_t half_steps) {
    if (!direction.allFinite() || direction.norm() <= 0.0 ||
        !std::isfinite(acceleration) || acceleration <= 0.0 ||
        !std::isfinite(dt_s) || dt_s <= 0.0 || half_steps == 0)
        return std::nullopt;
    direction.normalize();
    const Eigen::Vector2d command = acceleration * direction;
    TranslationPrimitive result;
    result.dt_s = dt_s;
    result.duration_s = 2.0 * static_cast<double>(half_steps) * dt_s;
    result.displacement =
        acceleration * std::pow(static_cast<double>(half_steps) * dt_s, 2) *
        direction;
    const auto phase = [](const Eigen::Vector2d& value) {
        return TranslationPhase{{
            {value.x(), value.x()}, {value.y(), value.y()}}};
    };
    result.phases.insert(result.phases.end(), half_steps, phase(command));
    result.phases.insert(result.phases.end(), half_steps, phase(-command));
    return result;
}

inline TranslationPrimitive reverseTranslation(
    const TranslationPrimitive& primitive) {
    TranslationPrimitive reverse = primitive;
    reverse.displacement = -primitive.displacement;
    reverse.phases.clear();
    for (const auto& phase : primitive.phases) {
        reverse.phases.push_back(TranslationPhase{{
            {-phase.acceleration.x.upper, -phase.acceleration.x.lower},
            {-phase.acceleration.y.upper, -phase.acceleration.y.lower}}});
    }
    return reverse;
}

}  // namespace gf
