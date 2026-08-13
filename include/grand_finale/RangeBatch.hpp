#pragma once

#include "grand_finale/Types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace gf {

struct RangeMeasurement {
    std::int64_t timestamp_ns;
    UndirectedEdge edge;
    double range_m;
    double variance_m2;
};

inline std::vector<RangeMeasurement> canonicalizeRangeBatch(
    std::vector<RangeMeasurement> measurements) {
    for (const RangeMeasurement& measurement : measurements) {
        if (!std::isfinite(measurement.range_m) || measurement.range_m < 0.0) {
            throw std::invalid_argument(
                "range_m must be finite and non-negative");
        }
        if (!std::isfinite(measurement.variance_m2) ||
            measurement.variance_m2 <= 0.0) {
            throw std::invalid_argument(
                "variance_m2 must be positive and finite");
        }
    }

    std::sort(
        measurements.begin(), measurements.end(),
        [](const RangeMeasurement& lhs, const RangeMeasurement& rhs) {
            return std::tie(lhs.timestamp_ns, lhs.edge.first, lhs.edge.second) <
                   std::tie(rhs.timestamp_ns, rhs.edge.first, rhs.edge.second);
        });

    for (std::size_t index = 1; index < measurements.size(); ++index) {
        const RangeMeasurement& previous = measurements[index - 1];
        const RangeMeasurement& current = measurements[index];
        if (previous.timestamp_ns == current.timestamp_ns &&
            previous.edge.first == current.edge.first &&
            previous.edge.second == current.edge.second) {
            throw std::invalid_argument(
                "duplicate range sample " +
                std::to_string(current.timestamp_ns) + ":" +
                current.edge.id());
        }
    }

    return measurements;
}

}  // namespace gf
