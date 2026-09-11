#pragma once
#include <cstdint>

namespace gf {
// Launch admission only. This predicate is never consulted by the planner,
// controller, estimator, or measurement generator after startup.
inline bool task32RegisteredTargetExperiment(double sigma_m,
    std::uint64_t gaussian_key, std::uint64_t link_key) {
    if (sigma_m == 0.0) return gaussian_key == 2027 && link_key == 134001;
    if (sigma_m != 0.5) return false;
    return (gaussian_key == 137029 && link_key == 138029) ||
           (gaussian_key == 139011 && link_key == 140011) ||
           (gaussian_key == 139029 && link_key == 140029) ||
           (gaussian_key == 139047 && link_key == 140047);
}
} // namespace gf
