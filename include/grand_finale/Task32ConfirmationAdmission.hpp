#pragma once
#include <cstdint>

namespace gf {
// One frozen confirmation group. Startup admission only, never a scientific
// branch in planning, estimation, measurements, or control. Production is off.
inline bool task32RegisteredTargetConfirmation(int mechanism, double sigma_m,
    std::uint64_t gaussian_key, std::uint64_t link_key) {
    if (mechanism != 2 || sigma_m != 0.5) return false;
    return (gaussian_key == 141101 && link_key == 142101) ||
           (gaussian_key == 141119 && link_key == 142119) ||
           (gaussian_key == 141137 && link_key == 142137);
}
} // namespace gf
