#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gf {
// Temporary reconstruction only.  The r4 geometry already smooths each DAG
// layer; this opt-in ablation removes only the outer time smoothstep.  The
// reference positions remain continuous.  Its common-centroid velocity has
// one-sided jumps at the clamped endpoints; no C1 time claim is made.
inline double task29LinearExpansionPhase(double elapsed_s) {
    if(!std::isfinite(elapsed_s)) throw std::invalid_argument("nonfinite reconstruction elapsed time");
    return std::clamp(elapsed_s/60.0,0.0,1.0);
}
} // namespace gf
