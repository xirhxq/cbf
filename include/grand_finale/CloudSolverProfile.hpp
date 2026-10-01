#pragma once
#include "grand_finale/Types.hpp"

namespace gf {
// Opt-in cloud research runner only. Existing builds remain Gurobi.
inline constexpr SolverProfile cloudSolverProfile() {
#ifdef GF_CLOUD_OSQP
    return SolverProfile::OpenSource;
#else
    return SolverProfile::Gurobi;
#endif
}
}
