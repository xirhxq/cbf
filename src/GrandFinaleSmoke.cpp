#include "grand_finale/GrandFinaleExperiment.hpp"

#include <iostream>

int main() {
    const auto summary = gf::GrandFinaleExperiment::runCanonical4p2(
        gf::SolverProfile::OpenSource, false, 2027);
    std::cout << "seed=2027"
              << ",certified_coverage=" << summary.certified_coverage_final
              << ",topology_version=" << summary.topology_version
              << ",mode=" << static_cast<int>(summary.final_mode)
              << ",minimum_hard_margin=" << summary.minimum_hard_margin
              << ",minimum_gamma_star=" << summary.minimum_gamma_star
              << ",hard_gate_violations=" << summary.hard_gate_violations
              << '\n';
    return summary.hard_gate_violations == 0 &&
                   summary.reached_certified_t100
        ? 0 : 1;
}
