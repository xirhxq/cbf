#include "grand_finale/Task32ExperimentAdmission.hpp"
#include <iostream>
#include <array>
#include <limits>

int main() {
    if (!gf::task32RegisteredTargetExperiment(0.5, 137029, 138029)) return 1;
    if (!gf::task32RegisteredTargetExperiment(0.0, 2027, 134001)) {
        std::cerr << "registered zero-Gaussian identity was rejected\n";
        return 2;
    }
    const std::array<std::array<std::uint64_t,2>,3> development{{
        {{139011,140011}},{{139029,140029}},{{139047,140047}}}};
    for (const auto& keys : development) {
        if (!gf::task32RegisteredTargetExperiment(0.5,keys[0],keys[1])) {
            std::cerr << "registered development identity was rejected\n";
            return 3;
        }
        if (gf::task32RegisteredTargetExperiment(0.0,keys[0],keys[1]) ||
            gf::task32RegisteredTargetExperiment(0.6,keys[0],keys[1]) ||
            gf::task32RegisteredTargetExperiment(0.5,keys[0],keys[1]+1)) return 4;
    }
    for (const auto& keys : std::array<std::array<std::uint64_t,2>,3>{{
        {{141101,142101}},{{141119,142119}},{{141137,142137}}}})
        if (gf::task32RegisteredTargetExperiment(0.5,keys[0],keys[1])) return 5;
    for (double sigma : { -0.5, 0.6, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN() })
        if (gf::task32RegisteredTargetExperiment(sigma,137029,138029)) return 6;
    if (gf::task32RegisteredTargetExperiment(0.5,137030,138029) ||
        gf::task32RegisteredTargetExperiment(0.5,137029,138030) ||
        gf::task32RegisteredTargetExperiment(0.5,2027,134001)) return 7;
    std::cout << "24 public launch-admission assertions passed; 0 plant\n";
}
