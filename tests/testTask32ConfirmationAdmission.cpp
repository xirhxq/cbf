#include "grand_finale/Task32ConfirmationAdmission.hpp"
#include "grand_finale/Task32ExperimentAdmission.hpp"
#include <array>
#include <iostream>
#include <limits>

int main() {
    // Public startup seam only: these calls neither sample noise nor construct plant.
    const std::array<std::array<std::uint64_t,2>,3> keys{{
        {{141101,142101}},{{141119,142119}},{{141137,142137}}}};
    unsigned assertions=0;
    for (const auto& pair:keys) {
        if (!gf::task32RegisteredTargetConfirmation(2,.5,pair[0],pair[1])) return 1;
        ++assertions;
        if (gf::task32RegisteredTargetExperiment(.5,pair[0],pair[1])) return 2;
        ++assertions; // Original development registry remains frozen.
        for (int mechanism : {0,1,3,-1}) {
            if (gf::task32RegisteredTargetConfirmation(mechanism,.5,pair[0],pair[1])) return 3;
            ++assertions;
        }
        for (const auto& other:keys) if (pair!=other) {
            if (gf::task32RegisteredTargetConfirmation(2,.5,pair[0],other[1])) return 4;
            ++assertions;
        }
        for (double sigma : {0.,-.5,.6,std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
            if (gf::task32RegisteredTargetConfirmation(2,sigma,pair[0],pair[1])) return 5;
            ++assertions;
        }
    }
    if (gf::task32RegisteredTargetConfirmation(2,.5,139011,140011) ||
        gf::task32RegisteredTargetConfirmation(2,.5,141102,142101)) return 6;
    assertions+=2;
    std::cout << assertions << " confirmation admission assertions passed; 0 initialization; 0 plant\n";
}
