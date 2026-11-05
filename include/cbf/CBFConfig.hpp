#ifndef CBF_CBF_CONFIG_HPP
#define CBF_CBF_CONFIG_HPP

#include "utils.h"

struct ClassKParameters {
    double coefficient;
    int power;
};

inline ClassKParameters readClassKParameters(
    const json &config,
    double defaultCoefficient,
    int defaultPower
) {
    const json alpha = config.value("alpha", json::object());
    return {
        alpha.value("coe", defaultCoefficient),
        alpha.value("pow", defaultPower)
    };
}

#endif
