#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace gf {
// Research-only physical acquisition model, independent of DAG and acceptance.
// Constants are the researcher-frozen contract, not scan parameters.
struct DistanceRangeAvailability {
    bool enabled=false;
    unsigned int link_seed=2027;
    double acquisitionProbability(double distance_m) const {
        if(!std::isfinite(distance_m)||distance_m<0)
            throw std::invalid_argument("invalid physical ranging distance");
        if(!enabled||distance_m<=850.0)return 1.0;
        return std::exp(-std::log(20.0)*(distance_m-850.0)/150.0);
    }
    bool acquired(double distance_m,double uniform) const {
        if(!std::isfinite(uniform)||uniform<0||uniform>=1)
            throw std::invalid_argument("invalid acquisition uniform");
        return uniform<acquisitionProbability(distance_m);
    }
};

// CBF2026 R&R overwater UWB channel model:
//   P_avail(d) = 1 / (1 + (d / d_r)^2) for d <= d_max, else 0
//   sigma(d)   = sigma_0 * (1 + (d / d_r)^2)
struct Cbf2026UwbChannelConfig {
    bool enabled = false;
    double d_r_m = 850.0;
    double sigma0_m = 0.5;
    double d_max_m = 1200.0;
    unsigned int link_seed = 2027;

    void validate() const {
        if (!std::isfinite(d_r_m) || d_r_m <= 0.0 ||
            !std::isfinite(sigma0_m) || sigma0_m <= 0.0 ||
            !std::isfinite(d_max_m) || d_max_m < 0.0) {
            throw std::invalid_argument("invalid CBF2026 UWB channel configuration");
        }
    }

    double acquisitionProbability(double distance_m) const {
        if (!std::isfinite(distance_m) || distance_m < 0.0) {
            throw std::invalid_argument("invalid physical ranging distance");
        }
        validate();
        if (!enabled) return 1.0;
        if (distance_m > d_max_m) return 0.0;
        const double ratio = distance_m / d_r_m;
        return 1.0 / (1.0 + ratio * ratio);
    }

    double rangingStdM(double distance_m) const {
        if (!std::isfinite(distance_m) || distance_m < 0.0) {
            throw std::invalid_argument("invalid physical ranging distance");
        }
        validate();
        const double ratio = distance_m / d_r_m;
        return sigma0_m * (1.0 + ratio * ratio);
    }

    double expectedFisherWeight(double distance_m) const {
        const double p = acquisitionProbability(distance_m);
        if (p <= 0.0) return 0.0;
        const double std_m = rangingStdM(distance_m);
        return p / (std_m * std_m);
    }

    bool acquired(double distance_m, double uniform) const {
        if (!std::isfinite(uniform) || uniform < 0.0 || uniform >= 1.0) {
            throw std::invalid_argument("invalid acquisition uniform");
        }
        return uniform < acquisitionProbability(distance_m);
    }
};
} // namespace gf
