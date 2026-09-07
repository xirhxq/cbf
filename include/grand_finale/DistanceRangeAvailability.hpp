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
} // namespace gf
