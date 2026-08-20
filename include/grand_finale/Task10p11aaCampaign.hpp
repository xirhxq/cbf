#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace gf {

struct Task10p11aaPreregisteredCampaign {
    std::vector<std::string> profile_order{"D20","D22","G1","G2","T1"};
    double d20_tau_mps2=20.0;
    double d22_tau_mps2=22.0;
    double graph_tau_mps2=14.0;
    std::size_t candidate_count=9;
    std::size_t g2_successor_witness_count=9;
    bool gamma_star_is_state_diagnostic=true;
    bool tau_is_feedback_threshold=true;
    bool g1_is_development_centralized_oracle=true;
    bool g2_is_finite_depth_not_recursive=true;
};

inline Task10p11aaPreregisteredCampaign
task10p11aaPreregisteredCampaign() { return {}; }

}  // namespace gf
