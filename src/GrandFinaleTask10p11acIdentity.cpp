#include "grand_finale/Task10p11acCampaign.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=2) {
        std::cerr<<"usage: GrandFinaleTask10p11acIdentity OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto tau20=gf::makeTask10p11rFixedBaselineFixture(
            gf::GammaFeedbackSelectionMode::LeastIntervention,20.0);
        const auto tau22=gf::makeTask10p11rFixedBaselineFixture(
            gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
        const auto base20=gf::task10p11acBaseIdentityJson(*tau20);
        const auto base22=gf::task10p11acBaseIdentityJson(*tau22);
        if (base20!=base22)
            throw std::runtime_error(
                "tau20_tau22_base_identity_mismatch");
        const nlohmann::json result={{"protocol",
            "task10p11ac-identity-v1"},{"valid",true},
            {"base_config_without_tau",base20},
            {"hard_gates",base20.at("hard_gates")},
            {"explicit_tau_values_mps2",{20.0,22.0}},
            {"only_explicit_profile_difference","predictive_gamma_tau_mps2"}};
        gf::writeTask10p11vJson(argv[1],result);
        std::cout<<result.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ac identity failed: "<<error.what()<<'\n';
        return 3;
    }
}
