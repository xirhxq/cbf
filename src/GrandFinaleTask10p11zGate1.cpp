#include "grand_finale/Task10p11zComponentOracle.hpp"
#include "grand_finale/Task10p11zTopologyOracle.hpp"

#include <filesystem>
#include <iostream>

int main(int argc,char** argv) {
    if (argc!=5) {
        std::cerr<<"usage: GrandFinaleTask10p11zGate1 SNAPSHOT_133 "
            "SNAPSHOT_132P8 SPARSE_130 OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto component=gf::runTask10p11zComponentGate(argv[1],13);
        const auto topology=gf::runTask10p11zEarlyTopologyGate(argv[3]);
        const auto gain_snapshot=gf::readTask10p11vJson(argv[2]);
        const auto gain=gf::task10p11x_detail::parameterFrame(
            gain_snapshot,14.0,gf::LinearHocbfGains{0.125,0.5});
        const nlohmann::json output={{"protocol","task10p11z-gate1-v1"},
            {"component",gf::task10p11zComponentGateJson(component)},
            {"dynamic_topology",gf::task10p11zTopologyGateJson(topology)},
            {"hocbf_gain_sensitivity",{{"trajectory_run",false},
                {"snapshot_time_s",132.8},
                {"historical_candidate",{{"lambda1_per_s",0.125},
                    {"lambda2_per_s",0.5}}},
                {"snapshot_result",gain},
                {"theoretical_domain","lambda1>0_and_lambda2>0"},
                {"changes_hocbf_rows_and_initial_extension_set",true},
                {"not_a_selector_threshold",true},
                {"profile_in_this_campaign",false}}},
            {"gamma_star_role","diagnostic_not_parameter"},
            {"trajectory_run_performed",false},
            {"claim_boundary",{{"snapshot_and_one_step_only",true},
                {"recursive_feasibility_claimed",false},
                {"task_11_entered",false}}}};
        gf::writeTask10p11vJson(argv[4],output);
        std::cout<<output.dump(2)<<'\n';
        return component.valid?0:3;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11z Gate 1 failed: "<<error.what()<<'\n';
        return 4;
    }
}
