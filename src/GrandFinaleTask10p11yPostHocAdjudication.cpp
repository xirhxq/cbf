#include "grand_finale/Task10p11yEvidence.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>

namespace {

using json=nlohmann::json;

json adjudicate(const json& manifest) {
    if (manifest.at("protocol")!=
        "task10p11y-post-hoc-input-manifest-v1")
        throw std::invalid_argument("unsupported adjudication manifest");
    const auto temporary=gf::readTask10p11vJson(
        manifest.at("temporary_summary").at("path").get<std::string>());
    if (temporary.at("protocol")!=
        "fixed-baseline-multi-path-recovery-v1")
        throw std::invalid_argument("unexpected temporary summary protocol");
    json advanced=json::array();
    double minimum_current=std::numeric_limits<double>::infinity();
    double minimum_successor=std::numeric_limits<double>::infinity();
    for (const auto& entry:temporary.at("trace")) {
        if (!entry.at("advanced").get<bool>()) continue;
        advanced.push_back(entry);
        minimum_current=std::min(minimum_current,entry.at("decision")
            .at("current_full_row_residual_mps2").get<double>());
        minimum_successor=std::min(minimum_successor,entry.at("decision")
            .at("successor").at("full_pair_minimum_residual_mps2")
            .get<double>());
    }
    if (advanced.size()!=2)
        throw std::runtime_error(
            "post-hoc adjudication expected exactly two advanced cycles");
    const auto full28=gf::readTask10p11vJson(
        manifest.at("full28d_133p0").at("path").get<std::string>());
    const auto& gate=full28.at("gate_a");
    return {{"protocol","task10p11y-post-hoc-adjudication-v1"},
        {"post_hoc_adjudication",true},{"simulation_rerun",false},
        {"original_result_publication_gate_passed",false},
        {"stage_b_formally_completed",false},
        {"stage_c_ranking_eligible",false},
        {"inputs",manifest},
        {"corrected",{{"advanced_component_cycles",advanced.size()},
            {"component_duration_s",0.1*advanced.size()},
            {"final_advanced_coverage",advanced.back().at("coverage_after")},
            {"minimum_applied_current_full_row_residual_mps2",
                gf::task10p11yMetric(minimum_current,"derived_from_advanced_only")},
            {"minimum_applied_successor_full_pair_residual_mps2",
                gf::task10p11yMetric(minimum_successor,
                    "derived_from_advanced_only")},
            {"failed_133p0_control_applied",false},
            {"stop_reason","pair_2_4_component_current_infeasible"}}},
        {"superseded_temporary_fields",{{"component_cycles",
                temporary.at("component_cycles")},
            {"component_duration_s",temporary.at("component_duration_s")},
            {"null_aggregates_valid",false}}},
        {"later_read_only_133p0_oracle",{{"part_of_original_result",false},
            {"current_full_pair_28d_feasible",
                gate.at("full_pair").at("feasible")},
            {"current_compiled_residual_mps2",
                gate.at("full_pair").at("minimum_compiled_residual_mps2")},
            {"successor_full_pair_28d_feasible",
                gate.at("successor").at("feasible")},
            {"successor_compiled_residual_mps2",
                gate.at("successor").at("minimum_compiled_residual_mps2")},
            {"independent_1113_row_residual_claimed",false}}},
        {"claim_boundary",{{"temporary_summary_renamed",false},
            {"old_trajectory_rerun",false},
            {"recursive_feasibility_claimed",false}}}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11yPostHocAdjudication "
            "INPUT_MANIFEST OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto manifest=gf::readTask10p11vJson(argv[1]);
        const auto result=adjudicate(manifest);
        gf::writeTask10p11vJson(argv[2],result);
        std::cout<<result.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"post-hoc adjudication failed: "<<error.what()<<'\n';
        return 3;
    }
}
