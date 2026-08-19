#include "grand_finale/Task10p11yEvidence.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

using json=nlohmann::json;

bool equivalent(const json& left,const json& right,double tolerance=1.0e-10) {
    if (left.type()!=right.type()) {
        if (left.is_number() && right.is_number())
            return std::abs(left.get<double>()-right.get<double>())<=tolerance;
        return false;
    }
    if (left.is_number_float())
        return std::abs(left.get<double>()-right.get<double>())<=tolerance;
    if (left.is_array()) {
        if (left.size()!=right.size()) return false;
        for (std::size_t index=0;index<left.size();++index)
            if (!equivalent(left.at(index),right.at(index),tolerance))
                return false;
        return true;
    }
    if (left.is_object()) {
        if (left.size()!=right.size()) return false;
        for (auto item=left.begin();item!=left.end();++item)
            if (!right.contains(item.key()) ||
                !equivalent(item.value(),right.at(item.key()),tolerance))
                return false;
        return true;
    }
    return left==right;
}

bool metricEquals(const json& metric,double expected,double tolerance=1.0e-10) {
    return metric.at("status")=="valid" &&
        std::abs(metric.at("value").get<double>()-expected)<=tolerance;
}

json verify(const json& result,const json& historic,
    const std::filesystem::path& produced_directory,
    const std::filesystem::path& historic_directory) {
    std::vector<std::string> failures;
    const auto require=[&](bool condition,const std::string& reason) {
        if (!condition) failures.push_back(reason);
    };
    require(result.at("protocol")==
        "task10p11y-early-entry-paired-checkpoint-v1","protocol");
    require(result.at("branch")=="C0","branch");
    require(metricEquals(result.at("first_intervention_time_s"),
        historic.at("first_intervention_time_s").get<double>()),
        "first_intervention_time");
    require(result.at("pair_cycles").get<std::size_t>()==
        historic.at("responsibility_cycles").get<std::size_t>(),
        "pair_cycles");
    require(std::abs(result.at("final_time_s").get<double>()-
        historic.at("simulated_time_s").get<double>())<=1.0e-10,
        "final_time");
    require(result.at("final_covered_cells").get<int>()==
        historic.at("covered_cells").get<int>(),"covered_cells");
    require(std::abs(result.at("final_coverage").get<double>()-
        historic.at("truth_coverage").get<double>())<=1.0e-12,
        "coverage");
    require(result.at("stop_reason").get<std::string>().find(
        "dynamic_pair_responsibility_interval_empty")!=std::string::npos,
        "stop_reason");
    require(result.at("checkpoint_count").get<std::size_t>()==6,
        "checkpoint_count");

    std::vector<const json*> produced_dynamic;
    for (const auto& entry:result.at("trace"))
        if (entry.at("advanced").get<bool>() &&
            entry.at("dynamic_pair_used").get<bool>())
            produced_dynamic.push_back(&entry);
    const auto& historic_trace=historic.at("responsibility_trace");
    require(produced_dynamic.size()==historic_trace.size(),
        "responsibility_trace_size");
    if (produced_dynamic.size()==historic_trace.size()) {
        for (std::size_t index=0;index<produced_dynamic.size();++index) {
            const auto& produced=*produced_dynamic[index];
            const auto& expected=historic_trace.at(index);
            require(std::abs(produced.at("decision_time_s").get<double>()-
                expected.at("decision_time_s").get<double>())<=1.0e-10,
                "decision_time_"+std::to_string(index));
            require(metricEquals(produced.at("selected_transfer_mps2"),
                expected.at("selected_transfer_mps2").get<double>(),1.0e-8),
                "selected_transfer_"+std::to_string(index));
            require(metricEquals(produced.at("coverage_after"),
                expected.at("truth_coverage").get<double>(),1.0e-12),
                "coverage_after_"+std::to_string(index));
        }
    }

    const std::vector<std::string> produced_names={
        "checkpoint-000-t132.4-dynamic_signed_transfer.json",
        "checkpoint-001-t132.5-dynamic_signed_transfer.json",
        "checkpoint-002-t132.6-dynamic_signed_transfer.json",
        "checkpoint-003-t132.7-dynamic_signed_transfer.json",
        "checkpoint-004-t132.8-dynamic_signed_transfer.json",
        "checkpoint-005-t132.9-fail_closed.json"};
    const std::vector<std::string> historic_names={
        "checkpoint-000-t132.4-first_dynamic_intervention.json",
        "checkpoint-001-t132.5-dynamic_intervention.json",
        "checkpoint-002-t132.6-dynamic_intervention.json",
        "checkpoint-003-t132.7-dynamic_intervention.json",
        "checkpoint-004-t132.8-dynamic_intervention.json",
        "checkpoint-005-t132.9-fail_closed.json"};
    const std::vector<std::string> offline_fields={"actual_rows",
        "canonical_request","dekf_internal","estimator","nominal_controls",
        "objective_28d","owner_row_counts","runtime","successor_parameters"};
    const std::vector<std::string> restart_fields={"plant","coverage",
        "controller","adapter","estimator_dekf","topology","exact_zoh",
        "runtime_s"};
    for (std::size_t index=0;index<produced_names.size();++index) {
        const auto produced=gf::readTask10p11vJson(
            produced_directory/produced_names[index]);
        const auto expected=gf::readTask10p11vJson(
            historic_directory/historic_names[index]);
        for (const auto& field:offline_fields)
            require(equivalent(produced.at(field),expected.at(field)),
                "offline_"+std::to_string(index)+"_"+field);
        for (const auto& field:restart_fields)
            require(equivalent(produced.at("restart_checkpoint").at(field),
                expected.at("restart_checkpoint").at(field)),
                "restart_"+std::to_string(index)+"_"+field);
    }
    return {{"protocol","task10p11y-c0-suffix-verification-v1"},
        {"valid",failures.empty()},
        {"reason",failures.empty()?"c0_suffix_reproduced":
            "c0_suffix_reproducibility_failed"},
        {"failures",failures},{"historic_event_count",6},
        {"scientific_fields_compared",offline_fields.size()+
            restart_fields.size()},
        {"tolerance",{{"state_and_snapshot",1.0e-10},
            {"selected_transfer",1.0e-8},{"coverage",1.0e-12}}},
        {"t1_authorized_by_this_gate",failures.empty()}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=6) {
        std::cerr<<"usage: GrandFinaleTask10p11yVerifyC0 C0_RESULT "
            "HISTORIC_RESULT C0_CHECKPOINT_DIR HISTORIC_CHECKPOINT_DIR "
            "OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto result=verify(gf::readTask10p11vJson(argv[1]),
            gf::readTask10p11vJson(argv[2]),argv[3],argv[4]);
        gf::writeTask10p11vJson(argv[5],result);
        std::cout<<result.dump(2)<<'\n';
        return result.at("valid").get<bool>()?0:3;
    } catch (const std::exception& error) {
        std::cerr<<"C0 verification failed: "<<error.what()<<'\n';
        return 4;
    }
}
