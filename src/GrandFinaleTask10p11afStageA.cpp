#include "grand_finale/Task10p11afSuccessorRecovery.hpp"

#include <filesystem>
#include <iostream>
#include <optional>

namespace {

using json=nlohmann::json;

json metric(const std::optional<double>& value,const std::string& reason) {
    return value.has_value()?json{{"status","valid"},{"value",*value},
        {"reason","observed_saved_cycle"}}:
        json{{"status","not_applicable"},{"value",nullptr},{"reason",reason}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=4) {
        std::cerr<<"usage: GrandFinaleTask10p11afStageA "
            "P3_RESULT_JSON P3_CHECKPOINT_DIR OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto result=gf::readTask10p11vJson(argv[1]);
        const std::filesystem::path directory=argv[2];
        if (result.at("profile")!="P3-tau22" ||
            result.at("valid")!=true ||
            result.at("advanced_cycles")!=1581)
            throw std::runtime_error("P3 Task10p11ae identity mismatch");
        std::optional<double> first_successor_infeasible;
        std::optional<double> first_current_infeasible;
        for (const auto& frame:result.at("margin_trace")) {
            const double time=frame.at("time_s").get<double>();
            if (!first_successor_infeasible.has_value() &&
                !frame.at("successor_full_pair_global_gamma_mps2").is_null() &&
                frame.at("successor_full_pair_global_gamma_mps2").
                    get<double>()< -1.0e-8)
                first_successor_infeasible=time;
            if (!first_current_infeasible.has_value() &&
                !frame.at("full_pair_global_gamma_mps2").is_null() &&
                frame.at("full_pair_global_gamma_mps2").get<double>()<
                    -1.0e-8)
                first_current_infeasible=time;
        }
        static const std::vector<std::string> packed{
            "checkpoint-0006-t132.4-frozen_time.json",
            "checkpoint-0007-t147.2-frozen_time.json",
            "checkpoint-0009-t158.0-first_margin_layer_divergence.json"};
        json audits=json::array();
        std::optional<double> last_g1_recoverable;
        std::optional<double> last_g2_recoverable;
        for (const auto& name:packed) {
            const auto audit=gf::task10p11afAuditPackedCheckpoint(
                directory/name);
            if (!audit.valid || audit.current_full_row_count!=1113)
                throw std::runtime_error("packed layer audit failed:"+name+
                                         ":"+audit.reason);
            if (audit.g1.valid && audit.g1.successor_distributed_feasible==false)
                last_g1_recoverable=audit.time_s;
            if (audit.g2.valid && audit.g2.successor_distributed_feasible==false)
                last_g2_recoverable=audit.time_s;
            auto encoded=gf::task10p11afLayerAuditJson(audit);
            encoded["checkpoint"]=name;
            audits.push_back(std::move(encoded));
        }
        const auto t95_snapshot=gf::readTask10p11vJson(directory/
            "checkpoint-0008-t153.9-t95.json");
        const auto t95_full=gf::runTask10p11sFull28dGateA(t95_snapshot);
        const auto final_snapshot=gf::readTask10p11vJson(directory/
            "checkpoint-0010-t158.1-fail_closed.json");
        const auto final_full=gf::runTask10p11sFull28dGateA(final_snapshot);
        const bool final_current=final_full.at("full_pair").at("feasible").
            get<bool>() && final_full.at("full_pair").at(
                "minimum_compiled_residual_mps2").get<double>()>=-1.0e-8;
        const json output={{"protocol","task10p11af-stage-a-v1"},
            {"valid",true},{"source_p3_result",argv[1]},
            {"saved_cycle_resolution_s",0.1},
            {"packed_layer_audits",std::move(audits)},
            {"state_only_packed_audits",json::array({{
                {"checkpoint","checkpoint-0008-t153.9-t95.json"},
                {"time_s",t95_snapshot.at("runtime").at("runtime_s")},
                {"reason","post_advance_checkpoint_has_no_same_boundary_owner_decision_ledger"},
                {"current_full_pair",t95_full.at("full_pair")},
                {"successor",t95_full.at("successor")}}})},
            {"earliest_saved_successor_full_pair_infeasible_s",metric(
                first_successor_infeasible,"not_observed")},
            {"earliest_saved_current_full_pair_infeasible_s",metric(
                first_current_infeasible,"not_observed_in_advanced_cycles")},
            {"last_packed_g1_successor_recovery_witness_s",metric(
                last_g1_recoverable,"not_observed")},
            {"last_packed_g2_predecessor_witness_s",metric(
                last_g2_recoverable,"not_observed")},
            {"final_fail_closed",{
                {"time_s",final_snapshot.at("runtime").at("runtime_s")},
                {"current_full_pair_feasible",final_current},
                {"full_pair",final_full.at("full_pair")}}},
            {"interpretation",{
                {"margin_trace_is_per_advanced_cycle",true},
                {"component_and_signed_transfer_are_packed_only",true},
                {"unsaved_component_crossing_interpolated",false},
                {"full_pair_model","fixed_topology_once_reserve_28d"},
                {"independent_full_row_count",1113},
                {"recursive_feasibility_claimed",false}}}};
        gf::writeTask10p11vJson(argv[3],output);
        std::cout<<output.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11af Stage A failed: "<<error.what()<<'\n';
        return 4;
    }
}
