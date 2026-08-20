#include "grand_finale/Task10p11aaGraphOracle.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <filesystem>
#include <iostream>
#include <set>

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11acOracleVerifier "
            "PACKED_CHECKPOINT OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto validation=gf::validateTask10p11sSnapshot(snapshot);
        if (!validation.complete)
            throw std::runtime_error("packed_snapshot_incomplete:"+
                                     validation.reason);
        const auto request=gf::task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto nominal_map=gf::task10p11s_capture_detail::nominalFromJson(
            snapshot.at("nominal_controls"));
        const auto nominal=gf::task10p11sOrderedControls(
            request.mobile_ids,nominal_map);
        const auto current=gf::task10p11aa_detail::fullStateAudit(
            request,nominal);
        nlohmann::json successor={{"performed",false},
            {"reason","current_full_pair_infeasible"}};
        if (current.feasible) {
            const auto controls=gf::task10p11sControlMap(
                request.mobile_ids,current.margin.controls);
            const auto successor_request=
                gf::rebuildTask10p11sSuccessorRequest(snapshot,controls);
            const auto successor_nominal=gf::task10p11sOrderedControls(
                successor_request.mobile_ids,controls);
            const auto audit=gf::task10p11aa_detail::fullStateAudit(
                successor_request,successor_nominal);
            successor={{"performed",true},{"reason","exact_zoh_h1"},
                {"feasible",audit.feasible},
                {"full_row_count",audit.problem.rows.size()},
                {"gamma_mps2",audit.recomputed_gamma},
                {"minimum_independent_full_row_residual_mps2",
                    audit.minimum_residual},
                {"limiting_row_id",audit.margin.limiting_row}};
        }
        const nlohmann::json output={{"protocol",
            "task10p11ac-independent-full-pair-successor-v1"},
            {"valid",true},{"input",argv[1]},
            {"runtime_s",snapshot.at("runtime").at("runtime_s")},
            {"current_full_pair",{{"feasible",current.feasible},
                {"full_row_count",current.problem.rows.size()},
                {"gamma_mps2",current.recomputed_gamma},
                {"minimum_independent_full_row_residual_mps2",
                    current.minimum_residual},
                {"limiting_row_id",current.margin.limiting_row}}},
            {"successor_full_pair",successor},
            {"recursive_feasibility_claimed",false}};
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ac oracle verification failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
