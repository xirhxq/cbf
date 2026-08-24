#include "grand_finale/Task10p11ahEarlyH2Recovery.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=4) return 2;
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto witness=gf::readTask10p11vJson(argv[2]);
        const auto audit=gf::auditTask10p11ahExistingH2Witness(
            snapshot,witness);
        const auto x2=gf::task10p11s_capture_detail::estimateFromJson(
            witness.at("x2"));
        const auto request2=gf::task10p11x_detail::requestAtEstimate(snapshot,x2);
        const auto u2=gf::task10p11ah_detail::controlsFromJson(
            witness.at("U2"),request2.mobile_ids);
        const double runtime=snapshot.at("runtime").at("runtime_s").get<double>()+
            2.0*snapshot.at("successor_parameters").at("dt_s").get<double>();
        const auto terminal_snapshot=gf::task10p11ah_detail::snapshotAtEstimate(
            snapshot,x2,request2,u2,runtime);
        const auto full_h1=gf::solveTask10p11agFullDomainPredecessor(
            terminal_snapshot,u2,60.0);
        const nlohmann::json output={{"protocol",
            "task10p11ah-existing-H2-terminal-audit-v1"},
            {"valid",audit.valid},{"reason",audit.reason},
            {"x0",{{"row_count",audit.x0_row_count},
                {"minimum_residual_mps2",gf::task10p11w_detail::number(
                    audit.x0_minimum_residual_mps2)}}},
            {"x1",{{"row_count",audit.x1_row_count},
                {"minimum_residual_mps2",gf::task10p11w_detail::number(
                    audit.x1_minimum_residual_mps2)}}},
            {"x2",{{"row_count",audit.x2_row_count},
                {"minimum_residual_mps2",gf::task10p11w_detail::number(
                    audit.x2_minimum_residual_mps2)},
                {"owner_local_gamma_mps2",gf::task10p11w_detail::number(
                    audit.terminal_owner_local_gamma_mps2)},
                {"signed_transfer_feasible",
                    audit.terminal_signed_transfer_feasible},
                {"H1_constructive_witness",audit.terminal_h1_witness},
                {"H1_constructive_margin_mps2",gf::task10p11w_detail::number(
                    audit.terminal_h1_margin_mps2)},
                {"H2_constructive_witness",audit.terminal_h2_witness},
                {"H2_constructive_margin_mps2",gf::task10p11w_detail::number(
                    audit.terminal_h2_margin_mps2)},
                {"full_domain_H1_classification",full_h1.classification},
                {"full_domain_H1_witness_found",full_h1.witness_found},
                {"full_domain_H1_best_successor_gamma_mps2",
                    gf::task10p11w_detail::number(
                        full_h1.best_successor_gamma_mps2)},
                {"full_domain_H1_strict_infeasibility_proven",
                    full_h1.strict_infeasibility_proven},
                {"native_distributed_terminal_audited",
                    audit.native_distributed_terminal_audited}}},
            {"terminal_classification",
                gf::task10p11ahTerminalClassificationName(
                    audit.terminal_classification)},
            {"claim_boundary",{{"existing_witness_only",true},
                {"native_tau22_terminal_requires_runtime_audit",true},
                {"negative_local_search_proves_infeasibility",false},
                {"recursive_feasibility_claimed",false}}}};
        gf::writeTask10p11vJson(argv[3],output);
        std::cout<<output.dump(2)<<'\n';
        return audit.valid?0:4;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ah terminal audit failed: "<<error.what()<<'\n';
        return 4;
    }
}
