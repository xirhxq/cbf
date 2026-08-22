#include "grand_finale/Task10p11agFullDomainPredecessor.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3) return 2;
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto request0=gf::task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto u0_map=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
        const auto u0=gf::task10p11sOrderedControls(request0.mobile_ids,u0_map);
        const auto problem0=gf::buildTask10p11sRows28d(
            gf::buildCanonicalHardRows(request0),request0.mobile_ids,true);
        const auto residual0=gf::task10p11w_detail::minimumResidual(problem0,u0);
        const auto estimate0=gf::task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        const auto x1=gf::task10p11aa_detail::predictEstimate(
            snapshot,estimate0,u0_map);
        const auto request1=gf::task10p11x_detail::requestAtEstimate(snapshot,x1);
        const auto problem1=gf::buildTask10p11sRows28d(
            gf::buildCanonicalHardRows(request1),request1.mobile_ids,true);
        std::set<gf::NodeId> all(request1.mobile_ids.begin(),request1.mobile_ids.end());
        const auto maximum1=gf::task10p11w_detail::solveRestricted(
            problem1,u0,u0,all,true);
        if (!maximum1.feasible || maximum1.controls.size()!=u0.size())
            throw std::runtime_error("x1 full-pair maximum-margin solve failed");
        const auto u1=maximum1.controls;
        const auto residual1=gf::task10p11w_detail::minimumResidual(problem1,u1);
        const auto gamma1=gf::task10p11aa_detail::gammaResidual(problem1,u1);
        const auto u1_map=gf::task10p11sControlMap(request1.mobile_ids,u1);
        const auto x2=gf::task10p11aa_detail::predictEstimate(snapshot,x1,u1_map);
        const auto request2=gf::task10p11x_detail::requestAtEstimate(snapshot,x2);
        const auto problem2=gf::buildTask10p11sRows28d(
            gf::buildCanonicalHardRows(request2),request2.mobile_ids,true);
        const auto maximum2=gf::task10p11w_detail::solveRestricted(
            problem2,u1,u1,all,true);
        if (!maximum2.feasible || maximum2.controls.size()!=u1.size())
            throw std::runtime_error("x2 full-pair maximum-margin solve failed");
        const auto u2=maximum2.controls;
        const auto residual2=gf::task10p11w_detail::minimumResidual(problem2,u2);
        const auto gamma2=gf::task10p11aa_detail::gammaResidual(problem2,u2);
        constexpr double tolerance=1.0e-8;
        const bool witness=residual0.first>=-tolerance &&
            residual1.first>=-tolerance && gamma1>=-tolerance &&
            residual2.first>=-tolerance && gamma2>=-tolerance;
        const nlohmann::json output={
            {"protocol","task10p11ag-full-domain-h2-witness-v1"},
            {"valid",true},{"witness_found",witness},
            {"classification",witness?"feasible_witness":"no_witness_from_frozen_construction"},
            {"construction","distributed_U0_then_full_pair_maximum_margin_U1_and_U2"},
            {"U0",gf::task10p11ag_detail::controlsJson(request0.mobile_ids,u0)},
            {"x1",gf::task10p11s_capture_detail::estimateJson(x1)},
            {"U1",gf::task10p11ag_detail::controlsJson(request1.mobile_ids,u1)},
            {"x2",gf::task10p11s_capture_detail::estimateJson(x2)},
            {"U2",gf::task10p11ag_detail::controlsJson(request2.mobile_ids,u2)},
            {"current",{{"row_count",problem0.rows.size()},
                {"minimum_residual_mps2",residual0.first},
                {"limiting_row_id",residual0.second},
                {"all_1113_residuals",gf::task10p11ag_detail::residualsJson(problem0,u0)}}},
            {"x1_full_pair",{{"row_count",problem1.rows.size()},
                {"global_gamma_mps2",gamma1},{"minimum_residual_mps2",residual1.first},
                {"limiting_row_id",residual1.second},
                {"all_1113_residuals",gf::task10p11ag_detail::residualsJson(problem1,u1)}}},
            {"x2_full_pair",{{"row_count",problem2.rows.size()},
                {"global_gamma_mps2",gamma2},{"minimum_residual_mps2",residual2.first},
                {"limiting_row_id",residual2.second},
                {"all_1113_residuals",gf::task10p11ag_detail::residualsJson(problem2,u2)}}},
            {"claim_boundary",{{"positive_residuals_prove_H2_existence",true},
                {"negative_result_would_prove_full_domain_infeasibility",false},
                {"recursive_feasibility_claimed",false},{"mpc_claimed",false}}}
        };
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return witness?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag H2 witness failed: "<<error.what()<<'\n';
        return 4;
    }
}
