#include "grand_finale/Task10p11ahEarlyH2Recovery.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3) return 2;
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto request0=gf::task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto distributed_map=
            gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
        const auto distributed=gf::task10p11sOrderedControls(
            request0.mobile_ids,distributed_map);
        const auto attempt=gf::task10p11ag_component_detail::evaluateComponent(
            snapshot,distributed,{2,9});
        if (!attempt.current_feasible || !attempt.successor_feasible)
            throw std::runtime_error("component H2 prefix unavailable");
        const auto problem0=gf::buildTask10p11sRows28d(
            gf::buildCanonicalHardRows(request0),request0.mobile_ids,true);
        const auto u0=attempt.current.controls;
        const auto u0_map=gf::task10p11sControlMap(request0.mobile_ids,u0);
        const auto x0=gf::task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        const auto x1=gf::task10p11aa_detail::predictEstimate(snapshot,x0,u0_map);
        const auto request1=gf::task10p11x_detail::requestAtEstimate(snapshot,x1);
        const auto rows1=gf::buildCanonicalHardRows(request1);
        const auto problem1=gf::buildTask10p11sRows28d(
            rows1,request1.mobile_ids,true);
        const auto u1=attempt.successor.controls;
        const auto u1_map=gf::task10p11sControlMap(request1.mobile_ids,u1);
        const auto frozen1=gf::task10p11ag_component_detail::
            successorOutsideLocalControls(rows1,request1,u0_map,
                gf::task10p11ag_component_detail::kSeedPair);
        const auto frozen1_map=gf::task10p11sControlMap(
            request1.mobile_ids,frozen1);
        if (!gf::task10p11ahOnlyComponentDiffers(
                frozen1_map,u1_map,{2,9},1.0e-12))
            throw std::runtime_error("successor outside owner identity failed");
        const auto x2=gf::task10p11aa_detail::predictEstimate(snapshot,x1,u1_map);
        const auto request2=gf::task10p11x_detail::requestAtEstimate(snapshot,x2);
        const auto rows2=gf::buildCanonicalHardRows(request2);
        const auto problem2=gf::buildTask10p11sRows28d(
            rows2,request2.mobile_ids,true);
        const std::set<gf::NodeId> all(request2.mobile_ids.begin(),
                                       request2.mobile_ids.end());
        const auto terminal=gf::task10p11w_detail::solveRestricted(
            problem2,u1,u1,all,true);
        const bool terminal_full=terminal.feasible &&
            terminal.margin>=-gf::task10p11ag_detail::kTolerance &&
            terminal.minimum_residual>=-gf::task10p11ag_detail::kTolerance;
        double local=-std::numeric_limits<double>::infinity();
        bool signed_transfer=false;
        gf::task10p11ah_detail::ContinuationWitness continuation;
        if (terminal_full) {
            const auto u2_map=gf::task10p11sControlMap(
                request2.mobile_ids,terminal.controls);
            local=gf::task10p11af_detail::minimumOwnerLocalGamma(rows2,request2);
            signed_transfer=gf::solveTask10p11tDynamicPair(
                rows2,request2.mobile_ids,u2_map,
                request2.acceleration_half_box,"collision:2--9").feasible;
            continuation=gf::task10p11ah_detail::continuationWitness(
                snapshot,x2,request2,terminal.controls);
        }
        const auto residual0=gf::task10p11w_detail::minimumResidual(problem0,u0);
        const auto residual1=gf::task10p11w_detail::minimumResidual(problem1,u1);
        const nlohmann::json output={{"protocol",
            "task10p11ah-component-H2-plan-audit-v1"},
            {"valid",true},{"component",nlohmann::json::array({2,9})},
            {"outside_current_controls_equal_distributed",
                gf::task10p11ahOnlyComponentDiffers(
                    distributed_map,u0_map,{2,9},1.0e-12)},
            {"outside_x1_controls_equal_canonical_local",true},
            {"x0",{{"row_count",problem0.rows.size()},
                {"minimum_residual_mps2",residual0.first},
                {"controls",gf::task10p11ag_detail::controlsJson(
                    request0.mobile_ids,u0)},
                {"all_1113_residuals",gf::task10p11ag_detail::residualsJson(
                    problem0,u0)}}},
            {"x1",{{"row_count",problem1.rows.size()},
                {"minimum_residual_mps2",residual1.first},
                {"controls",gf::task10p11ag_detail::controlsJson(
                    request1.mobile_ids,u1)},
                {"all_1113_residuals",gf::task10p11ag_detail::residualsJson(
                    problem1,u1)}}},
            {"x2",{{"row_count",problem2.rows.size()},
                {"full_pair_feasible",terminal_full},
                {"full_pair_margin_mps2",gf::task10p11w_detail::number(
                    terminal.margin)},
                {"minimum_residual_mps2",gf::task10p11w_detail::number(
                    terminal.minimum_residual)},
                {"owner_local_gamma_mps2",gf::task10p11w_detail::number(local)},
                {"signed_transfer_feasible",signed_transfer},
                {"H1_constructive_witness",continuation.h1},
                {"H1_constructive_margin_mps2",gf::task10p11w_detail::number(
                    continuation.h1_margin)},
                {"H2_constructive_witness",continuation.h2},
                {"H2_constructive_margin_mps2",gf::task10p11w_detail::number(
                    continuation.h2_margin)},
                {"controls",terminal_full?gf::task10p11ag_detail::controlsJson(
                    request2.mobile_ids,terminal.controls):nlohmann::json(nullptr)},
                {"all_1113_residuals",terminal_full?
                    gf::task10p11ag_detail::residualsJson(
                        problem2,terminal.controls):nlohmann::json(nullptr)}}},
            {"cumulative_component_control_deviation_l2_mps2",
                (u0-distributed).norm()+(u1-frozen1).norm()},
            {"claim_boundary",{{"offline_exact_ZOH_plan",true},
                {"terminal_native_tau22_not_yet_audited",true},
                {"recursive_feasibility_claimed",false},
                {"production_controller",false}}}};
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return terminal_full?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ah component plan audit failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
