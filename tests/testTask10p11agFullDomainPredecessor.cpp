#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task10p11agFullDomainPredecessor.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <filesystem>

namespace {

std::filesystem::path p3() {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ae-ledger-closed-eight-cell-confirmation"/"P3-tau22"/
        "checkpoints"/
        "checkpoint-0009-t158.0-first_margin_layer_divergence.json";
}

}  // namespace

TEST_CASE("Task 10.11ag freezes full-domain fixed-topology semantics") {
    const auto frozen=gf::task10p11agFrozenProtocol();
    CHECK(frozen.tau_mps2==doctest::Approx(22.0));
    CHECK(frozen.dt_s==doctest::Approx(0.1));
    CHECK(frozen.fixed_topology);
    CHECK(frozen.full_domain_u0);
    CHECK(frozen.full_domain_u1);
    CHECK_FALSE(frozen.recursive_feasibility_claimed);
}

TEST_CASE("full-domain evaluator reproduces the P3 distributed successor loss") {
    const auto snapshot=gf::readTask10p11vJson(p3());
    const auto request=gf::task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto controls=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
    const auto ordered=gf::task10p11sOrderedControls(
        request.mobile_ids,controls);
    const auto context=gf::task10p11ag_detail::makeContext(snapshot,ordered);
    const auto evaluation=gf::task10p11ag_detail::evaluate(context,ordered);
    REQUIRE(evaluation.solved);
    CHECK(context.current_problem.rows.size()==1113);
    CHECK(evaluation.successor_problem.rows.size()==1113);
    CHECK(evaluation.current_minimum_residual>=-1.0e-8);
    CHECK(evaluation.successor_gamma<0.0);
    CHECK_FALSE(evaluation.witness);
    CHECK(evaluation.successor_limiting_row==
        "collision:2--9:full-pair-once-reserve");
}

TEST_CASE("disabled Task 10.11ag recovery is observationally identical") {
    const auto snapshot=gf::readTask10p11vJson(p3());
    const auto controls=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
    const auto disabled=gf::task10p11afDecideTakeover(snapshot,controls,
        gf::Task10p11aaGraphDepth::H1,false);
    REQUIRE(disabled.valid);
    CHECK_FALSE(disabled.takeover_applied);
    CHECK(gf::task10p11zAppliedControlsMatch(
        controls,disabled.selected_controls,0.0));
}

TEST_CASE("Task 10.11ag evidence encoding rejects nonfinite values") {
    gf::Task10p11agFullDomainResult result;
    const auto encoded=gf::task10p11agFullDomainJson(result,{});
    CHECK(encoded.at("best_successor_gamma_mps2").is_null());
    CHECK(encoded.at("coverage_control_deviation_l2_mps2").is_null());
    CHECK(encoded.at("strict_infeasibility_proven")==false);
}
