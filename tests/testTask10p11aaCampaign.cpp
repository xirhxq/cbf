#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task10p11aaCampaign.hpp"
#include "grand_finale/Task10p11aaGraphOracle.hpp"
#include "grand_finale/Task10p11aaMarginAudit.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <filesystem>

namespace {

std::filesystem::path evidence(const std::string& relative) {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11z-stage-zero-recovery"/relative;
}

}  // namespace

TEST_CASE("Task 10.11aa preregistration freezes profiles before execution") {
    const auto frozen=gf::task10p11aaPreregisteredCampaign();
    CHECK(frozen.profile_order==
        std::vector<std::string>{"D20","D22","G1","G2","T1"});
    CHECK(frozen.d20_tau_mps2==doctest::Approx(20.0));
    CHECK(frozen.d22_tau_mps2==doctest::Approx(22.0));
    CHECK(frozen.graph_tau_mps2==doctest::Approx(14.0));
    CHECK(frozen.candidate_count==9);
    CHECK(frozen.g2_successor_witness_count==9);
    CHECK(frozen.gamma_star_is_state_diagnostic);
    CHECK(frozen.tau_is_feedback_threshold);
    CHECK(frozen.g1_is_development_centralized_oracle);
    CHECK(frozen.g2_is_finite_depth_not_recursive);
}

TEST_CASE("Gate A keeps all four margin layers distinct") {
    const auto frame=gf::task10p11aaAuditPackedCheckpoint(evidence(
        "R0/checkpoints/checkpoint-0001-t132.8-first_prediction.json"));
    CHECK(frame.valid);
    CHECK(frame.owner_local_gamma_mps2.size()==14);
    CHECK(std::isfinite(frame.minimum_owner_local_gamma_mps2));
    CHECK(frame.signed_transfer.valid);
    CHECK(frame.pair_component_margin.status!="not_solved");
    CHECK(frame.diagnostic_component==std::set<gf::NodeId>{2,4});
    CHECK(frame.monitored_pair=="reference:2->4");
    CHECK(frame.full_pair_margin.status!="not_solved");
    CHECK(frame.full_row_count==1113);
    CHECK(frame.oracle_successor_performed);
    CHECK(frame.oracle_successor_full_row_count==1113);
    CHECK(frame.margin_layers_are_interchangeable==false);
}

TEST_CASE("G1 uses nine full-pair candidates and exact-ZOH successor gamma") {
    const auto snapshot=gf::readTask10p11vJson(evidence(
        "R0/checkpoints/checkpoint-0000-t132.4-first_intervention.json"));
    const auto baseline=gf::task10p11aaAppliedControlsFromCheckpoint(snapshot);
    const auto decision=gf::task10p11aaDecideGraphOracle(
        snapshot,baseline,gf::Task10p11aaGraphDepth::H1);
    CHECK(decision.valid);
    CHECK(decision.candidates.size()==9);
    CHECK(decision.selected_index<9);
    CHECK(decision.current_endpoint.feasible);
    CHECK(decision.current_endpoint.margin_mps2==
        doctest::Approx(decision.current_endpoint.recomputed_gamma_mps2));
    for (const auto& candidate:decision.candidates) {
        CHECK(candidate.index<9);
        CHECK(candidate.alpha==doctest::Approx(
            static_cast<double>(candidate.index)/8.0));
        CHECK(candidate.current_row_count==1113);
        CHECK(candidate.successor_row_count==1113);
        CHECK(candidate.independent_current_residual_recomputed);
        CHECK(candidate.independent_successor_residual_recomputed);
    }
    CHECK_FALSE(decision.recursive_feasibility_claimed);
}

TEST_CASE("G2 is a finite 9-by-9 predecessor witness gate") {
    const auto snapshot=gf::readTask10p11vJson(evidence(
        "R0/checkpoints/checkpoint-0000-t132.4-first_intervention.json"));
    const auto baseline=gf::task10p11aaAppliedControlsFromCheckpoint(snapshot);
    const auto decision=gf::task10p11aaDecideGraphOracle(
        snapshot,baseline,gf::Task10p11aaGraphDepth::H2FiniteWitness);
    CHECK(decision.valid);
    CHECK(decision.candidates.size()==9);
    CHECK(decision.predecessor_semantics==
        "finite_9x9_full_pair_witness_not_unrestricted_predecessor");
    CHECK_FALSE(decision.recursive_feasibility_claimed);
}

TEST_CASE("D20 changes only the native predicted-gamma threshold") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,20.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    REQUIRE(fixture->adapter.config().predictive_gamma_tau_mps2.has_value());
    CHECK(*fixture->adapter.config().predictive_gamma_tau_mps2==
        doctest::Approx(20.0));
    const auto step=fixture->controller.advance();
    CHECK(step.step.advanced);
    CHECK(step.step.gamma_feedback.size()==14);
    CHECK(fixture->topologyFrozen());
}

TEST_CASE("G1 consumes the prepared native tau14 joint command as U0") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto runtime=fixture->adapter.runtimeSnapshot();
    const auto request=fixture->adapter.snapshotHardRowRequest(
        runtime.estimate,runtime.topology);
    const auto prepared=gf::task10p11zPrepareNativeBaseline(*fixture,
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0,std::nullopt);
    REQUIRE(prepared.valid);
    REQUIRE(prepared.control.step.advanced);
    const auto snapshot=gf::makeTask10p11sSnapshot(runtime,request,
        prepared.nominal_controls,fixture->adapter.config());
    const auto decision=gf::task10p11aaDecideGraphOracle(snapshot,
        prepared.control.step.applied_controls,
        gf::Task10p11aaGraphDepth::H1);
    CHECK(decision.valid);
    CHECK(decision.candidates.size()==9);
    CHECK(fixture->controller.successfulControlCycles()==0);
    CHECK(fixture->topologyFrozen());
}
