#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task10p11acCampaign.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <chrono>
#include <filesystem>

namespace {

gf::GrandFinaleGammaFeedbackDiagnostic diagnostic(
    std::size_t index,bool attained,const std::string& fallback={}) {
    gf::GrandFinaleGammaFeedbackDiagnostic value;
    value.current_gamma=1.0;
    value.nominal_predicted_gamma=2.0;
    value.maximum_margin_candidate_predicted_gamma=3.0;
    value.selected_predicted_gamma=2.5;
    value.current_hard_projection={0.1,0.2};
    value.maximum_margin_control={0.3,0.4};
    value.selected_nominal={0.2,0.3};
    value.intervened=index!=0;
    value.selected_candidate_index=index;
    value.tau_attainment_valid=fallback!=
        "invalid_prediction_use_current_projection";
    value.tau_attained=attained;
    value.fallback_reason=fallback;
    return value;
}

}  // namespace

TEST_CASE("Task 10.11ac feedback branches are mutually exclusive") {
    CHECK(gf::task10p11acClassifyFeedback(diagnostic(0,true))==
        gf::Task10p11acFeedbackBranch::NominalRetention);
    CHECK(gf::task10p11acClassifyFeedback(diagnostic(3,true))==
        gf::Task10p11acFeedbackBranch::LeastInterventionTauAttained);
    CHECK(gf::task10p11acClassifyFeedback(diagnostic(0,false,
        "tau_unattained_maximum_predicted_margin"))==
        gf::Task10p11acFeedbackBranch::TauUnattainedMaximumMarginFallback);
    CHECK(gf::task10p11acClassifyFeedback(diagnostic(0,false,
        "invalid_prediction_use_current_projection"))==
        gf::Task10p11acFeedbackBranch::InvalidPredictionProjectionFallback);
}

TEST_CASE("Task 10.11ac owner ledger rejects nonfinite values") {
    const auto entry=gf::task10p11acOwnerDecisionJson(
        2,diagnostic(4,true),{0.0,0.0},{0.1,0.2},9);
    CHECK(entry.at("selected_alpha")==doctest::Approx(0.5));
    CHECK(entry.at("branch")=="least_intervention_tau_attained");
    auto invalid=diagnostic(0,true);
    invalid.current_gamma=std::numeric_limits<double>::infinity();
    CHECK_THROWS(gf::task10p11acOwnerDecisionJson(
        2,invalid,{0.0,0.0},{0.1,0.2},9));

    auto prediction_invalid=diagnostic(0,false,
        "invalid_prediction_use_current_projection");
    prediction_invalid.nominal_predicted_gamma=
        -std::numeric_limits<double>::infinity();
    prediction_invalid.maximum_margin_candidate_predicted_gamma=
        -std::numeric_limits<double>::infinity();
    prediction_invalid.selected_predicted_gamma=
        -std::numeric_limits<double>::infinity();
    const auto invalid_entry=gf::task10p11acOwnerDecisionJson(
        2,prediction_invalid,{0.0,0.0},{0.1,0.2},9);
    CHECK(invalid_entry.at("branch")==
        "invalid_prediction_projection_fallback");
    CHECK(invalid_entry.at("nominal_predicted_gamma_mps2").at("status")==
        "invalid");
    CHECK(invalid_entry.at("nominal_predicted_gamma_mps2").at("value").
        is_null());
}

TEST_CASE("Task 10.11ac evidence writer rejects nonfinite JSON") {
    const auto path=std::filesystem::temp_directory_path()/
        "task10p11ac-nonfinite.json";
    CHECK_THROWS(gf::writeTask10p11vJson(path,
        {{"invalid",std::numeric_limits<double>::infinity()}}));
}

TEST_CASE("Task 10.11ac cycle statistics count only advanced cycles") {
    nlohmann::json decisions=nlohmann::json::array();
    for (gf::NodeId owner=1;owner<=14;++owner)
        decisions.push_back(gf::task10p11acOwnerDecisionJson(
            owner,diagnostic(0,true),{0.0,0.0},{0.1,0.2},9));
    gf::Task10p11acCycleStatistics statistics;
    statistics.observe(false,decisions);
    CHECK(statistics.advanced_cycles==0);
    CHECK(statistics.owner_branch_counts.empty());
    statistics.observe(true,decisions);
    CHECK(statistics.advanced_cycles==1);
    CHECK(statistics.feedback_applicable_owner_decisions==14);
    CHECK(statistics.not_applicable_owner_decisions==0);
    CHECK(statistics.owner_status_counts.at("valid_feedback_decision")==14);
    CHECK(statistics.owner_branch_counts.at("nominal_retention")==14);
    CHECK(statistics.owner_alpha_counts.at(0)==14);
}

TEST_CASE("Task 10.11ac observation leaves applied controls unchanged") {
    auto plain=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    auto observed=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    REQUIRE(plain->adapter.initializeStageZero().initialized);
    REQUIRE(observed->adapter.initializeStageZero().initialized);
    for (std::size_t cycle=0;cycle<2;++cycle) {
        const auto runtime=observed->adapter.runtimeSnapshot();
        const auto rows=gf::buildCanonicalHardRows(
            observed->adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology));
        const auto plain_step=plain->controller.advance();
        const auto observed_step=observed->controller.advance();
        REQUIRE(plain_step.step.advanced);
        REQUIRE(observed_step.step.advanced);
        REQUIRE(plain_step.step.applied_controls.size()==14);
        REQUIRE(observed_step.step.applied_controls.size()==14);
        const auto ledger=gf::task10p11aeOwnerLedger(
            runtime.estimate.mobile_ids,observed_step.step.gamma_feedback,
            observed->controller.lastNominalControls(),
            observed_step.step.applied_controls,rows,
            observed_step.step.dynamic_pair,true,9);
        CHECK(ledger.size()==14);
        for (const auto& [owner,control]:plain_step.step.applied_controls)
            CHECK((control-observed_step.step.applied_controls.at(owner)).norm()==
                doctest::Approx(0.0));
        CHECK(gf::captureTask10p11vRestartFields(*plain)==
              gf::captureTask10p11vRestartFields(*observed));
    }
}

TEST_CASE("Task 10.11ae dynamic pair ledger is complete and observational") {
    auto plain=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    auto observed=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    REQUIRE(plain->adapter.initializeStageZero().initialized);
    REQUIRE(observed->adapter.initializeStageZero().initialized);

    for (std::size_t cycle=0;cycle<2;++cycle) {
        const auto runtime=observed->adapter.runtimeSnapshot();
        const auto rows=gf::buildCanonicalHardRows(
            observed->adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology));
        const auto plain_step=plain->controller.
            advanceWithDynamicPairResponsibility("reference:2->4");
        const auto observed_step=observed->controller.
            advanceWithDynamicPairResponsibility("reference:2->4");
        REQUIRE(plain_step.step.advanced);
        REQUIRE(observed_step.step.advanced);
        REQUIRE(observed_step.step.dynamic_pair.applied);
        CHECK(observed_step.step.gamma_feedback.empty());
        const auto ledger=gf::task10p11aeOwnerLedger(
            runtime.estimate.mobile_ids,observed_step.step.gamma_feedback,
            observed->controller.lastNominalControls(),
            observed_step.step.applied_controls,rows,
            observed_step.step.dynamic_pair,true,9);
        REQUIRE(ledger.size()==14);
        std::size_t pair=0,frozen=0;
        for (const auto& entry:ledger) {
            const auto status=entry.at("status").get<std::string>();
            pair+=status=="not_applicable_dynamic_pair_override";
            frozen+=status=="not_applicable_other_frozen_control_path";
            CHECK_FALSE(entry.at("feedback_applicable").get<bool>());
            CHECK(entry.at("all_current_hard_rows_passed").get<bool>());
        }
        CHECK(pair==2);
        CHECK(frozen==12);
        gf::Task10p11acCycleStatistics statistics;
        statistics.observe(true,ledger);
        CHECK(statistics.feedback_applicable_owner_decisions==0);
        CHECK(statistics.not_applicable_owner_decisions==14);
        CHECK(statistics.owner_branch_counts.empty());
        CHECK(statistics.owner_alpha_counts.empty());
        for (const auto& [owner,control]:plain_step.step.applied_controls)
            CHECK((control-observed_step.step.applied_controls.at(owner)).norm()==
                doctest::Approx(0.0));
        for (const auto& [owner,rate]:plain_step.step.applied_yaw_rates_radps)
            CHECK(rate==doctest::Approx(
                observed_step.step.applied_yaw_rates_radps.at(owner)));
        CHECK(plain_step.step.minimum_hard_residual==doctest::Approx(
            observed_step.step.minimum_hard_residual));
        CHECK(gf::captureTask10p11vRestartFields(*plain)==
              gf::captureTask10p11vRestartFields(*observed));
    }

    const auto runtime=observed->adapter.runtimeSnapshot();
    const auto rows=gf::buildCanonicalHardRows(
        observed->adapter.snapshotHardRowRequest(
            runtime.estimate,runtime.topology));
    const auto plain_exit=plain->controller.advance();
    const auto observed_exit=observed->controller.advance();
    REQUIRE(plain_exit.step.advanced);
    REQUIRE(observed_exit.step.advanced);
    CHECK_FALSE(observed_exit.step.dynamic_pair.attempted);
    const auto exit_ledger=gf::task10p11aeOwnerLedger(
        runtime.estimate.mobile_ids,observed_exit.step.gamma_feedback,
        observed->controller.lastNominalControls(),
        observed_exit.step.applied_controls,rows,
        observed_exit.step.dynamic_pair,true,9);
    REQUIRE(exit_ledger.size()==14);
    for (const auto& entry:exit_ledger)
        CHECK(entry.at("status")=="valid_feedback_decision");
    CHECK(gf::captureTask10p11vRestartFields(*plain)==
          gf::captureTask10p11vRestartFields(*observed));
}

TEST_CASE("Task 10.11ae fail-closed ledger is explicit and finite") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto runtime=fixture->adapter.runtimeSnapshot();
    const auto rows=gf::buildCanonicalHardRows(
        fixture->adapter.snapshotHardRowRequest(
            runtime.estimate,runtime.topology));
    gf::GrandFinaleDynamicPairDiagnostic failed;
    failed.attempted=true;
    failed.reason="dynamic_pair_responsibility_interval_empty";
    failed.pair_base_id="reference:2->4";
    const auto ledger=gf::task10p11aeOwnerLedger(runtime.estimate.mobile_ids,
        {},fixture->controller.lastNominalControls(),{},rows,failed,false,9);
    REQUIRE(ledger.size()==14);
    for (const auto& entry:ledger) {
        CHECK(entry.at("status")=="invalid");
        CHECK(entry.at("actual_applied_control").is_null());
    }
    const auto path=std::filesystem::temp_directory_path()/
        ("task10p11ae-fail-closed-ledger-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count())+
         ".json");
    CHECK_NOTHROW(gf::writeTask10p11vJson(path,ledger));
}

TEST_CASE("Task 10.11ac base identity excludes only explicit tau") {
    auto tau20=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,20.0);
    auto tau22=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    CHECK(gf::task10p11acBaseIdentityJson(*tau20)==
          gf::task10p11acBaseIdentityJson(*tau22));
    CHECK(*tau20->adapter.config().predictive_gamma_tau_mps2==
        doctest::Approx(20.0));
    CHECK(*tau22->adapter.config().predictive_gamma_tau_mps2==
        doctest::Approx(22.0));
}

TEST_CASE("Task 10.11ac pre and post termination checkpoints match phase") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto directory=std::filesystem::temp_directory_path()/
        ("task10p11ac-termination-fixture-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    gf::writeTask10p11zTerminationEvidence(directory/"pre-result.json",
        directory/"pre-checkpoint.json",*fixture,
        {gf::Task10p11zTerminationBoundary::PreAdvance,false,false,
         "fixture_pre_advance"});
    CHECK(gf::readTask10p11vJson(directory/"pre-result.json").at(
        "termination_boundary")=="pre_advance");
    REQUIRE(fixture->controller.advance().step.advanced);
    gf::writeTask10p11zTerminationEvidence(directory/"post-result.json",
        directory/"post-checkpoint.json",*fixture,
        {gf::Task10p11zTerminationBoundary::PostAdvance,false,true,
         "fixture_post_advance"});
    const auto post=gf::readTask10p11vJson(directory/"post-result.json");
    CHECK(post.at("termination_boundary")=="post_advance");
    CHECK(post.at("checkpoint_kind")=="packed");
}
