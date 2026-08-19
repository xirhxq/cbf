#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <chrono>

namespace {

std::filesystem::path temporaryDirectory(const std::string& label) {
    return std::filesystem::temp_directory_path()/(
        "task10p11z-"+label+"-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
}

}  // namespace

TEST_CASE("disabled prevention compares native applied controls, not an offline reconstruction") {
    const auto sparse=gf::readTask10p11vJson(
        std::filesystem::path(PROJECT_ROOT).parent_path()/
        "docs/evidence/task10p11v-unique-recapture/checkpoints/"
        "sparse-1300-t130.0.json");
    auto baseline=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    auto disabled=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(baseline->adapter.initializeStageZero().initialized);
    REQUIRE(disabled->adapter.initializeStageZero().initialized);
    gf::restoreTask10p11vSparseRestartCheckpoint(*baseline,sparse);
    gf::restoreTask10p11vSparseRestartCheckpoint(*disabled,sparse);

    const auto baseline_step=baseline->controller.advance();
    const auto disabled_step=disabled->controller.advance();
    REQUIRE(baseline_step.step.advanced);
    REQUIRE(disabled_step.step.advanced);
    CHECK(gf::task10p11zAppliedControlsMatch(
        baseline_step.step.applied_controls,
        disabled_step.step.applied_controls,1.0e-12));
}

TEST_CASE("prepared baseline is the native controller proposal from the same restart state") {
    const auto sparse=gf::readTask10p11vJson(
        std::filesystem::path(PROJECT_ROOT).parent_path()/
        "docs/evidence/task10p11v-unique-recapture/checkpoints/"
        "sparse-1300-t130.0.json");
    auto baseline=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    auto source=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(baseline->adapter.initializeStageZero().initialized);
    REQUIRE(source->adapter.initializeStageZero().initialized);
    gf::restoreTask10p11vSparseRestartCheckpoint(*baseline,sparse);
    gf::restoreTask10p11vSparseRestartCheckpoint(*source,sparse);

    const auto expected=baseline->controller.advance();
    const auto prepared=gf::task10p11zPrepareNativeBaseline(
        *source,gf::GammaFeedbackSelectionMode::LeastIntervention,14.0,
        std::nullopt);
    INFO(prepared.reason);
    REQUIRE(expected.step.advanced);
    REQUIRE(prepared.valid);
    REQUIRE(prepared.control.step.advanced);
    CHECK_FALSE(prepared.active_pair.has_value());
    CHECK(gf::task10p11zAppliedControlsMatch(
        expected.step.applied_controls,
        prepared.control.step.applied_controls,1.0e-12));
}

TEST_CASE("restart evidence is captured before entering the control override") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto before=gf::task10p11zCaptureBeforeOverride(*fixture);
    bool callback=false;
    const auto control=fixture->controller.advanceWithDevelopmentControlOverride(
        [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
            const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
            const std::map<gf::NodeId,double>& yaw_rates) {
            callback=true;
            CHECK(runtime.runtime_s==doctest::Approx(before.runtime.runtime_s));
            CHECK(before.restart_fields.contains("plant"));
            return fixture->adapter.stepWithNominalAndYawRates(
                nominal,yaw_rates);
        });
    REQUIRE(callback);
    CHECK(control.step.advanced);
}

TEST_CASE("disabled prevention preserves the C0 native proposal and commit for every suffix frame") {
    const auto packed=gf::readTask10p11vJson(
        std::filesystem::path(PROJECT_ROOT).parent_path()/
        "docs/evidence/task10p11v-unique-recapture/checkpoints/"
        "checkpoint-000-t132.4-first_dynamic_intervention.json");
    auto c0=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    auto disabled=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(c0->adapter.initializeStageZero().initialized);
    REQUIRE(disabled->adapter.initializeStageZero().initialized);
    gf::restoreTask10p11vRestartState(*c0,packed.at("restart_checkpoint"));
    gf::restoreTask10p11vRestartState(
        *disabled,packed.at("restart_checkpoint"));
    std::optional<std::string> active_pair="reference:2->4";
    for (std::size_t cycle=0;cycle<6;++cycle) {
        const auto prepared=gf::task10p11zPrepareNativeBaseline(
            *disabled,gf::GammaFeedbackSelectionMode::LeastIntervention,14.0,
            active_pair);
        REQUIRE(prepared.valid);
        active_pair=prepared.active_pair;
        const auto expected=active_pair.has_value()
            ?c0->controller.advanceWithDynamicPairResponsibility(*active_pair)
            :c0->controller.advance();
        const auto actual=active_pair.has_value()
            ?disabled->controller.advanceWithDynamicPairResponsibility(
                *active_pair):disabled->controller.advance();
        INFO(cycle);
        CHECK(actual.step.advanced==expected.step.advanced);
        CHECK(actual.reason==expected.reason);
        CHECK(gf::task10p11zAppliedControlsMatch(
            actual.step.applied_controls,expected.step.applied_controls,
            1.0e-12));
        CHECK(gf::task10p11vRestartStateJson(*disabled)==
            gf::task10p11vRestartStateJson(*c0));
        if (!expected.step.advanced) break;
    }
}

TEST_CASE("every termination class publishes an explicit result and recoverable checkpoint") {
    for (const auto& [boundary,advanced,valid]:
         std::vector<std::tuple<gf::Task10p11zTerminationBoundary,bool,bool>>{
             {gf::Task10p11zTerminationBoundary::PreAdvance,false,false},
             {gf::Task10p11zTerminationBoundary::GuardRejection,false,true},
             {gf::Task10p11zTerminationBoundary::PostAdvance,true,false},
             {gf::Task10p11zTerminationBoundary::Exception,true,false},
             {gf::Task10p11zTerminationBoundary::NormalEnd,true,true}}) {
        auto fixture=gf::makeTask10p11rFixedBaselineFixture(
            gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
        REQUIRE(fixture->adapter.initializeStageZero().initialized);
        if (advanced) {
            const auto step=fixture->controller.advance();
            INFO(step.reason);
            REQUIRE(step.step.advanced);
        }
        const auto directory=temporaryDirectory("termination");
        const auto result_path=directory/"result.json";
        const auto checkpoint_path=directory/"termination.json";
        gf::writeTask10p11zTerminationEvidence(result_path,checkpoint_path,
            *fixture,{boundary,valid,advanced,"fixture_termination",
                {{"profile","fixture"}}});

        const auto result=gf::readTask10p11vJson(result_path);
        const auto checkpoint=gf::readTask10p11vJson(checkpoint_path);
        CHECK(result.at("valid").get<bool>()==valid);
        CHECK(result.at("advanced").get<bool>()==advanced);
        CHECK(result.at("termination_boundary")==
            gf::task10p11zTerminationBoundaryName(boundary));
        CHECK(result.at("checkpoint_published").get<bool>());
        if (advanced) {
            const auto audit=gf::auditTask10p11vRestartCheckpoint(checkpoint);
            CHECK(audit.deterministic_restart_complete);
            CHECK(audit.offline_oracle_complete);
        } else {
            auto restored_fixture=gf::makeTask10p11rFixedBaselineFixture(
                gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
            REQUIRE(restored_fixture->adapter.initializeStageZero().initialized);
            CHECK_NOTHROW(gf::restoreTask10p11vSparseRestartCheckpoint(
                *restored_fixture,checkpoint));
            CHECK(gf::task10p11vRestartStateJson(*restored_fixture)==
                gf::task10p11vRestartStateJson(*fixture));
        }
        CHECK(checkpoint.at("task10p11z").at("termination_boundary")==
            gf::task10p11zTerminationBoundaryName(boundary));
        std::filesystem::remove_all(directory);
    }
}
