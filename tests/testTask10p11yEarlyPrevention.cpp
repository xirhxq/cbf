#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11yEarlyPrevention.hpp"

#include <chrono>

namespace {

std::filesystem::path checkpoint(const std::string& name) {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/
        "docs/evidence/task10p11v-unique-recapture/checkpoints"/name;
}

}  // namespace

TEST_CASE("132.4 signed-transfer baseline has a valid current interval") {
    const auto snapshot=gf::readTask10p11vJson(checkpoint(
        "checkpoint-000-t132.4-first_dynamic_intervention.json"));
    const auto decision=gf::task10p11y_detail::decide(snapshot);
    INFO(decision.reason);
    CHECK(decision.current_pair.shared_interval.feasible);
    CHECK(decision.valid);
    CHECK(decision.candidates.empty());
    CHECK_FALSE(decision.preventive_trigger);
}

TEST_CASE("132.8 predictive trigger evaluates exactly nine scalar candidates") {
    const auto snapshot=gf::readTask10p11vJson(checkpoint(
        "checkpoint-004-t132.8-dynamic_intervention.json"));
    const auto decision=gf::task10p11y_detail::decide(snapshot);
    INFO(decision.reason);
    INFO(decision.selection);
    INFO(decision.selected_index);
    REQUIRE(decision.current_pair.shared_interval.feasible);
    REQUIRE(decision.preventive_trigger);
    REQUIRE(decision.candidates.size()==9);
    const bool recognized_selection=
        decision.selection=="least_coverage_deviation_restoring_local" ||
        decision.selection=="maximum_predicted_local_margin";
    CHECK(recognized_selection);
    CHECK(decision.candidates.at(decision.selected_index).current_feasible);
    const auto encoded=gf::task10p11y_detail::decisionJson(decision);
    CHECK_NOTHROW(gf::validateTask10p11vFiniteJson(encoded));
}

TEST_CASE("130.0 no-trigger T1 matches C0 and publishes a recoverable frame") {
    const auto sparse=gf::readTask10p11vJson(checkpoint(
        "sparse-1300-t130.0.json"));
    auto c0=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    auto t1=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(c0->adapter.initializeStageZero().initialized);
    REQUIRE(t1->adapter.initializeStageZero().initialized);
    gf::restoreTask10p11vSparseRestartCheckpoint(*c0,sparse);
    gf::restoreTask10p11vSparseRestartCheckpoint(*t1,sparse);
    const auto c0_step=c0->controller.advance();
    std::optional<gf::task10p11y_detail::Decision> decision;
    const auto t1_step=t1->controller.advanceWithDevelopmentControlOverride(
        [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
            const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
            const std::map<gf::NodeId,double>& yaw_rates) {
            const auto snapshot=gf::makeTask10p11sSnapshot(runtime,
                t1->adapter.snapshotHardRowRequest(
                    runtime.estimate,runtime.topology),nominal,
                t1->adapter.config());
            decision=gf::task10p11y_detail::decide(snapshot);
            if (!decision->valid) {
                gf::GrandFinaleSwarmStep rejected;
                rejected.reason=decision->reason;
                return rejected;
            }
            REQUIRE_FALSE(decision->preventive_trigger);
            REQUIRE(decision->baseline_fixed_half);
            return t1->adapter.stepWithNominalAndYawRates(
                nominal,yaw_rates);
        });
    REQUIRE(c0_step.step.advanced);
    REQUIRE(t1_step.step.advanced);
    REQUIRE(decision.has_value());
    REQUIRE(c0_step.step.applied_controls.size()==14);
    for (const auto& [owner,control]:c0_step.step.applied_controls)
        CHECK((control-t1_step.step.applied_controls.at(owner)).norm()==
            doctest::Approx(0.0).epsilon(1.0e-12));
    CHECK(gf::task10p11vRestartStateJson(*c0)==
        gf::task10p11vRestartStateJson(*t1));

    const auto directory=std::filesystem::temp_directory_path()/(
        "task10p11y-shadow-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto path=directory/"packed.json";
    const auto packed=gf::makeTask10p11yCurrentPackedCheckpoint(
        *t1,"fixture_after_one_cycle");
    gf::writeTask10p11vJson(path,packed);
    const auto restored=gf::readTask10p11vJson(path);
    const auto audit=gf::auditTask10p11vRestartCheckpoint(restored);
    INFO(audit.reason);
    CHECK(audit.offline_oracle_complete);
    CHECK(audit.deterministic_restart_complete);
    std::filesystem::remove_all(directory);
}
