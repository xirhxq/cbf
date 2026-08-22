#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task10p11afSuccessorRecovery.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <filesystem>
#include <chrono>

namespace {

std::filesystem::path p3(const std::string& name) {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ae-ledger-closed-eight-cell-confirmation"/"P3-tau22"/
        "checkpoints"/name;
}

}  // namespace

TEST_CASE("Task 10.11af freezes tau22, fixed topology, and nine candidates") {
    const auto frozen=gf::task10p11afFrozenProtocol();
    CHECK(frozen.tau_mps2==doctest::Approx(22.0));
    CHECK(frozen.candidate_count==9);
    CHECK(frozen.dt_s==doctest::Approx(0.1));
    CHECK(frozen.initialization=="P3");
    CHECK(frozen.monitored_pair=="collision:2--9");
    CHECK(frozen.fixed_topology);
    CHECK(frozen.development_oracle);
    CHECK_FALSE(frozen.recursive_feasibility_claimed);
}

TEST_CASE("P3 158.0 packed checkpoint exposes fourteen applied controls") {
    const auto snapshot=gf::readTask10p11vJson(p3(
        "checkpoint-0009-t158.0-first_margin_layer_divergence.json"));
    const auto controls=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
    CHECK(controls.size()==14);
    for (const auto& [owner,control]:controls) {
        CHECK(owner>=1);
        CHECK(owner<=14);
        CHECK(control.allFinite());
    }
}

TEST_CASE("P3 158.0 distinguishes local current and successor layers") {
    const auto audit=gf::task10p11afAuditPackedCheckpoint(p3(
        "checkpoint-0009-t158.0-first_margin_layer_divergence.json"));
    REQUIRE(audit.valid);
    CHECK(audit.current_full_row_count==1113);
    CHECK(audit.owner_local_gamma_mps2>=-1.0e-8);
    CHECK(audit.signed_transfer.valid);
    CHECK(audit.current_full_pair_margin.feasible);
    CHECK(audit.applied_current_full_row_residual_mps2>=-1.0e-8);
    CHECK_FALSE(audit.applied_successor_full_pair.feasible);
    CHECK(audit.g1.takeover_required);
    CHECK(audit.g1.oracle.candidates.size()==9);
    CHECK(audit.g2.oracle.candidates.size()==9);
    CHECK(audit.g2.oracle.predecessor_semantics==
        "finite_9x9_full_pair_witness_not_unrestricted_predecessor");
}

TEST_CASE("disabled takeover returns the distributed command unchanged") {
    const auto snapshot=gf::readTask10p11vJson(p3(
        "checkpoint-0009-t158.0-first_margin_layer_divergence.json"));
    const auto controls=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
    const auto disabled=gf::task10p11afDecideTakeover(snapshot,controls,
        gf::Task10p11aaGraphDepth::H1,false);
    REQUIRE(disabled.valid);
    CHECK_FALSE(disabled.takeover_applied);
    CHECK(gf::task10p11zAppliedControlsMatch(
        controls,disabled.selected_controls,0.0));
}

TEST_CASE("P3 158.0 restart reproduces the frozen distributed command") {
    const auto snapshot=gf::readTask10p11vJson(p3(
        "checkpoint-0009-t158.0-first_margin_layer_divergence.json"));
    const auto manifest=gf::readTask10p11vJson(
        std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ac-fixed-tau22-eight-cell-confirmation"/
        "initialization-manifest.json");
    const auto initialization=gf::task10p11acInitializationFromManifest(
        manifest,"P3");
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=initialization.positions;
    auto settings=gf::task10p11acSwarmSettings(
        scenario,initialization.velocities);
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::restoreTask10p11vRestartState(
        *fixture,snapshot.at("restart_checkpoint"));
    const auto prepared=gf::task10p11zPrepareNativeBaseline(
        *fixture,gf::GammaFeedbackSelectionMode::LeastIntervention,
        22.0,std::nullopt);
    REQUIRE(prepared.valid);
    REQUIRE(prepared.control.step.advanced);
    CHECK(gf::task10p11zAppliedControlsMatch(
        gf::task10p11afAppliedControlsFromCheckpoint(snapshot),
        prepared.control.step.applied_controls,1.0e-10));
    CHECK(fixture->topologyFrozen());
}

TEST_CASE("disabled development path preserves P3 plant estimator coverage and yaw") {
    const auto manifest=gf::readTask10p11vJson(
        std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ac-fixed-tau22-eight-cell-confirmation"/
        "initialization-manifest.json");
    const auto initialization=gf::task10p11acInitializationFromManifest(
        manifest,"P3");
    const auto make=[&]() {
        auto scenario=gf::task10p11rFixedBaselineScenario();
        scenario.mobile_positions=initialization.positions;
        auto settings=gf::task10p11acSwarmSettings(
            scenario,initialization.velocities);
        auto fixture=gf::makeTask10p11rFixedBaselineFixture(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
        if (!fixture->adapter.initializeStageZero().initialized)
            throw std::runtime_error("fixture initialization failed");
        return fixture;
    };
    auto expected=make();
    auto actual=make();
    const auto expected_step=expected->controller.advance();
    const auto prepared=gf::task10p11zPrepareNativeBaseline(
        *actual,gf::GammaFeedbackSelectionMode::LeastIntervention,
        22.0,std::nullopt);
    REQUIRE(expected_step.step.advanced);
    REQUIRE(prepared.valid);
    REQUIRE(prepared.control.step.advanced);
    const auto actual_step=actual->controller.advanceWithDevelopmentControlOverride(
        [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
            const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
            const std::map<gf::NodeId,double>& yaw_rates) {
            CHECK(gf::task10p11zAppliedControlsMatch(
                nominal,prepared.nominal_controls,1.0e-12));
            return actual->adapter.stepWithDevelopmentFullPairCertifiedControls(
                prepared.control.step.applied_controls,yaw_rates,
                runtime.estimator_token,runtime.topology_token,true,0.0);
        });
    REQUIRE(actual_step.step.advanced);
    CHECK(gf::task10p11zAppliedControlsMatch(
        expected_step.step.applied_controls,
        actual_step.step.applied_controls,1.0e-12));
    CHECK(expected_step.step.applied_yaw_rates_radps==
        actual_step.step.applied_yaw_rates_radps);
    CHECK(expected_step.step.truth_coverage==
        doctest::Approx(actual_step.step.truth_coverage));
    const auto expected_restart=gf::captureTask10p11vRestartFields(*expected);
    const auto actual_restart=gf::captureTask10p11vRestartFields(*actual);
    CHECK(expected_restart.at("plant")==actual_restart.at("plant"));
    CHECK(expected_restart.at("adapter")==actual_restart.at("adapter"));
    CHECK(expected_restart.at("coverage")==actual_restart.at("coverage"));
    CHECK(expected->topologyFrozen());
    CHECK(actual->topologyFrozen());
}

TEST_CASE("Task 10.11af evidence rejects nonfinite JSON and packs post state") {
    const auto directory=std::filesystem::temp_directory_path()/
        ("task10p11af-evidence-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK_THROWS(gf::writeTask10p11vJson(
        directory/"nonfinite.json",nlohmann::json{{"bad",
            std::numeric_limits<double>::infinity()}}));
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    const auto packed=gf::makeTask10p11yCurrentPackedCheckpoint(
        *fixture,"post_advance_fixture");
    CHECK(packed.at("runtime").at("runtime_s").get<double>()==
        doctest::Approx(0.1));
    CHECK(packed.at("restart_checkpoint").at("runtime_s").get<double>()==
        doctest::Approx(0.1));
}
