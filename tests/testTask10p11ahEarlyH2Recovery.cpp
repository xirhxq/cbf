#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task10p11ahEarlyH2Recovery.hpp"
#include "grand_finale/Task10p11ahTerminalRecoveryOptimizer.hpp"

#include <filesystem>
#include <chrono>

namespace {

std::filesystem::path evidence(const std::string& relative) {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ag-full-domain-predecessor-recovery"/relative;
}

std::filesystem::path manifestPath() {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ac-fixed-tau22-eight-cell-confirmation"/
        "initialization-manifest.json";
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeP3() {
    const auto manifest=gf::readTask10p11vJson(manifestPath());
    const auto initialization=gf::task10p11acInitializationFromManifest(
        manifest,"P3");
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=initialization.positions;
    auto settings=gf::task10p11acSwarmSettings(
        scenario,initialization.velocities);
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    if (!fixture->adapter.initializeStageZero().initialized)
        throw std::runtime_error("P3 fixture initialization failed");
    return fixture;
}

}  // namespace

TEST_CASE("Task 10.11ah freezes early H2 component semantics") {
    const auto frozen=gf::task10p11ahFrozenProtocol();
    CHECK(frozen.tau_mps2==doctest::Approx(22.0));
    CHECK(frozen.dt_s==doctest::Approx(0.1));
    CHECK(frozen.component==std::set<gf::NodeId>{2,9});
    CHECK(frozen.horizon_steps==2);
    CHECK(frozen.fixed_topology);
    CHECK_FALSE(frozen.recursive_feasibility_claimed);
}

TEST_CASE("post-hoc terminal optimizer freezes lexicographic component protocol") {
    const auto frozen=gf::task10p11ahPosthocOptimizerProtocol();
    CHECK(frozen.post_hoc_development_extension);
    CHECK(frozen.component==std::set<gf::NodeId>{2,9});
    CHECK(frozen.horizon_steps==2);
    CHECK(frozen.tau_mps2==doctest::Approx(22.0));
    CHECK(frozen.fixed_topology);
    CHECK(frozen.external_controls==
        "same_frame_canonical_distributed_controls");
    CHECK(frozen.optimizer=="deterministic_continuous_pattern_search");
    CHECK_FALSE(frozen.component_expansion_allowed);
    CHECK_FALSE(frozen.depth_expansion_allowed);
}

TEST_CASE("terminal optimizer orders feasibility then recovery then deviation") {
    gf::Task10p11ahPlanScore infeasible,recovery_missing,recovered,closer;
    infeasible.full_rows_feasible=false;
    infeasible.minimum_full_row_residual_mps2=-0.1;
    recovery_missing.full_rows_feasible=true;
    recovery_missing.terminal_recovered=false;
    recovery_missing.terminal_recovery_margin_mps2=-0.01;
    recovered.full_rows_feasible=true;
    recovered.terminal_recovered=true;
    recovered.cumulative_coverage_deviation_l2_mps2=2.0;
    closer=recovered;
    closer.cumulative_coverage_deviation_l2_mps2=1.0;
    CHECK(gf::task10p11ahBetterPlan(recovery_missing,infeasible));
    CHECK(gf::task10p11ahBetterPlan(recovered,recovery_missing));
    CHECK(gf::task10p11ahBetterPlan(closer,recovered));
    CHECK_FALSE(gf::task10p11ahBetterPlan(recovered,closer));
}

TEST_CASE("negative terminal optimizer outcomes retain certificate strength") {
    CHECK(gf::task10p11ahOptimizerClassificationName(
        gf::Task10p11ahOptimizerClassification::StrictlyInfeasible)==
        "strictly_infeasible");
    CHECK(gf::task10p11ahOptimizerClassificationName(
        gf::Task10p11ahOptimizerClassification::Undetermined)==
        "undetermined");
    CHECK(gf::task10p11ahOptimizerClassificationName(
        gf::Task10p11ahOptimizerClassification::LocalNoWitness)==
        "local_optimizer_no_witness");
}

TEST_CASE("legacy component prefix is rebuilt with native outside controls") {
    const auto snapshot=gf::readTask10p11vJson(
        evidence("gate2/derived-from-sparse-157.8.json"));
    const auto legacy=gf::readTask10p11vJson(
        std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ah-early-h2-component-recovery"/"development-attempts"/
        "legacy-component-prefix-plan.json");
    auto fixture=makeP3();
    gf::restoreTask10p11vRestartState(
        *fixture,snapshot.at("restart_checkpoint"));
    const auto x0=gf::task10p11ah_detail::controlsFromJson(
        legacy.at("x0").at("controls"),
        fixture->adapter.runtimeSnapshot().estimate.mobile_ids);
    const auto x1=gf::task10p11ah_detail::controlsFromJson(
        legacy.at("x1").at("controls"),
        fixture->adapter.runtimeSnapshot().estimate.mobile_ids);
    gf::Task10p11ahComponentPlan plan;
    plan.owner2_u0=x0.at(2); plan.owner9_u0=x0.at(9);
    plan.owner2_u1=x1.at(2); plan.owner9_u1=x1.at(9);
    const auto audit=gf::evaluateTask10p11ahTerminalRecoveryPlan(
        *fixture,std::nullopt,plan);
    REQUIRE(audit.valid);
    CHECK(audit.full_row_count_x0==1113);
    CHECK(audit.full_row_count_x1==1113);
    CHECK(audit.full_row_count_x2==1113);
    CHECK(audit.score.full_rows_feasible);
    CHECK_FALSE(audit.score.terminal_recovered);
    CHECK(audit.classification==
        gf::Task10p11ahOptimizerClassification::LocalNoWitness);
    CHECK(gf::task10p11ahOnlyComponentDiffers(
        audit.distributed_u0,audit.selected_u0,{2,9},1.0e-12));
    CHECK(gf::task10p11ahOnlyComponentDiffers(
        audit.distributed_u1,audit.selected_u1,{2,9},1.0e-12));
    CHECK(audit.terminal_native_successor_gamma_mps2<0.0);
}

TEST_CASE("deterministic continuous search respects frozen component bounds") {
    gf::Task10p11ahPatternSearchSettings settings;
    settings.step_schedule_mps2={1.0,0.5,0.25};
    settings.maximum_sweeps_per_step=2;
    settings.maximum_evaluations=80;
    settings.wall_clock_limit_s=5.0;
    gf::Task10p11ahComponentPlan start;
    const auto evaluator=[](const gf::Task10p11ahComponentPlan& plan) {
        gf::Task10p11ahPlanEvaluation value;
        value.valid=true;
        value.classification=
            gf::Task10p11ahOptimizerClassification::LocalNoWitness;
        const auto vector=gf::task10p11ahPlanVector(plan);
        value.score.full_rows_feasible=true;
        value.score.terminal_recovery_margin_mps2=
            1.0-(vector-Eigen::Matrix<double,8,1>::Constant(1.0)).squaredNorm();
        value.score.terminal_recovered=
            value.score.terminal_recovery_margin_mps2>=0.0;
        value.score.cumulative_coverage_deviation_l2_mps2=vector.norm();
        return value;
    };
    const auto result=gf::solveTask10p11ahTerminalRecovery(
        {start},evaluator,settings);
    REQUIRE(result.valid);
    CHECK(result.evaluations<=settings.maximum_evaluations);
    CHECK(gf::task10p11ahPlanVector(result.plan).cwiseAbs().maxCoeff()<=4.0);
    CHECK(result.evaluation.score.terminal_recovery_margin_mps2>
        evaluator(start).score.terminal_recovery_margin_mps2);
}

TEST_CASE("disabled early H2 decision preserves all distributed controls") {
    const auto snapshot=gf::readTask10p11vJson(
        evidence("gate2/derived-from-sparse-157.8.json"));
    const auto distributed=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
    const auto decision=gf::task10p11ahDecideEarlyH2(
        snapshot,distributed,false);
    REQUIRE(decision.valid);
    CHECK_FALSE(decision.triggered);
    CHECK_FALSE(decision.component_applied);
    CHECK(gf::task10p11zAppliedControlsMatch(
        distributed,decision.selected_controls,0.0));
}

TEST_CASE("early H2 entry is only the discrete feasible-to-lost boundary") {
    CHECK(gf::task10p11ahShouldEnter(true,false));
    CHECK_FALSE(gf::task10p11ahShouldEnter(false,false));
    CHECK_FALSE(gf::task10p11ahShouldEnter(true,true));
    CHECK_FALSE(gf::task10p11ahShouldEnter(false,true));
}

TEST_CASE("157.8 full witness has x0 x1 x2 full-row identity and terminal audit") {
    const auto snapshot=gf::readTask10p11vJson(
        evidence("gate2/derived-from-sparse-157.8.json"));
    const auto witness=gf::readTask10p11vJson(
        evidence("gate4/h2-witness-157.8.json"));
    const auto audit=gf::auditTask10p11ahExistingH2Witness(snapshot,witness);
    REQUIRE(audit.valid);
    CHECK(audit.x0_row_count==1113);
    CHECK(audit.x1_row_count==1113);
    CHECK(audit.x2_row_count==1113);
    CHECK(audit.x0_minimum_residual_mps2>=-1.0e-8);
    CHECK(audit.x1_minimum_residual_mps2>=-1.0e-8);
    CHECK(audit.x2_minimum_residual_mps2>=-1.0e-8);
    CHECK_FALSE(audit.terminal_h1_strictly_infeasible);
    CHECK(audit.terminal_classification==
        gf::Task10p11ahTerminalClassification::ComponentStillRequired);
}

TEST_CASE("component policy cannot change owners outside 2 and 9") {
    const auto snapshot=gf::readTask10p11vJson(
        evidence("gate2/derived-from-sparse-157.8.json"));
    const auto distributed=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
    auto candidate=distributed;
    candidate.at(2).x()+=0.1;
    candidate.at(9).y()-=0.1;
    CHECK(gf::task10p11ahOnlyComponentDiffers(
        distributed,candidate,{2,9},0.0));
    candidate.at(3).x()+=1.0e-6;
    CHECK_FALSE(gf::task10p11ahOnlyComponentDiffers(
        distributed,candidate,{2,9},0.0));
}

TEST_CASE("Task 10.11ah JSON uses null for unavailable finite diagnostics") {
    gf::Task10p11ahDecision decision;
    const auto encoded=gf::task10p11ahDecisionJson(decision);
    CHECK(encoded.at("current_h2_margin_mps2").is_null());
    CHECK(encoded.at("normal_next_h2_margin_mps2").is_null());
    CHECK(encoded.at("recursive_feasibility_claimed")==false);
}

TEST_CASE("157.8 restart preserves the exact pre-advance time boundary") {
    const auto snapshot=gf::readTask10p11vJson(
        evidence("gate2/derived-from-sparse-157.8.json"));
    auto fixture=makeP3();
    gf::restoreTask10p11vRestartState(
        *fixture,snapshot.at("restart_checkpoint"));
    const auto runtime=fixture->adapter.runtimeSnapshot();
    CHECK(runtime.runtime_s==doctest::Approx(157.8));
    CHECK(fixture->swarm.robots.front()->runtime==doctest::Approx(157.8));
    CHECK(fixture->topologyFrozen());
    CHECK(runtime.topology_token==
        snapshot.at("runtime").at("topology_token").get<std::uint64_t>());
}

TEST_CASE("disabled H2 wrapper preserves one native P3 transition exactly") {
    auto expected=makeP3();
    auto wrapped=makeP3();
    const auto expected_step=expected->controller.advance();
    const auto prepared=gf::task10p11zPrepareNativeBaseline(
        *wrapped,gf::GammaFeedbackSelectionMode::LeastIntervention,
        22.0,std::nullopt);
    REQUIRE(expected_step.step.advanced);
    REQUIRE(prepared.valid);
    REQUIRE(prepared.control.step.advanced);
    const auto disabled=gf::task10p11ahDecideEarlyH2(
        gf::makeTask10p11sSnapshot(
            wrapped->adapter.runtimeSnapshot(),
            wrapped->adapter.snapshotHardRowRequest(
                wrapped->adapter.runtimeSnapshot().estimate,
                wrapped->adapter.runtimeSnapshot().topology),
            prepared.nominal_controls,wrapped->adapter.config()),
        prepared.control.step.applied_controls,false);
    REQUIRE(disabled.valid);
    const auto wrapped_step=
        wrapped->controller.advanceWithDevelopmentControlOverride(
            [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
                const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
                const std::map<gf::NodeId,double>& yaw_rates) {
                CHECK(gf::task10p11zAppliedControlsMatch(
                    nominal,prepared.nominal_controls,1.0e-12));
                return wrapped->adapter.
                    stepWithDevelopmentFullPairCertifiedControls(
                        disabled.selected_controls,yaw_rates,
                        runtime.estimator_token,runtime.topology_token,
                        true,0.0);
            });
    REQUIRE(wrapped_step.step.advanced);
    CHECK(gf::task10p11zAppliedControlsMatch(
        expected_step.step.applied_controls,
        wrapped_step.step.applied_controls,1.0e-12));
    CHECK(expected_step.step.applied_yaw_rates_radps==
        wrapped_step.step.applied_yaw_rates_radps);
    CHECK(expected_step.step.minimum_hard_residual==
        doctest::Approx(wrapped_step.step.minimum_hard_residual));
    CHECK(expected_step.step.truth_coverage==
        doctest::Approx(wrapped_step.step.truth_coverage));
    const auto expected_restart=gf::captureTask10p11vRestartFields(*expected);
    const auto wrapped_restart=gf::captureTask10p11vRestartFields(*wrapped);
    CHECK(expected_restart.at("plant")==wrapped_restart.at("plant"));
    CHECK(expected_restart.at("adapter")==wrapped_restart.at("adapter"));
    CHECK(expected_restart.at("coverage")==wrapped_restart.at("coverage"));
    CHECK(expected_restart.at("controller")==wrapped_restart.at("controller"));
}

TEST_CASE("H2 evidence rejects nonfinite JSON and captures post-advance state") {
    const auto directory=std::filesystem::temp_directory_path()/
        ("task10p11ah-evidence-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK_THROWS(gf::writeTask10p11vJson(
        directory/"nonfinite.json",nlohmann::json{{"bad",
            std::numeric_limits<double>::infinity()}}));
    auto fixture=makeP3();
    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    const auto checkpoint=gf::makeTask10p11yCurrentPackedCheckpoint(
        *fixture,"task10p11ah_post_advance_fixture");
    CHECK(checkpoint.at("runtime").at("runtime_s").get<double>()==
        doctest::Approx(0.1));
    CHECK(checkpoint.at("restart_checkpoint").at("runtime_s").get<double>()==
        doctest::Approx(0.1));
    CHECK(gf::validateTask10p11sSnapshot(checkpoint).complete);
}
