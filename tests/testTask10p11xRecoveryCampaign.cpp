#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11xRecoveryCampaign.hpp"

TEST_CASE("campaign protocol is finite and immutable") {
    const auto value = gf::task10p11xPreregisteredParameters();
    REQUIRE(value.predictive_tau_mps2.size() == 2);
    CHECK(value.predictive_tau_mps2.at(0) == doctest::Approx(16.0));
    CHECK(value.predictive_tau_mps2.at(1) == doctest::Approx(18.0));
    REQUIRE(value.collision_gains.has_value());
    CHECK(value.collision_gains->lambda1_per_s == doctest::Approx(0.125));
    CHECK(value.collision_gains->lambda2_per_s == doctest::Approx(0.5));
    CHECK(value.topology_addition.id() == "1->4");
    CHECK(value.topology_removal.id() == "2->4");
}

TEST_CASE("Stage A uses only the six frozen packed checkpoints") {
    const auto root = std::filesystem::path(PROJECT_ROOT).parent_path() /
        "docs/evidence/task10p11v-unique-recapture/checkpoints";
    const std::vector<std::filesystem::path> paths = {
        root / "checkpoint-000-t132.4-first_dynamic_intervention.json",
        root / "checkpoint-001-t132.5-dynamic_intervention.json",
        root / "checkpoint-002-t132.6-dynamic_intervention.json",
        root / "checkpoint-003-t132.7-dynamic_intervention.json",
        root / "checkpoint-004-t132.8-dynamic_intervention.json",
        root / "checkpoint-005-t132.9-fail_closed.json"};
    const auto result = gf::runTask10p11xStageAPreregistration(paths);
    CHECK(result.at("checkpoint_count") == 6);
    CHECK(result.at("trajectory_run_performed") == false);
    CHECK(result.at("config_digest") == 1217127288044531733ULL);
    CHECK(result.at("routes").contains("distributed_preventive"));
    CHECK(result.at("routes").contains("pair_2_4_component"));
    CHECK(result.at("routes").contains("parameter"));
    CHECK(result.at("routes").contains("dynamic_topology"));
    CHECK(result.at("routes").at("pair_2_4_component")
        .at("offline_gate_passed") == true);
    CHECK(result.at("claim_boundary").at("recursive_feasibility_claimed") ==
        false);
    CHECK(result.at("claim_boundary").at("parameter_scan_performed") == false);
}

TEST_CASE("132.8 component decision is atomically full-row applicable") {
    const auto path = std::filesystem::path(PROJECT_ROOT).parent_path() /
        "docs/evidence/task10p11v-unique-recapture/checkpoints/"
        "checkpoint-004-t132.8-dynamic_intervention.json";
    const auto snapshot = gf::readTask10p11vJson(path);
    const auto decision = gf::task10p11x_detail::decidePairComponentFallback(
        snapshot, false);
    INFO(decision.reason);
    REQUIRE(decision.valid);
    CHECK(decision.use_component);
    CHECK(decision.successor_audit.component);
    CHECK(decision.successor_audit.full_pair);
    const auto request = gf::task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto rows = gf::buildCanonicalHardRows(request);
    const auto audit = gf::auditDevelopmentFullPairCertifiedControls(
        rows, request.mobile_ids, request.acceleration_half_box, 1.0e-8,
        decision.controls);
    INFO(audit.reason);
    CHECK(audit.valid);
    CHECK(audit.full_row_count == 1113);
    CHECK(audit.minimum_residual_mps2 >= -1.0e-8);
}

TEST_CASE("restored 132.8 boundary advances one exact-ZOH component step") {
    const auto path = std::filesystem::path(PROJECT_ROOT).parent_path() /
        "docs/evidence/task10p11v-unique-recapture/checkpoints/"
        "checkpoint-004-t132.8-dynamic_intervention.json";
    const auto checkpoint = gf::readTask10p11vJson(path);
    auto fixture = gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention, 14.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::restoreTask10p11vRestartState(
        *fixture, checkpoint.at("restart_checkpoint"));
    std::optional<gf::task10p11x_detail::ComponentFallbackDecision> decision;
    const auto step = fixture->controller.advanceWithDevelopmentControlOverride(
        [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
            const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
            const std::map<gf::NodeId,double>& yaw_rates) {
            const auto snapshot = gf::makeTask10p11sSnapshot(runtime,
                fixture->adapter.snapshotHardRowRequest(
                    runtime.estimate, runtime.topology),
                nominal, fixture->adapter.config());
            decision = gf::task10p11x_detail::decidePairComponentFallback(
                snapshot, false);
            if (!decision->valid) {
                gf::GrandFinaleSwarmStep rejected;
                rejected.reason = decision->reason;
                return rejected;
            }
            return fixture->adapter.stepWithDevelopmentFullPairCertifiedControls(
                decision->controls, yaw_rates, runtime.estimator_token,
                runtime.topology_token, decision->successor_audit.full_pair,
                decision->successor_audit.full_residual);
        });
    REQUIRE(decision.has_value());
    INFO(step.reason);
    REQUIRE(step.step.advanced);
    CHECK(fixture->swarm.robots.front()->runtime ==
        doctest::Approx(132.9).epsilon(1.0e-10));
    CHECK(step.step.minimum_hard_residual >= -1.0e-8);
    CHECK(fixture->topologyFrozen());
}
