#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"

#include <string>

namespace {

json stepDigest(const gf::SimpleCoverageControlStep& control) {
    const gf::GrandFinaleSwarmStep& step=control.step;
    json controls=json::object();
    for (const auto& [owner,control_value]:step.applied_controls)
        controls[std::to_string(owner)]={control_value.x(),control_value.y()};
    return {{"advanced",step.advanced},{"reason",step.reason},
        {"minimum_hard_residual",step.minimum_hard_residual},
        {"mode",static_cast<int>(step.mode)},{"controls",controls}};
}

}  // namespace

TEST_CASE("Task 11a freezes preregistration constants") {
    const auto constants=gf::task11aFrozenConstants();
    CHECK(constants.evaluation_period_ticks==50);
    CHECK(constants.minimum_dwell_ticks==10);
    CHECK(constants.proposals_enabled);
    CHECK(constants.minimum_indegree==2);
    CHECK(constants.maximum_indegree==2);
}

TEST_CASE("disabled proposals leave the fixed baseline bit-for-bit unchanged") {
    auto plain=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    auto wrapped=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(plain->adapter.initializeStageZero().initialized);
    REQUIRE(wrapped->adapter.initializeStageZero().initialized);
    auto constants=gf::task11aFrozenConstants();
    constants.proposals_enabled=false;
    gf::Task11aDynamicTopologyCoordinator coordinator(
        wrapped->swarm,wrapped->adapter,constants);
    for (std::size_t tick=0;tick<30;++tick) {
        const auto evaluation=coordinator.maybeEvaluateAndPropose(tick);
        if (tick%50==0) {
            CHECK(evaluation.kind=="disabled");
            CHECK(evaluation.outcome=="disabled");
        }
        const auto plain_step=plain->controller.advance();
        const auto wrapped_step=wrapped->controller.advance();
        REQUIRE(coordinator.recordCycle(wrapped_step)==false);
        const auto plain_digest=stepDigest(plain_step);
        const auto wrapped_digest=stepDigest(wrapped_step);
        REQUIRE(plain_digest==wrapped_digest);
        const auto plain_runtime=plain->adapter.runtimeSnapshot();
        const auto wrapped_runtime=wrapped->adapter.runtimeSnapshot();
        REQUIRE(plain_runtime.topology.size()==
            wrapped_runtime.topology.size());
        CHECK(plain_runtime.topology_token==wrapped_runtime.topology_token);
        CHECK(plain_runtime.mode==wrapped_runtime.mode);
        CHECK(plain->adapter.coverage().truthFraction()==
            wrapped->adapter.coverage().truthFraction());
    }
    CHECK(coordinator.attribution().evaluations==0);
    CHECK(wrapped->adapter.runtimeSnapshot().runtime_s==
        doctest::Approx(plain->adapter.runtimeSnapshot().runtime_s));
}

TEST_CASE("periodic evaluation holds the optimal topology without drift") {
    auto plain=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    auto wrapped=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(plain->adapter.initializeStageZero().initialized);
    REQUIRE(wrapped->adapter.initializeStageZero().initialized);
    auto constants=gf::task11aFrozenConstants();
    constants.proposals_enabled=true;
    gf::Task11aDynamicTopologyCoordinator coordinator(
        wrapped->swarm,wrapped->adapter,constants);
    // At stage zero the immediate trigger is not met.
    CHECK_FALSE(coordinator.immediateTriggerConditionMet());
    for (std::size_t tick=0;tick<51;++tick) {
        const auto evaluation=coordinator.maybeEvaluateAndPropose(tick);
        if (tick==0) {
            CHECK(evaluation.kind=="periodic");
            // The current assignment remains eligible-optimal: hold, and the
            // adapter is untouched.
            CHECK(evaluation.outcome=="no_change_optimal");
            CHECK_FALSE(wrapped->adapter.runtimeSnapshot().
                adapter_transition_pending);
        }
        const auto plain_step=plain->controller.advance();
        const auto wrapped_step=wrapped->controller.advance();
        coordinator.recordCycle(wrapped_step);
        REQUIRE(stepDigest(plain_step)==stepDigest(wrapped_step));
    }
    // Ticks 0 and 50 are both periodic evaluation points.
    CHECK(coordinator.attribution().evaluations==2);
    CHECK(coordinator.attribution().solver_optimal_no_change==2);
    CHECK(coordinator.attribution().certified_switches==0);
}

TEST_CASE("dwell and commit accounting attribute the frozen constants") {
    auto constants=gf::task11aFrozenConstants();
    constants.proposals_enabled=true;
    // Pure accounting probe: a coordinator over a live fixture, driven only
    // through the accounting paths (no controller advances).
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task11aDynamicTopologyCoordinator coordinator(
        fixture->swarm,fixture->adapter,constants);
    // A commit 5 ticks before the periodic evaluation point blocks it:
    // 50 - 45 = 5 < minimum_dwell_ticks.
    coordinator.noteCommitForTest(45);
    const auto blocked=coordinator.maybeEvaluateAndPropose(50);
    CHECK(blocked.outcome=="dwell_blocked");
    CHECK(coordinator.attribution().dwell_blocked_evaluations==1);
}
