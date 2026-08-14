#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/D1DevelopmentExperiment.hpp"

#include <fstream>
#include <set>

TEST_CASE("D1 frozen protocol expands to thirty-six paired cases") {
    const json raw = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) +
        "/config/grand_finale/d1_development_efficiency.json"));
    const gf::D1Protocol protocol = gf::parseD1Protocol(raw);
    const auto cases = gf::expandD1Cases(protocol);
    CHECK(cases.size() == 36);
    std::set<std::string> keys;
    std::map<std::string, std::set<std::string>> paired_digests;
    for (const auto& run : cases) {
        keys.insert(run.key());
        paired_digests[run.pairKey()].insert(run.shared_digest);
    }
    CHECK(keys.size() == 36);
    CHECK(paired_digests.size() == 12);
    for (const auto& [pair, digests] : paired_digests) {
        CAPTURE(pair);
        CHECK(digests.size() == 1);
    }
}

TEST_CASE("D1 protocol rejects a strategy-specific shared field") {
    json raw = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) +
        "/config/grand_finale/d1_development_efficiency.json"));
    raw["strategy_overrides"] = {
        {"greedy_certified", {{"sensor_radius_m", 4.0}}}};
    CHECK_THROWS_WITH_AS(
        gf::parseD1Protocol(raw),
        "D1 strategy overrides may only select topology strategy",
        std::invalid_argument);
}

TEST_CASE("D1 result keeps timeout and distinguishes true from proxy T95") {
    gf::D1Metrics metrics;
    metrics.observeCoverage(1.0, 0.94, 0.96);
    CHECK_FALSE(metrics.t95_true_s.has_value());
    REQUIRE(metrics.t95_proxy_s.has_value());
    CHECK(*metrics.t95_proxy_s == doctest::Approx(1.0));
    metrics.observeCoverage(1.1, 0.95, 0.97);
    REQUIRE(metrics.t95_true_s.has_value());
    CHECK(*metrics.t95_true_s == doctest::Approx(1.1));
    metrics.markTimeout();
    CHECK(metrics.outcome == gf::D1Outcome::Timeout);
    CHECK(metrics.denominator_included);
}

TEST_CASE("D1 greedy policy uses score then edge id and fixed never proposes") {
    const std::vector<gf::D1ReplacementCandidate> candidates{
        {{102, 2}, {1, 2}, 0.4},
        {{101, 2}, {1, 2}, 0.7},
        {{100, 2}, {1, 2}, 0.7}};
    CHECK_FALSE(gf::chooseD1Replacement(
        gf::D1TopologyStrategy::FixedDag, candidates).has_value());
    const auto greedy = gf::chooseD1Replacement(
        gf::D1TopologyStrategy::GreedyCertified, candidates);
    REQUIRE(greedy.has_value());
    CHECK(greedy->addition.id() == "100->2");
}

TEST_CASE("D1 tube exceedance is retained rather than relabeled safe") {
    gf::D1Metrics metrics;
    metrics.observeContainment(false, true);
    metrics.observeContainment(true, false);
    CHECK(metrics.position_tube_exceedance_steps == 1);
    CHECK(metrics.velocity_tube_exceedance_steps == 1);
    CHECK_FALSE(metrics.all_steps_within_development_tube);
}

TEST_CASE("D1 runner must drive RETREAT instead of braking forever") {
    CHECK(gf::d1PendingAction(gf::SupervisorMode::Retreat, false, 1) ==
          gf::D1PendingAction::BeginRetreat);
    CHECK(gf::d1PendingAction(gf::SupervisorMode::Union, true, 1) ==
          gf::D1PendingAction::FinishFreshTransition);
    CHECK(gf::d1PendingAction(gf::SupervisorMode::Hold, false, 0) ==
          gf::D1PendingAction::StepOnly);
}

TEST_CASE("D1 proposed and greedy share one fixed removal candidate pool") {
    const std::vector<gf::DirectedEdge> old{{100, 1}, {101, 1}, {100, 2}, {1, 2}};
    const std::vector<gf::D1ReplacementCandidate> candidates{
        {{102, 2}, {1, 2}, 0.7}, {{101, 2}, {1, 2}, 0.6}};
    const auto eligible = gf::d1ProposedEligibleEdges(old, candidates);
    CHECK(eligible.size() == 5);
    CHECK(std::none_of(eligible.begin(), eligible.end(),
        [](const auto& edge) { return edge.id() == "1->2"; }));
    auto invalid = candidates;
    invalid.back().removal = {100, 2};
    CHECK_THROWS_AS(gf::d1ProposedEligibleEdges(old, invalid),
                    std::invalid_argument);
}

TEST_CASE("D1 step failure taxonomy does not call every rejection a solver failure") {
    CHECK(gf::classifyD1StepFailure("hard_polytope_empty") ==
          gf::D1Outcome::ControlFailure);
    CHECK(gf::classifyD1StepFailure("osqp_failure") ==
          gf::D1Outcome::SolverFailure);
    CHECK(gf::classifyD1StepFailure("residual_failed") ==
          gf::D1Outcome::SafetyFailure);
}
