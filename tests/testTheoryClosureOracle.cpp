#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/TheoryClosureOracle.hpp"

TEST_CASE("Only full-state certified terminal sets in the mutual initial class are retained") {
    const std::vector<gf::TheoryTerminalNode> terminals{
        {0, true, {0}},
        {1, true, {1, 2}},
        {2, true, {3}},
        {3, false, {4}},
    };
    const std::vector<gf::TheoryReachabilityEdge> witnesses{
        {0, 1}, {1, 0},
        {1, 2},
        {0, 3}, {3, 0},
    };

    const auto mutual = gf::finiteTerminalGraphMutualClass(0, terminals, witnesses);

    CHECK(mutual == std::set<std::size_t>{0, 1});
    CHECK(gf::finiteGraphCoverageLabels(0, terminals, witnesses) ==
          std::set<int>{0, 1, 2});
}

TEST_CASE("A completed finite-service epoch strictly decreases finite coverage rank") {
    CHECK(gf::finiteRankStrictlyDescends(
        {0, 1, 2}, {0}, {0, 2}, 2));
    CHECK_FALSE(gf::finiteRankStrictlyDescends(
        {0, 1, 2}, {0}, {0}, 0));
    CHECK_FALSE(gf::finiteRankStrictlyDescends(
        {0, 1, 2}, {0}, {0, 2}, 1));
}

TEST_CASE("Finite-service contract rejects starvation retreat and local-only evidence") {
    CHECK(gf::finiteServiceContractHolds({true, true, true, true}));
    CHECK_FALSE(gf::finiteServiceContractHolds({true, true, true, false}));
    CHECK_FALSE(gf::finiteServiceContractHolds({true, true, false, true}));
    CHECK_FALSE(gf::finiteServiceContractHolds({true, false, true, true}));

    CHECK(gf::intervalSafetyClaimSupported({true, true, true, true, true}));
    CHECK_FALSE(gf::intervalSafetyClaimSupported({true, false, true, true, true}));
    CHECK_FALSE(gf::intervalSafetyClaimSupported({true, true, true, false, true}));
    CHECK_FALSE(gf::intervalSafetyClaimSupported({false, true, true, true, true}));

    const gf::FiniteServiceContract missing_witness{true, false, false, true};
    CHECK_FALSE(gf::finiteSetTerminationPremisesHold(
        {true, true, missing_witness, true}));
    CHECK_FALSE(gf::finiteSetTerminationPremisesHold(
        {true, false, {true, true, true, true}, true}));
    CHECK_FALSE(gf::finiteSetTerminationPremisesHold(
        {true, true, {true, true, true, true}, false}));
}

TEST_CASE("Every supervisor and bookkeeping jump fits the uniform dwell bound") {
    const gf::HybridJumpAudit equality{
        1.0, 0.1, 0.1, 11, 4, 11, 2, 66};
    CHECK(gf::finiteJumpCountConsistent(equality));

    gf::HybridJumpAudit too_many_macros = equality;
    too_many_macros.external_episodes = 12;
    CHECK_FALSE(gf::finiteJumpCountConsistent(too_many_macros));

    gf::HybridJumpAudit unbounded_internal = equality;
    unbounded_internal.observed_atomic_jumps = 67;
    CHECK_FALSE(gf::finiteJumpCountConsistent(unbounded_internal));

    gf::HybridJumpAudit zero_dwell = equality;
    zero_dwell.minimum_dwell_s = 0.0;
    CHECK_FALSE(gf::finiteJumpCountConsistent(zero_dwell));

    gf::HybridJumpAudit too_many_samples = equality;
    too_many_samples.sampled_cycles = 12;
    CHECK_FALSE(gf::finiteJumpCountConsistent(too_many_samples));
}
