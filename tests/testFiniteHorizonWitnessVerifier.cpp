#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/FiniteHorizonWitnessVerifier.hpp"

namespace {

gf::PairIntervalAuditRequest pairRequest(
    PairwiseSecondOrderBarrierKind kind,
    double first_x,
    double second_x,
    double uncertainty = 0.1) {
    return {
        {{{first_x, first_x}, {0.0, 0.0}},
         {{0.0, 0.0}, {0.0, 0.0}}},
        {{{second_x, second_x}, {0.0, 0.0}},
         {{0.0, 0.0}, {0.0, 0.0}}},
        {kind, kind == PairwiseSecondOrderBarrierKind::CommunicationUpper
                   ? 10.0 : 1.0,
         uncertainty, 1.0, 1.0, 1.0, 0.0},
        0.0};
}

gf::FiniteWitnessInterval validInterval() {
    return {0.5, {pairRequest(
                     PairwiseSecondOrderBarrierKind::CommunicationUpper,
                     0.0, 5.0),
                 pairRequest(
                     PairwiseSecondOrderBarrierKind::CollisionLower,
                     0.0, 5.0)},
            0.1, 1.0e-3, 0.1, 2};
}

gf::FiniteHorizonWitnessRequest validWitness() {
    return {1.0,
            {validInterval(), validInterval()},
            {1.0, 0.6, 0.1, {"10--1"}, {{0.5, "10--1", 1.0, 0.9}}},
            0.01,
            1.0e-4,
            true};
}

}  // namespace

TEST_CASE("Pair interval audit proves robust communication and collision initial sets") {
    const auto communication = gf::auditPairInterval(pairRequest(
        PairwiseSecondOrderBarrierKind::CommunicationUpper, 0.0, 5.0));
    CHECK(communication.valid);
    CHECK(communication.minimum_h >= 4.8);
    CHECK(communication.minimum_psi1 >= 4.8);

    const auto collision = gf::auditPairInterval(pairRequest(
        PairwiseSecondOrderBarrierKind::CollisionLower, 0.0, 5.0));
    CHECK(collision.valid);
    CHECK(collision.minimum_h >= 3.8);

    const auto too_far = gf::auditPairInterval(pairRequest(
        PairwiseSecondOrderBarrierKind::CommunicationUpper, 0.0, 10.0));
    CHECK_FALSE(too_far.valid);
    CHECK(too_far.reason == "reference_initial_set");

    const auto too_close = gf::auditPairInterval(pairRequest(
        PairwiseSecondOrderBarrierKind::CollisionLower, 0.0, 1.0));
    CHECK_FALSE(too_close.valid);
    CHECK(too_close.reason == "collision_initial_set");
}

TEST_CASE("Interval audit rejects endpoint-only reasoning when the enclosure crosses a hard boundary") {
    auto crosses = pairRequest(
        PairwiseSecondOrderBarrierKind::CommunicationUpper, 0.0, 5.0);
    crosses.second.position.x = {5.0, 11.0};
    const auto audit = gf::auditPairInterval(crosses);
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason == "reference_initial_set");
}

TEST_CASE("Finite witness accepts an explicit bounded refresh word and quantitative intervals") {
    const auto result = gf::verifyFiniteHorizonWitness(validWitness());
    CHECK(result.valid);
    CHECK(result.duration_upper_s == doctest::Approx(1.0));
    CHECK(result.minimum_effective_reference_count == 2);
    CHECK(result.minimum_pair_margin > 0.0);
}

TEST_CASE("Finite witness rejects missing refresh stale AoI and unverified actual QP") {
    auto missing = validWitness();
    missing.information.refreshes.clear();
    CHECK(gf::verifyFiniteHorizonWitness(missing).reason ==
          "range_refresh_missing");

    auto stale = validWitness();
    stale.information.initial_aoi_upper_s = 0.7;
    CHECK(gf::verifyFiniteHorizonWitness(stale).reason == "initial_aoi");

    auto unverified_qp = validWitness();
    unverified_qp.nominal_hard_margin = 1.0e-5;
    unverified_qp.solver_control_error = 1.0e-4;
    CHECK(gf::verifyFiniteHorizonWitness(unverified_qp).reason ==
          "qp_solution_map_unverified");

    auto weak_refs = validWitness();
    weak_refs.intervals.front().minimum_effective_reference_count = 1;
    CHECK(gf::verifyFiniteHorizonWitness(weak_refs).reason ==
          "effective_reference_count");
}
