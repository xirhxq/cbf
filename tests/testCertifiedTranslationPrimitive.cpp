#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedTranslationPrimitive.hpp"

#include <limits>

TEST_CASE("Exact ZOH interval propagation encloses hand-derived second-order motion") {
    const gf::SecondOrderBox2D initial{
        {{0.0, 1.0}, {0.0, 1.0}},
        {{1.0, 2.0}, {1.0, 2.0}}};
    const auto next = gf::propagateExactZoh(
        initial, {{2.0, 2.0}, {-1.0, -1.0}}, 0.5, 0.0);

    CHECK(next.position.x.lower <= 1.25);
    CHECK(next.position.x.upper >= 2.25);
    CHECK(next.position.y.lower <= 0.875);
    CHECK(next.position.y.upper >= 1.875);
    CHECK(next.velocity.x.lower <= 2.0);
    CHECK(next.velocity.x.upper >= 3.0);
    CHECK(next.velocity.y.lower <= 0.5);
    CHECK(next.velocity.y.upper >= 1.5);
}

TEST_CASE("Solver control error is included in the exact ZOH enclosure") {
    const gf::SecondOrderBox2D initial{
        {{0.0, 0.0}, {0.0, 0.0}},
        {{0.0, 0.0}, {0.0, 0.0}}};
    const auto exact = gf::propagateExactZoh(
        initial, {{1.0, 1.0}, {0.0, 0.0}}, 1.0, 0.0);
    const auto uncertain = gf::propagateExactZoh(
        initial, {{1.0, 1.0}, {0.0, 0.0}}, 1.0, 0.1);
    CHECK(exact.position.x.lower <= 0.5);
    CHECK(exact.position.x.upper >= 0.5);
    CHECK(uncertain.position.x.lower <= 0.45);
    CHECK(uncertain.position.x.upper >= 0.55);
    CHECK(uncertain.velocity.x.lower <= 0.9);
    CHECK(uncertain.velocity.x.upper >= 1.1);
}

TEST_CASE("Rest-to-rest bang-bang translation has finite literal displacement and reverse") {
    const auto forward = gf::makeRestToRestTranslation(
        {1.0, 0.0}, 0.4, 0.1, 5);
    REQUIRE(forward.has_value());
    CHECK(forward->phases.size() == 10);
    CHECK(forward->duration_s == doctest::Approx(1.0));
    CHECK(forward->displacement.x() == doctest::Approx(0.1));
    CHECK(forward->displacement.y() == doctest::Approx(0.0));

    const auto reverse = gf::reverseTranslation(*forward);
    CHECK(reverse.displacement.x() == doctest::Approx(-0.1));
    CHECK(reverse.duration_s == doctest::Approx(1.0));

    gf::SecondOrderBox2D state{
        {{0.0, 0.0}, {0.0, 0.0}},
        {{0.0, 0.0}, {0.0, 0.0}}};
    for (const auto& phase : forward->phases)
        state = gf::propagateExactZoh(state, phase.acceleration, 0.1, 0.0);
    CHECK(state.position.x.lower <= 0.1);
    CHECK(state.position.x.upper >= 0.1);
    CHECK(state.velocity.x.lower <= 0.0);
    CHECK(state.velocity.x.upper >= 0.0);
}

TEST_CASE("Translation primitive rejects invalid or saturated requests") {
    CHECK_FALSE(gf::makeRestToRestTranslation(
        {0.0, 0.0}, 0.4, 0.1, 5).has_value());
    CHECK_FALSE(gf::makeRestToRestTranslation(
        {1.0, 0.0}, 0.4, 0.1, 0).has_value());
    CHECK_FALSE(gf::makeRestToRestTranslation(
        {1.0, 0.0}, 0.0, 0.1, 5).has_value());
    CHECK_THROWS(gf::propagateExactZoh(
        {{{0.0, 0.0}, {0.0, 0.0}}, {{0.0, 0.0}, {0.0, 0.0}}},
        {{0.0, 0.0}, {0.0, 0.0}},
        std::numeric_limits<double>::quiet_NaN(), 0.0));
}
