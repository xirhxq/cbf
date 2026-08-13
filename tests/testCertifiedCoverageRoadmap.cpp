#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedCoverageRoadmap.hpp"

#include <limits>

namespace {

gf::MacroStateCandidate macro(
    std::size_t id,
    std::set<int> footprint,
    gf::VersionSemantics version =
        gf::VersionSemantics::FreshnessAlphaInvariant) {
    return gf::MacroStateCandidate{
        id,
        {-1.0, -2.0, -0.1, -0.1},
        {1.0, 2.0, 0.1, 0.1},
        gf::MacroStateMargins{0.2, 0.3, 0.4, 0.1, 1.0e-3, 0.05, 0.25},
        2,
        true,
        true,
        std::move(footprint),
        version};
}

gf::PrimitiveCandidate primitive(
    std::size_t from,
    std::size_t to,
    double duration,
    gf::PolicyObservationScope scope =
        gf::PolicyObservationScope::EstimatorHistoryOnly) {
    return gf::PrimitiveCandidate{
        from,
        to,
        scope,
        duration,
        {gf::WitnessIntervalAudit{
            duration, 0.1, 0.2, 0.2, 0.3, 0.3,
            0.05, 1.0e-3, 0.05, 2}},
        gf::FiniteInformationContract{
            duration, 1.0, 0.25, {"10--1"},
            {{0.5 * duration, "10--1", 1.0, 0.9}}},
        0.02,
        1.0e-5,
        4,
        1};
}

}  // namespace

TEST_CASE("Macro-state verification requires quantitative strict margins and alpha-invariant freshness") {
    CHECK(gf::verifyMacroState(macro(0, {17, 18})).has_value());

    auto zero_margin = macro(0, {17});
    zero_margin.margins.hard_control = 0.0;
    CHECK_FALSE(gf::verifyMacroState(zero_margin).has_value());

    auto one_reference = macro(0, {17});
    one_reference.minimum_effective_reference_count = 1;
    CHECK_FALSE(gf::verifyMacroState(one_reference).has_value());

    auto empty_footprint = macro(0, {});
    CHECK_FALSE(gf::verifyMacroState(empty_footprint).has_value());

    auto inverted_box = macro(0, {17});
    inverted_box.lower[0] = 2.0;
    CHECK_FALSE(gf::verifyMacroState(inverted_box).has_value());

    CHECK_FALSE(gf::verifyMacroState(macro(
        0, {17}, gf::VersionSemantics::AbsoluteNumericValue)).has_value());

    auto nonfinite = macro(0, {17});
    nonfinite.margins.fim = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(gf::verifyMacroState(nonfinite).has_value());
}

TEST_CASE("Primitive verification rejects truth access missing refresh and non-finite completion") {
    CHECK(gf::verifyPrimitiveContract(primitive(0, 1, 1.0)).has_value());
    CHECK_FALSE(gf::verifyPrimitiveContract(primitive(
        0, 1, 1.0, gf::PolicyObservationScope::TruthState)).has_value());
    CHECK_FALSE(gf::verifyPrimitiveContract(primitive(
        0, 1, 1.0, gf::PolicyObservationScope::FutureMeasurement)).has_value());

    auto no_refresh = primitive(0, 1, 1.0);
    no_refresh.information.refreshes.clear();
    CHECK_FALSE(gf::verifyPrimitiveContract(no_refresh).has_value());

    auto stale_refresh = primitive(0, 1, 1.0);
    stale_refresh.information.refreshes.front().time_s = 1.1;
    CHECK_FALSE(gf::verifyPrimitiveContract(stale_refresh).has_value());

    auto negative_interval = primitive(0, 1, 1.0);
    negative_interval.intervals.front().reference_psi1_margin = -1.0e-3;
    CHECK_FALSE(gf::verifyPrimitiveContract(negative_interval).has_value());

    auto no_terminal_inclusion = primitive(0, 1, 1.0);
    no_terminal_inclusion.terminal_inclusion_margin = -1.0e-6;
    CHECK_FALSE(gf::verifyPrimitiveContract(no_terminal_inclusion).has_value());

    auto infinite = primitive(0, 1, 1.0);
    infinite.duration_upper_s = std::numeric_limits<double>::infinity();
    CHECK_FALSE(gf::verifyPrimitiveContract(infinite).has_value());
}

TEST_CASE("Verified core excludes one-way and invalid witness labels") {
    const auto m0 = gf::verifyMacroState(macro(0, {10}));
    const auto m1 = gf::verifyMacroState(macro(1, {11, 12}));
    const auto m2 = gf::verifyMacroState(macro(2, {13}));
    REQUIRE(m0.has_value());
    REQUIRE(m1.has_value());
    REQUIRE(m2.has_value());

    const auto e01 = gf::verifyPrimitiveContract(primitive(0, 1, 1.0));
    const auto e10 = gf::verifyPrimitiveContract(primitive(1, 0, 2.0));
    const auto e12 = gf::verifyPrimitiveContract(primitive(1, 2, 1.0));
    REQUIRE(e01.has_value());
    REQUIRE(e10.has_value());
    REQUIRE(e12.has_value());

    const auto core = gf::buildVerifiedCore(
        0, {*m0, *m1, *m2}, {*e01, *e10, *e12});
    CHECK(core.node_ids == std::set<std::size_t>{0, 1});
    CHECK(core.coverage_cells == std::set<int>{10, 11, 12});
    CHECK(core.edges.size() == 2);
}

TEST_CASE("Frozen tour services each footprint once and has a literal finite upper bound") {
    const auto m0 = gf::verifyMacroState(macro(0, {10}));
    const auto m1 = gf::verifyMacroState(macro(1, {11, 12}));
    const auto e01 = gf::verifyPrimitiveContract(primitive(0, 1, 1.0));
    const auto e10 = gf::verifyPrimitiveContract(primitive(1, 0, 2.0));
    REQUIRE(m0.has_value());
    REQUIRE(m1.has_value());
    REQUIRE(e01.has_value());
    REQUIRE(e10.has_value());
    const auto core = gf::buildVerifiedCore(0, {*m0, *m1}, {*e01, *e10});

    const auto tour = gf::buildFrozenCoverageTour(core, 0, 0.25);

    CHECK(tour.node_visit_order == std::vector<std::size_t>{0, 1, 0});
    CHECK(tour.serviced_cells == std::set<int>{10, 11, 12});
    CHECK(tour.duration_upper_s == doctest::Approx(3.5));
    CHECK(gf::finiteTourUpperBound(tour) == doctest::Approx(3.5));

    CHECK_FALSE(gf::tryBuildFrozenCoverageTour(core, 0, 0.0).has_value());
}
