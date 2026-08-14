#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/TimeExpandedFiniteTourCertificate.hpp"

#include <Eigen/Dense>

#include <set>
#include <string>
#include <vector>

namespace {

gf::ShadowStateBox symmetricBox(double radius) {
    return {
        Eigen::VectorXd::Constant(4, -radius),
        Eigen::VectorXd::Constant(4, radius)};
}

gf::FiniteHorizonWitnessResult acceptedWitness(double duration) {
    return {true, "accepted", duration, 0.5, 2};
}

gf::FiniteTourHardAudit acceptedAudit() {
    return {true, true, true, true, true, true};
}

gf::FiniteTourRequest validTour() {
    gf::FiniteTourRequest request{
        {{0, symmetricBox(0.10)},
         {1, symmetricBox(0.20)},
         {2, symmetricBox(0.30)}},
        {{0, 1, acceptedWitness(1.0), symmetricBox(0.18),
          10, 10, acceptedAudit(), {"cell-a"}},
         {1, 2, acceptedWitness(1.5), symmetricBox(0.28),
          15, 15, acceptedAudit(), {"cell-b"}}},
        {"cell-a", "cell-b"},
        2.5};
    for (auto& edge : request.edges) {
        edge.blackout_contracts = {
            {"10--1", 0.1, 4, 0.1, 0.6, 1.0, 1.0, 0.9}};
        const std::size_t count = edge.expected_scalar_update_count;
        edge.shadow_cycles = {{
            Eigen::MatrixXd::Identity(4, 4),
            Eigen::MatrixXd::Zero(4, 1),
            Eigen::VectorXd::Zero(1), {}}};
        for (std::size_t index = 0; index < count; ++index) {
            edge.shadow_cycles.front().scalar_updates.push_back({
                "10--1",
                {Eigen::VectorXd::Constant(4, 0.001), 0.1}});
        }
    }
    return request;
}

}  // namespace

TEST_CASE("A finite time-expanded tour accepts monotone envelope growth without return equality") {
    const auto result = gf::verifyTimeExpandedFiniteTour(validTour());
    CHECK(result.valid);
    CHECK(result.reason == "accepted");
    CHECK(result.duration_upper_s == doctest::Approx(2.5));
    CHECK(result.certified_cells == std::set<std::string>{"cell-a", "cell-b"});
    CHECK(result.final_shadow_radius >= 0.28);
}

TEST_CASE("Stage order and full terminal inclusion are hard requirements") {
    auto repeated = validTour();
    repeated.nodes[2].stage = 1;
    CHECK(gf::verifyTimeExpandedFiniteTour(repeated).reason == "stage_order");

    auto terminal = validTour();
    terminal.nodes[1].shadow = symmetricBox(0.15);
    CHECK(gf::verifyTimeExpandedFiniteTour(terminal).reason ==
          "shadow_terminal_inclusion");

    auto runtime = validTour();
    runtime.edges[0].hard_audit.full_runtime_terminal = false;
    CHECK(gf::verifyTimeExpandedFiniteTour(runtime).reason ==
          "full_runtime_terminal");
}

TEST_CASE("Every sequential scalar slot requires a bound even when dropout is allowed") {
    auto missing = validTour();
    missing.edges[0].shadow_cycles.front().scalar_updates.pop_back();
    CHECK(gf::verifyTimeExpandedFiniteTour(missing).reason ==
          "scalar_update_bound_missing");
}

TEST_CASE("Hard gates witness duration and certified sensing cells fail closed") {
    auto hard = validTour();
    hard.edges[0].hard_audit.robust_hocbf = false;
    CHECK(gf::verifyTimeExpandedFiniteTour(hard).reason == "robust_hocbf");

    auto witness = validTour();
    witness.edges[0].witness.valid = false;
    witness.edges[0].witness.reason = "range_aoi_gap";
    CHECK(gf::verifyTimeExpandedFiniteTour(witness).reason ==
          "witness:range_aoi_gap");

    auto uncovered = validTour();
    uncovered.required_cells.insert("cell-c");
    CHECK(gf::verifyTimeExpandedFiniteTour(uncovered).reason ==
          "required_cell_uncovered");

    auto timeout = validTour();
    timeout.duration_upper_s = 2.4;
    CHECK(gf::verifyTimeExpandedFiniteTour(timeout).reason ==
          "tour_duration");
}

TEST_CASE("Finite edge-specific blackout contract is checked without word enumeration") {
    auto missing = validTour();
    missing.edges[0].blackout_contracts.clear();
    CHECK(gf::verifyTimeExpandedFiniteTour(missing).reason ==
          "blackout_contract_missing");

    auto stale = validTour();
    stale.edges[0].blackout_contracts.front().maximum_consecutive_dropouts = 5;
    CHECK(gf::verifyTimeExpandedFiniteTour(stale).reason ==
          "blackout_aoi_gap");

    auto quality = validTour();
    quality.edges[0].blackout_contracts.front().minimum_quality = 0.0;
    CHECK(gf::verifyTimeExpandedFiniteTour(quality).reason ==
          "blackout_measurement_contract");
}
