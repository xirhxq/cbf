#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/UnitScaleAudit.hpp"

#include <fstream>
#include "json.hpp"

TEST_CASE("Task 10.9 unit chain classifies inherited toy geometry") {
    const auto report = gf::auditUnitScale({
        40.0, 40.0, 2.0, 3.0, 850.0, 849.0, 0.1,
        0.4, 0.1, 1.0, true});
    CHECK(report.classification == gf::ScaleClassification::InheritedToyGeometry);
    CHECK(report.map_width_over_reference == doctest::Approx(40.0 / 850.0));
    CHECK(report.map_height_over_reference == doctest::Approx(40.0 / 850.0));
    CHECK(report.sensor_radius_over_grid == doctest::Approx(1.5));
    CHECK(report.acceleration_step_over_grid == doctest::Approx(0.002));
    CHECK(report.speed_step_over_grid == doctest::Approx(0.05));
    CHECK(report.canonical_row_units == "m/s^2");
}

TEST_CASE("Task 10.9 unit chain separates inactive valid scale from mismatch") {
    auto inactive = gf::auditUnitScale({
        80.0, 40.0, 2.0, 3.0, 850.0, 849.0, 0.1,
        0.4, 0.1, 1.0, false});
    CHECK(inactive.classification ==
          gf::ScaleClassification::ValidButReferenceInactive);

    gf::UnitScaleInputs invalid = {
        40.0, 40.0, 2.0, 3.0, 850.0, 849.0, 0.1,
        0.4, 0.1, 1.0, true};
    invalid.position_scale_to_metres = 0.001;
    CHECK_THROWS_WITH_AS(gf::auditUnitScale(invalid),
        "unit conversion changes approved distance semantics",
        std::invalid_argument);
}

TEST_CASE("Task 10.9 unit chain rejects scientifically incompatible values") {
    gf::UnitScaleInputs invalid = {
        40.0, 40.0, 2.0, 3.0, 850.0, 851.0, 0.1,
        0.4, 0.1, 1.0, false};
    CHECK_THROWS_AS(gf::auditUnitScale(invalid), std::invalid_argument);
}

TEST_CASE("Task 10.9 frozen mechanism config matches the independent fixtures") {
    const auto raw = nlohmann::json::parse(std::ifstream(
        std::string(PROJECT_ROOT) +
        "/config/grand_finale/task10p9_feasibility_gate.json"));
    CHECK(raw.at("easy_map_m") == nlohmann::json::array({20.0,20.0}));
    CHECK(raw.at("inactive_map_m") == nlohmann::json::array({80.0,40.0}));
    CHECK(raw.at("active_map_m") == nlohmann::json::array({1700.0,400.0}));
    CHECK(raw.at("active_fixed_x_m") ==
          nlohmann::json::array({0.0,800.0,1600.0}));
    CHECK(raw.at("active_overlap_slack_m").get<double>() ==
          doctest::Approx(99.0));
}
