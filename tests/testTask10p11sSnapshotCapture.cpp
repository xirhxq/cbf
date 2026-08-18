#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11sSnapshotCapture.hpp"

#include <filesystem>

TEST_CASE("Minimal capture round-trips the actual 14-owner fixed fixture") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto before=fixture->adapter.runtimeSnapshot();
    const auto request=fixture->adapter.snapshotHardRowRequest(
        before.estimate,before.topology);
    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    REQUIRE(fixture->controller.lastNominalControls().size()==14);

    const auto snapshot=gf::makeTask10p11sSnapshot(
        before,request,fixture->controller.lastNominalControls(),
        fixture->adapter.config());
    const auto direct=gf::validateTask10p11sSnapshot(snapshot);
    INFO(direct.reason);
    CHECK(direct.complete);
    CHECK(direct.owner_count==14);
    CHECK(direct.row_count>0);
    CHECK(direct.rows_match);
    CHECK(direct.objective_matches);

    const auto path=std::filesystem::temp_directory_path()/
        "task10p11s-minimal-snapshot-fixture.json";
    gf::writeTask10p11sSnapshot(path,snapshot);
    const auto loaded=gf::readTask10p11sSnapshot(path);
    const auto round_trip=gf::validateTask10p11sSnapshot(loaded);
    INFO(round_trip.reason);
    CHECK(round_trip.complete);
    CHECK(round_trip.row_count==direct.row_count);
    CHECK(loaded.at("objective_28d").size()==28);
    CHECK(loaded.at("preflight").at("complete").get<bool>());

    const auto controls=fixture->controller.lastNominalControls();
    const auto predicted=gf::predictNoMeasurementSnapshot(
        before.estimate,controls,fixture->adapter.config().dt_s,
        fixture->adapter.config().estimator_acceleration_variance);
    const auto expected_successor=fixture->adapter.snapshotHardRowRequest(
        predicted,before.topology);
    const auto rebuilt_successor=
        gf::rebuildTask10p11sSuccessorRequest(loaded,controls);
    const auto expected_rows=gf::buildCanonicalHardRows(expected_successor);
    const auto rebuilt_rows=gf::buildCanonicalHardRows(rebuilt_successor);
    REQUIRE(rebuilt_rows.size()==expected_rows.size());
    for (std::size_t index=0;index<expected_rows.size();++index)
        CHECK(gf::task10p11s_capture_detail::sameRow(
            rebuilt_rows[index],
            gf::task10p11s_capture_detail::rowJson(expected_rows[index])));
    std::filesystem::remove(path);
}

TEST_CASE("Minimal capture fails closed when an owner or row is missing") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto before=fixture->adapter.runtimeSnapshot();
    const auto request=fixture->adapter.snapshotHardRowRequest(
        before.estimate,before.topology);
    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    auto snapshot=gf::makeTask10p11sSnapshot(
        before,request,fixture->controller.lastNominalControls(),
        fixture->adapter.config());

    snapshot["canonical_request"]["mobile_ids"].erase(
        snapshot["canonical_request"]["mobile_ids"].end()-1);
    CHECK_FALSE(gf::validateTask10p11sSnapshot(snapshot).complete);

    snapshot=gf::makeTask10p11sSnapshot(
        before,request,fixture->controller.lastNominalControls(),
        fixture->adapter.config());
    snapshot["actual_rows"].erase(snapshot["actual_rows"].end()-1);
    const auto missing_row=gf::validateTask10p11sSnapshot(snapshot);
    CHECK_FALSE(missing_row.complete);
    CHECK_FALSE(missing_row.rows_match);
}
