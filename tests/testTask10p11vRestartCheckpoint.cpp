#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11sFull28dGateA.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <filesystem>

TEST_CASE("Existing offline snapshot fails closed as a runtime restart") {
    const auto path=std::filesystem::path(PROJECT_ROOT).parent_path()/
        "docs/evidence/task10p11s-long-run/failure-snapshot.json";
    const auto snapshot=gf::readTask10p11sSnapshot(path);
    const auto audit=gf::auditTask10p11vRestartCheckpoint(snapshot);
    CHECK(audit.offline_oracle_complete);
    CHECK_FALSE(audit.deterministic_restart_complete);
    CHECK(audit.missing_fields.size()>=4);
}

TEST_CASE("Short fixture checkpoint survives an independent file round trip") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto before=fixture->adapter.runtimeSnapshot();
    const auto request=fixture->adapter.snapshotHardRowRequest(
        before.estimate,before.topology);
    const auto restart_fields=gf::captureTask10p11vRestartFields(*fixture);
    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);

    const auto base=gf::makeTask10p11sSnapshot(
        before,request,fixture->controller.lastNominalControls(),
        fixture->adapter.config());
    const auto checkpoint=gf::makeTask10p11vRestartCheckpoint(
        base,restart_fields,"short_fixture");
    const auto direct=gf::auditTask10p11vRestartCheckpoint(checkpoint);
    INFO(direct.reason);
    INFO(nlohmann::json(direct.missing_fields).dump());
    CHECK(direct.offline_oracle_complete);
    CHECK(direct.capture_fields_complete);
    CHECK(direct.deterministic_restart_complete);

    const auto path=std::filesystem::temp_directory_path()/
        "task10p11v-restart-checkpoint-fixture.json";
    gf::writeTask10p11sSnapshot(path,checkpoint);
    const auto loaded=gf::readTask10p11sSnapshot(path);
    const auto round_trip=gf::auditTask10p11vRestartCheckpoint(loaded);
    INFO(round_trip.reason);
    CHECK(round_trip.offline_oracle_complete);
    CHECK(round_trip.capture_fields_complete);
    CHECK(loaded.at("restart_checkpoint").at("plant").at("robots").size()==14);
    CHECK(loaded.at("restart_checkpoint").at("coverage").at(
        "cell_count").get<std::size_t>()==90000);
    CHECK(loaded.at("restart_checkpoint").at("controller").at(
        "target_epoch").is_number_unsigned());

    const auto gate=gf::runTask10p11sFull28dGateA(loaded);
    CHECK(gate.at("snapshot_complete").get<bool>());
    CHECK(gate.at("full_pair").at("feasible").get<bool>());
    CHECK(gate.at("successor").at("performed").get<bool>());
    CHECK(gate.at("successor").at("feasible").get<bool>());

    auto malformed=loaded;
    malformed["restart_checkpoint"]["plant"]["robots"][0]["state"]={0.0};
    CHECK_FALSE(gf::auditTask10p11vRestartCheckpoint(
        malformed).capture_fields_complete);
    std::filesystem::remove(path);
}

TEST_CASE("Sparse restart reproduces the uninterrupted next control boundary") {
    auto shadow=gf::makeTask10p11rFixedBaselineFixture();
    REQUIRE(shadow->adapter.initializeStageZero().initialized);
    REQUIRE(shadow->controller.advance().step.advanced);
    REQUIRE(shadow->controller.advance().step.advanced);

    const auto sparse=gf::makeTask10p11vSparseRestartCheckpoint(
        *shadow,std::nullopt,2);
    const auto path=std::filesystem::temp_directory_path()/
        "task10p11v-sparse-restart-fixture.json";
    gf::writeTask10p11vJson(path,sparse);
    const auto loaded=gf::readTask10p11vJson(path);
    const auto before=gf::task10p11vRestartStateJson(*shadow);
    const auto shadow_step=shadow->controller.advance();
    REQUIRE(shadow_step.step.advanced);
    const auto shadow_after=gf::task10p11vRestartStateJson(*shadow);

    auto restored=gf::makeTask10p11rFixedBaselineFixture();
    REQUIRE(restored->adapter.initializeStageZero().initialized);
    const auto metadata=gf::restoreTask10p11vSparseRestartCheckpoint(
        *restored,loaded);
    CHECK(metadata.cycle==2);
    CHECK_FALSE(metadata.active_pair.has_value());
    CHECK(gf::task10p11vRestartStateJson(*restored)==before);
    const auto restored_step=restored->controller.advance();
    REQUIRE(restored_step.step.advanced);
    CHECK(gf::task10p11vRestartStateJson(*restored)==shadow_after);
    CHECK(restored->controller.lastNominalControls()==
        shadow->controller.lastNominalControls());
    std::filesystem::remove(path);
}
