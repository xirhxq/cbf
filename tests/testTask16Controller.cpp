#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task16ExperimentFixture.hpp"

TEST_CASE("Task 16 fixtures remove workspace rows without relaxing other hard rows") {
    for (const auto arm:{gf::Task16CoverageArm::HistoricalClipped,
                         gf::Task16CoverageArm::BoundaryDecoupled,
                         gf::Task16CoverageArm::FormationAware}) {
        auto fixture=gf::makeTask16ExperimentFixture(arm);
        CHECK(fixture->adapter.config().target_policy_task16_cbf2026);
        CHECK(fixture->adapter.config().task16_coverage_arm==arm);
        CHECK(fixture->adapter.config().boundary.policy==
            gf::BoundaryPolicy::None);
        REQUIRE(fixture->adapter.initializeStageZero().initialized);
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const auto rows=fixture->adapter.currentSnapshotHardRows(
            runtime.topology);
        std::size_t workspace=0,reference=0,collision=0,input=0;
        for (const auto& row:rows) {
            workspace+=row.kind==gf::CanonicalHardRowKind::Workspace;
            reference+=row.kind==gf::CanonicalHardRowKind::ReferenceDistance;
            collision+=row.kind==gf::CanonicalHardRowKind::Collision;
            input+=row.kind==gf::CanonicalHardRowKind::InputBox;
        }
        CHECK(workspace==0);
        CHECK(reference>0);
        CHECK(collision>0);
        CHECK(input>0);
    }
}

TEST_CASE("Task 16 Arm A emits a real-ID clipped ledger through the controller") {
    auto fixture=gf::makeTask16ExperimentFixture(
        gf::Task16CoverageArm::HistoricalClipped);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto step=fixture->controller.advance();
    REQUIRE(step.task16_allocation_evaluated);
    REQUIRE(step.task16_allocation.valid);
    CHECK_FALSE(step.task16_governor_evaluated);
    CHECK(step.committed_targets.size()==14);
    for (const auto& [owner,target]:step.committed_targets) {
        (void)owner;
        CHECK(target.x_index>=0);
        CHECK(target.y_index>=0);
        CHECK(target.center.x()>=0.0);
        CHECK(target.center.x()<=3000.0);
        CHECK(target.center.y()>=0.0);
        CHECK(target.center.y()<=3000.0);
    }
    CHECK(step.step.advanced);
    const auto second=fixture->controller.advance();
    CHECK_FALSE(second.task16_allocation_evaluated);
}

TEST_CASE("Task 16 Arms B and C persist real tasks and use common lambda") {
    for (const auto arm:{gf::Task16CoverageArm::BoundaryDecoupled,
                         gf::Task16CoverageArm::FormationAware}) {
        auto fixture=gf::makeTask16ExperimentFixture(arm);
        REQUIRE(fixture->adapter.initializeStageZero().initialized);
        const auto first=fixture->controller.advance();
        CAPTURE(static_cast<int>(arm));
        CAPTURE(first.reason);
        REQUIRE(first.task16_allocation_evaluated);
        REQUIRE(first.task16_allocation.valid);
        REQUIRE(first.task16_governor_evaluated);
        REQUIRE(first.step.advanced);
        CHECK(first.task16_common_fraction.size()==2);
        for (const auto& entry:first.task16_common_fraction) {
            CAPTURE(entry.first);
            const double fraction=entry.second;
            CHECK(fraction>=0.0);
            CHECK(fraction<=1.0);
        }
        for (const auto& [owner,target]:first.committed_targets) {
            (void)owner;
            CHECK(target.x_index>=0);
            CHECK(target.y_index>=0);
        }
        const auto epoch=first.target_epoch;
        const auto tasks=first.committed_targets;
        const auto second=fixture->controller.advance();
        REQUIRE(second.step.advanced);
        CHECK(second.target_epoch==epoch);
        for (const auto& [owner,target]:second.committed_targets)
            CHECK(target.id()==tasks.at(owner).id());
        if (arm==gf::Task16CoverageArm::FormationAware) {
            CHECK(first.task16_allocation.scanned_member_cell_pairs>0);
            CHECK(first.task16_allocation.allocation_wall_s>0.0);
        }
    }
}
