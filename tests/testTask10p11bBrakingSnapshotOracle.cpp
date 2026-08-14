#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/BrakingSnapshotOracle.hpp"

namespace {

gf::CanonicalHardRowRequest workspaceRequest(
    double distance_m,double outward_speed_mps,
    double position_radius_m=0.0,double velocity_radius_mps=0.0) {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={1};
    request.states[1]={{10.0-distance_m,0.0},{outward_speed_mps,0.0},{0.0,0.0}};
    request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box=0.4;
    request.require_snapshot_robust_rows=true;
    request.workspace_facets={{"x-upper",{1.0,0.0},10.0}};
    request.workspace_snapshot_tubes[1]={
        position_radius_m,velocity_radius_mps,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    return request;
}

const gf::BrakingSnapshotMetric& onlyMetric(
    const gf::BrakingSnapshotResult& result) {
    REQUIRE(result.metrics.size()==1);
    return result.metrics.front();
}

}

TEST_CASE("Task 10.11b workspace oracle derives literal stopping metrics") {
    const auto result=gf::evaluateBrakingSnapshot(
        workspaceRequest(10.0,1.0),0.5);
    REQUIRE(result.hard_polytope_nonempty);
    REQUIRE(result.snapshot_braking_admissible);
    CHECK_FALSE(result.horizon_sufficient);
    CHECK(result.reason=="braking_horizon_insufficient");
    const auto& metric=onlyMetric(result);
    CHECK(metric.source_id=="workspace:1:x-upper");
    CHECK(metric.robust_distance_m==doctest::Approx(10.0));
    CHECK(metric.robust_outward_speed_mps==doctest::Approx(1.0));
    CHECK(metric.available_inward_acceleration_mps2==doctest::Approx(0.4));
    CHECK(metric.stop_distance_m==doctest::Approx(1.25));
    CHECK(metric.stop_time_s==doctest::Approx(2.5));
    CHECK(metric.braking_slack_m==doctest::Approx(8.75));
}

TEST_CASE("Task 10.11b snapshot gate rejects negative stopping slack before hard rows empty") {
    const auto result=gf::evaluateBrakingSnapshot(
        workspaceRequest(4.0,2.0),10.0);
    CHECK(result.hard_polytope_nonempty);
    CHECK_FALSE(result.snapshot_braking_admissible);
    CHECK(result.reason=="snapshot_braking_inadmissible");
    CHECK(result.first_negative_source=="workspace:1:x-upper");
    const auto& metric=onlyMetric(result);
    CHECK(metric.stop_distance_m==doctest::Approx(5.0));
    CHECK(metric.braking_slack_m==doctest::Approx(-1.0));
}

TEST_CASE("Task 10.11b empty canonical polytope has precedence over braking classification") {
    const auto result=gf::evaluateBrakingSnapshot(
        workspaceRequest(1.0,1.0),10.0);
    CHECK_FALSE(result.hard_polytope_nonempty);
    CHECK_FALSE(result.snapshot_braking_admissible);
    CHECK(result.reason=="hard_polytope_empty");
}

TEST_CASE("Task 10.11b tube expansion can change positive braking slack to negative") {
    const auto low=gf::evaluateBrakingSnapshot(
        workspaceRequest(5.0,2.0),10.0);
    const auto high=gf::evaluateBrakingSnapshot(
        workspaceRequest(5.0,2.0,0.1,0.1),10.0);
    CHECK(onlyMetric(low).braking_slack_m==doctest::Approx(0.0));
    CHECK(onlyMetric(high).robust_distance_m==doctest::Approx(4.9));
    CHECK(onlyMetric(high).robust_outward_speed_mps==doctest::Approx(2.1));
    CHECK(onlyMetric(high).braking_slack_m<0.0);
}

TEST_CASE("Task 10.11b corner metrics do not masquerade as a joint invariant certificate") {
    const double radial=std::sqrt(3.2);
    const double pi=std::acos(-1.0);
    gf::CanonicalHardRowRequest request=workspaceRequest(4.0,radial);
    request.states[1].position={0.0,0.0};
    request.states[1].velocity={
        radial,2.0*(radial+std::cos(pi/6.0)*radial)};
    request.workspace_facets={
        {"first",{1.0,0.0},4.0},
        {"second",{-std::cos(pi/6.0),0.5},4.0}};
    const auto result=gf::evaluateBrakingSnapshot(request,10.0);
    REQUIRE(result.hard_polytope_nonempty);
    REQUIRE(result.metrics.size()==2);
    CHECK(result.metrics[0].braking_slack_m>=-1e-10);
    CHECK(result.metrics[1].braking_slack_m>=-1e-10);
    CHECK(result.snapshot_braking_admissible);
    CHECK(result.first_negative_source.empty());
}

TEST_CASE("Task 10.11b pair metrics retain fixed and shared responsibility") {
    gf::CanonicalHardRowRequest fixed;
    fixed.mobile_ids={1};
    fixed.fixed_ids={100};
    fixed.states[1]={{5.0,0.0},{-1.0,0.0},{0.0,0.0}};
    fixed.states[100]={{0.0,0.0},{0.0,0.0},{0.0,0.0}};
    fixed.collision_pairs={gf::UndirectedEdge::canonical(1,100)};
    fixed.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        10.0,0.0,1.0,1.0,1.0,0.0};
    fixed.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    fixed.acceleration_half_box=0.4;
    fixed.require_snapshot_robust_rows=true;
    fixed.collision_snapshot_tubes["1--100"]={0.0,0.0};
    const auto fixed_result=gf::evaluateBrakingSnapshot(fixed,10.0);
    const auto& fixed_metric=onlyMetric(fixed_result);
    CHECK(fixed_metric.robust_distance_m==doctest::Approx(4.0));
    CHECK(fixed_metric.robust_outward_speed_mps==doctest::Approx(1.0));
    CHECK(fixed_metric.available_inward_acceleration_mps2==doctest::Approx(0.4));

    auto shared=fixed;
    shared.mobile_ids={1,2};
    shared.fixed_ids.clear();
    shared.states[1]={{5.0,0.0},{-1.0,0.0},{0.0,0.0}};
    shared.states[2]={{-5.0,0.0},{1.0,0.0},{0.0,0.0}};
    shared.states.erase(100);
    shared.collision_pairs={gf::UndirectedEdge::canonical(1,2)};
    shared.collision_snapshot_tubes.clear();
    shared.collision_snapshot_tubes["1--2"]={0.0,0.0};
    const auto shared_result=gf::evaluateBrakingSnapshot(shared,10.0);
    const auto& shared_metric=onlyMetric(shared_result);
    CHECK(shared_metric.robust_distance_m==doctest::Approx(9.0));
    CHECK(shared_metric.robust_outward_speed_mps==doctest::Approx(2.0));
    CHECK(shared_metric.available_inward_acceleration_mps2==doctest::Approx(0.8));
    CHECK(shared_metric.stop_distance_m==doctest::Approx(2.5));
}

TEST_CASE("Task 10.11b other hard rows reduce available workspace braking support") {
    auto request=workspaceRequest(5.0,1.0);
    request.fixed_ids={100};
    request.states[100]={{0.0,0.0},{0.0,0.0},{0.0,0.0}};
    request.collision_pairs={gf::UndirectedEdge::canonical(1,100)};
    request.collision_spec.distanceLimit=4.9;
    request.collision_spec.totalReserve=2.0;
    request.collision_snapshot_tubes["1--100"]={0.0,0.0};
    const auto result=gf::evaluateBrakingSnapshot(request,10.0);
    const auto metric=std::find_if(
        result.metrics.begin(),result.metrics.end(),[](const auto& value) {
            return value.source_id=="workspace:1:x-upper";
        });
    REQUIRE(metric!=result.metrics.end());
    CHECK(metric->available_inward_acceleration_mps2==doctest::Approx(0.1));
    CHECK(metric->stop_distance_m==doctest::Approx(5.0));
    CHECK(metric->braking_slack_m==doctest::Approx(0.0));
}

TEST_CASE("Task 10.11b oracle rejects snapshots without an explicit robust tube contract") {
    auto request=workspaceRequest(10.0,1.0);
    request.require_snapshot_robust_rows=false;
    CHECK_THROWS_AS(
        gf::evaluateBrakingSnapshot(request,0.5),std::invalid_argument);
}
