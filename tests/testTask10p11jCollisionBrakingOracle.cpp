#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/BrakingSnapshotOracle.hpp"

namespace {

gf::CanonicalHardRowRequest pairRequest(
    double separation_m,double relative_closing_mps) {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={10,11};
    request.states[10]={{0.0,0.0},{0.5*relative_closing_mps,0.0},{0.0,0.0}};
    request.states[11]={{separation_m,0.0},{-0.5*relative_closing_mps,0.0},
                        {0.0,0.0}};
    request.collision_pairs={gf::UndirectedEdge::canonical(10,11)};
    request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        0.1,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box=4.0;
    request.require_snapshot_robust_rows=true;
    request.collision_snapshot_tubes["10--11"]={0.2,0.1};
    return request;
}

}

TEST_CASE("Task 10.11j pair oracle uses exact per-axis box direction support") {
    const auto request=pairRequest(10.0,4.0);
    const auto audit=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(10,11));
    REQUIRE(audit.valid);
    CHECK(audit.snapshot_coherent);
    CHECK(audit.first_owner==10);
    CHECK(audit.second_owner==11);
    CHECK(audit.robust_separation_margin_m==doctest::Approx(9.7));
    CHECK(audit.robust_radial_closing_speed_mps>
          doctest::Approx(4.0).epsilon(1e-12));
    // Pair direction is the x axis: two independent [-4,4]^2 boxes provide
    // exactly 8 m/s^2 relative separation support, not a Euclidean-ball value.
    CHECK(audit.input_box_relative_separation_support_mps2==
          doctest::Approx(8.0));
    CHECK(audit.full_hard_relative_separation_support_mps2<=8.0+1e-12);
    CHECK(audit.local_state_constants_coherent);
    CHECK(audit.local_coefficient_reserve_sum_mps2>=0.0);
}

TEST_CASE("Task 10.11j local half rows sum to the centralized robust pair row") {
    const auto request=pairRequest(10.0,4.0);
    const auto audit=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(10,11));
    REQUIRE(audit.valid);
    const Eigen::Vector2d first{-4.0,1.0};
    const Eigen::Vector2d second{4.0,-1.0};
    CHECK(audit.centralizedResidual(first,second)==doctest::Approx(
        audit.firstLocalResidual(first)+audit.secondLocalResidual(second)));
    CHECK(audit.centralizedResidual(first,second)>=-1e-10);
}

TEST_CASE("Task 10.11j rejects incoherent mixed-snapshot half rows") {
    const auto request=pairRequest(10.0,4.0);
    auto rows=gf::buildCanonicalHardRows(request);
    const auto second=std::find_if(rows.begin(),rows.end(),[](const auto& row) {
        return row.id=="collision:10--11:owner:11";
    });
    REQUIRE(second!=rows.end());
    second->barrier_hdot-=0.01;
    const auto audit=gf::evaluateMobilePairBrakingRequestRows(
        request,rows,gf::UndirectedEdge::canonical(10,11));
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason=="collision_half_row_snapshot_incoherent");
}

TEST_CASE("Task 10.11j exposes positive gamma with negative braking slack") {
    auto request=pairRequest(0.5,3.0);
    request.collision_snapshot_tubes["10--11"]={0.0,0.0};
    const auto rows=gf::buildCanonicalHardRows(request);
    const auto gamma=gf::solveCanonicalGammaStar(rows,10,4.0);
    const auto audit=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(10,11));
    REQUIRE(gamma.valid);
    CHECK(gamma.gamma>0.0);
    CHECK(audit.braking_slack_m<0.0);
}

TEST_CASE("Task 10.11j independent central oracle rejects equal local bias") {
    const auto request=pairRequest(10.0,4.0);
    auto rows=gf::buildCanonicalHardRows(request);
    for (auto& row:rows) {
        if (row.id=="collision:10--11:owner:10" ||
            row.id=="collision:10--11:owner:11") row.constant+=1.0;
    }
    const auto audit=gf::evaluateMobilePairBrakingRequestRows(
        request,rows,gf::UndirectedEdge::canonical(10,11));
    CHECK_FALSE(audit.valid);
    CHECK_FALSE(audit.independent_central_verified);
    CHECK(audit.reason=="collision_half_row_independent_central_mismatch");
}
