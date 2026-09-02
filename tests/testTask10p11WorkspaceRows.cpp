#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/ProgressCompatibility.hpp"

namespace {

gf::CanonicalHardRowRequest requestAt(
    const Eigen::Vector2d& position,const Eigen::Vector2d& velocity) {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids = {1};
    request.states[1] = {Point(position.x(),position.y()),velocity,
                         Eigen::Vector2d::Zero()};
    request.reference_spec = {
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec = {
        PairwiseSecondOrderBarrierKind::CollisionLower,
        0.1,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box = 0.4;
    request.require_snapshot_robust_rows = true;
    request.workspace_facets = {
        {"x-upper",Eigen::Vector2d::UnitX(),10.0}};
    request.workspace_snapshot_tubes[1] = {0.5,0.2};
    return request;
}

const gf::CanonicalHardRow& workspaceRow(
    const std::vector<gf::CanonicalHardRow>& rows) {
    return *std::find_if(rows.begin(),rows.end(),[](const auto& row) {
        return row.kind == gf::CanonicalHardRowKind::Workspace;
    });
}

}

TEST_CASE("Task 10.11 workspace row robustly protects a convex facet") {
    const auto row = workspaceRow(gf::buildCanonicalHardRows(
        requestAt({8.0,0.0},{0.3,0.0})));
    CHECK(row.id == "workspace:1:x-upper");
    CHECK(row.control_coefficient.x() == doctest::Approx(-1.0));
    CHECK(row.control_coefficient.y() == doctest::Approx(0.0));
    CHECK(row.barrier_h == doctest::Approx(1.5));
    CHECK(row.barrier_hdot == doctest::Approx(-0.5));
    CHECK(row.barrier_psi1 == doctest::Approx(1.0));
    CHECK(row.constant == doctest::Approx(0.5));
    CHECK(row.position_uncertainty_reserve_m == doctest::Approx(0.5));
    CHECK(row.velocity_uncertainty_reserve_mps == doctest::Approx(0.2));
    CHECK(row.coefficient_uncertainty_reserve == doctest::Approx(0.0));
}

TEST_CASE("Task 10.11 workspace row exposes equality and input conflict") {
    auto equality = requestAt({9.5,0.0},{0.0,0.0});
    const auto equality_row = workspaceRow(
        gf::buildCanonicalHardRows(equality));
    CHECK(equality_row.barrier_h == doctest::Approx(0.0));
    CHECK(equality_row.barrier_psi1 == doctest::Approx(-0.2));

    auto impossible = requestAt({9.4,0.0},{0.5,0.0});
    const auto rows = gf::buildCanonicalHardRows(impossible);
    const auto exact = gf::evaluateProgressCompatibility(
        rows,1,Eigen::Vector2d::Zero(),0.4,
        {100.0,0.0,1e-10,true});
    CHECK_FALSE(exact.polytope_nonempty);
    CHECK(exact.reason == "hard_polytope_empty");
}

TEST_CASE("Task 14 workspace linear class-K gains are explicit") {
    auto request=requestAt({8.0,0.0},{0.3,0.0});
    request.workspace_class_k=gf::WorkspaceClassK::Linear;
    request.workspace_alpha1_gain=0.5;
    request.workspace_alpha2_gain=2.0;
    const auto row=workspaceRow(gf::buildCanonicalHardRows(request));
    // h=1.5, hdot=-0.5: k1*hdot+k2*(hdot+k1*h)=0.25.
    CHECK(row.barrier_psi1==doctest::Approx(0.25));
    CHECK(row.constant==doctest::Approx(0.25));
}

TEST_CASE("Task 14 regularized braking class-K is zero at the boundary") {
    CHECK(gf::workspaceBrakingAlpha1(0.0,4.0,1.0)==
        doctest::Approx(0.0));
    auto request=requestAt({8.0,0.0},{0.3,0.0});
    request.workspace_class_k=gf::WorkspaceClassK::RegularizedBraking;
    request.workspace_braking_acceleration_mps2=4.0;
    request.workspace_braking_regularization_m=1.0;
    request.workspace_alpha2_gain=1.5;
    const auto row=workspaceRow(gf::buildCanonicalHardRows(request));
    const double alpha1=std::sqrt(2.0*4.0*(1.5+1.0))-
        std::sqrt(2.0*4.0*1.0);
    const double derivative=4.0/std::sqrt(2.0*4.0*(1.5+1.0));
    CHECK(row.barrier_psi1==doctest::Approx(-0.5+alpha1));
    CHECK(row.constant==doctest::Approx(
        derivative*(-0.5)+1.5*(-0.5+alpha1)));
}
