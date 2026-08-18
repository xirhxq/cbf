#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/BoundarySoftNominalSelector.hpp"

namespace {

gf::BoundarySoftRowRequest rowRequest() {
    gf::BoundarySoftRowRequest request;
    request.mobile_ids={1};
    request.states.emplace(1,PairwiseSecondOrderState2D{
        Point(9.0,0.0),{2.0,0.0},Eigen::Vector2d::Zero()});
    request.facets={{"x-upper",{1.0,0.0},10.0}};
    request.snapshot_tubes.emplace(1,gf::SingleSnapshotTube2D{
        0.25,0.5,gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment});
    return request;
}

gf::BoundarySoftQpRequest qpRequest(gf::SolverProfile profile) {
    return {profile,1,Eigen::Vector2d::Zero(),0.4,10.0,
            {{"soft-workspace:1:x-upper",1,{1.0,0.0},-1.0,
              -1.0,-1.0,0.25,0.5}},1e-7};
}

}

TEST_CASE("Soft boundary row uses robust position and velocity support") {
    const auto rows=gf::buildBoundarySoftRows(rowRequest());
    REQUIRE(rows.size()==1);
    CHECK(rows[0].control_coefficient.isApprox(Eigen::Vector2d(-1.0,0.0)));
    CHECK(rows[0].barrier_h==doctest::Approx(0.75));
    CHECK(rows[0].barrier_psi1==doctest::Approx(-1.75));
    CHECK(rows[0].constant==doctest::Approx(-4.25));
    CHECK(rows[0].position_reserve_m==doctest::Approx(0.25));
    CHECK(rows[0].velocity_reserve_mps==doctest::Approx(0.5));
}

TEST_CASE("Soft boundary QP uses nonnegative slack without hard authorization") {
    gf::BoundarySoftNominalSelector selector;
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        const auto result=selector.solve(qpRequest(profile));
        INFO(result.failure_reason);
        REQUIRE(result.solver_succeeded);
        REQUIRE(result.nominal_available);
        CHECK(result.selected_nominal.x()==doctest::Approx(0.4).epsilon(1e-5));
        CHECK(result.selected_nominal.y()==doctest::Approx(0.0).epsilon(1e-7));
        REQUIRE(result.slacks.size()==1);
        CHECK(result.slacks[0]==doctest::Approx(0.6).epsilon(1e-5));
        CHECK(result.minimum_soft_residual>=-1e-7);
    }
}

TEST_CASE("Soft slack cannot alter an independent hard residual") {
    const gf::CanonicalHardRow hard{"hard-x",gf::CanonicalHardRowKind::Auxiliary,
        1,std::nullopt,{1.0,0.0},{1.0,0.0},0.0,1.0,true};
    gf::BoundarySoftNominalSelector selector;
    const auto selected=selector.solve(qpRequest(gf::SolverProfile::OpenSource));
    REQUIRE(selected.nominal_available);
    CHECK(hard.margin(selected.selected_nominal)==
          doctest::Approx(selected.selected_nominal.x()));
    CHECK(selected.slacks[0]>0.0);
}
