#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/CanonicalHocbfQpController.hpp"

#include <cmath>

namespace {

gf::CanonicalHardRowRequest request(
    const Eigen::Vector2d& velocity,double radius) {
    gf::CanonicalHardRowRequest value;
    value.mobile_ids={1};
    value.states[1]={{0.0,0.0},velocity,Eigen::Vector2d::Zero()};
    value.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    value.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    value.acceleration_half_box=4.0;
    value.speed_limit_mps=30.0;
    value.speed_cbf_gain=1.0;
    value.speed_snapshot_tubes[1]={0.0,radius,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    return value;
}

const gf::CanonicalHardRow& speedRow(
    const std::vector<gf::CanonicalHardRow>& rows) {
    const auto found=std::find_if(rows.begin(),rows.end(),[](const auto& row) {
        return row.kind==gf::CanonicalHardRowKind::SpeedLimit;
    });
    REQUIRE(found!=rows.end());
    return *found;
}

}

TEST_CASE("Robust speed row lower-bounds velocity-ball and input-box residual") {
    const Eigen::Vector2d estimate(20.0,0.0);
    const double radius=2.0;
    const auto rows=gf::buildCanonicalHardRows(request(estimate,radius));
    const auto& row=speedRow(rows);

    CHECK(row.id=="speed:1");
    CHECK(row.owner==1);
    CHECK_FALSE(row.peer.has_value());
    CHECK(row.responsibility==doctest::Approx(1.0));
    CHECK(row.control_coefficient.x()==doctest::Approx(-40.0));
    CHECK(row.control_coefficient.y()==doctest::Approx(0.0));
    CHECK(row.barrier_h==doctest::Approx(30.0*30.0-22.0*22.0));
    CHECK(row.velocity_uncertainty_reserve_mps==doctest::Approx(radius));

    for (int direction=0;direction<32;++direction) {
        const double angle=2.0*M_PI*static_cast<double>(direction)/32.0;
        const Eigen::Vector2d actual=estimate+radius*Eigen::Vector2d(
            std::cos(angle),std::sin(angle));
        for (double ax : {-4.0,4.0}) for (double ay : {-4.0,4.0}) {
            const Eigen::Vector2d acceleration(ax,ay);
            const double true_residual=-2.0*actual.dot(acceleration)+
                (30.0*30.0-actual.squaredNorm());
            CHECK(row.margin(acceleration)<=true_residual+1.0e-10);
        }
    }
}

TEST_CASE("Robust speed initial set fails when the velocity tube crosses limit") {
    const auto rows=gf::buildCanonicalHardRows(
        request(Eigen::Vector2d(29.0,0.0),2.0));
    const auto& row=speedRow(rows);
    CHECK(row.barrier_h<0.0);
    CHECK(row.participates_in_gamma);
    gf::CanonicalHocbfQpController controller;
    const auto solved=controller.solve({gf::SolverProfile::OpenSource,1,1,1,
        gf::SupervisorMode::Search,Eigen::Vector2d::Zero(),4.0,rows,1e-7});
    CHECK_FALSE(solved.control_available);
    CHECK(solved.failure_reason=="speed_initial_set_violated");
}

TEST_CASE("Speed row is owner-only and does not alter pair responsibility") {
    auto value=request(Eigen::Vector2d(5.0,0.0),0.2);
    value.mobile_ids={1,2};
    value.states[2]={{8.0,0.0},Eigen::Vector2d::Zero(),
                     Eigen::Vector2d::Zero()};
    value.collision_pairs={gf::UndirectedEdge::canonical(1,2)};
    value.collision_snapshot_tubes["1--2"]={0.1,0.1,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    value.speed_snapshot_tubes[2]={0.0,0.2,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    const auto rows=gf::buildCanonicalHardRows(value);
    std::size_t speed_count=0;
    std::size_t half_count=0;
    for (const auto& row : rows) {
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) {
            ++speed_count;
            CHECK(row.responsibility==doctest::Approx(1.0));
            CHECK_FALSE(row.peer.has_value());
        }
        if (row.kind==gf::CanonicalHardRowKind::Collision) {
            ++half_count;
            CHECK(row.responsibility==doctest::Approx(0.5));
        }
    }
    CHECK(speed_count==2);
    CHECK(half_count==2);
}
