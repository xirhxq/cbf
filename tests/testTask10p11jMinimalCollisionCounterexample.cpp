#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/Task10p11jCollisionCausalFixture.hpp"

namespace {

double pairGamma(const gf::CanonicalHardRowRequest& request) {
    const auto rows=gf::buildCanonicalHardRows(request);
    return std::min(
        gf::solveCanonicalGammaStar(rows,10,4.0).gamma,
        gf::solveCanonicalGammaStar(rows,11,4.0).gamma);
}

}

TEST_CASE("Task 10.11j frozen failure reduces to two mobile collision half rows") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    const auto request=gf::task10p11jMinimalPairRequest(replay.ticks.back());
    const auto rows=gf::buildCanonicalHardRows(request);
    CHECK(pairGamma(request)==doctest::Approx(-0.0454972).epsilon(1e-5));
    const auto audit=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(10,11));
    REQUIRE(audit.valid);
    CHECK(audit.snapshot_coherent);
    CHECK(audit.braking_slack_m==doctest::Approx(7.32975).epsilon(1e-5));
    CHECK(audit.input_box_relative_separation_support_mps2==
          doctest::Approx(8.07723).epsilon(1e-5));
    const auto first_gamma=gf::solveCanonicalGammaStar(rows,10,4.0);
    const auto second_gamma=gf::solveCanonicalGammaStar(rows,11,4.0);
    CHECK(audit.centralizedResidual(
        {first_gamma.accelX,first_gamma.accelY},
        {second_gamma.accelX,second_gamma.accelY})==
        doctest::Approx(2.0*first_gamma.gamma));
}

TEST_CASE("Task 10.11j constructs exact viable and infeasible gamma boundaries") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    double lower=-1.0;
    double upper=1.0;
    for (int iteration=0;iteration<80;++iteration) {
        const double middle=0.5*(lower+upper);
        if (pairGamma(gf::task10p11jMinimalPairRequest(
                replay.ticks.back(),middle))>=0.0)
            upper=middle;
        else
            lower=middle;
    }
    const double boundary=0.5*(lower+upper);
    const auto equality=gf::task10p11jMinimalPairRequest(
        replay.ticks.back(),boundary);
    const auto viable=gf::task10p11jMinimalPairRequest(
        replay.ticks.back(),boundary+1e-5);
    const auto infeasible=gf::task10p11jMinimalPairRequest(
        replay.ticks.back(),boundary-1e-5);
    CHECK(std::abs(pairGamma(equality))<=1e-10);
    CHECK(pairGamma(viable)>0.0);
    CHECK(pairGamma(infeasible)<0.0);
    MESSAGE("TASK10P11J_MINIMAL_BOUNDARY radial_velocity_delta=",boundary,
        " gamma_equal=",pairGamma(equality),
        " gamma_positive=",pairGamma(viable),
        " gamma_negative=",pairGamma(infeasible));
}

TEST_CASE("Task 10.11j endpoint separation does not certify transition viability") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    const auto& failed=replay.ticks.back();
    REQUIRE(failed.first.raw_target_present);
    REQUIRE(failed.second.raw_target_present);
    CHECK((failed.first.raw_target-failed.second.raw_target).norm()>0.1);
    CHECK_FALSE(failed.exact_owner10_feasible);
    CHECK_FALSE(failed.exact_owner11_feasible);
}

TEST_CASE("Task 10.11j separates covariance-tube reserve from mean continuity") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    const auto with_tube=gf::task10p11jMinimalPairRequest(replay.ticks.back());
    auto zero_tube=with_tube;
    zero_tube.collision_snapshot_tubes["10--11"]={0.0,0.0};
    const double robust_gamma=pairGamma(with_tube);
    const double zero_tube_gamma=pairGamma(zero_tube);
    CHECK(robust_gamma<0.0);
    CHECK(zero_tube_gamma>robust_gamma);
    CHECK(zero_tube_gamma>0.0);
    CHECK(zero_tube_gamma==doctest::Approx(0.335611).epsilon(1e-5));
    MESSAGE("TASK10P11J_TUBE_ABLATION robust_gamma=",robust_gamma,
        " zero_tube_gamma=",zero_tube_gamma,
        " claim=frozen_snapshot_diagnostic_only");
}

TEST_CASE("Task 10.11j both profiles fail at the shared exact precheck") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    const auto rows=gf::buildCanonicalHardRows(
        gf::task10p11jMinimalPairRequest(replay.ticks.back()));
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        const auto solved=gf::CanonicalHocbfQpController{}.solve({
            profile,10,6748,1,gf::SupervisorMode::Search,
            Eigen::Vector2d::Zero(),4.0,rows,1e-7});
        CHECK_FALSE(solved.hard_polytope_nonempty);
        CHECK_FALSE(solved.control_available);
        CHECK(solved.failure_reason=="hard_polytope_empty");
    }
}
