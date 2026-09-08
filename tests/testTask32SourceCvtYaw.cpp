#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32SourceCvtYaw.hpp"
#include <limits>
#include <Eigen/Geometry>
#include <gurobi_c++.h>
#include <iostream>

TEST_CASE("finite-slack source yaw objective retains estimated translational bearing drift") {
    const Eigen::Vector2d goal(100*std::cos(.5),100*std::sin(.5));
    const Eigen::Vector2d velocity(.3*goal.y(),-.3*goal.x());
    const auto r=gf::task32SourceCvtYaw({0,0},velocity,0,goal,1);
    REQUIRE(r.valid);CHECK(r.active);
    // Independently worked soft-QP boundary: drift .3 plus tan(.25).
    CHECK(r.rate==doctest::Approx(.5553419212210363).epsilon(1e-11));
    CHECK(r.slack==doctest::Approx(0.));
}

TEST_CASE("undefined bearing is inactive and malformed measured state fails closed") {
    const auto same=gf::task32SourceCvtYaw({4,7},{2,-3},1,{4,7},1);
    CHECK(same.valid);CHECK_FALSE(same.active);CHECK(same.rate==0.);CHECK(same.slack==0.);
    CHECK_FALSE(gf::task32SourceCvtYaw({0,0},{0,0},0,{1,0},-1).valid);
    CHECK_FALSE(gf::task32SourceCvtYaw({0,0},{0,0},0,{1,0},0).valid);
    const double nan=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(gf::task32SourceCvtYaw({nan,0},{0,0},0,{1,0},1).valid);
    CHECK_FALSE(gf::task32SourceCvtYaw({0,0},{nan,0},0,{1,0},1).valid);
    CHECK_FALSE(gf::task32SourceCvtYaw({0,0},{0,0},nan,{1,0},1).valid);
    CHECK_FALSE(gf::task32SourceCvtYaw({0,0},{0,0},0,{nan,0},1).valid);
    CHECK_FALSE(gf::task32SourceCvtYaw({0,0},{0,0},0,{1,0},nan).valid);
}

TEST_CASE("finite slack exposes antipodal stationary point and bounded-rate tradeoff") {
    const auto antipodal=gf::task32SourceCvtYaw({0,0},{0,0},0,{-100,0},1);
    REQUIRE(antipodal.valid);CHECK(std::abs(antipodal.rate)<1e-12);
    CHECK(antipodal.slack==doctest::Approx(1.));
    const auto capped=gf::task32SourceCvtYaw({0,0},{0,0},0,{0,100},.1);
    CHECK(capped.rate==doctest::Approx(.1));CHECK(capped.slack==doctest::Approx(.45));
    const auto ahead=gf::task32SourceCvtYaw({0,0},{4,-3},0,{100,0},1);
    CHECK(ahead.rate==0.);CHECK(ahead.slack==0.);
}

TEST_CASE("source yaw solution matches independent numerical soft QP and frame transforms") {
    GRBEnv env(true);env.set(GRB_IntParam_OutputFlag,0);env.start();
    const Eigen::Rotation2Dd rotation(.71);const Eigen::Vector2d shift(31,-12);
    double max_rate_error=0.,max_objective_error=0.;int count=0;
    for(double error:{-3.13,-2.5,-1.,-.1,0.,.1,1.,2.5,3.13})
        for(double drift:{-2.,-.3,0.,.3,2.})for(double cap:{.1,1.}) {
            const Eigen::Vector2d goal(100*std::cos(error),100*std::sin(error));
            const Eigen::Vector2d velocity(drift*goal.y(),-drift*goal.x());
            const auto r=gf::task32SourceCvtYaw({0,0},velocity,0,goal,cap);
            REQUIRE(r.valid);CHECK(r.active);CHECK(std::abs(r.rate)<=cap);
            CHECK(r.slack>=0.);CHECK(r.coefficient*r.rate+r.slack-r.rhs>=-1e-12);
            GRBModel qp(env);qp.set(GRB_DoubleParam_BarConvTol,1e-10);qp.set(GRB_IntParam_NumericFocus,3);
            const auto w=qp.addVar(-cap,cap,0,GRB_CONTINUOUS);
            const auto slack=qp.addVar(0,GRB_INFINITY,0,GRB_CONTINUOUS);
            // Independent row from h's finite-difference spatial/yaw gradients,
            // not the analytic solver's returned coefficient/rhs.
            const auto h=[&](Eigen::Vector2d x,double yaw) {
                const auto d=(goal-x).eval();return -.5*(1-std::cos(std::atan2(d.y(),d.x())-yaw));
            };
            constexpr double eps=1e-4;
            const double dx=(h({eps,0},0)-h({-eps,0},0))/(2*eps);
            const double dy=(h({0,eps},0)-h({0,-eps},0))/(2*eps);
            const double da=(h({0,0},eps)-h({0,0},-eps))/(2*eps);
            qp.addConstr(da*w+slack+dx*velocity.x()+dy*velocity.y()+h({0,0},0)>=0);
            qp.setObjective(w*w+10*slack,GRB_MINIMIZE);qp.optimize();
            REQUIRE(qp.get(GRB_IntAttr_Status)==GRB_OPTIMAL);
            const double we=std::abs(w.get(GRB_DoubleAttr_X)-r.rate);
            const double oe=std::abs(qp.get(GRB_DoubleAttr_ObjVal)-(r.rate*r.rate+10*r.slack));
            max_rate_error=std::max(max_rate_error,we);max_objective_error=std::max(max_objective_error,oe);
            CHECK(we<2e-4);CHECK(oe<2e-6);
            const auto transformed=gf::task32SourceCvtYaw(shift,rotation*velocity,.71,rotation*goal+shift,cap);
            CHECK(transformed.valid);CHECK(std::abs(transformed.rate-r.rate)<1e-12);
            CHECK(std::abs(transformed.slack-r.slack)<1e-12);
            const auto repeat=gf::task32SourceCvtYaw({0,0},velocity,0,goal,cap);
            CHECK(repeat.rate==r.rate);CHECK(repeat.slack==r.slack);++count;
        }
    std::cout<<"SOURCE_YAW_INDEPENDENT_QP cases="<<count<<" max_rate_error="<<max_rate_error
        <<" max_objective_error="<<max_objective_error<<'\n';
}
