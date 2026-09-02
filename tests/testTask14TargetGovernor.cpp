#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task14TargetGovernor.hpp"

namespace {

std::vector<gf::WorkspaceFacet2D> box() {
    return {{"left",{-1.0,0.0},0.0},{"right",{1.0,0.0},100.0},
            {"bottom",{0.0,-1.0},0.0},{"top",{0.0,1.0},100.0}};
}

}  // namespace

TEST_CASE("Task 14 governor uses one braking-distance fraction") {
    const std::map<gf::NodeId,gf::Task14TargetGovernorState> states{
        {1,{{90.0,50.0},{4.0,0.0}}},{2,{{50.0,50.0},{0.0,0.0}}}};
    const std::map<gf::NodeId,Eigen::Vector2d> current{
        {1,{90.0,50.0}},{2,{50.0,50.0}}};
    const std::map<gf::NodeId,Eigen::Vector2d> desired{
        {1,{110.0,50.0}},{2,{70.0,50.0}}};
    const auto result=gf::task14AdvanceTargetGovernor(
        states,current,desired,box(),{0.1,4.0,1.0,0.0});
    REQUIRE(result.valid);
    // h=10, stopping distance=2, safe rate=8 m/s, hence beta=.08/2=.04.
    CHECK(result.minimum_stopping_margin_m==doctest::Approx(8.0));
    CHECK(result.common_fraction==doctest::Approx(0.04));
    CHECK(result.targets.at(1).x()==doctest::Approx(90.8));
    CHECK(result.targets.at(2).x()==doctest::Approx(50.8));
}

TEST_CASE("Task 14 governor freezes unsafe outward motion but permits inward") {
    const std::map<gf::NodeId,gf::Task14TargetGovernorState> states{
        {1,{{99.0,50.0},{4.0,0.0}}}};
    const std::map<gf::NodeId,Eigen::Vector2d> current{{1,{99.0,50.0}}};
    const std::vector<gf::WorkspaceFacet2D> right{
        {"right",{1.0,0.0},100.0}};
    auto result=gf::task14AdvanceTargetGovernor(states,current,
        {{1,{120.0,50.0}}},right,{0.1,4.0,1.0,0.0});
    REQUIRE(result.valid);
    CHECK(result.minimum_stopping_margin_m==doctest::Approx(-1.0));
    CHECK(result.common_fraction==0.0);
    result=gf::task14AdvanceTargetGovernor(states,current,
        {{1,{80.0,50.0}}},right,{0.1,4.0,1.0,0.0});
    REQUIRE(result.valid);
    CHECK(result.common_fraction==1.0);
    CHECK(result.targets.at(1).x()==doctest::Approx(80.0));
}
