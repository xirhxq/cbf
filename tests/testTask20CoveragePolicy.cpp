#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task20CoveragePolicy.hpp"

namespace {

gf::Task20CoverageRequest requestFor(std::size_t count) {
    gf::Task20CoverageRequest request;
    request.contract=gf::task20DagLatticeContract(
        gf::Task20LatticeMode::DualLadder);
    request.fixed_positions=gf::task10p11pStandardCoastalAnchors();
    const auto launch=gf::task10p11pStandardLaunchPositions();
    for (std::size_t index=0;index<launch.size();++index)
        request.agents.push_back({static_cast<gf::NodeId>(index+1),
            launch[index],{},M_PI/2.0,0.05});
    for (std::size_t index=0;index<count;++index)
        request.uncovered_cells.push_back({static_cast<int>(index),
            static_cast<int>(40+index),{5.0+10.0*index,405.0+10.0*index}});
    request.initial_distance_m.resize(90000);
    for (int x=0;x<300;++x) for (int y=0;y<300;++y)
        request.initial_distance_m[x*300+y]=10.0*y;
    return request;
}

}  // namespace

TEST_CASE("Task 20 uses one uniform real-ID rule for residual counts zero through three") {
    for (std::size_t count=0;count<=3;++count) {
        auto request=requestFor(count);
        request.policy=gf::Task20TargetPolicy::CoverageWavefront;
        const auto result=gf::allocateTask20Coverage(request);
        CAPTURE(count);
        if (count==0) {
            CHECK(result.complete);
            CHECK(result.targets.empty());
            continue;
        }
        REQUIRE(result.valid);
        std::set<std::string> active_ids;
        for (const auto& [unit,assignment]:result.assignments) {
            const std::string captured_unit=unit;
            CAPTURE(captured_unit);
            CHECK(assignment.task.x_index>=0);
            CHECK(assignment.task.y_index>=0);
            active_ids.insert(assignment.task.id());
        }
        CHECK(active_ids.size()==result.assignments.size());
        for (const auto& [owner,target]:result.targets) {
            const gf::NodeId captured_owner=owner;
            CAPTURE(captured_owner);
            CHECK(active_ids.count(target.id())==1);
        }
    }
}

TEST_CASE("Task 20 wavefront never skips the nearest incomplete distance band") {
    auto request=requestFor(0);
    request.policy=gf::Task20TargetPolicy::CoverageWavefront;
    request.uncovered_cells={{0,20,{5.0,205.0}},
                             {1,20,{15.0,205.0}},
                             {2,140,{25.0,1405.0}},
                             {3,299,{35.0,2995.0}}};
    request.wavefront_band_width_m=190.0;
    const auto result=gf::allocateTask20Coverage(request);
    REQUIRE(result.valid);
    CHECK(result.active_band==1);
    for (const auto& [unit,assignment]:result.assignments) {
        const std::string captured_unit=unit;
        CAPTURE(captured_unit);
        CHECK(assignment.task.y_index==20);
    }
}

TEST_CASE("Task 20 target selection and DAG-conditioned follower lifting are separate") {
    auto request=requestFor(3);
    request.policy=gf::Task20TargetPolicy::Cbf2026Voronoi;
    const auto result=gf::allocateTask20Coverage(request);
    REQUIRE(result.valid);
    REQUIRE(result.fronts.size()==result.assignments.size());
    const auto lifted=gf::task20LiftTargets(
        request.contract,request.fixed_positions,result.fronts);
    REQUIRE(lifted.valid);
    for (const auto& [owner,target]:result.targets)
        CHECK((target.center-lifted.targets.at(owner)).norm()<1.0e-12);
}
