#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task25P0MultiDag.hpp"

namespace {

gf::Task20CoverageRequest requestFor(
    gf::Task25DagMode mode,std::size_t residual_count) {
    gf::Task20CoverageRequest request;
    request.contract=gf::task25DagContract(mode);
    request.policy=gf::Task20TargetPolicy::Cbf2026Voronoi;
    request.fixed_positions=gf::task10p11pStandardCoastalAnchors();
    const auto launch=gf::task10p11pStandardLaunchPositions();
    for (std::size_t index=0;index<launch.size();++index)
        request.agents.push_back({static_cast<gf::NodeId>(index+1),
            launch[index],{0.0,20.0},M_PI/2.0,0.05});
    for (std::size_t index=0;index<residual_count;++index)
        request.uncovered_cells.push_back({static_cast<int>(index),
            static_cast<int>(100+index),
            {5.0+10.0*index,1005.0+10.0*index}});
    return request;
}

}  // namespace

TEST_CASE("Task 25 freezes five complete and distinct DAG lifting contracts") {
    std::set<std::string> signatures;
    for (const auto mode:{gf::Task25DagMode::H0Origin,
                          gf::Task25DagMode::H0Microfix,
                          gf::Task25DagMode::CrossBracedDualFront,
                          gf::Task25DagMode::Pinball5432,
                          gf::Task25DagMode::LongTriangleSingleLadder,
                          gf::Task25DagMode::SplitThreeFront}) {
        const auto contract=gf::task25DagContract(mode);
        CAPTURE(contract.id);
        REQUIRE(contract.valid);
        CHECK(contract.reference_edges.size()==28);
        CHECK(contract.member_roles.size()==14);
        CHECK(contract.topological_order.size()==17);
        CHECK(signatures.insert(contract.id+":"+
            contract.structural_signature).second);
    }
}

TEST_CASE("Task 25 isolates DAG-only rewires under the exact H0 lifting") {
    const auto origin=gf::task25DagContract(gf::Task25DagMode::H0Origin);
    const auto microfix=gf::task25DagContract(gf::Task25DagMode::H0Microfix);
    const auto cross=gf::task25DagContract(
        gf::Task25DagMode::CrossBracedDualFront);
    REQUIRE(origin.valid);
    REQUIRE(microfix.valid);
    REQUIRE(cross.valid);
    CHECK(gf::task25CoverageRoleEquivalent(origin,microfix));
    CHECK(gf::task25CoverageRoleEquivalent(origin,cross));
    CHECK(gf::task25DagReplacementCount(origin,microfix)==2);
    CHECK(gf::task25DagReplacementCount(origin,cross)==2);
    const auto fixed=gf::task10p11pStandardCoastalAnchors();
    const std::map<std::string,Eigen::Vector2d> fronts{
        {"A",{700.0,1900.0}},{"B",{2300.0,1900.0}}};
    const auto a=gf::task20LiftTargets(origin,fixed,fronts);
    const auto b=gf::task20LiftTargets(microfix,fixed,fronts);
    const auto c=gf::task20LiftTargets(cross,fixed,fronts);
    REQUIRE(a.valid);
    REQUIRE(b.valid);
    REQUIRE(c.valid);
    for (gf::NodeId owner=1;owner<=14;++owner) {
        CHECK((a.targets.at(owner)-b.targets.at(owner)).norm()<1.0e-12);
        CHECK((a.targets.at(owner)-c.targets.at(owner)).norm()<1.0e-12);
    }
}

TEST_CASE("Task 25 P0 uses one real-ID rule for zero through three residuals") {
    for (const auto mode:{gf::Task25DagMode::H0Origin,
                          gf::Task25DagMode::Pinball5432,
                          gf::Task25DagMode::SplitThreeFront})
        for (std::size_t count=0;count<=3;++count) {
            const auto result=gf::allocateTask20Coverage(requestFor(mode,count));
            CAPTURE(static_cast<int>(mode));
            CAPTURE(count);
            if (count==0) {
                CHECK(result.valid);
                CHECK(result.complete);
                CHECK(result.targets.empty());
                continue;
            }
            REQUIRE(result.valid);
            std::set<std::string> ids;
            for (const auto& [unit,assignment]:result.assignments) {
                const std::string captured_unit=unit;
                CAPTURE(captured_unit);
                CHECK(assignment.task.x_index>=0);
                CHECK(assignment.task.y_index>=0);
                CHECK(ids.insert(assignment.task.id()).second);
            }
            for (const auto& [owner,target]:result.targets) {
                const gf::NodeId captured_owner=owner;
                CAPTURE(captured_owner);
                CHECK(ids.count(target.id())==1);
            }
        }
}

TEST_CASE("Task 25 single unit uses its complete front frame and global pool") {
    auto request=requestFor(gf::Task25DagMode::Pinball5432,2);
    // Terminal front members 13 and 14 straddle the map.  Their average
    // position and yaw define a northward focus; member 14 alone is east.
    for (auto& agent:request.agents) {
        if (agent.id==13) {
            agent.position={1000.0,1000.0};
            agent.velocity={0.0,20.0};
            agent.yaw_rad=M_PI/2.0;
        } else if (agent.id==14) {
            agent.position={2000.0,1000.0};
            agent.velocity={0.0,20.0};
            agent.yaw_rad=M_PI/2.0;
        }
    }
    request.uncovered_cells={{0,100,{1505.0,1405.0}},
                             {1,100,{1905.0,1005.0}}};
    const auto result=gf::allocateTask20Coverage(request);
    REQUIRE(result.valid);
    REQUIRE(result.assignments.size()==1);
    CHECK(result.assignments.at("P").task.id()=="0:100");
}

TEST_CASE("Task 25 H0 P0 is request-level equivalent to Task 18 allocation") {
    auto generic=requestFor(gf::Task25DagMode::H0Origin,0);
    generic.uncovered_cells={{0,100,{5.0,1005.0}},
                             {100,100,{1005.0,1005.0}},
                             {200,100,{2005.0,1005.0}},
                             {299,299,{2995.0,2995.0}}};
    // A non-velocity yaw is intentional: Task 18 forward focus is defined by
    // the current yaw, while actual-velocity yaw is the separate yaw law.
    for (auto& agent:generic.agents)
        if (agent.id==7) agent.yaw_rad=0.25;
        else if (agent.id==14) agent.yaw_rad=2.1;

    gf::Task16CoverageRequest historical;
    historical.arm=gf::Task16CoverageArm::BoundaryDecoupled;
    historical.agents=generic.agents;
    historical.uncovered_cells=generic.uncovered_cells;
    historical.fixed_positions=generic.fixed_positions;
    historical.search_min={0.0,0.0};
    historical.search_max={3000.0,3000.0};
    historical.config.forward_focus_distance_m=400.0;
    historical.global_pool_fallback_for_empty_share=true;

    const auto expected=gf::allocateTask16Cbf2026Coverage(historical);
    const auto actual=gf::allocateTask20Coverage(generic);
    REQUIRE(expected.valid);
    REQUIRE(actual.valid);
    REQUIRE(expected.assignments.size()==actual.assignments.size());
    for (const auto& [unit,assignment]:actual.assignments) {
        const auto& legacy=expected.assignments.at(unit);
        CHECK(assignment.task.id()==legacy.task.id());
        CHECK((assignment.front-legacy.endpoint).norm()<1.0e-12);
    }
    REQUIRE(expected.targets.size()==actual.targets.size());
    for (const auto& [owner,target]:actual.targets) {
        CHECK(target.id()==expected.targets.at(owner).id());
        CHECK((target.center-expected.targets.at(owner).center).norm()<1.0e-12);
    }
}

TEST_CASE("Task 25 tie breaking is deterministic across one two and three units") {
    for (const auto mode:{gf::Task25DagMode::Pinball5432,
                          gf::Task25DagMode::H0Origin,
                          gf::Task25DagMode::SplitThreeFront}) {
        auto request=requestFor(mode,0);
        request.uncovered_cells={{2,100,{1505.0,1005.0}},
                                 {1,100,{1505.0,1005.0}},
                                 {0,100,{1505.0,1005.0}}};
        const auto first=gf::allocateTask20Coverage(request);
        const auto second=gf::allocateTask20Coverage(request);
        REQUIRE(first.valid);
        REQUIRE(second.valid);
        CHECK(first.assignments.size()==second.assignments.size());
        for (const auto& [unit,assignment]:first.assignments)
            CHECK(assignment.task.id()==second.assignments.at(unit).task.id());
    }
}

TEST_CASE("Task 25 long triangle is represented exactly by the common affine lifting") {
    const auto contract=gf::task25DagContract(
        gf::Task25DagMode::LongTriangleSingleLadder);
    const auto fixed=gf::task10p11pStandardCoastalAnchors();
    const Eigen::Vector2d front{1500.0,2550.0};
    const auto lifted=gf::task20LiftTargets(contract,fixed,{{"T",front}});
    REQUIRE(lifted.valid);
    const Eigen::Vector2d base{1500.0,-50.0};
    const double h=(front-base).norm()/13.0;
    const double d=std::sqrt(3.0)*h/2.0;
    CHECK((lifted.targets.at(1)-Eigen::Vector2d{1500.0-d,-50.0+h}).norm()<1.0e-9);
    CHECK((lifted.targets.at(2)-Eigen::Vector2d{1500.0+d,-50.0+h}).norm()<1.0e-9);
    CHECK((lifted.targets.at(14)-Eigen::Vector2d{1500.0+d,2550.0}).norm()<1.0e-9);
}
