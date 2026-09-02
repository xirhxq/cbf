#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task17PeriodicCoveragePolicy.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

namespace {

gf::FrontierCell task17Cell(int x,int y,double cx,double cy) {
    return {x,y,{cx,cy}};
}

gf::Task17PeriodicCoverageRequest task17Request(gf::Task17PeriodicArm arm) {
    gf::Task17PeriodicCoverageRequest request;
    request.arm=arm;
    for (int id=1;id<=14;++id)
        request.agents.push_back({static_cast<gf::NodeId>(id),
            {id<=7?0.0:1000.0,0.0},{0.0,0.0},
            id<=7?0.0:M_PI,0.0});
    request.approved_successor_agents=request.agents;
    request.fixed_positions={{100,{1200.0,-50.0}},{101,{1500.0,-50.0}},
        {102,{1800.0,-50.0}}};
    request.search_min={0.0,0.0};
    request.search_max={3000.0,3000.0};
    return request;
}

}  // namespace

TEST_CASE("Task 17 periodic planner is an independent default-off path") {
    const gf::GrandFinaleSwarmAdapterConfig config;
    CHECK_FALSE(config.target_policy_task17_periodic);
    CHECK(config.task17_periodic_arm==gf::Task17PeriodicArm::Voronoi);
    CHECK(config.task17_update_period_cycles==5);
    CHECK(config.task17_common_governor_enabled);
    CHECK_FALSE(config.task17_reference_compatible_formation);
    CHECK_FALSE(config.task17_member_aware_wide_formation);
    CHECK_FALSE(config.task17_coherent_service_wide_formation);
}

TEST_CASE("Task 17 coherent service-wide selection uses the responsible member focus") {
    auto request=task17Request(gf::Task17PeriodicArm::CurrentMemberDistance);
    request.coherent_service_wide_formation=true;
    request.uncovered_cells={task17Cell(40,40,405.0,405.0),
        task17Cell(200,200,2005.0,2005.0)};
    for (auto& agent:request.agents) {
        agent.position={1500.0,100.0};
        agent.yaw_rad=0.0;
    }
    request.agents[2].position={5.0,405.0};
    request.agents[2].yaw_rad=0.0;
    const auto result=gf::allocateTask17PeriodicCoverage(request);
    REQUIRE(result.valid);
    const auto& assignment=result.assignments.at("A");
    CHECK(assignment.task.id()=="40:40");
    CHECK(assignment.responsible_member==3);
    CHECK((assignment.target_centers.at(3)-assignment.task.center).norm()
        ==doctest::Approx(0.0));
}

TEST_CASE("Task 17 policy suffixes map to exact update periods") {
    CHECK(gf::task17UpdatePeriodCycles("task17b")==5);
    CHECK(gf::task17UpdatePeriodCycles("task17b-p2")==2);
    CHECK(gf::task17UpdatePeriodCycles("task17b-p10")==10);
    CHECK(gf::task17UpdatePeriodCycles("task17b-p20")==20);
}

TEST_CASE("Task 17 member-aware wide inverse passes the selected member into geometry") {
    auto request=task17Request(gf::Task17PeriodicArm::Voronoi);
    request.member_aware_wide_formation=true;
    request.uncovered_cells={task17Cell(150,150,1505.0,1505.0)};
    for (auto& agent:request.agents)
        agent.position={agent.id<=7?200.0:2800.0,200.0};
    request.agents[2].position=request.uncovered_cells.front().center;
    const auto result=gf::allocateTask17PeriodicCoverage(request);
    REQUIRE(result.valid);
    const auto& assignment=result.assignments.at("A");
    CHECK(assignment.responsible_member==3);
    CHECK((assignment.target_centers.at(3)-assignment.task.center).norm()
        ==doctest::Approx(0.0));
    CHECK((assignment.target_centers.at(7)-assignment.task.center).norm()
        >1.0);
    for (const auto& target:result.targets) {
        CHECK(target.second.id()=="150:150");
    }
}

TEST_CASE("Task 17 reference-compatible arm makes a real member the exact cell witness") {
    auto request=task17Request(gf::Task17PeriodicArm::Voronoi);
    request.reference_compatible_formation=true;
    request.uncovered_cells={task17Cell(0,299,5.0,2995.0),
        task17Cell(299,299,2995.0,2995.0)};
    const auto result=gf::allocateTask17PeriodicCoverage(request);
    REQUIRE(result.valid);
    const auto squads=gf::task13UnifiedCoverageSquads();
    for (const auto& squad:squads) {
        const auto& assignment=result.assignments.at(squad.name);
        CHECK((assignment.target_centers.at(assignment.responsible_member)-
            assignment.task.center).norm()==doctest::Approx(0.0));
        double maximum_edge=0.0;
        double minimum_separation=std::numeric_limits<double>::infinity();
        for (const auto& edge:squad.edges) {
            const auto owner=assignment.target_centers.at(edge.owner);
            const auto fixed=request.fixed_positions.find(edge.reference);
            const auto reference=fixed==request.fixed_positions.end()
                ?assignment.target_centers.at(edge.reference):fixed->second;
            maximum_edge=std::max(maximum_edge,(owner-reference).norm());
        }
        for (std::size_t first=0;first<squad.members.size();++first) {
            const auto a=assignment.target_centers.at(squad.members[first]);
            for (std::size_t second=first+1;second<squad.members.size();++second)
                minimum_separation=std::min(minimum_separation,
                    (a-assignment.target_centers.at(
                        squad.members[second])).norm());
            for (const auto& [id,position]:request.fixed_positions) {
                (void)id;
                minimum_separation=std::min(minimum_separation,
                    (a-position).norm());
            }
        }
        CHECK(maximum_edge<850.0);
        CHECK(minimum_separation>10.0);
    }
}

TEST_CASE("Task 17 applies one uniform real-ID rule to zero through three residual cells") {
    for (int count=0;count<=3;++count) {
        auto request=task17Request(gf::Task17PeriodicArm::Voronoi);
        if (count>=1) request.uncovered_cells.push_back(
            task17Cell(0,0,100.0,0.0));
        if (count>=2) request.uncovered_cells.push_back(
            task17Cell(1,0,900.0,0.0));
        if (count>=3) request.uncovered_cells.push_back(
            task17Cell(2,0,500.0,100.0));
        const auto result=gf::allocateTask17PeriodicCoverage(request);
        CAPTURE(count);
        CAPTURE(result.reason);
        REQUIRE(result.valid);
        CHECK(result.complete==(count==0));
        if (count==0) {
            CHECK(result.assignments.empty());
            CHECK(result.targets.empty());
            continue;
        }
        REQUIRE(result.assignments.size()==2);
        REQUIRE(result.targets.size()==14);
        std::set<std::string> allowed;
        for (const auto& cell:request.uncovered_cells)
            allowed.insert(cell.id());
        for (const auto& [owner,target]:result.targets) {
            (void)owner;
            CHECK(allowed.count(target.id())==1);
            CHECK(target.x_index>=0);
            CHECK(target.y_index>=0);
        }
        const auto a=result.assignments.at("A").task.id();
        const auto b=result.assignments.at("B").task.id();
        CHECK((count==1 ? a==b : a!=b));
        const auto repeated=gf::allocateTask17PeriodicCoverage(request);
        CHECK(result.request_digest==repeated.request_digest);
        CHECK(result.assignments.at("A").task.id()==
            repeated.assignments.at("A").task.id());
        CHECK(result.assignments.at("B").task.id()==
            repeated.assignments.at("B").task.id());
    }
}

TEST_CASE("Task 17 current-member distance changes only the partition owner") {
    auto request=task17Request(gf::Task17PeriodicArm::CurrentMemberDistance);
    request.uncovered_cells={task17Cell(40,0,400.0,0.0),
        task17Cell(90,0,900.0,0.0)};
    // Leaders suggest A for x=400 and B for x=900, but a nonleader in B is
    // closest to x=400 and a nonleader in A is closest to x=900.
    request.agents[0].position={900.0,0.0};
    request.agents[7].position={400.0,0.0};
    const auto result=gf::allocateTask17PeriodicCoverage(request);
    REQUIRE(result.valid);
    CHECK(result.partition_owner.at("40:0")==14);
    CHECK(result.partition_owner.at("90:0")==7);
    CHECK(result.assignments.at("A").task.id()=="90:0");
    CHECK(result.assignments.at("B").task.id()=="40:0");
    CHECK(result.scanned_member_cell_pairs==28);
}

TEST_CASE("Task 17 successor service arm uses the approved one-step state only for partitioning") {
    auto request=task17Request(gf::Task17PeriodicArm::SuccessorServiceTime);
    request.uncovered_cells={task17Cell(4,4,40.0,40.0),
        task17Cell(96,4,960.0,40.0)};
    request.config.sensor_outer_radius_m=60.0;
    request.config.sensor_half_angle_rad=M_PI;
    for (auto& agent:request.agents)
        agent.position={agent.id<=7?900.0:100.0,40.0};
    for (auto& agent:request.approved_successor_agents)
        agent.position={agent.id<=7?950.0:50.0,40.0};
    const auto result=gf::allocateTask17PeriodicCoverage(request);
    REQUIRE(result.valid);
    CHECK(result.partition_owner.at("4:4")==14);
    CHECK(result.partition_owner.at("96:4")==7);
    CHECK(result.assignments.at("A").responsible_member==7);
    CHECK(result.assignments.at("B").responsible_member==14);
    CHECK((result.assignments.at("A").target_centers.at(7)-
        result.assignments.at("A").task.center).norm()==doctest::Approx(0.0));
    CHECK((result.assignments.at("B").target_centers.at(14)-
        result.assignments.at("B").task.center).norm()==doctest::Approx(0.0));
}
