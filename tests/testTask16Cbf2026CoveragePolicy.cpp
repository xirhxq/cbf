#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task16Cbf2026CoveragePolicy.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

namespace {

gf::FrontierCell cell(int x,int y,double cx,double cy) {
    return {x,y,{cx,cy}};
}

std::vector<gf::Task16CoverageAgentState> agents() {
    std::vector<gf::Task16CoverageAgentState> value;
    for (int id=1;id<=14;++id)
        value.push_back({static_cast<gf::NodeId>(id),
            {id<=7?0.0:1000.0,0.0},{0.0,0.0},
            id<=7?0.0:M_PI,0.0});
    return value;
}

gf::Task16CoverageRequest request(gf::Task16CoverageArm arm) {
    gf::Task16CoverageRequest value;
    value.arm=arm;
    value.agents=agents();
    value.fixed_positions={{100,{1200.0,-50.0}},{101,{1500.0,-50.0}},
        {102,{1800.0,-50.0}}};
    value.search_min={0.0,0.0};
    value.search_max={3000.0,3000.0};
    return value;
}

}  // namespace

TEST_CASE("Task 16 public policy seam is available") {
    CHECK(gf::task16Cbf2026SourceCommit()==
        std::string("47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d"));
}

TEST_CASE("Task 16 research switch is default disabled") {
    const gf::GrandFinaleSwarmAdapterConfig config;
    CHECK_FALSE(config.target_policy_task16_cbf2026);
    CHECK(config.task16_coverage_arm==
        gf::Task16CoverageArm::HistoricalClipped);
    CHECK(config.task16_cvt_update_period_cycles==5);
    CHECK_FALSE(config.task16_tracking_envelope_enabled);
    CHECK(config.task16_reference_damping_reserve_multiples==
        doctest::Approx(1.0));
}

TEST_CASE("Task 16 forward formation matches worked CBF2026 geometry") {
    const auto squad=gf::task13UnifiedCoverageSquads()[0];
    const auto targets=gf::task16ForwardTargets(
        squad,{1500.0,-50.0},{1500.0,2950.0},std::nullopt);
    CHECK(targets.at(1).x()==doctest::Approx(1500.0));
    CHECK(targets.at(1).y()==doctest::Approx(700.0));
    CHECK(targets.at(2).x()==doctest::Approx(2149.519052838329));
    CHECK(targets.at(2).y()==doctest::Approx(1075.0));
    CHECK(targets.at(7).x()==doctest::Approx(1500.0));
    CHECK(targets.at(7).y()==doctest::Approx(2950.0));

    const auto outside=gf::task16ForwardTargets(squad,{1500.0,-50.0},
        {1500.0,3950.0},std::nullopt);
    const auto clipped=gf::task16ForwardTargets(squad,{1500.0,-50.0},
        {1500.0,3950.0},gf::Task16SearchBounds{{0.0,0.0},{3000.0,3000.0}});
    CHECK(outside.at(7).y()==doctest::Approx(3950.0));
    CHECK(clipped.at(7).y()==doctest::Approx(3000.0));
    CHECK(clipped.at(2).x()==doctest::Approx(2366.025403784439));
}

TEST_CASE("Task 16 Arms A and B reproduce end-robot Voronoi forward focus") {
    auto a=request(gf::Task16CoverageArm::HistoricalClipped);
    a.uncovered_cells={cell(0,0,100.0,0.0),cell(1,0,390.0,0.0),
        cell(2,0,610.0,0.0),cell(3,0,900.0,0.0)};
    auto b=a;
    b.arm=gf::Task16CoverageArm::BoundaryDecoupled;
    const auto ar=gf::allocateTask16Cbf2026Coverage(a);
    const auto br=gf::allocateTask16Cbf2026Coverage(b);
    REQUIRE(ar.valid);
    REQUIRE(br.valid);
    CHECK(ar.assignments.at("A").task.id()=="1:0");
    CHECK(ar.assignments.at("B").task.id()=="2:0");
    CHECK(br.assignments.at("A").task.id()=="1:0");
    CHECK(br.assignments.at("B").task.id()=="2:0");
    CHECK(ar.voronoi_owner.at("1:0")==7);
    CHECK(ar.voronoi_owner.at("2:0")==14);
    for (const auto& [owner,target]:br.targets)
        CHECK(target.id()==(owner<=7?"1:0":"2:0"));
}

TEST_CASE("Task 16 top-row tasks remain real under both clipping modes") {
    auto a=request(gf::Task16CoverageArm::HistoricalClipped);
    a.agents[6].position={1500.0,2500.0};
    a.agents[6].yaw_rad=M_PI/2.0;
    a.agents[13].position={2500.0,2500.0};
    a.agents[13].yaw_rad=M_PI/2.0;
    a.uncovered_cells={cell(0,299,5.0,2995.0),
        cell(299,299,2995.0,2995.0)};
    auto b=a;
    b.arm=gf::Task16CoverageArm::BoundaryDecoupled;
    const auto ar=gf::allocateTask16Cbf2026Coverage(a);
    const auto br=gf::allocateTask16Cbf2026Coverage(b);
    REQUIRE(ar.valid);
    REQUIRE(br.valid);
    for (const auto& [owner,target]:ar.targets) {
        (void)owner;
        CHECK(target.center.x()>=0.0);
        CHECK(target.center.x()<=3000.0);
        CHECK(target.center.y()>=0.0);
        CHECK(target.center.y()<=3000.0);
    }
    for (const auto& [owner,target]:br.targets)
        CHECK(target.id()==(owner<=7?"0:299":"299:299"));
    CHECK(std::max(br.assignments.at("A").task.y_index,
                   br.assignments.at("B").task.y_index)==299);
}

TEST_CASE("Task 16 Arm C assigns by current and approved one-step service") {
    auto c=request(gf::Task16CoverageArm::FormationAware);
    c.uncovered_cells={cell(4,4,40.0,40.0),cell(96,4,960.0,40.0)};
    c.config.sensor_outer_radius_m=60.0;
    c.config.sensor_half_angle_rad=M_PI;
    c.approved_successor_agents=c.agents;
    for (auto& agent:c.agents) {
        agent.position={agent.id<=7?900.0:100.0,40.0};
        agent.yaw_rad=0.0;
    }
    for (auto& agent:c.approved_successor_agents) {
        agent.position={agent.id<=7?950.0:50.0,40.0};
        agent.yaw_rad=0.0;
    }
    const auto result=gf::allocateTask16Cbf2026Coverage(c);
    REQUIRE(result.valid);
    CHECK(result.service_owner.at("4:4")==14);
    CHECK(result.service_owner.at("96:4")==7);
    CHECK(result.assignments.at("A").task.id()=="96:4");
    CHECK(result.assignments.at("B").task.id()=="4:4");
    CHECK(result.scanned_member_cell_pairs==28);
}

TEST_CASE("Task 16 deterministic ties use task then squad identity") {
    auto c=request(gf::Task16CoverageArm::FormationAware);
    c.uncovered_cells={cell(5,5,500.0,500.0)};
    c.approved_successor_agents=c.agents;
    for (auto& agent:c.agents) agent.position={500.0,500.0};
    for (auto& agent:c.approved_successor_agents)
        agent.position={500.0,500.0};
    c.config.sensor_half_angle_rad=M_PI;
    const auto first=gf::allocateTask16Cbf2026Coverage(c);
    const auto second=gf::allocateTask16Cbf2026Coverage(c);
    REQUIRE(first.valid);
    CHECK(first.service_owner.at("5:5")==7);
    CHECK(first.request_digest==second.request_digest);
    CHECK(first.assignments.size()==1);
    CHECK(first.assignments.begin()->first=="A");
}
