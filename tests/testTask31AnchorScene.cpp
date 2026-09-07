#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31AnchorScene.hpp"
#include "grand_finale/Task31InformationTelemetry.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("A six-anchor asset initializes the unchanged H0 stack with all physical range sources") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-design-r1.json");
    REQUIRE(f.good());nlohmann::json design;f>>design;
    auto input=design.at("chosen");input["width_m"]=4500.;input["height_m"]=2250.;input["mode_code"]=12;
    const auto scene=gf::task31AnchorScene(input);REQUIRE(scene.valid);CHECK(scene.fixed.size()==6);
    CHECK(scene.goal.valid);CHECK(scene.goal.reference_edges.size()==28);
    const auto old=gf::task25DagContractFromCode(0);
    const auto front=gf::task20LiftTargets(scene.goal,scene.fixed,{{"P",scene.frame_origin+Eigen::Vector2d(0,1000)}});
    REQUIRE(front.valid);
    for(const auto& [id,p]:front.targets) {
        const auto expected=input.at("final_positions").at(std::to_string(id));
        CHECK((p-Eigen::Vector2d(expected[0],expected[1])).norm()<1e-8);
    }
    const auto bridge=gf::task31CommonBridge(old,scene.goal,scene.fixed,scene.direction,scene.bridge_spacing_m,scene.ranking_span_m,scene.frame_origin);
    REQUIRE(bridge.valid);
    for(const auto& [id,p]:bridge.targets) {
        const auto expected=input.at("bridge_positions").at(std::to_string(id));
        CHECK((p-Eigen::Vector2d(expected[0],expected[1])).norm()<1e-8);
    }
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto cfg=gf::task19ProductionAdapterConfig();
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
    const auto init=adapter.initializeStageZero();INFO(init.reason);REQUIRE(init.initialized);
    const auto j=gf::task31InformationTelemetry(adapter);
    CHECK(j.at("accepted_batch").size()==175);CHECK(j.at("qualified_information").at("minimum_count")==19);
    CHECK(j.at("qualified_information").at("robust_fim_min").get<double>()>0);
    CHECK(adapter.runtimeSnapshot().topology==s.initial_topology);
    std::cout<<"TASK31_SIX_INITIAL "<<j.dump()<<'\n';
    auto bad=input;bad["physical_anchors"]["100"]={1801,-50};CHECK_FALSE(gf::task31AnchorScene(bad).valid);
    bad=input;bad["reference_edges"][0]={999,1};CHECK_FALSE(gf::task31AnchorScene(bad).valid);
}
