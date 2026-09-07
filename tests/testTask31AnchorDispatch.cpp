#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31AnchorScene.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("A frozen six-anchor paired mode is registered before motion but cannot bypass actual graph handoff") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-design-r1.json");REQUIRE(f.good());
    nlohmann::json j;f>>j;j=j.at("chosen");j["width_m"]=4500.;j["height_m"]=2250.;j["mode_code"]=12;
    const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;
    cfg.target_policy_task20_dag_lattice=true;cfg.task20_lattice_mode=0;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    REQUIRE(adapter.initializeStageZero().initialized);
    controller.registerExternalCoverageContract(12,scene.goal);
    CHECK_THROWS(controller.commitExternalCoverageMode(12,scene.goal));
    REQUIRE(controller.advance().step.advanced);const auto ledger=controller.committedTargets();
    const auto before=adapter.runtimeSnapshot();
    CHECK_THROWS(controller.registerExternalCoverageContract(12,scene.goal));
    CHECK(adapter.runtimeSnapshot().topology==before.topology);
    CHECK(adapter.runtimeSnapshot().estimator_token==before.estimator_token);
    for(const auto& [id,c]:ledger)CHECK(controller.committedTargets().at(id).id()==c.id());
}

TEST_CASE("Reconstruction accepts an immutable physical-anchor paired asset only at startup") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-design-r1.json");nlohmann::json j;f>>j;
    j=j.at("chosen");j["width_m"]=4500.;j["height_m"]=2250.;j["mode_code"]=12;const auto scene=gf::task31AnchorScene(j);
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor reconstruct(adapter,controller,
        "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,true,scene);
    const auto before=adapter.runtimeSnapshot();reconstruct.beforeStep();
    CHECK(adapter.runtimeSnapshot().estimator_token==before.estimator_token);
    CHECK(adapter.runtimeSnapshot().topology==before.topology);
}
