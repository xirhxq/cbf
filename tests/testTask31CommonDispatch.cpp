#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task31 common bridge remains default off and starts without changing graph, estimator or real task ledger") {
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;
    s.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
    REQUIRE(controller.advance().step.advanced);
    const std::string action="pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront";
    gf::Task26ExternalReconstructor a(adapter,controller,action,.1,1,false,true);
    gf::Task26ExternalReconstructor b(adapter,controller,action,.1,1,false,true,false);
    CHECK(a.telemetry()==b.telemetry());
    CHECK_THROWS(gf::Task26ExternalReconstructor(adapter,controller,action,.1,1,false,false,true));
    gf::Task26ExternalReconstructor common(adapter,controller,action,.1,1,false,true,true);
    CHECK(common.telemetry().at("task31").at("bridge")=="union-depth-nearest-anchor-third");
    const auto before=adapter.runtimeSnapshot();const auto ledger=controller.committedTargets();
    const auto coverage=adapter.coverage().certifiedFraction();common.beforeStep();const auto after=adapter.runtimeSnapshot();
    CHECK(before.topology==after.topology);CHECK(before.estimator_token==after.estimator_token);
    CHECK((before.estimate.mean.array()==after.estimate.mean.array()).all());CHECK(coverage==adapter.coverage().certifiedFraction());
    const auto j=common.telemetry();
    for(const auto& [id,task]:ledger) {
        CHECK(controller.committedTargets().at(id).id()==task.id());
        const auto p=j.at("motion_reference").at(std::to_string(id));
        CHECK(std::hypot(p[0].get<double>()-task.center.x(),p[1].get<double>()-task.center.y())<1e-8);
    }
    const auto asset=j.at("task31").at("common_bridge");
    CHECK(asset.at("geometry_edges").size()==51);CHECK(asset.at("spacing_m")==150);
    CHECK(asset.at("continuation_basis")=="canonical_old_at_actual_admission_fraction");
}
