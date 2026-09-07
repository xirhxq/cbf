#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("Explicit fixed H0 observer keeps the launch graph mapping and estimator without an external request") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");REQUIRE(f.good());
    nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    cfg.distance_range_availability={true,131211};
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
    const auto before=adapter.runtimeSnapshot();
    gf::Task26ExternalReconstructor observer(adapter,controller,
        "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,false,scene,true,0);
    CHECK(adapter.runtimeSnapshot().estimator_token==before.estimator_token);
    CHECK(controller.targetEpoch()==0);
    for(int k=0;k<20;++k) {
        observer.beforeStep();REQUIRE(controller.advance().step.advanced);
        const auto t=observer.telemetry();CHECK(t["active_mode"]==0);CHECK(t["stage"]=="search");CHECK(t["request_count"]==0);
        CHECK(adapter.runtimeSnapshot().topology==before.topology);
        CHECK(controller.committedTargets().size()==14);
    }
    CHECK(observer.report()["planned_request_count"]==0);CHECK(observer.report()["requests"].empty());
}

TEST_CASE("A fixed control request cannot replace an initialized H0 graph") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");
    nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
    const auto before=adapter.runtimeSnapshot();
    CHECK_THROWS(gf::Task26ExternalReconstructor(adapter,controller,
        "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,false,scene,true));
    CHECK(adapter.runtimeSnapshot().topology==before.topology);
    CHECK(adapter.runtimeSnapshot().estimator_token==before.estimator_token);
    CHECK(controller.targetEpoch()==0);
    // Failed observer setup did not consume the registration or mutate state.
    CHECK_NOTHROW(gf::Task26ExternalReconstructor(adapter,controller,
        "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,false,scene));
}

TEST_CASE("Explicit fixed Pinball control starts in the real goal graph and never fabricates a request") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");
    REQUIRE(f.good());nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    for(int variant:{0,1,2}) {
        auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
        s.initial_topology=scene.goal.reference_edges;
        for(size_t k=0;k<s.mobile_positions.size();++k)
            s.mobile_positions[k]+=Eigen::Vector2d(750+variant*std::sin(k),variant*std::cos(k));
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;
        cfg.target_policy_task20_dag_lattice=true;cfg.task20_lattice_mode=0;
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:s.mobile_ids)settings["initial"]["velocity"]["values"].push_back(
            {variant*std::cos(id),variant*std::sin(id)});
        Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
        const auto initial=adapter.runtimeSnapshot();
        gf::Task26ExternalReconstructor observer(adapter,controller,
            "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",
            60,1,false,true,false,scene,true);
        CHECK(adapter.runtimeSnapshot().estimator_token==initial.estimator_token);
        const int ticks=variant==0?605:20;
        for(int tick=0;tick<ticks;++tick) {
            observer.beforeStep();const auto step=controller.advance();INFO(variant<<" "<<tick<<" "<<step.step.reason);
            REQUIRE(step.step.advanced);const auto state=observer.telemetry();
            CHECK(state["active_mode"]==12);CHECK(state["stage"]=="search");
            CHECK(state["request_count"]==0);CHECK(state["task_ledger_active"]==true);
            CHECK(gf::task25_detail::edgeSet(adapter.runtimeSnapshot().topology)==gf::task25_detail::edgeSet(scene.goal.reference_edges));
            REQUIRE(controller.committedTargets().size()==14);
            for(const auto& [id,cell]:controller.committedTargets()) {
                CHECK(cell.x_index>=0);CHECK(cell.y_index>=0);
            }
        }
        const auto report=observer.report();CHECK(report["requests"].empty());CHECK(report["events"].empty());
        CHECK(report["planned_request_count"]==0);CHECK(report["untriggered_request_count"]==0);
        std::cout<<"TASK31_FIXED_FIXTURE "<<nlohmann::json({{"variant",variant},{"safe_ticks",ticks},
            {"requests",0},{"mode",12},{"fixed_anchors",scene.fixed.size()}}).dump()<<'\n';
    }
}
