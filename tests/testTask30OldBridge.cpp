#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task30 explicit canonical continuation preserves C5 exactly and fixes new endpoint") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto canonical=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    const auto scaled=gf::task20LiftTargets(old,fixed,gf::task30SimilarityBridgeFronts(old,fixed,gf::task26CompactFronts(old,fixed),1.25)).targets;
    const auto b=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
    const gf::Task29RoleCenterPath original(goal,canonical,b),candidate(goal,scaled,b);
    for(int tick=0;tick<=1800;++tick) {
        const double s=tick*.1/60;
        const auto legacy=original.evaluateContinuingFront(s),identity=original.evaluateContinuingFront(s,canonical);
        const auto q=candidate.evaluateContinuingFront(s,canonical);
        for(const auto& [id,p]:legacy) {
            CHECK((p.array()==identity.at(id).array()).all());
            if(s>=1)CHECK((p.array()==q.at(id).array()).all());
        }
    }
    for(const auto& [id,p]:b) {
        CHECK((candidate.evaluateContinuingFront(1,canonical).at(id).array()==p.array()).all());
        CHECK((candidate.evaluateContinuingFront(1-1e-8,canonical).at(id)-p).norm()<1e-3);
        CHECK((candidate.evaluateContinuingFront(1+1e-8,canonical).at(id)-p).norm()<1e-3);
        CHECK((candidate.evaluateContinuingFront(0,canonical).at(id).array()==scaled.at(id).array()).all());
    }
    auto bad=canonical;bad.erase(1);CHECK_THROWS(candidate.evaluateContinuingFront(2,bad));
    bad=canonical;bad.at(1).x()=std::numeric_limits<double>::infinity();CHECK_THROWS(candidate.evaluateContinuingFront(2,bad));
}

TEST_CASE("Task30 old-only option is default off and requires frozen C5") {
    auto scenario=gf::task10p11rFixedBaselineScenario();auto cfg=gf::task19ProductionAdapterConfig();
    CHECK(cfg.target_policy_task18_cbf2026_outer);
    cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    REQUIRE(adapter.initializeStageZero().initialized);
    const std::string action="pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront";
    gf::Task26ExternalReconstructor off(adapter,controller,action),explicit_off(adapter,controller,action,60,1,false);
    CHECK(off.telemetry()==explicit_off.telemetry());
    gf::Task26ExternalReconstructor on(adapter,controller,action,60,1.25,true);
    CHECK(on.telemetry().at("task30_old_bridge").at("new_scale")==1);
    CHECK_FALSE(on.telemetry().contains("task30_bridge"));
    CHECK_THROWS(gf::Task26ExternalReconstructor(adapter,controller,"pinball",60,1,true));
}
