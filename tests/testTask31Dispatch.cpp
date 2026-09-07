#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task31TriangularLattice.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task31 paired mapping defaults off and cannot precede actual graph handoff") {
    auto s=gf::task10p11rFixedBaselineScenario();auto c=gf::task19ProductionAdapterConfig();
    CHECK(c.target_policy_task18_cbf2026_outer);
    c.target_policy_task18_cbf2026_outer=false;c.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,c);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    REQUIRE(adapter.initializeStageZero().initialized);
    const std::string action="pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront";
    gf::Task26ExternalReconstructor a(adapter,controller,action),b(adapter,controller,action,60,1,false,false);
    CHECK(a.telemetry()==b.telemetry());
    gf::Task26ExternalReconstructor research(adapter,controller,action,60,1,false,true);
    CHECK(research.telemetry().at("task31").at("triangular_final")==true);
    REQUIRE(research.telemetry().at("task31").contains("current_information"));
    CHECK(research.telemetry().at("task31").at("current_information").at("accepted_batch").size()==133);
    CHECK_FALSE(a.telemetry().contains("task31"));
    CHECK_THROWS(gf::Task26ExternalReconstructor(adapter,controller,"pinball",60,1,false,true));
    const auto goal=gf::task31TriangularLattice(gf::task25DagContractFromCode(12),s.fixed_positions,{0,1});
    REQUIRE(goal.valid);
    CHECK_THROWS(controller.commitExternalCoverageMode(12,goal.contract));
}

TEST_CASE("Task31 complete labelled mapping is used by P0 after a certified goal graph") {
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;
    s.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0);
    const auto goal=gf::task31TriangularLattice(gf::task25DagContractFromCode(12),s.fixed_positions,{0,1});
    const auto initial=gf::task20LiftTargets(old,s.fixed_positions,gf::task26CompactFronts(old,s.fixed_positions)).targets;
    s.initial_topology=goal.contract.reference_edges;s.mobile_positions.clear();
    for(auto id:s.mobile_ids)s.mobile_positions.push_back(initial.at(id));
    auto c=gf::task19ProductionAdapterConfig();c.target_policy_task18_cbf2026_outer=false;c.target_policy_task20_dag_lattice=true;c.task20_lattice_mode=12;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,c);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
    controller.commitExternalCoverageMode(12,goal.contract);
    REQUIRE(controller.advance().step.advanced);
    const auto& targets=controller.committedTargets();const auto inverse=gf::task20FrontForMemberPose(goal.contract,s.fixed_positions,14,targets.at(14).center);
    REQUIRE(inverse.valid);const auto q=gf::task20LiftTargets(goal.contract,s.fixed_positions,{{"P",inverse.front}});
    REQUIRE(q.valid);
    for(auto id:s.mobile_ids){CHECK((q.targets.at(id)-targets.at(id).center).norm()<1e-8);CHECK(targets.at(id).x_index>=0);CHECK(targets.at(id).y_index>=0);}
}
