#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task26 full DAG actions use deterministic acyclic single-owner handoffs") {
    for (int target:{11,12}) {
        auto a=gf::task25DagContractFromCode(0);
        const auto b=gf::task25DagContractFromCode(target);
        const auto plan=gf::task26ReplacementPlan(a.reference_edges,b.reference_edges);
        REQUIRE(plan.valid);
        CHECK(plan.replacements.size()==gf::task25DagReplacementCount(a,b));
        for (const auto& pair:plan.replacements) {
            CHECK(pair.first.owner==pair.second.owner);
            auto united=a.reference_edges;
            united.push_back(pair.first);
            CHECK(gf::task26ValidDag(united,3));
            a.reference_edges=gf::transition_certifier_detail::without(united,pair.second);
            CHECK(gf::task26ValidDag(a.reference_edges,2));
        }
        CHECK(gf::task25_detail::edgeSet(a.reference_edges)==gf::task25_detail::edgeSet(b.reference_edges));
    }
}

TEST_CASE("Task26 motion takeover preserves real task ledger and requires matching DAG for mode commit") {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto config=gf::task19ProductionAdapterConfig();
    config.target_policy_task18_cbf2026_outer=false;
    config.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,config);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    REQUIRE(adapter.initializeStageZero().initialized);
    REQUIRE(controller.advance().step.advanced);
    const auto ledger=controller.committedTargets();
    std::map<gf::NodeId,Eigen::Vector2d> reference;
    for (const auto& [id,cell]:ledger) reference[id]=cell.center+Eigen::Vector2d(25,0);
    CHECK_THROWS(controller.commitExternalCoverageMode(12));
    controller.setExternalReconstructionReference(reference);
    const auto step=controller.advance();
    REQUIRE(step.step.advanced);
    for (const auto& [id,cell]:ledger) {
        CHECK(step.committed_targets.at(id).id()==cell.id());
        CHECK((step.applied_target_centers.at(id)-reference.at(id)).norm()<1e-9);
    }
    controller.setExternalReconstructionReference(std::nullopt);
    CHECK_THROWS(controller.setExternalReconstructionReference(std::map<gf::NodeId,Eigen::Vector2d>{}));
    gf::Task26ExternalReconstructor external(adapter,controller,"pinball",0.1);
    external.beforeStep();
    CHECK(external.telemetry().at("stage")=="contracting");
    CHECK(external.telemetry().at("task_ledger_active")==false);
    CHECK(external.telemetry().at("active_mode")==0);
    CHECK(external.telemetry().at("pending_mode")==12);
    CHECK(external.report().at("requests").size()==1);
    CHECK(adapter.supervisor().topologyVersion()==1);
    REQUIRE(controller.advance().step.advanced);
    for (const auto& [id,cell]:ledger)
        CHECK(controller.committedTargets().at(id).id()==cell.id());
    REQUIRE(adapter.beginReplacement({8,2},{101,2},false));
    const auto before_union=adapter.runtimeSnapshot();
    REQUIRE(before_union.topology.size()==29);
    const auto union_step=controller.advance();
    REQUIRE(union_step.step.advanced);
    CHECK(union_step.step.mode==gf::SupervisorMode::Union);
    CHECK(union_step.desired_yaw_rad.size()==14);
    for (std::size_t i=0;i<14;++i) {
        const auto id=before_union.estimate.mobile_ids[i];
        const Eigen::Vector2d v=before_union.estimate.mean.segment<2>(4*i+2);
        if (v.norm()>1e-9)
            CHECK(union_step.desired_yaw_rad.at(id)==doctest::Approx(std::atan2(v.y(),v.x())));
    }
    CHECK(adapter.finishReplacementAfterFreshCycle(true));
    CHECK(adapter.runtimeSnapshot().topology.size()==28);
}
