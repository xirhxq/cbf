#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task29 compact height full fourteen-member old-DAG fixture retains actual gates under offsets and nonzero unequal velocity") {
    for(int variant:{0,1,2}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
        auto startfront=gf::task26CompactFronts(old,scenario.fixed_positions),zero=startfront,far=startfront;
        for(auto& [u,p]:zero)p.y()=-50;for(auto& [u,p]:far)p.y()=1350;
        const auto start=gf::task20LiftTargets(old,scenario.fixed_positions,startfront).targets;
        const auto hypothetical=gf::task20LiftTargets(old,scenario.fixed_positions,far).targets;
        auto edges=old.reference_edges;edges.insert(edges.end(),goal.reference_edges.begin(),goal.reference_edges.end());
        const auto p=gf::task29ProjectedCompact(old,scenario.fixed_positions,zero,{0,1},hypothetical,edges,850,3*std::sqrt(.1),10);
        REQUIRE(p.valid); // Uniform high-displacement geometry fixture, not a task or a seed-derived request.
        scenario.mobile_positions.clear();for(auto id:scenario.mobile_ids)
            scenario.mobile_positions.push_back(start.at(id)+Eigen::Vector2d(variant*2*std::sin(id),variant*2*std::cos(id)));
        auto config=gf::task19ProductionAdapterConfig();config.target_policy_task18_cbf2026_outer=false;
        config.target_policy_task20_dag_lattice=true;config.task20_lattice_mode=0;
        auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:scenario.mobile_ids) {
            const Eigen::Vector2d v=variant==2?Eigen::Vector2d(2*std::cos(id),2*std::sin(id)):Eigen::Vector2d(variant*2,variant);
            settings["initial"]["velocity"]["values"].push_back({v.x(),v.y()});
        }
        Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,config);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
        INFO("variant="<<variant);const auto initial=adapter.initializeStageZero();INFO(initial.reason);REQUIRE(initial.initialized);
        REQUIRE(controller.advance().step.advanced);const auto ledger=controller.committedTargets();
        for(int tick=0;tick<800;++tick) {
            auto fronts=startfront;const double h=gf::task26SmoothStep(tick*.1/60);
            for(auto& [u,q]:fronts)q=(1-h)*q+h*p.fronts.at(u);
            const auto targets=gf::task20LiftTargets(old,scenario.fixed_positions,fronts);REQUIRE(targets.valid);
            controller.setExternalReconstructionReference(targets.targets);
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);
            CHECK(gf::task25_detail::edgeSet(adapter.runtimeSnapshot().topology)==gf::task25_detail::edgeSet(old.reference_edges));
            for(const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
        }
        const auto plan=gf::task26ReplacementPlan(old.reference_edges,goal.reference_edges);
        const auto audits=adapter.auditReplacementPlan(plan.replacements);std::size_t qualified=0;
        for(const auto& a:audits)qualified+=a.valid;
        std::cout<<"TASK29_COMPACT_FOCUSED "<<nlohmann::json({{"variant",variant},{"safe_ticks",800},
            {"endpoint_snapshot_qualified_plan_states",qualified},{"plan_states",plan.replacements.size()},
            {"boundary","old DAG stayed active; static projection never admits a new edge"}}).dump()<<'\n';
    }
}
