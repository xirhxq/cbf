#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task29 single common time law full fourteen-member fixture with offsets and unequal nonzero motion") {
    for(int variant:{0,1,2}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
        const auto a=gf::task20LiftTargets(old,scenario.fixed_positions,gf::task26CompactFronts(old,scenario.fixed_positions)).targets;
        const auto b=gf::task20LiftTargets(goal,scenario.fixed_positions,gf::task26CompactFronts(goal,scenario.fixed_positions)).targets;
        gf::Task28LayerPath path(goal,a,b,gf::Task28LayerPath::Kind::CenteredFrame);
        scenario.initial_topology=goal.reference_edges;scenario.mobile_positions.clear();
        for(auto id:scenario.mobile_ids)
            scenario.mobile_positions.push_back(a.at(id)+Eigen::Vector2d(variant*2*std::sin(id),variant*2*std::cos(id)));
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;
        cfg.target_policy_task20_dag_lattice=true;cfg.task20_lattice_mode=12;
        auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:scenario.mobile_ids) {
            const Eigen::Vector2d v=variant==2?Eigen::Vector2d(2*std::cos(id),2*std::sin(id)):Eigen::Vector2d(variant*2,variant);
            settings["initial"]["velocity"]["values"].push_back({v.x(),v.y()});
        }
        Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,cfg);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
        INFO("variant="<<variant);const auto init=adapter.initializeStageZero();INFO(init.reason);REQUIRE(init.initialized);
        REQUIRE(controller.advance().step.advanced);const auto ledger=controller.committedTargets();
        for(int tick=0;tick<1200;++tick) {
            controller.setExternalReconstructionReference(path.evaluate(gf::task29LinearExpansionPhase(tick*.1)));
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);
            CHECK(gf::task25_detail::edgeSet(adapter.runtimeSnapshot().topology)==gf::task25_detail::edgeSet(goal.reference_edges));
            for(const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
        }
        const auto r=adapter.runtimeSnapshot();double error=0,speed=0;
        for(std::size_t i=0;i<r.estimate.mobile_ids.size();++i) {
            error=std::max(error,(r.estimate.mean.segment<2>(4*i)-b.at(r.estimate.mobile_ids[i])).norm());
            speed=std::max(speed,r.estimate.mean.segment<2>(4*i+2).norm());
        }
        std::cout<<"TASK29_LINEAR_PHASE_FOCUSED "<<nlohmann::json({{"variant",variant},{"safe_ticks",1200},
            {"maximum_tracking_error_m",error},{"maximum_speed_mps",speed},
            {"boundary","full fourteen-member safety and ledger fixture, not a saved noisy checkpoint or request-completion proof"}}).dump()<<'\n';
    }
}
