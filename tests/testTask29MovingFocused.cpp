#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task29 full fourteen-member real-controller fixture includes position offsets and unequal nonzero velocity") {
    for (int variant:{0,1,2}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
        const auto a=gf::task20LiftTargets(old,scenario.fixed_positions,gf::task26CompactFronts(old,scenario.fixed_positions)).targets;
        const auto b=gf::task20LiftTargets(goal,scenario.fixed_positions,gf::task26CompactFronts(goal,scenario.fixed_positions)).targets;
        gf::Task28LayerPath path(goal,a,b,gf::Task28LayerPath::Kind::CenteredFrame);
        scenario.initial_topology=goal.reference_edges;scenario.mobile_positions.clear();
        for (auto id:scenario.mobile_ids)scenario.mobile_positions.push_back(a.at(id)+Eigen::Vector2d(variant*2*std::sin(id),variant*2*std::cos(id)));
        auto config=gf::task19ProductionAdapterConfig();config.target_policy_task18_cbf2026_outer=false;
        config.target_policy_task20_dag_lattice=true;config.task20_lattice_mode=12;
        auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for (auto id:scenario.mobile_ids) {
            const Eigen::Vector2d v=variant==2?Eigen::Vector2d(2*std::cos(id),2*std::sin(id)):Eigen::Vector2d(variant*2,variant);
            settings["initial"]["velocity"]["values"].push_back({v.x(),v.y()});
        }
        Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,config);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
        INFO("variant="<<variant);const auto initialized=adapter.initializeStageZero();INFO(initialized.reason);REQUIRE(initialized.initialized);
        REQUIRE(controller.advance().step.advanced);
        const auto ledger=controller.committedTargets();double largest_residual=0;int moving_ticks=0,legacy_ticks=0;
        for (int tick=0;tick<1200;++tick) {
            const auto q=path.evaluate(gf::task26SmoothStep(tick*.1/60));
            controller.setExternalReconstructionReference(q);
            const auto r=adapter.runtimeSnapshot();std::map<gf::NodeId,gf::Task29MotionState> state;
            for (std::size_t k=0;k<r.estimate.mobile_ids.size();++k) {
                const auto id=r.estimate.mobile_ids[k];
                state[id]={r.estimate.mean.segment<2>(4*k),r.estimate.mean.segment<2>(4*k+2),
                    config.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumPositionEigenvalue(r.estimate,id)))+config.certified_shadow_single_position_support_m,
                    config.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumVelocityEigenvalue(r.estimate,id)))};
            }
            const auto audit=gf::task29MovingCompletion(goal,scenario.fixed_positions,q,state,true,true);
            REQUIRE(audit.valid);largest_residual=std::max(largest_residual,audit.maximum_coordinated_speed_bound);
            moving_ticks+=audit.moving_instant_ready;legacy_ticks+=audit.legacy_instant_ready;
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);
            for (const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
        }
        std::cout<<"TASK29_FOCUSED "<<nlohmann::json({{"variant",variant},{"safe_ticks",1200},
            {"moving_instant_ticks",moving_ticks},{"legacy_instant_ticks",legacy_ticks},{"max_coordinated_bound_mps",largest_residual}}).dump()<<'\n';
    }
}
