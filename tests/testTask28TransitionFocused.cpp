#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task28TransitionPath.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

namespace {
void runPathFixture(bool common_finish) {
    for (int variant:{0,1}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();
        scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
        const auto a=gf::task20LiftTargets(old,scenario.fixed_positions,gf::task26CompactFronts(old,scenario.fixed_positions)).targets;
        const auto b=gf::task20LiftTargets(goal,scenario.fixed_positions,gf::task26CompactFronts(goal,scenario.fixed_positions)).targets;
        gf::Task28LayerPath path(goal,a,b,common_finish);
        scenario.initial_topology=goal.reference_edges;scenario.mobile_positions.clear();
        for (auto id:scenario.mobile_ids)
            scenario.mobile_positions.push_back(a.at(id)+Eigen::Vector2d(variant*2.0,variant*-1.0));
        auto config=gf::task19ProductionAdapterConfig();
        config.target_policy_task18_cbf2026_outer=false;config.target_policy_task20_dag_lattice=true;
        config.task20_lattice_mode=12;
        auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for (auto id:scenario.mobile_ids) { (void)id;settings["initial"]["velocity"]["values"].push_back({variant*2.0,variant*1.0}); }
        Swarm swarm(settings);
        gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,config);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
        INFO("variant="<<variant);
        const auto init=adapter.initializeStageZero();INFO(init.reason);REQUIRE(init.initialized);
        CHECK_THROWS(controller.setExternalReconstructionReference(path.evaluate(0)));
        // Initialize a genuine P0 task ledger through its normal public step.
        REQUIRE(controller.advance().step.advanced);
        std::map<gf::NodeId,gf::FrontierCell> ledger;
        for (int tick=0;tick<1200;++tick) {
            controller.setExternalReconstructionReference(path.evaluate(gf::task26SmoothStep(tick*.1/60.)));
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);
            if (tick==0) ledger=controller.committedTargets();
            for (const auto& [id,cell]:ledger) CHECK(controller.committedTargets().at(id).id()==cell.id());
        }
        const auto r=adapter.runtimeSnapshot();nlohmann::json record;
        double maxerror=0,maxspeed=0;
        for (std::size_t k=0;k<r.estimate.mobile_ids.size();++k) {
            const auto id=r.estimate.mobile_ids[k];const Eigen::Vector2d p=r.estimate.mean.segment<2>(4*k),v=r.estimate.mean.segment<2>(4*k+2);
            maxerror=std::max(maxerror,(p-b.at(id)).norm());maxspeed=std::max(maxspeed,v.norm());
            record[std::to_string(id)]={{"position",{p.x(),p.y()}},{"velocity",{v.x(),v.y()}},{"final_target",{b.at(id).x(),b.at(id).y()}}};
        }
        std::cout<<"TASK28_FOCUSED "<<nlohmann::json({{"common_finish",common_finish},{"variant",variant},{"ticks",1200},{"maximum_tracking_error_m",maxerror},{"maximum_speed_mps",maxspeed},{"states",record}}).dump()<<'\n';
        // This fixture checks actual gates and task identity; convergence is
        // measured, not assumed from the nominal all-pair geometry.
    }
}
}
TEST_CASE("Task28 full fourteen-member path fixture with offset and nonzero velocity") {runPathFixture(false);}
TEST_CASE("Task28 common-finish full fourteen-member fixture") {runPathFixture(true);}
