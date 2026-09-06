#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

static void runRoleCenterFixture(bool smooth_clock) {
    for(int variant:{0,1,2}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
        const auto a=gf::task20LiftTargets(old,scenario.fixed_positions,gf::task26CompactFronts(old,scenario.fixed_positions)).targets;
        const auto b=gf::task20LiftTargets(goal,scenario.fixed_positions,gf::task26CompactFronts(goal,scenario.fixed_positions)).targets;
        gf::Task29RoleCenterPath path(goal,a,b);
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
            const double s=gf::task29LinearExpansionPhase(tick*.1);
            controller.setExternalReconstructionReference(path.evaluate(smooth_clock?gf::task26SmoothStep(s):s));
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);
            CHECK(gf::task25_detail::edgeSet(adapter.runtimeSnapshot().topology)==gf::task25_detail::edgeSet(goal.reference_edges));
            for(const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
        }
        const auto r=adapter.runtimeSnapshot();double error=0,speed=0;
        for(std::size_t i=0;i<r.estimate.mobile_ids.size();++i) {
            error=std::max(error,(r.estimate.mean.segment<2>(4*i)-b.at(r.estimate.mobile_ids[i])).norm());
            speed=std::max(speed,r.estimate.mean.segment<2>(4*i+2).norm());
        }
        CHECK(error<=180);CHECK(speed<=3);
        std::cout<<"TASK29_ROLE_CENTER_FOCUSED "<<nlohmann::json({{"variant",variant},{"safe_ticks",1200},
            {"time_law",smooth_clock?"smoothstep":"linear"},
            {"maximum_tracking_error_m",error},{"maximum_speed_mps",speed},
            {"boundary","full fourteen-member safety and ledger fixture; end bounds are not the complete external-request/noise confirmation gate"}}).dump()<<'\n';
    }
}

TEST_CASE("Task29 role-center full fourteen-member fixture with offsets and unequal initial velocities") {
    runRoleCenterFixture(false);
}

TEST_CASE("Task29 C4 original smooth clock full fourteen-member fixture") {
    runRoleCenterFixture(true);
}

TEST_CASE("Task29 continuing front full fourteen-member moving completion fixture") {
    for(int variant:{0,1,2}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
        const auto a=gf::task20LiftTargets(old,scenario.fixed_positions,gf::task26CompactFronts(old,scenario.fixed_positions)).targets;
        const auto b=gf::task20LiftTargets(goal,scenario.fixed_positions,gf::task26CompactFronts(goal,scenario.fixed_positions)).targets;
        const gf::Task29RoleCenterPath path(goal,a,b);
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
        int dwell=0,legacy_dwell=0,safe_ticks=0;bool completed=false;
        gf::Task29CompletionAudit audit;
        for(int tick=0;tick<1200;++tick) {
            const double phase=tick*.1/60;
            const auto q=path.evaluateContinuingFront(phase);controller.setExternalReconstructionReference(q);
            const auto r=adapter.runtimeSnapshot();
            const bool graph=gf::task25_detail::edgeSet(r.topology)==gf::task25_detail::edgeSet(goal.reference_edges)&&!r.adapter_transition_pending;
            REQUIRE(graph);const auto info=adapter.currentReferenceAudit();
            const bool healthy=info.minimum_effective_reference_count>=2&&info.minimum_information_edge_count>=2&&
                info.minimum_robust_fim_cone_lower_bound>=1e-6&&
                info.maximum_posterior_eigenvalue<=cfg.maximum_posterior_eigenvalue_m2&&info.minimum_range_aoi_margin_s>=0;
            std::map<gf::NodeId,gf::Task29MotionState> states;
            for(std::size_t k=0;k<r.estimate.mobile_ids.size();++k) {
                const auto id=r.estimate.mobile_ids[k];
                states[id]={r.estimate.mean.segment<2>(4*k),r.estimate.mean.segment<2>(4*k+2),
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumPositionEigenvalue(r.estimate,id)))+cfg.certified_shadow_single_position_support_m,
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumVelocityEigenvalue(r.estimate,id)))};
            }
            audit=gf::task29MovingCompletion(goal,scenario.fixed_positions,q,states,graph,healthy);REQUIRE(audit.valid);
            dwell=audit.moving_instant_ready?dwell+1:0;
            legacy_dwell=audit.legacy_instant_ready?legacy_dwell+1:0;
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);++safe_ticks;
            for(const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
            if(phase>=1&&dwell>=10){completed=true;break;}
        }
        double front_speed=0;for(const auto& [unit,v]:audit.front_velocities)front_speed=std::max(front_speed,v.norm());
        std::cout<<"TASK29_CONTINUING_FRONT_FOCUSED "<<nlohmann::json({{"variant",variant},{"safe_ticks",safe_ticks},
            {"moving_complete",completed},{"moving_dwell_ticks",dwell},{"legacy_dwell_ticks",legacy_dwell},
            {"maximum_tracking_error_m",audit.maximum_position_bound},{"rms_tracking_error_m",audit.rms_position_bound},
            {"maximum_absolute_speed_mps",audit.maximum_absolute_speed_bound},{"common_front_speed_mps",front_speed},
            {"maximum_coordinated_speed_bound_mps",audit.maximum_coordinated_speed_bound},
            {"boundary","full fourteen-member safety and moving completion fixture; not full request, plant T100 or noise confirmation"}}).dump()<<'\n';
        CHECK(completed);
    }
}
