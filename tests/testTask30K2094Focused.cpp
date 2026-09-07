#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task30 k2094 full fourteen-member old-only bridge fixture with offsets and unequal velocities") {
    const double kappa=2.094;

    for(int variant:{0,1,2}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
        const auto a=gf::task20LiftTargets(old,scenario.fixed_positions,
            gf::task30SimilarityBridgeFronts(old,scenario.fixed_positions,gf::task26CompactFronts(old,scenario.fixed_positions),kappa)).targets;
        const auto b=gf::task20LiftTargets(goal,scenario.fixed_positions,
            gf::task26CompactFronts(goal,scenario.fixed_positions)).targets;
        const auto canonical=gf::task20LiftTargets(old,scenario.fixed_positions,gf::task26CompactFronts(old,scenario.fixed_positions)).targets;
        const gf::Task29RoleCenterPath path(goal,a,b);
        scenario.initial_topology=goal.reference_edges;scenario.mobile_positions.clear();
        for(auto id:scenario.mobile_ids)scenario.mobile_positions.push_back(a.at(id)+Eigen::Vector2d(variant*2*std::sin(id),variant*2*std::cos(id)));
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
        INFO("kappa="<<kappa<<" variant="<<variant);
        const auto init=adapter.initializeStageZero();INFO(init.reason);REQUIRE(init.initialized);
        REQUIRE(controller.advance().step.advanced);const auto ledger=controller.committedTargets();
        int dwell=0,legacy_dwell=0,safe_ticks=0;bool completed=false;
        gf::Task29CompletionAudit audit;
        for(int tick=0;tick<1200;++tick) {
            const double phase=tick*.1/60;const auto q=path.evaluateContinuingFront(phase,canonical);
            controller.setExternalReconstructionReference(q);const auto r=adapter.runtimeSnapshot();
            const bool graph=gf::task25_detail::edgeSet(r.topology)==gf::task25_detail::edgeSet(goal.reference_edges)&&!r.adapter_transition_pending;
            REQUIRE(graph);const auto info=adapter.currentReferenceAudit();
            const bool healthy=info.minimum_effective_reference_count>=2&&info.minimum_information_edge_count>=2&&
                info.minimum_robust_fim_cone_lower_bound>=1e-6&&info.maximum_posterior_eigenvalue<=cfg.maximum_posterior_eigenvalue_m2&&info.minimum_range_aoi_margin_s>=0;
            std::map<gf::NodeId,gf::Task29MotionState> states;
            for(std::size_t k=0;k<r.estimate.mobile_ids.size();++k) {
                const auto id=r.estimate.mobile_ids[k];
                states[id]={r.estimate.mean.segment<2>(4*k),r.estimate.mean.segment<2>(4*k+2),
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumPositionEigenvalue(r.estimate,id)))+cfg.certified_shadow_single_position_support_m,
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumVelocityEigenvalue(r.estimate,id)))};
            }
            audit=gf::task29MovingCompletion(goal,scenario.fixed_positions,q,states,graph,healthy);REQUIRE(audit.valid);
            dwell=audit.moving_instant_ready?dwell+1:0;legacy_dwell=audit.legacy_instant_ready?legacy_dwell+1:0;
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);++safe_ticks;
            for(const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
            if(phase>=1&&dwell>=10){completed=true;break;}
        }
        double front_speed=0;for(const auto& [u,v]:audit.front_velocities)front_speed=std::max(front_speed,v.norm());
        std::cout<<"TASK30_OLDONLY_FOCUSED "<<nlohmann::json({{"kappa",kappa},{"variant",variant},{"safe_ticks",safe_ticks},
            {"moving_complete",completed},{"moving_dwell_ticks",dwell},{"legacy_dwell_ticks",legacy_dwell},
            {"maximum_tracking_error_m",audit.maximum_position_bound},{"rms_tracking_error_m",audit.rms_position_bound},
            {"maximum_absolute_speed_mps",audit.maximum_absolute_speed_bound},{"common_front_speed_mps",front_speed},
            {"maximum_coordinated_speed_bound_mps",audit.maximum_coordinated_speed_bound},
            {"boundary","full14 labelled expansion in actual goal DAG; no claim of full fresh make-before-break or plant T100"}}).dump()<<'\n';
        // Capability is measured, not assumed: a safe incomplete fixture is
        // retained and diagnosed before deciding on the full request window.
    }
}
