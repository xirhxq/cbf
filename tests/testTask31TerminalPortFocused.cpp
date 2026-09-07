#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("terminal port full14 expansion retains fixed information and moving gates") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-terminal-port-r1.json");
    REQUIRE(f.good());nlohmann::json asset;f>>asset;
    const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
    REQUIRE(scene.front_similarity_gain==1.);REQUIRE(scene.front_port_binding=="positive_cross_terminal");
    for(int variant:{0,1,2}) {
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions=scene.fixed;
        const auto old=gf::task25DagContractFromCode(0);const auto goal=scene.goal;
        const auto bridge=gf::task31CommonBridge(old,goal,scene.fixed,scene.direction,
            scene.bridge_spacing_m,scene.ranking_span_m,scene.frame_origin);
        REQUIRE(bridge.valid);const auto a=bridge.targets;
        const auto b=gf::task20LiftTargets(goal,scene.fixed,{{"P",scene.frame_origin+scene.final_front_offset}}).targets;
        const auto canonical=gf::task20LiftTargets(old,scenario.fixed_positions,
            gf::task26CompactFronts(old,scenario.fixed_positions)).targets;
        const gf::Task29RoleCenterPath path(goal,a,b);
        const auto continuation=gf::task31UnscaledContinuationRates(scene,canonical);
        scenario.initial_topology=goal.reference_edges;scenario.mobile_positions.clear();
        for(auto id:scenario.mobile_ids)scenario.mobile_positions.push_back(a.at(id)+
            Eigen::Vector2d(variant*2*std::sin(id),variant*2*std::cos(id)));
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;
        cfg.target_policy_task20_dag_lattice=true;cfg.task20_lattice_mode=12;
        cfg.distance_range_availability={true,131211};
        REQUIRE(cfg.range_noise_std_m==0);REQUIRE(cfg.range_dropout_probability==0);
        auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:scenario.mobile_ids) {
            const Eigen::Vector2d v=variant==2?Eigen::Vector2d(2*std::cos(id),2*std::sin(id)):Eigen::Vector2d(variant*2,variant);
            settings["initial"]["velocity"]["values"].push_back({v.x(),v.y()});
        }
        Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,
            scenario.fixed_positions,scenario.initial_topology,cfg);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
        INFO("variant="<<variant);const auto init=adapter.initializeStageZero();INFO(init.reason);REQUIRE(init.initialized);
        controller.registerExternalCoverageContract(12,goal);controller.commitExternalCoverageMode(12,goal);
        REQUIRE(controller.advance().step.advanced);const auto ledger=controller.committedTargets();
        int dwell=0,safe_ticks=0;bool completed=false;double minimum_fim=1e100,maximum_post=0;
        gf::Task29CompletionAudit audit;
        // Observation cap180s, not a changed path or60s reference time law.
        for(int tick=0;tick<1800;++tick) {
            const double phase=tick*.1/60;const auto q=path.evaluateContinuingFrontWithRates(phase,continuation);
            controller.setExternalReconstructionReference(q);const auto r=adapter.runtimeSnapshot();
            const bool graph=gf::task25_detail::edgeSet(r.topology)==gf::task25_detail::edgeSet(goal.reference_edges)&&!r.adapter_transition_pending;
            REQUIRE(graph);const auto info=adapter.currentReferenceAudit();
            const bool healthy=info.minimum_effective_reference_count>=2&&info.minimum_information_edge_count>=2&&
                info.minimum_robust_fim_cone_lower_bound>=1e-6&&info.maximum_posterior_eigenvalue<=cfg.maximum_posterior_eigenvalue_m2&&info.minimum_range_aoi_margin_s>=0;
            minimum_fim=std::min(minimum_fim,info.minimum_robust_fim_cone_lower_bound);
            maximum_post=std::max(maximum_post,info.maximum_posterior_eigenvalue);
            std::map<gf::NodeId,gf::Task29MotionState> states;
            for(std::size_t k=0;k<r.estimate.mobile_ids.size();++k) {
                const auto id=r.estimate.mobile_ids[k];
                states[id]={r.estimate.mean.segment<2>(4*k),r.estimate.mean.segment<2>(4*k+2),
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumPositionEigenvalue(r.estimate,id)))+cfg.certified_shadow_single_position_support_m,
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumVelocityEigenvalue(r.estimate,id)))};
            }
            REQUIRE(healthy);
            for(const auto& edge:r.topology) {
                const auto& owner=states.at(edge.owner);
                const Eigen::Vector2d other=scenario.fixed_positions.count(edge.reference)
                    ?scenario.fixed_positions.at(edge.reference):states.at(edge.reference).position;
                const double reserve=owner.position_support+(scenario.fixed_positions.count(edge.reference)?0:states.at(edge.reference).position_support);
                CHECK((owner.position-other).norm()+reserve<850);
            }
            audit=gf::task29MovingCompletion(goal,scenario.fixed_positions,q,states,graph,healthy);REQUIRE(audit.valid);
            dwell=audit.moving_instant_ready?dwell+1:0;
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);++safe_ticks;
            for(const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
            if(phase>=1&&dwell>=10){completed=true;break;}
        }
        std::cout<<"TASK31_PORT_FOCUSED "<<nlohmann::json({{"variant",variant},{"link_seed",131211},
            {"ports",scene.terminal_ports},{"safe_ticks",safe_ticks},{"moving_complete",completed},
            {"moving_dwell_ticks",dwell},{"formal_robust_fim_min",minimum_fim},{"posterior_max_m2",maximum_post},
            {"maximum_tracking_error_m",audit.maximum_position_bound},{"rms_tracking_error_m",audit.rms_position_bound},
            {"maximum_coordinated_speed_bound_mps",audit.maximum_coordinated_speed_bound},
            {"boundary","Full14 expansion under actual goal DAG;180s observation, unchanged60s reference law. Not full fresh make-before-break or T100."}}).dump()<<std::endl;
    }
}
