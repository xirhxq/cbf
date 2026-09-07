#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>
#include <cstdlib>

TEST_CASE("terminal port fixed development noise full14 expansion qualification") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-terminal-port-r1.json");
    REQUIRE(f.good());nlohmann::json asset;f>>asset;
    const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
    const auto old=gf::task25DagContractFromCode(0);const auto goal=scene.goal;
    const auto bridge=gf::task31CommonBridge(old,goal,scene.fixed,scene.direction,
        scene.bridge_spacing_m,scene.ranking_span_m,scene.frame_origin);REQUIRE(bridge.valid);
    const auto b=gf::task20LiftTargets(goal,scene.fixed,{{"P",scene.frame_origin+scene.final_front_offset}}).targets;
    const auto canonical=gf::task20LiftTargets(old,scene.fixed,gf::task26CompactFronts(old,scene.fixed)).targets;
    const gf::Task29RoleCenterPath path(goal,bridge.targets,b);
    const auto rates=gf::task31UnscaledContinuationRates(scene,canonical);
    // Existing development seeds, never confirmation. Same dispersed nonzero
    // velocity/position variant for every seed; no seed-specific geometry.
    for(const unsigned seed:{131021u,131037u,131053u}) {
        // Test-only filtered replay of a diagnosed failure, never a policy
        // branch or a changed noise field. Default still runs all three.
        const char* filter=std::getenv("TASK31_FIXTURE_DIAGNOSTIC_SEED");
        if(filter&&std::stoul(filter)!=seed)continue;
        INFO("seed="<<seed);
        auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
        scenario.fixed_positions=scene.fixed;scenario.initial_topology=goal.reference_edges;
        scenario.mobile_positions.clear();
        for(auto id:scenario.mobile_ids)scenario.mobile_positions.push_back(bridge.targets.at(id)+
            Eigen::Vector2d(4*std::sin(id),4*std::cos(id)));
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;
        cfg.target_policy_task20_dag_lattice=true;cfg.task20_lattice_mode=12;
        cfg.distance_range_availability={true,131211};cfg.range_noise_std_m=.5;cfg.range_random_seed=seed;
        REQUIRE(cfg.range_dropout_probability==0);
        auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:scenario.mobile_ids)settings["initial"]["velocity"]["values"].push_back({2*std::cos(id),2*std::sin(id)});
        Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scene.fixed,scenario.initial_topology,cfg);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
        const auto init=adapter.initializeStageZero();INFO(init.reason);REQUIRE(init.initialized);
        controller.registerExternalCoverageContract(12,goal);controller.commitExternalCoverageMode(12,goal);
        REQUIRE(controller.advance().step.advanced);const auto ledger=controller.committedTargets();
        int dwell=0,safe=0,qualification_wait_ticks=0;bool completed=false;double fim=1e100,post=0;
        gf::Task29CompletionAudit audit;
        for(int tick=0;tick<1800;++tick) {
            const double phase=tick*.1/60;const auto q=path.evaluateContinuingFrontWithRates(phase,rates);
            controller.setExternalReconstructionReference(q);const auto r=adapter.runtimeSnapshot();
            const bool graph=gf::task25_detail::edgeSet(r.topology)==gf::task25_detail::edgeSet(goal.reference_edges)&&!r.adapter_transition_pending;
            REQUIRE(graph);const auto info=adapter.currentReferenceAudit();
            const bool formal_information_healthy=info.minimum_information_edge_count>=2&&
                info.minimum_robust_fim_cone_lower_bound>=1e-6&&info.maximum_posterior_eigenvalue<=cfg.maximum_posterior_eigenvalue_m2&&info.minimum_range_aoi_margin_s>=0;
            // Match the production reconstruction contract: eligibility is
            // required for admission/completion, not a perpetual flight gate.
            // Actual rows and formal information remain mandatory every tick.
            const bool healthy=formal_information_healthy&&info.minimum_effective_reference_count>=2;
            if(!healthy)++qualification_wait_ticks;
            INFO("tick="<<tick<<" pre_control_s="<<r.runtime_s
                 <<" effective_refs="<<info.minimum_effective_reference_count
                 <<" effective_owner="<<info.minimum_effective_reference_owner
                 <<" information_count="<<info.minimum_information_edge_count
                 <<" formal_robust_fim="<<info.minimum_robust_fim_cone_lower_bound
                 <<" posterior="<<info.maximum_posterior_eigenvalue
                 <<" posterior_limit="<<cfg.maximum_posterior_eigenvalue_m2
                 <<" aoi_margin="<<info.minimum_range_aoi_margin_s);
            if(!healthy&&qualification_wait_ticks==1) {
                std::map<std::string,gf::RangeLinkState> links;
                for(const auto& [id,v]:r.range_links)links[id]={v.age_s,v.quality};
                const gf::EligibilityThresholds thresholds{cfg.add_reference_distance_m,cfg.reference_distance_m,
                    cfg.maximum_range_aoi_s,cfg.maximum_reference_position_eigenvalue_m2,cfg.minimum_range_quality,cfg.uncertainty_sigma};
                const auto eligible=gf::buildEligibility(r.estimate,links,thresholds,r.topology);
                const auto hard=gf::buildCanonicalHardRows(adapter.snapshotHardRowRequest(r.estimate,r.topology));
                nlohmann::json edges=nlohmann::json::array();
                for(const auto& e:r.topology) {
                    for(const auto& k:eligible.candidates)if(k.edge==e)
                        edges.push_back({{"edge",e.id()},{"eligible",k.eligible},{"reason",k.reason},
                            {"robust_distance_m",k.robust_distance_m},{"aoi_s",k.age_s},
                            {"hocbf_initial_audited",gf::referenceEdgeInitialSetAudited(hard,e,scenario.mobile_ids)}});
                }
                std::cout<<"TASK31_PORT_NOISE_GATE_FAILURE "<<nlohmann::json({{"seed",seed},{"tick",tick},{"time_s",r.runtime_s},
                    {"effective_references",info.minimum_effective_reference_count},{"effective_owner",info.minimum_effective_reference_owner},
                    {"formal_information_count",info.minimum_information_edge_count},{"formal_fim",info.minimum_robust_fim_cone_lower_bound},
                    {"posterior",info.maximum_posterior_eigenvalue},{"aoi_margin",info.minimum_range_aoi_margin_s},{"edges",edges}}).dump()<<std::endl;
            }
            REQUIRE(formal_information_healthy);fim=std::min(fim,info.minimum_robust_fim_cone_lower_bound);post=std::max(post,info.maximum_posterior_eigenvalue);
            std::map<gf::NodeId,gf::Task29MotionState> states;
            for(std::size_t k=0;k<r.estimate.mobile_ids.size();++k) {
                const auto id=r.estimate.mobile_ids[k];states[id]={r.estimate.mean.segment<2>(4*k),r.estimate.mean.segment<2>(4*k+2),
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumPositionEigenvalue(r.estimate,id)))+cfg.certified_shadow_single_position_support_m,
                    cfg.uncertainty_sigma*std::sqrt(std::max(0.,gf::detail::maximumVelocityEigenvalue(r.estimate,id)))};
            }
            const auto current_hard=gf::buildCanonicalHardRows(adapter.snapshotHardRowRequest(r.estimate,r.topology));
            for(const auto& edge:r.topology) {
                CHECK(gf::referenceEdgeInitialSetAudited(current_hard,edge,scenario.mobile_ids));
                const auto& owner=states.at(edge.owner);const bool fixed=scene.fixed.count(edge.reference);
                const Eigen::Vector2d other=fixed?scene.fixed.at(edge.reference):states.at(edge.reference).position;
                CHECK((owner.position-other).norm()+owner.position_support+(fixed?0:states.at(edge.reference).position_support)<850);
            }
            audit=gf::task29MovingCompletion(goal,scene.fixed,q,states,graph,healthy);REQUIRE(audit.valid);
            dwell=audit.moving_instant_ready?dwell+1:0;
            const auto step=controller.advance();INFO("tick="<<tick<<" reason="<<step.step.reason);REQUIRE(step.step.advanced);++safe;
            for(const auto& [id,cell]:ledger)CHECK(controller.committedTargets().at(id).id()==cell.id());
            if(phase>=1&&dwell>=10){completed=true;break;}
        }
        std::cout<<"TASK31_PORT_NOISE_FOCUSED "<<nlohmann::json({{"seed",seed},{"link_seed",131211},{"sigma_m",.5},
            {"safe_ticks",safe},{"moving_complete",completed},{"moving_dwell_ticks",dwell},{"qualification_wait_ticks",qualification_wait_ticks},
            {"formal_robust_fim_min",fim},{"posterior_max_m2",post},
            {"maximum_tracking_error_m",audit.maximum_position_bound},{"rms_tracking_error_m",audit.rms_position_bound},
            {"maximum_coordinated_speed_bound_mps",audit.maximum_coordinated_speed_bound},
            {"boundary","Full14 goal-DAG expansion fixture, same displaced/dispersed-velocity case for all development seeds. Not full make-before-break, coverage or confirmation."}}).dump()<<std::endl;
    }
}
