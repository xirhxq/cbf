#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"
#include "grand_finale/Task17GridTelemetry.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/Task19MinimalDagSwitcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

namespace {

using json=nlohmann::json;

struct PhaseATemplate {
    std::string id;
    std::string description;
    std::vector<Eigen::Vector2d> positions;
    std::vector<gf::DirectedEdge> topology;
};

std::vector<Eigen::Vector2d> scaledPositions(double scale) {
    // Branch roots at nodes 1 (1380,10) and 8 (1620,10); spacing scaled
    // about each root along the standard launch geometry.
    const std::vector<Eigen::Vector2d> launch=
        gf::task10p11pStandardLaunchPositions();
    std::vector<Eigen::Vector2d> result;
    for (std::size_t index=0;index<launch.size();++index) {
        const gf::NodeId id=static_cast<gf::NodeId>(index+1);
        const Eigen::Vector2d root=
            (id<=7)?launch[0]:launch[7];
        result.push_back(root+scale*(launch[index]-root));
    }
    return result;
}

PhaseATemplate makeTemplate(const std::string& id) {
    const auto launch=gf::task10p11pStandardLaunchPositions();
    const auto origin=gf::task10p11rFixedReferenceTopology();
    if (id=="origin") {
        return {"origin","frozen ladder as-is (origin template)",
            launch,origin};
    }
    if (id=="stretch3") {
        return {"stretch3",
            "ladder graph unchanged, inter-node spacing x3 about branch "
            "roots (sweep baseline widened)",
            scaledPositions(3.0),origin};
    }
    if (id=="contract_half") {
        return {"contract_half",
            "ladder graph unchanged, inter-node spacing x0.5 about branch "
            "roots (formation tightened)",
            scaledPositions(0.5),origin};
    }
    if (id=="reanchor") {
        // Branch A re-rooted at anchor 102 (was 101); branch B re-rooted
        // at anchor pair 100+101 (was 101+102).  Ladder graphs unchanged.
        std::vector<gf::DirectedEdge> topology;
        for (const auto& edge:origin) {
            if (edge==gf::DirectedEdge(101,1)) topology.emplace_back(102,1);
            else if (edge==gf::DirectedEdge(101,2))
                topology.emplace_back(102,2);
            else if (edge==gf::DirectedEdge(101,8))
                topology.emplace_back(100,8);
            else topology.push_back(edge);
        }
        return {"reanchor",
            "ladder graphs unchanged, branch A re-rooted at anchor 102, "
            "branch B re-rooted at anchor pair 100+101 (anchor roles "
            "swapped between branches)",
            launch,topology};
    }
    if (id=="crosslink") {
        // Cross-branch interlink, acyclic: node 10 re-references {9,4}
        // (drops 8) and node 11 re-references {10,5} (drops 9); branch B's
        // upper half hangs off branch A's tail.  Indegree stays 2.
        std::vector<gf::DirectedEdge> topology;
        for (const auto& edge:origin) {
            if (edge==gf::DirectedEdge(8,10)) topology.emplace_back(4,10);
            else if (edge==gf::DirectedEdge(9,11))
                topology.emplace_back(5,11);
            else topology.push_back(edge);
        }
        return {"crosslink",
            "origin ladder with cross-branch interlink: node 10 "
            "re-references {9,4}, node 11 re-references {10,5} "
            "(acyclic, indegree preserved)",
            launch,topology};
    }
    if (id=="microfix") {
        // Causal micro-experiment (researcher-approved 2026-09-01):
        // origin graph with exactly two edge changes - owner 2
        // re-referenced 101->100 and owner 9 re-referenced 101->102 -
        // aligning each owner's anchor with its target side (the
        // bias-inversion finding: branch A targets 76-83% on the 100
        // side, branch B 85-92% on the 102 side, while owners 2/9 anchor
        // at center anchor 101 and historically hit the reference gate
        // there).
        std::vector<gf::DirectedEdge> topology;
        for (const auto& edge:origin) {
            if (edge==gf::DirectedEdge(101,2)) topology.emplace_back(100,2);
            else if (edge==gf::DirectedEdge(101,9))
                topology.emplace_back(102,9);
            else topology.push_back(edge);
        }
        return {"microfix",
            "origin graph + two edge changes (101->2 becomes 100->2, "
            "101->9 becomes 102->9); all else frozen",
            launch,topology};
    }
    throw std::runtime_error("unknown template:"+id);
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeFixture(
    const PhaseATemplate& def,double tau,bool policy_v2,
    bool velocity_augmented_rows,bool policy_v3,bool policy_v6,
    bool leader_reachability_filter,bool policy_h2,
    gf::GammaFeedbackSelectionMode selection,double service_standoff_m,
    double range_noise_std_m,double range_dropout_probability,
    unsigned int range_random_seed,double acceleration_half_box_mps2,
    gf::WorkspaceClassK workspace_class_k,double workspace_alpha1_gain,
    double workspace_alpha2_gain,double workspace_braking_acceleration_mps2,
    double workspace_braking_regularization_m,bool target_homotopy_enabled,
    double target_homotopy_rate_gain,double target_homotopy_workspace_guard_m,
    bool unified_h2_cbf2026_wide_virtual_formation,
    bool target_policy_task15_forward,
    bool target_policy_task16_cbf2026,
    gf::Task16CoverageArm task16_coverage_arm,
    bool task16_tracking_envelope_enabled,
    std::size_t task16_plant_speed_facet_count,
    double task16_reference_damping_reserve_multiples,
    double task16_speed_row_limit_mps,
    bool target_policy_task17_periodic,
    gf::Task17PeriodicArm task17_periodic_arm,
    bool task17_common_governor_enabled,
    bool task17_reference_compatible_formation,
    bool task17_member_aware_wide_formation,
    bool task17_coherent_service_wide_formation,
    std::size_t task17_update_period_cycles,
    bool target_policy_task18_cbf2026_outer,
    bool task18_common_governor_enabled,
    bool task18_collision_only_vaug,
    gf::Task18YawObjective task18_yaw_objective,
    std::size_t task18_update_period_cycles) {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=def.positions;
    scenario.initial_topology=def.topology;
    auto settings=gf::task10p11pSwarmSettings(scenario,
        gf::SolverProfile::Gurobi);
    // Rung B'' speed domain: single nominal row @30, gain 7, no
    // saturation, preflight demoted (31 m/s fuse), truth initial-set gate.
    // Bool order: srnm, margin, family, analytic, demoted, throttle,
    // throttle_v2, rows_removed, rung_b, v4_prime, bare, rung_b_prime,
    // rung_b2, target_policy_v2, velocity_augmented_rows,
    // target_policy_v3.
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),
        selection,tau,true,
        false,false,false,false,false,false,
        false,false,false,false,false,true,policy_v2,
        velocity_augmented_rows,policy_v3,policy_v6,
        leader_reachability_filter,policy_h2,service_standoff_m,
        range_noise_std_m,range_dropout_probability,range_random_seed,
        acceleration_half_box_mps2,workspace_class_k,
        workspace_alpha1_gain,workspace_alpha2_gain,
        workspace_braking_acceleration_mps2,
        workspace_braking_regularization_m,target_homotopy_enabled,
        acceleration_half_box_mps2,target_homotopy_rate_gain,
        target_homotopy_workspace_guard_m,
        unified_h2_cbf2026_wide_virtual_formation,
        target_policy_task15_forward,target_policy_task16_cbf2026,
        task16_coverage_arm,task16_tracking_envelope_enabled,
        task16_plant_speed_facet_count,
        task16_reference_damping_reserve_multiples,
        task16_speed_row_limit_mps,
        target_policy_task17_periodic,task17_periodic_arm,
        task17_common_governor_enabled,
        task17_reference_compatible_formation,
        task17_member_aware_wide_formation,
        task17_coherent_service_wide_formation,
        task17_update_period_cycles,
        target_policy_task18_cbf2026_outer,
        task18_common_governor_enabled,
        task18_collision_only_vaug,
        task18_yaw_objective,
        task18_update_period_cycles);
}

}  // namespace

// GrandFinale production/search evidence runner.  Historical policies remain
// explicit; an omitted policy selects the researcher-frozen Task 18 baseline.
int main(int argc,char** argv) {
    if (argc<5||argc>22) {
        std::cerr<<"usage: GrandFinaleTask13PhaseARun TEMPLATE OUTPUT_JSON "
            "PROGRESS_DIR TELEMETRY_JSONL [TAU] [WINDOW_S] [POLICY] [ROWS] "
            "[RANGE_NOISE_STD_M] [DROPOUT_PROBABILITY] [RANGE_SEED] "
            "[SERVICE_STANDOFF_M] [ACCELERATION_HALF_BOX_MPS2] "
            "[linear|braking] [WS_K1] [WS_K2] [WS_BRAKE_A] [WS_EPS] "
            "[TARGET_HOMOTOPY_0_OR_1] [HOMOTOPY_RATE_GAIN] "
            "[HOMOTOPY_GUARD_M]\n";
        return 2;
    }
    const auto started=std::chrono::steady_clock::now();
    const auto elapsed=[&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
    };
    try {
        const std::string template_id=argv[1];
        const auto production_defaults=gf::task19ProductionDefaults();
        const std::string policy=argc>=8?argv[7]:
            production_defaults.policy;
        const bool policy_production=policy==production_defaults.policy;
        const bool policy_task19_switch=
            policy=="task19-switch-origin-microfix";
        const bool production_semantics=
            policy_production||policy_task19_switch;
        const double window_s=argc>=7?std::stod(argv[6]):
            (production_semantics?production_defaults.resource_watchdog_s:
                500.0);
        const double tau=argc>=6?std::stod(argv[5]):
            (production_semantics?production_defaults.predictive_tau_mps2:
                22.0);
        const bool policy_v2=policy=="v2";
        // v3 and v4 share the target_policy_v3 flag: v3 was the centroid-
        // primary form (rejected), v4 is the frontier-pacing +
        // centroid-direction-scoring form (three-strikes round).
        const bool policy_v3=policy=="v3"||policy=="v4"||policy=="v5"||
            policy=="v7";
        const bool leader_reachability_filter=policy=="v5";
        const bool policy_v6=policy=="v6";
        const bool policy_h2=policy=="h2"||policy=="h2diag"||
            policy=="h2center"||policy=="h2wide";
        const bool policy_h2wide=policy=="h2wide";
        const bool policy_task15=policy=="task15";
        const bool policy_task16=policy=="task16a"||policy=="task16b"||
            policy=="task16c";
        const bool policy_task17=policy.rfind("task17b",0)==0||
            policy.rfind("task17c",0)==0||
            policy.rfind("task17simple",0)==0;
        const bool policy_task18=production_semantics||
            policy.rfind("task18",0)==0;
        const auto task16_arm=policy=="task16c"
            ?gf::Task16CoverageArm::FormationAware:
            policy=="task16b"?gf::Task16CoverageArm::BoundaryDecoupled:
            gf::Task16CoverageArm::HistoricalClipped;
        const auto task17_arm=policy.rfind("task17c",0)==0
            ?gf::Task17PeriodicArm::SuccessorServiceTime:
            policy.rfind("task17simple",0)==0
                ?gf::Task17PeriodicArm::CurrentMemberDistance:
                 gf::Task17PeriodicArm::Voronoi;
        const bool task17_common_governor_enabled=
            policy.find("-direct")==std::string::npos;
        const bool task17_reference_compatible_formation=
            policy.find("-refcompat")!=std::string::npos;
        const bool task17_member_aware_wide_formation=
            policy.find("-memberwide")!=std::string::npos;
        const bool task17_coherent_service_wide_formation=
            policy.find("-servicewide")!=std::string::npos;
        const std::size_t task17_update_period_cycles=
            gf::task17UpdatePeriodCycles(policy);
        const bool task18_common_governor_enabled=
            policy.find("-lambda")!=std::string::npos;
        const bool task18_collision_only_vaug=
            policy.find("-collisionvaug")!=std::string::npos;
        const auto task18_yaw_objective=
            policy.find("-legacyyaw")!=std::string::npos
                ?gf::Task18YawObjective::LegacyCvtSoftCbf:
            policy.find("-taskyaw")!=std::string::npos
                ?gf::Task18YawObjective::SharedTask:
            policy.find("-coneyaw")!=std::string::npos
                ?gf::Task18YawObjective::VelocityTaskCone:
            production_semantics||policy.find("-velyaw")!=std::string::npos
                ?gf::Task18YawObjective::ActualVelocity:
                 gf::Task18YawObjective::IndividualFormationTarget;
        const double service_standoff_m=argc>=13?std::stod(argv[12]):
            (policy=="h2center"?0.0:350.0);
        const double acceleration_half_box_mps2=argc>=14?
            std::stod(argv[13]):4.0;
        const std::string workspace_class_k_name=argc>=15?argv[14]:"linear";
        const auto workspace_class_k=workspace_class_k_name=="braking"
            ?gf::WorkspaceClassK::RegularizedBraking:
             gf::WorkspaceClassK::Linear;
        const double workspace_alpha1_gain=argc>=16?std::stod(argv[15]):1.0;
        const double workspace_alpha2_gain=argc>=17?std::stod(argv[16]):1.0;
        const double workspace_braking_acceleration_mps2=argc>=18?
            std::stod(argv[17]):4.0;
        const double workspace_braking_regularization_m=argc>=19?
            std::stod(argv[18]):1.0;
        const bool target_homotopy_enabled=argc>=20?
            std::stoi(argv[19])!=0:false;
        const double target_homotopy_rate_gain=argc>=21?
            std::stod(argv[20]):1.0;
        const double target_homotopy_workspace_guard_m=argc>=22?
            std::stod(argv[21]):1.5;
        const auto gamma_selection=policy=="h2diag"||
            policy.find("-projection")!=std::string::npos
            ?gf::GammaFeedbackSelectionMode::DiagnosticsOnly
            :policy.find("-maxmargin")!=std::string::npos
                ?gf::GammaFeedbackSelectionMode::MaximumPredictedMargin
                :gf::GammaFeedbackSelectionMode::LeastIntervention;
        if (policy_v2&&policy_v6) {
            std::cerr<<"policies v2 and v6 are mutually exclusive\n";
            return 2;
        }
        if (policy=="v4") {
            std::cout<<"policy v4 (frontier pacing + centroid direction "
                "scoring)\n";
        }
        if (policy_v2&&policy_v3) {
            std::cerr<<"policies v2 and v3 are mutually exclusive\n";
            return 2;
        }
        const std::string rows_mode=argc>=9?argv[8]:
            (production_semantics?"vaug-speed29p9":"classic");
        const bool velocity_augmented_rows=
            rows_mode.rfind("vaug",0)==0;
        const bool task16_tracking_envelope_enabled=
            rows_mode=="vaug-envelope";
        const std::size_t task16_plant_speed_facet_count=
            rows_mode=="vaug-facets"?64:0;
        const double task16_reference_damping_reserve_multiples=
            (rows_mode=="vaug-rate10"||rows_mode=="vaug-envelope"||
             rows_mode=="vaug-facets")?4.0:1.0;
        const double task16_speed_row_limit_mps=
            rows_mode.find("speed29p9")!=std::string::npos?29.9:0.0;
        const double range_noise_std_m=argc>=10?std::stod(argv[9]):0.0;
        const double range_dropout_probability=argc>=11?
            std::stod(argv[10]):0.0;
        const unsigned int range_random_seed=argc>=12?
            static_cast<unsigned int>(std::stoul(argv[11])):2027U;
        const auto def=makeTemplate(template_id);
        if (policy_task19_switch&&template_id!="origin")
            throw std::invalid_argument(
                "Task 19 switcher must start from production origin DAG");
        auto fixture=makeFixture(def,tau,policy_v2,
            velocity_augmented_rows,policy_v3,policy_v6,
            leader_reachability_filter,policy_h2,gamma_selection,
            service_standoff_m,range_noise_std_m,
            range_dropout_probability,range_random_seed,
            acceleration_half_box_mps2,workspace_class_k,
            workspace_alpha1_gain,workspace_alpha2_gain,
            workspace_braking_acceleration_mps2,
            workspace_braking_regularization_m,target_homotopy_enabled,
            target_homotopy_rate_gain,
            target_homotopy_workspace_guard_m,policy_h2wide,
            policy_task15,policy_task16,task16_arm,
            task16_tracking_envelope_enabled,
            task16_plant_speed_facet_count,
            task16_reference_damping_reserve_multiples,
            task16_speed_row_limit_mps,
            policy_task17,task17_arm,task17_common_governor_enabled,
            task17_reference_compatible_formation,
            task17_member_aware_wide_formation,
            task17_coherent_service_wide_formation,
            task17_update_period_cycles,
            policy_task18,task18_common_governor_enabled,
            task18_collision_only_vaug,
            task18_yaw_objective,gf::task17UpdatePeriodCycles(policy));
        if (!fixture->adapter.initializeStageZero().initialized) {
            // Qualification-gate boundary point: recorded, not run.
            json boundary={{"protocol","task13-phase-a-run-v1"},
                {"preregistration","task-13-phase-a-launch-2026-09-01"},
                {"template",template_id},{"description",def.description},
                {"tau_mps2",tau},{"qualified",false},
                {"boundary_reason","stage_zero_initialization_failed"},
                {"complete",true}};
            gf::writeTask10p11vJson(argv[2],boundary);
            std::cout<<boundary.dump(2)<<'\n';
            return 0;
        }
        std::unique_ptr<gf::Task19OriginMicrofixSwitcher> task19_switcher;
        if (policy_task19_switch)
            task19_switcher=
                std::make_unique<gf::Task19OriginMicrofixSwitcher>(
                    fixture->adapter);
        const auto request0=gf::task10p11zCaptureBeforeOverride(*fixture).
            request;
        const auto rows0=gf::buildCanonicalHardRows(request0);
        json topology_json=json::array();
        for (const auto& edge:def.topology)
            topology_json.push_back(
                std::to_string(edge.reference)+"->"+
                std::to_string(edge.owner));
        json positions_json=json::array();
        for (std::size_t index=0;index<def.positions.size();++index)
            positions_json.push_back({index+1,
                def.positions[index].x(),def.positions[index].y()});
        std::size_t speed_limit_rows=0,plant_facet_rows=0,workspace_rows=0;
        for (const auto& row:rows0) {
            if (row.kind==gf::CanonicalHardRowKind::SpeedLimit)
                ++speed_limit_rows;
            if (row.kind==
                gf::CanonicalHardRowKind::PlantSpeedAppliedControl)
                ++plant_facet_rows;
            if (row.kind==gf::CanonicalHardRowKind::Workspace)
                ++workspace_rows;
        }
        const auto& config=fixture->adapter.config();
        json record={{"protocol",policy_task19_switch
                ?"grand-finale-task19-minimal-dag-switch-v1":
                policy_production
                ?"grand-finale-production-search-v1":policy_task18
                ?"task18-cbf2026-behavioral-recovery-v1":
                policy_task17?"task17-periodic-run-v1":
                "task13-phase-a-run-v1"},
            {"preregistration",policy_production
                ?"task19-production-and-fixed-vs-switching-2026-09-03":
                policy_task18?"task18-cbf2026-behavioral-recovery-2026-09-03":
                policy_task17?"task17-periodic-campaign-2026-09-02":
                "task-13-phase-a-launch-2026-09-01"},
            {"template",template_id},{"description",def.description},
            {"tau_mps2",tau},{"qualified",true},
            {"policy",policy},
            {"gamma_selection",gamma_selection==
                gf::GammaFeedbackSelectionMode::DiagnosticsOnly
                    ?"diagnostics_only":gamma_selection==
                gf::GammaFeedbackSelectionMode::MaximumPredictedMargin
                    ?"maximum_predicted_margin":"least_intervention"},
            {"gamma_selection_code",static_cast<int>(
                fixture->adapter.config().gamma_feedback_selection)},
            {"rows_mode",rows_mode},
            {"window_s",window_s},
            {"production_default_selected",policy_production},
            {"task19_switcher_enabled",policy_task19_switch},
            {"task19_switcher_config",policy_task19_switch?json({
                {"initial_dag","origin"},{"target_dag","microfix"},
                {"coverage_role_mapping","task18-origin-equivalent"},
                {"signal","rolling_cbf_intervention_fraction"},
                {"window_ticks",250},{"minimum_dwell_ticks",2250},
                {"evaluation_period_ticks",50},
                {"trigger_fraction_strictly_greater_than",0.40},
                {"transition","two certified single-edge replacements; "
                    "one fresh SEARCH cycle between replacements"}}):
                json(nullptr)},
            {"task19_switch_events",json::array()},
            {"coverage_checkpoint_s",
                production_defaults.coverage_checkpoint_s},
            {"resource_watchdog_s",window_s},
            {"noise_profile",{{"range_noise_std_m",range_noise_std_m},
                {"range_dropout_probability",range_dropout_probability},
                {"range_random_seed",range_random_seed},
                {"field","counter_based_f_seed_tick_edge"},
                {"maximum_range_aoi_s",config.maximum_range_aoi_s}}},
            {"identity_t0",{{"topology_matches_template",
                fixture->topologyFrozen()}}},
            {"template_topology",std::move(topology_json)},
            {"template_positions",std::move(positions_json)},
            {"runtime_topology_t0",[&]() {
                json edges=json::array();
                for (const auto& edge:
                    fixture->adapter.runtimeSnapshot().topology)
                    edges.push_back(edge.id());
                return edges; }()},
            {"row_counts",{{"speed_limit_rows",speed_limit_rows},
                {"plant_facet_rows",plant_facet_rows},
                {"workspace_rows",workspace_rows},
                {"total",rows0.size()}}},
            {"config_digest",{{"speed_row_nominal",
                config.speed_row_nominal},
                {"speed_row_nominal_limit_mps",
                config.speed_row_nominal_limit_mps},
                {"speed_limit_mps",config.speed_limit_mps},
                {"plant_speed_facet_count",config.plant_speed_facet_count},
                {"speed_rows_removed",config.speed_rows_removed},
                {"nominal_speed_saturation_mps",
                config.nominal_speed_saturation_mps},
                {"speed_preflight_demoted",config.speed_preflight_demoted},
                {"speed_preflight_fuse_mps",
                config.speed_preflight_fuse_mps},
                {"speed_initial_set_truth_gate",
                config.speed_initial_set_truth_gate},
                {"speed_cbf_gain",config.speed_cbf_gain},
                {"range_noise_std_m",config.range_noise_std_m},
                {"range_dropout_probability",
                config.range_dropout_probability},
                {"range_random_seed",config.range_random_seed},
                {"maximum_range_aoi_s",config.maximum_range_aoi_s},
                {"boundary_policy",
                static_cast<int>(config.boundary.policy)},
                {"flight_polygon_source",
                static_cast<int>(config.boundary.flight_polygon_source)},
                {"target_policy_v2",config.target_policy_v2},
                {"demand_recompute_interval_s",
                config.demand_recompute_interval_s},
                {"target_lock_epsilon_m",config.target_lock_epsilon_m},
                {"target_lock_dwell_cycles",config.target_lock_dwell_cycles},
                {"target_lock_progress_epsilon_m",
                config.target_lock_progress_epsilon_m},
                {"reachability_hysteresis_m",
                config.reachability_hysteresis_m},
                {"projection_passes",config.projection_passes},
                {"speed_tracking_gain",config.speed_tracking_gain},
                {"speed_tracking_blend_m",config.speed_tracking_blend_m},
                {"target_policy_v3",config.target_policy_v3},
                {"leader_tie_break_tolerance_m",
                config.leader_tie_break_tolerance_m},
                {"leader_reachability_filter",
                config.leader_reachability_filter},
                {"target_policy_v6",config.target_policy_v6},
                {"v6_neighborhood_radius_m",config.v6_neighborhood_radius_m},
                {"target_policy_unified_h2",
                config.target_policy_unified_h2},
                {"unified_h2_minimum_half_width_m",
                config.unified_h2_minimum_half_width_m},
                {"unified_h2_fan_ratio",config.unified_h2_fan_ratio},
                {"unified_h2_shortlist_per_squad",
                config.unified_h2_shortlist_per_squad},
                {"unified_h2_service_standoff_m",
                config.unified_h2_service_standoff_m},
                {"unified_h2_cbf2026_wide_virtual_formation",
                config.unified_h2_cbf2026_wide_virtual_formation},
                {"target_policy_task15_forward",
                config.target_policy_task15_forward},
                {"target_policy_task16_cbf2026",
                config.target_policy_task16_cbf2026},
                {"task16_coverage_arm",
                static_cast<int>(config.task16_coverage_arm)},
                {"task16_cbf2026_source_commit",
                gf::task16Cbf2026SourceCommit()},
                {"task16_reference_speed_mps",policy_task16
                    ?json(gf::task16AnalyticReferenceSpeedMps(
                        config.speed_limit_mps,config.acceleration_half_box,
                        config.velocity_gain,config.
                            task16_reference_damping_reserve_multiples))
                    :json(nullptr)},
                {"task16_reference_damping_reserve_multiples",
                    config.task16_reference_damping_reserve_multiples},
                {"task16_tracking_envelope_m",policy_task16
                    ?json(gf::task16AnalyticTrackingEnvelopeM(
                        gf::task16AnalyticReferenceSpeedMps(
                            config.speed_limit_mps,
                            config.acceleration_half_box,
                            config.velocity_gain,config.
                                task16_reference_damping_reserve_multiples),
                        config.velocity_gain,config.position_gain,
                        config.speed_limit_mps,
                        config.acceleration_half_box))
                    :json(nullptr)},
                {"task16_tracking_envelope_enabled",
                    config.task16_tracking_envelope_enabled},
                {"target_policy_task17_periodic",
                    config.target_policy_task17_periodic},
                {"task17_periodic_arm",
                    static_cast<int>(config.task17_periodic_arm)},
                {"task17_update_period_cycles",
                    config.task17_update_period_cycles},
                {"task17_common_governor_enabled",
                    config.task17_common_governor_enabled},
                {"task17_reference_compatible_formation",
                    config.task17_reference_compatible_formation},
                {"task17_member_aware_wide_formation",
                    config.task17_member_aware_wide_formation},
                {"task17_coherent_service_wide_formation",
                    config.task17_coherent_service_wide_formation},
                {"target_policy_task18_cbf2026_outer",
                    config.target_policy_task18_cbf2026_outer},
                {"task18_update_period_cycles",
                    config.task18_update_period_cycles},
                {"task18_common_governor_enabled",
                    config.task18_common_governor_enabled},
                {"task18_collision_only_vaug",
                    config.task18_collision_only_vaug},
                {"task18_yaw_objective",
                    static_cast<int>(config.task18_yaw_objective)},
                {"task15_forward_shortlist_capacity",
                config.task15_forward_shortlist_capacity},
                {"task15_forward_update_period_cycles",
                config.task15_forward_update_period_cycles},
                {"task15_forward_endpoint_standoff_m",
                config.task15_forward_endpoint_standoff_m},
                {"velocity_augmented_rows",config.velocity_augmented_rows},
                {"velocity_augmented_rows",config.velocity_augmented_rows},
                {"row_slack_epsilon_m",config.row_slack_epsilon_m},
                {"acceleration_half_box",config.acceleration_half_box},
                {"workspace_class_k",workspace_class_k_name},
                {"workspace_alpha1_gain",config.workspace_alpha1_gain},
                {"workspace_alpha2_gain",config.workspace_alpha2_gain},
                {"workspace_braking_acceleration_mps2",
                    config.workspace_braking_acceleration_mps2},
                {"workspace_braking_regularization_m",
                    config.workspace_braking_regularization_m},
                {"target_homotopy_enabled",
                    config.target_homotopy_enabled},
                {"target_homotopy_braking_acceleration_mps2",
                    config.target_homotopy_braking_acceleration_mps2},
                {"target_homotopy_rate_gain",
                    config.target_homotopy_rate_gain},
                {"target_homotopy_workspace_guard_m",
                    config.target_homotopy_workspace_guard_m},
                {"residual_tolerance",config.residual_tolerance},
                {"dt_s",config.dt_s},
                {"truth_initial_set_gate_mps",
                (policy_h2||policy_task15||policy_task16||policy_task17||
                    policy_task18)
                    ?30.0:30.01}}},
            {"s3_stop_rule",
                "terminate_at_certified_t100_else_budget_censored"},
            {"evaluations",json::array()},
            {"complete",false}};
        std::filesystem::create_directories(argv[3]);
        gf::writeTask10p11vJson(
            std::filesystem::path(argv[3])/"00-config.json",record);
        std::ofstream telemetry(argv[4]);
        if (!telemetry) throw std::runtime_error(
            "cannot open telemetry stream");
        const std::filesystem::path grid_delta_path=
            std::filesystem::path(argv[3])/"gridworld-delta.jsonl";
        std::ofstream grid_delta(grid_delta_path);
        if (!grid_delta) throw std::runtime_error(
            "cannot open Task 17 GridWorld delta stream");
        GridWorld previous_certified=
            fixture->adapter.coverage().certifiedGrid();
        GridWorld previous_truth=fixture->adapter.coverage().truthGrid();
        const auto initial_grid=gf::task17GridSnapshot(
            previous_certified,previous_truth);
        gf::Task19EfficiencyAccumulator efficiency(
            initial_grid.valid_count,fixture->adapter.config().dt_s);
        grid_delta<<json({{"kind","initial"},
            {"x_cells",initial_grid.x_cells},
            {"y_cells",initial_grid.y_cells},
            {"valid_count",initial_grid.valid_count},
            {"x_limits",{previous_certified.xLim.first,
                         previous_certified.xLim.second}},
            {"y_limits",{previous_certified.yLim.first,
                         previous_certified.yLim.second}},
            {"valid_bits_hex",initial_grid.valid_bits_hex},
            {"certified_bits_hex",initial_grid.certified_bits_hex},
            {"truth_bits_hex",initial_grid.truth_bits_hex},
            {"certified_count",initial_grid.certified_count},
            {"truth_count",initial_grid.truth_count},
            {"certified_hash",initial_grid.certified_hash},
            {"truth_hash",initial_grid.truth_hash}}).dump()<<'\n';
        double max_truth_overspeed=0.0,max_interval_overspeed=0.0;
        std::size_t truth_overspeed_ticks=0,interval_overspeed_ticks=0;
        std::optional<std::size_t> t95_tick,t100_tick;
        std::optional<std::size_t> certified_checkpoint_500_count;
        std::optional<std::size_t> truth_checkpoint_500_count;
        std::size_t tick=0;
        bool hard_stop=false;
        bool truth_gate_violation=false;
        bool runtime_safety_violation=false;
        std::string runtime_safety_reason;
        double truth_gate_max_speed=0.0;
        double maximum_actual_reference_m=0.0;
        double maximum_target_reference_m=0.0;
        double minimum_actual_separation_m=
            std::numeric_limits<double>::infinity();
        double minimum_target_separation_m=
            std::numeric_limits<double>::infinity();
        double minimum_robust_fim=
            std::numeric_limits<double>::infinity();
        double maximum_posterior_m2=0.0;
        double minimum_aoi_margin_s=
            std::numeric_limits<double>::infinity();
        double minimum_gamma=std::numeric_limits<double>::infinity();
        double minimum_qp_residual=std::numeric_limits<double>::infinity();
        double maximum_speed_mps=0.0;
        double maximum_axis_control_mps2=0.0;
        double maximum_acceleration_norm_mps2=0.0;
        double minimum_robust_reference_margin_m=
            std::numeric_limits<double>::infinity();
        double minimum_robust_separation_margin_m=
            std::numeric_limits<double>::infinity();
        double minimum_robust_workspace_margin_m=
            std::numeric_limits<double>::infinity();
        std::size_t intervention_owner_ticks=0,total_owner_ticks=0;
        std::size_t target_switches=0,last_target_epoch=0;
        std::size_t active_squad_ticks=0,current_active_squads=0;
        double minimum_target_governor_fraction=1.0;
        double minimum_target_governor_stopping_margin_m=
            std::numeric_limits<double>::infinity();
        std::size_t target_governor_ticks=0;
        int highest_selected_task_row=-1;
        Eigen::Vector2d nominal_target_min=Eigen::Vector2d::Constant(
            std::numeric_limits<double>::infinity());
        Eigen::Vector2d nominal_target_max=Eigen::Vector2d::Constant(
            -std::numeric_limits<double>::infinity());
        Eigen::Vector2d applied_target_min=nominal_target_min;
        Eigen::Vector2d applied_target_max=nominal_target_max;
        Eigen::Vector2d actual_position_min=nominal_target_min;
        Eigen::Vector2d actual_position_max=nominal_target_max;
        double total_distance_m=0.0;
        double squared_control_energy_proxy=0.0;
        double maximum_outside_distance_m=0.0;
        std::vector<double> task16_allocation_wall_s;
        std::vector<double> task19_scheduler_wall_s;
        gf::SimpleCoverageControlStep last_step;
        gf::Task10p11ComputeProfile profiler;
        const auto mobile_ids=
            fixture->adapter.runtimeSnapshot().estimate.mobile_ids;
        const auto robot_by_id=[&](gf::NodeId id)->Robot& {
            for (const auto& robot:fixture->swarm.robots)
                if (static_cast<gf::NodeId>(robot->id)==id) return *robot;
            throw std::runtime_error("unknown robot id");
        };
        while (fixture->adapter.runtimeSnapshot().runtime_s<window_s) {
            if (fixture->adapter.config().speed_initial_set_truth_gate) {
                truth_gate_max_speed=0.0;
                for (const auto& robot:fixture->swarm.robots) {
                    const Eigen::VectorXd tv=robot->model->getVelocity();
                    truth_gate_max_speed=std::max(truth_gate_max_speed,
                        tv.head<2>().norm());
                }
                const double hard_speed_limit=
                    (policy_h2||policy_task15||policy_task16||policy_task17||
                        policy_task18)
                        ?30.0:30.01;
                if (truth_gate_max_speed>hard_speed_limit+1e-9) {
                    truth_gate_violation=true;
                    hard_stop=true;
                    record["evaluations"].push_back({{"tick",tick},
                        {"advanced",false},
                        {"reason","speed_initial_set_truth_violated"},
                        {"coverage_fraction",
                        (policy_h2||policy_task15||policy_task16||
                            policy_task17||policy_task18)
                            ?fixture->adapter.coverage().certifiedFraction()
                            :fixture->adapter.coverage().truthFraction()},
                        {"minimum_hard_residual",json(nullptr)}});
                    break;
                }
            }
            // Interval-audit fix (B0-a code round): capture the PRE-advance
            // truth velocity; the retired form paired post-advance velocity
            // with the same tick's control (a two-tick projection).
            std::map<gf::NodeId,Eigen::Vector2d> pre_velocities;
            std::map<gf::NodeId,Eigen::Vector2d> pre_positions;
            for (const auto& robot:fixture->swarm.robots)
                {
                const auto id=static_cast<gf::NodeId>(robot->id);
                pre_velocities[id]=Eigen::Vector2d(
                    robot->model->getVelocity().head<2>());
                pre_positions[id]={robot->model->getStateVariable("x"),
                    robot->model->getStateVariable("y")};
                }
            last_step=fixture->controller.advance();
            if (last_step.step.advanced)
                squared_control_energy_proxy+=
                    gf::task19SquaredControlEnergyIncrement(
                        last_step.step.applied_controls,config.dt_s);
            if (last_step.task16_allocation_evaluated&&
                last_step.task16_allocation.valid)
                task16_allocation_wall_s.push_back(
                    last_step.task16_allocation.allocation_wall_s);
            if (last_step.task17_allocation_evaluated&&
                last_step.task17_allocation.valid)
                task16_allocation_wall_s.push_back(
                    last_step.task17_allocation.allocation_wall_s);
            if (last_step.task18_allocation_evaluated&&
                last_step.task18_allocation.valid)
                task16_allocation_wall_s.push_back(
                    last_step.task18_allocation.allocation_wall_s);
            if (last_step.target_governor_evaluated) {
                ++target_governor_ticks;
                minimum_target_governor_fraction=std::min(
                    minimum_target_governor_fraction,
                    last_step.target_governor_common_fraction);
                minimum_target_governor_stopping_margin_m=std::min(
                    minimum_target_governor_stopping_margin_m,
                    last_step.target_governor_minimum_stopping_margin_m);
            }
            profiler.merge(last_step.compute_profile);
            const double certified_fraction=
                fixture->adapter.coverage().certifiedFraction();
            const double truth_fraction=
                fixture->adapter.coverage().truthFraction();
            const GridWorld& current_certified=
                fixture->adapter.coverage().certifiedGrid();
            const GridWorld& current_truth=
                fixture->adapter.coverage().truthGrid();
            const auto grid_tick=gf::task17GridDelta(previous_certified,
                previous_truth,current_certified,current_truth);
            efficiency.observe(grid_tick.certified_count);
            grid_delta<<json({{"kind","delta"},{"tick",tick},
                {"runtime_s",fixture->adapter.runtimeSnapshot().runtime_s},
                {"certified_new_ids",grid_tick.certified_new_ids},
                {"truth_new_ids",grid_tick.truth_new_ids},
                {"certified_count",grid_tick.certified_count},
                {"truth_count",grid_tick.truth_count}}).dump()<<'\n';
            previous_certified=current_certified;
            previous_truth=current_truth;
            if (!certified_checkpoint_500_count.has_value()&&
                fixture->adapter.runtimeSnapshot().runtime_s>=
                    production_defaults.coverage_checkpoint_s-1.0e-9) {
                certified_checkpoint_500_count=grid_tick.certified_count;
                truth_checkpoint_500_count=grid_tick.truth_count;
            }
            const double fraction=(policy_h2||policy_task15||policy_task16||
                policy_task17||policy_task18)
                ?certified_fraction:truth_fraction;
            if (!t95_tick.has_value()&&fraction>=0.95) t95_tick=tick;
            if (!t100_tick.has_value()&&fraction>=1.0-1e-12) t100_tick=tick;
            if (last_step.step.advanced) {
                double truth_max=0.0,interval_max=0.0;
                for (const auto& robot:fixture->swarm.robots) {
                    const Eigen::VectorXd truth_velocity=
                        robot->model->getVelocity();
                    const Eigen::Vector2d u=
                        last_step.step.applied_controls.at(
                            static_cast<gf::NodeId>(robot->id));
                    const double truth_speed=truth_velocity.head<2>().norm();
                    truth_max=std::max(truth_max,truth_speed-30.0);
                    const auto audit=gf::auditPlantSpeedExactZoh(
                        pre_velocities.at(static_cast<gf::NodeId>(
                            robot->id)),u,0.1,30.0,1.0e-9);
                    interval_max=std::max(interval_max,
                        audit.maximum_interval_speed_mps-30.0);
                }
                max_truth_overspeed=std::max(max_truth_overspeed,truth_max);
                max_interval_overspeed=std::max(max_interval_overspeed,
                    interval_max);
                if (truth_max>0.0) ++truth_overspeed_ticks;
                if (interval_max>0.0) ++interval_overspeed_ticks;
                if ((policy_h2||policy_task15||policy_task16||policy_task17||
                    policy_task18) &&
                    (truth_max>1e-9||interval_max>1e-9)) {
                    runtime_safety_violation=true;
                    runtime_safety_reason="hard_speed_limit_violated";
                    hard_stop=true;
                }
            } else {
                hard_stop=true;
            }
            const auto snapshot=fixture->adapter.runtimeSnapshot();
            for (const auto& row:
                 fixture->adapter.currentSnapshotHardRows(snapshot.topology)) {
                if (row.kind==gf::CanonicalHardRowKind::ReferenceDistance)
                    minimum_robust_reference_margin_m=std::min(
                        minimum_robust_reference_margin_m,row.barrier_h);
                else if (row.kind==gf::CanonicalHardRowKind::Collision)
                    minimum_robust_separation_margin_m=std::min(
                        minimum_robust_separation_margin_m,row.barrier_h);
                else if (row.kind==gf::CanonicalHardRowKind::Workspace)
                    minimum_robust_workspace_margin_m=std::min(
                        minimum_robust_workspace_margin_m,row.barrier_h);
            }
            const auto raw_nominals=
                fixture->controller.lastNominalControls();
            const auto throttle=fixture->adapter.lastThrottleTelemetry();
            if (last_step.unified_allocation_evaluated&&
                last_step.unified_allocation.valid)
                current_active_squads=
                    last_step.unified_allocation.active_squads;
            if (last_step.task15_allocation_evaluated&&
                last_step.task15_allocation.valid)
                current_active_squads=
                    last_step.task15_allocation.active_squads;
            if (last_step.task16_allocation_evaluated&&
                last_step.task16_allocation.valid)
                current_active_squads=
                    last_step.task16_allocation.active_squads;
            if (last_step.task17_allocation_evaluated&&
                last_step.task17_allocation.valid)
                current_active_squads=
                    last_step.task17_allocation.active_squads;
            if (last_step.task18_allocation_evaluated&&
                last_step.task18_allocation.valid)
                current_active_squads=
                    last_step.task18_allocation.active_squads;
            active_squad_ticks+=current_active_squads;
            if (last_step.target_epoch!=last_target_epoch) {
                if (last_target_epoch!=0) ++target_switches;
                last_target_epoch=last_step.target_epoch;
            }
            const auto information=fixture->adapter.currentReferenceAudit();
            minimum_robust_fim=std::min(minimum_robust_fim,
                information.minimum_robust_fim_cone_lower_bound);
            maximum_posterior_m2=std::max(maximum_posterior_m2,
                information.maximum_posterior_eigenvalue);
            minimum_aoi_margin_s=std::min(minimum_aoi_margin_s,
                information.minimum_range_aoi_margin_s);
            minimum_qp_residual=std::min(minimum_qp_residual,
                last_step.step.minimum_hard_residual);
            std::map<gf::NodeId,Eigen::Vector2d> truth_positions;
            for (const auto& robot:fixture->swarm.robots) {
                const gf::NodeId id=static_cast<gf::NodeId>(robot->id);
                truth_positions[id]={robot->model->getStateVariable("x"),
                    robot->model->getStateVariable("y")};
                actual_position_min=actual_position_min.cwiseMin(
                    truth_positions[id]);
                actual_position_max=actual_position_max.cwiseMax(
                    truth_positions[id]);
                const double dx=std::max({0.0,-truth_positions[id].x(),
                    truth_positions[id].x()-3000.0});
                const double dy=std::max({0.0,-truth_positions[id].y(),
                    truth_positions[id].y()-3000.0});
                maximum_outside_distance_m=std::max(
                    maximum_outside_distance_m,std::hypot(dx,dy));
                total_distance_m+=(truth_positions[id]-
                    pre_positions.at(id)).norm();
                maximum_speed_mps=std::max(maximum_speed_mps,
                    robot->model->getVelocity().head<2>().norm());
            }
            const auto actual_position=[&](gf::NodeId id) {
                const auto mobile=truth_positions.find(id);
                if (mobile!=truth_positions.end()) return mobile->second;
                return snapshot.estimate.fixed_positions.at(id);
            };
            const auto target_position=[&](gf::NodeId id) {
                const auto applied=last_step.applied_target_centers.find(id);
                if (applied!=last_step.applied_target_centers.end())
                    return applied->second;
                const auto committed=last_step.committed_targets.find(id);
                if (committed!=last_step.committed_targets.end())
                    return committed->second.center;
                return snapshot.estimate.fixed_positions.at(id);
            };
            double tick_actual_reference=0.0,tick_target_reference=0.0;
            for (const auto& edge:snapshot.topology) {
                tick_actual_reference=std::max(tick_actual_reference,
                    (actual_position(edge.owner)-
                     actual_position(edge.reference)).norm());
                if (last_step.committed_targets.size()==14)
                    tick_target_reference=std::max(tick_target_reference,
                        (target_position(edge.owner)-
                         target_position(edge.reference)).norm());
            }
            double tick_actual_separation=
                std::numeric_limits<double>::infinity();
            double tick_target_separation=
                std::numeric_limits<double>::infinity();
            for (std::size_t first=0;first<mobile_ids.size();++first) {
                for (std::size_t second=first+1;second<mobile_ids.size();
                     ++second) {
                    tick_actual_separation=std::min(tick_actual_separation,
                        (actual_position(mobile_ids[first])-
                         actual_position(mobile_ids[second])).norm());
                    if (last_step.committed_targets.size()==14)
                        tick_target_separation=std::min(tick_target_separation,
                            (target_position(mobile_ids[first])-
                             target_position(mobile_ids[second])).norm());
                }
                for (const auto& [fixed_id,fixed_position]:
                     snapshot.estimate.fixed_positions) {
                    tick_actual_separation=std::min(tick_actual_separation,
                        (actual_position(mobile_ids[first])-
                         fixed_position).norm());
                    if (last_step.committed_targets.size()==14)
                        tick_target_separation=std::min(tick_target_separation,
                            (target_position(mobile_ids[first])-
                             fixed_position).norm());
                    (void)fixed_id;
                }
            }
            maximum_actual_reference_m=std::max(
                maximum_actual_reference_m,tick_actual_reference);
            maximum_target_reference_m=std::max(
                maximum_target_reference_m,tick_target_reference);
            minimum_actual_separation_m=std::min(
                minimum_actual_separation_m,tick_actual_separation);
            minimum_target_separation_m=std::min(
                minimum_target_separation_m,tick_target_separation);
            for (const auto& [owner,target]:last_step.committed_targets) {
                (void)owner;
                highest_selected_task_row=std::max(
                    highest_selected_task_row,target.y_index);
                nominal_target_min=nominal_target_min.cwiseMin(target.center);
                nominal_target_max=nominal_target_max.cwiseMax(target.center);
            }
            for (const auto& [owner,target]:
                 last_step.applied_target_centers) {
                (void)owner;
                applied_target_min=applied_target_min.cwiseMin(target);
                applied_target_max=applied_target_max.cwiseMax(target);
            }
            if ((policy_h2||policy_task15||policy_task16||policy_task17||
                policy_task18) &&
                (tick_actual_reference>850.0+1e-9 ||
                    tick_actual_separation<10.0-1e-9 ||
                    (policy_h2 && !policy_h2wide &&
                        (tick_target_reference>=850.0-1e-9 ||
                         tick_target_separation<=10.0+1e-9)))) {
                runtime_safety_violation=true;
                runtime_safety_reason="reference_or_separation_violated";
                hard_stop=true;
            }
            json owners=json::array();
            for (gf::NodeId owner:mobile_ids) {
                const Eigen::Index off=4*static_cast<Eigen::Index>(
                    std::distance(mobile_ids.begin(),
                        std::find(mobile_ids.begin(),mobile_ids.end(),
                            owner)));
                const Eigen::Vector2d estimate_position=
                    snapshot.estimate.mean.segment<2>(off);
                const Eigen::Vector2d estimate_velocity=
                    snapshot.estimate.mean.segment<2>(off+2);
                json diagnostic_json=json(nullptr);
                const auto diagnostic=
                    last_step.step.gamma_feedback.find(owner);
                if (diagnostic!=last_step.step.gamma_feedback.end())
                    {
                    minimum_gamma=std::min(minimum_gamma,
                        diagnostic->second.current_gamma);
                    intervention_owner_ticks+=diagnostic->second.intervened;
                    ++total_owner_ticks;
                    diagnostic_json={{"n_sel",{
                        diagnostic->second.selected_nominal.x(),
                        diagnostic->second.selected_nominal.y()}},
                        {"n_projected",{
                            diagnostic->second.current_hard_projection.x(),
                            diagnostic->second.current_hard_projection.y()}},
                        {"gamma",gf::task10p11w_detail::number(
                            diagnostic->second.current_gamma)},
                        {"intervened",diagnostic->second.intervened},
                        {"fallback",diagnostic->second.fallback_reason},
                        {"dominant_row",diagnostic->second.dominant_row}};
                    }
                json applied_json=json(nullptr);
                const auto applied=
                    last_step.step.applied_controls.find(owner);
                if (applied!=last_step.step.applied_controls.end())
                    {
                    maximum_axis_control_mps2=std::max(
                        maximum_axis_control_mps2,
                        applied->second.cwiseAbs().maxCoeff());
                    maximum_acceleration_norm_mps2=std::max(
                        maximum_acceleration_norm_mps2,
                        applied->second.norm());
                    applied_json={applied->second.x(),applied->second.y()};
                    }
                json raw_json=json(nullptr);
                const auto raw=raw_nominals.find(owner);
                if (raw!=raw_nominals.end())
                    raw_json={raw->second.x(),raw->second.y()};
                json target=json(nullptr);
                const auto committed=last_step.committed_targets.find(owner);
                if (committed!=last_step.committed_targets.end())
                    target={{"id",committed->second.id()},
                        {"nominal_center",{committed->second.center.x(),
                            committed->second.center.y()}},
                        {"applied_center",[&]() {
                            const auto applied_target=
                                last_step.applied_target_centers.find(owner);
                            return applied_target==
                                last_step.applied_target_centers.end()
                                ?json(nullptr):json({applied_target->second.x(),
                                    applied_target->second.y()}); }()}};
                const Robot& robot=robot_by_id(owner);
                const double actual_yaw=
                    robot.model->getStateVariable("yawRad");
                const Eigen::Vector2d actual_velocity{
                    robot.model->getStateVariable("vx"),
                    robot.model->getStateVariable("vy")};
                const json velocity_heading=actual_velocity.norm()>1.0e-9
                    ?json(std::atan2(actual_velocity.y(),actual_velocity.x()))
                    :json(nullptr);
                const json yaw_velocity_error=actual_velocity.norm()>1.0e-9
                    ?json(std::abs(gf::wrapYawRad(actual_yaw-
                        std::atan2(actual_velocity.y(),actual_velocity.x()))))
                    :json(nullptr);
                const auto desired=last_step.desired_yaw_rad.find(owner);
                const auto task_bearing=last_step.task_bearing_rad.find(owner);
                const auto applied_yaw_rate=
                    last_step.step.applied_yaw_rates_radps.find(owner);
                owners.push_back({{"id",owner},
                    {"est",{estimate_position.x(),estimate_position.y(),
                        estimate_velocity.x(),estimate_velocity.y()}},
                    {"truth",{robot.model->getStateVariable("x"),
                        robot.model->getStateVariable("y"),
                        robot.model->getStateVariable("vx"),
                        robot.model->getStateVariable("vy")}},
                    {"u_applied",std::move(applied_json)},
                    {"n_raw",std::move(raw_json)},
                    {"n_sel_gamma",std::move(diagnostic_json)},
                    {"yaw",{{"actual_rad",actual_yaw},
                        {"desired_rad",desired==
                            last_step.desired_yaw_rad.end()?json(nullptr):
                            json(desired->second)},
                        {"rate_radps",applied_yaw_rate==
                            last_step.step.applied_yaw_rates_radps.end()
                            ?json(nullptr):json(applied_yaw_rate->second)},
                        {"velocity_heading_rad",velocity_heading},
                        {"yaw_velocity_abs_error_rad",yaw_velocity_error},
                        {"task_bearing_rad",task_bearing==
                            last_step.task_bearing_rad.end()?json(nullptr):
                            json(task_bearing->second)}}},
                    {"target",std::move(target)}});
            }
            gf::Task19DagSwitchEvent task19_switch_event;
            if (task19_switcher) {
                const auto scheduler_started=
                    std::chrono::steady_clock::now();
                task19_switch_event=task19_switcher->afterStep(
                    tick,last_step.step);
                task19_scheduler_wall_s.push_back(
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now()-scheduler_started).
                            count());
                if (task19_switch_event.signal_evaluated||
                    task19_switch_event.switch_requested||
                    task19_switch_event.transition_started||
                    task19_switch_event.transition_finished)
                    record["task19_switch_events"].push_back({
                        {"tick",tick},{"runtime_s",snapshot.runtime_s},
                        {"signal_evaluated",
                            task19_switch_event.signal_evaluated},
                        {"switch_requested",
                            task19_switch_event.switch_requested},
                        {"transition_started",
                            task19_switch_event.transition_started},
                        {"transition_finished",
                            task19_switch_event.transition_finished},
                        {"rolling_intervention_fraction",
                            task19_switch_event.
                                rolling_intervention_fraction},
                        {"reason",task19_switch_event.reason}});
            }
            telemetry<<json({{"tick",tick},
                {"runtime_s",snapshot.runtime_s},
                {"coverage_fraction",fraction},
                {"certified_coverage_fraction",certified_fraction},
                {"truth_coverage_fraction",truth_fraction},
                {"geometry",{{"maximum_actual_reference_m",
                    tick_actual_reference},{"maximum_target_reference_m",
                    tick_target_reference},{"minimum_actual_separation_m",
                    tick_actual_separation},{"minimum_target_separation_m",
                    tick_target_separation}}},
                {"information",{{"minimum_robust_fim",
                    gf::task10p11w_detail::number(
                        information.minimum_robust_fim_cone_lower_bound)},
                    {"maximum_posterior_m2",
                    gf::task10p11w_detail::number(
                        information.maximum_posterior_eigenvalue)},
                    {"minimum_aoi_margin_s",gf::task10p11w_detail::number(
                        information.minimum_range_aoi_margin_s)}}},
                {"minimum_qp_residual",gf::task10p11w_detail::number(
                    last_step.step.minimum_hard_residual)},
                {"robust_margins",{{"reference_m",
                    gf::task10p11w_detail::number(
                        minimum_robust_reference_margin_m)},
                    {"separation_m",gf::task10p11w_detail::number(
                        minimum_robust_separation_margin_m)},
                    {"workspace_m",gf::task10p11w_detail::number(
                        minimum_robust_workspace_margin_m)}}},
                {"active_squads",current_active_squads},
                {"advanced",last_step.step.advanced},
                {"reason",last_step.step.reason},
                {"target_epoch",last_step.target_epoch},
                {"target_governor",{{"evaluated",
                    last_step.target_governor_evaluated},
                    {"common_fraction",
                    last_step.target_governor_common_fraction},
                    {"minimum_stopping_margin_m",
                    gf::task10p11w_detail::number(last_step.
                        target_governor_minimum_stopping_margin_m)},
                    {"reselect_required",
                    last_step.target_governor_reselect_required},
                    {"feasibility_evaluations",
                    last_step.target_governor_feasibility_evaluations}}},
                {"task16",{{"allocation_evaluated",
                    last_step.task16_allocation_evaluated},
                    {"allocation_wall_s",
                    last_step.task16_allocation_evaluated
                        ?json(last_step.task16_allocation.allocation_wall_s)
                        :json(nullptr)},
                    {"scanned_member_cell_pairs",
                    last_step.task16_allocation.scanned_member_cell_pairs},
                    {"governor_evaluated",
                    last_step.task16_governor_evaluated},
                    {"common_fraction",last_step.task16_common_fraction},
                    {"stalled_squads",last_step.task16_stalled_squads}}},
                {"task16_governor_rejections",
                    last_step.task16_governor_rejections},
                {"task17",{{"allocation_evaluated",
                    last_step.task17_allocation_evaluated},
                    {"allocation_wall_s",
                    last_step.task17_allocation_evaluated
                        ?json(last_step.task17_allocation.allocation_wall_s)
                        :json(nullptr)},
                    {"scanned_member_cell_pairs",
                    last_step.task17_allocation.scanned_member_cell_pairs},
                    {"duplicate_task_count",
                    last_step.task17_allocation.duplicate_task_count},
                    {"common_fraction",last_step.task17_common_fraction}}},
                {"task18",{{"allocation_evaluated",
                    last_step.task18_allocation_evaluated},
                    {"allocation_wall_s",
                    last_step.task18_allocation_evaluated
                        ?json(last_step.task18_allocation.allocation_wall_s)
                        :json(nullptr)},
                    {"common_fraction",last_step.task18_common_fraction}}},
                {"task19_switch",{{"enabled",policy_task19_switch},
                    {"signal_evaluated",
                        task19_switch_event.signal_evaluated},
                    {"switch_requested",
                        task19_switch_event.switch_requested},
                    {"transition_started",
                        task19_switch_event.transition_started},
                    {"transition_finished",
                        task19_switch_event.transition_finished},
                    {"rolling_intervention_fraction",
                        task19_switch_event.rolling_intervention_fraction},
                    {"reason",task19_switch_event.reason}}},
                {"throttle",{{"active",throttle.active},
                    {"s",throttle.s}}},
                {"owners",std::move(owners)}}).dump()<<'\n';
            if (tick%100==0||hard_stop)
                record["evaluations"].push_back({{"tick",tick},
                    {"advanced",last_step.step.advanced},
                    {"reason",last_step.step.reason},
                    {"coverage_fraction",fraction},
                    {"minimum_hard_residual",gf::task10p11w_detail::number(
                        last_step.step.minimum_hard_residual)}});
            if (hard_stop) break;
            ++tick;
            if (t100_tick.has_value()) break;
        }
        telemetry.close();
        const auto final_grid=gf::task17GridSnapshot(
            fixture->adapter.coverage().certifiedGrid(),
            fixture->adapter.coverage().truthGrid());
        grid_delta<<json({{"kind","final"},{"tick",tick},
            {"runtime_s",fixture->adapter.runtimeSnapshot().runtime_s},
            {"valid_count",final_grid.valid_count},
            {"certified_bits_hex",final_grid.certified_bits_hex},
            {"truth_bits_hex",final_grid.truth_bits_hex},
            {"certified_count",final_grid.certified_count},
            {"truth_count",final_grid.truth_count},
            {"certified_hash",final_grid.certified_hash},
            {"truth_hash",final_grid.truth_hash}}).dump()<<'\n';
        grid_delta.close();
        json profiler_json=json::array();
        static const std::vector<gf::Task10p11ComputePhase> phases={
            gf::Task10p11ComputePhase::CandidateBundleConstruction,
            gf::Task10p11ComputePhase::EstimatorPropagation,
            gf::Task10p11ComputePhase::CanonicalRowRebuild,
            gf::Task10p11ComputePhase::ExactHardProjection,
            gf::Task10p11ComputePhase::CurrentGamma,
            gf::Task10p11ComputePhase::PredictedGamma,
            gf::Task10p11ComputePhase::RobustQpSetup,
            gf::Task10p11ComputePhase::RobustQpSolve,
            gf::Task10p11ComputePhase::ResidualTokenAudit,
            gf::Task10p11ComputePhase::SolverInitialization,
            gf::Task10p11ComputePhase::SolverModelUpdate,
            gf::Task10p11ComputePhase::GridWorldTarget,
            gf::Task10p11ComputePhase::CurrentCanonicalRowRebuild,
            gf::Task10p11ComputePhase::PredictedCanonicalRowRebuild,
            gf::Task10p11ComputePhase::FinalQp,
            gf::Task10p11ComputePhase::InformationAudit,
            gf::Task10p11ComputePhase::TruthOnlyAudit,
            gf::Task10p11ComputePhase::OnlineEstimator,
            gf::Task10p11ComputePhase::PlantPreflightZoh};
        double profiled_total=0.0;
        for (const auto phase:phases) {
            const auto summary=profiler.summary(phase);
            if (summary.calls==0) continue;
            profiled_total+=summary.total_s;
            profiler_json.push_back({{"phase",
                gf::task10p11ComputePhaseName(phase)},
                {"calls",summary.calls},
                {"total_s",gf::task10p11w_detail::number(summary.total_s)},
                {"median_s",gf::task10p11w_detail::number(
                    summary.median_s)},
                {"p95_s",gf::task10p11w_detail::number(summary.p95_s)},
                {"maximum_s",gf::task10p11w_detail::number(
                    summary.maximum_s)}});
        }
        record["t95_tick"]=(t95_tick.has_value()?json(*t95_tick):
            json(nullptr));
        record["t100_tick"]=(t100_tick.has_value()?json(*t100_tick):
            json(nullptr));
        record["t50_tick"]=efficiency.t50_tick().has_value()
            ?json(*efficiency.t50_tick()):json(nullptr);
        record["t99_tick"]=efficiency.t99_tick().has_value()
            ?json(*efficiency.t99_tick()):json(nullptr);
        record["j_uncovered_cell_seconds"]=
            efficiency.j_uncovered_cell_seconds();
        if (!certified_checkpoint_500_count.has_value()&&
            t100_tick.has_value()&&
            *t100_tick<=static_cast<std::size_t>(std::llround(
                production_defaults.coverage_checkpoint_s/config.dt_s))) {
            certified_checkpoint_500_count=final_grid.certified_count;
            truth_checkpoint_500_count=final_grid.truth_count;
        }
        record["coverage_checkpoint_500_s"]={
            {"certified_count",certified_checkpoint_500_count.has_value()
                ?json(*certified_checkpoint_500_count):json(nullptr)},
            {"truth_count",truth_checkpoint_500_count.has_value()
                ?json(*truth_checkpoint_500_count):json(nullptr)},
            {"status",certified_checkpoint_500_count.has_value()
                ?(t100_tick.has_value()&&
                  *t100_tick<=static_cast<std::size_t>(std::llround(
                    production_defaults.coverage_checkpoint_s/config.dt_s))
                    ?"complete_before_checkpoint":"observed")
                :"not_observed"}};
        record["final_coverage_fraction"]=
            (policy_h2||policy_task15||policy_task16||policy_task17||
                policy_task18)
                ?fixture->adapter.coverage().certifiedFraction():
                fixture->adapter.coverage().truthFraction();
        record["final_certified_coverage_fraction"]=
            fixture->adapter.coverage().certifiedFraction();
        record["final_truth_coverage_fraction"]=
            fixture->adapter.coverage().truthFraction();
        record["cycles"]=tick;
        record["hard_stop"]=hard_stop;
        record["failure"]={
            {"advanced",truth_gate_violation?false:last_step.step.advanced},
            {"reason",truth_gate_violation?
                json("speed_initial_set_truth_violated"):
                runtime_safety_violation?json(runtime_safety_reason):
                json(last_step.step.reason)},
            {"truth_gate_max_speed_mps",truth_gate_violation?
                json(truth_gate_max_speed):json(nullptr)},
            {"same_reason_as_fixed_132p4",!truth_gate_violation&&
                last_step.step.reason=="current_gamma_negative"}};
        record["realism"]={{"max_truth_overspeed_mps",
            gf::task10p11w_detail::number(max_truth_overspeed)},
            {"truth_overspeed_ticks",truth_overspeed_ticks},
            {"max_interval_overspeed_mps",
            gf::task10p11w_detail::number(max_interval_overspeed)},
            {"interval_overspeed_ticks",interval_overspeed_ticks},
            {"fuse_limit_mps",config.speed_preflight_fuse_mps},
            {"fuse_tripped",max_interval_overspeed>1.0}};
        const auto task16_percentile=[&](double q)->json {
            if (task16_allocation_wall_s.empty()) return nullptr;
            auto values=task16_allocation_wall_s;
            std::sort(values.begin(),values.end());
            const std::size_t index=static_cast<std::size_t>(std::floor(
                q*static_cast<double>(values.size()-1)));
            return values[index];
        };
        record["task16_trajectory"]={
            {"highest_selected_task_row",highest_selected_task_row},
            {"nominal_target_coordinate_range",{
                {"minimum",{nominal_target_min.x(),nominal_target_min.y()}},
                {"maximum",{nominal_target_max.x(),nominal_target_max.y()}}}},
            {"applied_target_coordinate_range",{
                {"minimum",{applied_target_min.x(),applied_target_min.y()}},
                {"maximum",{applied_target_max.x(),applied_target_max.y()}}}},
            {"actual_coordinate_range",{
                {"minimum",{actual_position_min.x(),actual_position_min.y()}},
                {"maximum",{actual_position_max.x(),actual_position_max.y()}}}},
            {"maximum_outside_distance_m",
                maximum_outside_distance_m},
            {"total_distance_m",total_distance_m},
            {"squared_control_energy_proxy",
                squared_control_energy_proxy},
            {"allocator_calls",task16_allocation_wall_s.size()},
            {"allocator_wall_p50_s",task16_percentile(0.50)},
            {"allocator_wall_p95_s",task16_percentile(0.95)},
            {"allocator_wall_max_s",task16_percentile(1.00)}};
        if (task19_switcher) {
            const auto scheduler_percentile=[&](double q)->json {
                if (task19_scheduler_wall_s.empty()) return nullptr;
                auto values=task19_scheduler_wall_s;
                std::sort(values.begin(),values.end());
                const std::size_t index=static_cast<std::size_t>(std::floor(
                    q*static_cast<double>(values.size()-1)));
                return values[index];
            };
            const auto& audit=task19_switcher->audit();
            record["task19_switcher"]={{"complete",
                    task19_switcher->complete()},
                {"failed",task19_switcher->failed()},
                {"signal_evaluations",audit.signal_evaluations},
                {"switch_requests",audit.switch_requests},
                {"preflight_rejections",audit.preflight_rejections},
                {"transition_start_failures",
                    audit.transition_start_failures},
                {"certified_edge_replacements",
                    audit.certified_edge_replacements},
                {"trigger_tick",audit.trigger_tick.has_value()
                    ?json(*audit.trigger_tick):json(nullptr)},
                {"trigger_fraction",audit.trigger_fraction},
                {"failure_reason",audit.failure_reason},
                {"scheduler_wall_p50_s",scheduler_percentile(0.50)},
                {"scheduler_wall_p95_s",scheduler_percentile(0.95)},
                {"scheduler_wall_max_s",scheduler_percentile(1.00)}};
        }
        if (policy_task17) {
            record["task17_trajectory"]=record["task16_trajectory"];
            record["task17_gridworld"]={{"delta_path",
                grid_delta_path.string()},
                {"initial_certified_count",initial_grid.certified_count},
                {"initial_truth_count",initial_grid.truth_count},
                {"final_certified_count",final_grid.certified_count},
                {"final_truth_count",final_grid.truth_count},
                {"final_certified_hash",final_grid.certified_hash},
                {"final_truth_hash",final_grid.truth_hash}};
        }
        if (production_semantics) {
            record["task19_gridworld"]={{"delta_path",
                grid_delta_path.string()},
                {"initial_certified_count",initial_grid.certified_count},
                {"initial_truth_count",initial_grid.truth_count},
                {"final_certified_count",final_grid.certified_count},
                {"final_truth_count",final_grid.truth_count},
                {"final_certified_hash",final_grid.certified_hash},
                {"final_truth_hash",final_grid.truth_hash}};
        }
        record["safety_and_information"]={
            {"maximum_actual_reference_m",maximum_actual_reference_m},
            {"maximum_target_reference_m",maximum_target_reference_m},
            {"minimum_actual_separation_m",minimum_actual_separation_m},
            {"minimum_target_separation_m",minimum_target_separation_m},
            {"minimum_robust_fim",gf::task10p11w_detail::number(
                minimum_robust_fim)},
            {"maximum_posterior_m2",maximum_posterior_m2},
            {"minimum_aoi_margin_s",gf::task10p11w_detail::number(
                minimum_aoi_margin_s)},
            {"minimum_gamma",gf::task10p11w_detail::number(minimum_gamma)},
            {"minimum_qp_residual",gf::task10p11w_detail::number(
                minimum_qp_residual)},
            {"minimum_robust_reference_margin_m",
                gf::task10p11w_detail::number(
                    minimum_robust_reference_margin_m)},
            {"maximum_robust_reference_m",
                gf::task10p11w_detail::number(
                    850.0-minimum_robust_reference_margin_m)},
            {"minimum_robust_separation_margin_m",
                gf::task10p11w_detail::number(
                    minimum_robust_separation_margin_m)},
            {"minimum_robust_separation_m",
                gf::task10p11w_detail::number(
                    10.0+minimum_robust_separation_margin_m)},
            {"minimum_robust_workspace_margin_m",
                gf::task10p11w_detail::number(
                    minimum_robust_workspace_margin_m)},
            {"maximum_speed_mps",maximum_speed_mps},
            {"maximum_axis_control_mps2",maximum_axis_control_mps2},
            {"maximum_acceleration_norm_mps2",
                maximum_acceleration_norm_mps2},
            {"intervention_owner_ticks",intervention_owner_ticks},
            {"total_owner_ticks",total_owner_ticks},
            {"intervention_fraction",total_owner_ticks==0?0.0:
                static_cast<double>(intervention_owner_ticks)/
                    total_owner_ticks},
            {"target_switches",target_switches},
            {"target_governor_ticks",target_governor_ticks},
            {"minimum_target_governor_fraction",
                minimum_target_governor_fraction},
            {"minimum_target_governor_stopping_margin_m",
                gf::task10p11w_detail::number(
                    minimum_target_governor_stopping_margin_m)},
            {"mean_active_squads",tick==0?0.0:
                static_cast<double>(active_squad_ticks)/(tick+1)},
            {"runtime_safety_violation",runtime_safety_violation},
            {"runtime_safety_reason",runtime_safety_reason}};
        record["profiler"]={{"note",
            "cumulative over run; interval telemetry audits the pre-advance "
            "truth velocity (two-tick defect fixed in this code round)"},
            {"profiled_total_s",gf::task10p11w_detail::number(
                profiled_total)},
            {"phases",std::move(profiler_json)}};
        record["complete"]=true;
        record["run_outcome"]=gf::task19RunOutcomeName(
            gf::classifyTask19Run(t100_tick,hard_stop,
                runtime_safety_violation));
        record["right_censored"]=!t100_tick.has_value()&&
            !hard_stop&&!runtime_safety_violation;
        record["wall_time_s"]=elapsed();
        gf::writeTask10p11vJson(argv[2],record);
        std::cout<<record.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 13 Phase A run failed: "<<error.what()<<'\n';
        return 4;
    }
}
