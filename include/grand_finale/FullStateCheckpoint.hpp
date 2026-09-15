#pragma once
#include "grand_finale/CheckpointCodec.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

namespace gf::checkpoint {
template<>struct Empty<DirectedEdge>{static DirectedEdge make(){return {1,2};}};
template<>struct Empty<UndirectedEdge>{static UndirectedEdge make(){return UndirectedEdge::canonical(1,2);}};
template<>struct Empty<RangeMeasurement>{static RangeMeasurement make(){return {0,Empty<UndirectedEdge>::make(),0,1};}};
template<>struct Empty<AcceptedRangeUpdateAudit>{static AcceptedRangeUpdateAudit make(){return {Empty<RangeMeasurement>::make(),0,0,0};}};
template<>struct Empty<RangeGenerationAudit>{static RangeGenerationAudit make(){return {0,Empty<UndirectedEdge>::make(),0,1,0,false};}};
template<>struct Empty<TransitionProposal>{static TransitionProposal make(){return {{},{1,2},{1,2},0,0};}};
#define GF_CP_F(name) f(#name,v.name)
#define GF_CP_RECORD(type,body) template<>struct Codec<type>:RecordCodec<type>{static void fields(Fields& f,type& v){body}};
GF_CP_RECORD(DirectedEdge,GF_CP_F(reference);GF_CP_F(owner);if(v.reference==v.owner)throw std::invalid_argument("checkpoint self edge");)
GF_CP_RECORD(UndirectedEdge,GF_CP_F(first);GF_CP_F(second);if(v.first>=v.second)throw std::invalid_argument("checkpoint noncanonical measurement edge");)
GF_CP_RECORD(FrontierCell,GF_CP_F(x_index);GF_CP_F(y_index);GF_CP_F(center);)
GF_CP_RECORD(RangeMeasurement,GF_CP_F(timestamp_ns);GF_CP_F(edge);GF_CP_F(range_m);GF_CP_F(variance_m2);)
GF_CP_RECORD(AcceptedRangeUpdateAudit,GF_CP_F(measurement);GF_CP_F(master);GF_CP_F(innovation);GF_CP_F(innovation_variance);)
GF_CP_RECORD(RangeGenerationAudit,GF_CP_F(batch);GF_CP_F(edge);GF_CP_F(true_distance_m);GF_CP_F(probability);GF_CP_F(uniform);GF_CP_F(acquired);)
GF_CP_RECORD(DekfRestartState,GF_CP_F(mobile_ids);GF_CP_F(means);GF_CP_F(propagation_factors);GF_CP_F(correlation_rows);GF_CP_F(fixed_positions);GF_CP_F(version);GF_CP_F(last_measurement);)
GF_CP_RECORD(CertifiedCoverageRestartState,GF_CP_F(truth);GF_CP_F(certified);)
GF_CP_RECORD(TransitionProposal,GF_CP_F(old_edges);GF_CP_F(new_edge);GF_CP_F(old_edge);GF_CP_F(expected_topology_version);GF_CP_F(expected_estimator_version);)
GF_CP_RECORD(CertifiedTopologyState,GF_CP_F(valid);GF_CP_F(reason);GF_CP_F(minimum_gamma);)
GF_CP_RECORD(TransitionCertificate,GF_CP_F(valid);GF_CP_F(forward_valid);GF_CP_F(reverse_valid);GF_CP_F(reason);GF_CP_F(topology_version);GF_CP_F(estimator_version);GF_CP_F(old_edges);GF_CP_F(new_edge);GF_CP_F(old_edge);GF_CP_F(union_edges);GF_CP_F(successor_edges);GF_CP_F(old_state);GF_CP_F(union_state);GF_CP_F(successor_state);GF_CP_F(minimum_gamma);)
GF_CP_RECORD(SupervisorThresholds,GF_CP_F(minimum_dwell_s);GF_CP_F(gamma_trigger);GF_CP_F(gamma_accept);)
GF_CP_RECORD(Task20CoverageUnit,GF_CP_F(id);GF_CP_F(members);GF_CP_F(base_anchors);GF_CP_F(leader);GF_CP_F(front_members);GF_CP_F(frame_origin);GF_CP_F(search_observation_members);)
GF_CP_RECORD(Task20MemberRole,GF_CP_F(member);GF_CP_F(coverage_unit);GF_CP_F(axial_fraction);GF_CP_F(triangular_fraction);)
GF_CP_RECORD(Task20DagLatticeContract,GF_CP_F(valid);GF_CP_F(reason);GF_CP_F(id);GF_CP_F(structural_signature);GF_CP_F(reference_edges);GF_CP_F(coverage_units);GF_CP_F(member_roles);GF_CP_F(topological_order);GF_CP_F(fixed_anchor_ids);)
GF_CP_RECORD(Task32UnitFrontLedger,GF_CP_F(task);GF_CP_F(applied_front);GF_CP_F(active);)
GF_CP_RECORD(BoundaryExcursionAudit,GF_CP_F(maximum_outside_distance_m);GF_CP_F(any_outside_duration_s);GF_CP_F(owner_outside_duration_s);GF_CP_F(maximum_simultaneous_outside);GF_CP_F(maximum_position_norm_m);GF_CP_F(maximum_position);GF_CP_F(outside_observer_new_truth_cells);)
GF_CP_RECORD(SimpleCoveragePolicyConfig,GF_CP_F(allocation_epoch_cycles);GF_CP_F(fairness_promotion_age);GF_CP_F(maximum_certification_failures);GF_CP_F(comparison_tolerance);GF_CP_F(target_projection);)
GF_CP_RECORD(NaturalSettlingConfig,GF_CP_F(position_tolerance_m);GF_CP_F(speed_tolerance_mps);GF_CP_F(yaw_tolerance_rad);GF_CP_F(dwell_cycles);)
GF_CP_RECORD(LeaderCoverageBranchSpec,GF_CP_F(members);GF_CP_F(leader);GF_CP_F(coverage_origin);GF_CP_F(rotation_rad);GF_CP_F(preferred_fixed_roots);GF_CP_F(ladder_segments);)
GF_CP_RECORD(DistanceRangeAvailability,GF_CP_F(enabled);GF_CP_F(link_seed);)
GF_CP_RECORD(BoundaryPolicyConfig,GF_CP_F(policy);GF_CP_F(flight_polygon_source);GF_CP_F(explicit_flight_polygon);GF_CP_F(soft_slack_weight);)
GF_CP_RECORD(ProgressCompatibilityConfig,GF_CP_F(max_projection_norm);GF_CP_F(min_direction_ratio);GF_CP_F(comparison_tolerance);GF_CP_F(zero_nominal_is_compatible);)
GF_CP_RECORD(GrandFinaleSwarmAdapterConfig,
    GF_CP_F(solver_profile);GF_CP_F(dt_s);GF_CP_F(minimum_dwell_s);GF_CP_F(acceleration_half_box);
    GF_CP_F(speed_limit_mps);GF_CP_F(speed_cbf_gain);GF_CP_F(plant_speed_facet_count);GF_CP_F(speed_row_nominal);
    GF_CP_F(speed_row_nominal_limit_mps);GF_CP_F(speed_preflight_fuse_mps);GF_CP_F(speed_preflight_demoted);
    GF_CP_F(tau_margin_gate_enabled);GF_CP_F(tau_family_predict);GF_CP_F(tau_analytic_first_order);
    GF_CP_F(nominal_throttle_enabled);GF_CP_F(throttle_gamma_th_mps2);GF_CP_F(throttle_gamma_floor_mps2);GF_CP_F(throttle_v2_enabled);GF_CP_F(throttle_v2_endpoint_family);
    GF_CP_F(speed_rows_removed);GF_CP_F(nominal_speed_saturation_mps);GF_CP_F(speed_initial_set_truth_gate);
    GF_CP_F(target_policy_v2);GF_CP_F(target_policy_v3);GF_CP_F(leader_reachability_filter);GF_CP_F(leader_tie_break_tolerance_m);GF_CP_F(target_policy_v6);
    GF_CP_F(target_policy_unified_h2);GF_CP_F(unified_h2_minimum_half_width_m);GF_CP_F(unified_h2_fan_ratio);GF_CP_F(unified_h2_shortlist_per_squad);GF_CP_F(unified_h2_service_standoff_m);GF_CP_F(unified_h2_cbf2026_wide_virtual_formation);
    GF_CP_F(target_policy_task15_forward);GF_CP_F(task15_forward_shortlist_capacity);GF_CP_F(task15_forward_update_period_cycles);GF_CP_F(task15_forward_endpoint_standoff_m);
    GF_CP_F(target_policy_task16_cbf2026);GF_CP_F(task16_coverage_arm);GF_CP_F(task16_cvt_update_period_cycles);GF_CP_F(task16_reference_damping_reserve_multiples);GF_CP_F(task16_tracking_envelope_enabled);
    GF_CP_F(target_policy_task17_periodic);GF_CP_F(task17_periodic_arm);GF_CP_F(task17_update_period_cycles);GF_CP_F(task17_common_governor_enabled);GF_CP_F(task17_reference_compatible_formation);GF_CP_F(task17_member_aware_wide_formation);GF_CP_F(task17_coherent_service_wide_formation);
    GF_CP_F(target_policy_task18_cbf2026_outer);GF_CP_F(task18_collision_only_vaug);GF_CP_F(task18_update_period_cycles);GF_CP_F(task18_common_governor_enabled);GF_CP_F(task18_yaw_objective);
    GF_CP_F(target_policy_task20_dag_lattice);GF_CP_F(task20_lattice_mode);GF_CP_F(task20_target_policy);GF_CP_F(task20_update_period_cycles);GF_CP_F(task32_target_mechanism);GF_CP_F(task32_front_rate_mps);GF_CP_F(task20_wavefront_band_width_m);
    GF_CP_F(task21_progress_axis_x);GF_CP_F(task21_progress_axis_y);GF_CP_F(task21_cross_axis_x);GF_CP_F(task21_cross_axis_y);GF_CP_F(task21_local_window_cells);
    GF_CP_F(v6_neighborhood_radius_m);GF_CP_F(demand_recompute_interval_s);GF_CP_F(target_lock_epsilon_m);GF_CP_F(target_lock_dwell_cycles);GF_CP_F(target_lock_progress_epsilon_m);GF_CP_F(reachability_hysteresis_m);GF_CP_F(projection_passes);GF_CP_F(speed_tracking_gain);GF_CP_F(speed_tracking_blend_m);
    GF_CP_F(velocity_augmented_rows);GF_CP_F(row_slack_epsilon_m);GF_CP_F(workspace_class_k);GF_CP_F(workspace_alpha1_gain);GF_CP_F(workspace_alpha2_gain);GF_CP_F(workspace_braking_acceleration_mps2);GF_CP_F(workspace_braking_regularization_m);
    GF_CP_F(target_homotopy_enabled);GF_CP_F(target_homotopy_braking_acceleration_mps2);GF_CP_F(target_homotopy_rate_gain);GF_CP_F(target_homotopy_workspace_guard_m);
    GF_CP_F(maximum_yaw_rate_radps);GF_CP_F(position_gain);GF_CP_F(velocity_gain);GF_CP_F(estimator_acceleration_variance);GF_CP_F(initial_position_variance_m2);GF_CP_F(initial_velocity_variance_m2);
    GF_CP_F(certified_error_bound_m);GF_CP_F(certified_shadow_single_position_support_m);GF_CP_F(certified_shadow_single_velocity_support_mps);GF_CP_F(certified_shadow_relative_position_support_m);GF_CP_F(certified_shadow_relative_velocity_support_mps);
    GF_CP_F(maximum_accepted_range_innovation_m);GF_CP_F(range_random_seed);GF_CP_F(range_noise_std_m);GF_CP_F(range_dropout_probability);GF_CP_F(distance_range_availability);
    GF_CP_F(sensor_radius_m);GF_CP_F(coverage_footprint_kind);GF_CP_F(coverage_inner_radius_m);GF_CP_F(coverage_half_angle_rad);
    GF_CP_F(reference_distance_m);GF_CP_F(add_reference_distance_m);GF_CP_F(reference_uncertainty_m);GF_CP_F(uncertainty_sigma);GF_CP_F(maximum_reference_position_eigenvalue_m2);GF_CP_F(maximum_posterior_eigenvalue_m2);GF_CP_F(maximum_range_aoi_s);GF_CP_F(minimum_range_quality);
    GF_CP_F(collision_distance_m);GF_CP_F(collision_lambda1);GF_CP_F(collision_lambda2);GF_CP_F(residual_tolerance);GF_CP_F(qp_oracle_tolerance);GF_CP_F(boundary);GF_CP_F(gamma_feedback_selection);GF_CP_F(predictive_gamma_tau_mps2);GF_CP_F(gamma_feedback_homotopy_segments);GF_CP_F(gamma_feedback_tolerance);GF_CP_F(progress_compatibility);
)
#undef GF_CP_RECORD
#undef GF_CP_F
} // namespace gf::checkpoint

namespace gf {
// Public runner seam. Capture is observational; restore never calls allocator,
// measurement generation or transition acceptance. Files are an outer layer.
class FullStateCheckpoint {
    using Json=checkpoint::Json;
    using Fields=checkpoint::Fields;
    static void guard(const GrandFinaleSwarmAdapter& a,const Task10p11hSimpleCoverageController& c) {
        const auto& p=a.config_;
        if(!a.stage_zero_initialized_||c.dynamic_pair_override_||c.development_control_override_||
           a.nominal_override_||a.yaw_rate_override_)
            throw std::logic_error("checkpoint requires completed control boundary");
        if((!p.target_policy_task20_dag_lattice&&!p.target_policy_task18_cbf2026_outer)||
           p.target_policy_v2||p.target_policy_v3||p.target_policy_v6||p.target_policy_unified_h2||
           p.target_policy_task15_forward||p.target_policy_task16_cbf2026||p.target_policy_task17_periodic||
           p.target_homotopy_enabled||p.task18_common_governor_enabled||p.task20_target_policy!=0||
           (p.task32_target_mechanism!=0&&p.task32_target_mechanism!=2)||
           p.boundary.policy==BoundaryPolicy::SoftSearchRetention||p.solver_profile!=SolverProfile::Gurobi)
            throw std::invalid_argument("unsupported checkpoint policy/solver; no partial restore");
    }
    static Json immutable(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c) {
        return Json{{"adapter_config",checkpoint::encode(a.config_)},{"swarm_config",s.config},
            {"mobile_ids",checkpoint::encode(a.mobile_ids_)},{"anchors",checkpoint::encode(a.fixed_positions_)},
            {"policy",checkpoint::encode(c.config_)},{"settling",checkpoint::encode(c.settling_)},
            {"branches",checkpoint::encode(c.branches_)},{"registered_modes",checkpoint::encode(c.registered_external_contracts_)}};
    }
    static void supervisor(Fields& f,HybridSupervisor& v) {
#define F(x) f(#x,v.x)
        F(thresholds_);F(mode_);F(last_transition_s_);F(topology_);F(topology_version_);F(pending_);
#undef F
    }
    static void adapter(Fields& f,GrandFinaleSwarmAdapter& v) {
#define F(x) f(#x,v.x)
        auto ekf=v.estimator_.restartState();f("estimator",ekf);
        auto masks=v.coverage_.restartState();f("coverage",masks);
        if(f.loading){v.estimator_.restoreRestartState(ekf);v.coverage_.restoreRestartState(masks);}
        Json sup=f.loading?f.object.at("supervisor"):Json::object();Fields sf{sup,f.loading,{}};supervisor(sf,v.supervisor_);sf.finish();
        f.names.insert("supervisor");if(!f.loading)f.object["supervisor"]=std::move(sup);
        F(limiting_family_memory_);F(throttle_telemetry_active);F(throttle_telemetry_s);F(throttle_telemetry_min_gamma);F(throttle_telemetry_owner);
        F(throttle_telemetry_v2_active);F(throttle_telemetry_v2_endpoint_s);F(throttle_telemetry_v2_endpoint_signal);
        F(pending_proposal_);F(pending_certificate_);F(union_control_cycles_);F(stage_zero_initialized_);F(transition_stack_);F(pending_is_retreat_);F(last_certification_reason_);
        F(range_last_observation_s_);F(range_quality_);F(range_variance_m2_);F(last_accepted_range_batch_audit_);F(last_range_generation_audit_);F(range_batch_count_);
#undef F
    }
    static void controller(Fields& f,Task10p11hSimpleCoverageController& v) {
#define F(x) f(#x,v.x)
        F(targets_);F(external_reconstruction_reference_);F(external_coverage_mode_);F(external_coverage_contract_);F(external_force_reallocate_);F(governed_targets_);
        F(target_epoch_);F(consecutive_failures_);F(successful_control_cycles_);F(control_boundaries_);F(phase_);F(t100_coverage_s_);F(settling_dwell_cycles_);F(last_nominal_controls_);
        F(task16_applied_targets_);F(task16_last_applied_controls_);F(task16_last_applied_yaw_rates_);
        F(task18_last_allocation_cycle_);F(task20_last_allocation_cycle_);F(task32_unit_front_ledger_);F(boundary_excursion_);
#ifdef GF_CHECKPOINT_R2_OVERLAY
        F(external_coverage_restored_);
#endif
#undef F
    }
    static void grid(Fields& f,GridWorld& g) {
        const Json geometry={{"x_limits",checkpoint::encode(g.xLim)},{"y_limits",checkpoint::encode(g.yLim)},
            {"x_cells",g.xNum},{"y_cells",g.yNum},{"valid_count",g.validCount},{"valid",checkpoint::encode(g.valid)}};
        if(f.loading&&f.object.at("geometry")!=geometry)throw std::invalid_argument("checkpoint grid domain mismatch");
        f.names.insert("geometry");if(!f.loading)f.object["geometry"]=geometry;
        f("visited",g.vis);if(g.vis.size()!=g.valid.size())throw std::invalid_argument("checkpoint grid mask size");
    }
    static void plant(Fields& f,Swarm& s) {
        Json bots=f.loading?f.object.at("robots"):Json::array();
        if(f.loading&&bots.size()!=s.robots.size())throw std::invalid_argument("checkpoint owner count");
        for(std::size_t i=0;i<s.robots.size();++i){auto& r=*s.robots[i];Json j=f.loading?bots.at(i):Json::object();Fields rf{j,f.loading,{}};
            if(f.loading&&j.at("id")!=r.id)throw std::invalid_argument("checkpoint owner identity");
            rf.names.insert("id");if(!f.loading)j["id"]=r.id;
            Eigen::VectorXd x=r.model->getX(),u=r.model->getControlInput();rf("state",x);rf("control",u);rf("runtime",r.runtime);rf("position_covariance",r.positionCovariance);
            rf("updated_grid",r.updatedGridWorld);rf("my_formation",r.myFormation);rf("covariance_formation",r.myCovarianceFormation);
            Json gj=f.loading?j.at("grid"):Json::object();Fields gf{gj,f.loading,{}};grid(gf,r.gridWorld);gf.finish();rf.names.insert("grid");if(!f.loading)j["grid"]=std::move(gj);
            rf.finish();if(f.loading){if(!x.allFinite()||!u.allFinite()||!std::isfinite(r.runtime))throw std::invalid_argument("checkpoint nonfinite plant");r.model->setStateVector(x);r.model->setControlInput(u);}else bots.push_back(std::move(j));
        }
        f.names.insert("robots");if(!f.loading)f.object["robots"]=std::move(bots);
        Json gj=f.loading?f.object.at("truth_grid"):Json::object();Fields gf{gj,f.loading,{}};grid(gf,s.gridWorldGroundTruth);gf.finish();f.names.insert("truth_grid");if(!f.loading)f.object["truth_grid"]=std::move(gj);
        f("updated_truth",s.updatedGridWorldGroundTruth);
    }
    template<class T,class Fn>static Json component(T& v,Fn fn){Json j=Json::object();Fields f{j,false,{}};fn(f,v);f.finish();return j;}
    template<class T,class Fn>static void loadComponent(T& v,const Json& j,Fn fn){Json copy=j;Fields f{copy,true,{}};fn(f,v);f.finish();}
    static void load(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c,const Json& j){
        loadComponent(s,j.at("plant"),plant);loadComponent(a,j.at("adapter"),adapter);loadComponent(c,j.at("controller"),controller);
    }
public:
    static Json capture(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c,const Json& identity) {
        guard(a,c);return Json{{"schema","grand-finale-full-state-v1"},{"identity",identity},{"immutable",immutable(s,a,c)},
            {"plant",component(s,plant)},{"adapter",component(a,adapter)},{"controller",component(c,controller)}};
    }
    static void restore(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c,const Json& saved,const Json& identity) {
        guard(a,c);
        if(saved.size()!=6||saved.at("schema")!="grand-finale-full-state-v1"||saved.at("identity")!=identity||saved.at("immutable")!=immutable(s,a,c))
            throw std::invalid_argument("checkpoint schema/binary/configuration identity mismatch");
        const auto before=capture(s,a,c,identity);
        try{load(s,a,c,saved);if(capture(s,a,c,identity)!=saved)throw std::runtime_error("checkpoint restored state mismatch");}
        catch(...){load(s,a,c,before);throw;}
    }
};
} // namespace gf
