// Included only by the isolated engineering runner, after its loop locals.
gf::RunnerCheckpoint checkpoints(checkpoint_original_argv);
if(checkpoints.enabled()&&task19_switcher)throw std::invalid_argument("dynamic scheduler checkpoints are outside this engineering contract");
const auto checkpoint_metrics=[&](json data,bool loading){
    gf::checkpoint::Fields f{data,loading,{}};
#define F(x) f(#x,x)
    F(max_truth_overspeed);F(max_interval_overspeed);F(truth_overspeed_ticks);F(interval_overspeed_ticks);
    F(t95_tick);F(t100_tick);F(certified_checkpoint_500_count);F(truth_checkpoint_500_count);F(tick);
    F(hard_stop);F(truth_gate_violation);F(runtime_safety_violation);F(runtime_safety_reason);F(truth_gate_max_speed);
    F(maximum_actual_reference_m);F(maximum_target_reference_m);F(minimum_actual_separation_m);F(minimum_target_separation_m);
    F(minimum_robust_fim);F(maximum_posterior_m2);F(minimum_aoi_margin_s);F(minimum_gamma);F(minimum_qp_residual);
    F(maximum_speed_mps);F(maximum_axis_control_mps2);F(maximum_acceleration_norm_mps2);
    F(minimum_robust_reference_margin_m);F(minimum_robust_separation_margin_m);F(minimum_robust_workspace_margin_m);
    F(intervention_owner_ticks);F(total_owner_ticks);F(target_switches);F(last_target_epoch);F(active_squad_ticks);F(current_active_squads);
    F(minimum_target_governor_fraction);F(minimum_target_governor_stopping_margin_m);F(target_governor_ticks);F(highest_selected_task_row);
    F(nominal_target_min);F(nominal_target_max);F(applied_target_min);F(applied_target_max);F(actual_position_min);F(actual_position_max);
    F(total_distance_m);F(squared_control_energy_proxy);F(maximum_outside_distance_m);
    F(task16_allocation_wall_s);F(task19_scheduler_wall_s);F(last_step.step.advanced);F(last_step.step.reason);
    auto evaluations=record.at("evaluations");F(evaluations);if(loading)record["evaluations"]=std::move(evaluations);
#undef F
    json e=loading?data.at("efficiency"):json::object();gf::checkpoint::Fields ef{e,loading,{}};gf::RunnerCheckpoint::efficiency(ef,efficiency);ef.finish();f.names.insert("efficiency");if(!loading)data["efficiency"]=std::move(e);
    json p=loading?data.at("profiler"):json::object();gf::checkpoint::Fields pf{p,loading,{}};gf::RunnerCheckpoint::profile(pf,profiler);pf.finish();f.names.insert("profiler");if(!loading)data["profiler"]=std::move(p);
    f.finish();return data;
};
if(checkpoints.restoring()) {
    const auto& saved=checkpoints.loaded();
    if(saved.at("metrics").at("hard_stop").get<bool>()||saved.at("metrics").at("runtime_safety_violation").get<bool>())
        throw std::invalid_argument("safety-stopped checkpoint is read-only, not resumable");
    gf::RunnerCheckpoint::restoreEngine(fixture->swarm,fixture->adapter,fixture->controller,external_reconstructor.get(),saved.at("engine"),checkpoints.identity());
    checkpoint_metrics(saved.at("metrics"),true);
    if(tick!=saved.at("tick").get<std::size_t>()||
       gf::RunnerCheckpoint::controlTick(fixture->adapter.runtimeSnapshot().runtime_s,config.dt_s)!=tick||
       checkpoints.seekTick()>gf::RunnerCheckpoint::controlTick(window_s,config.dt_s))
        throw std::invalid_argument("runner checkpoint absolute time mismatch");
    previous_certified=fixture->adapter.coverage().certifiedGrid();previous_truth=fixture->adapter.coverage().truthGrid();
    record["checkpoint_lineage"]=checkpoints.ancestry();
}
std::optional<json> checkpoint_boundary_key;
const auto save_checkpoint=[&](bool force=false){
    if(!checkpoints.enabled())return;
    if(external_reconstructor) {
        const auto key=gf::ReconstructionCheckpoint::boundaryKey(*external_reconstructor);
        force=force||!checkpoint_boundary_key||key!=*checkpoint_boundary_key||
            gf::ReconstructionCheckpoint::requestDue(*external_reconstructor);
        checkpoint_boundary_key=key;
    }
    if(checkpoints.shouldSave(tick,force))checkpoints.save(tick,fixture->adapter.runtimeSnapshot().runtime_s,
        gf::RunnerCheckpoint::captureEngine(fixture->swarm,fixture->adapter,fixture->controller,external_reconstructor.get(),checkpoints.identity()),
        checkpoint_metrics(json::object(),false),hard_stop||t100_tick.has_value());
};
