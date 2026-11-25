#pragma once

#include "grand_finale/RouteBMissionRunner.hpp"
#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"
#include "grand_finale/RouteBWave9Validations.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

inline bool checkThreeBeaconCorridorCondition(
    const std::vector<double>& beacon_xs,
    double r_cor_m) {
    const std::size_t M = beacon_xs.size();
    if (M < 3 || !std::isfinite(r_cor_m) || r_cor_m <= 0.0) {
        return false;
    }
    std::vector<double> s(M - 1, 0.0);
    double s_max = 0.0;
    for (std::size_t r = 0; r + 1 < M; ++r) {
        s[r] = beacon_xs[r + 1] - beacon_xs[r];
        if (!std::isfinite(s[r]) || s[r] <= 0.0) {
            return false;
        }
        s_max = std::max(s_max, s[r]);
    }
    if (s_max > r_cor_m + 1.0e-9) {
        return false;
    }
    if (0.5 * s.front() + s[1] > r_cor_m + 1.0e-9) {
        return false;
    }
    if (0.5 * s.back() + s[M - 3] > r_cor_m + 1.0e-9) {
        return false;
    }
    if (M >= 4) {
        for (std::size_t m = 0; m + 3 < M; ++m) {
            if (0.5 * (s[m] + s[m + 1] + s[m + 2]) > r_cor_m + 1.0e-9) {
                return false;
            }
        }
    }
    return true;
}

struct ReversibleRoundTripResult {
    bool reached_t95 = false;
    bool reached_t100 = false;
    std::optional<double> t95_certified_s;
    std::optional<double> t100_certified_s;
    double forward_reconfig_s = 0.0;
    double return_entry_s = 0.0;
    double return_reconfig_s = 0.0;
    double total_time_s = 0.0;
    bool return_completed = false;
    bool final_topology_matched = false;
    double final_certified_coverage = 0.0;
    int false_certified_count = 0;
    std::size_t collision_violations = 0;
    std::size_t reference_violations = 0;
    std::size_t axis_speed_violations = 0;
    std::size_t static_parent_violations = 0;
    std::size_t dag_violations = 0;
    std::size_t total_mbb_swaps = 0;
    double position_rmse_m = 0.0;
    double overall_envelope_coverage = 0.0;
    double worst_agent_envelope_coverage = 0.0;
};

struct ReversibleSearchComparisonBundle {
    std::string domain_name = "reversible_coastal_pentagon";
    std::string channel_group = "degraded";
    double area_km2 = 0.0;
    std::vector<Eigen::Vector2d> vertices;
    std::vector<double> beacon_xs;
    RouteBMissionArmResult fixed_dual;
    RouteBMissionArmResult fixed_single;
    ReversibleRoundTripResult round_trip;
};

inline RouteBConvexDomainSpec buildReversiblePentagonDomainSpec() {
    RouteBConvexDomainSpec dom;
    dom.name = "reversible_coastal_pentagon";
    dom.description =
        "Convex pentagon requiring Dual flanks and Single deep apex: neither static formation reaches 100%, while Dual -> Single -> Dual certifies 100% and returns";
    dom.vertices = {
        {-100.0, 220.0},
        {1180.0, 220.0},
        {1140.0, 720.0},
        {650.0, 1280.0},
        {120.0, 720.0},
    };
    const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0};
    dom.initial_tri = buildStaggeredTriangulation(
        b_xs, {{1, 2, 3}, {}, {4, 5, 6}},
        380.0, 90.0, 100.0, 140.0);
    dom.target_tri = buildStaggeredTriangulation(
        b_xs, {{}, {1, 2, 3, 4, 5, 6}, {}},
        380.0, 90.0, 100.0, 140.0);
    dom.use_direct_matched = true;
    return dom;
}

inline ReversibleSearchComparisonBundle runReversibleConvexSearchComparison(
    const std::string& channel_group = "degraded") {
    const auto dom = buildReversiblePentagonDomainSpec();
    const double d_r = (channel_group == "nominal") ? 850.0 : 600.0;
    const double sig0 = (channel_group == "nominal") ? 0.5 : 1.0;

    // Static baselines on identical polygon
    RouteBMissionConfig cfg_dual;
    cfg_dual.domain_name = dom.name;
    cfg_dual.custom_domain = dom;
    cfg_dual.patrol_radius_m = 105.0;
    cfg_dual.v_cruise_mps = 15.0;
    cfg_dual.accel_mps2 = 2.0;
    cfg_dual.d_pipe_m = 60.0;
    cfg_dual.t_fork_s = 15.0;
    cfg_dual.max_mission_s = 350.0;
    cfg_dual.uwb_d_r_m = d_r;
    cfg_dual.uwb_sigma0_m = sig0;
    cfg_dual.uwb_d_max_m = 1200.0;
    cfg_dual.link_seed = 20261123;
    cfg_dual.skip_switch_arm = true;
    const auto res_dual = runTwoRegimeMissionComparison(cfg_dual);

    auto dom_single_init = dom;
    std::swap(dom_single_init.initial_tri, dom_single_init.target_tri);
    RouteBMissionConfig cfg_single = cfg_dual;
    cfg_single.custom_domain = dom_single_init;
    const auto res_single = runTwoRegimeMissionComparison(cfg_single);

    ReversibleSearchComparisonBundle bundle;
    bundle.channel_group = channel_group;
    bundle.area_km2 = res_dual.area_m2 / 1.0e6;
    bundle.vertices = dom.vertices;
    bundle.beacon_xs = {0.0, 400.0, 800.0, 1200.0};
    bundle.fixed_dual = res_dual.stay;
    bundle.fixed_single = res_single.stay;

    // Continuous round-trip Dual -> Single -> (T100 Search) -> Dual on one adapter
    RouteBPipelinedConfig pcfg;
    pcfg.dt_s = 0.1;
    pcfg.v_cruise_mps = 15.0;
    pcfg.accel_mps2 = 2.0;
    pcfg.a_max_mps2 = 4.0;
    pcfg.d_safe_m = 10.0;
    pcfg.d_dep_m = 850.0;
    pcfg.d_pipe_m = 60.0;
    pcfg.d_clear_m = 32.0;
    pcfg.t_settle_s = 1.0;
    pcfg.uwb_d_r_m = d_r;
    pcfg.uwb_sigma0_m = sig0;
    pcfg.uwb_d_max_m = 1200.0;
    pcfg.link_seed = 20261123;
    pcfg.maximum_range_aoi_s = 3.5;

    Task10p10Scenario scen;
    scen.id = "route_b_wave10_" + dom.name;
    scen.width_m = 1600.0;
    scen.height_m = 1600.0;
    scen.mobile_ids = dom.initial_tri.mobile_ids;
    for (NodeId id : scen.mobile_ids) {
        scen.mobile_positions.push_back(dom.initial_tri.positions.at(id));
    }
    scen.fixed_positions = dom.initial_tri.beacons;
    scen.initial_topology = dom.initial_tri.initialTopologyEdges();

    json settings = task10p10SwarmSettings(scen, SolverProfile::OpenSource);
    std::vector<int> base_indices;
    for (std::size_t i = 0; i < dom.initial_tri.beacon_ids.size(); ++i) {
        base_indices.push_back(static_cast<int>(i));
    }
    settings["formation"]["bases-id"] = {base_indices};
    settings["execute"]["time-total"] = 2250.0;
    settings["execute"]["time-step"] = pcfg.dt_s;
    settings["searching"]["front-sector"]["outer-radius"] = 400.0;
    json site = {
        {"schema", "gf-search-polygon-v1"},
        {"id", dom.name},
        {"vertices_m", json::array()},
        {"cell_size_m", 10.0},
    };
    for (const auto& v : dom.vertices) {
        site["vertices_m"].push_back({v.x(), v.y()});
    }
    settings = searchPolygonSettings(settings, site);

    GrandFinaleSwarmAdapterConfig acfg;
    acfg.solver_profile = SolverProfile::OpenSource;
    acfg.dt_s = pcfg.dt_s;
    acfg.minimum_dwell_s = pcfg.dt_s;
    acfg.acceleration_half_box = pcfg.a_max_mps2;
    acfg.collision_distance_m = pcfg.d_safe_m;
    acfg.reference_distance_m = pcfg.d_dep_m;
    acfg.add_reference_distance_m = pcfg.d_dep_m - 1.0;
    acfg.reference_uncertainty_m = 0.0;
    acfg.uncertainty_sigma = 3.0;
    acfg.uncertainty_scale_mode = UncertaintyScaleMode::TraceRoot;
    acfg.axis_velocity_limit_mps = 25.0;
    acfg.axis_velocity_cbf_gain = 5.0;
    acfg.estimator_acceleration_variance = 0.25 * 0.25;
    acfg.initial_position_variance_m2 = 0.04;
    acfg.initial_velocity_variance_m2 = 0.01;
    acfg.maximum_range_aoi_s = pcfg.maximum_range_aoi_s;
    acfg.normalized_range_innovation_gate_sigma = 4.0;
    acfg.minimal_runtime_gating = true;
    acfg.tau_margin_gate_enabled = true;
    acfg.tau_analytic_first_order = true;
    acfg.cbf2026_uwb_channel.enabled = true;
    acfg.cbf2026_uwb_channel.d_r_m = pcfg.uwb_d_r_m;
    acfg.cbf2026_uwb_channel.sigma0_m = pcfg.uwb_sigma0_m;
    acfg.cbf2026_uwb_channel.d_max_m = pcfg.uwb_d_max_m;
    acfg.cbf2026_uwb_channel.link_seed = pcfg.link_seed;
    acfg.sensor_radius_m = 400.0;
    acfg.coverage_footprint_kind = CoverageFootprintKind::ForwardSector;
    acfg.coverage_half_angle_rad = M_PI / 3.0;
    acfg.maximum_yaw_rate_radps = 1.0;

    Swarm swarm(settings);
    GrandFinaleSwarmAdapter adapter(
        swarm, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
    if (!adapter.initializeStageZero().initialized) {
        throw std::runtime_error("reversible round-trip stage_zero_init_failed");
    }
    RouteBClosedLoopAuditor auditor(
        scen.mobile_ids, scen.fixed_positions, pcfg.d_safe_m, pcfg.d_dep_m, 25.0,
        3.0, UncertaintyScaleMode::TraceRoot, pcfg.dt_s);

    auto vel_yaws = [&](const GrandFinaleRuntimeSnapshot& snap) {
        std::map<NodeId, double> cur, out_yaw;
        for (const auto& robot : swarm.robots) {
            cur[robot->id] = robot->model->getStateVariable("yawRad");
        }
        for (std::size_t i = 0; i < snap.estimate.mobile_ids.size(); ++i) {
            const NodeId id = snap.estimate.mobile_ids[i];
            const Eigen::Vector2d v_est = snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i) + 2);
            out_yaw[id] = computeVelocityAlignedYawRate(v_est, cur.at(id), 1.0);
        }
        return out_yaw;
    };

    auto [fwd_moves, mid_single] = planDirectMatchedTransfer(dom.initial_tri, dom.target_tri);
    auto [rev_moves, back_dual] = planDirectMatchedTransfer(mid_single, dom.initial_tri);

    enum class RtPhase { InitSweep, FwdEntry, FwdReconfig, SingleSweep, RevEntry, RevReconfig, Done };
    RtPhase phase = RtPhase::InitSweep;

    route_b_mission_detail::FrontierSweepPolicy init_sweep(
        dom.initial_tri, 105.0, 0.8, 1.8, 4.0, 1.0, 400.0, false, 0.0, pcfg.d_dep_m);
    std::optional<RouteBPolarEntryExecutor> fwd_entry, rev_entry;
    std::optional<RouteBPipelinedSession> fwd_pipe, rev_pipe;
    std::optional<route_b_mission_detail::FrontierSweepPolicy> mid_sweep;
    double entry_elapsed = 0.0;

    auto start_entry = [&](const RouteBTriangulation& dest_slots, std::optional<RouteBPolarEntryExecutor>& exec_opt) {
        const auto snap = adapter.runtimeSnapshot();
        std::map<NodeId, Eigen::Vector2d> p0, v0;
        for (std::size_t i = 0; i < scen.mobile_ids.size(); ++i) {
            const NodeId id = snap.estimate.mobile_ids[i];
            p0[id] = snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i));
            v0[id] = snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i) + 2);
        }
        Eigen::Vector2d pivot = Eigen::Vector2d::Zero();
        for (const auto& [bid, bp] : dest_slots.beacons) {
            (void)bid;
            pivot += bp;
        }
        pivot /= static_cast<double>(dest_slots.beacons.size());
        RouteBPolarEntryConfig ecfg;
        ecfg.accel_budget_mps2 = 2.0;
        ecfg.axis_speed_budget_mps = pcfg.v_cruise_mps;
        ecfg.brake_accel_mps2 = 2.0;
        ecfg.arrival_pos_tol_m = 3.0;
        ecfg.arrival_speed_tol_mps = 0.5;
        exec_opt.emplace(p0, v0, dest_slots.positions, pivot, ecfg);
        entry_elapsed = 0.0;
    };

    auto& rt = bundle.round_trip;
    const std::size_t max_ticks = 22000;
    for (std::size_t tick = 0; tick < max_ticks && phase != RtPhase::Done; ++tick) {
        const double t_s = static_cast<double>(tick + 1) * pcfg.dt_s;
        if (phase == RtPhase::InitSweep) {
            std::map<NodeId, Eigen::Vector2d> nom;
            std::map<NodeId, double> yaw;
            init_sweep.computeCommands(swarm, adapter, adapter.runtimeSnapshot().estimate, tick, nom, yaw);
            const auto st = adapter.stepWithNominalAndYawRates(nom, yaw);
            if (!st.advanced) throw std::runtime_error("InitSweep aborted: " + st.reason);
            auditor.observe(swarm, adapter.runtimeSnapshot());
            if (t_s >= 15.0 - 1.0e-9) {
                start_entry(dom.initial_tri, fwd_entry);
                phase = RtPhase::FwdEntry;
            }
        } else if (phase == RtPhase::FwdEntry) {
            const auto snap = adapter.runtimeSnapshot();
            const auto nom = fwd_entry->nominalControls(snap.estimate, entry_elapsed);
            const auto yaw = vel_yaws(snap);
            const auto st = adapter.stepWithNominalAndYawRates(nom, yaw);
            if (!st.advanced) throw std::runtime_error("FwdEntry aborted: " + st.reason);
            auditor.observe(swarm, adapter.runtimeSnapshot());
            entry_elapsed += pcfg.dt_s;
            if (fwd_entry->arrived(adapter.runtimeSnapshot().estimate, entry_elapsed)) {
                fwd_pipe.emplace(swarm, adapter, dom.initial_tri, mid_single, fwd_moves, pcfg, &auditor);
                phase = RtPhase::FwdReconfig;
            }
        } else if (phase == RtPhase::FwdReconfig) {
            auto yaw = vel_yaws(adapter.runtimeSnapshot());
            const bool done = fwd_pipe->stepOnce(&yaw);
            if (fwd_pipe->finished() && !fwd_pipe->completed()) {
                throw std::runtime_error("FwdReconfig failed: " + fwd_pipe->reason());
            }
            if (done) {
                rt.forward_reconfig_s = t_s;
                rt.static_parent_violations += fwd_pipe->staticParentViolations();
                rt.dag_violations += fwd_pipe->dagViolations();
                rt.total_mbb_swaps += fwd_pipe->totalMbbSwaps();
                mid_sweep.emplace(
                    mid_single, 105.0, 0.8, 1.8, 4.0, 1.0, 400.0, false, 0.0, pcfg.d_dep_m);
                phase = RtPhase::SingleSweep;
            }
        } else if (phase == RtPhase::SingleSweep) {
            std::map<NodeId, Eigen::Vector2d> nom;
            std::map<NodeId, double> yaw;
            mid_sweep->computeCommands(swarm, adapter, adapter.runtimeSnapshot().estimate, tick, nom, yaw);
            const auto st = adapter.stepWithNominalAndYawRates(nom, yaw);
            if (!st.advanced) throw std::runtime_error("SingleSweep aborted: " + st.reason);
            auditor.observe(swarm, adapter.runtimeSnapshot());
            if (adapter.coverage().reachedCertifiedT100()) {
                rt.return_entry_s = t_s;
                start_entry(mid_single, rev_entry);
                phase = RtPhase::RevEntry;
            }
        } else if (phase == RtPhase::RevEntry) {
            const auto snap = adapter.runtimeSnapshot();
            const auto nom = rev_entry->nominalControls(snap.estimate, entry_elapsed);
            const auto yaw = vel_yaws(snap);
            const auto st = adapter.stepWithNominalAndYawRates(nom, yaw);
            if (!st.advanced) throw std::runtime_error("RevEntry aborted: " + st.reason);
            auditor.observe(swarm, adapter.runtimeSnapshot());
            entry_elapsed += pcfg.dt_s;
            if (rev_entry->arrived(adapter.runtimeSnapshot().estimate, entry_elapsed)) {
                rev_pipe.emplace(swarm, adapter, mid_single, back_dual, rev_moves, pcfg, &auditor);
                phase = RtPhase::RevReconfig;
            }
        } else if (phase == RtPhase::RevReconfig) {
            auto yaw = vel_yaws(adapter.runtimeSnapshot());
            const bool done = rev_pipe->stepOnce(&yaw);
            if (rev_pipe->finished() && !rev_pipe->completed()) {
                throw std::runtime_error("RevReconfig failed: " + rev_pipe->reason());
            }
            if (done) {
                rt.return_reconfig_s = t_s - rt.return_entry_s;
                rt.total_time_s = t_s;
                rt.return_completed = true;
                rt.static_parent_violations += rev_pipe->staticParentViolations();
                rt.dag_violations += rev_pipe->dagViolations();
                rt.total_mbb_swaps += rev_pipe->totalMbbSwaps();
                // Check final position error vs back_dual
                double max_err = 0.0;
                for (std::size_t i = 0; i < scen.mobile_ids.size(); ++i) {
                    const NodeId id = scen.mobile_ids[i];
                    const Point pt = swarm.robots[i]->model->xy();
                    max_err = std::max(max_err, (Eigen::Vector2d(pt.x, pt.y) - back_dual.positions.at(id)).norm());
                }
                rt.final_topology_matched = (max_err <= 4.0);
                phase = RtPhase::Done;
            }
        }

        const double cf = adapter.coverage().certifiedFraction();
        if (!rt.reached_t95 && cf >= 0.95 - 1.0e-12) {
            rt.reached_t95 = true;
            rt.t95_certified_s = t_s;
        }
        if (!rt.reached_t100 && adapter.coverage().reachedCertifiedT100()) {
            rt.reached_t100 = true;
            rt.t100_certified_s = t_s;
        }
    }

    rt.final_certified_coverage = adapter.coverage().certifiedFraction();
    rt.false_certified_count = adapter.coverage().falseCertifiedCount();
    const auto sum = auditor.summarize();
    rt.collision_violations = sum.collision_violations;
    rt.reference_violations = sum.reference_violations;
    rt.axis_speed_violations = sum.axis_speed_violations;
    rt.position_rmse_m = sum.position_rmse_m;
    rt.overall_envelope_coverage = sum.overall_envelope_coverage;
    double worst = 1.0;
    for (const auto& [nid, cv] : sum.per_agent_envelope_coverage) {
        (void)nid;
        worst = std::min(worst, cv);
    }
    rt.worst_agent_envelope_coverage = worst;
    return bundle;
}

struct MarinePerturbationTrial {
    unsigned int seed = 0;
    bool completed = false;
    double makespan_s = 0.0;
    double rmse_m = 0.0;
    double overall_cenv = 0.0;
    double worst_cenv = 0.0;
    std::size_t violations = 0;
};

struct MarinePerturbationBatch {
    std::vector<MarinePerturbationTrial> runs;
    std::size_t safe_pass_count = 0;
    double makespan_mean_s = 0.0;
    double makespan_std_s = 0.0;
    double rmse_mean_m = 0.0;
    double rmse_std_m = 0.0;
    double cenv_mean = 0.0;
    double cenv_std = 0.0;
    double worst_cenv_min = 1.0;
};

inline MarinePerturbationBatch runMarinePerturbationMonteCarlo(int num_trials = 8) {
    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
    const auto p6 = buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3 = buildStaggeredTriangulation(
        b4, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto moves = planLabelledTransfer(p6, dl3);

    MarinePerturbationBatch batch;
    std::vector<double> mk, rms, cev;
    for (int k = 0; k < num_trials; ++k) {
        RouteBPipelinedConfig cfg;
        cfg.serial_mode = false;
        cfg.v_cruise_mps = 14.0;
        cfg.accel_mps2 = 2.0;
        cfg.d_pipe_m = 60.0;
        cfg.d_clear_m = 32.0;
        cfg.t_settle_s = 1.0;
        cfg.uwb_d_r_m = 600.0;
        cfg.uwb_sigma0_m = 1.0;
        cfg.uwb_d_max_m = 1200.0;
        cfg.link_seed = 2001u + static_cast<unsigned int>(k);
        cfg.range_measurement_scope = RangeMeasurementScope::ActiveTopologyAndAuxiliary;
        cfg.max_sim_time_s = 1200.0;
        cfg.plant_wind_disturbance_mps2 = Eigen::Vector2d(0.10, 0.06);
        cfg.range_positive_bias_m = 0.20;
        for (std::size_t idx = 0; idx < p6.beacon_ids.size(); ++idx) {
            const NodeId bid = p6.beacon_ids[idx];
            const double sx = (idx % 2 == 0) ? 0.08 : -0.08;
            const double sy = (idx < 2) ? 0.06 : -0.06;
            cfg.beacon_truth_offsets_m[bid] = Eigen::Vector2d(sx, sy);
        }
        const auto res = runClosedLoopReconfiguration(p6, dl3, moves, cfg);
        double wc = 1.0;
        for (const auto& [nid, cv] : res.audit.per_agent_envelope_coverage) {
            (void)nid;
            wc = std::min(wc, cv);
        }
        const std::size_t viol =
            res.audit.collision_violations + res.audit.reference_violations +
            res.audit.axis_speed_violations + res.static_parent_violations +
            res.dag_violations;
        const bool ok = res.completed && res.final_topology_matched && viol == 0;
        if (ok) ++batch.safe_pass_count;
        batch.worst_cenv_min = std::min(batch.worst_cenv_min, wc);
        batch.runs.push_back({
            cfg.link_seed, ok, res.makespan_s, res.audit.position_rmse_m,
            res.audit.overall_envelope_coverage, wc, viol});
        mk.push_back(res.makespan_s);
        rms.push_back(res.audit.position_rmse_m);
        cev.push_back(res.audit.overall_envelope_coverage);
    }

    auto mean_std = [](const std::vector<double>& v, double& m, double& s) {
        m = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
        double ss = 0.0;
        for (double x : v) ss += (x - m) * (x - m);
        s = (v.size() > 1) ? std::sqrt(ss / static_cast<double>(v.size() - 1)) : 0.0;
    };
    mean_std(mk, batch.makespan_mean_s, batch.makespan_std_s);
    mean_std(rms, batch.rmse_mean_m, batch.rmse_std_m);
    mean_std(cev, batch.cenv_mean, batch.cenv_std);
    return batch;
}

}  // namespace gf
