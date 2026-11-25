#pragma once

#include "grand_finale/RouteBMissionRunner.hpp"
#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace gf {

enum class TwoRegimeDecisionKind {
    Regime1ReachabilitySwitch,
    Regime2EfficiencySwitch,
    Regime2SubcriticalStay
};

struct OnlineTwoRegimeInputs {
    double uncertified_max_reach_m = 0.0;
    double current_formation_reach_m = 0.0;
    double target_formation_reach_m = 0.0;
    double remaining_area_m2 = 0.0;
    double measured_r0_m2ps = 0.0;
    double predicted_r1_m2ps = 0.0;
    double predicted_t_trans_s = 0.0;
    double predicted_a_trans_m2 = 0.0;
};

struct OnlineTwoRegimeDecision {
    TwoRegimeDecisionKind regime = TwoRegimeDecisionKind::Regime2SubcriticalStay;
    bool should_switch = false;
    double remaining_area_m2 = 0.0;
    double a_crit_m2 = std::numeric_limits<double>::infinity();
    double predicted_t_stay_s = std::numeric_limits<double>::infinity();
    double predicted_t_switch_s = std::numeric_limits<double>::infinity();
};

inline OnlineTwoRegimeDecision evaluateOnlineTwoRegimeDecision(
    const OnlineTwoRegimeInputs& in) {
    OnlineTwoRegimeDecision out;
    out.remaining_area_m2 = std::max(0.0, in.remaining_area_m2);

    // Regime 1: uncertified offshore band exceeds current formation reach but fits target reach.
    if (in.uncertified_max_reach_m > in.current_formation_reach_m + 1.0e-6 &&
        in.target_formation_reach_m >= in.uncertified_max_reach_m - 1.0e-6) {
        out.regime = TwoRegimeDecisionKind::Regime1ReachabilitySwitch;
        out.should_switch = true;
        out.a_crit_m2 = 0.0;
        return out;
    }

    // Regime 2: both formations can reach Omega_rem; apply first-order break-even criterion.
    if (in.measured_r0_m2ps > 1.0e-9) {
        out.predicted_t_stay_s = out.remaining_area_m2 / in.measured_r0_m2ps;
    }
    if (in.predicted_r1_m2ps > in.measured_r0_m2ps + 1.0e-9 &&
        in.measured_r0_m2ps > 1.0e-9) {
        const double dr = in.predicted_r1_m2ps - in.measured_r0_m2ps;
        out.a_crit_m2 =
            (in.measured_r0_m2ps * in.predicted_r1_m2ps / dr) * in.predicted_t_trans_s -
            (in.measured_r0_m2ps / dr) * in.predicted_a_trans_m2;
        out.predicted_t_switch_s =
            in.predicted_t_trans_s +
            std::max(0.0, out.remaining_area_m2 - in.predicted_a_trans_m2) /
                in.predicted_r1_m2ps;
        if (out.remaining_area_m2 > out.a_crit_m2) {
            out.regime = TwoRegimeDecisionKind::Regime2EfficiencySwitch;
            out.should_switch = true;
            return out;
        }
    }

    out.regime = TwoRegimeDecisionKind::Regime2SubcriticalStay;
    out.should_switch = false;
    return out;
}

inline RouteBConvexDomainSpec buildSubcriticalCoastalRegime2Spec() {
    RouteBConvexDomainSpec dom;
    dom.name = "subcritical_coastal_regime2";
    dom.description =
        "Sub-critical coastal convex belt below A_crit where both single and dual chains reach 100% online and staying outperforms switching";
    dom.vertices = {
        {-400.0, 120.0},
        {2400.0, 120.0},
        {2100.0, 1380.0},
        {-100.0, 1380.0},
    };
    const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0, 1600.0, 2000.0};
    dom.initial_tri = buildStaggeredTriangulation(
        b_xs, {{}, {}, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {}, {}},
        380.0, 90.0, 100.0, 140.0);
    dom.target_tri = buildStaggeredTriangulation(
        b_xs, {{1, 2, 3, 4, 5}, {}, {}, {}, {6, 7, 8, 9, 10}},
        380.0, 90.0, 100.0, 140.0);
    dom.use_direct_matched = true;
    return dom;
}

struct SubcriticalMissionBundle {
    RouteBMissionComparison comparison;
    OnlineTwoRegimeInputs decision_inputs;
    OnlineTwoRegimeDecision online_decision;
};

inline double estimateCertifiedReachY(
    const RouteBTriangulation& tri,
    double patrol_radius_m,
    double sensor_radius_m,
    double conservative_reserve_m = 40.0) {
    double max_y = 0.0;
    for (const auto& [nid, pos] : tri.positions) {
        (void)nid;
        max_y = std::max(max_y, pos.y());
    }
    return max_y + patrol_radius_m + sensor_radius_m - conservative_reserve_m;
}

inline OnlineTwoRegimeDecision evaluateMissionArmDecisionAtFork(
    const RouteBMissionComparison& res,
    const RouteBTriangulation& initial_tri,
    const RouteBTriangulation& target_tri,
    double domain_max_y_m,
    double patrol_radius_m,
    double sensor_radius_m,
    double n_units_src,
    double n_units_dst,
    double predicted_t_trans_s,
    double predicted_a_trans_m2,
    OnlineTwoRegimeInputs& out_inputs) {
    double cov_fork = res.fork_certified_coverage;
    double r0_meas = 2000.0;
    if (!res.stay.coverage_curve.empty()) {
        std::size_t idx_start = 0;
        double best_start_diff = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < res.stay.coverage_curve.size(); ++i) {
            const double diff = std::abs(res.stay.coverage_curve[i].time_s - res.fork_time_s);
            if (diff < best_start_diff) {
                best_start_diff = diff;
                idx_start = i;
            }
        }
        cov_fork = res.stay.coverage_curve[idx_start].certified_coverage;
        const double t_start = res.stay.coverage_curve[idx_start].time_s;
        std::size_t idx_end = res.stay.coverage_curve.size() - 1;
        for (std::size_t i = idx_start + 1; i < res.stay.coverage_curve.size(); ++i) {
            idx_end = i;
            if (res.stay.coverage_curve[i].time_s >= t_start + 60.0 - 1.0e-6) {
                break;
            }
        }
        if (idx_end > idx_start) {
            const double dt_eval = std::max(1.0, res.stay.coverage_curve[idx_end].time_s - t_start);
            r0_meas = std::max(
                100.0,
                ((res.stay.coverage_curve[idx_end].certified_coverage - cov_fork) * res.area_m2) / dt_eval);
        } else {
            const double t0 = res.stay.coverage_curve.front().time_s;
            const double cov0 = res.stay.coverage_curve.front().certified_coverage;
            const double dt_eval = std::max(1.0, t_start - t0);
            r0_meas = std::max(100.0, ((cov_fork - cov0) * res.area_m2) / dt_eval);
        }
    }

    out_inputs.uncertified_max_reach_m = domain_max_y_m;
    out_inputs.current_formation_reach_m =
        estimateCertifiedReachY(initial_tri, patrol_radius_m, sensor_radius_m);
    out_inputs.target_formation_reach_m =
        estimateCertifiedReachY(target_tri, patrol_radius_m, sensor_radius_m);
    out_inputs.remaining_area_m2 = (1.0 - cov_fork) * res.area_m2;
    out_inputs.measured_r0_m2ps = r0_meas;
    out_inputs.predicted_r1_m2ps = r0_meas * (n_units_dst / std::max(1.0, n_units_src));
    out_inputs.predicted_t_trans_s = predicted_t_trans_s;
    out_inputs.predicted_a_trans_m2 = predicted_a_trans_m2;
    return evaluateOnlineTwoRegimeDecision(out_inputs);
}

inline SubcriticalMissionBundle runSubcriticalCoastalMission(
    const std::string& channel_group = "nominal",
    bool record_trajectories = false) {
    const auto dom = buildSubcriticalCoastalRegime2Spec();
    RouteBMissionConfig mcfg;
    mcfg.domain_name = dom.name;
    mcfg.custom_domain = dom;
    mcfg.cell_size_m = 10.0;
    mcfg.sensor_radius_m = 400.0;
    mcfg.dynamic_chain_sweep = true;
    mcfg.search_speed_cap_mps = 5.0;
    mcfg.patrol_radius_m = 1800.0;
    mcfg.v_cruise_mps = 23.5;
    mcfg.accel_mps2 = 3.5;
    mcfg.d_pipe_m = 45.0;
    mcfg.t_settle_s = 0.2;
    mcfg.t_fork_s = 10.0;
    mcfg.max_mission_s = 2200.0;
    if (channel_group == "degraded") {
        mcfg.uwb_d_r_m = 600.0;
        mcfg.uwb_sigma0_m = 1.0;
    } else {
        mcfg.uwb_d_r_m = 850.0;
        mcfg.uwb_sigma0_m = 0.5;
    }
    mcfg.uwb_d_max_m = 1200.0;
    mcfg.link_seed = 20261123;
    mcfg.curve_sample_interval_s = 10.0;
    mcfg.record_trajectory_samples = record_trajectories;
    mcfg.trajectory_sample_interval_s = 20.0;

    SubcriticalMissionBundle out;
    out.comparison = runTwoRegimeMissionComparison(mcfg);
    out.online_decision = evaluateMissionArmDecisionAtFork(
        out.comparison,
        dom.initial_tri,
        dom.target_tri,
        1380.0,
        mcfg.patrol_radius_m,
        mcfg.sensor_radius_m,
        1.0,
        2.0,
        604.0,
        0.16 * out.comparison.area_m2,
        out.decision_inputs);
    return out;
}

struct RouteBComputeAndCommStats {
    int scale_n = 0;
    std::size_t ticks = 0;
    std::size_t qp_calls = 0;
    double qp_solve_mean_ms = 0.0;
    double qp_solve_p99_ms = 0.0;
    double qp_solve_max_ms = 0.0;
    double qp_total_mean_ms = 0.0;
    double qp_total_p99_ms = 0.0;
    double estimator_mean_ms = 0.0;
    double estimator_p99_ms = 0.0;
    double tick_total_mean_ms = 0.0;
    double tick_total_p99_ms = 0.0;
    double mean_packets_per_tick = 0.0;
    double mean_accepted_packets_per_tick = 0.0;
    double mean_bandwidth_bytes_per_s = 0.0;
};

inline RouteBComputeAndCommStats summarizeTimingResult(
    int scale_n,
    const RouteBPipelinedRunResult& res,
    double dt_s = 0.1) {
    auto summarize_vec_ms = [](std::vector<double> vals,
                               double& mean_ms,
                               double& p99_ms,
                               double& max_ms) {
        if (vals.empty()) {
            mean_ms = p99_ms = max_ms = 0.0;
            return;
        }
        const double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
        mean_ms = 1.0e3 * (sum / static_cast<double>(vals.size()));
        std::sort(vals.begin(), vals.end());
        const std::size_t idx99 = std::min(
            vals.size() - 1,
            static_cast<std::size_t>(std::ceil(0.99 * static_cast<double>(vals.size()))) - 1);
        p99_ms = 1.0e3 * vals[idx99];
        max_ms = 1.0e3 * vals.back();
    };

    RouteBComputeAndCommStats s;
    s.scale_n = scale_n;
    s.ticks = res.ticks;
    s.qp_calls = res.per_uav_qp_solve_s.size();
    summarize_vec_ms(res.per_uav_qp_solve_s, s.qp_solve_mean_ms, s.qp_solve_p99_ms, s.qp_solve_max_ms);
    double dummy_max = 0.0;
    summarize_vec_ms(res.per_uav_qp_total_s, s.qp_total_mean_ms, s.qp_total_p99_ms, dummy_max);
    summarize_vec_ms(res.estimator_samples_s, s.estimator_mean_ms, s.estimator_p99_ms, dummy_max);
    summarize_vec_ms(res.tick_total_samples_s, s.tick_total_mean_ms, s.tick_total_p99_ms, dummy_max);
    if (res.ticks > 0) {
        s.mean_packets_per_tick =
            static_cast<double>(res.total_scheduled_range_packets) / static_cast<double>(res.ticks);
        s.mean_accepted_packets_per_tick =
            static_cast<double>(res.total_accepted_range_packets) / static_cast<double>(res.ticks);
        // Compact 48-byte UWB ranging + covariance-compressed coordination packet per scheduled edge
        // plus N 32-byte state/token broadcasts per 10 Hz tick.
        const double packets_per_s = (s.mean_packets_per_tick + static_cast<double>(scale_n)) / dt_s;
        s.mean_bandwidth_bytes_per_s =
            (s.mean_packets_per_tick * 48.0 + static_cast<double>(scale_n) * 32.0) / dt_s;
        (void)packets_per_s;
    }
    return s;
}

inline RouteBComputeAndCommStats measureRouteBComputeAndComm(int scale_n) {
    RouteBTriangulation src, dst;
    std::vector<RouteBMove> moves;
    RouteBPipelinedConfig cfg;
    cfg.serial_mode = false;
    cfg.accel_mps2 = 2.0;
    cfg.d_clear_m = 32.0;
    cfg.t_settle_s = 1.0;
    cfg.uwb_d_r_m = 850.0;
    cfg.uwb_sigma0_m = 0.5;
    cfg.uwb_d_max_m = 1200.0;
    cfg.link_seed = 20261123;
    cfg.range_measurement_scope = RangeMeasurementScope::ActiveTopologyAndAuxiliary;
    cfg.record_timing_stats = true;

    if (scale_n == 6) {
        const std::vector<double> b3 = {0.0, 400.0, 800.0};
        src = buildStaggeredTriangulation(b3, {{1, 2, 3}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
        dst = buildStaggeredTriangulation(b3, {{6, 5, 4, 3, 2, 1}, {}}, 380.0, 90.0, 100.0, 140.0);
        std::tie(moves, dst) = planDirectMatchedTransfer(src, dst);
        cfg.v_cruise_mps = 14.0;
        cfg.d_pipe_m = 60.0;
        cfg.max_sim_time_s = 1200.0;
    } else if (scale_n == 14) {
        const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
        src = buildStaggeredTriangulation(
            b4, {{1, 2, 3, 4, 5, 6, 7}, {}, {8, 9, 10, 11, 12, 13, 14}}, 380.0, 90.0, 100.0, 140.0);
        dst = buildStaggeredTriangulation(
            b4, {{}, {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {}}, 380.0, 90.0, 100.0, 140.0);
        std::tie(moves, dst) = planDirectMatchedTransfer(src, dst);
        cfg.v_cruise_mps = 15.0;
        cfg.d_pipe_m = 55.0;
        cfg.max_sim_time_s = 1800.0;
    } else if (scale_n == 20) {
        const std::vector<double> b5 = {0.0, 400.0, 800.0, 1200.0, 1600.0};
        src = buildStaggeredTriangulation(
            b5, {{1, 2, 3, 4, 5}, {6, 7, 8, 9, 10}, {11, 12, 13, 14, 15}, {16, 17, 18, 19, 20}},
            380.0, 90.0, 100.0, 140.0);
        dst = buildStaggeredTriangulation(
            b5, {{10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {}, {}, {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}},
            380.0, 90.0, 100.0, 140.0);
        std::tie(moves, dst) = planDirectMatchedTransfer(src, dst);
        cfg.v_cruise_mps = 16.0;
        cfg.d_pipe_m = 55.0;
        cfg.max_sim_time_s = 1800.0;
    } else {
        throw std::invalid_argument("unsupported scale_n for timing benchmark");
    }

    const auto res = runClosedLoopReconfiguration(src, dst, moves, cfg);
    if (!res.completed) {
        throw std::runtime_error("timing benchmark failed to complete: " + res.reason);
    }
    return summarizeTimingResult(scale_n, res, cfg.dt_s);
}

inline RouteBPipelinedRunResult runMildMarinePerturbationCheck() {
    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
    const auto p6 = buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3 = buildStaggeredTriangulation(
        b4, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto moves = planLabelledTransfer(p6, dl3);

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
    cfg.link_seed = 1001;
    cfg.range_measurement_scope = RangeMeasurementScope::ActiveTopologyAndAuxiliary;
    cfg.max_sim_time_s = 1200.0;
    // Mild unmodeled perturbations: bounded continuous wind gust (0.12 m/s^2),
    // positive sea-surface multipath bias (+0.20 m), and decimeter shore beacon drift (0.10 m).
    cfg.plant_wind_disturbance_mps2 = Eigen::Vector2d(0.10, 0.06);
    cfg.range_positive_bias_m = 0.20;
    for (std::size_t idx = 0; idx < p6.beacon_ids.size(); ++idx) {
        const NodeId bid = p6.beacon_ids[idx];
        const double sx = (idx % 2 == 0) ? 0.08 : -0.08;
        const double sy = (idx < 2) ? 0.06 : -0.06;
        cfg.beacon_truth_offsets_m[bid] = Eigen::Vector2d(sx, sy);
    }
    return runClosedLoopReconfiguration(p6, dl3, moves, cfg);
}

}  // namespace gf
