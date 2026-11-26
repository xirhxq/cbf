#pragma once

#include "grand_finale/RouteBMissionRunner.hpp"
#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"
#include "grand_finale/RouteBWave10Validations.hpp"
#include "grand_finale/RouteBWave9Validations.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct TwoSidedBerthBounds {
    bool feasible = false;
    double h_b_min_m = 0.0;
    double h_b_max_m = 0.0;
};

inline TwoSidedBerthBounds computeTwoSidedBerthBounds(
    const std::vector<double>& beacon_xs,
    double iota_req) {
    TwoSidedBerthBounds out;
    if (beacon_xs.size() < 2 || !std::isfinite(iota_req) ||
        iota_req <= 0.0 || iota_req > 2.0 / 3.0 + 1.0e-12) {
        return out;
    }
    double s_min = std::numeric_limits<double>::infinity();
    double s_max = 0.0;
    for (std::size_t r = 0; r + 1 < beacon_xs.size(); ++r) {
        const double sr = beacon_xs[r + 1] - beacon_xs[r];
        if (!std::isfinite(sr) || sr <= 0.0) {
            return out;
        }
        s_min = std::min(s_min, sr);
        s_max = std::max(s_max, sr);
    }
    const double clamped_iota = std::min(iota_req, 2.0 / 3.0);
    out.h_b_min_m = 0.5 * s_max * std::sqrt(clamped_iota / (2.0 - clamped_iota));
    out.h_b_max_m = s_min * (1.0 - clamped_iota) /
                    std::sqrt(clamped_iota * (2.0 - clamped_iota));
    out.feasible = (out.h_b_min_m <= out.h_b_max_m + 1.0e-9);
    return out;
}

inline bool checkTwoSidedBerthCondition(
    const std::vector<double>& beacon_xs,
    double h_b_m,
    double iota_req) {
    if (!std::isfinite(h_b_m) || h_b_m <= 0.0) {
        return false;
    }
    const auto b = computeTwoSidedBerthBounds(beacon_xs, iota_req);
    if (!b.feasible) {
        return false;
    }
    return (h_b_m >= b.h_b_min_m - 1.0e-9) && (h_b_m <= b.h_b_max_m + 1.0e-9);
}

inline double computePlanarMultiFormationReachableFraction(
    const std::vector<const RouteBTriangulation*>& tris,
    const std::vector<Eigen::Vector2d>& polygon_vertices,
    double patrol_radius_m,
    double sensor_radius_m,
    double d_dep_m = 850.0,
    double reserve_m = 40.0,
    double cell_size_m = 20.0) {
    struct HopData {
        std::map<NodeId, int> hop;
        std::map<NodeId, Eigen::Vector2d> root_anchor;
    };
    std::vector<HopData> per_tri(tris.size());
    for (std::size_t ti = 0; ti < tris.size(); ++ti) {
        const RouteBTriangulation& tri = *tris[ti];
        auto& hop = per_tri[ti].hop;
        auto& root_anchor = per_tri[ti].root_anchor;
        for (NodeId b : tri.beacon_ids) {
            hop[b] = 0;
            root_anchor[b] = tri.beacons.at(b);
        }
        for (std::size_t iter = 0; iter < tri.mobile_ids.size(); ++iter) {
            for (NodeId id : tri.mobile_ids) {
                if (hop.count(id)) continue;
                const auto ps = tri.parents.at(id);
                if (hop.count(ps.first) && hop.count(ps.second)) {
                    hop[id] = 1 + std::max(hop.at(ps.first), hop.at(ps.second));
                    const bool b1 = tri.isBeacon(ps.first);
                    const bool b2 = tri.isBeacon(ps.second);
                    if (b1 && b2) {
                        root_anchor[id] = 0.5 * (tri.beacons.at(ps.first) + tri.beacons.at(ps.second));
                    } else if (b1) {
                        root_anchor[id] = tri.beacons.at(ps.first);
                    } else if (b2) {
                        root_anchor[id] = tri.beacons.at(ps.second);
                    } else {
                        root_anchor[id] = (hop.at(ps.first) <= hop.at(ps.second))
                            ? root_anchor.at(ps.first)
                            : root_anchor.at(ps.second);
                    }
                }
            }
        }
    }

    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (const auto& v : polygon_vertices) {
        x_min = std::min(x_min, v.x());
        x_max = std::max(x_max, v.x());
        y_min = std::min(y_min, v.y());
        y_max = std::max(y_max, v.y());
    }
    auto in_poly = [&](const Eigen::Vector2d& p) {
        bool inside = false;
        const std::size_t n = polygon_vertices.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const auto& a = polygon_vertices[i];
            const auto& b = polygon_vertices[j];
            const bool cross = ((a.y() > p.y()) != (b.y() > p.y())) &&
                (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / ((b.y() - a.y()) + 1.0e-18) + a.x());
            if (cross) inside = !inside;
        }
        return inside;
    };

    std::size_t total_cells = 0;
    std::size_t reachable_cells = 0;
    for (double y = y_min + 0.5 * cell_size_m; y <= y_max; y += cell_size_m) {
        for (double x = x_min + 0.5 * cell_size_m; x <= x_max; x += cell_size_m) {
            const Eigen::Vector2d z(x, y);
            if (!in_poly(z)) continue;
            ++total_cells;
            bool ok = false;
            for (std::size_t ti = 0; ti < tris.size() && !ok; ++ti) {
                const RouteBTriangulation& tri = *tris[ti];
                const auto& hop = per_tri[ti].hop;
                const auto& root_anchor = per_tri[ti].root_anchor;
                for (NodeId id : tri.mobile_ids) {
                    const Eigen::Vector2d& q = tri.positions.at(id);
                    const int h = hop.count(id) ? hop.at(id) : 1;
                    const double anchor_dist = (q - root_anchor.at(id)).norm();
                    const double r_patrol_eff = std::max(
                        0.0,
                        std::min(patrol_radius_m, static_cast<double>(h) * d_dep_m - anchor_dist));
                    const double rad = r_patrol_eff + sensor_radius_m - reserve_m;
                    if ((z - q).norm() <= rad) {
                        ok = true;
                        break;
                    }
                }
            }
            if (ok) ++reachable_cells;
        }
    }
    return total_cells > 0
        ? static_cast<double>(reachable_cells) / static_cast<double>(total_cells)
        : 0.0;
}

inline double computePlanarReachableFraction(
    const RouteBTriangulation& tri,
    const std::vector<Eigen::Vector2d>& polygon_vertices,
    double patrol_radius_m,
    double sensor_radius_m,
    double d_dep_m = 850.0,
    double reserve_m = 40.0,
    double cell_size_m = 20.0) {
    return computePlanarMultiFormationReachableFraction(
        {&tri}, polygon_vertices, patrol_radius_m, sensor_radius_m, d_dep_m, reserve_m, cell_size_m);
}

inline double computePlanarUnionReachableFraction(
    const RouteBTriangulation& tri_src,
    const RouteBTriangulation& tri_dst,
    const std::vector<Eigen::Vector2d>& polygon_vertices,
    double patrol_radius_m,
    double sensor_radius_m,
    double d_dep_m = 850.0,
    double reserve_m = 40.0,
    double cell_size_m = 20.0) {
    return computePlanarMultiFormationReachableFraction(
        {&tri_src, &tri_dst},
        polygon_vertices,
        patrol_radius_m,
        sensor_radius_m,
        d_dep_m,
        reserve_m,
        cell_size_m);
}

struct PlanarTwoRegimeDecisionSummary {
    std::string domain_name;
    double area_km2 = 0.0;
    double src_reachable_fraction = 0.0;
    double dst_reachable_fraction = 0.0;
    double union_reachable_fraction = 0.0;
    TwoRegimeDecisionKind regime = TwoRegimeDecisionKind::Regime2SubcriticalStay;
    bool should_switch = false;
    double a_crit_km2 = 0.0;
    double remaining_area_km2 = 0.0;
};

inline std::vector<PlanarTwoRegimeDecisionSummary> evaluateAllPlanarTwoRegimeDomains() {
    std::vector<PlanarTwoRegimeDecisionSummary> out;
    const auto base = standardRouteBConvexDomains();
    auto find_std = [&](const std::string& name) {
        for (const auto& d : base) {
            if (d.name == name) return d;
        }
        throw std::runtime_error("missing std domain: " + name);
    };

    struct SpecInput {
        RouteBConvexDomainSpec dom;
        double area_km2;
        double patrol_radius_m;
        double measured_r0_m2ps;
        double n_src;
        double n_dst;
        double predicted_t_trans_s;
        double predicted_a_trans_km2;
        double fork_cov_frac;
    };

    RouteBConvexDomainSpec broad_dom;
    broad_dom.name = "wide_broad_regime2";
    broad_dom.vertices = {{-2050.0, 120.0}, {4050.0, 120.0}, {2950.0, 2050.0}, {-950.0, 2050.0}};
    const std::vector<double> b6 = {0.0, 400.0, 800.0, 1200.0, 1600.0, 2000.0};
    broad_dom.initial_tri = buildStaggeredTriangulation(
        b6, {{}, {}, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {}, {}}, 380.0, 90.0, 100.0, 140.0);
    broad_dom.target_tri = buildStaggeredTriangulation(
        b6, {{1, 2, 3, 4, 5}, {}, {}, {}, {6, 7, 8, 9, 10}}, 380.0, 90.0, 100.0, 140.0);

    const std::vector<SpecInput> items = {
        {find_std("narrow_deep_offshore"), 0.494, 105.0, 2800.0, 2.0, 1.0, 905.3, 0.08, 0.847},
        {find_std("oblique_asymmetric"), 0.839, 105.0, 2800.0, 1.0, 2.0, 562.0, 0.08, 0.879},
        {buildSubcriticalCoastalRegime2Spec(), 3.1506, 1800.0, 3513.3, 1.0, 2.0, 604.0, 0.504, 0.3438},
        {broad_dom, 9.650, 2600.0, 4035.0, 1.0, 2.0, 603.8, 1.573, 0.1220},
    };

    for (const auto& it : items) {
        PlanarTwoRegimeDecisionSummary s;
        s.domain_name = it.dom.name;
        s.area_km2 = it.area_km2;
        s.src_reachable_fraction = computePlanarReachableFraction(
            it.dom.initial_tri, it.dom.vertices, it.patrol_radius_m, 400.0, 850.0, 40.0, 20.0);
        s.dst_reachable_fraction = computePlanarReachableFraction(
            it.dom.target_tri, it.dom.vertices, it.patrol_radius_m, 400.0, 850.0, 40.0, 20.0);
        s.union_reachable_fraction = computePlanarUnionReachableFraction(
            it.dom.initial_tri, it.dom.target_tri, it.dom.vertices, it.patrol_radius_m, 400.0, 850.0, 40.0, 20.0);
        s.remaining_area_km2 = (1.0 - it.fork_cov_frac) * it.area_km2;

        if (s.src_reachable_fraction < 0.99 &&
            s.union_reachable_fraction > s.src_reachable_fraction + 0.025) {
            s.regime = TwoRegimeDecisionKind::Regime1ReachabilitySwitch;
            s.should_switch = true;
            s.a_crit_km2 = 0.0;
        } else {
            OnlineTwoRegimeInputs in;
            in.uncertified_max_reach_m = 0.0;
            in.current_formation_reach_m = 1.0;
            in.target_formation_reach_m = 1.0;
            in.remaining_area_m2 = s.remaining_area_km2 * 1.0e6;
            in.measured_r0_m2ps = it.measured_r0_m2ps;
            in.predicted_r1_m2ps = it.measured_r0_m2ps * (it.n_dst / it.n_src);
            in.predicted_t_trans_s = it.predicted_t_trans_s;
            in.predicted_a_trans_m2 = it.predicted_a_trans_km2 * 1.0e6;
            const auto d = evaluateOnlineTwoRegimeDecision(in);
            s.regime = d.regime;
            s.should_switch = d.should_switch;
            s.a_crit_km2 = d.a_crit_m2 / 1.0e6;
        }
        out.push_back(std::move(s));
    }
    return out;
}

struct NonuniformNominalMissionResult {
    bool stay_reached_t100 = false;
    bool switch_reached_t100 = false;
    double stay_final_cov = 0.0;
    std::optional<double> stay_t95_s;
    std::optional<double> switch_t95_s;
    std::optional<double> switch_t100_s;
    double reconfig_s = 0.0;
    int false_certified_count = 0;
    std::size_t collision_violations = 0;
    std::size_t reference_violations = 0;
    double worst_agent_envelope_coverage = 0.0;
    double overall_envelope_coverage = 0.0;
};

inline NonuniformNominalMissionResult runNonuniformPentagonNominalMission() {
    const std::vector<double> b_xs = {0.0, 320.0, 720.0, 1080.0};
    RouteBConvexDomainSpec pent;
    pent.name = "nonuniform_oblique_pentagon";
    pent.description = "Irregular oblique pentagon over (320, 400, 360) m under nominal UWB";
    pent.vertices = {{60.0, 220.0}, {880.0, 250.0}, {1140.0, 640.0}, {680.0, 980.0}, {180.0, 740.0}};
    pent.initial_tri = buildStaggeredTriangulation(b_xs, {{1, 2, 3}, {}, {4, 5, 6}}, 360.0, 68.0, 100.0, 140.0);
    pent.target_tri = buildStaggeredTriangulation(b_xs, {{}, {6, 5, 4, 3, 2, 1}, {}}, 360.0, 68.0, 100.0, 140.0);
    pent.use_direct_matched = false;

    RouteBMissionConfig mcfg;
    mcfg.domain_name = pent.name;
    mcfg.custom_domain = pent;
    mcfg.cell_size_m = 10.0;
    mcfg.sensor_radius_m = 400.0;
    mcfg.t_fork_s = 12.0;
    mcfg.max_mission_s = 900.0;
    mcfg.patrol_radius_m = 105.0;
    mcfg.v_cruise_mps = 14.0;
    mcfg.accel_mps2 = 2.0;
    mcfg.d_pipe_m = 50.0;
    mcfg.t_settle_s = 1.0;
    mcfg.uwb_d_r_m = 850.0;
    mcfg.uwb_sigma0_m = 0.5;
    mcfg.uwb_d_max_m = 1200.0;
    mcfg.link_seed = 1001;

    const auto cmp = runTwoRegimeMissionComparison(mcfg);
    NonuniformNominalMissionResult r;
    r.stay_reached_t100 = cmp.stay.reached_t100;
    r.switch_reached_t100 = cmp.switch_arm.reached_t100;
    r.stay_final_cov = cmp.stay.final_certified_coverage;
    r.stay_t95_s = cmp.stay.t95_certified_s;
    r.switch_t95_s = cmp.switch_arm.t95_certified_s;
    r.switch_t100_s = cmp.switch_arm.t100_certified_s;
    r.reconfig_s = cmp.switch_arm.reconfig_complete_s - cmp.switch_arm.reconfig_entry_s;
    r.false_certified_count =
        cmp.stay.false_certified_count + cmp.switch_arm.false_certified_count;
    r.collision_violations =
        cmp.stay.audit.collision_violations + cmp.switch_arm.audit.collision_violations;
    r.reference_violations =
        cmp.stay.audit.reference_violations + cmp.switch_arm.audit.reference_violations;
    r.overall_envelope_coverage = cmp.switch_arm.audit.overall_envelope_coverage;
    double wc = 1.0;
    for (const auto& [nid, cv] : cmp.switch_arm.audit.per_agent_envelope_coverage) {
        (void)nid;
        wc = std::min(wc, cv);
    }
    r.worst_agent_envelope_coverage = wc;
    return r;
}

inline ReversibleRoundTripResult runPerturbedReversibleSearchValidation() {
    const auto dom = buildReversiblePentagonDomainSpec();

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
    pcfg.waypoint_pos_tol_m = 4.0;
    pcfg.waypoint_speed_tol_mps = 1.4;
    pcfg.terminal_pos_tol_m = 4.0;
    pcfg.terminal_speed_tol_mps = 1.2;
    pcfg.uwb_d_r_m = 600.0;
    pcfg.uwb_sigma0_m = 1.0;
    pcfg.uwb_d_max_m = 1200.0;
    pcfg.link_seed = 20261123;
    pcfg.maximum_range_aoi_s = 3.5;
    pcfg.plant_wind_disturbance_mps2 = Eigen::Vector2d(0.10, 0.06);
    pcfg.range_positive_bias_m = 0.20;
    for (std::size_t idx = 0; idx < dom.initial_tri.beacon_ids.size(); ++idx) {
        const NodeId bid = dom.initial_tri.beacon_ids[idx];
        const double sx = (idx % 2 == 0) ? 0.08 : -0.08;
        const double sy = (idx < 2) ? 0.06 : -0.06;
        pcfg.beacon_truth_offsets_m[bid] = Eigen::Vector2d(sx, sy);
    }

    Task10p10Scenario scen;
    scen.id = "route_b_wave11_perturbed_reversible";
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
    acfg.range_measurement_scope = RangeMeasurementScope::AllWithinCutoff;
    acfg.sensor_radius_m = 400.0;
    acfg.coverage_footprint_kind = CoverageFootprintKind::ForwardSector;
    acfg.coverage_half_angle_rad = M_PI / 3.0;
    acfg.maximum_yaw_rate_radps = 1.0;
    acfg.plant_wind_disturbance_mps2 = pcfg.plant_wind_disturbance_mps2;
    acfg.range_positive_bias_m = pcfg.range_positive_bias_m;
    acfg.beacon_truth_offsets_m = pcfg.beacon_truth_offsets_m;

    Swarm swarm(settings);
    GrandFinaleSwarmAdapter adapter(
        swarm, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
    if (!adapter.initializeStageZero().initialized) {
        throw std::runtime_error("perturbed reversible stage_zero_init_failed");
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
        ecfg.velocity_capture_tol_mps = 1.2;
        ecfg.arrival_pos_tol_m = 4.0;
        ecfg.arrival_speed_tol_mps = 1.2;
        exec_opt.emplace(p0, v0, dest_slots.positions, pivot, ecfg);
        entry_elapsed = 0.0;
    };

    ReversibleRoundTripResult rt;
    const std::size_t max_ticks = 24000;
    for (std::size_t tick = 0; tick < max_ticks && phase != RtPhase::Done; ++tick) {
        const double t_s = static_cast<double>(tick + 1) * pcfg.dt_s;
        if (phase == RtPhase::InitSweep) {
            std::map<NodeId, Eigen::Vector2d> nom;
            std::map<NodeId, double> yaw;
            init_sweep.computeCommands(swarm, adapter, adapter.runtimeSnapshot().estimate, tick, nom, yaw);
            const auto st = adapter.stepWithNominalAndYawRates(nom, yaw);
            if (!st.advanced) throw std::runtime_error("PertInitSweep aborted: " + st.reason);
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
            if (!st.advanced) throw std::runtime_error("PertFwdEntry aborted: " + st.reason);
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
                throw std::runtime_error("PertFwdReconfig failed: " + fwd_pipe->reason());
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
            if (!st.advanced) throw std::runtime_error("PertSingleSweep aborted: " + st.reason);
            auditor.observe(swarm, adapter.runtimeSnapshot());
            if (adapter.coverage().reachedCertifiedT100() || t_s >= 900.0) {
                rt.return_entry_s = t_s;
                start_entry(mid_single, rev_entry);
                phase = RtPhase::RevEntry;
            }
        } else if (phase == RtPhase::RevEntry) {
            const auto snap = adapter.runtimeSnapshot();
            const auto nom = rev_entry->nominalControls(snap.estimate, entry_elapsed);
            const auto yaw = vel_yaws(snap);
            const auto st = adapter.stepWithNominalAndYawRates(nom, yaw);
            if (!st.advanced) throw std::runtime_error("PertRevEntry aborted: " + st.reason);
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
                throw std::runtime_error("PertRevReconfig failed: " + rev_pipe->reason());
            }
            if (done) {
                rt.return_reconfig_s = t_s - rt.return_entry_s;
                rt.total_time_s = t_s;
                rt.return_completed = true;
                rt.static_parent_violations += rev_pipe->staticParentViolations();
                rt.dag_violations += rev_pipe->dagViolations();
                rt.total_mbb_swaps += rev_pipe->totalMbbSwaps();
                double max_err = 0.0;
                for (std::size_t i = 0; i < scen.mobile_ids.size(); ++i) {
                    const NodeId id = scen.mobile_ids[i];
                    const Point pt = swarm.robots[i]->model->xy();
                    max_err = std::max(max_err, (Eigen::Vector2d(pt.x, pt.y) - back_dual.positions.at(id)).norm());
                }
                rt.final_topology_matched = (max_err <= 8.0);
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
    return rt;
}

}  // namespace gf
