#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBMissionRunner.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

namespace {

// Pre-optimization full-grid observeSector + full-grid uncovered scan reference
void referenceObserveSectorFull(
    GridWorld& truth,
    GridWorld& cert,
    double half_diag,
    const Point& truth_pos,
    const Point& est_pos,
    double err_bound_m,
    double inner_r,
    double outer_r,
    double half_ang,
    double heading) {
    const double reserve = err_bound_m + half_diag;
    auto ang_err = [heading](double dy, double dx) {
        const double b = std::atan2(dy, dx);
        return std::abs(std::atan2(std::sin(b - heading), std::cos(b - heading)));
    };
    const double r_bound = outer_r + 1.0e-6;
    const double x_min = std::min(truth_pos.x, est_pos.x) - r_bound;
    const double x_max = std::max(truth_pos.x, est_pos.x) + r_bound;
    const double y_min = std::min(truth_pos.y, est_pos.y) - r_bound;
    const double y_max = std::max(truth_pos.y, est_pos.y) + r_bound;
    for (int x = 0; x < truth.xNum; ++x) {
        const double cx = truth.getCellCenterX(x);
        if (cx < x_min || cx > x_max) continue;
        for (int y = 0; y < truth.yNum; ++y) {
            const int idx = truth.getIndex(x, y);
            if (!truth.valid[idx] || (truth.vis[idx] && cert.vis[idx])) continue;
            const double cy = truth.getCellCenterY(y);
            if (cy < y_min || cy > y_max) continue;
            const double tdx = cx - truth_pos.x;
            const double tdy = cy - truth_pos.y;
            const double td2 = tdx * tdx + tdy * tdy;
            if (td2 <= (outer_r + 1e-12) * (outer_r + 1e-12)) {
                const double td = std::sqrt(td2);
                if (td >= inner_r - 1e-12 && td <= outer_r + 1e-12) {
                    const bool tang = td <= 1e-12 || ang_err(tdy, tdx) <= half_ang + 1e-12;
                    if (tang) truth.setValue(x, y, true);
                }
            }
            const double edx = cx - est_pos.x;
            const double edy = cy - est_pos.y;
            const double max_est = outer_r + 1e-12 - reserve;
            if (max_est >= 0.0 && edx * edx + edy * edy <= max_est * max_est) {
                const double ed = std::hypot(edx, edy);
                const bool rad_ok = ed + reserve <= outer_r + 1e-12 &&
                    (inner_r <= 1e-12 || ed - reserve >= inner_r - 1e-12);
                if (rad_ok) {
                    bool ang_ok = half_ang >= M_PI - 1e-12;
                    if (!ang_ok && ed > reserve) {
                        const double ares = std::asin(std::min(1.0, reserve / ed));
                        ang_ok = ang_err(edy, edx) + ares <= half_ang + 1e-12;
                    }
                    if (ang_ok) cert.setValue(x, y, true);
                }
            }
        }
    }
}

std::vector<Eigen::Vector2d> referenceScanUncovered(const GridWorld& cert_grid) {
    std::vector<Eigen::Vector2d> uncovered;
    const double x0 = cert_grid.xLim.first;
    const double x1 = cert_grid.xLim.second;
    const double y0 = cert_grid.yLim.first;
    const double y1 = cert_grid.yLim.second;
    for (int ix = 0; ix < cert_grid.xNum; ++ix) {
        const double rx = (static_cast<double>(ix) + 0.5) / static_cast<double>(cert_grid.xNum);
        const double cx = x0 * (1.0 - rx) + x1 * rx;
        for (int iy = 0; iy < cert_grid.yNum; ++iy) {
            const int idx = cert_grid.getIndex(ix, iy);
            if (cert_grid.valid[idx] && !cert_grid.vis[idx]) {
                const double ry = (static_cast<double>(iy) + 0.5) / static_cast<double>(cert_grid.yNum);
                uncovered.emplace_back(cx, y0 * (1.0 - ry) + y1 * ry);
            }
        }
    }
    return uncovered;
}

}  // namespace

TEST_CASE("TDD-11b FrontierSweepPolicy unsearched index cache tracks uncovered count and accelerates 10m grid") {
    const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0, 1600.0, 2000.0};
    const auto tri = gf::buildStaggeredTriangulation(
        b_xs, {{}, {}, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {}, {}},
        380.0, 90.0, 100.0, 140.0);

    gf::Task10p10Scenario scen;
    scen.id = "bench_10m";
    scen.width_m = 4500.0;
    scen.height_m = 2500.0;
    scen.mobile_ids = tri.mobile_ids;
    for (gf::NodeId id : scen.mobile_ids) {
        scen.mobile_positions.push_back(tri.positions.at(id));
    }
    scen.fixed_positions = tri.beacons;
    scen.initial_topology = tri.initialTopologyEdges();

    json settings = gf::task10p10SwarmSettings(scen, gf::SolverProfile::OpenSource);
    std::vector<int> base_indices;
    for (std::size_t i = 0; i < tri.beacon_ids.size(); ++i) {
        base_indices.push_back(static_cast<int>(i));
    }
    settings["formation"]["bases-id"] = {base_indices};
    settings["execute"]["time-total"] = 500.0;
    settings["execute"]["time-step"] = 0.1;
    settings["searching"]["front-sector"]["outer-radius"] = 400.0;
    settings["world"]["spacing"] = 10.0;
    settings["world"]["boundary"] = {
        {-2050.0, 120.0}, {4050.0, 120.0}, {2950.0, 2050.0}, {-950.0, 2050.0}
    };
    settings["world"]["cell-domain"] = "closed-polygon-centers-v1";

    gf::GrandFinaleSwarmAdapterConfig acfg;
    acfg.solver_profile = gf::SolverProfile::OpenSource;
    acfg.dt_s = 0.1;
    acfg.minimum_dwell_s = 0.1;
    acfg.acceleration_half_box = 4.0;
    acfg.collision_distance_m = 10.0;
    acfg.reference_distance_m = 850.0;
    acfg.add_reference_distance_m = 849.0;
    acfg.uncertainty_sigma = 3.0;
    acfg.uncertainty_scale_mode = gf::UncertaintyScaleMode::TraceRoot;
    acfg.axis_velocity_limit_mps = 25.0;
    acfg.axis_velocity_cbf_gain = 5.0;
    acfg.minimal_runtime_gating = true;
    acfg.cbf2026_uwb_channel.enabled = true;
    acfg.cbf2026_uwb_channel.d_r_m = 850.0;
    acfg.cbf2026_uwb_channel.sigma0_m = 0.5;
    acfg.cbf2026_uwb_channel.d_max_m = 1200.0;
    acfg.cbf2026_uwb_channel.link_seed = 20261123;
    acfg.sensor_radius_m = 400.0;
    acfg.coverage_footprint_kind = gf::CoverageFootprintKind::ForwardSector;
    acfg.coverage_half_angle_rad = M_PI / 3.0;
    acfg.maximum_yaw_rate_radps = 1.8;

    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
    REQUIRE(adapter.initializeStageZero().initialized);
    REQUIRE(adapter.coverage().truthGrid().validCount == 96500);

    gf::route_b_mission_detail::FrontierSweepPolicy policy(
        tri, 2600.0, 0.8, 1.8, 4.0, 1.8, 400.0, true, 5.0, 850.0);

    const auto t0 = std::chrono::steady_clock::now();
    constexpr int kTicks = 100;
    for (int tick = 0; tick < kTicks; ++tick) {
        const auto snap = adapter.runtimeSnapshot();
        std::map<gf::NodeId, Eigen::Vector2d> nom;
        std::map<gf::NodeId, double> yaw;
        policy.computeCommands(swarm, adapter, snap.estimate, static_cast<std::size_t>(tick), nom, yaw);
        if (tick % 10 == 0) {
            const int expected_uncovered =
                adapter.coverage().certifiedGrid().validCount - adapter.coverage().certifiedCoveredCount();
            CHECK(static_cast<int>(policy.cachedUnsearchedCount()) == expected_uncovered);
        }
        const auto step = adapter.stepWithNominalAndYawRates(nom, yaw);
        REQUIRE(step.advanced);
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "[Benchmark 10m 10-UAV full closed-loop 100 ticks] total_ms=" << elapsed_ms
              << " ms_per_tick=" << (elapsed_ms / kTicks)
              << " rtf_vs_100ms=" << (100.0 / (elapsed_ms / kTicks))
              << " cached_unsearched=" << policy.cachedUnsearchedCount() << std::endl;
    CHECK(adapter.coverage().certifiedCoveredCount() > 0);

    // Side-by-side map-subsystem benchmark (10 UAVs moving across 96,500 cells for 200 ticks, refresh every 10 ticks)
    GridWorld base_domain(settings["world"]);
    GridWorld ref_truth = base_domain;
    GridWorld ref_cert = base_domain;
    gf::CertifiedCoverageTracker opt_tracker(base_domain);
    const double cell_w = (base_domain.xLim.second - base_domain.xLim.first) / base_domain.xNum;
    const double cell_h = (base_domain.yLim.second - base_domain.yLim.first) / base_domain.yNum;
    const double half_diag = 0.5 * std::hypot(cell_w, cell_h);

    constexpr int kMapTicks = 200;
    std::size_t ref_uncov_sink = 0;
    const auto rt0 = std::chrono::steady_clock::now();
    for (int tick = 0; tick < kMapTicks; ++tick) {
        for (int u = 0; u < 10; ++u) {
            const double px = -1400.0 + 480.0 * u + 12.0 * tick;
            const double py = 320.0 + 140.0 * (u % 5) + 4.0 * tick;
            referenceObserveSectorFull(
                ref_truth, ref_cert, half_diag,
                Point(px, py), Point(px + 1.2, py - 0.8), 3.5,
                0.0, 400.0, M_PI / 3.0, -M_PI + 0.05 * tick + 0.3 * u);
        }
        if (tick % 10 == 0) {
            ref_uncov_sink += referenceScanUncovered(ref_cert).size();
        }
    }
    const double ref_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - rt0).count();

    std::vector<int> opt_idx;
    std::vector<Eigen::Vector2d> opt_pts;
    bool opt_init = false;
    std::size_t opt_uncov_sink = 0;
    const auto ot0 = std::chrono::steady_clock::now();
    for (int tick = 0; tick < kMapTicks; ++tick) {
        for (int u = 0; u < 10; ++u) {
            const double px = -1400.0 + 480.0 * u + 12.0 * tick;
            const double py = 320.0 + 140.0 * (u % 5) + 4.0 * tick;
            opt_tracker.observeSector(
                Point(px, py), Point(px + 1.2, py - 0.8), 3.5,
                0.0, 400.0, M_PI / 3.0, -M_PI + 0.05 * tick + 0.3 * u);
        }
        if (tick % 10 == 0) {
            const GridWorld& cg = opt_tracker.certifiedGrid();
            if (!opt_init) {
                const double x0 = cg.xLim.first;
                const double x1 = cg.xLim.second;
                const double y0 = cg.yLim.first;
                const double y1 = cg.yLim.second;
                for (int ix = 0; ix < cg.xNum; ++ix) {
                    const double rx = (static_cast<double>(ix) + 0.5) / static_cast<double>(cg.xNum);
                    const double cx = x0 * (1.0 - rx) + x1 * rx;
                    for (int iy = 0; iy < cg.yNum; ++iy) {
                        const int idx = cg.getIndex(ix, iy);
                        if (cg.valid[idx] && !cg.vis[idx]) {
                            const double ry = (static_cast<double>(iy) + 0.5) / static_cast<double>(cg.yNum);
                            opt_idx.push_back(idx);
                            opt_pts.emplace_back(cx, y0 * (1.0 - ry) + y1 * ry);
                        }
                    }
                }
                opt_init = true;
            } else {
                std::size_t w = 0;
                for (std::size_t k = 0; k < opt_idx.size(); ++k) {
                    if (!cg.vis[static_cast<std::size_t>(opt_idx[k])]) {
                        opt_idx[w] = opt_idx[k];
                        opt_pts[w] = opt_pts[k];
                        ++w;
                    }
                }
                opt_idx.resize(w);
                opt_pts.resize(w);
            }
            opt_uncov_sink += opt_idx.size();
        }
    }
    const double opt_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ot0).count();

    CHECK(ref_uncov_sink == opt_uncov_sink);
    CHECK(opt_tracker.truthGrid().vis == ref_truth.vis);
    CHECK(opt_tracker.certifiedGrid().vis == ref_cert.vis);
    std::cout << "[Benchmark 10m map+frontier subsystem] ref_ms_per_tick=" << (ref_ms / kMapTicks)
              << " opt_ms_per_tick=" << (opt_ms / kMapTicks)
              << " speedup=" << (ref_ms / opt_ms) << "x" << std::endl;
    CHECK(opt_ms < ref_ms);
}

TEST_CASE("TDD-11b Post-optimization oblique_asymmetric mission matches pre-optimization metrics identically") {
    gf::RouteBMissionConfig mcfg;
    mcfg.domain_name = "oblique_asymmetric";
    mcfg.t_fork_s = 12.0;
    mcfg.max_mission_s = 650.0;
    mcfg.patrol_radius_m = 105.0;
    mcfg.v_cruise_mps = 14.0;
    mcfg.uwb_d_r_m = 850.0;
    mcfg.uwb_sigma0_m = 0.5;
    mcfg.uwb_d_max_m = 1200.0;
    mcfg.link_seed = 20261123;
    mcfg.curve_sample_interval_s = 5.0;

    const auto res = gf::runTwoRegimeMissionComparison(mcfg);
    std::cout << "[TDD-6 oblique_asymmetric nominal post-opt] stay_wall=" << res.stay.wall_clock_s
              << " sw_wall=" << res.switch_arm.wall_clock_s
              << " stay_cov=" << res.stay.final_certified_coverage
              << " sw_cov=" << res.switch_arm.final_certified_coverage << std::endl;

    CHECK(res.checkpoint_byte_identical);
    REQUIRE(res.stay.t95_certified_s.has_value());
    REQUIRE(res.switch_arm.t95_certified_s.has_value());
    CHECK(*res.stay.t95_certified_s == doctest::Approx(18.7));
    CHECK(res.stay.final_certified_coverage == doctest::Approx(0.9596947293107561));
    CHECK(res.switch_arm.reconfig_complete_s == doctest::Approx(585.3));
    CHECK(*res.switch_arm.t95_certified_s == doctest::Approx(467.6));
    CHECK(res.switch_arm.final_certified_coverage == doctest::Approx(0.9729310756021942));
}
