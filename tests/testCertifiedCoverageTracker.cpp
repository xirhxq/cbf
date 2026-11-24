#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedCoverageTracker.hpp"
#include "grand_finale/SearchPolygonDomain.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

// Brute-force full-grid reference matching original pre-optimization math
void referenceObserve(
    GridWorld& truth,
    GridWorld& cert,
    double half_diag,
    const Point& truth_pos,
    const Point& est_pos,
    double err_bound_m,
    double sensor_radius_m) {
    for (int x = 0; x < truth.xNum; ++x) {
        for (int y = 0; y < truth.yNum; ++y) {
            const Point cell(truth.getCellCenterX(x), truth.getCellCenterY(y));
            if (std::hypot(cell.x - truth_pos.x, cell.y - truth_pos.y) <= sensor_radius_m) {
                truth.setValue(x, y, true);
            }
            if (std::hypot(cell.x - est_pos.x, cell.y - est_pos.y) + err_bound_m + half_diag <= sensor_radius_m) {
                cert.setValue(x, y, true);
            }
        }
    }
}

void referenceObserveSector(
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

}  // namespace

TEST_CASE("Certified coverage is tightened by error bound and cell half diagonal") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    const Point estimate(1.0, 1.0);

    tracker.observe(estimate, estimate, 0.1, 1.2);
    CHECK(tracker.truthCoveredCount() == 1);
    CHECK(tracker.certifiedCoveredCount() == 0);

    tracker.observe(estimate, estimate, 0.1, 1.6);
    CHECK(tracker.certifiedCoveredCount() == 1);
    CHECK(tracker.certifiedCoveredCount() <= tracker.truthCoveredCount());
}

TEST_CASE("A finite rectangular GridWorld reaches certified T100") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    tracker.observe(Point(1.0, 1.0), Point(1.0, 1.0), 0.1, 1.6);
    CHECK_FALSE(tracker.reachedCertifiedT100());
    tracker.observe(Point(3.0, 1.0), Point(3.0, 1.0), 0.1, 1.6);
    CHECK(tracker.reachedCertifiedT100());
    CHECK(tracker.certifiedFraction() == doctest::Approx(1.0));
}

TEST_CASE("Truth and certified maps remain distinct under estimator displacement") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    tracker.observe(Point(1.0, 1.0), Point(3.0, 1.0), 0.1, 1.6);

    CHECK(tracker.truthCovered(0, 0));
    CHECK_FALSE(tracker.certifiedCovered(0, 0));
    CHECK(tracker.certifiedCovered(1, 0));
    CHECK(tracker.falseCertifiedCount() == 1);
    CHECK_FALSE(tracker.certifiedSubsetOfTruth());
}

TEST_CASE("A containing uncertainty radius is conservative and prevents false coverage") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    tracker.observe(Point(1.0, 1.0), Point(1.6, 1.0), 0.7, 1.6);

    CHECK(tracker.truthCoveredCount() == 1);
    CHECK(tracker.certifiedCoveredCount() == 0);
    CHECK(tracker.falseCertifiedCount() == 0);
    CHECK(tracker.certifiedSubsetOfTruth());
}

TEST_CASE("Forward-sector coverage uses frozen heading and angular tightening") {
    gf::CertifiedCoverageTracker east({-4.0,4.0},4,{-1.0,1.0},1);
    east.observeSector(
        Point(0.0,0.0),Point(0.0,0.0),0.0,
        0.0,5.0,M_PI/3.0,0.0);
    CHECK_FALSE(east.truthCovered(0,0));
    CHECK_FALSE(east.truthCovered(1,0));
    CHECK(east.truthCovered(2,0));
    CHECK(east.truthCovered(3,0));
    CHECK_FALSE(east.certifiedCovered(2,0));
    CHECK(east.certifiedCovered(3,0));

    gf::CertifiedCoverageTracker west({-4.0,4.0},4,{-1.0,1.0},1);
    west.observeSector(
        Point(0.0,0.0),Point(0.0,0.0),0.0,
        0.0,5.0,M_PI/3.0,M_PI);
    CHECK(west.truthCovered(0,0));
    CHECK(west.truthCovered(1,0));
    CHECK_FALSE(west.truthCovered(2,0));
    CHECK_FALSE(west.truthCovered(3,0));
}

TEST_CASE("Search-external footprint contributes only intersecting denominator cells") {
    gf::CertifiedCoverageTracker tracker({0.0,4.0},2,{0.0,2.0},1);
    tracker.observeSector(
        Point(-1.0,1.0),Point(-1.0,1.0),0.0,
        0.0,3.0,M_PI/3.0,0.0);
    CHECK(tracker.truthCovered(0,0));
    CHECK_FALSE(tracker.truthCovered(1,0));
    CHECK(tracker.truthCoveredCount()==1);
}

TEST_CASE("Exact equivalence with full-grid reference across convex polygon, +-pi wrap, inner radius, and restart") {
    json world_cfg;
    world_cfg["spacing"] = 10.0;
    world_cfg["boundary"] = {
        {-200.0, 100.0}, {1200.0, 120.0}, {1000.0, 900.0}, {0.0, 820.0}
    };
    world_cfg["cell-domain"] = "closed-polygon-centers-v1";
    GridWorld domain(world_cfg);

    gf::CertifiedCoverageTracker tracker(domain);
    GridWorld ref_truth = domain;
    GridWorld ref_cert = domain;
    const double cell_w = (domain.xLim.second - domain.xLim.first) / domain.xNum;
    const double cell_h = (domain.yLim.second - domain.yLim.first) / domain.yNum;
    const double half_diag = 0.5 * std::hypot(cell_w, cell_h);

    struct StepCase {
        bool sector;
        Point tpos;
        Point epos;
        double err;
        double inner_r;
        double outer_r;
        double half_ang;
        double heading;
    };
    const std::vector<StepCase> steps = {
        {true, Point(100.0, 350.0), Point(104.0, 347.0), 8.5, 45.0, 400.0, M_PI / 3.0, M_PI - 0.08},
        {true, Point(250.0, 420.0), Point(248.0, 422.0), 5.0, 60.0, 400.0, M_PI / 3.0, -M_PI + 0.05},
        {true, Point(620.0, 500.0), Point(615.0, 505.0), 12.0, 0.0, 400.0, M_PI / 3.0, -0.5 * M_PI},
        {true, Point(950.0, 680.0), Point(952.0, 678.0), 3.0, 30.0, 400.0, M_PI, 0.25},
        {false, Point(500.0, 220.0), Point(502.0, 219.0), 4.2, 0.0, 260.0, 0.0, 0.0},
    };

    std::optional<gf::CertifiedCoverageRestartState> saved_restart;
    std::vector<bool> saved_ref_truth;
    std::vector<bool> saved_ref_cert;

    for (std::size_t si = 0; si < steps.size(); ++si) {
        const auto& sc = steps[si];
        if (sc.sector) {
            tracker.observeSector(sc.tpos, sc.epos, sc.err, sc.inner_r, sc.outer_r, sc.half_ang, sc.heading);
            referenceObserveSector(ref_truth, ref_cert, half_diag, sc.tpos, sc.epos, sc.err, sc.inner_r, sc.outer_r, sc.half_ang, sc.heading);
        } else {
            tracker.observe(sc.tpos, sc.epos, sc.err, sc.outer_r);
            referenceObserve(ref_truth, ref_cert, half_diag, sc.tpos, sc.epos, sc.err, sc.outer_r);
        }

        int ref_t_cnt = 0;
        int ref_c_cnt = 0;
        for (std::size_t idx = 0; idx < ref_truth.vis.size(); ++idx) {
            CHECK(tracker.truthGrid().vis[idx] == ref_truth.vis[idx]);
            CHECK(tracker.certifiedGrid().vis[idx] == ref_cert.vis[idx]);
            if (ref_truth.valid[idx] && ref_truth.vis[idx]) ++ref_t_cnt;
            if (ref_cert.valid[idx] && ref_cert.vis[idx]) ++ref_c_cnt;
        }
        CHECK(tracker.truthCoveredCount() == ref_t_cnt);
        CHECK(tracker.certifiedCoveredCount() == ref_c_cnt);

        if (si == 1) {
            saved_restart = tracker.restartState();
            saved_ref_truth = ref_truth.vis;
            saved_ref_cert = ref_cert.vis;
        }
    }

    // Check restoreRestartState updates O(1) counters cleanly
    tracker.restoreRestartState(*saved_restart);
    int restored_t_cnt = 0;
    int restored_c_cnt = 0;
    for (std::size_t idx = 0; idx < saved_ref_truth.size(); ++idx) {
        CHECK(tracker.truthGrid().vis[idx] == saved_ref_truth[idx]);
        CHECK(tracker.certifiedGrid().vis[idx] == saved_ref_cert[idx]);
        if (domain.valid[idx] && saved_ref_truth[idx]) ++restored_t_cnt;
        if (domain.valid[idx] && saved_ref_cert[idx]) ++restored_c_cnt;
    }
    CHECK(tracker.truthCoveredCount() == restored_t_cnt);
    CHECK(tracker.certifiedCoveredCount() == restored_c_cnt);
}

TEST_CASE("Benchmark 10m broad trapezoid (96,500 valid cells) observeSector throughput") {
    json world_cfg;
    world_cfg["spacing"] = 10.0;
    world_cfg["boundary"] = {
        {-2050.0, 120.0}, {4050.0, 120.0}, {2950.0, 2050.0}, {-950.0, 2050.0}
    };
    world_cfg["cell-domain"] = "closed-polygon-centers-v1";
    GridWorld domain(world_cfg);
    REQUIRE(domain.validCount == 96500);

    gf::CertifiedCoverageTracker tracker(domain);
    const auto t0 = std::chrono::steady_clock::now();
    constexpr int kTicks = 200;
    for (int tick = 0; tick < kTicks; ++tick) {
        const double ang = -M_PI + 0.03 * tick;
        for (int u = 0; u < 10; ++u) {
            const double px = -1200.0 + 480.0 * u;
            const double py = 400.0 + 130.0 * (u % 5);
            tracker.observeSector(
                Point(px, py), Point(px + 1.5, py - 1.0), 4.0,
                0.0, 400.0, M_PI / 3.0, ang + 0.4 * u);
        }
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "[Benchmark 10m observeSector 10 UAVs] total_ms=" << elapsed_ms
              << " ms_per_tick=" << (elapsed_ms / kTicks)
              << " cert_count=" << tracker.certifiedCoveredCount() << std::endl;
    CHECK(tracker.certifiedCoveredCount() > 0);
}
