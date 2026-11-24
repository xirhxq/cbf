#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBMissionRunner.hpp"

#include <cmath>
#include <set>
#include <vector>

namespace {

std::vector<bool> decodeRle(const std::vector<std::pair<int, int>>& rle, std::size_t total_size) {
    std::vector<bool> mask(total_size, false);
    for (const auto& [start, len] : rle) {
        REQUIRE(start >= 0);
        REQUIRE(len > 0);
        REQUIRE(static_cast<std::size_t>(start + len) <= total_size);
        for (int k = 0; k < len; ++k) {
            mask[static_cast<std::size_t>(start + k)] = true;
        }
    }
    return mask;
}

}  // namespace

TEST_CASE("TDD-11 Read-only mission snapshots preserve bit-identical results and encode valid active edges and RLE masks") {
    gf::RouteBMissionConfig base_cfg;
    base_cfg.domain_name = "wide_shallow_coastal";
    base_cfg.t_fork_s = 2.0;
    base_cfg.max_mission_s = 700.0;
    base_cfg.patrol_radius_m = 105.0;
    base_cfg.v_cruise_mps = 14.0;
    base_cfg.uwb_d_r_m = 850.0;
    base_cfg.uwb_sigma0_m = 0.5;
    base_cfg.uwb_d_max_m = 1200.0;
    base_cfg.link_seed = 20261123;
    base_cfg.curve_sample_interval_s = 5.0;

    auto snap_cfg = base_cfg;
    snap_cfg.record_mission_snapshots = true;
    snap_cfg.snapshot_times_s = {2.0, 120.0, 320.0, 520.0};
    snap_cfg.skip_stay_arm = true;

    const auto base_res = gf::runTwoRegimeMissionComparison(base_cfg);
    const auto snap_res = gf::runTwoRegimeMissionComparison(snap_cfg);

    // 1. Read-only invariance: switch_arm must be identical with snapshots & skip_stay_arm
    CHECK(snap_res.valid_cells == base_res.valid_cells);
    CHECK(snap_res.area_m2 == doctest::Approx(base_res.area_m2));
    CHECK(snap_res.switch_arm.reconfig_completed == base_res.switch_arm.reconfig_completed);
    CHECK(snap_res.switch_arm.status_reason == base_res.switch_arm.status_reason);
    CHECK(snap_res.switch_arm.reconfig_complete_s == doctest::Approx(base_res.switch_arm.reconfig_complete_s));
    CHECK(snap_res.switch_arm.final_time_s == doctest::Approx(base_res.switch_arm.final_time_s));
    CHECK(snap_res.switch_arm.reached_t95 == base_res.switch_arm.reached_t95);
    CHECK(snap_res.switch_arm.reached_t100 == base_res.switch_arm.reached_t100);
    REQUIRE(base_res.switch_arm.t100_certified_s.has_value());
    REQUIRE(snap_res.switch_arm.t100_certified_s.has_value());
    CHECK(*snap_res.switch_arm.t100_certified_s == doctest::Approx(*base_res.switch_arm.t100_certified_s));
    CHECK(snap_res.switch_arm.final_truth_coverage == doctest::Approx(base_res.switch_arm.final_truth_coverage));
    CHECK(snap_res.switch_arm.final_certified_coverage == doctest::Approx(base_res.switch_arm.final_certified_coverage));
    CHECK(snap_res.switch_arm.false_certified_count == base_res.switch_arm.false_certified_count);
    CHECK(snap_res.switch_arm.total_mbb_swaps == base_res.switch_arm.total_mbb_swaps);
    CHECK(snap_res.switch_arm.audit.position_rmse_m == doctest::Approx(base_res.switch_arm.audit.position_rmse_m));
    CHECK(snap_res.switch_arm.audit.peak_position_scale_radius_m ==
          doctest::Approx(base_res.switch_arm.audit.peak_position_scale_radius_m));
    REQUIRE(snap_res.switch_arm.coverage_curve.size() == base_res.switch_arm.coverage_curve.size());
    for (std::size_t i = 0; i < base_res.switch_arm.coverage_curve.size(); ++i) {
        CHECK(snap_res.switch_arm.coverage_curve[i].time_s ==
              doctest::Approx(base_res.switch_arm.coverage_curve[i].time_s));
        CHECK(snap_res.switch_arm.coverage_curve[i].certified_coverage ==
              doctest::Approx(base_res.switch_arm.coverage_curve[i].certified_coverage));
    }

    // 2. Grid metadata & RLE validation
    const auto& gm = snap_res.grid_metadata;
    CHECK(gm.valid_cells == snap_res.valid_cells);
    CHECK(gm.x_num > 0);
    CHECK(gm.y_num > 0);
    CHECK(gm.bbox_ix_min <= gm.bbox_ix_max);
    CHECK(gm.bbox_iy_min <= gm.bbox_iy_max);
    const std::size_t total_grid = static_cast<std::size_t>(gm.x_num) * static_cast<std::size_t>(gm.y_num);
    const auto valid_mask = decodeRle(gm.valid_rle, total_grid);
    int decoded_valid = 0;
    for (bool b : valid_mask) {
        if (b) ++decoded_valid;
    }
    CHECK(decoded_valid == gm.valid_cells);

    // 3. Mission snapshots completeness, monotonicity, active_edges, and certified_rle subset check
    const auto& snaps = snap_res.switch_arm.mission_snapshots;
    REQUIRE(snaps.size() >= 5);

    auto has_near_time = [&](double target_t) {
        for (const auto& s : snaps) {
            if (std::abs(s.time_s - target_t) <= 0.06) return true;
        }
        return false;
    };
    CHECK(has_near_time(2.0));
    CHECK(has_near_time(120.0));
    CHECK(has_near_time(320.0));
    CHECK(has_near_time(520.0));
    CHECK(has_near_time(snap_res.switch_arm.reconfig_complete_s));
    CHECK(has_near_time(*snap_res.switch_arm.t100_certified_s));

    for (std::size_t i = 0; i < snaps.size(); ++i) {
        const auto& s = snaps[i];
        if (i > 0) {
            CHECK(s.time_s > snaps[i - 1].time_s);
        }
        CHECK(s.positions.size() == 12);
        CHECK(s.beacons.size() == 5);
        CHECK(s.active_edges.size() >= 24);
        for (const auto& [p, c] : s.active_edges) {
            const bool p_valid = (s.positions.count(p) > 0) || (s.beacons.count(p) > 0);
            const bool c_valid = (s.positions.count(c) > 0);
            CHECK(p_valid);
            CHECK(c_valid);
            CHECK(p != c);
        }
        const auto cert_mask = decodeRle(s.certified_rle, total_grid);
        int cert_cnt = 0;
        for (std::size_t idx = 0; idx < total_grid; ++idx) {
            if (cert_mask[idx]) {
                CHECK(valid_mask[idx]);
                ++cert_cnt;
            }
        }
        CHECK(cert_cnt == s.certified_count);
        CHECK(static_cast<double>(cert_cnt) / static_cast<double>(gm.valid_cells) ==
              doctest::Approx(s.certified_coverage));
    }
    CHECK(snaps.back().certified_coverage == doctest::Approx(1.0));
}
