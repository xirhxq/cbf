#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"

#include <iostream>

TEST_CASE("TDD-9a Coastal berth height 40 m vs 100 m degrades closed-loop scale radius and RMSE under degraded UWB") {
    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
    const auto src_40 = gf::buildStaggeredTriangulation(
        b4, {{1, 2}, {}, {}}, 300.0, 80.0, 40.0, 140.0);
    const auto dst_40 = gf::buildStaggeredTriangulation(
        b4, {{}, {}, {2, 1}}, 300.0, 80.0, 40.0, 140.0);
    const auto moves_40 = gf::planLabelledTransfer(src_40, dst_40);

    const auto src_100 = gf::buildStaggeredTriangulation(
        b4, {{1, 2}, {}, {}}, 300.0, 80.0, 100.0, 140.0);
    const auto dst_100 = gf::buildStaggeredTriangulation(
        b4, {{}, {}, {2, 1}}, 300.0, 80.0, 100.0, 140.0);
    const auto moves_100 = gf::planLabelledTransfer(src_100, dst_100);

    gf::RouteBPipelinedConfig cfg;
    cfg.serial_mode = false;
    cfg.v_cruise_mps = 14.0;
    cfg.accel_mps2 = 2.0;
    cfg.d_pipe_m = 60.0;
    cfg.d_clear_m = 32.0;
    cfg.t_settle_s = 12.0;
    cfg.uwb_d_r_m = 600.0;
    cfg.uwb_sigma0_m = 1.0;
    cfg.uwb_d_max_m = 1200.0;
    cfg.link_seed = 1001;
    cfg.range_measurement_scope = gf::RangeMeasurementScope::ActiveTopologyAndAuxiliary;
    cfg.enable_auxiliary_handover_ranges = true;
    cfg.record_scale_radius_series = true;

    const auto res_40 = gf::runClosedLoopReconfiguration(src_40, dst_40, moves_40, cfg);
    const auto res_100 = gf::runClosedLoopReconfiguration(src_100, dst_100, moves_100, cfg);

    std::cout << "[TDD-9a] hb=40 ok=" << res_40.completed
              << " peak_scale=" << res_40.audit.peak_position_scale_radius_m
              << " rmse=" << res_40.audit.position_rmse_m
              << " | hb=100 ok=" << res_100.completed
              << " peak_scale=" << res_100.audit.peak_position_scale_radius_m
              << " rmse=" << res_100.audit.position_rmse_m << std::endl;

    CHECK(res_100.completed);
    CHECK(res_100.audit.collision_violations == 0);
    CHECK(res_100.audit.reference_violations == 0);
    CHECK(res_100.audit.axis_speed_violations == 0);
    CHECK_FALSE(res_100.scale_radius_series.empty());
    CHECK_FALSE(res_40.scale_radius_series.empty());
    CHECK(res_100.audit.peak_position_scale_radius_m < res_40.audit.peak_position_scale_radius_m);
    CHECK(res_100.audit.position_rmse_m < res_40.audit.position_rmse_m);
}

TEST_CASE("TDD-9b Two-reference collinear edge crossing vs circumcenter three-reference dual handover") {
    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
    const auto p6 = gf::buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3 = gf::buildStaggeredTriangulation(
        b4, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto moves = gf::planLabelledTransfer(p6, dl3);

    gf::RouteBPipelinedConfig cfg_3ref;
    cfg_3ref.serial_mode = false;
    cfg_3ref.v_cruise_mps = 14.0;
    cfg_3ref.accel_mps2 = 2.0;
    cfg_3ref.d_pipe_m = 60.0;
    cfg_3ref.d_clear_m = 32.0;
    cfg_3ref.t_settle_s = 1.0;
    cfg_3ref.uwb_d_r_m = 600.0;
    cfg_3ref.uwb_sigma0_m = 1.0;
    cfg_3ref.uwb_d_max_m = 1200.0;
    cfg_3ref.link_seed = 1001;
    cfg_3ref.range_measurement_scope = gf::RangeMeasurementScope::ActiveTopologyAndAuxiliary;
    cfg_3ref.enable_auxiliary_handover_ranges = true;
    cfg_3ref.record_scale_radius_series = true;

    auto cfg_2ref = cfg_3ref;
    cfg_2ref.enable_auxiliary_handover_ranges = false;
    cfg_2ref.strict_two_ref_collinear_baseline = true;
    cfg_2ref.max_sim_time_s = 900.0;

    const auto res_3ref = gf::runClosedLoopReconfiguration(p6, dl3, moves, cfg_3ref);
    const auto res_2ref = gf::runClosedLoopReconfiguration(p6, dl3, moves, cfg_2ref);

    std::cout << "[TDD-9b] 2ref ok=" << res_2ref.completed << " reason=" << res_2ref.reason
              << " peak_scale=" << res_2ref.audit.peak_position_scale_radius_m
              << " peak_trP=" << res_2ref.audit.peak_position_covariance_trace_m2
              << " rmse=" << res_2ref.audit.position_rmse_m
              << " | 3ref ok=" << res_3ref.completed
              << " peak_scale=" << res_3ref.audit.peak_position_scale_radius_m
              << " peak_trP=" << res_3ref.audit.peak_position_covariance_trace_m2
              << " rmse=" << res_3ref.audit.position_rmse_m << std::endl;

    CHECK(res_3ref.completed);
    CHECK(res_3ref.static_parent_violations == 0);
    CHECK(res_3ref.dag_violations == 0);
    CHECK(res_3ref.audit.collision_violations == 0);
    CHECK(res_3ref.audit.reference_violations == 0);
    CHECK(res_3ref.audit.axis_speed_violations == 0);
    CHECK(res_3ref.audit.peak_position_scale_radius_m < res_2ref.audit.peak_position_scale_radius_m);
    CHECK(res_3ref.audit.peak_position_covariance_trace_m2 < res_2ref.audit.peak_position_covariance_trace_m2);
}
