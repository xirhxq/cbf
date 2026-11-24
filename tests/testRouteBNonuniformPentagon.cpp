#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBMissionRunner.hpp"
#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"

#include <iostream>

TEST_CASE("TDD-10 Non-uniform coastal beacon spacings (320, 400, 360) m reversible multi-unit transfer and convex pentagon coverage") {
    const std::vector<double> b_xs = {0.0, 320.0, 720.0, 1080.0};
    const auto tri_dual = gf::buildStaggeredTriangulation(
        b_xs, {{1, 2, 3}, {}, {4, 5, 6}}, 360.0, 68.0, 100.0, 140.0);
    const auto tri_center = gf::buildStaggeredTriangulation(
        b_xs, {{}, {6, 5, 4, 3, 2, 1}, {}}, 360.0, 68.0, 100.0, 140.0);

    const auto fwd_moves = gf::planLabelledTransfer(tri_dual, tri_center);
    const auto rev_moves = gf::planLabelledTransfer(tri_center, tri_dual);

    gf::RouteBPipelinedConfig pcfg;
    pcfg.serial_mode = false;
    pcfg.v_cruise_mps = 14.0;
    pcfg.accel_mps2 = 2.0;
    pcfg.d_pipe_m = 50.0;
    pcfg.d_clear_m = 32.0;
    pcfg.t_settle_s = 1.0;
    pcfg.uwb_d_r_m = 600.0;
    pcfg.uwb_sigma0_m = 1.0;
    pcfg.uwb_d_max_m = 1200.0;
    pcfg.link_seed = 1001;
    pcfg.record_trajectory_samples = true;

    const auto fwd = gf::runClosedLoopReconfiguration(tri_dual, tri_center, fwd_moves, pcfg);
    const auto rev = gf::runClosedLoopReconfiguration(tri_center, tri_dual, rev_moves, pcfg);

    CHECK(fwd.completed);
    CHECK(rev.completed);
    CHECK(fwd.static_parent_violations == 0);
    CHECK(rev.static_parent_violations == 0);
    CHECK(fwd.dag_violations == 0);
    CHECK(rev.dag_violations == 0);
    CHECK(fwd.audit.collision_violations == 0);
    CHECK(rev.audit.collision_violations == 0);
    CHECK(fwd.audit.reference_violations == 0);
    CHECK(rev.audit.reference_violations == 0);
    CHECK(fwd.audit.axis_speed_violations == 0);
    CHECK(rev.audit.axis_speed_violations == 0);
    CHECK_FALSE(fwd.trajectory_samples.empty());

    gf::RouteBConvexDomainSpec pent;
    pent.name = "nonuniform_oblique_pentagon";
    pent.description = "Irregular oblique asymmetric convex pentagon over non-uniform coastal beacons (320, 400, 360) m";
    pent.vertices = {
        {60.0, 220.0}, {880.0, 250.0}, {1140.0, 640.0}, {680.0, 980.0}, {180.0, 740.0}
    };
    pent.initial_tri = tri_dual;
    pent.target_tri = tri_center;
    pent.use_direct_matched = false;

    gf::RouteBMissionConfig mcfg;
    mcfg.domain_name = pent.name;
    mcfg.custom_domain = pent;
    mcfg.cell_size_m = 10.0;
    mcfg.sensor_radius_m = 400.0;
    mcfg.t_fork_s = 12.0;
    mcfg.max_mission_s = 900.0;
    mcfg.patrol_radius_m = 105.0;
    mcfg.d_pipe_m = 50.0;
    mcfg.uwb_d_r_m = 600.0;
    mcfg.uwb_sigma0_m = 1.0;
    mcfg.uwb_d_max_m = 1200.0;
    mcfg.link_seed = 1001;
    mcfg.curve_sample_interval_s = 5.0;

    const auto mission = gf::runTwoRegimeMissionComparison(mcfg);
    CHECK(mission.checkpoint_byte_identical);
    CHECK(mission.switch_arm.reconfig_completed);
    CHECK(mission.switch_arm.reached_t100);
    CHECK(mission.switch_arm.false_certified_count == 0);
    CHECK(mission.switch_arm.certified_subset_of_truth);
    CHECK(mission.switch_arm.audit.collision_violations == 0);
    CHECK(mission.switch_arm.audit.reference_violations == 0);
    CHECK(mission.switch_arm.audit.axis_speed_violations == 0);
}
