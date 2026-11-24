#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBMissionRunner.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <optional>
#include <vector>

namespace {

gf::GrandFinaleSwarmAdapterConfig makeOpenSourceSingleUavConfig(
    double maximum_yaw_rate_radps) {
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
    acfg.maximum_yaw_rate_radps = maximum_yaw_rate_radps;
    return acfg;
}

}  // namespace

TEST_CASE("RouteBMissionConfig defaults max yaw rate to 1.0 rad/s and computeVelocityAlignedYawRate enforces deadband and bounds") {
    gf::RouteBMissionConfig cfg;
    CHECK(cfg.max_yaw_rate_radps == doctest::Approx(1.0));

    // Speed deadband (< 2.0 m/s -> 0.0)
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(0.0, 0.0), 0.5, 1.0) == doctest::Approx(0.0));
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(1.5, 1.0), -0.8, 1.0) == doctest::Approx(0.0));
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(1.99, 0.0), 1.2, 1.0) == doctest::Approx(0.0));

    // Angle deadband (< 0.05 rad -> 0.0)
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(10.0, 0.0), 0.04, 1.0) == doctest::Approx(0.0));
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(10.0, 0.0), -0.049, 1.0) == doctest::Approx(0.0));
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(0.0, 10.0), M_PI / 2.0 - 0.04, 1.0) == doctest::Approx(0.0));

    // Proportional region and +/-1.0 rad/s symmetric clamping
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(5.0, 0.0), -0.25, 1.0) == doctest::Approx(1.8 * 0.25));
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(5.0, 0.0), 0.30, 1.0) == doctest::Approx(-1.8 * 0.30));
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(0.0, 10.0), 0.0, 1.0) == doctest::Approx(1.0));
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(0.0, -10.0), 0.0, 1.0) == doctest::Approx(-1.0));

    // Shortest-arc wrap across +/-pi
    const double wrap_neg = gf::computeVelocityAlignedYawRate(
        Eigen::Vector2d(-10.0, -1.0), M_PI - 0.1, 1.0);
    CHECK(wrap_neg > 0.0);
    CHECK(wrap_neg < 1.0);
    const double wrap_pos = gf::computeVelocityAlignedYawRate(
        Eigen::Vector2d(-10.0, 1.0), -M_PI + 0.1, 1.0);
    CHECK(wrap_pos < 0.0);
    CHECK(wrap_pos > -1.0);

    // Invalid / zero limit guard
    CHECK(gf::computeVelocityAlignedYawRate(Eigen::Vector2d(5.0, 0.0), 0.2, 0.0) == doctest::Approx(0.0));
}

TEST_CASE("FrontierSweepPolicy enforces 30 m lookahead hysteresis and 5 m / 0.05 rad yaw deadbands without spinning") {
    const std::vector<double> b_xs = {0.0, 400.0};
    const auto tri = gf::buildStaggeredTriangulation(
        b_xs, {{1}}, 380.0, 60.0, 100.0, 140.0);

    // Subcase A: domain 50..450 x 150..550 (10m grid) has cells both <30m and >=30m from (200, 300)
    {
        gf::Task10p10Scenario scen;
        scen.id = "yaw_lookahead_hysteresis";
        scen.width_m = 600.0;
        scen.height_m = 700.0;
        scen.mobile_ids = tri.mobile_ids;
        scen.mobile_positions = {Eigen::Vector2d(200.0, 300.0)};
        scen.fixed_positions = tri.beacons;
        scen.initial_topology = tri.initialTopologyEdges();

        json settings = gf::task10p10SwarmSettings(scen, gf::SolverProfile::OpenSource);
        settings["formation"]["bases-id"] = {{0, 1}};
        settings["execute"]["time-total"] = 100.0;
        settings["execute"]["time-step"] = 0.1;
        settings["searching"]["front-sector"]["outer-radius"] = 400.0;
        settings["world"]["spacing"] = 10.0;
        settings["world"]["boundary"] = {
            {50.0, 150.0}, {450.0, 150.0}, {450.0, 550.0}, {50.0, 550.0}
        };
        settings["world"]["cell-domain"] = "closed-polygon-centers-v1";

        const auto acfg = makeOpenSourceSingleUavConfig(1.0);
        Swarm swarm(settings);
        gf::GrandFinaleSwarmAdapter adapter(
            swarm, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
        REQUIRE(adapter.initializeStageZero().initialized);

        gf::route_b_mission_detail::FrontierSweepPolicy policy(
            tri, 105.0, 0.8, 1.8, 4.0, 1.0, 400.0, false, 0.0, 850.0);
        CHECK(policy.searchAimDistanceDeadbandM() == doctest::Approx(5.0));

        const auto snap0 = adapter.runtimeSnapshot();
        std::map<gf::NodeId, Eigen::Vector2d> nom;
        std::map<gf::NodeId, double> yaw;
        policy.computeCommands(swarm, adapter, snap0.estimate, 0, nom, yaw);

        const auto aim0 = policy.aimPoint(1);
        REQUIRE(aim0.has_value());
        CHECK((*aim0 - Eigen::Vector2d(200.0, 300.0)).norm() >= 30.0 - 1.0e-9);
        CHECK(std::abs(yaw.at(1)) <= 1.0 + 1.0e-12);

        // Next updateTargets tick (tick=10) with UAV shifted to (235, 300) while aim0 remains uncertified:
        // hysteresis must retain the exact same aim0 center instead of jumping to a closer cell.
        auto est_shifted = snap0.estimate;
        est_shifted.mean.segment<2>(0) = Eigen::Vector2d(235.0, 300.0);
        policy.computeCommands(swarm, adapter, est_shifted, 10, nom, yaw);
        const auto aim1 = policy.aimPoint(1);
        REQUIRE(aim1.has_value());
        CHECK((*aim1 - *aim0).norm() == doctest::Approx(0.0));
    }

    // Subcase B: tiny 1-cell domain directly centered near UAV (< 5.0 m -> yaw rate strictly 0.0)
    {
        gf::Task10p10Scenario scen;
        scen.id = "yaw_dist_deadband";
        scen.width_m = 600.0;
        scen.height_m = 500.0;
        scen.mobile_ids = tri.mobile_ids;
        scen.mobile_positions = {Eigen::Vector2d(202.0, 302.0)};
        scen.fixed_positions = tri.beacons;
        scen.initial_topology = tri.initialTopologyEdges();

        json settings = gf::task10p10SwarmSettings(scen, gf::SolverProfile::OpenSource);
        settings["formation"]["bases-id"] = {{0, 1}};
        settings["execute"]["time-total"] = 100.0;
        settings["execute"]["time-step"] = 0.1;
        settings["searching"]["front-sector"]["outer-radius"] = 400.0;
        settings["world"]["spacing"] = 10.0;
        settings["world"]["boundary"] = {
            {195.0, 295.0}, {205.0, 295.0}, {205.0, 305.0}, {195.0, 305.0}
        };
        settings["world"]["cell-domain"] = "closed-polygon-centers-v1";

        const auto acfg = makeOpenSourceSingleUavConfig(1.0);
        Swarm swarm(settings);
        gf::GrandFinaleSwarmAdapter adapter(
            swarm, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
        REQUIRE(adapter.initializeStageZero().initialized);
        REQUIRE(adapter.coverage().truthGrid().validCount == 1);

        // Turn UAV orthogonal to cell so error would spin if dist deadband weren't enforced
        swarm.robots[0]->model->setStateVariable("yawRad", -M_PI / 2.0);
        gf::route_b_mission_detail::FrontierSweepPolicy policy(
            tri, 105.0, 0.8, 1.8, 4.0, 1.0, 400.0, false, 0.0, 850.0);

        const auto snap = adapter.runtimeSnapshot();
        std::map<gf::NodeId, Eigen::Vector2d> nom;
        std::map<gf::NodeId, double> yaw;
        policy.computeCommands(swarm, adapter, snap.estimate, 0, nom, yaw);
        const auto aim = policy.aimPoint(1);
        REQUIRE(aim.has_value());
        CHECK((*aim - Eigen::Vector2d(202.0, 302.0)).norm() < 5.0);
        CHECK(yaw.at(1) == doctest::Approx(0.0));
    }

    // Subcase C: all uncovered cells < 30 m but >= 5 m -> fallback to nearest cell & angle deadband 0.05 rad
    {
        gf::Task10p10Scenario scen;
        scen.id = "yaw_near_fallback_and_angle_deadband";
        scen.width_m = 600.0;
        scen.height_m = 500.0;
        scen.mobile_ids = tri.mobile_ids;
        scen.mobile_positions = {Eigen::Vector2d(200.0, 300.0)};
        scen.fixed_positions = tri.beacons;
        scen.initial_topology = tri.initialTopologyEdges();

        json settings = gf::task10p10SwarmSettings(scen, gf::SolverProfile::OpenSource);
        settings["formation"]["bases-id"] = {{0, 1}};
        settings["execute"]["time-total"] = 100.0;
        settings["execute"]["time-step"] = 0.1;
        settings["searching"]["front-sector"]["outer-radius"] = 400.0;
        settings["world"]["spacing"] = 10.0;
        // Put cell behind/south at (200, 280) so initial yaw does not cover it during initializeStageZero()
        settings["world"]["boundary"] = {
            {195.0, 275.0}, {205.0, 275.0}, {205.0, 285.0}, {195.0, 285.0}
        };
        settings["world"]["cell-domain"] = "closed-polygon-centers-v1";

        const auto acfg = makeOpenSourceSingleUavConfig(1.0);
        Swarm swarm(settings);
        gf::GrandFinaleSwarmAdapter adapter(
            swarm, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
        REQUIRE(adapter.initializeStageZero().initialized);
        REQUIRE(adapter.coverage().truthGrid().validCount == 1);
        REQUIRE(adapter.coverage().certifiedCoveredCount() == 0);

        gf::route_b_mission_detail::FrontierSweepPolicy policy(
            tri, 105.0, 0.8, 1.8, 4.0, 1.0, 400.0, false, 0.0, 850.0);

        const auto snap = adapter.runtimeSnapshot();
        std::map<gf::NodeId, Eigen::Vector2d> nom;
        std::map<gf::NodeId, double> yaw;

        // Cell center is (200, 280) -> desired heading -pi/2 rad; |err| = 0.03 (< 0.05) -> rate == 0.0
        swarm.robots[0]->model->setStateVariable("yawRad", -M_PI / 2.0 + 0.03);
        policy.computeCommands(swarm, adapter, snap.estimate, 0, nom, yaw);
        const auto aim = policy.aimPoint(1);
        REQUIRE(aim.has_value());
        CHECK((*aim - Eigen::Vector2d(200.0, 300.0)).norm() > 15.0);
        CHECK((*aim - Eigen::Vector2d(200.0, 300.0)).norm() < 30.0);
        CHECK(yaw.at(1) == doctest::Approx(0.0));

        // Large yaw error (yaw=0 -> desired -pi/2) -> smooth turn toward target, strictly bounded in [-1.0, 1.0]
        swarm.robots[0]->model->setStateVariable("yawRad", 0.0);
        policy.computeCommands(swarm, adapter, snap.estimate, 1, nom, yaw);
        CHECK(yaw.at(1) == doctest::Approx(-1.0));
    }
}

TEST_CASE("Closed-loop oblique_asymmetric Entry+Reconfig+Search drastically reduces cumulative yaw change vs constant spin") {
    gf::RouteBMissionConfig mcfg;
    mcfg.domain_name = "oblique_asymmetric";
    mcfg.t_fork_s = 12.0;
    mcfg.max_mission_s = 650.0;
    mcfg.patrol_radius_m = 105.0;
    mcfg.v_cruise_mps = 14.0;
    mcfg.uwb_d_r_m = 600.0;
    mcfg.uwb_sigma0_m = 1.0;
    mcfg.uwb_d_max_m = 1200.0;
    mcfg.link_seed = 20261123;
    mcfg.skip_stay_arm = true;

    const auto res = gf::runTwoRegimeMissionComparison(mcfg);
    std::cout << "[YawControl oblique_asymmetric degraded switch] cov="
              << res.switch_arm.final_certified_coverage
              << " mean_cum_yaw_rad=" << res.switch_arm.audit.mean_cumulative_abs_yaw_rad
              << " max_cum_yaw_rad=" << res.switch_arm.audit.max_cumulative_abs_yaw_rad
              << " mean_abs_rate=" << res.switch_arm.audit.mean_abs_yaw_rate_radps
              << " max_abs_rate=" << res.switch_arm.audit.max_abs_yaw_rate_radps << std::endl;
    CHECK(res.switch_arm.reconfig_completed);
    CHECK(res.switch_arm.audit.collision_violations == 0);
    CHECK(res.switch_arm.audit.reference_violations == 0);
    CHECK(res.switch_arm.audit.axis_speed_violations == 0);
    CHECK(res.switch_arm.audit.max_abs_yaw_rate_radps <= 1.0 + 1.0e-6);

    // Old constant spin during ~582 s Entry+Reconfig alone spun ~0.5*1.8*570 = 513 rad per UAV (>80 rotations).
    // Singularity-free velocity-aligned + lookahead yaw keeps mean cumulative |Delta psi| drastically smaller.
    CHECK(res.switch_arm.audit.mean_cumulative_abs_yaw_rad > 0.5);
    CHECK(res.switch_arm.audit.mean_cumulative_abs_yaw_rad < 80.0);
    CHECK(res.switch_arm.audit.mean_abs_yaw_rate_radps < 0.15);
}
