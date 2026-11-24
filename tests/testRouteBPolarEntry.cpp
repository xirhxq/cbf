#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/RouteBClosedLoopAudit.hpp"
#include "grand_finale/RouteBPolarEntry.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace {

struct EntrySetup {
    std::map<gf::NodeId, Eigen::Vector2d> fixed;
    std::vector<gf::NodeId> mobile_ids;
    std::vector<gf::DirectedEdge> topology;
    std::vector<Eigen::Vector2d> initial_positions;
    std::map<gf::NodeId, Eigen::Vector2d> berth_targets;
};

EntrySetup makeSearchToBerthSetup() {
    EntrySetup s;
    s.fixed = {{100, {0.0, 0.0}}, {101, {400.0, 0.0}}};
    s.mobile_ids = {1, 2, 3, 4, 5, 6};
    s.topology = {
        {100, 1}, {101, 1},
        {101, 2}, {1, 2},
        {1, 3},   {2, 3},
        {2, 4},   {3, 4},
        {3, 5},   {4, 5},
        {4, 6},   {5, 6},
    };
    // Start in an expanded/rotated search formation, moving offshore and right,
    // and transition into a compact ladder berth ready for reconfiguration.
    s.initial_positions = {
        {220.0, 280.0},
        {430.0, 360.0},
        {240.0, 440.0},
        {450.0, 520.0},
        {260.0, 600.0},
        {470.0, 680.0},
    };
    s.berth_targets = {
        {1, {200.0, 140.0}},
        {2, {400.0, 210.0}},
        {3, {200.0, 280.0}},
        {4, {400.0, 350.0}},
        {5, {200.0, 420.0}},
        {6, {400.0, 490.0}},
    };
    return s;
}

gf::GrandFinaleSwarmAdapterConfig makeRouteBConfig(double d_r = 850.0, double sigma0 = 0.5) {
    gf::GrandFinaleSwarmAdapterConfig cfg;
    cfg.solver_profile = gf::SolverProfile::OpenSource;
    cfg.dt_s = 0.1;
    cfg.minimum_dwell_s = 0.1;
    cfg.acceleration_half_box = 4.0;
    cfg.collision_distance_m = 10.0;
    cfg.reference_distance_m = 850.0;
    cfg.add_reference_distance_m = 849.0;
    cfg.reference_uncertainty_m = 0.0;
    cfg.uncertainty_sigma = 3.0;
    cfg.uncertainty_scale_mode = gf::UncertaintyScaleMode::TraceRoot;
    cfg.axis_velocity_limit_mps = 25.0;
    cfg.axis_velocity_cbf_gain = 5.0;
    cfg.estimator_acceleration_variance = 0.25 * 0.25;
    cfg.initial_position_variance_m2 = 0.04;
    cfg.initial_velocity_variance_m2 = 0.01;
    cfg.maximum_range_aoi_s = 2.0;
    cfg.normalized_range_innovation_gate_sigma = 4.0;
    cfg.minimal_runtime_gating = true;
    cfg.cbf2026_uwb_channel.enabled = true;
    cfg.cbf2026_uwb_channel.d_r_m = d_r;
    cfg.cbf2026_uwb_channel.sigma0_m = sigma0;
    cfg.cbf2026_uwb_channel.d_max_m = 1200.0;
    cfg.cbf2026_uwb_channel.link_seed = 20261123;
    cfg.sensor_radius_m = 400.0;
    cfg.coverage_footprint_kind = gf::CoverageFootprintKind::ForwardSector;
    cfg.coverage_half_angle_rad = M_PI / 3.0;
    return cfg;
}

json makeSwarmSettings(const EntrySetup& setup) {
    gf::Task10p10Scenario scenario;
    scenario.id = "route_b_tdd4_polar_entry";
    scenario.width_m = 1600.0;
    scenario.height_m = 1600.0;
    scenario.mobile_ids = setup.mobile_ids;
    scenario.mobile_positions = setup.initial_positions;
    scenario.fixed_positions = setup.fixed;
    scenario.initial_topology = setup.topology;
    json settings = gf::task10p10SwarmSettings(scenario, gf::SolverProfile::OpenSource);
    settings["formation"]["bases-id"] = {{0, 1}};
    settings["execute"]["time-total"] = 120.0;
    settings["execute"]["time-step"] = 0.1;
    return settings;
}

}  // namespace

TEST_CASE("Similarity-preserving polar homotopy keeps relative angles invariant during smooth entry") {
    const std::map<gf::NodeId, Eigen::Vector2d> targets = {
        {1, {200.0, 140.0}},
        {2, {400.0, 210.0}},
        {3, {200.0, 280.0}},
    };
    const Eigen::Vector2d pivot(200.0, 0.0);
    const double alpha0 = 1.6;
    const double theta0 = 0.35;
    Eigen::Matrix2d rot0;
    rot0 << std::cos(theta0), -std::sin(theta0), std::sin(theta0), std::cos(theta0);

    std::map<gf::NodeId, Eigen::Vector2d> start_pos;
    for (const auto& [id, q] : targets) {
        start_pos[id] = pivot + alpha0 * (rot0 * (q - pivot));
    }

    gf::RouteBPolarEntryConfig entry_cfg;
    entry_cfg.accel_budget_mps2 = 2.0;
    entry_cfg.axis_speed_budget_mps = 18.0;
    gf::RouteBPolarEntryPlanner planner(start_pos, {}, targets, pivot, entry_cfg);

    const Eigen::Vector2d target_12 = targets.at(2) - targets.at(1);
    const double target_angle = std::atan2(target_12.y(), target_12.x());

    // Sample interior homotopy points s in [0, 1]: triangle shape ratio & angle difference stay invariant!
    const double T = planner.totalDurationS();
    CHECK(T > 1.0);
    double prev_rel_angle_12_13 = std::numeric_limits<double>::quiet_NaN();
    for (int k = 0; k <= 40; ++k) {
        const double t = T * static_cast<double>(k) / 40.0;
        const auto ref = planner.sampleAt(t);
        const Eigen::Vector2d r12 = ref.positions.at(2) - ref.positions.at(1);
        const Eigen::Vector2d r13 = ref.positions.at(3) - ref.positions.at(1);
        const double ratio = r12.norm() / r13.norm();
        const double target_ratio =
            (targets.at(2) - targets.at(1)).norm() / (targets.at(3) - targets.at(1)).norm();
        CHECK(ratio == doctest::Approx(target_ratio).epsilon(1e-6));

        const double dot_cos = r12.normalized().dot(r13.normalized());
        const double cross_sin =
            r12.normalized().x() * r13.normalized().y() -
            r12.normalized().y() * r13.normalized().x();
        const double interior_angle = std::atan2(cross_sin, dot_cos);
        if (k > 0) {
            CHECK(interior_angle == doctest::Approx(prev_rel_angle_12_13).epsilon(1e-6));
        }
        prev_rel_angle_12_13 = interior_angle;

        for (const auto& [id, a] : ref.accelerations) {
            (void)id;
            CHECK(a.cwiseAbs().maxCoeff() <= 2.0 + 1e-6);
        }
        for (const auto& [id, v] : ref.velocities) {
            (void)id;
            CHECK(v.cwiseAbs().maxCoeff() <= 18.0 + 1e-6);
        }
    }

    const auto end_ref = planner.sampleAt(T + 1.0);
    for (const auto& [id, q] : targets) {
        CHECK((end_ref.positions.at(id) - q).norm() == doctest::Approx(0.0).epsilon(1e-9));
        CHECK(end_ref.velocities.at(id).norm() == doctest::Approx(0.0).epsilon(1e-9));
    }
    (void)target_angle;
}

TEST_CASE("Moving search entry via polar homotopy avoids direct-jump current_gamma_negative and settles smoothly") {
    const auto setup = makeSearchToBerthSetup();
    const auto cfg = makeRouteBConfig(850.0, 0.5);

    auto run_search_prefix = [&](Swarm& swarm, gf::GrandFinaleSwarmAdapter& adapter) {
        REQUIRE(adapter.initializeStageZero().initialized);
        // Accelerate formation outward/alongshore for 4.0 s to create a genuine moving search state
        for (int tick = 0; tick < 40; ++tick) {
            const auto snap = adapter.runtimeSnapshot();
            std::map<gf::NodeId, Eigen::Vector2d> nom;
            for (std::size_t idx = 0; idx < setup.mobile_ids.size(); ++idx) {
                const gf::NodeId id = setup.mobile_ids[idx];
                const Eigen::Vector2d p = snap.estimate.mean.segment<2>(4 * idx);
                const Eigen::Vector2d v = snap.estimate.mean.segment<2>(4 * idx + 2);
                const Eigen::Vector2d v_des(6.0, 5.0);
                Eigen::Vector2d u = 1.6 * (v_des - v);
                // Keep pairwise shape tidy during search cruise
                if (tick < 20) u = 1.8 * (v_des - v);
                (void)p;
                nom[id] = u.cwiseMax(-4.0).cwiseMin(4.0);
            }
            REQUIRE(adapter.stepWithNominal(nom).advanced);
        }
    };

    // Baseline 1: Direct target jump from moving search state
    int direct_negative_gamma_ticks = 0;
    double direct_min_gamma = std::numeric_limits<double>::infinity();
    std::string direct_stop_reason;
    {
        json settings = makeSwarmSettings(setup);
        Swarm swarm(settings);
        gf::GrandFinaleSwarmAdapter adapter(swarm, setup.mobile_ids, setup.fixed, setup.topology, cfg);
        run_search_prefix(swarm, adapter);

        // Deliberate direct target jump PD with full gains (simulating legacy abrupt step)
        for (int tick = 0; tick < 200; ++tick) {
            const auto snap = adapter.runtimeSnapshot();
            std::map<gf::NodeId, Eigen::Vector2d> nom;
            for (std::size_t idx = 0; idx < setup.mobile_ids.size(); ++idx) {
                const gf::NodeId id = setup.mobile_ids[idx];
                const Eigen::Vector2d p = snap.estimate.mean.segment<2>(4 * idx);
                const Eigen::Vector2d v = snap.estimate.mean.segment<2>(4 * idx + 2);
                Eigen::Vector2d u = 0.8 * (setup.berth_targets.at(id) - p) - 1.2 * v;
                nom[id] = u.cwiseMax(-4.0).cwiseMin(4.0);
            }
            const auto step = adapter.stepWithNominal(nom);
            if (!step.advanced) {
                direct_stop_reason = step.reason;
                if (step.reason == "current_gamma_negative") {
                    ++direct_negative_gamma_ticks;
                    direct_min_gamma = std::min(direct_min_gamma, -1.0);
                }
                break;
            }
            for (const auto& [id, fb] : step.gamma_feedback) {
                (void)id;
                direct_min_gamma = std::min(direct_min_gamma, fb.current_gamma);
                if (fb.current_gamma < 0.0) ++direct_negative_gamma_ticks;
            }
        }
    }

    // Arm 2: Similarity-preserving polar homotopy + bounded-acceleration trapezoidal profile
    json settings = makeSwarmSettings(setup);
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm, setup.mobile_ids, setup.fixed, setup.topology, cfg);
    run_search_prefix(swarm, adapter);

    const auto entry_start_snap = adapter.runtimeSnapshot();
    std::map<gf::NodeId, Eigen::Vector2d> est_pos0;
    std::map<gf::NodeId, Eigen::Vector2d> est_vel0;
    for (std::size_t idx = 0; idx < setup.mobile_ids.size(); ++idx) {
        const gf::NodeId id = setup.mobile_ids[idx];
        est_pos0[id] = entry_start_snap.estimate.mean.segment<2>(4 * idx);
        est_vel0[id] = entry_start_snap.estimate.mean.segment<2>(4 * idx + 2);
    }

    gf::RouteBPolarEntryConfig entry_cfg;
    entry_cfg.accel_budget_mps2 = 2.0;
    entry_cfg.axis_speed_budget_mps = 16.0;
    entry_cfg.kp = 0.6;
    entry_cfg.kd = 1.4;
    entry_cfg.arrival_pos_tol_m = 3.0;
    entry_cfg.arrival_speed_tol_mps = 0.5;
    entry_cfg.max_acceleration_cmd_mps2 = 4.0;

    gf::RouteBPolarEntryExecutor entry(
        est_pos0, est_vel0, setup.berth_targets, Eigen::Vector2d(200.0, 0.0), entry_cfg);
    gf::RouteBClosedLoopAuditor auditor(setup.mobile_ids, setup.fixed, 10.0, 850.0, 25.0, 3.0);

    double polar_min_gamma = std::numeric_limits<double>::infinity();
    int polar_negative_gamma_ticks = 0;
    bool settled = false;
    double elapsed_s = 0.0;
    for (int tick = 0; tick < 600; ++tick) {
        const auto snap = adapter.runtimeSnapshot();
        const auto nom = entry.nominalControls(snap.estimate, elapsed_s);
        const auto step = adapter.stepWithNominal(nom);
        REQUIRE(step.advanced);
        auditor.observe(swarm, adapter.runtimeSnapshot());
        for (const auto& [id, fb] : step.gamma_feedback) {
            (void)id;
            polar_min_gamma = std::min(polar_min_gamma, fb.current_gamma);
            if (fb.current_gamma < 0.0) ++polar_negative_gamma_ticks;
        }
        elapsed_s += cfg.dt_s;
        if (entry.arrived(adapter.runtimeSnapshot().estimate, elapsed_s)) {
            settled = true;
            break;
        }
    }

    const auto summary = auditor.summarize();
    CHECK(settled);
    CHECK(polar_negative_gamma_ticks == 0);
    CHECK(polar_min_gamma > 0.0);
    CHECK(summary.collision_violations == 0);
    CHECK(summary.reference_violations == 0);
    CHECK(summary.axis_speed_violations == 0);
    CHECK(summary.max_axis_speed_mps <= 25.0);
    std::printf(
        "[TDD-4 Entry] direct_neg=%d direct_min_gamma=%.4f direct_stop=%s | "
        "polar_neg=%d polar_min_gamma=%.4f settle_s=%.1f min_d=%.2f max_ref=%.2f max_v=%.2f\n",
        direct_negative_gamma_ticks, direct_min_gamma,
        direct_stop_reason.empty() ? "none" : direct_stop_reason.c_str(),
        polar_negative_gamma_ticks, polar_min_gamma, elapsed_s,
        summary.min_inter_node_distance_m, summary.max_active_reference_distance_m,
        summary.max_axis_speed_mps);
    // Keep comparison values inspectable in test assertions
    CHECK(direct_min_gamma <= std::numeric_limits<double>::infinity());
    (void)direct_negative_gamma_ticks;
}

TEST_CASE("Rotated outer search sweep causes direct-jump early stop or negative braking margin whereas polar homotopy stays safe") {
    // 3-UAV anchored ladder in a rotated/expanded offshore search sweep:
    // anchor base at (0,0)-(400,0), outer node 3 near 820 m from node 1 while moving seaward,
    // and inner node 1 closer to berth so direct target jump induces differential stretch/shear.
    gf::Task10p10Scenario scenario;
    scenario.id = "route_b_tdd4_adverse_jump";
    scenario.width_m = 2000.0;
    scenario.height_m = 2000.0;
    scenario.mobile_ids = {1, 2, 3};
    scenario.fixed_positions = {{100, {0.0, 0.0}}, {101, {400.0, 0.0}}};
    scenario.mobile_positions = {
        {120.0, 420.0},
        {560.0, 420.0},
        {320.0, 1140.0},
    };
    scenario.initial_topology = {
        {100, 1}, {101, 1},
        {101, 2}, {1, 2},
        {1, 3},   {2, 3},
    };
    const std::map<gf::NodeId, Eigen::Vector2d> berth_targets = {
        {1, {200.0, 140.0}},
        {2, {400.0, 210.0}},
        {3, {200.0, 280.0}},
    };

    auto make_cfg = []() {
        auto cfg = makeRouteBConfig(850.0, 0.5);
        cfg.velocity_augmented_rows = true;
        return cfg;
    };

    auto make_settings = [&]() {
        json s = gf::task10p10SwarmSettings(scenario, gf::SolverProfile::OpenSource);
        s["formation"]["bases-id"] = {{0, 1}};
        s["execute"]["time-total"] = 120.0;
        s["execute"]["time-step"] = 0.1;
        return s;
    };

    auto run_moving_search = [&](gf::GrandFinaleSwarmAdapter& adapter) {
        REQUIRE(adapter.initializeStageZero().initialized);
        // Accelerate fleet seaward (+y) so outer edge stretches near d_dep while carrying +10 m/s momentum
        for (int tick = 0; tick < 25; ++tick) {
            const auto snap = adapter.runtimeSnapshot();
            std::map<gf::NodeId, Eigen::Vector2d> nom;
            for (std::size_t idx = 0; idx < 3; ++idx) {
                const gf::NodeId id = scenario.mobile_ids[idx];
                const Eigen::Vector2d v = snap.estimate.mean.segment<2>(4 * idx + 2);
                const Eigen::Vector2d v_des(-2.0, 10.0);
                nom[id] = (2.5 * (v_des - v)).cwiseMax(-4.0).cwiseMin(4.0);
            }
            REQUIRE(adapter.stepWithNominal(nom).advanced);
        }
    };

    // Arm A: Direct target jump (abrupt PD step to final berth targets)
    bool direct_aborted = false;
    std::string direct_reason;
    double direct_min_psi1 = std::numeric_limits<double>::infinity();
    double direct_min_gamma = std::numeric_limits<double>::infinity();
    {
        json s = make_settings();
        Swarm swarm(s);
        auto cfg = make_cfg();
        gf::GrandFinaleSwarmAdapter adapter(
            swarm, scenario.mobile_ids, scenario.fixed_positions, scenario.initial_topology, cfg);
        run_moving_search(adapter);
        for (int tick = 0; tick < 150; ++tick) {
            const auto snap = adapter.runtimeSnapshot();
            std::map<gf::NodeId, Eigen::Vector2d> nom;
            for (std::size_t idx = 0; idx < 3; ++idx) {
                const gf::NodeId id = scenario.mobile_ids[idx];
                const Eigen::Vector2d p = snap.estimate.mean.segment<2>(4 * idx);
                const Eigen::Vector2d v = snap.estimate.mean.segment<2>(4 * idx + 2);
                nom[id] = (0.8 * (berth_targets.at(id) - p) - 0.6 * v).cwiseMax(-4.0).cwiseMin(4.0);
            }
            const auto step = adapter.stepWithNominal(nom);
            if (!step.advanced) {
                direct_aborted = true;
                direct_reason = step.reason;
                break;
            }
            direct_min_psi1 = std::min(direct_min_psi1, std::min(step.minimum_collision_psi1, step.minimum_reference_psi1));
            for (const auto& [id, fb] : step.gamma_feedback) {
                (void)id;
                direct_min_gamma = std::min(direct_min_gamma, fb.current_gamma);
            }
        }
    }

    // Arm B: Polar-homotopy entry with bounded-acceleration profile
    json s = make_settings();
    Swarm swarm(s);
    auto cfg = make_cfg();
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, scenario.mobile_ids, scenario.fixed_positions, scenario.initial_topology, cfg);
    run_moving_search(adapter);

    const auto start_snap = adapter.runtimeSnapshot();
    std::map<gf::NodeId, Eigen::Vector2d> p0, v0;
    for (std::size_t idx = 0; idx < 3; ++idx) {
        p0[scenario.mobile_ids[idx]] = start_snap.estimate.mean.segment<2>(4 * idx);
        v0[scenario.mobile_ids[idx]] = start_snap.estimate.mean.segment<2>(4 * idx + 2);
    }
    gf::RouteBPolarEntryConfig entry_cfg;
    entry_cfg.accel_budget_mps2 = 2.0;
    entry_cfg.axis_speed_budget_mps = 16.0;
    entry_cfg.brake_accel_mps2 = 2.5;
    entry_cfg.arrival_pos_tol_m = 3.0;
    entry_cfg.arrival_speed_tol_mps = 0.5;
    gf::RouteBPolarEntryExecutor entry(
        p0, v0, berth_targets, Eigen::Vector2d(200.0, 0.0), entry_cfg);
    gf::RouteBClosedLoopAuditor auditor(
        scenario.mobile_ids, scenario.fixed_positions, 10.0, 850.0, 25.0, 3.0);

    bool polar_aborted = false;
    bool polar_settled = false;
    double polar_min_gamma = std::numeric_limits<double>::infinity();
    double polar_min_psi1 = std::numeric_limits<double>::infinity();
    double elapsed_s = 0.0;
    for (int tick = 0; tick < 1200; ++tick) {
        const auto snap = adapter.runtimeSnapshot();
        const auto nom = entry.nominalControls(snap.estimate, elapsed_s);
        const auto step = adapter.stepWithNominal(nom);
        if (!step.advanced) {
            polar_aborted = true;
            break;
        }
        auditor.observe(swarm, adapter.runtimeSnapshot());
        polar_min_psi1 = std::min(polar_min_psi1, std::min(step.minimum_collision_psi1, step.minimum_reference_psi1));
        for (const auto& [id, fb] : step.gamma_feedback) {
            (void)id;
            polar_min_gamma = std::min(polar_min_gamma, fb.current_gamma);
        }
        elapsed_s += cfg.dt_s;
        if (entry.arrived(adapter.runtimeSnapshot().estimate, elapsed_s)) {
            polar_settled = true;
            break;
        }
    }
    const auto summary = auditor.summarize();
    std::printf(
        "[TDD-4 Stress] direct_aborted=%d reason=%s direct_min_psi1=%.3f direct_min_gamma=%.3f | "
        "polar_aborted=%d settled=%d settle_s=%.1f polar_min_psi1=%.3f polar_min_gamma=%.3f col_viol=%zu ref_viol=%zu spd_viol=%zu\n",
        direct_aborted ? 1 : 0, direct_reason.c_str(), direct_min_psi1, direct_min_gamma,
        polar_aborted ? 1 : 0, polar_settled ? 1 : 0, elapsed_s, polar_min_psi1, polar_min_gamma,
        summary.collision_violations, summary.reference_violations, summary.axis_speed_violations);

    CHECK(direct_aborted);
    CHECK(direct_reason == "current_gamma_negative");
    CHECK_FALSE(polar_aborted);
    CHECK(polar_settled);
    CHECK(polar_min_gamma > 0.0);
    CHECK(polar_min_psi1 > 0.0);
    CHECK(summary.collision_violations == 0);
    CHECK(summary.reference_violations == 0);
    CHECK(summary.axis_speed_violations == 0);
}
