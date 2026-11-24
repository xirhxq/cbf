#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/DistanceRangeAvailability.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/ReferenceEligibility.hpp"
#include "grand_finale/RouteBClosedLoopAudit.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

namespace {

json makeLadderSettings6(const std::vector<Eigen::Vector2d>& positions,
                           const std::map<gf::NodeId, Eigen::Vector2d>& fixed) {
    gf::Task10p10Scenario scenario;
    scenario.id = "route_b_tdd3_ladder6";
    scenario.width_m = 1600.0;
    scenario.height_m = 1600.0;
    scenario.mobile_ids = {1, 2, 3, 4, 5, 6};
    scenario.mobile_positions = positions;
    scenario.fixed_positions = fixed;
    scenario.initial_topology = {
        {100, 1}, {101, 1},
        {101, 2}, {1, 2},
        {1, 3},   {2, 3},
        {2, 4},   {3, 4},
        {3, 5},   {4, 5},
        {4, 6},   {5, 6},
    };
    json settings = gf::task10p10SwarmSettings(scenario, gf::SolverProfile::OpenSource);
    settings["formation"]["bases-id"] = {{0, 1}};
    settings["execute"]["time-total"] = 30.0;
    settings["execute"]["time-step"] = 0.1;
    return settings;
}

}  // namespace

TEST_CASE("CBF2026 UWB physical channel obeys 1/(1+(d/d_r)^2), d_max hard cutoff, and sigma(d)") {
    gf::Cbf2026UwbChannelConfig nominal;
    nominal.enabled = true;
    nominal.d_r_m = 850.0;
    nominal.sigma0_m = 0.5;
    nominal.d_max_m = 1200.0;
    nominal.link_seed = 20261123;

    CHECK(nominal.acquisitionProbability(0.0) == doctest::Approx(1.0));
    CHECK(nominal.acquisitionProbability(850.0) == doctest::Approx(0.5));
    CHECK(nominal.acquisitionProbability(1200.0) ==
          doctest::Approx(1.0 / (1.0 + std::pow(1200.0 / 850.0, 2))));
    CHECK(nominal.acquisitionProbability(1200.001) == 0.0);
    CHECK(nominal.rangingStdM(0.0) == doctest::Approx(0.5));
    CHECK(nominal.rangingStdM(850.0) == doctest::Approx(1.0));
    CHECK(nominal.expectedFisherWeight(1200.001) == 0.0);
    CHECK_FALSE(nominal.acquired(1200.1, 0.0));

    gf::Cbf2026UwbChannelConfig degraded;
    degraded.enabled = true;
    degraded.d_r_m = 600.0;
    degraded.sigma0_m = 1.0;
    degraded.d_max_m = 1200.0;

    CHECK(degraded.acquisitionProbability(600.0) == doctest::Approx(0.5));
    CHECK(degraded.acquisitionProbability(1200.0) == doctest::Approx(0.2));
    CHECK(degraded.acquisitionProbability(1200.5) == 0.0);
    CHECK(degraded.rangingStdM(600.0) == doctest::Approx(2.0));
    CHECK(degraded.rangingStdM(1200.0) == doctest::Approx(5.0));
    CHECK(degraded.expectedFisherWeight(600.0) == doctest::Approx(0.5 / 4.0));

    CHECK_THROWS_AS(nominal.acquisitionProbability(-1.0), std::invalid_argument);
    CHECK_THROWS_AS(nominal.acquisitionProbability(std::numeric_limits<double>::quiet_NaN()),
                    std::invalid_argument);
    CHECK_THROWS_AS(nominal.acquired(500.0, 1.0), std::invalid_argument);
}

TEST_CASE("Uncertainty scale 3*sqrt(tr(P)) strictly dominates lambda_max and envelopes 3-sigma ellipse") {
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(4, 4);
    const double angle = 0.52;
    Eigen::Matrix2d rot;
    rot << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle);
    Eigen::Matrix2d p_diag = Eigen::Vector2d(0.25, 0.09).asDiagonal();
    Eigen::Matrix2d v_diag = Eigen::Vector2d(0.04, 0.01).asDiagonal();
    cov.block<2, 2>(0, 0) = rot * p_diag * rot.transpose();
    cov.block<2, 2>(2, 2) = rot * v_diag * rot.transpose();

    const gf::JointEstimateSnapshot snap{{1}, mean, cov, {{100, {0.0, 0.0}}}};
    const double r_eig = gf::detail::positionScaleRadius(
        snap, 1, 3.0, gf::UncertaintyScaleMode::MaxEigenvalue);
    const double r_trace = gf::detail::positionScaleRadius(
        snap, 1, 3.0, gf::UncertaintyScaleMode::TraceRoot);
    CHECK(r_eig == doctest::Approx(3.0 * 0.5));
    CHECK(r_trace == doctest::Approx(3.0 * std::sqrt(0.34)));
    CHECK(r_trace > r_eig);

    const double v_trace = gf::detail::velocityScaleRadius(
        snap, 1, 3.0, gf::UncertaintyScaleMode::TraceRoot);
    CHECK(v_trace == doctest::Approx(3.0 * std::sqrt(0.05)));

    // Verify every point on the 3-sigma Mahalanobis ellipse is inside disc of radius r_trace.
    const Eigen::Matrix2d p_pos = cov.block<2, 2>(0, 0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(p_pos);
    const Eigen::Matrix2d sqrt_p =
        solver.eigenvectors() * solver.eigenvalues().cwiseSqrt().asDiagonal() *
        solver.eigenvectors().transpose();
    for (int k = 0; k < 64; ++k) {
        const double th = 2.0 * M_PI * static_cast<double>(k) / 64.0;
        const Eigen::Vector2d e = 3.0 * sqrt_p * Eigen::Vector2d(std::cos(th), std::sin(th));
        CHECK(e.norm() <= r_trace + 1e-12);
    }
}

TEST_CASE("Single-axis 25.0 m/s velocity limit allows diagonal 20 m/s and brakes axis overshoot") {
    gf::CanonicalHardRowRequest req;
    req.mobile_ids = {1};
    req.fixed_ids = {100};
    req.states[100] = {Point(0.0, 0.0), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    req.reference_spec = {PairwiseSecondOrderBarrierKind::CommunicationUpper,
                          850.0, 0.0, 1.0, 1.0, 1.0, 0.0};
    req.collision_spec = {PairwiseSecondOrderBarrierKind::CollisionLower,
                          10.0, 0.0, 1.0, 1.0, 1.0, 0.0};
    req.acceleration_half_box = 4.0;
    req.axis_velocity_limit_mps = 25.0;
    req.axis_velocity_cbf_gain = 5.0;
    req.plant_speed_dt_s = 0.1;
    req.require_snapshot_robust_rows = true;
    req.speed_snapshot_tubes[1] = {0.0, 0.4, gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};

    // Case 1: vx=20, vy=20 (Euclidean speed 28.28 > 25, but per-axis <= 25 - 0.4 = 24.6)
    req.states[1] = {Point(100.0, 100.0), Eigen::Vector2d(20.0, 20.0), Eigen::Vector2d::Zero()};
    const auto diag_rows = gf::buildCanonicalHardRows(req);
    int axis_row_count = 0;
    for (const auto& r : diag_rows) {
        if (r.kind == gf::CanonicalHardRowKind::AxisVelocityLimit) {
            ++axis_row_count;
            CHECK(r.margin(Eigen::Vector2d::Zero()) > 0.0);
        }
    }
    CHECK(axis_row_count == 4);

    // Case 2: vx=24.8, rho=0.4 -> positive ux must be constrained, negative ux must be feasible
    req.states[1] = {Point(100.0, 100.0), Eigen::Vector2d(24.8, 0.0), Eigen::Vector2d::Zero()};
    const auto brake_rows = gf::buildCanonicalHardRows(req);
    const gf::CanonicalHardRow* vx_upper = nullptr;
    for (const auto& r : brake_rows) {
        if (r.id == "axis_velocity:1:vx:upper") vx_upper = &r;
    }
    REQUIRE(vx_upper != nullptr);
    CHECK(vx_upper->margin(Eigen::Vector2d(1.0, 0.0)) < 0.0);
    CHECK(vx_upper->margin(Eigen::Vector2d(-2.0, 0.0)) > 0.0);
}

TEST_CASE("Minimal runtime gating removes FIM/epsilon pauses and audits 6-UAV closed-loop under both UWB groups") {
    const std::map<gf::NodeId, Eigen::Vector2d> fixed = {
        {100, {0.0, 0.0}},
        {101, {400.0, 0.0}},
    };
    const std::vector<Eigen::Vector2d> positions = {
        {200.0, 380.0},
        {400.0, 470.0},
        {200.0, 560.0},
        {400.0, 650.0},
        {200.0, 740.0},
        {400.0, 830.0},
    };
    const std::vector<gf::DirectedEdge> topology = {
        {100, 1}, {101, 1},
        {101, 2}, {1, 2},
        {1, 3},   {2, 3},
        {2, 4},   {3, 4},
        {3, 5},   {4, 5},
        {4, 6},   {5, 6},
    };

    for (const auto [d_r, sigma0] : std::vector<std::pair<double, double>>{{850.0, 0.5}, {600.0, 1.0}}) {
        json settings = makeLadderSettings6(positions, fixed);
        Swarm swarm(settings);

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

        gf::GrandFinaleSwarmAdapter adapter(swarm, {1, 2, 3, 4, 5, 6}, fixed, topology, cfg);
        REQUIRE(adapter.initializeStageZero().initialized);

        // Verify range generation honors d_max = 1200 m hard cutoff
        for (const auto& gen : adapter.lastRangeGenerationAudit()) {
            if (gen.true_distance_m > 1200.0) {
                CHECK(gen.probability == 0.0);
                CHECK_FALSE(gen.acquired);
            }
        }

        // Verify minimal_runtime_gating allows transition certifier even when uncertainty > 0.95 m
        // (posterior lambda_max would fail old 0.1 m^2 threshold under degraded UWB!)
        gf::RouteBClosedLoopAuditor auditor({1, 2, 3, 4, 5, 6}, fixed, 10.0, 850.0, 25.0, 3.0);
        for (int tick = 0; tick < 80; ++tick) {
            std::map<gf::NodeId, Eigen::Vector2d> nominal;
            const auto snap = adapter.runtimeSnapshot();
            for (std::size_t idx = 0; idx < 6; ++idx) {
                const gf::NodeId id = snap.estimate.mobile_ids[idx];
                const Eigen::Vector2d p = snap.estimate.mean.segment<2>(4 * idx);
                const Eigen::Vector2d v = snap.estimate.mean.segment<2>(4 * idx + 2);
                const Eigen::Vector2d q = positions[idx];
                Eigen::Vector2d u = 0.4 * (q - p) - 0.8 * v;
                nominal[id] = u.cwiseMax(-4.0).cwiseMin(4.0);
            }
            const auto step = adapter.stepWithNominal(nominal);
            REQUIRE(step.advanced);
            auditor.observe(swarm, adapter.runtimeSnapshot());
        }
        const auto summary = auditor.summarize();
        CHECK(summary.collision_violations == 0);
        CHECK(summary.reference_violations == 0);
        CHECK(summary.axis_speed_violations == 0);
        CHECK(summary.min_inter_node_distance_m > 10.0);
        CHECK(summary.max_active_reference_distance_m < 850.0);
        CHECK(summary.max_axis_speed_mps <= 25.0);
        CHECK(summary.position_rmse_m >= 0.0);
        CHECK(summary.position_rmse_m < 5.0);
        CHECK(summary.overall_envelope_coverage > 0.80);
        CHECK(summary.per_agent_envelope_coverage.size() == 6);
    }
}

TEST_CASE("Minimal gating disables posterior and FIM transition gates while preserving 1-bit fresh range check") {
    const std::map<gf::NodeId, Eigen::Vector2d> fixed = {
        {100, {0.0, 0.0}},
        {101, {400.0, 0.0}},
    };
    const std::vector<Eigen::Vector2d> positions = {
        {200.0, 320.0},
        {500.0, 320.0},
    };
    const std::vector<gf::DirectedEdge> topology = {
        {100, 1}, {101, 1},
        {101, 2}, {1, 2},
    };

    auto run_swap_check = [&](bool minimal_gating, bool stale_link) {
        gf::Task10p10Scenario scenario;
        scenario.id = "route_b_tdd3_minimal_gate_cert";
        scenario.width_m = 1600.0;
        scenario.height_m = 1600.0;
        scenario.mobile_ids = {1, 2};
        scenario.mobile_positions = positions;
        scenario.fixed_positions = fixed;
        scenario.initial_topology = topology;
        json settings = gf::task10p10SwarmSettings(scenario, gf::SolverProfile::OpenSource);
        settings["formation"]["bases-id"] = {{0, 1}};
        settings["execute"]["time-total"] = 10.0;
        settings["execute"]["time-step"] = 0.1;
        Swarm swarm(settings);

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
        // Intentionally inflate initial variance above legacy 0.1 m^2 threshold
        cfg.initial_position_variance_m2 = 0.25;
        cfg.initial_velocity_variance_m2 = 0.01;
        cfg.maximum_posterior_eigenvalue_m2 = 0.1;
        cfg.maximum_reference_position_eigenvalue_m2 = 0.1;
        cfg.maximum_range_aoi_s = 1.5;
        cfg.normalized_range_innovation_gate_sigma = 4.0;
        cfg.minimal_runtime_gating = minimal_gating;
        if (stale_link) {
            // Drop all ranges so pre-swap link ages exceed maximum_range_aoi_s
            cfg.range_dropout_probability = 1.0;
        } else {
            cfg.cbf2026_uwb_channel.enabled = true;
            cfg.cbf2026_uwb_channel.d_r_m = 600.0;
            cfg.cbf2026_uwb_channel.sigma0_m = 1.0;
            cfg.cbf2026_uwb_channel.d_max_m = 1200.0;
            cfg.cbf2026_uwb_channel.link_seed = 20261123;
        }

        gf::GrandFinaleSwarmAdapter adapter(swarm, {1, 2}, fixed, topology, cfg);
        REQUIRE(adapter.initializeStageZero().initialized);
        // Step zero-nominal for 2.0s so stale_link ages > 1.5s while high-noise keeps P above 0.1 if applicable;
        // Also notice we audit directly when supervisor checks replacement {100, 2} replacing {1, 2}.
        std::map<gf::NodeId, Eigen::Vector2d> hold{{1, {0.0, 0.0}}, {2, {0.0, 0.0}}};
        for (int k = 0; k < 20; ++k) {
            REQUIRE(adapter.stepWithNominal(hold).advanced);
        }
        return adapter.auditReplacement({100, 2}, {1, 2});
    };

    const auto legacy_cert = run_swap_check(/*minimal=*/false, /*stale=*/false);
    // Under degraded UWB, either posterior or reference_covariance triggers under 0.1 m^2 cap
    CHECK_FALSE(legacy_cert.valid);

    const auto minimal_fresh_cert = run_swap_check(/*minimal=*/true, /*stale=*/false);
    CHECK(minimal_fresh_cert.valid);

    const auto minimal_stale_cert = run_swap_check(/*minimal=*/true, /*stale=*/true);
    CHECK_FALSE(minimal_stale_cert.valid);
}
