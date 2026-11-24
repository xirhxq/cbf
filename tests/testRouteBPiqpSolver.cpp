#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/ProgressCompatibility.hpp"
#include "grand_finale/RouteBClosedLoopAudit.hpp"
#include "grand_finale/RouteBPolarEntry.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"
#include "optimisers/optimisers"

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <map>
#include <vector>

namespace {

std::vector<gf::CanonicalHardRow> sampleHocbfRows() {
    // CanonicalHardRow encodes: control_coefficient^T u + constant >= 0
    return {
        gf::makeCanonicalGammaRow("box-x-upper", 1, {-1.0, 0.0}, 4.0),
        gf::makeCanonicalGammaRow("box-x-lower", 1, {1.0, 0.0}, 4.0),
        gf::makeCanonicalGammaRow("box-y-upper", 1, {0.0, -1.0}, 4.0),
        gf::makeCanonicalGammaRow("box-y-lower", 1, {0.0, 1.0}, 4.0),
        gf::makeCanonicalGammaRow("halfspace-a", 1, {1.0, 0.0}, 0.0),
        gf::makeCanonicalGammaRow("halfspace-b", 1, {0.6, 0.8}, -1.2),
    };
}

gf::CanonicalQpRequest sampleRequest(const Eigen::Vector2d& nominal) {
    return gf::CanonicalQpRequest{
        gf::SolverProfile::OpenSource, 1, 7, 11, gf::SupervisorMode::Search,
        nominal, 4.0, sampleHocbfRows(), 1e-7};
}

}  // namespace

TEST_CASE("Vendored PIQP optimiser is registered and matches exact 2D HOCBF projection") {
    const auto available = getAvailableOptimisers();
    REQUIRE(std::find(available.begin(), available.end(), "PIQP") != available.end());

    json settings = {
        {"k_delta", 1.0},
        {"absolute-tolerance", 1.0e-9},
        {"relative-tolerance", 1.0e-9},
        {"maximum-iterations", 1000000},
        {"explicit-row-scaling", true},
    };
    auto optimiser = createOptimiser("PIQP", settings);
    REQUIRE(optimiser != nullptr);

    const Eigen::Vector2d nominal(-1.5, 0.25);
    const auto rows = sampleHocbfRows();
    const auto exact = gf::evaluateProgressCompatibility(
        rows, 1, nominal, 4.0,
        gf::ProgressCompatibilityConfig{
            std::numeric_limits<double>::max(), 0.0, 1.0e-9, true});
    REQUIRE(exact.polytope_nonempty);

    optimiser->start(2, 2);
    optimiser->setObjective(nominal);
    for (const auto& row : rows) {
        optimiser->addLinearConstraint(row.control_coefficient, -row.constant);
    }
    // Include one large-scale numerically stiff row that leaves exact polytope unchanged
    Eigen::VectorXd stiff_row(2);
    stiff_row << 1.0e6, 0.0;
    optimiser->addLinearConstraint(stiff_row, -5.0e5);

    const Eigen::VectorXd sol = optimiser->solve();
    REQUIRE(sol.size() >= 2);
    REQUIRE(sol.allFinite());

    const json st = optimiser->getStatus();
    CHECK(st.value("status", "") == "optimal");
    CHECK(st.value("solver", "") == "PIQP");
    CHECK(st.value("iteration_count", -1) > 0);
    CHECK(st.value("primal_residual", 1.0) <= 1.0e-7);
    CHECK(st.value("dual_residual", 1.0) <= 1.0e-7);
    CHECK(st.value("min_linear_residual", -1.0) >= -1.0e-7);
    CHECK((sol.head<2>() - exact.projection).norm() <= 1.0e-7);
}

TEST_CASE("CanonicalHocbfQpController OpenSource profile uses PIQP and passes oracle & empty-polytope gates") {
    gf::CanonicalHocbfQpController controller;
    const auto cold = controller.solve(sampleRequest({-1.5, 0.25}));
    const auto warm = controller.solve(sampleRequest({-1.5, 0.25}));

    REQUIRE(cold.solver_succeeded);
    REQUIRE(cold.residual_verified);
    REQUIRE(cold.control_available);
    CHECK(cold.exact_oracle_error <= 1.0e-7);
    CHECK(cold.minimum_hard_residual >= -1.0e-7);
    CHECK(cold.solver_cold_start);
    CHECK_FALSE(cold.solver_instance_reused);
    CHECK_FALSE(warm.solver_cold_start);
    CHECK(warm.solver_instance_reused);
    CHECK((cold.control - warm.control).norm() <= 1.0e-10);

    auto impossible = sampleRequest({0.0, 0.0});
    impossible.rows.push_back(
        gf::makeCanonicalGammaRow("contradict", 1, {-1.0, 0.0}, -1.0));
    const auto rejected = controller.solve(impossible);
    CHECK_FALSE(rejected.hard_polytope_nonempty);
    CHECK_FALSE(rejected.control_available);
    CHECK(rejected.failure_reason == "hard_polytope_empty");

    // Corner optimum u* == u_nom == (4, 4) regression (duality gap accuracy < 1e-5)
    const auto corner = controller.solve(sampleRequest({4.0, 4.0}));
    REQUIRE(corner.solver_succeeded);
    REQUIRE(corner.residual_verified);
    CHECK(corner.exact_oracle_error <= 1.0e-5);

    // Multi-scale interior regression captured from TDD-7 broad-domain sweep (const up to 3637)
    std::vector<gf::CanonicalHardRow> tdd7_rows = {
        gf::makeCanonicalGammaRow("axis_velocity:6:vx:lower", 6, {1.0, 0.0}, 125.464),
        gf::makeCanonicalGammaRow("axis_velocity:6:vx:upper", 6, {-1.0, 0.0}, 115.511),
        gf::makeCanonicalGammaRow("axis_velocity:6:vy:lower", 6, {0.0, 1.0}, 144.861),
        gf::makeCanonicalGammaRow("axis_velocity:6:vy:upper", 6, {0.0, -1.0}, 96.1136),
        gf::makeCanonicalGammaRow("collision:1--6:owner:6", 6, {-0.946713, -0.322079}, 2013.09),
        gf::makeCanonicalGammaRow("collision:6--105:owner:6", 6, {-0.987703, 0.156341}, 3637.28),
        gf::makeCanonicalGammaRow("collision:6--7:owner:6", 6, {-0.999934, 0.0115035}, 336.46),
        gf::makeCanonicalGammaRow("collision:6--8:owner:6", 6, {-0.991606, 0.129298}, 333.935),
        gf::makeCanonicalGammaRow("input:6:ax:lower", 6, {1.0, 0.0}, 4.0),
        gf::makeCanonicalGammaRow("input:6:ax:upper", 6, {-1.0, 0.0}, 4.0),
        gf::makeCanonicalGammaRow("input:6:ay:lower", 6, {0.0, 1.0}, 4.0),
        gf::makeCanonicalGammaRow("input:6:ay:upper", 6, {0.0, -1.0}, 4.0),
        gf::makeCanonicalGammaRow("reference:7->6:owner:6", 6, {0.999934, -0.0115035}, 70.5701),
        gf::makeCanonicalGammaRow("reference:8->6:owner:6", 6, {0.991606, -0.129298}, 73.2387),
    };
    const auto tdd7_res = controller.solve(gf::CanonicalQpRequest{
        gf::SolverProfile::OpenSource, 6, 8, 12, gf::SupervisorMode::Search,
        {2.0653, -0.684857}, 4.0, tdd7_rows, 1.0e-7});
    REQUIRE(tdd7_res.solver_succeeded);
    REQUIRE(tdd7_res.residual_verified);
    REQUIRE(tdd7_res.control_available);
    CHECK(tdd7_res.solver_iteration_count < 250);
    CHECK(tdd7_res.exact_oracle_error <= 1.0e-6);
}

TEST_CASE("Closed-loop Route B polar entry executes cleanly under OpenSource PIQP profile") {
    gf::Task10p10Scenario scenario;
    scenario.id = "route_b_piqp_smoke";
    scenario.width_m = 1600.0;
    scenario.height_m = 1600.0;
    scenario.mobile_ids = {1, 2, 3};
    scenario.fixed_positions = {{100, {0.0, 0.0}}, {101, {400.0, 0.0}}};
    scenario.mobile_positions = {{200.0, 260.0}, {420.0, 320.0}, {220.0, 420.0}};
    scenario.initial_topology = {
        {100, 1}, {101, 1},
        {101, 2}, {1, 2},
        {1, 3}, {2, 3},
    };

    json settings = gf::task10p10SwarmSettings(scenario, gf::SolverProfile::OpenSource);
    REQUIRE(settings.at("optimiser").get<std::string>() == "PIQP");
    settings["formation"]["bases-id"] = {{0, 1}};
    settings["execute"]["time-total"] = 40.0;
    settings["execute"]["time-step"] = 0.1;

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
    cfg.cbf2026_uwb_channel.d_r_m = 850.0;
    cfg.cbf2026_uwb_channel.sigma0_m = 0.5;
    cfg.cbf2026_uwb_channel.d_max_m = 1200.0;
    cfg.cbf2026_uwb_channel.link_seed = 20261124;

    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, scenario.mobile_ids, scenario.fixed_positions,
        scenario.initial_topology, cfg);
    REQUIRE(adapter.initializeStageZero().initialized);

    gf::RouteBClosedLoopAuditor auditor(
        scenario.mobile_ids, scenario.fixed_positions, 10.0, 850.0, 25.0, 3.0);

    for (int tick = 0; tick < 30; ++tick) {
        const auto snap = adapter.runtimeSnapshot();
        std::map<gf::NodeId, Eigen::Vector2d> nom;
        std::map<gf::NodeId, double> yaw_rates;
        for (std::size_t idx = 0; idx < scenario.mobile_ids.size(); ++idx) {
            const gf::NodeId id = scenario.mobile_ids[idx];
            const Eigen::Vector2d v = snap.estimate.mean.segment<2>(4 * idx + 2);
            const Eigen::Vector2d u = 1.5 * (Eigen::Vector2d(4.0, 4.0) - v);
            nom[id] = u.cwiseMax(-cfg.acceleration_half_box).cwiseMin(cfg.acceleration_half_box);
            yaw_rates[id] = 0.0;
        }
        const auto step = adapter.stepWithNominalAndYawRates(nom, yaw_rates);
        if (!step.advanced) {
            FAIL("step failed at tick=" << tick << " reason=" << step.reason);
        }
        auditor.observe(swarm, adapter.runtimeSnapshot());
    }

    const auto summary = auditor.summarize();
    CHECK(summary.collision_violations == 0);
    CHECK(summary.reference_violations == 0);
    CHECK(summary.axis_speed_violations == 0);
}
