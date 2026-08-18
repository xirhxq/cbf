#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11pOperationalEnvelope.hpp"

TEST_CASE("Task 10.11p standard coastal scene is an isometric CBF2026 transform") {
    const auto scenario=gf::task10p11pStandardCoastalScenario();
    CHECK(gf::task10p11pCbf2026SourceCommit()==
          "47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d");
    CHECK(scenario.width_m==doctest::Approx(3000.0));
    CHECK(scenario.height_m==doctest::Approx(3000.0));
    CHECK(scenario.mobile_ids==gf::task10p10MobileIds(14));
    REQUIRE(scenario.mobile_positions.size()==14);
    CHECK(scenario.fixed_positions.at(100).isApprox(
        Eigen::Vector2d(1200.0,-50.0)));
    CHECK(scenario.fixed_positions.at(101).isApprox(
        Eigen::Vector2d(1500.0,-50.0)));
    CHECK(scenario.fixed_positions.at(102).isApprox(
        Eigen::Vector2d(1800.0,-50.0)));
    CHECK(scenario.mobile_positions.front().isApprox(
        Eigen::Vector2d(1380.0,10.0)));
    CHECK(scenario.mobile_positions.at(6).isApprox(
        Eigen::Vector2d(1320.0,130.0)));
    CHECK(scenario.mobile_positions.at(7).isApprox(
        Eigen::Vector2d(1620.0,10.0)));
    CHECK(scenario.mobile_positions.back().isApprox(
        Eigen::Vector2d(1680.0,130.0)));
    CHECK(scenario.initial_topology.size()==28);

    const auto branches=gf::task10p11pStandardCoverageBranches();
    REQUIRE(branches.size()==2);
    CHECK(branches[0].leader==7);
    CHECK(branches[1].leader==14);
    CHECK(branches[0].coverage_origin.isApprox(
        Eigen::Vector2d(1500.0,-50.0)));
    CHECK(branches[1].coverage_origin.isApprox(
        Eigen::Vector2d(1500.0,-50.0)));
}

TEST_CASE("Task 10.11p preregistration is explicit and not inferred from results") {
    const auto raw=json::parse(std::ifstream(
        std::string(PROJECT_ROOT)+
        "/config/grand_finale/task10p11p_standard_scale_gate.json"));
    CHECK(raw.at("cbf2026_source_commit")==
          gf::task10p11pCbf2026SourceCommit());
    CHECK(raw.at("map_m")==json::array({3000.0,3000.0}));
    CHECK(raw.at("grid_spacing_m")==doctest::Approx(10.0));
    CHECK(raw.at("valid_cells")==90000);
    CHECK(raw.at("timeout_s")==doctest::Approx(500.0));
    CHECK(raw.at("prefix_horizon_s")==doctest::Approx(60.0));
    CHECK(raw.at("plant_speed_norm_limit_mps")==doctest::Approx(30.0));
    CHECK(raw.at("plant_speed_facet_count")==64);
    CHECK(raw.at("operational_speed_matrix_status")=="paused_not_run");
    CHECK(raw.at("predictive_policies").size()==3);
    CHECK(raw.at("collision_distance_m")==doctest::Approx(10.0));
    CHECK(raw.at("reference_keep_m")==doctest::Approx(850.0));
    CHECK(raw.at("reference_add_m")==doctest::Approx(849.0));
}

TEST_CASE("Task 10.11p formal settings freeze scale yaw footprint and hard model") {
    const auto scenario=gf::task10p11pStandardCoastalScenario();
    const auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    CHECK(settings.at("world").at("spacing")==doctest::Approx(10.0));
    CHECK(settings.at("initial").at("yawDeg")==doctest::Approx(90.0));
    CHECK(settings.at("searching").at("front-sector").at("outer-radius")==
          doctest::Approx(400.0));
    CHECK(settings.at("searching").at("front-sector").at("half-angle-deg")==
          doctest::Approx(60.0));

    const auto config=gf::task10p11pAdapterConfig(
        gf::SolverProfile::Gurobi,30.0,
        gf::GammaFeedbackSelectionMode::LeastIntervention,0.0);
    CHECK(config.dt_s==doctest::Approx(0.1));
    CHECK(config.acceleration_half_box==doctest::Approx(4.0));
    CHECK(config.speed_limit_mps==doctest::Approx(30.0));
    CHECK(config.plant_speed_facet_count==64);
    CHECK(config.collision_distance_m==doctest::Approx(10.0));
    CHECK(config.reference_distance_m==doctest::Approx(850.0));
    CHECK(config.add_reference_distance_m==doctest::Approx(849.0));
    CHECK(config.boundary.policy==gf::BoundaryPolicy::None);
}

TEST_CASE("Task 10.11p denominator and stage-zero geometry are explicit") {
    const auto audit=gf::task10p11pAnalyticStageZeroAudit();
    CHECK(audit.valid_cell_count==90000);
    CHECK(audit.minimum_mobile_mobile_distance_m>10.0);
    CHECK(audit.minimum_mobile_fixed_distance_m>10.0);
    CHECK(audit.maximum_initial_reference_distance_m<849.0);
    CHECK(audit.initial_truth_fraction>0.0);
    CHECK(audit.initial_truth_fraction<0.95);
}

TEST_CASE("Task 10.11p formal Gurobi stage zero passes every frozen gate") {
    const auto scenario=gf::task10p11pStandardCoastalScenario();
    auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm,scenario.mobile_ids,scenario.fixed_positions,
        scenario.initial_topology,
        gf::task10p11pAdapterConfig(
            gf::SolverProfile::Gurobi,30.0,
            gf::GammaFeedbackSelectionMode::LeastIntervention,0.0));
    const auto initialized=adapter.initializeStageZero();
    INFO(initialized.reason);
    REQUIRE(initialized.initialized);
    CHECK(swarm.gridWorldGroundTruth.validCount==90000);
    CHECK(initialized.truth_coverage>0.0);
    CHECK(initialized.truth_coverage<0.95);
    const auto information=adapter.currentReferenceAudit();
    CHECK(information.minimum_effective_reference_count>=2);
    CHECK(information.minimum_robust_fim_cone_lower_bound>1.0e-6);
    CHECK(information.maximum_posterior_eigenvalue<=
          adapter.config().maximum_posterior_eigenvalue_m2);
    CHECK(information.minimum_range_aoi_margin_s>=0.0);
    const auto rows=adapter.currentSnapshotHardRows(
        adapter.supervisor().topology());
    CHECK(std::count_if(rows.begin(),rows.end(),[](const auto& row) {
        return row.kind==gf::CanonicalHardRowKind::PlantSpeedAppliedControl;
    })==64*scenario.mobile_ids.size());
    CHECK(std::none_of(rows.begin(),rows.end(),[](const auto& row) {
        return row.kind==gf::CanonicalHardRowKind::SpeedLimit;
    }));
    for (const auto owner:scenario.mobile_ids) {
        const auto exact=gf::evaluateProgressCompatibility(
            rows,owner,Eigen::Vector2d::Zero(),4.0,
            {std::numeric_limits<double>::max(),0.0,1e-10,true});
        INFO("owner=",owner," reason=",exact.reason);
        CHECK(exact.polytope_nonempty);
    }
}

TEST_CASE("Nominal-only causal step preserves speed and input but never claims safety") {
    const auto scenario=gf::task10p11pStandardCoastalScenario();
    auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm,scenario.mobile_ids,scenario.fixed_positions,
        scenario.initial_topology,
        gf::task10p11pAdapterConfig(
            gf::SolverProfile::Gurobi,30.0,
            gf::GammaFeedbackSelectionMode::LeastIntervention,0.0));
    REQUIRE(adapter.initializeStageZero().initialized);
    std::map<gf::NodeId,Eigen::Vector2d> nominal;
    std::map<gf::NodeId,double> yaw;
    for (const auto owner:scenario.mobile_ids) {
        nominal[owner]=Eigen::Vector2d(4.0,4.0);
        yaw[owner]=0.0;
    }
    const auto step=adapter.stepNominalOnlyCausalDiagnostic(nominal,yaw);
    INFO(step.reason);
    REQUIRE(step.advanced);
    CHECK_FALSE(step.safety_authorized);
    CHECK(step.restricted_row_count==(64+4)*scenario.mobile_ids.size());
    CHECK(step.full_hard_row_count>step.restricted_row_count);
    CHECK(step.applied_controls.size()==scenario.mobile_ids.size());
    CHECK(step.current_gamma.size()==scenario.mobile_ids.size());
    CHECK(step.local_maximum_predicted_gamma.size()==scenario.mobile_ids.size());
    CHECK(step.minimum_restricted_residual>=
          -adapter.config().residual_tolerance);
    CHECK(step.minimum_plant_speed_applied_control_residual>=
          -adapter.config().residual_tolerance);
    CHECK(std::isfinite(step.minimum_full_hard_residual));
    CHECK(step.truth_coverage>0.0);
}

TEST_CASE("Standard nominal controller uses production leader policy and estimator state") {
    const auto scenario=gf::task10p11pStandardCoastalScenario();
    auto fixture=gf::makeTask10p11pNominalFixture(
        gf::SolverProfile::Gurobi,30.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto first=fixture->controller.advance();
    INFO(first.diagnostic.reason);
    REQUIRE(first.diagnostic.advanced);
    CHECK(first.allocation.valid);
    CHECK(first.allocation.request_digest!=0);
    CHECK(first.targets.size()==scenario.mobile_ids.size());
    CHECK(first.targets.at(7).x_index>=0);
    CHECK(first.targets.at(14).x_index>=0);
    CHECK(first.target_epoch==1);
    CHECK(first.diagnostic.safety_authorized==false);
}

TEST_CASE("Nominal causal metrics latch coverage and preserve unsafe counts") {
    gf::Task10p11pNominalMetrics metrics;
    metrics.observe({0.1,0.94,11.0,12.0,29.0,2.0,-1.0,-0.5,4,0.1,0.2,
        28.9,29.1});
    CHECK_FALSE(metrics.t95_s.has_value());
    CHECK_FALSE(metrics.t100_s.has_value());
    CHECK(metrics.collision_violation_pair_ticks==0);
    metrics.observe({0.2,0.96,9.0,12.0,31.0,-0.1,-2.0,-1.0,5,0.2,0.1,
        29.2,29.5});
    REQUIRE(metrics.t95_s.has_value());
    CHECK(*metrics.t95_s==doctest::Approx(0.2));
    CHECK(metrics.collision_violation_pair_ticks==1);
    CHECK(metrics.speed_violation_ticks==1);
    metrics.observe({0.3,1.0,8.0,9.0,32.0,-0.2,-3.0,-2.0,6,0.3,0.0,
        29.4,29.8});
    REQUIRE(metrics.t100_s.has_value());
    CHECK(*metrics.t100_s==doctest::Approx(0.3));
    metrics.observe({0.4,1.0,7.0,8.0,33.0,-0.3,-4.0,-3.0,7,0.4,-0.1,
        29.6,30.1});
    CHECK(*metrics.t95_s==doctest::Approx(0.2));
    CHECK(*metrics.t100_s==doctest::Approx(0.3));
    CHECK(metrics.minimum_truth_mobile_distance_m==doctest::Approx(7.0));
    CHECK(metrics.minimum_truth_mobile_fixed_distance_m==doctest::Approx(8.0));
    CHECK(metrics.first_current_gamma_negative_s==doctest::Approx(0.2));
    CHECK(metrics.first_local_predicted_gamma_negative_s==doctest::Approx(0.1));
    CHECK(metrics.maximum_covered_cells==7);
    metrics.observe({0.5,1.0,7.0,8.0,29.0,-0.3,
        std::numeric_limits<double>::quiet_NaN(),-3.0,7,0.1,0.0,29.0,29.2});
    CHECK(metrics.local_predicted_gamma_invalid_ticks==1);
    CHECK(metrics.maximum_estimated_speed_mps==doctest::Approx(29.6));
    CHECK(metrics.maximum_tube_robust_speed_mps==doctest::Approx(30.1));
}
