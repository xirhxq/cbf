#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

#include <fstream>
#include <cstring>

namespace {

json settings4p2() {
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
    settings["num"] = 4;
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0, 1}};
    settings["bases"] = {{4.0, 4.0}, {4.0, 16.0}};
    settings["initial"]["position"]["positions"] = {
        {7.0, 7.0}, {7.0, 13.0}, {12.0, 7.0}, {12.0, 13.0}};
    settings["initial"]["velocity"]["values"] = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    settings["world"]["spacing"] = 1.0;
    settings["searching"]["downward"]["radius"] = 2.0;
    return settings;
}

std::vector<gf::DirectedEdge> topology() {
    return {{10, 1}, {11, 1}, {10, 2}, {1, 2},
            {11, 3}, {1, 3}, {2, 4}, {3, 4}};
}

gf::GrandFinaleSwarmAdapterConfig adapterConfig(gf::SolverProfile profile) {
    gf::GrandFinaleSwarmAdapterConfig config;
    config.solver_profile = profile;
    config.dt_s = 0.1;
    config.acceleration_half_box = 0.4;
    config.sensor_radius_m = 2.0;
    // Historical Task 10.5 mechanism fixture.  The formal Task 10.11g model
    // uses 10 m and never relies on this value as a default.
    config.collision_distance_m = 0.1;
    return config;
}

}  // namespace

TEST_CASE("Counter-based range field is reproducible and edge-order invariant") {
    const auto edge=gf::UndirectedEdge::canonical(2,101);
    const auto same=gf::range_noise_detail::sample(40009,37,edge,0.02);
    const auto repeated=gf::range_noise_detail::sample(40009,37,edge,0.02);
    const auto reversed=gf::range_noise_detail::sample(
        40009,37,gf::UndirectedEdge::canonical(101,2),0.02);
    CHECK(same.dropped==repeated.dropped);
    CHECK(std::memcmp(&same.standard_normal,&repeated.standard_normal,
        sizeof(double))==0);
    CHECK(same.dropped==reversed.dropped);
    CHECK(std::memcmp(&same.standard_normal,&reversed.standard_normal,
        sizeof(double))==0);
    const auto other_seed=gf::range_noise_detail::sample(
        40013,37,edge,0.02);
    CHECK(std::memcmp(&same.standard_normal,&other_seed.standard_normal,
        sizeof(double))!=0);
}

TEST_CASE("Counter-based range field preserves the initial no-dropout contract") {
    const auto edge=gf::UndirectedEdge::canonical(2,101);
    CHECK_FALSE(gf::range_noise_detail::sample(40009,0,edge,1.0).dropped);
    CHECK(gf::range_noise_detail::sample(40009,1,edge,1.0).dropped);
    const auto finite=gf::range_noise_detail::sample(40009,1,edge,0.0);
    CHECK_FALSE(finite.dropped);
    CHECK(std::isfinite(finite.standard_normal));
}

TEST_CASE("GrandFinale adapter rejects an omitted formal collision distance") {
    json settings=settings4p2();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapterConfig missing;
    CHECK_THROWS_WITH_AS(
        gf::GrandFinaleSwarmAdapter(
            swarm,{1,2,3,4},{{10,{4.0,4.0}},{11,{4.0,16.0}}},
            topology(),missing),
        "invalid GrandFinale safety configuration",std::invalid_argument);
}

TEST_CASE("Formal Swarm adapter closes SEARCH HOLD RETREAT REFORM and UNION QP flows") {
    json settings = settings4p2();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), adapterConfig(gf::SolverProfile::OpenSource));

    const auto search = adapter.step();
    REQUIRE(search.advanced);
    CHECK(search.mode == gf::SupervisorMode::Search);
    CHECK(search.certified_control_count == 4);
    CHECK(search.minimum_hard_residual >= -1e-7);
    CHECK(search.estimator_version_after > search.estimator_version_before);
    CHECK(search.gamma_policy_work.policy_evaluations == 1);
    CHECK(search.gamma_policy_work.canonical_row_rebuilds == 37);
    CHECK(search.gamma_policy_work.exact_gamma_solves == 40);
    REQUIRE(search.gamma_feedback.size() == 4);
    for (const auto& [owner, diagnostic] : search.gamma_feedback) {
        (void)owner;
        CHECK(std::isfinite(diagnostic.current_gamma));
        CHECK(std::isfinite(diagnostic.nominal_predicted_gamma));
        CHECK(std::isfinite(
            diagnostic.maximum_margin_candidate_predicted_gamma));
        CHECK(std::isfinite(diagnostic.selected_predicted_gamma));
        CHECK_FALSE(diagnostic.intervened);
        CHECK(search.applied_controls.at(owner).isApprox(
            diagnostic.selected_nominal, 1e-5));
    }

    adapter.supervisor().requestReformation(
        swarm.robots.front()->runtime, true, true);
    const auto reform = adapter.step();
    REQUIRE(reform.advanced);
    CHECK(reform.mode == gf::SupervisorMode::Reform);

    REQUIRE(adapter.beginReplacement({11, 2}, {10, 2}));
    const auto union_step = adapter.step();
    REQUIRE(union_step.advanced);
    CHECK(union_step.mode == gf::SupervisorMode::Union);
    CHECK(union_step.certified_control_count == 4);
    CHECK(adapter.unionControlCycles() == 1);
    REQUIRE(adapter.finishReplacementAfterFreshCycle());
    CHECK(adapter.supervisor().mode() == gf::SupervisorMode::Search);
    CHECK(adapter.transitionStackSize() == 1);

    json retreat_settings = settings4p2();
    Swarm retreat_swarm(retreat_settings);
    gf::GrandFinaleSwarmAdapter retreat_adapter(
        retreat_swarm, {1, 2, 3, 4},
        {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}}, topology(),
        adapterConfig(gf::SolverProfile::OpenSource));
    REQUIRE(retreat_adapter.step().advanced);
    retreat_adapter.supervisor().requestRetreat(
        retreat_swarm.robots.front()->runtime, true);
    const auto retreat = retreat_adapter.step();
    REQUIRE(retreat.advanced);
    CHECK(retreat.mode == gf::SupervisorMode::Retreat);

    json hold_settings = settings4p2();
    Swarm hold_swarm(hold_settings);
    gf::GrandFinaleSwarmAdapter hold_adapter(
        hold_swarm, {1, 2, 3, 4},
        {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}}, topology(),
        adapterConfig(gf::SolverProfile::OpenSource));
    REQUIRE(hold_adapter.step().advanced);
    hold_adapter.supervisor().requestRetreat(
        hold_swarm.robots.front()->runtime, false);
    const auto hold = hold_adapter.step();
    REQUIRE(hold.advanced);
    CHECK(hold.mode == gf::SupervisorMode::Hold);
}

TEST_CASE("Formal coverage nominal depends on estimator state rather than simulator truth") {
    json settings = settings4p2();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), adapterConfig(gf::SolverProfile::OpenSource));
    const auto before = adapter.currentNominalControls();
    for (const auto& robot : swarm.robots) {
        robot->model->setStateVariable(
            "x", robot->model->getStateVariable("x") + 100.0);
        robot->model->setStateVariable(
            "y", robot->model->getStateVariable("y") - 50.0);
        robot->model->setStateVariable("vx", 9.0);
        robot->model->setStateVariable("vy", -7.0);
    }
    const auto after = adapter.currentNominalControls();
    REQUIRE(before.size() == after.size());
    for (const auto& [owner, nominal] : before)
        CHECK(after.at(owner).isApprox(nominal, 1e-12));
}

TEST_CASE("Explicit predictive mode remains separate from current canonical gamma") {
    json settings = settings4p2();
    Swarm swarm(settings);
    auto config = adapterConfig(gf::SolverProfile::OpenSource);
    config.gamma_feedback_selection =
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), config);
    const auto result = adapter.step();
    REQUIRE(result.advanced);
    REQUIRE(result.gamma_feedback.size() == 4);
    for (const auto& [owner, diagnostic] : result.gamma_feedback) {
        (void)owner;
        CHECK(std::isfinite(diagnostic.current_gamma));
        CHECK(std::isfinite(diagnostic.selected_predicted_gamma));
        CHECK(diagnostic.controllable_predicted_gamma_range >= -1e-12);
        CHECK(diagnostic.current_gamma != doctest::Approx(0.05));
        CHECK(diagnostic.current_gamma != doctest::Approx(0.1));
        CHECK(result.applied_controls.at(owner).isApprox(
            diagnostic.selected_nominal, 1e-5));
    }
}

TEST_CASE("Negative current canonical gamma fails closed before physical advancement") {
    json settings = settings4p2();
    Swarm swarm(settings);
    auto config = adapterConfig(gf::SolverProfile::OpenSource);
    config.collision_distance_m = 100.0;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), config);
    const double runtime = swarm.robots.front()->runtime;
    const auto estimator_version = adapter.estimator().version();
    const auto result = adapter.step();
    CHECK_FALSE(result.advanced);
    CHECK(result.reason == "current_gamma_negative");
    CHECK(swarm.robots.front()->runtime == doctest::Approx(runtime));
    CHECK(adapter.estimator().version() == estimator_version);
}

#ifdef ENABLE_GUROBI
TEST_CASE("Gurobi profile uses the same formal Swarm adapter control path") {
    json settings = settings4p2();
    settings["optimiser"] = "Gurobi";
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), adapterConfig(gf::SolverProfile::Gurobi));
    const auto result = adapter.step();
    REQUIRE(result.advanced);
    CHECK(result.certified_control_count == 4);
    CHECK(result.minimum_hard_residual >= -1e-7);
}
#endif

TEST_CASE("A heterogeneous-covariance new edge enlarges the union HOCBF tube") {
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(8, 8);
    covariance.block<2, 2>(0, 0) =
        1.0e-4 * Eigen::Matrix2d::Identity();
    covariance.block<2, 2>(4, 4) =
        0.04 * Eigen::Matrix2d::Identity();
    const gf::JointEstimateSnapshot snapshot{
        {1, 2}, mean, covariance, {{10, Eigen::Vector2d::Zero()}}};
    const double old_tube = gf::posteriorPairPositionTube(
        snapshot, {10, 1}, 3.0);
    const double union_new_edge_tube = gf::posteriorPairPositionTube(
        snapshot, {2, 1}, 3.0);
    CHECK(old_tube == doctest::Approx(0.03));
    CHECK(union_new_edge_tube == doctest::Approx(0.63));
    CHECK(union_new_edge_tube > old_tube);
}

TEST_CASE("Missing range history rejects switching without advancing topology or physics") {
    json settings = settings4p2();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), adapterConfig(gf::SolverProfile::OpenSource));
    const auto version = adapter.supervisor().topologyVersion();
    const double runtime = swarm.robots.front()->runtime;
    CHECK_FALSE(adapter.beginReplacement({11, 2}, {10, 2}));
    CHECK(adapter.supervisor().topologyVersion() == version);
    CHECK(swarm.robots.front()->runtime == doctest::Approx(runtime));
    CHECK(adapter.supervisor().mode() == gf::SupervisorMode::Search);
}

TEST_CASE("Certified primitive nominal uses the formal QP path and fails closed before physics") {
    json settings = settings4p2();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), adapterConfig(gf::SolverProfile::OpenSource));
    const std::map<gf::NodeId, Eigen::Vector2d> common{
        {1, {0.01, 0.0}}, {2, {0.01, 0.0}},
        {3, {0.01, 0.0}}, {4, {0.01, 0.0}}};
    const auto step = adapter.stepWithNominal(common);
    REQUIRE(step.advanced);
    REQUIRE(step.applied_controls.size() == 4);
    for (const auto& [id, control] : step.applied_controls) {
        CHECK((control - common.at(id)).norm() <= 1.0e-5);
    }

    const double runtime = swarm.robots.front()->runtime;
    const auto estimator_version = adapter.estimator().version();
    auto incomplete = common;
    incomplete.erase(4);
    const auto rejected = adapter.stepWithNominal(incomplete);
    CHECK_FALSE(rejected.advanced);
    CHECK(rejected.reason == "primitive_nominal_incomplete");
    CHECK(swarm.robots.front()->runtime == doctest::Approx(runtime));
    CHECK(adapter.estimator().version() == estimator_version);
}

TEST_CASE("Boundary policy isolates none and soft from hard canonical ledger") {
    json none_settings = settings4p2();
    json soft_settings = settings4p2();
    Swarm none_swarm(none_settings);
    Swarm soft_swarm(soft_settings);
    auto none_config=adapterConfig(gf::SolverProfile::OpenSource);
    none_config.boundary.policy=gf::BoundaryPolicy::None;
    auto soft_config=none_config;
    soft_config.boundary.policy=gf::BoundaryPolicy::SoftSearchRetention;
    gf::GrandFinaleSwarmAdapter none_adapter(
        none_swarm,{1,2,3,4},{{10,{4.0,4.0}},{11,{4.0,16.0}}},
        topology(),none_config);
    gf::GrandFinaleSwarmAdapter soft_adapter(
        soft_swarm,{1,2,3,4},{{10,{4.0,4.0}},{11,{4.0,16.0}}},
        topology(),soft_config);
    const auto none_request=none_adapter.currentSnapshotHardRowRequest(topology());
    const auto soft_request=soft_adapter.currentSnapshotHardRowRequest(topology());
    CHECK(none_request.workspace_facets.empty());
    CHECK(soft_request.workspace_facets.empty());
    const auto none_rows=gf::buildCanonicalHardRows(none_request);
    const auto soft_rows=gf::buildCanonicalHardRows(soft_request);
    REQUIRE(none_rows.size()==soft_rows.size());
    for (std::size_t index=0;index<none_rows.size();++index) {
        CHECK(none_rows[index].id==soft_rows[index].id);
        CHECK(none_rows[index].control_coefficient.isApprox(
            soft_rows[index].control_coefficient,1e-12));
        CHECK(none_rows[index].constant==doctest::Approx(
            soft_rows[index].constant).epsilon(1e-12));
    }
}

TEST_CASE("Formal adapter exposes snapshot robust workspace rows only in hard mode") {
    json settings = settings4p2();
    Swarm swarm(settings);
    auto config = adapterConfig(gf::SolverProfile::OpenSource);
    config.boundary.policy=gf::BoundaryPolicy::HardFlightBoundary;
    config.boundary.flight_polygon_source=gf::FlightPolygonSource::SearchPolygon;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), config);

    const auto request = adapter.currentSnapshotHardRowRequest(topology());
    CHECK(request.workspace_facets.size() == 4);
    CHECK(request.workspace_snapshot_tubes.size() == 4);
    const auto rows = gf::buildCanonicalHardRows(request);
    CHECK(std::count_if(rows.begin(),rows.end(),[](const auto& row) {
        return row.kind == gf::CanonicalHardRowKind::Workspace;
    }) == 16);
    for (const auto& row : rows) {
        if (row.kind != gf::CanonicalHardRowKind::Workspace) continue;
        CHECK(row.tube_provenance ==
              gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment);
        CHECK(row.position_uncertainty_reserve_m > 0.0);
        CHECK(row.velocity_uncertainty_reserve_mps > 0.0);
    }
}

TEST_CASE("Soft boundary nominal remains separate while applied hard residual passes") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        json settings=settings4p2();
        if (profile==gf::SolverProfile::Gurobi) settings["optimiser"]="Gurobi";
        Swarm swarm(settings);
        auto config=adapterConfig(profile);
        config.boundary.policy=gf::BoundaryPolicy::SoftSearchRetention;
        gf::GrandFinaleSwarmAdapter adapter(
            swarm,{1,2,3,4},{{10,{4.0,4.0}},{11,{4.0,16.0}}},
            topology(),config);
        const auto step=adapter.step();
        INFO(step.reason);
        REQUIRE(step.advanced);
        REQUIRE(step.boundary_soft_nominal.size()==4);
        CHECK(step.minimum_hard_residual>=-config.residual_tolerance);
        const auto rows=adapter.currentSnapshotHardRows(topology());
        CHECK(std::none_of(rows.begin(),rows.end(),[](const auto& row) {
            return row.kind==gf::CanonicalHardRowKind::Workspace;
        }));
        for (const auto& [owner,soft] : step.boundary_soft_nominal) {
            (void)owner;
            CHECK(soft.nominal_available);
            CHECK(soft.minimum_soft_residual>=-config.residual_tolerance);
        }
    }
}

TEST_CASE("Runtime snapshot retains estimator factors range state timer and transition bookkeeping") {
    json settings = settings4p2();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), adapterConfig(gf::SolverProfile::OpenSource));
    REQUIRE(adapter.step().advanced);

    const auto snapshot = adapter.runtimeSnapshot();
    CHECK(snapshot.runtime_s == doctest::Approx(0.1));
    CHECK(snapshot.estimate.mobile_ids == std::vector<gf::NodeId>{1, 2, 3, 4});
    CHECK(snapshot.dekf.propagation_factors.size() == 4);
    CHECK(snapshot.dekf.correlation_rows.size() == 4);
    CHECK(snapshot.range_links.size() == 14);
    CHECK(snapshot.mode == gf::SupervisorMode::Search);
    CHECK(snapshot.timer_since_supervisor_transition_s == doctest::Approx(0.1));
    CHECK_FALSE(snapshot.supervisor_transition_pending);
    CHECK_FALSE(snapshot.adapter_transition_pending);
    CHECK(snapshot.transition_stack_size == 0);
    CHECK(snapshot.union_control_cycles == 0);
    CHECK(snapshot.estimator_token == adapter.estimator().version());
    CHECK(snapshot.topology_token == adapter.supervisor().topologyVersion());
    CHECK(snapshot.freshness == gf::FreshnessRelation::NoPending);
}

TEST_CASE("Actual union hard rows use the higher-covariance new endpoint tube") {
    json low_settings = settings4p2();
    json high_settings = settings4p2();
    Swarm low_swarm(low_settings);
    Swarm high_swarm(high_settings);
    auto low_config = adapterConfig(gf::SolverProfile::OpenSource);
    auto high_config = low_config;
    high_config.initial_position_variance_m2 = 0.04;
    gf::GrandFinaleSwarmAdapter low_adapter(
        low_swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), low_config);
    gf::GrandFinaleSwarmAdapter high_adapter(
        high_swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), high_config);
    auto union_topology = topology();
    union_topology.push_back({11, 2});
    const auto low_rows = low_adapter.currentSnapshotHardRows(union_topology);
    const auto high_rows = high_adapter.currentSnapshotHardRows(union_topology);
    const auto low_row = std::find_if(low_rows.begin(), low_rows.end(), [](const auto& item) {
        return item.id == "reference:11->2:owner:2";
    });
    const auto high_row = std::find_if(high_rows.begin(), high_rows.end(), [](const auto& item) {
        return item.id == "reference:11->2:owner:2";
    });
    REQUIRE(low_row != low_rows.end());
    REQUIRE(high_row != high_rows.end());
    CHECK(high_row->barrier_h < low_row->barrier_h);
    CHECK(high_row->constant < low_row->constant);
}

TEST_CASE("Formal hard rows include velocity covariance in hdot and constant") {
    json low_settings = settings4p2();
    json high_settings = settings4p2();
    Swarm low_swarm(low_settings);
    Swarm high_swarm(high_settings);
    auto low_config = adapterConfig(gf::SolverProfile::OpenSource);
    auto high_config = low_config;
    high_config.initial_velocity_variance_m2 = 0.04;
    gf::GrandFinaleSwarmAdapter low_adapter(
        low_swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), low_config);
    gf::GrandFinaleSwarmAdapter high_adapter(
        high_swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), high_config);
    const auto low_rows = low_adapter.currentSnapshotHardRows(topology());
    const auto high_rows = high_adapter.currentSnapshotHardRows(topology());
    const auto find_row = [](const auto& rows, const std::string& id) {
        return std::find_if(rows.begin(), rows.end(), [&](const auto& row) {
            return row.id == id;
        });
    };
    const auto low = find_row(low_rows, "reference:10->1:owner:1");
    const auto high = find_row(high_rows, "reference:10->1:owner:1");
    REQUIRE(low != low_rows.end());
    REQUIRE(high != high_rows.end());
    REQUIRE(low->tube_provenance.has_value());
    CHECK(*low->tube_provenance ==
          gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment);
    CHECK(high->barrier_h == doctest::Approx(low->barrier_h));
    CHECK(high->barrier_hdot < low->barrier_hdot);
    CHECK(high->barrier_psi1 < low->barrier_psi1);
    CHECK(high->constant < low->constant);
}

TEST_CASE("Invalid snapshot robust tube fails before physics without zero control") {
    json settings = settings4p2();
    settings["initial"]["position"]["positions"][1] = {7.0, 7.0};
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), adapterConfig(gf::SolverProfile::OpenSource));
    const double runtime_before = swarm.robots.front()->runtime;
    CHECK_NOTHROW([&] {
        const auto result = adapter.step();
        CHECK_FALSE(result.advanced);
        CHECK(result.reason == "snapshot_robust_hard_row_invalid");
        CHECK(result.applied_controls.empty());
    }());
    CHECK(swarm.robots.front()->runtime == doctest::Approx(runtime_before));
}

TEST_CASE("Finite-tour shadow supports tighten the actual formal hard rows") {
    json baseline_settings = settings4p2();
    json shadow_settings = settings4p2();
    Swarm baseline_swarm(baseline_settings);
    Swarm shadow_swarm(shadow_settings);
    auto baseline_config = adapterConfig(gf::SolverProfile::OpenSource);
    auto shadow_config = baseline_config;
    baseline_config.uncertainty_sigma = 0.0;
    shadow_config.uncertainty_sigma = 0.0;
    shadow_config.certified_shadow_single_position_support_m = 0.2;
    shadow_config.certified_shadow_relative_position_support_m = 0.4;
    shadow_config.maximum_accepted_range_innovation_m = 0.1;
    gf::GrandFinaleSwarmAdapter baseline(
        baseline_swarm, {1, 2, 3, 4},
        {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}}, topology(),
        baseline_config);
    gf::GrandFinaleSwarmAdapter shadow(
        shadow_swarm, {1, 2, 3, 4},
        {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}}, topology(),
        shadow_config);
    const auto baseline_rows = baseline.currentSnapshotHardRows(topology());
    const auto shadow_rows = shadow.currentSnapshotHardRows(topology());
    const auto find_row = [](const auto& rows, const std::string& id) {
        return std::find_if(rows.begin(), rows.end(), [&](const auto& row) {
            return row.id == id;
        });
    };
    const auto baseline_reference = find_row(
        baseline_rows, "reference:10->1:owner:1");
    const auto shadow_reference = find_row(
        shadow_rows, "reference:10->1:owner:1");
    const auto baseline_collision = find_row(
        baseline_rows, "collision:1--2:owner:1");
    const auto shadow_collision = find_row(
        shadow_rows, "collision:1--2:owner:1");
    REQUIRE(baseline_reference != baseline_rows.end());
    REQUIRE(shadow_reference != shadow_rows.end());
    REQUIRE(baseline_collision != baseline_rows.end());
    REQUIRE(shadow_collision != shadow_rows.end());
    CHECK(shadow_reference->barrier_h < baseline_reference->barrier_h);
    CHECK(shadow_reference->constant < baseline_reference->constant);
    CHECK(shadow_collision->barrier_h < baseline_collision->barrier_h);
    CHECK(shadow_collision->constant < baseline_collision->constant);
}

TEST_CASE("Permuted mobile IDs are rejected before any certified control can be applied") {
    json settings = settings4p2();
    Swarm swarm(settings);
    CHECK_THROWS_AS(
        gf::GrandFinaleSwarmAdapter(
            swarm, {2, 1, 3, 4},
            {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}}, topology(),
            adapterConfig(gf::SolverProfile::OpenSource)),
        std::invalid_argument);
}

TEST_CASE("The formal adapter rejects a non-positive supervisor dwell time") {
    json settings = settings4p2();
    Swarm swarm(settings);
    auto config = adapterConfig(gf::SolverProfile::OpenSource);
    config.minimum_dwell_s = 0.0;
    CHECK_THROWS_AS(
        gf::GrandFinaleSwarmAdapter(
            swarm, {1, 2, 3, 4},
            {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}}, topology(), config),
        std::invalid_argument);
}

TEST_CASE("Effective reference initial-set audit rejects negative and incomplete half rows") {
    const auto make_row = [](
        std::string id, gf::NodeId owner, double h, double psi1) {
        gf::CanonicalHardRow row;
        row.id = std::move(id);
        row.kind = gf::CanonicalHardRowKind::ReferenceDistance;
        row.owner = owner;
        row.barrier_h = h;
        row.barrier_psi1 = psi1;
        return row;
    };
    std::vector<gf::CanonicalHardRow> fixed_rows{
        make_row("reference:10->1:owner:1", 1, 0.0, 0.0)};
    CHECK(gf::referenceEdgeInitialSetAudited(
        fixed_rows, {10, 1}, {1, 2}));
    fixed_rows.front().barrier_h = -1.0e-6;
    CHECK_FALSE(gf::referenceEdgeInitialSetAudited(
        fixed_rows, {10, 1}, {1, 2}));
    fixed_rows.front().barrier_h = 0.0;
    fixed_rows.front().barrier_psi1 = -1.0e-6;
    CHECK_FALSE(gf::referenceEdgeInitialSetAudited(
        fixed_rows, {10, 1}, {1, 2}));

    std::vector<gf::CanonicalHardRow> mobile_rows{
        make_row("reference:1->2:owner:2", 2, 0.0, 0.0)};
    CHECK_FALSE(gf::referenceEdgeInitialSetAudited(
        mobile_rows, {1, 2}, {1, 2}));
    mobile_rows.push_back(
        make_row("reference:1->2:owner:1", 1, 0.0, 0.0));
    CHECK(gf::referenceEdgeInitialSetAudited(
        mobile_rows, {1, 2}, {1, 2}));
    mobile_rows.back().barrier_h = -1.0e-6;
    CHECK_FALSE(gf::referenceEdgeInitialSetAudited(
        mobile_rows, {1, 2}, {1, 2}));
}
