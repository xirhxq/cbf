#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11qStandardSafetyOn.hpp"

TEST_CASE("Task 10.11q policies share frozen physics and differ only in gamma selection") {
    const auto least=gf::task10p11qAdapterConfig(
        gf::SolverProfile::Gurobi,
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    const auto maximum=gf::task10p11qAdapterConfig(
        gf::SolverProfile::Gurobi,
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,std::nullopt);
    CHECK(least.acceleration_half_box==doctest::Approx(4.0));
    CHECK(least.speed_limit_mps==doctest::Approx(30.0));
    CHECK(least.plant_speed_facet_count==64);
    CHECK(least.collision_distance_m==doctest::Approx(10.0));
    CHECK(least.reference_distance_m==doctest::Approx(850.0));
    CHECK(least.add_reference_distance_m==doctest::Approx(849.0));
    CHECK(least.dt_s==doctest::Approx(0.1));
    CHECK(least.boundary.policy==gf::BoundaryPolicy::None);
    CHECK(least.gamma_feedback_selection==
          gf::GammaFeedbackSelectionMode::LeastIntervention);
    REQUIRE(least.predictive_gamma_tau_mps2.has_value());
    CHECK(*least.predictive_gamma_tau_mps2==doctest::Approx(14.0));
    CHECK(maximum.gamma_feedback_selection==
          gf::GammaFeedbackSelectionMode::MaximumPredictedMargin);
    CHECK_FALSE(maximum.predictive_gamma_tau_mps2.has_value());
    auto least_without_policy=least;
    auto maximum_without_policy=maximum;
    least_without_policy.gamma_feedback_selection=
        gf::GammaFeedbackSelectionMode::DiagnosticsOnly;
    maximum_without_policy.gamma_feedback_selection=
        gf::GammaFeedbackSelectionMode::DiagnosticsOnly;
    least_without_policy.predictive_gamma_tau_mps2.reset();
    maximum_without_policy.predictive_gamma_tau_mps2.reset();
    CHECK(gf::task10p11qConfigDigest(least_without_policy)==
          gf::task10p11qConfigDigest(maximum_without_policy));
}

TEST_CASE("Standard proposed fixture starts formal safety-on control at stage zero") {
    auto fixture=gf::makeTask10p11qSafetyOnFixture(
        gf::SolverProfile::Gurobi,
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,std::nullopt);
    const auto initialized=fixture->adapter.initializeStageZero();
    INFO(initialized.reason);
    REQUIRE(initialized.initialized);
    const auto step=fixture->controller.advance();
    INFO(step.reason);
    REQUIRE(step.control.step.advanced);
    CHECK(step.control.step.certified_control_count==14);
    CHECK(step.control.step.minimum_hard_residual>=
          -fixture->adapter.config().residual_tolerance);
    CHECK(step.control.step.minimum_plant_speed_applied_control_residual>=
          -fixture->adapter.config().residual_tolerance);
    CHECK(step.control.step.applied_controls.size()==14);
    CHECK(step.simulated_time_s==doctest::Approx(0.1));
    CHECK(step.topology_audit_due);
    CHECK_FALSE(step.stopped_at_t100);
    const auto information=fixture->adapter.currentReferenceAudit();
    CHECK(information.minimum_effective_reference_owner!=0);
    CHECK(information.minimum_fim_owner!=0);
    CHECK(information.minimum_robust_fim_owner!=0);
    CHECK(information.maximum_posterior_owner!=0);
    CHECK(information.minimum_range_aoi_owner!=0);
    CHECK_FALSE(information.minimum_range_aoi_edge.empty());
}

TEST_CASE("Safety-on metric latch stops at first truth T100 without settling") {
    gf::Task10p11qRunMetrics metrics;
    CHECK_FALSE(metrics.observeCoverage(12.0,0.949));
    CHECK_FALSE(metrics.t95_s.has_value());
    CHECK_FALSE(metrics.observeCoverage(12.1,0.95));
    REQUIRE(metrics.t95_s.has_value());
    CHECK(*metrics.t95_s==doctest::Approx(12.1));
    CHECK(metrics.observeCoverage(15.3,1.0));
    REQUIRE(metrics.t100_s.has_value());
    CHECK(*metrics.t100_s==doctest::Approx(15.3));
    CHECK(metrics.observeCoverage(15.4,1.0));
    CHECK(*metrics.t100_s==doctest::Approx(15.3));
}

TEST_CASE("Production topology request is built from frozen runtime and target ledger") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalBindingActiveScenario(),
        gf::SolverProfile::Gurobi,[] {
            gf::BoundaryPolicyConfig boundary;
            boundary.policy=gf::BoundaryPolicy::None;
            return boundary;
        }());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController coverage(
        fixture->swarm,fixture->adapter);
    REQUIRE(coverage.advance().step.advanced);
    std::map<gf::NodeId,Eigen::Vector2d> raw;
    for (const auto& [owner,target]:coverage.committedTargets())
        raw[owner]=target.center;
    raw.at(14)={300.0,1700.0};

    const auto built=gf::task10p11qBuildProposalRequest(
        fixture->adapter,raw,coverage.targetEpoch(),1);
    INFO(built.reason);
    REQUIRE(built.required);
    REQUIRE(built.request.has_value());
    const auto runtime=fixture->adapter.runtimeSnapshot();
    CHECK(built.request->transition.old_edges==runtime.topology);
    CHECK(built.request->transition.estimator_version==runtime.estimator_token);
    CHECK(built.request->transition.topology_version==runtime.topology_token);
    CHECK(built.request->transition.raw_ledger_version==coverage.targetEpoch());
    CHECK(built.owner!=0);
    REQUIRE(built.removal.has_value());
    CHECK(built.removal->owner==built.owner);
    CHECK(built.request->topology.required_edges.size()==
          runtime.topology.size()-1);
    CHECK(std::all_of(built.request->topology.required_edges.begin(),
        built.request->topology.required_edges.end(),[&](const auto& edge) {
            return edge.id()!=built.removal->id() &&
                std::any_of(runtime.topology.begin(),runtime.topology.end(),
                    [&](const auto& old) { return old.id()==edge.id(); });
        }));
    CHECK(built.request->topology.eligible_edges.size()>
          runtime.topology.size()-1);
    CHECK(std::none_of(built.request->topology.eligible_edges.begin(),
        built.request->topology.eligible_edges.end(),[&](const auto& edge) {
            return edge.id()==built.removal->id();
        }));
}

TEST_CASE("Target-cone pressure triggers a single replacement before distance pressure") {
    auto fixture=gf::makeTask10p11qSafetyOnFixture(
        gf::SolverProfile::Gurobi,
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,std::nullopt);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    REQUIRE(fixture->coverage_controller.advance().step.advanced);
    std::map<gf::NodeId,Eigen::Vector2d> raw;
    for (const auto& [owner,target]:fixture->coverage_controller.committedTargets())
        raw[owner]=target.center;
    raw.at(8)={2100.0,-50.0};

    const auto built=gf::task10p11qBuildProposalRequest(
        fixture->adapter,raw,fixture->coverage_controller.targetEpoch(),1);
    INFO(built.reason);
    REQUIRE(built.required);
    CHECK(built.owner==8);
    REQUIRE(built.removal.has_value());
    CHECK(built.removal->owner==8);
    REQUIRE(built.request.has_value());
    CHECK(built.request->topology.required_edges.size()==
          fixture->adapter.supervisor().topology().size()-1);
}
