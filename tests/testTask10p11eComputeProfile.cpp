#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11ComputeProfile.hpp"

TEST_CASE("Task 10.11e compute profile preserves calls and deterministic summaries") {
    gf::Task10p11ComputeProfile profile;
    profile.record(gf::Task10p11ComputePhase::CanonicalRowRebuild,0.30,false);
    profile.record(gf::Task10p11ComputePhase::CanonicalRowRebuild,0.10,true);
    profile.record(gf::Task10p11ComputePhase::CanonicalRowRebuild,0.20,true);
    profile.record(gf::Task10p11ComputePhase::DiagnosticSerialization,0.04,true);

    const auto rows=profile.summary(
        gf::Task10p11ComputePhase::CanonicalRowRebuild);
    CHECK(rows.calls==3);
    CHECK(rows.cold_calls==1);
    CHECK(rows.steady_calls==2);
    CHECK(rows.total_s==doctest::Approx(0.60));
    CHECK(rows.median_s==doctest::Approx(0.20));
    CHECK(rows.p95_s==doctest::Approx(0.30));
    CHECK(rows.maximum_s==doctest::Approx(0.30));
    CHECK(profile.summary(
        gf::Task10p11ComputePhase::DiagnosticSerialization).calls==1);
}

TEST_CASE("Task 10.11e profiler names every frozen phase separately") {
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::CandidateBundleConstruction)==
        "candidate_bundle_construction");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::EstimatorPropagation)==
        "estimator_propagation");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::CanonicalRowRebuild)==
        "canonical_row_rebuild");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::ExactHardProjection)==
        "exact_hard_projection");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::CurrentGamma)=="current_gamma");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::PredictedGamma)=="predicted_gamma");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::RobustQpSetup)=="robust_qp_setup");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::RobustQpSolve)=="robust_qp_solve");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::ResidualTokenAudit)==
        "residual_token_audit");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::DiagnosticSerialization)==
        "diagnostic_serialization");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::SolverInitialization)==
        "solver_initialization");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::SolverModelUpdate)==
        "solver_model_update");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::GridWorldTarget)==
        "gridworld_target");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::CurrentCanonicalRowRebuild)==
        "current_canonical_row_rebuild");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::PredictedCanonicalRowRebuild)==
        "predicted_canonical_row_rebuild");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::FinalQp)=="final_qp");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::InformationAudit)==
        "information_audit");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::TruthOnlyAudit)==
        "truth_only_audit");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::OnlineEstimator)=="online_estimator");
    CHECK(gf::task10p11ComputePhaseName(
        gf::Task10p11ComputePhase::PlantPreflightZoh)==
        "plant_preflight_zoh");
}
