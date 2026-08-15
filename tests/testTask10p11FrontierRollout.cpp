#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/FrontierViabilityRollout.hpp"

gf::FrontierRolloutRequest rolloutRequest(gf::SolverProfile profile) {
    gf::FrontierRolloutRequest request;
    request.profile=profile;
    request.hard_row_request.mobile_ids={1};
    request.hard_row_request.states[1]={{5,5},{0,0},{0,0}};
    request.hard_row_request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850,0,1,1,1,0};
    request.hard_row_request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        0.1,0,1,1,1,0};
    request.hard_row_request.acceleration_half_box=0.4;
    request.hard_row_request.require_snapshot_robust_rows=true;
    request.hard_row_request.workspace_facets={
        {"x-upper",{1,0},100},{"x-lower",{-1,0},0},
        {"y-upper",{0,1},100},{"y-lower",{0,-1},0}};
    request.hard_row_request.workspace_snapshot_tubes[1]={0.1,0.2};
    request.targets={{1,{20,5}}};
    request.position_gain=0.4;
    request.velocity_gain=0.8;
    request.dt_s=0.1;
    request.cycles=5;
    request.estimator_snapshot.mobile_ids={1};
    request.estimator_snapshot.mean=Eigen::VectorXd::Zero(4);
    request.estimator_snapshot.mean<<5.0,5.0,0.0,0.0;
    request.estimator_snapshot.covariance=Eigen::MatrixXd::Zero(4,4);
    request.estimator_snapshot.covariance.diagonal()<<0.01,0.01,0.04,0.04;
    request.estimator_acceleration_variance=0.0;
    request.gamma_feedback_config.acceleration_half_box=0.4;
    request.gamma_feedback_config.homotopy_segments=2;
    request.gamma_feedback_config.selection_mode=
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin;
    const auto hard_template=request.hard_row_request;
    request.canonical_request_builder=[hard_template](
        const gf::JointEstimateSnapshot& snapshot) {
        auto hard=hard_template;
        const Eigen::Vector4d state=snapshot.mean.segment<4>(0);
        hard.states[1]={{state.x(),state.y()},state.tail<2>(),
                        Eigen::Vector2d::Zero()};
        const Eigen::Matrix2d position_covariance=
            snapshot.covariance.block<2,2>(0,0);
        const Eigen::Matrix2d velocity_covariance=
            snapshot.covariance.block<2,2>(2,2);
        hard.workspace_snapshot_tubes[1]={
            std::sqrt(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                          position_covariance).eigenvalues().maxCoeff()),
            std::sqrt(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                          velocity_covariance).eigenvalues().maxCoeff())};
        return hard;
    };
    return request;
}

TEST_CASE("Task 10.11 rollout spans the full allocation epoch and expands tube") {
    for (const auto profile : {gf::SolverProfile::OpenSource,gf::SolverProfile::Gurobi}) {
        const auto result=gf::evaluateFrontierRollout(rolloutRequest(profile));
        INFO("reason=",result.reason," failed_cycle=",result.failed_cycle,
             " source=",result.first_negative_source,
             " trace=",result.braking_trace.size());
        REQUIRE(result.accepted);
        CHECK(result.completed_cycles == 5);
        CHECK(result.duration_s == doctest::Approx(0.5));
        CHECK(result.final_request.workspace_snapshot_tubes.at(1)
            .position_radius_m == doctest::Approx(std::sqrt(0.02)));
        CHECK(result.final_request.workspace_snapshot_tubes.at(1)
            .velocity_radius_mps == doctest::Approx(0.2));
        CHECK(result.minimum_robust_residual >= -1e-7);
        CHECK(result.gamma_policy_work.policy_evaluations == 5);
        CHECK(result.gamma_policy_work.canonical_row_rebuilds == 20);
        CHECK(result.gamma_policy_work.exact_gamma_solves == 20);
        CHECK(result.qp_solves == 5);
        CHECK(result.policy_trace.size() == 5);
        for (const auto& policy : result.policy_trace) {
            CHECK(policy.selected_controls.size() == 1);
            CHECK(policy.selections.at(1).valid);
            CHECK(policy.current_rows.size() == 8);
        }
        REQUIRE(result.braking_trace.size() == 5);
        for (std::size_t cycle=0;cycle<result.braking_trace.size();++cycle) {
            CHECK(result.braking_trace[cycle].cycle == cycle);
            CHECK(result.braking_trace[cycle].snapshot.hard_polytope_nonempty);
            CHECK(result.braking_trace[cycle].snapshot.snapshot_braking_admissible);
        }
        CHECK(result.first_negative_source.empty());
    }
}

TEST_CASE("Rollout and formal batch policy agree on the same snapshot and target") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.cycles=1;
    const auto snapshot=request.estimator_snapshot;
    const std::map<gf::NodeId,Eigen::Vector2d> nominal{{1,{0.4,0.0}}};
    gf::CanonicalGammaFeedbackEvaluationContext context;
    const auto formal=gf::evaluateCanonicalGammaFeedbackBatch(
        snapshot,nominal,request.gamma_feedback_config,request.dt_s,
        request.estimator_acceleration_variance,
        request.canonical_request_builder,context);
    const auto rollout=gf::evaluateFrontierRollout(request);

    REQUIRE(formal.valid);
    REQUIRE(rollout.accepted);
    REQUIRE(rollout.policy_trace.size()==1);
    const auto& predicted=rollout.policy_trace.front();
    REQUIRE(predicted.valid);
    CHECK(predicted.current_rows.size()==formal.current_rows.size());
    for (std::size_t index=0;index<formal.current_rows.size();++index) {
        CHECK(predicted.current_rows[index].id==formal.current_rows[index].id);
        CHECK(predicted.current_rows[index].constant==
            doctest::Approx(formal.current_rows[index].constant));
        CHECK(predicted.current_rows[index].control_coefficient.isApprox(
            formal.current_rows[index].control_coefficient,1e-12));
    }
    CHECK(predicted.selected_controls.at(1).isApprox(
        formal.selected_controls.at(1),1e-12));
    CHECK(predicted.stages.at(1).current_gamma==
        doctest::Approx(formal.stages.at(1).current_gamma));
    CHECK(predicted.selections.at(1).selected_predicted_gamma==
        doctest::Approx(formal.selections.at(1).selected_predicted_gamma));
}

TEST_CASE("Tau zero is only an explicit hard-feasibility boundary mode") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.cycles=1;
    request.gamma_feedback_config.selection_mode=
        gf::GammaFeedbackSelectionMode::LeastIntervention;
    request.gamma_feedback_config.predictive_tau_mps2=0.0;
    const auto result=gf::evaluateFrontierRollout(request);
    REQUIRE(result.accepted);
    REQUIRE(result.policy_trace.size()==1);
    REQUIRE(result.policy_trace.front().selections.at(1).valid);
    CHECK(result.policy_trace.front().selections.at(1)
        .selected_predicted_gamma>=-1e-10);
}

TEST_CASE("Task 10.11 rollout rejects an empty hard polytope without advancing") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.hard_row_request.states[1].position={99.9,5};
    request.hard_row_request.states[1].velocity={1,0};
    const auto result=gf::evaluateFrontierRollout(request);
    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "hard_polytope_empty");
    CHECK(result.completed_cycles == 0);
    CHECK(result.failed_cycle == 0);
    REQUIRE(result.braking_trace.size() == 1);
    CHECK(result.braking_trace.front().snapshot.reason == "hard_polytope_empty");
}

TEST_CASE("Task 10.11b rollout rejects snapshot braking inadmissibility before applying control") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.hard_row_request.states[1].position={95.4,5.0};
    request.hard_row_request.states[1].velocity={2.0,0.0};
    const auto result=gf::evaluateFrontierRollout(request);
    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "snapshot_braking_inadmissible");
    CHECK(result.completed_cycles == 0);
    CHECK(result.failed_cycle == 0);
    REQUIRE(result.braking_trace.size() == 1);
    CHECK(result.first_negative_source == "workspace:1:x-upper");
    CHECK(result.braking_trace.front().snapshot.minimum_braking_slack_m ==
        doctest::Approx(-1.55));
}

TEST_CASE("Task 10.11b rollout records horizon insufficiency without claiming terminal safety") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.hard_row_request.states[1].position={10.0,5.0};
    request.hard_row_request.states[1].velocity={1.0,0.0};
    request.targets[1]={20.0,5.0};
    const auto result=gf::evaluateFrontierRollout(request);
    REQUIRE_FALSE(result.braking_trace.empty());
    CHECK(result.braking_trace.front().snapshot.reason ==
        "braking_horizon_insufficient");
    CHECK(result.braking_trace.front().snapshot.snapshot_braking_admissible);
}
