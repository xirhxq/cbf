#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/BrakingSnapshotOracle.hpp"
#include "grand_finale/SnapshotRobustPairRow.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"
#include "grand_finale/Task10p11gReadinessFixture.hpp"
#include "grand_finale/Task10p11nGainSynthesis.hpp"

#include <limits>

namespace {

PairwiseSecondOrderState2D leftState(double closing_speed_mps) {
    return {{0.0,0.0},{0.5*closing_speed_mps,0.0},{0.0,0.0}};
}

PairwiseSecondOrderState2D rightState(
    double separation_m,double closing_speed_mps) {
    return {{separation_m,0.0},{-0.5*closing_speed_mps,0.0},{0.0,0.0}};
}

PairwiseSecondOrderRowSpec collisionSpec(
    double lambda1,double lambda2) {
    return {PairwiseSecondOrderBarrierKind::CollisionLower,
            10.0,0.0,1.0,lambda1,lambda2,0.0};
}

struct FormalGainRun {
    gf::LinearHocbfGains gains;
    bool initialized=false;
    bool advanced_through_horizon=false;
    double failure_time_s=std::numeric_limits<double>::infinity();
    std::string reason;
    double initial_minimum_h=std::numeric_limits<double>::infinity();
    double initial_minimum_psi1=std::numeric_limits<double>::infinity();
    double minimum_collision_h=std::numeric_limits<double>::infinity();
    double minimum_collision_psi1=std::numeric_limits<double>::infinity();
    double minimum_interval_collision_clearance_m=
        std::numeric_limits<double>::infinity();
    double minimum_current_gamma=std::numeric_limits<double>::infinity();
    double minimum_selected_successor_gamma=
        std::numeric_limits<double>::infinity();
    double minimum_applied_residual=std::numeric_limits<double>::infinity();
    double first_intervention_s=std::numeric_limits<double>::infinity();
    double first_hard_projection_s=std::numeric_limits<double>::infinity();
    double pair23_closing_at_first_hard_mps=
        std::numeric_limits<double>::quiet_NaN();
    double minimum_pair23_braking_slack_m=
        std::numeric_limits<double>::infinity();
    double first_negative_pair23_braking_s=
        std::numeric_limits<double>::infinity();
    std::string dominant_row_at_minimum_gamma;
    double final_truth_coverage=0.0;
};

FormalGainRun runFormalLeaderEasy(
    const gf::LinearHocbfGains& gains,gf::SolverProfile profile,
    int horizon_cycles=100) {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    const auto scenario=gf::task10p11hCoastalLeaderEasyScenario();
    auto settings=gf::task10p11gSwarmSettings(scenario,profile);
    Swarm swarm(settings);
    auto config=gf::task10p11gAdapterConfig(profile,boundary);
    config.collision_lambda1=gains.lambda1_per_s;
    config.collision_lambda2=gains.lambda2_per_s;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm,scenario.mobile_ids,scenario.fixed_positions,
        scenario.initial_topology,config);
    FormalGainRun result;
    result.gains=gains;
    const auto stage_zero=adapter.initializeStageZero();
    result.initialized=stage_zero.initialized;
    result.reason=stage_zero.reason;
    if (!stage_zero.initialized) return result;
    const auto initial=adapter.runtimeSnapshot();
    for (const auto& row:adapter.currentSnapshotHardRows(initial.topology)) {
        if (row.kind!=gf::CanonicalHardRowKind::Collision) continue;
        result.initial_minimum_h=std::min(
            result.initial_minimum_h,row.barrier_h);
        result.initial_minimum_psi1=std::min(
            result.initial_minimum_psi1,row.barrier_psi1);
    }
    result.minimum_collision_h=result.initial_minimum_h;
    result.minimum_collision_psi1=result.initial_minimum_psi1;
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    for (int cycle=0;cycle<horizon_cycles;++cycle) {
        const auto before=adapter.runtimeSnapshot();
        const double before_s=before.runtime_s;
        const auto before_request=adapter.snapshotHardRowRequest(
            before.estimate,before.topology);
        const auto before_rows=gf::buildCanonicalHardRows(before_request);
        for (const auto& row:before_rows) {
            if (row.kind!=gf::CanonicalHardRowKind::Collision) continue;
            result.minimum_collision_h=std::min(
                result.minimum_collision_h,row.barrier_h);
            result.minimum_collision_psi1=std::min(
                result.minimum_collision_psi1,row.barrier_psi1);
        }
        const auto pair23=gf::evaluateMobilePairBrakingRequestRows(
            before_request,before_rows,
            gf::UndirectedEdge::canonical(2,3));
        if (pair23.valid) {
            result.minimum_pair23_braking_slack_m=std::min(
                result.minimum_pair23_braking_slack_m,
                pair23.braking_slack_m);
            if (pair23.braking_slack_m<0.0 &&
                !std::isfinite(result.first_negative_pair23_braking_s))
                result.first_negative_pair23_braking_s=before_s;
        }
        const auto step=controller.advance();
        if (!step.step.advanced) {
            result.failure_time_s=before_s;
            result.reason=step.reason;
            result.final_truth_coverage=adapter.coverage().truthFraction();
            return result;
        }
        const auto no_measurement=gf::predictNoMeasurementSnapshot(
            before.estimate,step.step.applied_controls,config.dt_s,
            config.estimator_acceleration_variance);
        const auto predicted_request=adapter.snapshotHardRowRequest(
            no_measurement,before.topology);
        for (const auto& edge:before_request.collision_pairs) {
            const auto state=[&](gf::NodeId id)
                -> const PairwiseSecondOrderState2D& {
                return before_request.states.at(id);
            };
            const Eigen::Vector2d first_position(
                state(edge.first).position.x,state(edge.first).position.y);
            const Eigen::Vector2d second_position(
                state(edge.second).position.x,state(edge.second).position.y);
            const Eigen::Vector2d first_control=
                step.step.applied_controls.count(edge.first)
                ? step.step.applied_controls.at(edge.first)
                : Eigen::Vector2d::Zero();
            const Eigen::Vector2d second_control=
                step.step.applied_controls.count(edge.second)
                ? step.step.applied_controls.at(edge.second)
                : Eigen::Vector2d::Zero();
            const double tube=std::max(
                before_request.collision_snapshot_tubes.at(edge.id())
                    .position_radius_m,
                predicted_request.collision_snapshot_tubes.at(edge.id())
                    .position_radius_m);
            const auto interval=gf::auditExactZohPairInterval(
                first_position-second_position,
                state(edge.first).velocity-state(edge.second).velocity,
                first_control-second_control,config.dt_s,tube,
                config.collision_distance_m);
            REQUIRE(interval.valid);
            result.minimum_interval_collision_clearance_m=std::min(
                result.minimum_interval_collision_clearance_m,
                interval.minimum_robust_clearance_m);
        }
        result.minimum_applied_residual=std::min(
            result.minimum_applied_residual,step.step.minimum_hard_residual);
        for (const auto& [owner,diagnostic]:step.step.gamma_feedback) {
            (void)owner;
            if (diagnostic.current_gamma<result.minimum_current_gamma) {
                result.minimum_current_gamma=diagnostic.current_gamma;
                result.dominant_row_at_minimum_gamma=diagnostic.dominant_row;
            }
            result.minimum_selected_successor_gamma=std::min(
                result.minimum_selected_successor_gamma,
                diagnostic.selected_predicted_gamma);
            if (diagnostic.intervened &&
                !std::isfinite(result.first_intervention_s))
                result.first_intervention_s=before_s;
            if ((diagnostic.current_hard_projection-
                    diagnostic.selected_nominal).norm()>1.0e-6 &&
                !std::isfinite(result.first_hard_projection_s)) {
                result.first_hard_projection_s=before_s;
                if (pair23.valid)
                    result.pair23_closing_at_first_hard_mps=
                        pair23.robust_radial_closing_speed_mps;
            }
        }
    }
    result.advanced_through_horizon=true;
    result.reason="horizon_complete";
    result.final_truth_coverage=adapter.coverage().truthFraction();
    return result;
}

gf::Task10p10Scenario gainHoldout4p2Scenario() {
    return {"task10p11n_holdout_4p2",400.0,600.0,{1,2,3,4},
        {{100.0,50.0},{160.0,50.0},{100.0,110.0},{160.0,110.0}},
        {{100,{100.0,-50.0}},{101,{250.0,-50.0}}},
        {{100,1},{101,1},{100,2},{101,2},
         {100,3},{101,3},{100,4},{101,4}}};
}

struct OneStepHoldoutResult {
    bool initialized=false;
    bool advanced=false;
    std::string reason;
    double residual=-std::numeric_limits<double>::infinity();
};

OneStepHoldoutResult runOneStepHoldout(
    const gf::LinearHocbfGains& gains,gf::SolverProfile profile) {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    const auto scenario=gainHoldout4p2Scenario();
    auto settings=gf::task10p11gSwarmSettings(scenario,profile);
    Swarm swarm(settings);
    auto config=gf::task10p11gAdapterConfig(profile,boundary);
    config.collision_lambda1=gains.lambda1_per_s;
    config.collision_lambda2=gains.lambda2_per_s;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm,scenario.mobile_ids,scenario.fixed_positions,
        scenario.initial_topology,config);
    OneStepHoldoutResult result;
    const auto stage_zero=adapter.initializeStageZero();
    result.initialized=stage_zero.initialized;
    result.reason=stage_zero.reason;
    if (!stage_zero.initialized) return result;
    const auto step=adapter.step();
    result.advanced=step.advanced;
    result.reason=step.reason;
    result.residual=step.minimum_hard_residual;
    return result;
}

}

TEST_CASE("general linear class-K gains change psi1 and the central collision row") {
    const auto row=gf::buildSnapshotRobustPairRow(
        leftState(4.0),rightState(20.0,4.0),collisionSpec(0.5,2.0),
        {0.0,0.0},4.0);

    CHECK(row.barrier_h_lower==doctest::Approx(10.0));
    CHECK(row.barrier_hdot_lower==doctest::Approx(-4.0));
    CHECK(row.barrier_psi1_lower==doctest::Approx(1.0));
    CHECK(row.central_constant_lower==doctest::Approx(0.0));
}

TEST_CASE("unit class-K gains retain the inherited robust row exactly") {
    const auto row=gf::buildSnapshotRobustPairRow(
        leftState(4.0),rightState(20.0,4.0),collisionSpec(1.0,1.0),
        {0.2,0.1},4.0);

    CHECK(row.barrier_h_lower==doctest::Approx(9.8));
    CHECK(row.barrier_hdot_lower==doctest::Approx(-4.1400005000375));
    CHECK(row.barrier_psi1_lower==doctest::Approx(5.6599994999625));
    CHECK(row.central_constant_lower==doctest::Approx(1.519998999925));
}

TEST_CASE("nonpositive class-K gains are rejected") {
    CHECK_THROWS_AS(gf::buildSnapshotRobustPairRow(
        leftState(0.0),rightState(20.0,0.0),collisionSpec(0.0,1.0),
        {0.0,0.0},4.0),std::invalid_argument);
    CHECK_THROWS_AS(gf::buildSnapshotRobustPairRow(
        leftState(0.0),rightState(20.0,0.0),collisionSpec(1.0,-0.1),
        {0.0,0.0},4.0),std::invalid_argument);
}

TEST_CASE("independent half-row audit reconstructs general class-K constants") {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={1,2};
    request.states[1]=leftState(4.0);
    request.states[2]=rightState(20.0,4.0);
    request.collision_pairs={gf::UndirectedEdge::canonical(1,2)};
    request.collision_spec=collisionSpec(0.5,2.0);
    request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box=4.0;
    request.require_snapshot_robust_rows=true;
    request.collision_snapshot_tubes["1--2"]={0.0,0.0};

    const auto audit=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(1,2));
    REQUIRE(audit.valid);
    CHECK(audit.independent_central_verified);
    CHECK(audit.independent_central_constant==doctest::Approx(0.0));
    CHECK(audit.first_constant==doctest::Approx(0.0));
    CHECK(audit.second_constant==doctest::Approx(0.0));
}

TEST_CASE("formal adapter applies configured gains only to collision rows") {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    const auto scenario=gf::task10p11hCoastalLeaderEasyScenario();
    auto settings=gf::task10p11gSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    Swarm swarm(settings);
    auto config=gf::task10p11gAdapterConfig(
        gf::SolverProfile::Gurobi,boundary);
    config.collision_lambda1=0.25;
    config.collision_lambda2=1.5;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm,scenario.mobile_ids,scenario.fixed_positions,
        scenario.initial_topology,config);
    const auto stage_zero=adapter.initializeStageZero();
    REQUIRE(stage_zero.initialized);
    const auto runtime=adapter.runtimeSnapshot();
    const auto request=adapter.snapshotHardRowRequest(
        runtime.estimate,runtime.topology);

    CHECK(request.collision_spec.lambda1==doctest::Approx(0.25));
    CHECK(request.collision_spec.lambda2==doctest::Approx(1.5));
    CHECK(request.reference_spec.lambda1==doctest::Approx(1.0));
    CHECK(request.reference_spec.lambda2==doctest::Approx(1.0));
}

TEST_CASE("gain synthesis domain is finite physics-derived and immutable") {
    const auto candidates=gf::task10p11nGainCandidates();
    REQUIRE(candidates.size()==16);
    CHECK(candidates.front()==gf::LinearHocbfGains{0.125,0.5});
    CHECK(candidates.back()==gf::LinearHocbfGains{1.0,2.0});
    CHECK(std::count(candidates.begin(),candidates.end(),
          gf::LinearHocbfGains{1.0,1.0})==1);
}

TEST_CASE("isolated exact braking oracle reproduces inherited and widened margins") {
    const auto inherited=gf::auditIsolatedPairGain(
        {1.0,1.0},37.2,22.5,8.0,0.1);
    const auto widened=gf::auditIsolatedPairGain(
        {1.0,2.0},37.2,22.5,8.0,0.1);
    REQUIRE(inherited.valid);
    REQUIRE(widened.valid);
    CHECK(inherited.minimum_central_margin_mps2==
          doctest::Approx(-2.440625).epsilon(1e-12));
    CHECK(inherited.minimum_time_s==
          doctest::Approx(0.8125).epsilon(1e-12));
    CHECK(inherited.minimum_endpoint_margin_mps2==
          doctest::Approx(-2.44).epsilon(1e-12));
    CHECK(inherited.minimum_endpoint_time_s==
          doctest::Approx(0.8).epsilon(1e-12));
    CHECK_FALSE(inherited.complete_braking_nonnegative);
    CHECK(widened.minimum_central_margin_mps2==
          doctest::Approx(1.11875).epsilon(1e-12));
    CHECK(widened.complete_braking_nonnegative);
}

TEST_CASE("exact-ZOH pair interval audit finds an interior distance minimum") {
    const auto audit=gf::auditExactZohPairInterval(
        {20.0,0.0},{-4.0,0.0},{8.0,0.0},1.0,0.2,10.0);
    REQUIRE(audit.valid);
    CHECK(audit.minimum_time_s==doctest::Approx(0.5).epsilon(1e-10));
    CHECK(audit.minimum_nominal_distance_m==
          doctest::Approx(19.0).epsilon(1e-10));
    CHECK(audit.minimum_robust_clearance_m==
          doctest::Approx(8.8).epsilon(1e-10));
}

TEST_CASE("frozen gain domain runs from formal stage zero without dropping failures") {
    const auto candidates=gf::task10p11nGainCandidates();
    std::vector<FormalGainRun> results;
    for (const auto& candidate:candidates) {
        results.push_back(runFormalLeaderEasy(
            candidate,gf::SolverProfile::Gurobi));
        const auto& result=results.back();
        MESSAGE("TASK10P11N_GAIN l1=",candidate.lambda1_per_s,
            " l2=",candidate.lambda2_per_s,
            " initialized=",result.initialized,
            " horizon=",result.advanced_through_horizon,
            " fail_t=",result.failure_time_s,
            " reason=",result.reason,
            " initial_h=",result.initial_minimum_h,
            " initial_psi1=",result.initial_minimum_psi1,
            " min_h=",result.minimum_collision_h,
            " min_psi1=",result.minimum_collision_psi1,
            " interval_clearance=",result.minimum_interval_collision_clearance_m,
            " gamma=",result.minimum_current_gamma,
            " successor_gamma=",result.minimum_selected_successor_gamma,
            " residual=",result.minimum_applied_residual,
            " intervene=",result.first_intervention_s,
            " hard_project=",result.first_hard_projection_s,
            " pair23_closing=",result.pair23_closing_at_first_hard_mps,
            " pair23_slack=",result.minimum_pair23_braking_slack_m,
            " pair23_negative=",result.first_negative_pair23_braking_s,
            " dominant=",result.dominant_row_at_minimum_gamma,
            " coverage=",result.final_truth_coverage);
        if (result.initialized && result.minimum_applied_residual<
                std::numeric_limits<double>::infinity())
            CHECK(result.minimum_applied_residual>=-1.0e-7);
    }
    REQUIRE(results.size()==16);
    const auto baseline=std::find_if(
        results.begin(),results.end(),[](const auto& result) {
            return result.gains==gf::LinearHocbfGains{1.0,1.0};
        });
    REQUIRE(baseline!=results.end());
    REQUIRE(baseline->initialized);
    CHECK_FALSE(baseline->advanced_through_horizon);
    CHECK(baseline->failure_time_s==doctest::Approx(4.8));
    CHECK(baseline->reason=="current_gamma_negative");
}

TEST_CASE("frozen 4 plus 2 holdout keeps Gurobi and OSQP hard semantics") {
    const std::vector<gf::LinearHocbfGains> representatives{
        {0.125,0.5},{0.5,1.0},{1.0,0.5},{1.0,1.0}};
    for (const auto& gains:representatives) {
        const auto gurobi=runOneStepHoldout(gains,gf::SolverProfile::Gurobi);
        const auto osqp=runOneStepHoldout(gains,gf::SolverProfile::OpenSource);
        CAPTURE(gains.lambda1_per_s);
        CAPTURE(gains.lambda2_per_s);
        CAPTURE(gurobi.reason);
        CAPTURE(osqp.reason);
        REQUIRE(gurobi.initialized);
        REQUIRE(osqp.initialized);
        REQUIRE(gurobi.advanced);
        REQUIRE(osqp.advanced);
        CHECK(gurobi.residual>=-1.0e-7);
        CHECK(osqp.residual>=-1.0e-7);
    }
}

TEST_CASE("cross-axis gain holdout is rotationally identical") {
    const auto gains=gf::LinearHocbfGains{1.0,2.0};
    const auto x_axis=gf::buildSnapshotRobustPairRow(
        leftState(4.0),rightState(20.0,4.0),
        collisionSpec(gains.lambda1_per_s,gains.lambda2_per_s),
        {0.2,0.1},4.0);
    PairwiseSecondOrderState2D lower{{0.0,0.0},{0.0,2.0},{0.0,0.0}};
    PairwiseSecondOrderState2D upper{{0.0,20.0},{0.0,-2.0},{0.0,0.0}};
    const auto y_axis=gf::buildSnapshotRobustPairRow(
        lower,upper,collisionSpec(gains.lambda1_per_s,gains.lambda2_per_s),
        {0.2,0.1},4.0);
    CHECK(y_axis.barrier_h_lower==doctest::Approx(x_axis.barrier_h_lower));
    CHECK(y_axis.barrier_hdot_lower==doctest::Approx(x_axis.barrier_hdot_lower));
    CHECK(y_axis.barrier_psi1_lower==doctest::Approx(x_axis.barrier_psi1_lower));
    CHECK(y_axis.central_constant_lower==
          doctest::Approx(x_axis.central_constant_lower));
}

TEST_CASE("all frozen gains preserve the multi-row joint holdout at rest") {
    for (const auto& gains:gf::task10p11nGainCandidates()) {
        gf::CanonicalHardRowRequest request;
        request.mobile_ids={1,2,3};
        request.fixed_ids={100,101};
        request.states[1]={{0.0,0.0},{0.0,0.0},{0.0,0.0}};
        request.states[2]={{-30.0,0.0},{0.0,0.0},{0.0,0.0}};
        request.states[3]={{0.0,-30.0},{0.0,0.0},{0.0,0.0}};
        request.states[100]={{100.0,0.0},{0.0,0.0},{0.0,0.0}};
        request.states[101]={{0.0,100.0},{0.0,0.0},{0.0,0.0}};
        request.collision_pairs={gf::UndirectedEdge::canonical(1,2),
                                 gf::UndirectedEdge::canonical(1,3)};
        request.reference_edges={{100,1},{101,1}};
        request.reference_spec={
            PairwiseSecondOrderBarrierKind::CommunicationUpper,
            850.0,0.0,1.0,1.0,1.0,0.0};
        request.collision_spec=collisionSpec(
            gains.lambda1_per_s,gains.lambda2_per_s);
        request.acceleration_half_box=4.0;
        request.speed_limit_mps=30.0;
        request.require_snapshot_robust_rows=true;
        request.collision_snapshot_tubes["1--2"]={0.2,0.1};
        request.collision_snapshot_tubes["1--3"]={0.2,0.1};
        request.reference_snapshot_tubes["100->1"]={0.2,0.1};
        request.reference_snapshot_tubes["101->1"]={0.2,0.1};
        for (gf::NodeId owner:request.mobile_ids)
            request.speed_snapshot_tubes[owner]={0.0,0.1};
        const auto rows=gf::buildCanonicalHardRows(request);
        const auto exact=gf::evaluateProgressCompatibility(
            rows,1,Eigen::Vector2d::Zero(),4.0,
            {std::numeric_limits<double>::max(),0.0,1.0e-10,true});
        CAPTURE(gains.lambda1_per_s);
        CAPTURE(gains.lambda2_per_s);
        CHECK(exact.polytope_nonempty);
        for (const auto& row:rows) {
            if (row.kind==gf::CanonicalHardRowKind::Collision) {
                CHECK(row.barrier_h>=0.0);
                CHECK(row.barrier_psi1>=0.0);
            }
        }
    }
}

TEST_CASE("near-boundary synthesized snapshot retains Gurobi OSQP QP parity") {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    const auto scenario=gf::task10p11hCoastalLeaderEasyScenario();
    auto settings=gf::task10p11gSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    Swarm swarm(settings);
    auto config=gf::task10p11gAdapterConfig(
        gf::SolverProfile::Gurobi,boundary);
    config.collision_lambda1=0.125;
    config.collision_lambda2=0.5;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm,scenario.mobile_ids,scenario.fixed_positions,
        scenario.initial_topology,config);
    REQUIRE(adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    for (int cycle=0;cycle<76;++cycle)
        REQUIRE(controller.advance().step.advanced);
    const auto before=adapter.runtimeSnapshot();
    REQUIRE(before.runtime_s==doctest::Approx(7.6));
    const auto rows=adapter.currentSnapshotHardRows(before.topology);
    const auto executed=controller.advance();
    REQUIRE(executed.step.advanced);
    const gf::NodeId owner=7;
    const auto diagnostic=executed.step.gamma_feedback.at(owner);
    gf::CanonicalQpRequest request;
    request.owner=owner;
    request.estimator_version=before.estimator_token;
    request.topology_version=before.topology_token;
    request.mode=before.mode;
    request.nominal=diagnostic.selected_nominal;
    request.acceleration_half_box=4.0;
    request.rows=rows;
    request.residual_tolerance=1.0e-7;
    gf::CanonicalHocbfQpController gurobi_controller;
    gf::CanonicalHocbfQpController osqp_controller;
    request.profile=gf::SolverProfile::Gurobi;
    const auto gurobi=gurobi_controller.solve(request);
    request.profile=gf::SolverProfile::OpenSource;
    const auto osqp=osqp_controller.solve(request);
    REQUIRE(gurobi.control_available);
    REQUIRE(osqp.control_available);
    CHECK(gurobi.minimum_hard_residual>=-1.0e-7);
    CHECK(osqp.minimum_hard_residual>=-1.0e-7);
    CHECK((gurobi.control-osqp.control).norm()<1.0e-4);
    CHECK((gurobi.control-executed.step.applied_controls.at(owner)).norm()<
          1.0e-4);
}
