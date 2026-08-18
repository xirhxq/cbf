#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11oSuccessorFeasibility.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

#include <map>

TEST_CASE("Task 10.11o pair successor oracle certifies a negative relaxed upper bound") {
    gf::PairSuccessorUpperRequest request;
    request.first.position={193.219,15.256};
    request.first.velocity={15.182,-7.12992};
    request.second.position={240.384,17.8254};
    request.second.velocity={-7.29343,-5.57744};
    request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        10.0,0.0,1.0,1.0,1.0,0.0};
    request.successor_tube={0.26218,0.0543};
    request.acceleration_half_box=4.0;
    request.dt_s=0.1;
    request.branch_tolerance_mps2=1.0e-4;

    const auto result=gf::certifyRelaxedPairSuccessorGammaUpper(request);
    REQUIRE(result.valid);
    CHECK(result.certified_upper_gamma_mps2<0.0);
    CHECK(result.best_relative_acceleration.x()==doctest::Approx(-8.0));
    CHECK(result.best_relative_acceleration.y()==doctest::Approx(-8.0));
    CHECK(result.boxes_examined>0);
}

TEST_CASE("Task 10.11o branch upper dominates dense pair-only samples") {
    gf::PairSuccessorUpperRequest request;
    request.first.position={193.219,15.256};
    request.first.velocity={15.182,-7.12992};
    request.second.position={240.384,17.8254};
    request.second.velocity={-7.29343,-5.57744};
    request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        10.0,0.0,1.0,1.0,1.0,0.0};
    request.successor_tube={0.26218,0.0543};
    request.acceleration_half_box=4.0;
    request.dt_s=0.1;
    request.branch_tolerance_mps2=1.0e-5;
    const auto result=gf::certifyRelaxedPairSuccessorGammaUpper(request);
    REQUIRE(result.valid);
    for (int ix=0;ix<=40;++ix) for (int iy=0;iy<=40;++iy) {
        const Eigen::Vector2d acceleration(
            -8.0+16.0*static_cast<double>(ix)/40.0,
            -8.0+16.0*static_cast<double>(iy)/40.0);
        CHECK(gf::task10p11o_detail::pairOnlySuccessorGamma(
                  request,acceleration)<=
              result.certified_upper_gamma_mps2+1.0e-12);
    }
}

TEST_CASE("Task 10.11o brackets the actual-prefix K1 loss between 4.6 and 4.7 seconds") {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalLeaderEasyScenario(),
        gf::SolverProfile::Gurobi,boundary);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);

    double local_lower_4p6=-std::numeric_limits<double>::infinity();
    double local_lower_4p7=-std::numeric_limits<double>::infinity();
    double pair_upper_4p7=std::numeric_limits<double>::infinity();
    double current_gamma_4p7=-std::numeric_limits<double>::infinity();
    double k2_constructive_lower_4p6=-std::numeric_limits<double>::infinity();
    double component_k2_lower_4p6=-std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_relative=Eigen::Vector2d::Zero();
    Eigen::Vector2d applied_relative=Eigen::Vector2d::Zero();

    for (int cycle=0;cycle<48;++cycle) {
        const auto before=fixture->adapter.runtimeSnapshot();
        const auto topology=before.topology;
        const auto current_rows=fixture->adapter.currentSnapshotHardRows(topology);
        const auto step=controller.advance();
        REQUIRE(step.step.advanced);
        if (before.runtime_s<4.6-1.0e-12) continue;

        const auto& owner2=step.step.gamma_feedback.at(2);
        const auto& owner3=step.step.gamma_feedback.at(3);
        const double local_lower=std::min(
            owner2.nominal_predicted_gamma+
                owner2.controllable_predicted_gamma_range,
            owner3.nominal_predicted_gamma+
                owner3.controllable_predicted_gamma_range);
        if (before.runtime_s==doctest::Approx(4.6)) {
            local_lower_4p6=local_lower;
            std::map<gf::NodeId,Eigen::Vector2d> zero_controls;
            for (const auto owner:before.estimate.mobile_ids)
                zero_controls.emplace(owner,Eigen::Vector2d::Zero());
            const auto predicted=gf::predictNoMeasurementSnapshot(
                before.estimate,zero_controls,fixture->adapter.config().dt_s,
                fixture->adapter.config().estimator_acceleration_variance);
            const auto current_request=
                fixture->adapter.snapshotHardRowRequest(before.estimate,topology);
            const auto successor_request=
                fixture->adapter.snapshotHardRowRequest(predicted,topology);
            gf::PairSuccessorUpperRequest request;
            request.first=current_request.states.at(2);
            request.second=current_request.states.at(3);
            request.collision_spec=current_request.collision_spec;
            request.successor_tube=
                successor_request.collision_snapshot_tubes.at("2--3");
            request.acceleration_half_box=current_request.acceleration_half_box;
            request.dt_s=fixture->adapter.config().dt_s;
            request.branch_tolerance_mps2=1.0e-5;
            const auto one_step=
                gf::certifyRelaxedPairSuccessorGammaUpper(request);
            auto witness_snapshot=before.estimate;
            double witness_min=std::numeric_limits<double>::infinity();
            for (int witness_step=0;witness_step<2;++witness_step) {
                const auto witness_rows=gf::buildCanonicalHardRows(
                    fixture->adapter.snapshotHardRowRequest(
                        witness_snapshot,topology));
                std::map<gf::NodeId,Eigen::Vector2d> witness_controls;
                for (const auto owner:witness_snapshot.mobile_ids) {
                    const auto gamma=gf::solveCanonicalGammaStar(
                        witness_rows,owner,4.0);
                    REQUIRE(gamma.valid);
                    REQUIRE(gamma.gamma>=0.0);
                    witness_controls.emplace(
                        owner,Eigen::Vector2d(gamma.accelX,gamma.accelY));
                }
                witness_snapshot=gf::predictNoMeasurementSnapshot(
                    witness_snapshot,witness_controls,
                    fixture->adapter.config().dt_s,
                    fixture->adapter.config().estimator_acceleration_variance);
                const auto successor_rows=gf::buildCanonicalHardRows(
                    fixture->adapter.snapshotHardRowRequest(
                        witness_snapshot,topology));
                witness_min=std::numeric_limits<double>::infinity();
                for (const auto owner:witness_snapshot.mobile_ids) {
                    const auto gamma=gf::solveCanonicalGammaStar(
                        successor_rows,owner,4.0);
                    REQUIRE(gamma.valid);
                    witness_min=std::min(witness_min,gamma.gamma);
                }
            }
            k2_constructive_lower_4p6=witness_min;

            std::map<gf::NodeId,Eigen::Vector2d> component_controls;
            for (const auto owner:before.estimate.mobile_ids)
                component_controls.emplace(
                    owner,step.step.gamma_feedback.at(owner).
                        current_hard_projection);
            component_controls.at(2)=
                step.step.gamma_feedback.at(2).maximum_margin_control;
            component_controls.at(3)=
                step.step.gamma_feedback.at(3).maximum_margin_control;
            auto component_snapshot=gf::predictNoMeasurementSnapshot(
                before.estimate,component_controls,
                fixture->adapter.config().dt_s,
                fixture->adapter.config().estimator_acceleration_variance);
            const auto component_rows=gf::buildCanonicalHardRows(
                fixture->adapter.snapshotHardRowRequest(
                    component_snapshot,topology));
            std::map<gf::NodeId,Eigen::Vector2d> second_controls;
            for (const auto owner:component_snapshot.mobile_ids) {
                const auto gamma=gf::solveCanonicalGammaStar(
                    component_rows,owner,4.0);
                REQUIRE(gamma.valid);
                REQUIRE(gamma.gamma>=0.0);
                second_controls.emplace(
                    owner,Eigen::Vector2d(gamma.accelX,gamma.accelY));
            }
            component_snapshot=gf::predictNoMeasurementSnapshot(
                component_snapshot,second_controls,
                fixture->adapter.config().dt_s,
                fixture->adapter.config().estimator_acceleration_variance);
            const auto component_second_rows=gf::buildCanonicalHardRows(
                fixture->adapter.snapshotHardRowRequest(
                    component_snapshot,topology));
            component_k2_lower_4p6=std::numeric_limits<double>::infinity();
            for (const auto owner:component_snapshot.mobile_ids) {
                const auto gamma=gf::solveCanonicalGammaStar(
                    component_second_rows,owner,4.0);
                REQUIRE(gamma.valid);
                component_k2_lower_4p6=std::min(
                    component_k2_lower_4p6,gamma.gamma);
            }
            MESSAGE("TASK10P11O_PREFIX t=4.6 local_candidate_lower=",
                local_lower_4p6," relaxed_pair_upper=",
                one_step.certified_upper_gamma_mps2,
                " best_relative=",one_step.best_relative_acceleration.x(),",",
                one_step.best_relative_acceleration.y(),
                " k2_constructive_lower=",k2_constructive_lower_4p6,
                " component_k2_lower=",component_k2_lower_4p6);
        }
        if (before.runtime_s!=doctest::Approx(4.7)) continue;

        const auto gamma2=gf::solveCanonicalGammaStar(current_rows,2,4.0);
        const auto gamma3=gf::solveCanonicalGammaStar(current_rows,3,4.0);
        REQUIRE(gamma2.valid);
        REQUIRE(gamma3.valid);
        current_gamma_4p7=std::min(gamma2.gamma,gamma3.gamma);
        local_lower_4p7=local_lower;

        std::map<gf::NodeId,Eigen::Vector2d> zero_controls;
        for (const auto owner:before.estimate.mobile_ids)
            zero_controls.emplace(owner,Eigen::Vector2d::Zero());
        const auto predicted=gf::predictNoMeasurementSnapshot(
            before.estimate,zero_controls,fixture->adapter.config().dt_s,
            fixture->adapter.config().estimator_acceleration_variance);
        const auto successor_request=
            fixture->adapter.snapshotHardRowRequest(predicted,topology);
        const auto current_request=
            fixture->adapter.snapshotHardRowRequest(before.estimate,topology);
        gf::PairSuccessorUpperRequest request;
        request.first=current_request.states.at(2);
        request.second=current_request.states.at(3);
        request.collision_spec=current_request.collision_spec;
        request.successor_tube=
            successor_request.collision_snapshot_tubes.at("2--3");
        request.acceleration_half_box=current_request.acceleration_half_box;
        request.dt_s=fixture->adapter.config().dt_s;
        request.branch_tolerance_mps2=1.0e-5;
        const auto upper=gf::certifyRelaxedPairSuccessorGammaUpper(request);
        REQUIRE(upper.valid);
        pair_upper_4p7=upper.certified_upper_gamma_mps2;
        best_relative=upper.best_relative_acceleration;
        applied_relative=step.step.applied_controls.at(2)-
            step.step.applied_controls.at(3);
        MESSAGE("TASK10P11O_PREFIX t=",before.runtime_s,
            " current_gamma=",current_gamma_4p7,
            " local_candidate_lower=",local_lower_4p7,
            " relaxed_pair_upper=",pair_upper_4p7,
            " best_relative=",best_relative.x(),",",best_relative.y(),
            " applied_relative=",applied_relative.x(),",",applied_relative.y(),
            " boxes=",upper.boxes_examined);
    }

    CHECK(local_lower_4p6>0.0);
    CHECK(k2_constructive_lower_4p6>0.0);
    CHECK(component_k2_lower_4p6>0.0);
    CHECK(current_gamma_4p7>0.0);
    CHECK(local_lower_4p7<0.0);
    CHECK(pair_upper_4p7<0.0);
    CHECK(best_relative.x()==doctest::Approx(-8.0));
    CHECK(best_relative.y()==doctest::Approx(-8.0));
    CHECK(applied_relative.x()==doctest::Approx(-8.0).epsilon(1e-6));
    CHECK(applied_relative.y()==doctest::Approx(-8.0).epsilon(1e-6));
}

TEST_CASE("Task 10.11o records the stage-zero local maximum-predicted policy boundary") {
    const auto scenario=gf::task10p11hCoastalLeaderEasyScenario();
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    auto settings=gf::task10p11gSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    Swarm swarm(settings);
    auto config=gf::task10p11gAdapterConfig(
        gf::SolverProfile::Gurobi,boundary);
    config.gamma_feedback_selection=
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin;
    config.predictive_gamma_tau_mps2=std::nullopt;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm,scenario.mobile_ids,scenario.fixed_positions,
        scenario.initial_topology,config);
    REQUIRE(adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    std::string reason;
    for (int cycle=0;cycle<80;++cycle) {
        const auto step=controller.advance();
        reason=step.reason;
        if (!step.step.advanced) break;
    }
    const auto runtime=adapter.runtimeSnapshot();
    MESSAGE("TASK10P11O_LOCAL_MAX stop=",runtime.runtime_s,
        " reason=",reason);
    CHECK(runtime.runtime_s>=4.8);
}
