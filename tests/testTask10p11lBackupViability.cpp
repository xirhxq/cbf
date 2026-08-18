#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/BrakingSnapshotOracle.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"


namespace {

gf::CanonicalHardRowRequest formalPair(
    double separation_m,double relative_closing_mps) {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={10,11};
    request.states[10]={{0.0,0.0},{0.5*relative_closing_mps,0.0},{0.0,0.0}};
    request.states[11]={{separation_m,0.0},{-0.5*relative_closing_mps,0.0},
                        {0.0,0.0}};
    request.collision_pairs={gf::UndirectedEdge::canonical(10,11)};
    request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        10.0,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box=4.0;
    request.require_snapshot_robust_rows=true;
    request.collision_snapshot_tubes["10--11"]={0.2,0.1};
    return request;
}

bool ownerFeasible(
    const std::vector<gf::CanonicalHardRow>& rows,gf::NodeId owner) {
    return gf::evaluateProgressCompatibility(
        rows,owner,Eigen::Vector2d::Zero(),4.0,
        {std::numeric_limits<double>::max(),0.0,1e-10,true})
        .polytope_nonempty;
}

gf::Task10p10Scenario formal4p2Scenario() {
    return {"task10p11l_formal_4p2",400.0,600.0,{1,2,3,4},
        {{100.0,50.0},{160.0,50.0},{100.0,110.0},{160.0,110.0}},
        {{100,{100.0,-50.0}},{101,{250.0,-50.0}}},
        {{100,1},{101,1},{100,2},{101,2},
         {100,3},{101,3},{100,4},{101,4}}};
}

}

TEST_CASE("Task 10.11l ten metre mobile pair keeps tube outside the hard distance") {
    const auto request=formalPair(20.0,4.0);
    const auto audit=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(10,11));
    REQUIRE(audit.valid);
    CHECK(audit.robust_separation_margin_m==doctest::Approx(9.8));
    CHECK(audit.input_box_relative_separation_support_mps2==
          doctest::Approx(8.0));
    CHECK(audit.independent_central_verified);
    const Eigen::Vector2d first{-4.0,1.0};
    const Eigen::Vector2d second{4.0,-1.0};
    CHECK(audit.centralizedResidual(first,second)==doctest::Approx(
        audit.firstLocalResidual(first)+audit.secondLocalResidual(second)));
}

TEST_CASE("Task 10.11l can have positive current gamma after braking slack is lost") {
    auto request=formalPair(10.4,3.0);
    request.collision_snapshot_tubes["10--11"]={0.0,0.0};
    const auto rows=gf::buildCanonicalHardRows(request);
    const auto gamma=gf::solveCanonicalGammaStar(rows,10,4.0);
    const auto audit=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(10,11));
    REQUIRE(gamma.valid);
    REQUIRE(audit.valid);
    CHECK(gamma.gamma>0.0);
    CHECK(audit.braking_slack_m<0.0);
}

TEST_CASE("Task 10.11l physical braking slack does not imply HOCBF successor viability") {
    // One-dimensional closing motion embedded in the two-dimensional model.
    // The current robust HOCBF polytope is nonempty and the physical stopping
    // distance fits inside the clearance, yet even the maximum separating
    // acceleration reaches a successor whose HOCBF polytope is empty.
    auto request=formalPair(47.2,22.5);
    request.collision_snapshot_tubes["10--11"]={0.0,0.0};
    const auto rows=gf::buildCanonicalHardRows(request);
    const auto current=gf::solveCanonicalGammaStar(rows,10,4.0);
    const auto braking=gf::evaluateMobilePairBraking(
        request,gf::UndirectedEdge::canonical(10,11));
    REQUIRE(current.valid);
    REQUIRE(braking.valid);
    CHECK(current.gamma==doctest::Approx(0.1).epsilon(1e-12));
    CHECK(braking.braking_slack_m>5.0);

    gf::JointEstimateSnapshot snapshot;
    snapshot.mobile_ids={10,11};
    snapshot.mean=Eigen::VectorXd::Zero(8);
    snapshot.mean.segment<4>(0)<<0.0,0.0,11.25,0.0;
    snapshot.mean.segment<4>(4)<<47.2,0.0,-11.25,0.0;
    snapshot.covariance=Eigen::MatrixXd::Zero(8,8);
    const std::map<gf::NodeId,Eigen::Vector2d> maximum_separation{
        {10,{-4.0,0.0}},{11,{4.0,0.0}}};
    const auto successor=gf::predictNoMeasurementSnapshot(
        snapshot,maximum_separation,0.1,0.0);
    auto successor_request=request;
    successor_request.states[10]={
        Point(successor.mean(0),successor.mean(1)),
        successor.mean.segment<2>(2),Eigen::Vector2d::Zero()};
    successor_request.states[11]={
        Point(successor.mean(4),successor.mean(5)),
        successor.mean.segment<2>(6),Eigen::Vector2d::Zero()};
    const auto successor_rows=gf::buildCanonicalHardRows(successor_request);
    const auto successor_gamma=gf::solveCanonicalGammaStar(
        successor_rows,10,4.0);
    REQUIRE(successor_gamma.valid);
    CHECK(successor_gamma.gamma==doctest::Approx(-0.205).epsilon(1e-12));
    CHECK_FALSE(ownerFeasible(successor_rows,10));
}

TEST_CASE("Task 10.11l mobile fixed collision uses full local responsibility") {
    auto request=formalPair(20.0,0.0);
    request.fixed_ids={100};
    request.states[100]={{0.0,20.0},{0.0,0.0},{0.0,0.0}};
    request.collision_pairs.push_back(gf::UndirectedEdge::canonical(10,100));
    request.collision_snapshot_tubes["10--100"]={0.2,0.1};
    const auto rows=gf::buildCanonicalHardRows(request);
    const auto found=std::find_if(rows.begin(),rows.end(),[](const auto& row) {
        return row.id=="collision:10--100:owner:10";
    });
    REQUIRE(found!=rows.end());
    CHECK(found->responsibility==doctest::Approx(1.0));
    CHECK(found->barrier_h==doctest::Approx(9.8));
    CHECK(std::none_of(rows.begin(),rows.end(),[](const auto& row) {
        return row.id=="collision:10--100:owner:100";
    }));
}

TEST_CASE("Task 10.11l braking boundary has positive equality and negative cases") {
    auto positive=formalPair(11.1,4.0);
    positive.collision_snapshot_tubes["10--11"]={0.0,0.0};
    auto equality=formalPair(11.0,4.0);
    equality.collision_snapshot_tubes["10--11"]={0.0,0.0};
    auto negative=formalPair(10.9,4.0);
    negative.collision_snapshot_tubes["10--11"]={0.0,0.0};
    const auto edge=gf::UndirectedEdge::canonical(10,11);
    CHECK(gf::evaluateMobilePairBraking(positive,edge).braking_slack_m>0.0);
    CHECK(gf::evaluateMobilePairBraking(equality,edge).braking_slack_m==
          doctest::Approx(0.0).epsilon(1e-12));
    CHECK(gf::evaluateMobilePairBraking(negative,edge).braking_slack_m<0.0);
}

TEST_CASE("Task 10.11l collision reference speed and input rows have joint support") {
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
    request.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        10.0,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box=4.0;
    request.speed_limit_mps=30.0;
    request.require_snapshot_robust_rows=true;
    request.collision_snapshot_tubes["1--2"]={0.2,0.1};
    request.collision_snapshot_tubes["1--3"]={0.2,0.1};
    request.reference_snapshot_tubes["100->1"]={0.2,0.1};
    request.reference_snapshot_tubes["101->1"]={0.2,0.1};
    for (gf::NodeId owner : request.mobile_ids)
        request.speed_snapshot_tubes[owner]={0.0,0.1};
    const auto rows=gf::buildCanonicalHardRows(request);
    CHECK(ownerFeasible(rows,1));
    const auto snapshot=gf::evaluateBrakingSnapshot(request,0.5);
    CHECK(snapshot.hard_polytope_nonempty);
    CHECK(snapshot.snapshot_braking_admissible);
}

TEST_CASE("Task 10.11l formal 4+2 and coastal 14+3 initial ledgers are nonempty") {
    for (const auto scenario : {formal4p2Scenario(),
                                gf::task10p11hCoastalLeaderEasyScenario()}) {
        CAPTURE(scenario.id);
        gf::BoundaryPolicyConfig boundary;
        boundary.policy=gf::BoundaryPolicy::None;
        auto fixture=gf::makeTask10p11gFixture(
            scenario,gf::SolverProfile::OpenSource,boundary);
        const auto stage=fixture->adapter.initializeStageZero();
        INFO(stage.reason);
        REQUIRE(stage.initialized);
        CHECK(fixture->adapter.config().collision_distance_m==
              doctest::Approx(10.0));
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const auto rows=fixture->adapter.currentSnapshotHardRows(runtime.topology);
        for (const auto owner : scenario.mobile_ids) {
            CAPTURE(owner);
            CHECK(ownerFeasible(rows,owner));
            const auto gamma=gf::solveCanonicalGammaStar(rows,owner,4.0);
            REQUIRE(gamma.valid);
            CHECK(gamma.gamma>=-1e-10);
        }
    }
}

TEST_CASE("Task 10.11l records the first formal ten metre 14+3 stop") {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalLeaderEasyScenario(),
        gf::SolverProfile::OpenSource,boundary);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    gf::SimpleCoverageControlStep stopped;
    for (int cycle=0;cycle<60;++cycle) {
        stopped=controller.advance();
        if (!stopped.step.advanced) break;
    }
    REQUIRE_FALSE(stopped.step.advanced);
    const auto runtime=fixture->adapter.runtimeSnapshot();
    const auto rows=fixture->adapter.currentSnapshotHardRows(runtime.topology);
    gf::CanonicalHocbfQpController qp;
    std::size_t exact_empty=0;
    std::size_t open_failures=0;
    std::size_t gurobi_failures=0;
    for (const auto owner : runtime.estimate.mobile_ids) {
        const auto gamma=gf::solveCanonicalGammaStar(rows,owner,4.0);
        if (!gamma.valid || gamma.gamma<0.0) ++exact_empty;
        Eigen::Vector2d nominal=Eigen::Vector2d::Zero();
        const auto diagnostic=stopped.step.gamma_feedback.find(owner);
        if (diagnostic!=stopped.step.gamma_feedback.end())
            nominal=diagnostic->second.selected_nominal;
        const auto make_request=[&](gf::SolverProfile profile) {
            return gf::CanonicalQpRequest{profile,owner,
                runtime.estimator_token,runtime.topology_token,
                runtime.mode,nominal,4.0,rows,1e-7};
        };
        const auto open=qp.solve(make_request(gf::SolverProfile::OpenSource));
        const auto gurobi=qp.solve(make_request(gf::SolverProfile::Gurobi));
        if (!open.control_available) ++open_failures;
        if (!gurobi.control_available) ++gurobi_failures;
        if (!open.control_available || !gurobi.control_available ||
            !gamma.valid || gamma.gamma<0.0) {
            MESSAGE("TASK10P11L_FORMAL14_STOP owner=",owner,
                " gamma=",gamma.gamma," open=",open.failure_reason,
                "/",open.solver_status," gurobi=",gurobi.failure_reason,
                "/",gurobi.solver_status,
                " exact_projection=",open.hard_polytope_nonempty);
        }
    }
    MESSAGE("TASK10P11L_FORMAL14_STOP t=",runtime.runtime_s,
        " reason=",stopped.reason," exact_empty=",exact_empty,
        " open_failures=",open_failures,
        " gurobi_failures=",gurobi_failures);
    CHECK(runtime.runtime_s==doctest::Approx(4.8));
    CHECK(stopped.reason=="current_gamma_negative");
    CHECK(exact_empty==2);
    CHECK(open_failures==2);
    CHECK(gurobi_failures==2);
}

TEST_CASE("Task 10.11l diagnoses one-step synchronized backup viability") {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalLeaderEasyScenario(),
        gf::SolverProfile::Gurobi,boundary);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    double first_actual_successor_negative_s=std::numeric_limits<double>::infinity();
    double first_maximum_successor_negative_s=std::numeric_limits<double>::infinity();
    double stopped_s=std::numeric_limits<double>::infinity();
    double final_braking_slack=std::numeric_limits<double>::quiet_NaN();
    bool final_pair_controls_are_maximum=false;
    for (int cycle=0;cycle<60;++cycle) {
        const auto before=fixture->adapter.runtimeSnapshot();
        const auto topology=before.topology;
        const auto current_rows=fixture->adapter.currentSnapshotHardRows(topology);
        const auto current_request=fixture->adapter.snapshotHardRowRequest(
            before.estimate,topology);
        const auto braking=gf::evaluateMobilePairBraking(
            current_request,gf::UndirectedEdge::canonical(2,3));
        double current_min=std::numeric_limits<double>::infinity();
        for (const auto owner:before.estimate.mobile_ids) {
            const auto gamma=gf::solveCanonicalGammaStar(current_rows,owner,4.0);
            current_min=std::min(current_min,gamma.gamma);
        }
        const auto step=controller.advance();
        if (!step.step.advanced) {
            stopped_s=before.runtime_s;
            MESSAGE("TASK10P11L_BACKUP_DIAG stop=",before.runtime_s,
                " current_min=",current_min," reason=",step.reason);
            break;
        }
        std::map<gf::NodeId,Eigen::Vector2d> maximum_margin;
        for (const auto& [owner,value]:step.step.gamma_feedback)
            maximum_margin.emplace(owner,value.maximum_margin_control);
        const auto actual_predicted=gf::predictNoMeasurementSnapshot(
            before.estimate,step.step.applied_controls,
            fixture->adapter.config().dt_s,
            fixture->adapter.config().estimator_acceleration_variance);
        const auto backup_predicted=gf::predictNoMeasurementSnapshot(
            before.estimate,maximum_margin,
            fixture->adapter.config().dt_s,
            fixture->adapter.config().estimator_acceleration_variance);
        const auto actual_rows=gf::buildCanonicalHardRows(
            fixture->adapter.snapshotHardRowRequest(actual_predicted,topology));
        const auto backup_rows=gf::buildCanonicalHardRows(
            fixture->adapter.snapshotHardRowRequest(backup_predicted,topology));
        double actual_min=std::numeric_limits<double>::infinity();
        double backup_min=std::numeric_limits<double>::infinity();
        gf::NodeId actual_owner=0;
        gf::NodeId backup_owner=0;
        for (const auto owner:before.estimate.mobile_ids) {
            const auto actual=gf::solveCanonicalGammaStar(actual_rows,owner,4.0);
            const auto backup=gf::solveCanonicalGammaStar(backup_rows,owner,4.0);
            if (actual.gamma<actual_min) {
                actual_min=actual.gamma;
                actual_owner=owner;
            }
            if (backup.gamma<backup_min) {
                backup_min=backup.gamma;
                backup_owner=owner;
            }
        }
        if (actual_min<0.0 && !std::isfinite(first_actual_successor_negative_s))
            first_actual_successor_negative_s=before.runtime_s;
        if (backup_min<0.0 && !std::isfinite(first_maximum_successor_negative_s))
            first_maximum_successor_negative_s=before.runtime_s;
        if (before.runtime_s>=4.0) {
            MESSAGE("TASK10P11L_BACKUP_DIAG t=",before.runtime_s,
                " current=",current_min," actual_next=",actual_min,
                " actual_owner=",actual_owner," max_next=",backup_min,
                " max_owner=",backup_owner," pair23_braking=",
                braking.braking_slack_m," stop_t=",braking.stop_time_s,
                " sep_support=",
                braking.full_hard_relative_separation_support_mps2);
        }
        if (before.runtime_s==doctest::Approx(4.7)) {
            final_braking_slack=braking.braking_slack_m;
            final_pair_controls_are_maximum=
                (step.step.applied_controls.at(2)-maximum_margin.at(2)).norm()<1e-5 &&
                (step.step.applied_controls.at(3)-maximum_margin.at(3)).norm()<1e-5;
            const auto index2=gf::detail::mobileIndex(before.estimate,2);
            const auto index3=gf::detail::mobileIndex(before.estimate,3);
            const Eigen::Vector4d state2=before.estimate.mean.segment<4>(4*index2);
            const Eigen::Vector4d state3=before.estimate.mean.segment<4>(4*index3);
            MESSAGE("TASK10P11L_PAIR23 p2=",state2.x(),",",
                state2.y()," v2=",state2.z(),",",
                state2.w()," p3=",state3.x(),",",
                state3.y()," v3=",state3.z(),",",
                state3.w()," u2=",step.step.applied_controls.at(2).x(),",",
                step.step.applied_controls.at(2).y()," u3=",
                step.step.applied_controls.at(3).x(),",",
                step.step.applied_controls.at(3).y()," umax2=",
                maximum_margin.at(2).x(),",",maximum_margin.at(2).y(),
                " umax3=",maximum_margin.at(3).x(),",",
                maximum_margin.at(3).y());
            for (const auto& row:actual_rows) {
                if ((row.owner==2 && row.peer==3) ||
                    (row.owner==3 && row.peer==2)) {
                    MESSAGE("TASK10P11L_PAIR23_NEXT row=",row.id,
                        " n=",row.control_coefficient.x(),",",
                        row.control_coefficient.y()," c=",row.constant,
                        " h=",row.barrier_h," psi1=",row.barrier_psi1);
                }
            }
        }
    }
    CHECK(first_actual_successor_negative_s==doctest::Approx(4.7));
    CHECK(first_maximum_successor_negative_s==doctest::Approx(4.7));
    CHECK(stopped_s==doctest::Approx(4.8));
    CHECK(final_braking_slack>0.0);
    CHECK(final_pair_controls_are_maximum);
}

TEST_CASE("Task 10.11l bounded maximum-margin counterfactual start times") {
    gf::BoundaryPolicyConfig boundary;
    boundary.policy=gf::BoundaryPolicy::None;
    std::map<double,double> stop_times;
    for (const double start_s:{4.0,4.2,4.4,4.5,4.6,4.7}) {
        auto fixture=gf::makeTask10p11gFixture(
            gf::task10p11hCoastalLeaderEasyScenario(),
            gf::SolverProfile::Gurobi,boundary);
        REQUIRE(fixture->adapter.initializeStageZero().initialized);
        gf::Task10p11hSimpleCoverageController controller(
            fixture->swarm,fixture->adapter);
        bool advanced=true;
        std::string reason;
        for (int cycle=0;cycle<60 && advanced;++cycle) {
            const auto before=fixture->adapter.runtimeSnapshot();
            gf::GrandFinaleSwarmStep step;
            if (before.runtime_s+1.0e-12<start_s) {
                step=controller.advance().step;
            } else {
                const auto rows=fixture->adapter.currentSnapshotHardRows(
                    before.topology);
                std::map<gf::NodeId,Eigen::Vector2d> maximum_margin;
                for (const auto owner:before.estimate.mobile_ids) {
                    const auto gamma=gf::solveCanonicalGammaStar(rows,owner,4.0);
                    if (!gamma.valid || gamma.gamma<0.0) {
                        reason="current_gamma_negative";
                        advanced=false;
                        break;
                    }
                    maximum_margin.emplace(owner,
                        Eigen::Vector2d(gamma.accelX,gamma.accelY));
                }
                if (!advanced) break;
                step=fixture->adapter.stepWithNominal(maximum_margin);
            }
            advanced=step.advanced;
            reason=step.reason;
        }
        MESSAGE("TASK10P11L_MAX_COUNTERFACTUAL start=",start_s,
            " stop=",fixture->adapter.runtimeSnapshot().runtime_s,
            " advanced=",advanced," reason=",reason);
        stop_times.emplace(start_s,fixture->adapter.runtimeSnapshot().runtime_s);
    }
    CHECK(stop_times.at(4.0)==doctest::Approx(6.0));
    CHECK(stop_times.at(4.2)==doctest::Approx(6.0));
    CHECK(stop_times.at(4.4)==doctest::Approx(6.0));
    CHECK(stop_times.at(4.5)==doctest::Approx(6.0));
    CHECK(stop_times.at(4.6)==doctest::Approx(4.9));
    CHECK(stop_times.at(4.7)==doctest::Approx(4.8));
}
