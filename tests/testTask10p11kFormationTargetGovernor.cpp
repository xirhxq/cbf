#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/FormationConsistentTargetGovernor.hpp"
#include "grand_finale/TargetLiftTransitionPrototype.hpp"

namespace {

using gf::NodeId;

gf::FormationTargetTransitionRequest baseRequest() {
    gf::FormationTargetTransitionRequest request;
    request.mobile_ids={1,2};
    request.committed={{1,{-1.0,0.0}},{2,{1.0,0.0}}};
    request.candidate={{1,{1.0,0.0}},{2,{-1.0,0.0}}};
    request.committed_slots={{1,{0,0}},{2,{0,1}}};
    request.candidate_slots=request.committed_slots;
    request.minimum_segment_separation_m=0.5;
    request.minimum_endpoint_separation_m=0.5;
    request.reference_plan_distance_m=849.0;
    return request;
}

}  // namespace

TEST_CASE("Task 10.11k exact segment minimum rejects endpoint-valid exchange") {
    auto request=baseRequest();
    request.committed_slots={{1,{0,0}},{2,{1,0}}};
    request.candidate_slots=request.committed_slots;
    CHECK((request.candidate.at(1)-request.candidate.at(2)).norm()==
          doctest::Approx(2.0));
    const auto minimum=gf::minimumTargetTransitionSeparation(
        request.committed.at(1),request.candidate.at(1),
        request.committed.at(2),request.candidate.at(2));
    CHECK(minimum.distance_m==doctest::Approx(0.0));
    CHECK(minimum.s==doctest::Approx(0.5));
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    REQUIRE(audit.valid);
    CHECK(audit.selected_alpha==doctest::Approx(0.25));
    CHECK(audit.minimum_segment_separation_m==doctest::Approx(1.0));
}

TEST_CASE("Task 10.11k slot permutation cannot manufacture acceptance") {
    auto request=baseRequest();
    request.candidate=request.committed;
    std::swap(request.candidate_slots.at(1),request.candidate_slots.at(2));
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason=="target_transition_blocked:slot_identity");
}

TEST_CASE("Task 10.11k whole-ledger alpha is common to all owners") {
    auto request=baseRequest();
    request.committed_slots={{1,{0,0}},{2,{1,0}}};
    request.candidate_slots=request.committed_slots;
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    REQUIRE(audit.valid);
    REQUIRE(audit.selected.size()==2);
    const Eigen::Vector2d expected1=(1.0-audit.selected_alpha)*
        request.committed.at(1)+audit.selected_alpha*request.candidate.at(1);
    const Eigen::Vector2d expected2=(1.0-audit.selected_alpha)*
        request.committed.at(2)+audit.selected_alpha*request.candidate.at(2);
    CHECK(audit.selected.at(1).isApprox(expected1,1e-12));
    CHECK(audit.selected.at(2).isApprox(expected2,1e-12));
}

TEST_CASE("Task 10.11k all positive alphas block without mutating ledger") {
    auto request=baseRequest();
    request.committed_slots={{1,{0,0}},{2,{1,0}}};
    request.candidate_slots=request.committed_slots;
    request.committed={{1,{-0.055,0.0}},{2,{0.055,0.0}}};
    request.candidate={{1,{0.055,0.0}},{2,{-0.055,0.0}}};
    request.minimum_segment_separation_m=0.1;
    request.minimum_endpoint_separation_m=0.1;
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason=="target_transition_blocked:segment_separation");
    CHECK(audit.selected.at(1).isApprox(request.committed.at(1),1e-12));
    CHECK(audit.selected.at(2).isApprox(request.committed.at(2),1e-12));
    CHECK(gf::nextTargetTransitionBlockState(2,3).terminate);
    CHECK_FALSE(gf::nextTargetTransitionBlockState(1,3).terminate);
}

TEST_CASE("Task 10.11k same-branch order reversal is rejected before alpha search") {
    const auto request=baseRequest();
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason=="target_transition_blocked:slot_order");
    CHECK(audit.evaluated_alphas.empty());
}

TEST_CASE("Task 10.11k duplicate branch rank is rejected") {
    auto request=baseRequest();
    request.committed_slots={{1,{0,0}},{2,{0,0}}};
    request.candidate_slots=request.committed_slots;
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason=="target_transition_blocked:duplicate_slot");
}

TEST_CASE("Task 10.11k alpha ladder is immutable research configuration") {
    CHECK(gf::frozenTargetTransitionAlphaLadder()==
          std::vector<double>{1.0,0.5,0.25,0.125,0.0625});
}

TEST_CASE("Task 10.11k fixed graph reference distance is convexly preserved") {
    gf::FormationTargetTransitionRequest request;
    request.mobile_ids={1,2};
    request.committed={{1,{0.0,0.0}},{2,{800.0,0.0}}};
    request.candidate={{1,{0.0,100.0}},{2,{800.0,100.0}}};
    request.committed_slots={{1,{0,0}},{2,{0,1}}};
    request.candidate_slots=request.committed_slots;
    request.reference_edges={{2,1}};
    request.reference_plan_distance_m=849.0;
    request.minimum_segment_separation_m=0.1;
    request.minimum_endpoint_separation_m=0.1;
    CHECK(gf::fixedGraphReferenceEndpointsValid(request));
    for (double alpha : {0.0,0.125,0.5,1.0}) {
        const auto ledger=gf::interpolateFormationTargetLedger(
            request.committed,request.candidate,alpha);
        CHECK(gf::targetReferenceDistancesValid(
            ledger,request.reference_edges,849.0,1e-12));
    }
}

TEST_CASE("Task 10.11k cross-branch segment conflicts are not skipped") {
    auto request=baseRequest();
    request.committed_slots={{1,{0,0}},{2,{1,0}}};
    request.candidate_slots=request.committed_slots;
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    REQUIRE(audit.valid);
    CHECK(audit.selected_alpha==doctest::Approx(0.25));
    CHECK(audit.audited_cross_branch_pairs==1);
}

TEST_CASE("Task 10.11k target FIM planning score keeps positive and negative oracles") {
    gf::TargetGeometryGateRequest request;
    request.mobile_ids={1};
    request.fixed_ids={100,101};
    request.reference_edges={{100,1},{101,1}};
    request.targets={{1,{0.0,0.0}},{100,{1.0,0.0}},{101,{0.0,1.0}}};
    request.raw_targets={{1,{0.0,0.0}}};
    request.position_support_m={{1,0.0},{100,0.0},{101,0.0}};
    request.range_variances_m2={{"1--100",1.0},{"1--101",1.0}};
    request.edge_information_valid={{"100->1",true},{"101->1",true}};
    request.posterior_valid={{1,true}};
    request.minimum_collision_distance_m=0.1;
    request.minimum_target_fim_planning_score=0.5;
    request.minimum_target_cone_planning_score=0.5;
    request.maximum_target_deformation_m=1.0;
    CHECK(gf::evaluateTargetGeometryGates(request).valid);
    request.targets[101]={2.0,0.0};
    const auto rejected=gf::evaluateTargetGeometryGates(request);
    CHECK_FALSE(rejected.valid);
    CHECK(rejected.reason=="target_fim_planning_score");
}
