#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/FormationConsistentTargetGovernor.hpp"
#include "grand_finale/Task10p11jCollisionCausalFixture.hpp"

namespace {

std::map<gf::NodeId,gf::FormationTargetSlotIdentity> frozenSlots() {
    std::map<gf::NodeId,gf::FormationTargetSlotIdentity> result;
    const auto branches=gf::task10p11hLeaderCoverageSpec();
    for (std::size_t branch=0;branch<branches.size();++branch)
        for (std::size_t rank=0;rank<branches[branch].members.size();++rank)
            result[branches[branch].members[rank]]={branch,rank};
    return result;
}

gf::FormationTargetLedger centers(
    const std::map<gf::NodeId,gf::FrontierCell>& targets) {
    gf::FormationTargetLedger result;
    for (const auto& [owner,target] : targets) result[owner]=target.center;
    return result;
}

}  // namespace

TEST_CASE("Task 10.11k frozen 4.0 target change passes pure target gates at alpha one") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    const auto before=std::find_if(replay.ticks.begin(),replay.ticks.end(),
        [](const auto& tick) { return std::abs(tick.time_before_s-3.9)<1e-12; });
    const auto changed=std::find_if(replay.ticks.begin(),replay.ticks.end(),
        [](const auto& tick) { return std::abs(tick.time_before_s-4.0)<1e-12; });
    REQUIRE(before!=replay.ticks.end());
    REQUIRE(changed!=replay.ticks.end());
    REQUIRE(changed->target_changed);
    gf::FormationTargetTransitionRequest request;
    for (gf::NodeId owner=1;owner<=14;++owner) request.mobile_ids.push_back(owner);
    request.committed=centers(before->raw_targets_after);
    request.candidate=centers(changed->raw_targets_after);
    request.committed_slots=frozenSlots();
    request.candidate_slots=request.committed_slots;
    request.fixed_targets=gf::task10p11hCoastalAnchors();
    request.reference_edges=changed->topology;
    request.reference_plan_distance_m=849.0;
    request.minimum_segment_separation_m=0.1;
    request.minimum_endpoint_separation_m=0.1;
    const auto audit=gf::selectFormationConsistentTargetTransition(request);
    double global_min=std::numeric_limits<double>::infinity();
    gf::NodeId global_first=0;
    gf::NodeId global_second=0;
    double owners_10_11_min=std::numeric_limits<double>::infinity();
    for (std::size_t first=0;first<request.mobile_ids.size();++first) {
        for (std::size_t second=first+1;second<request.mobile_ids.size();++second) {
            const gf::NodeId first_id=request.mobile_ids[first];
            const gf::NodeId second_id=request.mobile_ids[second];
            const double separation=gf::minimumTargetTransitionSeparation(
                request.committed.at(first_id),request.candidate.at(first_id),
                request.committed.at(second_id),request.candidate.at(second_id)).distance_m;
            if (separation<global_min) {
                global_min=separation;
                global_first=first_id;
                global_second=second_id;
            }
            if (first_id==10 && second_id==11) owners_10_11_min=separation;
        }
    }
    INFO(audit.reason);
    REQUIRE(audit.valid);
    MESSAGE("TASK10P11K_K2 alpha=",audit.selected_alpha,
        " min_segment_m=",audit.minimum_segment_separation_m,
        " min_endpoint_m=",audit.minimum_endpoint_separation_m,
        " same_branch_pairs=",audit.audited_same_branch_pairs,
        " cross_branch_pairs=",audit.audited_cross_branch_pairs,
        " consequence=frozen_jump_not_decomposed");
    MESSAGE("TASK10P11L_A global_pair=",global_first,"--",global_second,
        " global_segment_min_m=",global_min,
        " owners_10_11_segment_min_m=",owners_10_11_min,
        " hard_collision_center_distance_m=0.1");
    CHECK(audit.selected_alpha==doctest::Approx(1.0));
    CHECK(audit.minimum_segment_separation_m>0.1);
    CHECK(audit.minimum_endpoint_separation_m>0.1);
    CHECK(replay.failure_time_s==doctest::Approx(4.5));
    CHECK(replay.ticks.back().first.current_gamma==
          doctest::Approx(-0.0454972).epsilon(1e-5));
}

TEST_CASE("Task 10.11k preserves the frozen causal RED evidence") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    REQUIRE(replay.t100_s.has_value());
    CHECK(*replay.t100_s==doctest::Approx(3.9));
    const auto changed=std::find_if(replay.ticks.begin(),replay.ticks.end(),
        [](const auto& tick) { return std::abs(tick.time_before_s-4.0)<1e-12; });
    REQUIRE(changed!=replay.ticks.end());
    CHECK(changed->target_changed);
    CHECK(changed->nominal_target_change_effect_mps2>9.0);
    CHECK(replay.failure_time_s==doctest::Approx(4.5));
    CHECK(replay.reason=="current_gamma_negative");
}

TEST_CASE("Task 10.11l reclassifies the raw 4.0 target transition at ten metres") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    const auto before=std::find_if(replay.ticks.begin(),replay.ticks.end(),
        [](const auto& tick) { return std::abs(tick.time_before_s-3.9)<1e-12; });
    const auto changed=std::find_if(replay.ticks.begin(),replay.ticks.end(),
        [](const auto& tick) { return std::abs(tick.time_before_s-4.0)<1e-12; });
    REQUIRE(before!=replay.ticks.end());
    REQUIRE(changed!=replay.ticks.end());
    const auto committed=centers(before->raw_targets_after);
    const auto candidate=centers(changed->raw_targets_after);
    const auto pair_6_8=gf::minimumTargetTransitionSeparation(
        committed.at(6),candidate.at(6),committed.at(8),candidate.at(8));
    const auto pair_10_11=gf::minimumTargetTransitionSeparation(
        committed.at(10),candidate.at(10),committed.at(11),candidate.at(11));
    CHECK(pair_6_8.distance_m==doctest::Approx(4.73034).epsilon(1e-5));
    CHECK(pair_6_8.distance_m<10.0);
    CHECK(pair_10_11.distance_m==doctest::Approx(108.803).epsilon(1e-5));
    CHECK(pair_10_11.distance_m>10.0);
    MESSAGE("TASK10P11L_TARGET_REJECTION pair=6--8 minimum_m=",
        pair_6_8.distance_m," required_m=10 classification=target_transition_rejected");
    MESSAGE("TASK10P11L_SEPARATE_FINITE_INPUT pair=10--11 minimum_m=",
        pair_10_11.distance_m," failure_time_s=",replay.failure_time_s);
}
