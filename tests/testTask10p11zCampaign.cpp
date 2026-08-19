#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11zCampaign.hpp"

namespace {

std::filesystem::path checkpoint(const std::string& relative) {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/relative;
}
}  // namespace

TEST_CASE("all profiles are frozen before the first trajectory") {
    const auto protocol=gf::task10p11zPreregisteredCampaign();
    CHECK(protocol.profile_order==
        std::vector<std::string>{"R0","R1","R2","R3","R4"});
    CHECK(protocol.r0_tau_mps2==doctest::Approx(14.0));
    CHECK(protocol.r1_tau_mps2==doctest::Approx(14.0));
    CHECK(protocol.r2_tau_mps2==doctest::Approx(18.0));
    CHECK(protocol.r3_tau_mps2==doctest::Approx(14.0));
    CHECK(protocol.candidate_count==9);
    CHECK(protocol.component_size_limit==3);
    CHECK(protocol.gate1_component==std::set<gf::NodeId>{2,4,6});
    CHECK(protocol.r4_status=="offline_rejected");
    CHECK(protocol.gamma_star_is_diagnostic_not_parameter);
}

TEST_CASE("R1 uses nine scalar candidates and never a 4D endpoint") {
    const auto snapshot=gf::readTask10p11vJson(checkpoint(
        "docs/evidence/task10p11v-unique-recapture/checkpoints/"
        "checkpoint-004-t132.8-dynamic_intervention.json"));
    const auto request=gf::task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto rows=gf::buildCanonicalHardRows(request);
    const auto nominal=gf::task10p11s_capture_detail::nominalFromJson(
        snapshot.at("nominal_controls"));
    const auto baseline=gf::solveTask10p11tDistributedLocalStep(
        rows,request.mobile_ids,nominal,request.acceleration_half_box,
        gf::task10p11y_detail::kPair);
    REQUIRE(baseline.feasible);
    const auto decision=gf::task10p11y_detail::decideWithPreparedBaseline(
        snapshot,baseline.controls,baseline.pair.selected_transfer_mps2,false);
    INFO(decision.reason);
    REQUIRE(decision.valid);
    REQUIRE(decision.preventive_trigger);
    CHECK(decision.candidates.size()==9);
    CHECK(decision.current_pair.shared_interval.feasible);
}

TEST_CASE("R3 selects the Gate 1 certificate-minimum component at 133.0") {
    const auto snapshot=gf::readTask10p11vJson(checkpoint(
        "docs/evidence/fixed-baseline-recovery-campaign/stage-b/"
        "pair-2-4-component/checkpoints/"
        "checkpoint-0001-t133.0-fail_closed.json"));
    const auto frame=gf::task10p11w_detail::readFrame(checkpoint(
        "docs/evidence/fixed-baseline-recovery-campaign/stage-b/"
        "pair-2-4-component/checkpoints/"
        "checkpoint-0001-t133.0-fail_closed.json"));
    const auto baseline=gf::task10p11sControlMap(
        frame.request.mobile_ids,frame.frozen);
    const auto decision=gf::task10p11zDecideAdaptiveComponent(
        snapshot,baseline,3);
    INFO(decision.reason);
    REQUIRE(decision.valid);
    CHECK(decision.use_component);
    CHECK(decision.component==std::set<gf::NodeId>{2,4,6});
    CHECK(decision.current_full_residual_mps2>=-1.0e-8);
    CHECK(decision.successor_full_residual_mps2>=-1.0e-8);
}
