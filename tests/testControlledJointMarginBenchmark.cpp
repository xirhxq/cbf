#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "bridge/ControlledJointMarginBenchmark.hpp"

#include <array>

TEST_CASE("Controlled joint-margin low pressure retains the hard-QP command") {
    for (double lateralOffset : controlledJointMarginFrozenOffsets()) {
        const auto hard = runControlledJointMarginCell({
                ControlledJointMarginPressure::Low,
                ControlledJointMarginController::HardQp,
                lateralOffset,
        });
        const auto feedback = runControlledJointMarginCell({
                ControlledJointMarginPressure::Low,
                ControlledJointMarginController::MarginFeedback,
                lateralOffset,
        });

        REQUIRE(hard.valid);
        REQUIRE(feedback.valid);
        CHECK(hard.currentTaskFeasible);
        CHECK(feedback.currentTaskFeasible);
        CHECK(hard.command(0) == doctest::Approx(2.0).epsilon(1e-12));
        CHECK(hard.command(1) == doctest::Approx(0.0).epsilon(1e-12));
        CHECK(feedback.command(0) == doctest::Approx(hard.command(0)).epsilon(1e-12));
        CHECK(feedback.command(1) == doctest::Approx(hard.command(1)).epsilon(1e-12));
        CHECK(feedback.selectedCandidateIndex == 0);
        CHECK(hard.actualNextGamma > CONTROLLED_JOINT_MARGIN_TAU_MPS2);
        CHECK(feedback.actualNextGamma > CONTROLLED_JOINT_MARGIN_TAU_MPS2);
    }
}

TEST_CASE("Controlled joint-margin critical pressure exposes a joint-only conflict") {
    for (double lateralOffset : controlledJointMarginFrozenOffsets()) {
        const auto hard = runControlledJointMarginCell({
                ControlledJointMarginPressure::Critical,
                ControlledJointMarginController::HardQp,
                lateralOffset,
        });
        const auto feedback = runControlledJointMarginCell({
                ControlledJointMarginPressure::Critical,
                ControlledJointMarginController::MarginFeedback,
                lateralOffset,
        });

        REQUIRE(hard.valid);
        REQUIRE(feedback.valid);
        CHECK(hard.currentTaskFeasible);
        CHECK(hard.currentGamma > 0.0);
        CHECK(hard.nominalNextGamma < -1e-3);
        REQUIRE(hard.nominalNextSingletonGammas.size() == 2);
        CHECK(hard.nominalNextSingletonGammas.at(0) > 0.1);
        CHECK(hard.nominalNextSingletonGammas.at(1) > 0.1);
        CHECK(feedback.selectedCandidateIndex == 3);
        CHECK(feedback.command(0) < hard.command(0));
        CHECK(feedback.actualNextGamma > CONTROLLED_JOINT_MARGIN_TAU_MPS2);
        CHECK(feedback.predictedNextGamma
              == doctest::Approx(feedback.actualNextGamma).epsilon(1e-12));
    }
}

TEST_CASE("Controlled joint-margin contract rejects values outside the frozen matrix") {
    CHECK_THROWS_AS(
            runControlledJointMarginCell({
                    ControlledJointMarginPressure::Critical,
                    ControlledJointMarginController::MarginFeedback,
                    0.75,
            }),
            std::invalid_argument);
}

TEST_CASE("Controlled joint-margin JSON binds the complete cell and source fingerprint") {
    const auto result = runControlledJointMarginCell({
            ControlledJointMarginPressure::Critical,
            ControlledJointMarginController::MarginFeedback,
            0.0,
    });
    const std::string fingerprint(64, 'a');
    const auto payload = controlledJointMarginResultJson(result, fingerprint);

    CHECK(payload.at("schema") == "controlled-joint-margin-cell-v1");
    CHECK(payload.at("source_fingerprint_sha256") == fingerprint);
    CHECK(payload.at("specification").at("pressure") == "critical");
    CHECK(payload.at("specification").at("controller") == "margin-feedback");
    CHECK(payload.at("specification").at("lateral_offset_m") == 0.0);
    CHECK(payload.at("rows").size() == 2);
    CHECK(payload.at("candidates").size() == 9);
    CHECK(payload.at("selection").at("candidate_index") == 3);
    CHECK(payload.at("execution").at("actual_next_gamma_star_mps2").get<double>()
          > CONTROLLED_JOINT_MARGIN_TAU_MPS2);

    CHECK_THROWS_AS(
            controlledJointMarginResultJson(result, "not-a-sha256"),
            std::invalid_argument);
}
