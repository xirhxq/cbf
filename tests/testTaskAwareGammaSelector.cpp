#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "bridge/TaskAwareGammaSelector.hpp"

#include <vector>

TEST_CASE("Task-aware gamma selector retains the nominal task control when it has reserve") {
    const std::vector<BridgeTaskAwareGammaCandidate> candidates = {
            {1.0, 0.0, 0.25},
            {0.0, 1.0, 0.80},
    };

    const auto selection = bridgeSelectTaskAwareGammaCandidate(
            candidates, 1.0, 0.0, 0.0);

    CHECK(selection.selected);
    CHECK(selection.reserveSatisfied);
    CHECK(selection.nominalRetained);
    CHECK(selection.accelX == doctest::Approx(1.0));
    CHECK(selection.accelY == doctest::Approx(0.0));
    CHECK(selection.taskDeviation == doctest::Approx(0.0));
}

TEST_CASE("Task-aware gamma selector minimizes task deviation within the safe candidate set") {
    const std::vector<BridgeTaskAwareGammaCandidate> candidates = {
            {1.0, 0.0, -0.20},
            {0.20, 0.0, 0.10},
            {0.00, 1.0, 0.80},
    };

    const auto selection = bridgeSelectTaskAwareGammaCandidate(
            candidates, 1.0, 0.0, 0.0);

    CHECK(selection.selected);
    CHECK(selection.reserveSatisfied);
    CHECK_FALSE(selection.nominalRetained);
    CHECK(selection.accelX == doctest::Approx(0.20));
    CHECK(selection.accelY == doctest::Approx(0.0));
    CHECK(selection.taskDeviation == doctest::Approx(0.80));
}

TEST_CASE("Task-aware gamma selector falls back to the largest reserve when no candidate is safe") {
    const std::vector<BridgeTaskAwareGammaCandidate> candidates = {
            {1.0, 0.0, -0.70},
            {0.20, 0.0, -0.10},
            {0.00, 1.0, -0.25},
    };

    const auto selection = bridgeSelectTaskAwareGammaCandidate(
            candidates, 1.0, 0.0, 0.0);

    CHECK(selection.selected);
    CHECK_FALSE(selection.reserveSatisfied);
    CHECK(selection.accelX == doctest::Approx(0.20));
    CHECK(selection.accelY == doctest::Approx(0.0));
}
