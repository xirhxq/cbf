#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "bridge/ReserveTaskHomotopy.hpp"

#include <limits>
#include <vector>

TEST_CASE("Reserve-to-task homotopy includes both endpoints") {
    const auto candidates = buildReserveTaskHomotopy(
            1.0, -2.0, 5.0, 2.0, 4);

    REQUIRE(candidates.size() == 5);
    for (size_t index = 0; index < candidates.size(); ++index) {
        const double alpha = static_cast<double>(index) / 4.0;
        CHECK(candidates[index].alpha == doctest::Approx(alpha));
        CHECK(candidates[index].accelX == doctest::Approx(1.0 + 4.0 * alpha));
        CHECK(candidates[index].accelY == doctest::Approx(-2.0 + 4.0 * alpha));
    }
}

TEST_CASE("Reserve-to-task homotopy rejects invalid construction inputs") {
    CHECK(buildReserveTaskHomotopy(0.0, 0.0, 1.0, 1.0, 0).empty());
    CHECK(buildReserveTaskHomotopy(
            std::numeric_limits<double>::quiet_NaN(), 0.0,
            1.0, 1.0, 4).empty());
    CHECK(buildReserveTaskHomotopy(
            0.0, 0.0,
            std::numeric_limits<double>::infinity(), 1.0, 4).empty());
}

TEST_CASE("Homotopy selection retains the filtered task execution when safe") {
    auto candidates = buildReserveTaskHomotopy(1.0, 0.0, -1.0, 0.0, 4);
    REQUIRE(candidates.size() == 5);
    candidates[0].predictedBudget = 0.25;
    candidates[1].predictedBudget = 0.80;
    candidates[2].predictedBudget = 1.20;
    candidates[3].predictedBudget = 1.40;
    candidates[4].predictedBudget = 1.50;

    const auto selection = selectReserveTaskHomotopy(
            candidates, 1.0, 0.0, 0.0);

    CHECK(selection.selected);
    CHECK(selection.gateSatisfied);
    CHECK(selection.nominalRetained);
    CHECK(selection.alpha == doctest::Approx(0.0));
    CHECK(selection.accelX == doctest::Approx(1.0));
    CHECK(selection.accelY == doctest::Approx(0.0));
    CHECK(selection.taskDeviation == doctest::Approx(0.0));
}

TEST_CASE("Homotopy selection uses the least alpha that satisfies the gate") {
    auto candidates = buildReserveTaskHomotopy(1.0, 0.0, -1.0, 0.0, 4);
    REQUIRE(candidates.size() == 5);
    candidates[0].predictedBudget = -0.40;
    candidates[1].predictedBudget = -0.10;
    candidates[2].predictedBudget = 0.05;
    candidates[3].predictedBudget = 0.80;
    candidates[4].predictedBudget = 1.50;

    const auto selection = selectReserveTaskHomotopy(
            candidates, 1.0, 0.0, 0.0);

    CHECK(selection.selected);
    CHECK(selection.gateSatisfied);
    CHECK_FALSE(selection.nominalRetained);
    CHECK(selection.alpha == doctest::Approx(0.5));
    CHECK(selection.accelX == doctest::Approx(0.0));
    CHECK(selection.candidateIndex == 2);
    CHECK(selection.taskDeviation == doctest::Approx(1.0));
}

TEST_CASE("Homotopy selection falls back to maximum score then minimum alpha") {
    const std::vector<BridgeReserveTaskCandidate> candidates = {
            {0.75, -0.5, 0.0, -0.10},
            {0.25, 0.5, 0.0, -0.10},
            {0.50, 0.0, 0.0, -0.40},
    };

    const auto selection = selectReserveTaskHomotopy(
            candidates, 1.0, 0.0, 0.0);

    CHECK(selection.selected);
    CHECK_FALSE(selection.gateSatisfied);
    CHECK(selection.alpha == doctest::Approx(0.25));
    CHECK(selection.accelX == doctest::Approx(0.5));
    CHECK(selection.predictedBudget == doctest::Approx(-0.10));
    CHECK(selection.selectedMaximumPredictedBudget);
    CHECK(selection.maximumCandidateIndex == 1);
    CHECK(selection.maximumPredictedBudget == doctest::Approx(-0.10));
}

TEST_CASE("Homotopy gate uses the exact budget sign") {
    const std::vector<BridgeReserveTaskCandidate> candidates = {
            {0.00, 1.0, 0.0, -5.0e-10},
            {0.25, 0.5, 0.0, 0.0},
    };

    const auto selection = selectReserveTaskHomotopy(
            candidates, 1.0, 0.0, 0.0, 1.0e-9);

    REQUIRE(selection.selected);
    CHECK(selection.gateSatisfied);
    CHECK(selection.alpha == doctest::Approx(0.25));
    CHECK(selection.predictedBudget == doctest::Approx(0.0));
}

TEST_CASE("Homotopy fallback preserves every strict budget ordering") {
    const std::vector<BridgeReserveTaskCandidate> candidates = {
            {0.00, 1.0, 0.0, -1.0},
            {0.25, 0.5, 0.0, -1.0 + 5.0e-10},
    };

    const auto selection = selectReserveTaskHomotopy(
            candidates, 1.0, 0.0, 0.0, 1.0e-9);

    REQUIRE(selection.selected);
    CHECK_FALSE(selection.gateSatisfied);
    CHECK(selection.alpha == doctest::Approx(0.25));
    CHECK(selection.predictedBudget == doctest::Approx(-1.0 + 5.0e-10));
    CHECK(selection.selectedMaximumPredictedBudget);
    CHECK(selection.maximumCandidateIndex == 1);
    CHECK(selection.maximumPredictedBudget
          == doctest::Approx(-1.0 + 5.0e-10));
}

TEST_CASE("Homotopy selection breaks exact duplicates lexicographically") {
    const std::vector<BridgeReserveTaskCandidate> candidates = {
            {0.25, 0.5, 0.5, 0.20},
            {0.25, 0.5, -0.5, 0.20},
            {0.25, -0.5, 0.5, 0.20},
    };

    const auto selection = selectReserveTaskHomotopy(
            candidates, 0.0, 0.0, 0.0);

    CHECK(selection.selected);
    CHECK(selection.gateSatisfied);
    CHECK(selection.alpha == doctest::Approx(0.25));
    CHECK(selection.accelX == doctest::Approx(-0.5));
    CHECK(selection.accelY == doctest::Approx(0.5));
}

TEST_CASE("Homotopy selection rejects invalid selection inputs") {
    const std::vector<BridgeReserveTaskCandidate> candidates = {
            {0.0, 0.0, 0.0, 0.0},
    };

    CHECK_FALSE(selectReserveTaskHomotopy(
            candidates,
            std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0).selected);
    CHECK_FALSE(selectReserveTaskHomotopy(
            candidates, 0.0, 0.0,
            std::numeric_limits<double>::infinity()).selected);
    CHECK_FALSE(selectReserveTaskHomotopy({}, 0.0, 0.0, 0.0).selected);
}

TEST_CASE("Maximum-reserve ablation selects the gamma-witness endpoint") {
    auto candidates = buildReserveTaskHomotopy(
            1.0, 0.0, -1.0, 0.0, 8);
    REQUIRE(candidates.size() == 9);
    for (size_t index = 0; index < candidates.size(); ++index) {
        candidates[index].predictedBudget =
                index == candidates.size() - 1 ? -0.25 : 1.0;
    }

    const auto selected = selectMaximumReserveHomotopy(
            candidates, 1.0, 0.0, 0.0, 1.0e-9);

    REQUIRE(selected.selected);
    CHECK_FALSE(selected.gateSatisfied);
    CHECK_FALSE(selected.nominalRetained);
    CHECK(selected.alpha == doctest::Approx(1.0));
    CHECK(selected.candidateIndex == 8);
    CHECK(selected.accelX == doctest::Approx(-1.0));
    CHECK(selected.predictedBudget == doctest::Approx(-0.25));
    CHECK(selected.taskDeviation == doctest::Approx(2.0));
    CHECK_FALSE(selected.selectedMaximumPredictedBudget);
    CHECK(selected.maximumCandidateIndex == 0);
    CHECK(selected.maximumPredictedBudget == doctest::Approx(1.0));
}
