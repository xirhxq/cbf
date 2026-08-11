#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "bridge/ExactGammaStar2D.hpp"
#include "bridge/FullRowPredictiveFeedback.hpp"
#include "bridge/FullRowPredictionAudit.hpp"
#include "bridge/LookaheadCollisionGate.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace {

double fourCornerGamma(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double halfBox) {
    double best = -std::numeric_limits<double>::infinity();
    for (double ax : {-halfBox, halfBox}) {
        for (double ay : {-halfBox, halfBox}) {
            double worst = std::numeric_limits<double>::infinity();
            for (const auto &residual : residuals) {
                worst = std::min(
                        worst,
                        residual.constant - residual.ax * ax - residual.ay * ay);
            }
            best = std::max(best, worst);
        }
    }
    return best;
}

double directGammaAt(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double accelX,
        double accelY) {
    double gamma = std::numeric_limits<double>::infinity();
    for (const auto &residual : residuals) {
        gamma = std::min(
                gamma,
                residual.constant
                        - residual.ax * accelX
                        - residual.ay * accelY);
    }
    return gamma;
}

}  // namespace

TEST_CASE("Affine-margin adapter preserves the physical acceleration sign") {
    const auto residual = bridgeGammaStarResidualFromAffineMargin(1.0, -2.0, 0.5);
    const auto valueAt = [&](double ax, double ay) {
        return residual.constant - residual.ax * ax - residual.ay * ay;
    };

    CHECK(valueAt(1.0, 0.5) == doctest::Approx(0.5));
    CHECK(valueAt(-1.0, 0.5) == doctest::Approx(-1.5));
}

TEST_CASE("Exact gamma-star keeps an interior optimum that corner scoring misses") {
    const std::vector<BridgeGammaStarResidual2D> residuals = {
            {1.0, 0.0, 0.5},
            {-1.0, 0.0, 0.5},
    };

    CHECK(exactBridgeGammaStar2D(residuals, 2.0) == doctest::Approx(0.5));
    CHECK(fourCornerGamma(residuals, 2.0) == doctest::Approx(-1.5));
}

TEST_CASE("Exact gamma-star reverses the corner-only candidate ranking") {
    const std::vector<BridgeGammaStarResidual2D> interiorCandidate = {
            {1.0, 0.0, 0.5},
            {-1.0, 0.0, 0.5},
    };
    const std::vector<BridgeGammaStarResidual2D> constantCandidate = {
            {0.0, 0.0, -0.2},
    };

    CHECK(exactBridgeGammaStar2D(interiorCandidate, 2.0)
          > exactBridgeGammaStar2D(constantCandidate, 2.0));
    CHECK(fourCornerGamma(interiorCandidate, 2.0)
          < fourCornerGamma(constantCandidate, 2.0));
}

TEST_CASE("Exact gamma-star returns a deterministic interior witness") {
    const std::vector<BridgeGammaStarResidual2D> residuals = {
            {1.0, 0.0, 0.5},
            {-1.0, 0.0, 0.5},
            {0.0, 1.0, 0.5},
            {0.0, -1.0, 0.5},
    };

    const auto solution = solveExactBridgeGammaStar2D(residuals, 2.0);
    REQUIRE(solution.valid);
    CHECK(solution.gamma == doctest::Approx(0.5).epsilon(1.0e-12));
    CHECK(solution.accelX == doctest::Approx(0.0).epsilon(1.0e-12));
    CHECK(solution.accelY == doctest::Approx(0.0).epsilon(1.0e-12));
    CHECK(solution.gamma == doctest::Approx(directGammaAt(
            residuals, solution.accelX, solution.accelY)).epsilon(1.0e-12));

    for (int repeat = 0; repeat < 10; ++repeat) {
        const auto repeated = solveExactBridgeGammaStar2D(residuals, 2.0);
        CHECK(repeated.valid);
        CHECK(repeated.accelX == solution.accelX);
        CHECK(repeated.accelY == solution.accelY);
    }
}

TEST_CASE("Exact gamma-star returns a box-corner witness") {
    const std::vector<BridgeGammaStarResidual2D> residuals = {
            {1.0, 2.0, 0.0},
    };

    const auto solution = solveExactBridgeGammaStar2D(residuals, 2.0);
    REQUIRE(solution.valid);
    CHECK(solution.gamma == doctest::Approx(6.0).epsilon(1.0e-12));
    CHECK(solution.accelX == doctest::Approx(-2.0).epsilon(1.0e-12));
    CHECK(solution.accelY == doctest::Approx(-2.0).epsilon(1.0e-12));
    CHECK(solution.gamma == doctest::Approx(directGammaAt(
            residuals, solution.accelX, solution.accelY)).epsilon(1.0e-12));
}

TEST_CASE("Exact gamma-star preserves a negative optimal budget and witness") {
    const std::vector<BridgeGammaStarResidual2D> residuals = {
            {1.0, 0.0, -0.5},
            {-1.0, 0.0, -0.5},
            {0.0, 1.0, -0.5},
            {0.0, -1.0, -0.5},
    };

    const auto solution = solveExactBridgeGammaStar2D(residuals, 1.0);
    REQUIRE(solution.valid);
    CHECK(solution.gamma == doctest::Approx(-0.5).epsilon(1.0e-12));
    CHECK(solution.accelX == doctest::Approx(0.0).epsilon(1.0e-12));
    CHECK(solution.accelY == doctest::Approx(0.0).epsilon(1.0e-12));
    CHECK(solution.gamma == doctest::Approx(directGammaAt(
            residuals, solution.accelX, solution.accelY)).epsilon(1.0e-12));
}

TEST_CASE("Exact gamma-star preserves sub-nanounit positive sign and corner witness") {
    const std::vector<BridgeGammaStarResidual2D> residuals = {
            {-1.0e-10, -1.0e-10, -2.0e-10},
    };

    const auto solution = solveExactBridgeGammaStar2D(residuals, 2.0);
    REQUIRE(solution.valid);
    CHECK(solution.gamma == doctest::Approx(2.0e-10).epsilon(1.0e-12));
    CHECK(solution.gamma > 0.0);
    CHECK(solution.accelX == doctest::Approx(2.0).epsilon(1.0e-12));
    CHECK(solution.accelY == doctest::Approx(2.0).epsilon(1.0e-12));
    CHECK(solution.gamma == directGammaAt(
            residuals, solution.accelX, solution.accelY));
}

TEST_CASE("Exact gamma-star handles empty, invalid, and degenerate inputs") {
    const auto empty = solveExactBridgeGammaStar2D({}, 1.0);
    CHECK(empty.valid);
    CHECK(std::isinf(empty.gamma));
    CHECK(empty.gamma > 0.0);
    CHECK(empty.accelX == 0.0);
    CHECK(empty.accelY == 0.0);

    const auto invalidBox = solveExactBridgeGammaStar2D({{1.0, 0.0, 0.0}}, -1.0);
    CHECK_FALSE(invalidBox.valid);
    CHECK(std::isnan(invalidBox.gamma));

    const auto invalidRow = solveExactBridgeGammaStar2D(
            {{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}, 1.0);
    CHECK_FALSE(invalidRow.valid);
    CHECK(std::isnan(invalidRow.gamma));

    const std::vector<BridgeGammaStarResidual2D> flat = {{0.0, 0.0, 0.25}};
    const auto tied = solveExactBridgeGammaStar2D(flat, 2.0);
    REQUIRE(tied.valid);
    CHECK(tied.gamma == doctest::Approx(0.25).epsilon(1.0e-12));
    CHECK(tied.accelX == 0.0);
    CHECK(tied.accelY == 0.0);
    CHECK(tied.gamma == doctest::Approx(directGammaAt(
            flat, tied.accelX, tied.accelY)).epsilon(1.0e-12));
}

TEST_CASE("Exact gamma-star restores defining box faces before feasibility") {
    const std::vector<BridgeGammaStarResidual2D> proposedResiduals = {
            bridgeGammaStarResidualFromAffineMargin(
                    0.8375422562964534, -0.5463725550554732,
                    554.2231530316814),
            bridgeGammaStarResidualFromAffineMargin(
                    0.9976223081857685, -0.06891828647100462,
                    368.8650824356202),
            bridgeGammaStarResidualFromAffineMargin(
                    0.8488154099432093, 0.5286893226110603,
                    284.4051706504333),
            bridgeGammaStarResidualFromAffineMargin(
                    -0.8375422562964534, 0.5463725550554732,
                    285.77684696831864),
            bridgeGammaStarResidualFromAffineMargin(
                    -0.9976223081857685, 0.06891828647100462,
                    471.1349175643798),
    };
    const std::vector<BridgeGammaStarResidual2D> legacyResiduals = {
            bridgeGammaStarResidualFromAffineMargin(
                    0.9117150985126489, -0.41082305089182963,
                    1359.3991374848165),
            bridgeGammaStarResidualFromAffineMargin(
                    0.8912821705424319, -0.4534491068170401,
                    1230.7858685651815),
            bridgeGammaStarResidualFromAffineMargin(
                    0.6746865276750176, -0.73810438921187,
                    698.5781111821115),
            bridgeGammaStarResidualFromAffineMargin(
                    -0.9117150985126489, 0.41082305089182963,
                    -519.3991374848166),
            bridgeGammaStarResidualFromAffineMargin(
                    -0.8912821705424319, 0.4534491068170401,
                    -390.78586856518154),
    };

    const auto proposed = solveExactBridgeGammaStar2D(proposedResiduals, 2.0);
    const auto legacy = solveExactBridgeGammaStar2D(legacyResiduals, 2.0);

    REQUIRE(proposed.valid);
    CHECK(proposed.gamma == doctest::Approx(286.170773666071).epsilon(1.0e-12));
    CHECK(proposed.accelX == doctest::Approx(0.8343679463394313).epsilon(1.0e-12));
    CHECK(proposed.accelY == doctest::Approx(2.0).epsilon(1.0e-12));
    REQUIRE(legacy.valid);
    CHECK(legacy.gamma == doctest::Approx(-516.7540611860077).epsilon(1.0e-12));
    CHECK(legacy.accelX == -2.0);
    CHECK(legacy.accelY == 2.0);
}

TEST_CASE("Full-row scoring takes the minimum over every predicted step") {
    std::map<int, BridgePredictionState2D> mobileStates = {
            {1, {Point(100.0, 0.0), Eigen::Vector2d(-5.0, 0.0),
                 Eigen::Vector2d::Zero()}},
    };
    std::map<int, Point> fixedBases;
    std::vector<BridgeFullRowDescriptor> rows;
    const std::vector<double> targets = {1.5, -0.25, 0.5, 2.0};
    const std::vector<double> curvature = {0.2, 2.0, 2.0, 2.0};

    for (int step = 1; step <= 4; ++step) {
        const double reciprocalSum = 5.0 - static_cast<double>(step);
        const double product = 2.0 * curvature[step - 1];
        const double sum = reciprocalSum * product;
        const double discriminant = std::sqrt(sum * sum - 4.0 * product);
        PairwiseSecondOrderRowSpec spec;
        spec.kind = PairwiseSecondOrderBarrierKind::CollisionLower;
        spec.distanceLimit = 10.0;
        spec.k = 1.0;
        spec.lambda1 = 0.5 * (sum + discriminant);
        spec.lambda2 = 0.5 * (sum - discriminant);

        const double desiredConstant =
                targets[step - 1] + curvature[step - 1] * step * step;
        const double baseX = 100.0 - spec.distanceLimit
                             - 5.0 * reciprocalSum
                             - (desiredConstant - 1.0) / product;
        fixedBases[step] = Point(baseX, 0.0);
        rows.push_back({
                "collision:base:" + std::to_string(step),
                1,
                BridgeReferenceKind::FixedBase,
                step,
                spec,
        });
    }

    const auto score = scoreFullRowCandidate(
            1, Eigen::Vector2d(1.0, 0.0),
            mobileStates, fixedBases, rows,
            1.0, 1.0, 4);

    REQUIRE(score.valid);
    REQUIRE(score.stepBudgets.size() == 4);
    for (size_t index = 0; index < targets.size(); ++index) {
        CHECK(score.stepBudgets[index]
              == doctest::Approx(targets[index]).epsilon(1.0e-10));
    }
    CHECK(score.minimumBudget == doctest::Approx(-0.25).epsilon(1.0e-10));
    CHECK(score.worstStep == 2);
    CHECK(score.dominantRow == "collision:base:2");
}

TEST_CASE("Full-row scoring preserves sub-nanounit strict minima") {
    std::map<int, BridgePredictionState2D> mobileStates = {
            {1, {Point(100.0, 0.0), Eigen::Vector2d(-5.0, 0.0),
                 Eigen::Vector2d::Zero()}},
    };
    std::map<int, Point> fixedBases;
    std::vector<BridgeFullRowDescriptor> rows;
    const std::vector<double> targets = {
            1.5, -0.25, -0.25 - 5.0e-10, 2.0};
    const std::vector<double> curvature = {0.2, 2.0, 2.0, 2.0};

    for (int step = 1; step <= 4; ++step) {
        const double reciprocalSum = 5.0 - static_cast<double>(step);
        const double product = 2.0 * curvature[step - 1];
        const double sum = reciprocalSum * product;
        const double discriminant = std::sqrt(sum * sum - 4.0 * product);
        PairwiseSecondOrderRowSpec spec;
        spec.kind = PairwiseSecondOrderBarrierKind::CollisionLower;
        spec.distanceLimit = 10.0;
        spec.k = 1.0;
        spec.lambda1 = 0.5 * (sum + discriminant);
        spec.lambda2 = 0.5 * (sum - discriminant);

        const double desiredConstant =
                targets[step - 1] + curvature[step - 1] * step * step;
        const double baseX = 100.0 - spec.distanceLimit
                             - 5.0 * reciprocalSum
                             - (desiredConstant - 1.0) / product;
        fixedBases[step] = Point(baseX, 0.0);
        rows.push_back({
                "collision:base:" + std::to_string(step),
                1,
                BridgeReferenceKind::FixedBase,
                step,
                spec,
        });
    }

    const auto score = scoreFullRowCandidate(
            1, Eigen::Vector2d(1.0, 0.0),
            mobileStates, fixedBases, rows,
            1.0, 1.0, 4);

    REQUIRE(score.valid);
    CHECK(score.minimumBudget == doctest::Approx(targets[2]).epsilon(1.0e-12));
    CHECK(score.worstStep == 3);
    CHECK(score.dominantRow == "collision:base:3");
}

TEST_CASE("Communication rows reverse a collision-only candidate ranking") {
    const std::map<int, BridgePredictionState2D> mobileStates = {
            {1, {Point(840.0, 0.0), Eigen::Vector2d::Zero(),
                 Eigen::Vector2d::Zero()}},
    };
    const std::map<int, Point> fixedBases = {{0, Point(0.0, 0.0)}};

    PairwiseSecondOrderRowSpec collision;
    collision.kind = PairwiseSecondOrderBarrierKind::CollisionLower;
    collision.distanceLimit = 10.0;
    collision.k = 1.0;
    collision.lambda1 = 1.0;
    collision.lambda2 = 1.0;
    PairwiseSecondOrderRowSpec communication = collision;
    communication.kind = PairwiseSecondOrderBarrierKind::CommunicationUpper;
    communication.distanceLimit = 850.0;

    const std::vector<BridgeFullRowDescriptor> collisionOnly = {
            {"collision:base:0", 1, BridgeReferenceKind::FixedBase, 0, collision},
    };
    auto fullRows = collisionOnly;
    fullRows.push_back({
            "communication:base:0", 1,
            BridgeReferenceKind::FixedBase, 0, communication});

    const auto collisionA = scoreFullRowCandidate(
            1, Eigen::Vector2d(1.0, 0.0), mobileStates, fixedBases,
            collisionOnly, 1.0, 1.0, 1);
    const auto collisionB = scoreFullRowCandidate(
            1, Eigen::Vector2d(-1.0, 0.0), mobileStates, fixedBases,
            collisionOnly, 1.0, 1.0, 1);
    const auto fullA = scoreFullRowCandidate(
            1, Eigen::Vector2d(1.0, 0.0), mobileStates, fixedBases,
            fullRows, 1.0, 1.0, 1);
    const auto fullB = scoreFullRowCandidate(
            1, Eigen::Vector2d(-1.0, 0.0), mobileStates, fixedBases,
            fullRows, 1.0, 1.0, 1);

    REQUIRE(collisionA.valid);
    REQUIRE(collisionB.valid);
    REQUIRE(fullA.valid);
    REQUIRE(fullB.valid);
    CHECK(collisionA.minimumBudget > collisionB.minimumBudget);
    CHECK(fullA.minimumBudget < fullB.minimumBudget);
    CHECK(fullA.dominantRow == "communication:base:0");
    CHECK(fullB.dominantRow == "communication:base:0");
}

TEST_CASE("Predictive rollout uses held inputs and exact ZOH at every step") {
    const std::map<int, BridgePredictionState2D> initial = {
            {1, {Point(1.0, -2.0), Eigen::Vector2d(3.0, -4.0),
                 Eigen::Vector2d(99.0, 99.0)}},
            {2, {Point(-3.0, 5.0), Eigen::Vector2d(0.5, -1.0),
                 Eigen::Vector2d(-0.25, 0.75)}},
    };
    const Eigen::Vector2d candidate(2.0, 6.0);
    const auto rollout = rolloutBridgePredictionStates(
            1, candidate, initial, 0.5, 3);
    REQUIRE(rollout.size() == 3);

    auto expectedSelfPosition = Eigen::Vector2d(1.0, -2.0);
    auto expectedSelfVelocity = Eigen::Vector2d(3.0, -4.0);
    auto expectedNeighbourPosition = Eigen::Vector2d(-3.0, 5.0);
    auto expectedNeighbourVelocity = Eigen::Vector2d(0.5, -1.0);
    for (size_t step = 0; step < rollout.size(); ++step) {
        const auto expectedSelf = propagateDoubleIntegratorPlanarZoh(
                expectedSelfPosition, expectedSelfVelocity, candidate, 0.5);
        const auto expectedNeighbour = propagateDoubleIntegratorPlanarZoh(
                expectedNeighbourPosition, expectedNeighbourVelocity,
                initial.at(2).heldAcceleration, 0.5);
        expectedSelfPosition = expectedSelf.position;
        expectedSelfVelocity = expectedSelf.velocity;
        expectedNeighbourPosition = expectedNeighbour.position;
        expectedNeighbourVelocity = expectedNeighbour.velocity;

        CHECK(rollout[step].at(1).position.x
              == doctest::Approx(expectedSelf.position(0)).epsilon(1.0e-12));
        CHECK(rollout[step].at(1).position.y
              == doctest::Approx(expectedSelf.position(1)).epsilon(1.0e-12));
        CHECK(rollout[step].at(1).velocity(0)
              == doctest::Approx(expectedSelf.velocity(0)).epsilon(1.0e-12));
        CHECK(rollout[step].at(2).position.x
              == doctest::Approx(expectedNeighbour.position(0)).epsilon(1.0e-12));
        CHECK(rollout[step].at(2).position.y
              == doctest::Approx(expectedNeighbour.position(1)).epsilon(1.0e-12));
        CHECK(rollout[step].at(2).velocity(1)
              == doctest::Approx(expectedNeighbour.velocity(1)).epsilon(1.0e-12));
    }

    const auto fixed = bridgeFixedBasePredictionState(Point(7.0, -8.0));
    CHECK(fixed.position.x == doctest::Approx(7.0));
    CHECK(fixed.position.y == doctest::Approx(-8.0));
    CHECK(fixed.velocity.isZero(0.0));
    CHECK(fixed.heldAcceleration.isZero(0.0));
}

TEST_CASE("Prediction audit resolves selected forecasts at the matching future sample") {
    const std::map<int, BridgePredictionState2D> initial = {
            {1, {Point(0.0, 0.0), Eigen::Vector2d(1.0, 0.0),
                 Eigen::Vector2d::Zero()}},
            {2, {Point(30.0, 0.0), Eigen::Vector2d::Zero(),
                 Eigen::Vector2d::Zero()}},
    };
    PairwiseSecondOrderRowSpec collision;
    collision.kind = PairwiseSecondOrderBarrierKind::CollisionLower;
    collision.distanceLimit = 10.0;
    collision.k = 1.0;
    collision.lambda1 = 1.0;
    collision.lambda2 = 1.0;
    const std::vector<BridgeFullRowDescriptor> rows = {
            {"collision:2", 1, BridgeReferenceKind::Mobile, 2, collision},
    };
    const Eigen::Vector2d candidate(0.5, 0.0);
    const auto rollout = rolloutBridgePredictionStates(
            1, candidate, initial, 0.2, 2);
    REQUIRE(rollout.size() == 2);

    std::vector<double> predictedBudgets;
    for (const auto &predicted : rollout) {
        const auto evaluated = evaluateBridgeFullRows(1, predicted, {}, rows);
        REQUIRE(evaluated.size() == 1);
        const auto solution = solveExactBridgeGammaStar2D(
                {bridgeGammaStarResidualFromAffineMargin(
                        evaluated[0].row.uCoe(0),
                        evaluated[0].row.uCoe(1),
                        evaluated[0].row.constTerm)},
                2.0);
        REQUIRE(solution.valid);
        predictedBudgets.push_back(solution.gamma);
    }

    auto pending = buildBridgePredictionAuditEntries(
            7, 1.4, 1, 0.2, rollout, predictedBudgets);
    REQUIRE(pending.size() == 2);
    CHECK(pending[0].originStep == 7);
    CHECK(pending[0].dueStep == 8);
    CHECK(pending[1].dueStep == 9);

    const auto early = resolveBridgePredictionAudits(
            7, 1.4, initial, {}, rows, 2.0, pending);
    CHECK(early.empty());
    CHECK(pending.size() == 2);

    const auto exact = resolveBridgePredictionAudits(
            8, 1.6, rollout[0], {}, rows, 2.0, pending);
    REQUIRE(exact.size() == 1);
    REQUIRE(exact[0].valid);
    CHECK(exact[0].robotId == 1);
    CHECK(exact[0].horizonStep == 1);
    CHECK(exact[0].maxPositionError == doctest::Approx(0.0));
    CHECK(exact[0].maxVelocityError == doctest::Approx(0.0));
    CHECK(exact[0].maxAccelerationError == doctest::Approx(0.0));
    CHECK(exact[0].actualBudget
          == doctest::Approx(exact[0].predictedBudget).epsilon(1.0e-12));
    CHECK(exact[0].budgetError == doctest::Approx(0.0).epsilon(1.0e-12));
    REQUIRE(exact[0].stateErrors.size() == 2);
    CHECK(pending.size() == 1);

    auto mismatched = rollout[1];
    mismatched.at(2).position.x += 0.25;
    mismatched.at(2).heldAcceleration.x() += 0.1;
    const auto observed = resolveBridgePredictionAudits(
            9, 1.8, mismatched, {}, rows, 2.0, pending);
    REQUIRE(observed.size() == 1);
    REQUIRE(observed[0].valid);
    CHECK(observed[0].maxPositionError == doctest::Approx(0.25));
    CHECK(observed[0].maxAccelerationError == doctest::Approx(0.1));
    CHECK(std::abs(observed[0].budgetError) > 0.0);
    CHECK(pending.empty());
}

TEST_CASE("Joint task forecast overlays every mobile acceleration synchronously") {
    const std::map<int, BridgePredictionState2D> observed = {
        {1, {Point(10.0, 20.0), Eigen::Vector2d(1.0, 2.0),
             Eigen::Vector2d(0.1, 0.2)}},
        {2, {Point(30.0, 40.0), Eigen::Vector2d(3.0, 4.0),
             Eigen::Vector2d(0.3, 0.4)}},
    };
    const std::map<int, Eigen::Vector2d> taskAccelerations = {
        {1, Eigen::Vector2d(-1.0, 1.5)},
        {2, Eigen::Vector2d(2.0, -0.5)},
    };

    const auto forecast = bridgePredictionStatesWithAccelerations(
        observed,
        taskAccelerations);

    REQUIRE(forecast.size() == observed.size());
    CHECK(forecast.at(1).position.distance_to(observed.at(1).position) == 0.0);
    CHECK(forecast.at(2).velocity == observed.at(2).velocity);
    CHECK(forecast.at(1).heldAcceleration == taskAccelerations.at(1));
    CHECK(forecast.at(2).heldAcceleration == taskAccelerations.at(2));
    CHECK(observed.at(1).heldAcceleration != forecast.at(1).heldAcceleration);

    auto missing = taskAccelerations;
    missing.erase(2);
    CHECK_THROWS_AS(
        bridgePredictionStatesWithAccelerations(observed, missing),
        std::invalid_argument);
}

TEST_CASE("Prediction audit rejects overdue and time-mismatched forecasts") {
    const std::map<int, BridgePredictionState2D> states = {
            {1, {Point(0.0, 0.0), Eigen::Vector2d::Zero(),
                 Eigen::Vector2d::Zero()}},
    };
    PairwiseSecondOrderRowSpec collision;
    collision.kind = PairwiseSecondOrderBarrierKind::CollisionLower;
    collision.distanceLimit = 10.0;
    collision.k = 1.0;
    collision.lambda1 = 1.0;
    collision.lambda2 = 1.0;
    const std::map<int, Point> bases = {{0, Point(20.0, 0.0)}};
    const std::vector<BridgeFullRowDescriptor> rows = {
            {"collision:base", 1, BridgeReferenceKind::FixedBase, 0, collision},
    };

    auto overdue = buildBridgePredictionAuditEntries(
            0, 0.0, 1, 0.5, {states}, {1.0});
    REQUIRE(overdue.size() == 1);
    const auto late = resolveBridgePredictionAudits(
            2, 1.0, states, bases, rows, 2.0, overdue);
    REQUIRE(late.size() == 1);
    CHECK_FALSE(late[0].valid);
    CHECK(late[0].error == "prediction audit entry is overdue");
    CHECK(overdue.empty());

    auto wrongTime = buildBridgePredictionAuditEntries(
            0, 0.0, 1, 0.5, {states}, {1.0});
    REQUIRE(wrongTime.size() == 1);
    const auto mismatch = resolveBridgePredictionAudits(
            1, 0.5 + 1.0e-6, states, bases, rows, 2.0, wrongTime);
    REQUIRE(mismatch.size() == 1);
    CHECK_FALSE(mismatch[0].valid);
    CHECK(mismatch[0].error
          == "prediction audit time does not match its forecast horizon");
    CHECK(wrongTime.empty());
}

TEST_CASE("Lookahead-distance gate recognizes a closing predicted neighbour") {
    const std::vector<BridgeLookaheadNeighbourState2D> neighbours = {
        {100.0, 0.0, -1.0, 0.0},
    };

    CHECK(bridgeLookaheadDistanceClosingTrigger(
            0.0, 0.0, 0.0, 0.0, neighbours, 150.0));
}

TEST_CASE("Lookahead-distance gate rejects an opening predicted neighbour") {
    const std::vector<BridgeLookaheadNeighbourState2D> neighbours = {
        {100.0, 0.0, 1.0, 0.0},
    };

    CHECK_FALSE(bridgeLookaheadDistanceClosingTrigger(
            0.0, 0.0, 0.0, 0.0, neighbours, 150.0));
}

TEST_CASE("Lookahead-distance gate scans beyond an opening in-range neighbour") {
    const std::vector<BridgeLookaheadNeighbourState2D> neighbours = {
        {50.0, 0.0, 1.0, 0.0},
        {100.0, 0.0, -1.0, 0.0},
    };

    CHECK(bridgeLookaheadDistanceClosingTrigger(
            0.0, 0.0, 0.0, 0.0, neighbours, 150.0));
}
