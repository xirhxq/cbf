#ifndef CBF_BRIDGE_FULL_ROW_PREDICTIVE_FEEDBACK_HPP
#define CBF_BRIDGE_FULL_ROW_PREDICTIVE_FEEDBACK_HPP

#include "bridge/ExactGammaStar2D.hpp"
#include "cbf/PairwiseSecondOrderCBF.hpp"
#include "models/DoubleIntegrate2D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

enum class BridgeReferenceKind {
    Mobile,
    FixedBase
};

struct BridgePredictionState2D {
    Point position;
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d heldAcceleration = Eigen::Vector2d::Zero();
};

struct BridgeFullRowDescriptor {
    std::string identity;
    int ownerRobotId = 0;
    BridgeReferenceKind referenceKind = BridgeReferenceKind::Mobile;
    int referenceId = 0;
    PairwiseSecondOrderRowSpec spec;
};

struct BridgeFullRowScore {
    bool valid = false;
    double minimumBudget = -std::numeric_limits<double>::infinity();
    int worstStep = 0;
    std::string dominantRow;
    std::vector<double> stepBudgets;
    std::vector<std::string> stepDominantRows;
};

struct BridgeEvaluatedFullRow2D {
    std::string identity;
    PairwiseSecondOrderRow row;
};

inline BridgePredictionState2D bridgeFixedBasePredictionState(
        const Point &position) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
        throw std::invalid_argument("fixed-base prediction position must be finite");
    }
    return {position, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
}

inline std::vector<std::map<int, BridgePredictionState2D>>
rolloutBridgePredictionStates(
        int robotId,
        const Eigen::Vector2d &candidateAcceleration,
        const std::map<int, BridgePredictionState2D> &initialStates,
        double dt,
        int lookaheadSteps) {
    if (robotId <= 0 || initialStates.find(robotId) == initialStates.end()
        || !candidateAcceleration.allFinite()
        || !std::isfinite(dt) || dt <= 0.0
        || lookaheadSteps <= 0) {
        return {};
    }

    auto states = initialStates;
    for (auto &[id, state] : states) {
        (void) id;
        if (!std::isfinite(state.position.x) || !std::isfinite(state.position.y)
            || !state.velocity.allFinite()
            || !state.heldAcceleration.allFinite()) {
            return {};
        }
    }
    states.at(robotId).heldAcceleration = candidateAcceleration;

    std::vector<std::map<int, BridgePredictionState2D>> rollout;
    rollout.reserve(static_cast<size_t>(lookaheadSteps));
    for (int step = 0; step < lookaheadSteps; ++step) {
        for (auto &[id, state] : states) {
            (void) id;
            const Eigen::Vector2d position(state.position.x, state.position.y);
            const auto next = propagateDoubleIntegratorPlanarZoh(
                    position,
                    state.velocity,
                    state.heldAcceleration,
                    dt);
            state.position = Point(next.position(0), next.position(1));
            state.velocity = next.velocity;
        }
        rollout.push_back(states);
    }
    return rollout;
}

inline std::vector<BridgeEvaluatedFullRow2D> evaluateBridgeFullRows(
        int robotId,
        const std::map<int, BridgePredictionState2D> &mobileStates,
        const std::map<int, Point> &fixedBases,
        const std::vector<BridgeFullRowDescriptor> &rows) {
    const auto selfIt = mobileStates.find(robotId);
    if (selfIt == mobileStates.end()) {
        throw std::invalid_argument("full-row owner state is missing");
    }

    std::vector<BridgeEvaluatedFullRow2D> evaluated;
    for (const auto &descriptor : rows) {
        if (descriptor.ownerRobotId != robotId) {
            continue;
        }

        BridgePredictionState2D reference;
        if (descriptor.referenceKind == BridgeReferenceKind::Mobile) {
            const auto referenceIt = mobileStates.find(descriptor.referenceId);
            if (referenceIt == mobileStates.end()) {
                throw std::invalid_argument("full-row mobile reference is missing");
            }
            reference = referenceIt->second;
        } else {
            const auto referenceIt = fixedBases.find(descriptor.referenceId);
            if (referenceIt == fixedBases.end()) {
                throw std::invalid_argument("full-row fixed-base reference is missing");
            }
            reference = bridgeFixedBasePredictionState(referenceIt->second);
        }

        const PairwiseSecondOrderState2D self{
                selfIt->second.position,
                selfIt->second.velocity,
                selfIt->second.heldAcceleration,
        };
        const PairwiseSecondOrderState2D neighbour{
                reference.position,
                reference.velocity,
                reference.heldAcceleration,
        };
        evaluated.push_back({
                descriptor.identity,
                buildPairwiseSecondOrderRow(self, neighbour, descriptor.spec),
        });
    }
    if (evaluated.empty()) {
        throw std::invalid_argument("full-row owner has no descriptors");
    }
    return evaluated;
}

inline BridgeFullRowScore scoreFullRowCandidate(
        int robotId,
        const Eigen::Vector2d &candidateAcceleration,
        const std::map<int, BridgePredictionState2D> &mobileStates,
        const std::map<int, Point> &fixedBases,
        const std::vector<BridgeFullRowDescriptor> &rows,
        double accelerationBound,
        double dt,
        int lookaheadSteps) {
    BridgeFullRowScore score;
    if (robotId <= 0
        || !candidateAcceleration.allFinite()
        || !std::isfinite(accelerationBound) || accelerationBound < 0.0
        || std::abs(candidateAcceleration(0)) > accelerationBound + 1.0e-9
        || std::abs(candidateAcceleration(1)) > accelerationBound + 1.0e-9
        || !std::isfinite(dt) || dt <= 0.0
        || lookaheadSteps <= 0) {
        return score;
    }

    bool hasOwnerRow = false;
    for (const auto &descriptor : rows) {
        hasOwnerRow = hasOwnerRow || descriptor.ownerRobotId == robotId;
    }
    if (!hasOwnerRow) {
        return score;
    }

    const auto rollout = rolloutBridgePredictionStates(
            robotId, candidateAcceleration, mobileStates, dt, lookaheadSteps);
    if (rollout.size() != static_cast<size_t>(lookaheadSteps)) {
        return score;
    }

    score.minimumBudget = std::numeric_limits<double>::infinity();
    score.stepBudgets.reserve(rollout.size());
    score.stepDominantRows.reserve(rollout.size());
    try {
        for (size_t stepIndex = 0; stepIndex < rollout.size(); ++stepIndex) {
            const auto &states = rollout[stepIndex];
            const auto evaluatedRows = evaluateBridgeFullRows(
                    robotId, states, fixedBases, rows);
            std::vector<BridgeGammaStarResidual2D> residuals;
            std::vector<std::string> identities;
            residuals.reserve(evaluatedRows.size());
            identities.reserve(evaluatedRows.size());
            for (const auto &evaluated : evaluatedRows) {
                residuals.push_back(bridgeGammaStarResidualFromAffineMargin(
                        evaluated.row.uCoe(0),
                        evaluated.row.uCoe(1),
                        evaluated.row.constTerm));
                identities.push_back(evaluated.identity);
            }

            const auto solution = solveExactBridgeGammaStar2D(
                    residuals, accelerationBound);
            if (!solution.valid || !std::isfinite(solution.gamma)) {
                return {};
            }

            std::string stepDominant;
            double dominantMargin = std::numeric_limits<double>::infinity();
            for (size_t rowIndex = 0; rowIndex < residuals.size(); ++rowIndex) {
                const auto &residual = residuals[rowIndex];
                const double margin = residual.constant
                                      - residual.ax * solution.accelX
                                      - residual.ay * solution.accelY;
                if (margin < dominantMargin
                    || (margin == dominantMargin
                        && (stepDominant.empty()
                            || identities[rowIndex] < stepDominant))) {
                    dominantMargin = margin;
                    stepDominant = identities[rowIndex];
                }
            }

            score.stepBudgets.push_back(solution.gamma);
            score.stepDominantRows.push_back(stepDominant);
            if (solution.gamma < score.minimumBudget) {
                score.minimumBudget = solution.gamma;
                score.worstStep = static_cast<int>(stepIndex) + 1;
                score.dominantRow = stepDominant;
            }
        }
    } catch (const std::exception &) {
        return {};
    }

    score.valid = score.stepBudgets.size() == rollout.size()
                  && score.stepDominantRows.size() == rollout.size()
                  && std::isfinite(score.minimumBudget)
                  && score.worstStep > 0
                  && !score.dominantRow.empty();
    return score;
}

#endif  // CBF_BRIDGE_FULL_ROW_PREDICTIVE_FEEDBACK_HPP
