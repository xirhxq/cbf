#ifndef CBF_BRIDGE_FULL_ROW_PREDICTION_AUDIT_HPP
#define CBF_BRIDGE_FULL_ROW_PREDICTION_AUDIT_HPP

#include "bridge/FullRowPredictiveFeedback.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

inline constexpr double BRIDGE_PREDICTION_AUDIT_TIME_TOLERANCE = 1.0e-12;

struct BridgePredictionAuditEntry {
    std::uint64_t originStep = 0;
    std::uint64_t dueStep = 0;
    double originTime = 0.0;
    double predictedTime = 0.0;
    int robotId = 0;
    int horizonStep = 0;
    double predictedBudget = -std::numeric_limits<double>::infinity();
    std::map<int, BridgePredictionState2D> predictedStates;
};

struct BridgePredictionStateError {
    int robotId = 0;
    BridgePredictionState2D predicted;
    BridgePredictionState2D observed;
    double positionError = std::numeric_limits<double>::infinity();
    double velocityError = std::numeric_limits<double>::infinity();
    double accelerationError = std::numeric_limits<double>::infinity();
};

struct BridgePredictionAuditResult {
    bool valid = false;
    std::string error;
    std::uint64_t originStep = 0;
    std::uint64_t dueStep = 0;
    std::uint64_t observedStep = 0;
    double originTime = 0.0;
    double predictedTime = 0.0;
    double observedTime = 0.0;
    int robotId = 0;
    int horizonStep = 0;
    double predictedBudget = -std::numeric_limits<double>::infinity();
    double actualBudget = -std::numeric_limits<double>::infinity();
    double budgetError = std::numeric_limits<double>::infinity();
    double maxPositionError = std::numeric_limits<double>::infinity();
    double maxVelocityError = std::numeric_limits<double>::infinity();
    double maxAccelerationError = std::numeric_limits<double>::infinity();
    std::vector<BridgePredictionStateError> stateErrors;
};

inline std::vector<BridgePredictionAuditEntry> buildBridgePredictionAuditEntries(
        std::uint64_t originStep,
        double originTime,
        int robotId,
        double dt,
        const std::vector<std::map<int, BridgePredictionState2D>> &rollout,
        const std::vector<double> &stepBudgets) {
    if (robotId <= 0 || !std::isfinite(originTime)
        || !std::isfinite(dt) || dt <= 0.0
        || rollout.empty() || rollout.size() != stepBudgets.size()) {
        return {};
    }

    std::vector<BridgePredictionAuditEntry> entries;
    entries.reserve(rollout.size());
    for (size_t index = 0; index < rollout.size(); ++index) {
        if (!std::isfinite(stepBudgets[index])
            || rollout[index].find(robotId) == rollout[index].end()) {
            return {};
        }
        for (const auto &[id, state] : rollout[index]) {
            (void) id;
            if (!std::isfinite(state.position.x)
                || !std::isfinite(state.position.y)
                || !state.velocity.allFinite()
                || !state.heldAcceleration.allFinite()) {
                return {};
            }
        }
        const auto horizonStep = static_cast<std::uint64_t>(index + 1);
        entries.push_back({
                originStep,
                originStep + horizonStep,
                originTime,
                originTime + static_cast<double>(horizonStep) * dt,
                robotId,
                static_cast<int>(horizonStep),
                stepBudgets[index],
                rollout[index],
        });
    }
    return entries;
}

inline BridgePredictionAuditResult resolveBridgePredictionAudit(
        const BridgePredictionAuditEntry &entry,
        std::uint64_t observedStep,
        double observedTime,
        const std::map<int, BridgePredictionState2D> &observedStates,
        const std::map<int, Point> &fixedBases,
        const std::vector<BridgeFullRowDescriptor> &rows,
        double accelerationBound) {
    BridgePredictionAuditResult result;
    result.originStep = entry.originStep;
    result.dueStep = entry.dueStep;
    result.observedStep = observedStep;
    result.originTime = entry.originTime;
    result.predictedTime = entry.predictedTime;
    result.observedTime = observedTime;
    result.robotId = entry.robotId;
    result.horizonStep = entry.horizonStep;
    result.predictedBudget = entry.predictedBudget;

    if (!std::isfinite(observedTime)
        || !std::isfinite(accelerationBound) || accelerationBound < 0.0) {
        result.error = "invalid audit inputs";
        return result;
    }
    if (observedStep != entry.dueStep) {
        result.error = observedStep > entry.dueStep
                ? "prediction audit entry is overdue"
                : "prediction audit entry was resolved early";
        return result;
    }
    if (std::abs(observedTime - entry.predictedTime)
            > BRIDGE_PREDICTION_AUDIT_TIME_TOLERANCE) {
        result.error = "prediction audit time does not match its forecast horizon";
        return result;
    }

    result.maxPositionError = 0.0;
    result.maxVelocityError = 0.0;
    result.maxAccelerationError = 0.0;
    for (const auto &[id, predicted] : entry.predictedStates) {
        const auto observedIt = observedStates.find(id);
        if (observedIt == observedStates.end()) {
            result.error = "observed mobile state is missing";
            return result;
        }
        const auto &observed = observedIt->second;
        const Eigen::Vector2d predictedPosition(
                predicted.position.x, predicted.position.y);
        const Eigen::Vector2d observedPosition(
                observed.position.x, observed.position.y);
        BridgePredictionStateError stateError;
        stateError.robotId = id;
        stateError.predicted = predicted;
        stateError.observed = observed;
        stateError.positionError =
                (observedPosition - predictedPosition).norm();
        stateError.velocityError =
                (observed.velocity - predicted.velocity).norm();
        stateError.accelerationError =
                (observed.heldAcceleration - predicted.heldAcceleration).norm();
        if (!std::isfinite(stateError.positionError)
            || !std::isfinite(stateError.velocityError)
            || !std::isfinite(stateError.accelerationError)) {
            result.error = "observed mobile state is nonfinite";
            return result;
        }
        result.maxPositionError = std::max(
                result.maxPositionError, stateError.positionError);
        result.maxVelocityError = std::max(
                result.maxVelocityError, stateError.velocityError);
        result.maxAccelerationError = std::max(
                result.maxAccelerationError, stateError.accelerationError);
        result.stateErrors.push_back(stateError);
    }

    try {
        const auto evaluated = evaluateBridgeFullRows(
                entry.robotId, observedStates, fixedBases, rows);
        std::vector<BridgeGammaStarResidual2D> residuals;
        residuals.reserve(evaluated.size());
        for (const auto &row : evaluated) {
            residuals.push_back(bridgeGammaStarResidualFromAffineMargin(
                    row.row.uCoe(0), row.row.uCoe(1), row.row.constTerm));
        }
        const auto solution = solveExactBridgeGammaStar2D(
                residuals, accelerationBound);
        if (!solution.valid || !std::isfinite(solution.gamma)) {
            result.error = "actual full-row budget is invalid";
            return result;
        }
        result.actualBudget = solution.gamma;
        result.budgetError = result.actualBudget - result.predictedBudget;
    } catch (const std::exception &error) {
        result.error = error.what();
        return result;
    }

    result.valid = std::isfinite(result.budgetError)
                   && result.stateErrors.size() == entry.predictedStates.size();
    return result;
}

inline std::vector<BridgePredictionAuditResult> resolveBridgePredictionAudits(
        std::uint64_t observedStep,
        double observedTime,
        const std::map<int, BridgePredictionState2D> &observedStates,
        const std::map<int, Point> &fixedBases,
        const std::vector<BridgeFullRowDescriptor> &rows,
        double accelerationBound,
        std::vector<BridgePredictionAuditEntry> &pending) {
    std::vector<BridgePredictionAuditResult> results;
    std::vector<BridgePredictionAuditEntry> remaining;
    remaining.reserve(pending.size());
    for (const auto &entry : pending) {
        if (entry.dueStep > observedStep) {
            remaining.push_back(entry);
            continue;
        }
        results.push_back(resolveBridgePredictionAudit(
                entry,
                observedStep,
                observedTime,
                observedStates,
                fixedBases,
                rows,
                accelerationBound));
    }
    pending = std::move(remaining);
    return results;
}

#endif  // CBF_BRIDGE_FULL_ROW_PREDICTION_AUDIT_HPP
