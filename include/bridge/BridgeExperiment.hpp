#ifndef CBF_BRIDGE_EXPERIMENT_HPP
#define CBF_BRIDGE_EXPERIMENT_HPP

#include "utils.h"
#include "bridge/PostDetectionResponse.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

inline constexpr double BRIDGE_FULL_ROW_GUARD_REPRODUCTION_TOLERANCE = 1.0e-10;
inline constexpr double BRIDGE_FULL_ROW_QP_REPRODUCTION_TOLERANCE = 1.0e-5;
inline constexpr double BRIDGE_FULL_ROW_QP_HARD_MARGIN_TOLERANCE = 1.0e-8;
inline constexpr double BRIDGE_FULL_ROW_NUMERICAL_REPAIR_TOLERANCE = 1.0e-10;

struct BridgeTargetConfig {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
};

struct BridgeExperimentConfig {
    bool enabled = false;
    std::string row = "debug";
    std::string taskMode = "search";
    std::string searchPolicy = "coverage";
    bool jointSingleLadderGoalsEnabled = false;
    bool postDetectionResponseEnabled = false;
    double responseDistanceM = 0.0;
    double responseTargetRadiusM = BRIDGE_R13_TARGET_RADIUS_M;
    double responseDwellTimeS = BRIDGE_R13_DWELL_TIME_S;
    double responseTerminalCommunicationMarginM =
            BRIDGE_R13_TERMINAL_COMMUNICATION_MARGIN_M;
    int jointSingleLadderLeaderId = 4;
    double jointSingleLadderRotationDeg = 60.0;
    double jointSingleLadderGoalMaxRange = 849.0;
    double jointSingleLadderGoalMinSeparation = 10.0;
    int jointSingleLadderInitialRow = 4;
    int jointSingleLadderInitialColumn = 11;
    bool stopOnDetection = false;
    std::string topologyPolicy = "fixed";
    std::string safetyFilter = "first-order-cbf";
    double reportCadence = 1.0;
    bool nominalGuardEnabled = false;
    std::string nominalGuardMode = "hocbf-feasible-projection";
    double nominalGuardTolerance = 1.0e-9;
    bool relaySupportGuardEnabled = false;
    double relaySupportRobustMargin = 0.0;
    bool supportChainGuardEnabled = false;
    double supportChainRobustMargin = 0.0;
    std::string supportChainGuardScope = "first-anchor";
    bool supportChainStateReserveEnabled = false;
    double supportChainStateReserveBaseMargin = 0.0;
    double supportChainStateReserveHeadroom = 0.0;
    double supportChainStateReserveTightenMargin = 0.0;
    double supportChainStateReserveClosingRateGain = 0.0;
    double supportChainStateReserveMaxMargin = 0.0;
    bool goalDiversionEnabled = false;
    double goalDiversionDistance = 120.0;
    double goalDiversionRadial = 4.0;
    double goalDiversionSeparationScale = 1.0;
    double goalDiversionMaxOffset = 200.0;
    std::string goalDiversionPairScope = "all";
    int goalDiversionPairIdA = 3;
    int goalDiversionPairIdB = 4;
    int goalDiversionLookaheadSteps = 0;
    double goalDiversionLookaheadDistance = 0.0;
    double goalDiversionLookaheadRadial = 0.0;
    bool goalDiversionMultiPair = false;
    bool gammaStarFeedbackEnabled = false;
    std::string gammaStarFeedbackMode = "reserve-task-homotopy";
    std::string gammaStarFeedbackAnalysisRole = "main";
    std::string gammaStarFeedbackSelectionRule = "least-intervention";
    std::string gammaStarFeedbackConstraintExecution = "hard";
    int gammaStarHomotopyIntervals = 8;
    int gammaStarLookaheadSteps = 1;
    double gammaStarPredictiveGate = 0.0;
    BridgeTargetConfig target;
};

inline double bridgeWorldExtent(const json &config, int axis) {
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    for (const auto &point : config.at("world").at("boundary")) {
        double value = point.at(axis).get<double>();
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }
    return maxValue - minValue;
}

inline BridgeExperimentConfig loadBridgeExperimentConfig(const json &config) {
    BridgeExperimentConfig bridge;
    if (!config.contains("bridge")) {
        return bridge;
    }

    const json &source = config.at("bridge");
    bridge.enabled = source.value("enabled", false);
    bridge.row = source.value("row", bridge.row);
    bridge.taskMode = source.value("task-mode", bridge.taskMode);
    bridge.searchPolicy = source.value("search-policy", bridge.searchPolicy);
    bridge.topologyPolicy = source.value("topology-policy", bridge.topologyPolicy);
    bridge.safetyFilter = source.value("safety-filter", bridge.safetyFilter);
    bridge.reportCadence = source.value("report-cadence", bridge.reportCadence);
    if (source.contains("search")) {
        bridge.stopOnDetection = source.at("search").value(
                "stop-on-detection", bridge.stopOnDetection);
    }

    if (bridge.reportCadence <= 0.0 || !std::isfinite(bridge.reportCadence)) {
        throw std::invalid_argument("bridge.report-cadence must be positive and finite");
    }

    if (source.contains("nominal") && source.at("nominal").contains("guard")) {
        const json &guard = source.at("nominal").at("guard");
        bridge.nominalGuardEnabled = guard.value("enabled", false);
        bridge.nominalGuardMode = guard.value("mode", bridge.nominalGuardMode);
        bridge.nominalGuardTolerance = guard.value("tolerance", bridge.nominalGuardTolerance);
        if (bridge.nominalGuardTolerance <= 0.0 || !std::isfinite(bridge.nominalGuardTolerance)) {
            throw std::invalid_argument("bridge.nominal.guard.tolerance must be positive and finite");
        }
        if (bridge.nominalGuardMode != "hocbf-feasible-projection") {
            throw std::invalid_argument("unsupported bridge.nominal.guard.mode");
        }
    }
    if (source.contains("nominal") && source.at("nominal").contains("relay-support-guard")) {
        const json &guard = source.at("nominal").at("relay-support-guard");
        bridge.relaySupportGuardEnabled = guard.value("enabled", false);
        bridge.relaySupportRobustMargin = guard.value("robust-margin", bridge.relaySupportRobustMargin);
        if (bridge.relaySupportRobustMargin < 0.0 || !std::isfinite(bridge.relaySupportRobustMargin)) {
            throw std::invalid_argument("bridge.nominal.relay-support-guard.robust-margin must be non-negative and finite");
        }
    }
    if (source.contains("nominal") && source.at("nominal").contains("support-chain-guard")) {
        const json &guard = source.at("nominal").at("support-chain-guard");
        bridge.supportChainGuardEnabled = guard.value("enabled", false);
        bridge.supportChainRobustMargin = guard.value("robust-margin", bridge.supportChainRobustMargin);
        if (bridge.supportChainRobustMargin < 0.0 || !std::isfinite(bridge.supportChainRobustMargin)) {
            throw std::invalid_argument("bridge.nominal.support-chain-guard.robust-margin must be non-negative and finite");
        }
        bridge.supportChainGuardScope = guard.value("scope", bridge.supportChainGuardScope);
        if (bridge.supportChainGuardScope != "first-anchor"
            && bridge.supportChainGuardScope != "all-active-edges") {
            throw std::invalid_argument("unsupported bridge.nominal.support-chain-guard.scope");
        }
        if (guard.contains("state-dependent-reserve")) {
            const json &stateReserve = guard.at("state-dependent-reserve");
            bridge.supportChainStateReserveEnabled = stateReserve.value("enabled", false);
            bridge.supportChainStateReserveBaseMargin =
                stateReserve.value("base-margin", bridge.supportChainRobustMargin);
            bridge.supportChainStateReserveHeadroom = stateReserve.value("headroom-margin", 0.0);
            bridge.supportChainStateReserveTightenMargin =
                stateReserve.value("tighten-margin", bridge.supportChainRobustMargin);
            bridge.supportChainStateReserveClosingRateGain =
                stateReserve.value("closing-rate-gain", 0.0);
            bridge.supportChainStateReserveMaxMargin =
                stateReserve.value("max-margin", bridge.supportChainRobustMargin);
            auto assertNonnegFinite = [](double value, const char *field) {
                if (value < 0.0 || !std::isfinite(value)) {
                    throw std::invalid_argument(
                        std::string("bridge.nominal.support-chain-guard.state-dependent-reserve.")
                        + field + " must be non-negative and finite");
                }
            };
            assertNonnegFinite(bridge.supportChainStateReserveBaseMargin, "base-margin");
            assertNonnegFinite(bridge.supportChainStateReserveHeadroom, "headroom-margin");
            assertNonnegFinite(bridge.supportChainStateReserveTightenMargin, "tighten-margin");
            assertNonnegFinite(bridge.supportChainStateReserveClosingRateGain, "closing-rate-gain");
            assertNonnegFinite(bridge.supportChainStateReserveMaxMargin, "max-margin");
            if (bridge.supportChainStateReserveMaxMargin < bridge.supportChainStateReserveBaseMargin) {
                throw std::invalid_argument(
                    "bridge.nominal.support-chain-guard.state-dependent-reserve.max-margin must be >= base-margin");
            }
        }
    }

    if (source.contains("nominal") && source.at("nominal").contains("goal-diversion")) {
        const json &diversion = source.at("nominal").at("goal-diversion");
        bridge.goalDiversionEnabled = diversion.value("enabled", false);
        bridge.goalDiversionDistance = diversion.value("distance-threshold", bridge.goalDiversionDistance);
        bridge.goalDiversionRadial = diversion.value("radial-threshold", bridge.goalDiversionRadial);
        bridge.goalDiversionSeparationScale = diversion.value("separation-scale", bridge.goalDiversionSeparationScale);
        bridge.goalDiversionMaxOffset = diversion.value("max-offset", bridge.goalDiversionMaxOffset);
        bridge.goalDiversionPairScope = diversion.value("pair-scope", bridge.goalDiversionPairScope);
        bridge.goalDiversionPairIdA = diversion.value("pair-id-a", bridge.goalDiversionPairIdA);
        bridge.goalDiversionPairIdB = diversion.value("pair-id-b", bridge.goalDiversionPairIdB);
        bridge.goalDiversionLookaheadSteps = diversion.value("lookahead-steps", bridge.goalDiversionLookaheadSteps);
        bridge.goalDiversionLookaheadDistance = diversion.value("lookahead-distance-threshold", bridge.goalDiversionDistance);
        bridge.goalDiversionLookaheadRadial = diversion.value("lookahead-radial-threshold", bridge.goalDiversionRadial);
        bridge.goalDiversionMultiPair = diversion.value("multi-pair", bridge.goalDiversionMultiPair);
        auto assertPosFinite = [](double value, const char *field) {
            if (value <= 0.0 || !std::isfinite(value)) {
                throw std::invalid_argument(
                    std::string("bridge.nominal.goal-diversion.") + field
                    + " must be positive and finite");
            }
        };
        assertPosFinite(bridge.goalDiversionDistance, "distance-threshold");
        assertPosFinite(bridge.goalDiversionRadial, "radial-threshold");
        if (bridge.goalDiversionSeparationScale < 0.0 || !std::isfinite(bridge.goalDiversionSeparationScale)) {
            throw std::invalid_argument(
                "bridge.nominal.goal-diversion.separation-scale must be non-negative and finite");
        }
        if (bridge.goalDiversionMaxOffset <= 0.0 || !std::isfinite(bridge.goalDiversionMaxOffset)) {
            throw std::invalid_argument(
                "bridge.nominal.goal-diversion.max-offset must be positive and finite");
        }
        if (bridge.goalDiversionPairScope != "all"
            && bridge.goalDiversionPairScope != "named-pair") {
            throw std::invalid_argument(
                "unsupported bridge.nominal.goal-diversion.pair-scope (expected 'all' or 'named-pair')");
        }
        if (bridge.goalDiversionLookaheadSteps < 0 || !std::isfinite(static_cast<double>(bridge.goalDiversionLookaheadSteps))) {
            throw std::invalid_argument(
                "bridge.nominal.goal-diversion.lookahead-steps must be non-negative and finite");
        }
        if (bridge.goalDiversionLookaheadSteps > 0) {
            assertPosFinite(bridge.goalDiversionLookaheadDistance, "lookahead-distance-threshold");
            assertPosFinite(bridge.goalDiversionLookaheadRadial, "lookahead-radial-threshold");
        }
        if (bridge.goalDiversionMultiPair && bridge.goalDiversionPairScope == "named-pair") {
            throw std::invalid_argument(
                "bridge.nominal.goal-diversion.multi-pair requires pair-scope 'all'");
        }
    }

    if (source.contains("nominal") && source.at("nominal").contains("gamma-star-feedback")) {
        const json &feedback = source.at("nominal").at("gamma-star-feedback");
        bridge.gammaStarFeedbackEnabled = feedback.value("enabled", false);
        bridge.gammaStarFeedbackMode = feedback.value("mode", bridge.gammaStarFeedbackMode);
        bridge.gammaStarFeedbackAnalysisRole = feedback.value(
                "analysis-role", bridge.gammaStarFeedbackAnalysisRole);
        bridge.gammaStarFeedbackSelectionRule = feedback.value(
                "selection-rule", bridge.gammaStarFeedbackSelectionRule);
        bridge.gammaStarFeedbackConstraintExecution = feedback.value(
                "constraint-execution",
                bridge.gammaStarFeedbackConstraintExecution);
        bridge.gammaStarHomotopyIntervals = feedback.value(
                "homotopy-intervals", bridge.gammaStarHomotopyIntervals);
        bridge.gammaStarLookaheadSteps = feedback.value("lookahead-steps", bridge.gammaStarLookaheadSteps);
        bridge.gammaStarPredictiveGate = feedback.value(
                "predictive-gate", bridge.gammaStarPredictiveGate);

        if (bridge.gammaStarFeedbackEnabled) {
            const std::array<const char *, 7> obsoleteFields = {
                    "safe-threshold",
                    "accel-half-box",
                    "direction-count",
                    "magnitude-count",
                    "candidate-scale",
                    "predictive-threshold",
                    "lookahead-distance",
            };
            for (const char *field : obsoleteFields) {
                if (feedback.contains(field)) {
                    throw std::invalid_argument(
                            std::string("bridge.nominal.gamma-star-feedback.")
                            + field + " is incompatible with reserve-task-homotopy");
                }
            }
            if (bridge.gammaStarFeedbackMode != "reserve-task-homotopy") {
                throw std::invalid_argument(
                        "bridge.nominal.gamma-star-feedback.mode must be reserve-task-homotopy");
            }
            if (bridge.gammaStarFeedbackAnalysisRole == "main") {
                if (bridge.gammaStarHomotopyIntervals != 8
                    || bridge.gammaStarFeedbackSelectionRule
                            != "least-intervention"
                    || bridge.gammaStarFeedbackConstraintExecution != "hard") {
                    throw std::invalid_argument(
                            "main reserve-task-homotopy requires M=8, least-intervention selection, and hard rows");
                }
            } else if (bridge.gammaStarFeedbackAnalysisRole == "sensitivity") {
                if ((bridge.gammaStarHomotopyIntervals != 4
                     && bridge.gammaStarHomotopyIntervals != 16)
                    || bridge.gammaStarFeedbackSelectionRule
                            != "least-intervention"
                    || bridge.gammaStarFeedbackConstraintExecution != "hard") {
                    throw std::invalid_argument(
                            "sensitivity reserve-task-homotopy permits only M=4/16, least-intervention selection, and hard rows");
                }
            } else if (bridge.gammaStarFeedbackAnalysisRole == "comparator") {
                if (bridge.gammaStarHomotopyIntervals != 8
                    || bridge.gammaStarFeedbackSelectionRule
                            != "least-intervention"
                    || bridge.gammaStarFeedbackConstraintExecution != "soft") {
                    throw std::invalid_argument(
                            "the predictive-soft comparator requires M=8, least-intervention selection, and soft rows");
                }
            } else if (bridge.gammaStarFeedbackAnalysisRole == "ablation") {
                if (bridge.gammaStarHomotopyIntervals != 8
                    || bridge.gammaStarFeedbackSelectionRule
                            != "maximum-reserve"
                    || bridge.gammaStarFeedbackConstraintExecution != "hard") {
                    throw std::invalid_argument(
                            "the maximum-reserve ablation requires M=8, maximum-reserve selection, and hard rows");
                }
            } else {
                throw std::invalid_argument(
                        "gamma-star feedback analysis-role must be main, sensitivity, comparator, or ablation");
            }
            if (bridge.gammaStarLookaheadSteps != 1) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires one-step recursive feasibility (H=1)");
            }
            if (!std::isfinite(bridge.gammaStarPredictiveGate)
                || bridge.gammaStarPredictiveGate < 0.0) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires a finite nonnegative predictive gate");
            }
            if (!bridge.enabled) {
                throw std::invalid_argument("gamma-star feedback requires bridge.enabled");
            }
            if (bridge.topologyPolicy != "fixed") {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires fixed bridge topology");
            }
            if (bridge.safetyFilter != "second-order-hocbf") {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires the second-order HOCBF safety filter");
            }
            if (!bridge.nominalGuardEnabled) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires the hard nominal feasibility guard");
            }
            if (bridge.nominalGuardTolerance != 1.0e-9) {
                throw std::invalid_argument(
                        "reserve-task-homotopy freezes the numerical guard tolerance at 1e-9");
            }
            if (config.value("model", "") != "DoubleIntegrate2D") {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires DoubleIntegrate2D");
            }
            if (config.value("optimiser", "") != "Gurobi") {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires the audited high-accuracy Gurobi QP path");
            }
            if (config.value("num", 0) != 4) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires exactly four mobile robots");
            }
            const json bases = config.value("bases", json::array());
            if (!bases.is_array() || bases.size() != 2) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires exactly two fixed bases");
            }
            for (const auto &base : bases) {
                if (!base.is_array() || base.size() < 2
                    || !base.at(0).is_number() || !base.at(1).is_number()
                    || !std::isfinite(base.at(0).get<double>())
                    || !std::isfinite(base.at(1).get<double>())) {
                    throw std::invalid_argument(
                            "reserve-task-homotopy base coordinates must be finite planar points");
                }
            }
            if (config.value("execute", json::object())
                    .value("execution-mode", "distributed") != "distributed") {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires distributed local QPs");
            }
            if (!source.contains("topology")
                || !source.at("topology").contains("fixed-references")
                || source.at("topology").at("fixed-references").empty()) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires explicit fixed references");
            }
            const json &topology = source.at("topology");
            if (std::abs(topology.value("max-range", 0.0) - 850.0) > 1.0e-12
                || std::abs(topology.value("uncertainty-multiplier", 0.0)) > 1.0e-12
                || std::abs(topology.value("certified-margin", 0.0)) > 1.0e-12
                || !topology.value("certified-only", false)
                || !topology.value("fail-safe-hold", false)) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires a raw 850 m fixed-topology certificate with fail-safe hold");
            }

            const json &cbfs = config.value("cbfs", json::object());
            const json &highOrder = cbfs.value("high-order", json::object());
            if (!highOrder.value("enabled", false)) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires high-order CBFs");
            }
            const double accelerationBound = highOrder.value("acceleration-bound", 0.0);
            if (!std::isfinite(accelerationBound) || accelerationBound <= 0.0) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires a positive physical acceleration bound");
            }
            const double lambda1 = highOrder.value("lambda1", 1.0);
            const double lambda2 = highOrder.value("lambda2", 1.0);
            const double sampledReserve =
                    highOrder.value("sampled-data-reserve", 0.0);
            if (!std::isfinite(lambda1) || lambda1 <= 0.0
                || !std::isfinite(lambda2) || lambda2 <= 0.0
                || !std::isfinite(sampledReserve) || sampledReserve < 0.0) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires positive HOCBF gains and a nonnegative finite constant reserve");
            }
            const bool feasibilitySlackEnabled =
                    highOrder.value("feasibility-slack", json::object())
                            .value("enabled", false);
            if (bridge.gammaStarFeedbackConstraintExecution == "soft") {
                const double slackPenalty = cbfs.value(
                        "objective-function", json::object())
                        .value("k_delta", 0.0);
                if (!feasibilitySlackEnabled
                    || !std::isfinite(slackPenalty)
                    || slackPenalty != 1000.0) {
                    throw std::invalid_argument(
                            "the predictive-soft comparator requires feasibility slack with penalty 1000");
                }
            } else if (feasibilitySlackEnabled) {
                throw std::invalid_argument(
                        "hard reserve-task-homotopy forbids HOCBF feasibility slack");
            }

            const json &withoutSlack = cbfs.value("without-slack", json::object());
            if (withoutSlack.value("method", "all") != "all") {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires every hard row, not a minimum-row surrogate");
            }
            const json &safety = withoutSlack.value("safety", json::object());
            const json &communication = withoutSlack.value("comm-fixed", json::object());
            const double safetyDistance = safety.value("safe-distance", 0.0);
            const double safetyTightening = safety.value(
                    "safe-distance-tightening-margin", 0.0);
            const double communicationRange = communication.value(
                    "max-range", 0.0);
            const double communicationTightening = communication.value(
                    "range-tightening-margin", 0.0);
            if (!safety.value("on", false)
                || std::abs(safetyDistance - 10.0) > 1.0e-12
                || !std::isfinite(safetyTightening)
                || safetyTightening < 0.0
                || safety.value("consider-uncertainty", true)
                || safety.value("pair-state-reserve", json::object())
                        .value("enabled", false)) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires 10 m safety rows with a finite non-negative fixed tightening and only the shared constant reserve");
            }
            if (!communication.value("on", false)
                || std::abs(communicationRange - 850.0) > 1.0e-12
                || !std::isfinite(communicationTightening)
                || communicationTightening < 0.0
                || communicationTightening >= communicationRange
                || communication.value("consider-uncertainty", true)
                || !communication.value("compensate-velocity", true)
                || communication.value("state-dependent-reserve", json::object())
                        .value("enabled", false)) {
                throw std::invalid_argument(
                        "reserve-task-homotopy requires 850 m communication rows with a finite fixed tightening, held-reference dynamics, and only the shared constant reserve");
            }
            if (withoutSlack.value("comm-auto", json::object())
                    .value("on", false)) {
                throw std::invalid_argument(
                        "reserve-task-homotopy forbids automatic communication rewiring");
            }
            for (const auto &[name, entry] : withoutSlack.items()) {
                if (name == "safety" || name == "comm-fixed"
                    || name == "comm-auto" || name == "method") {
                    continue;
                }
                if (entry.is_object() && entry.value("on", false)) {
                    throw std::invalid_argument(
                            "reserve-task-homotopy forbids hard rows outside the declared full family");
                }
            }
            if (config.value("position_covariance", json::object())
                    .value("enable", false)) {
                throw std::invalid_argument(
                        "reserve-task-homotopy forbids position uncertainty");
            }

            const json &withSlack = cbfs.value("with-slack", json::object());
            for (const auto &entry : withSlack) {
                if (entry.is_object() && entry.value("on", false)) {
                    throw std::invalid_argument(
                            "reserve-task-homotopy forbids unrelated soft task CBFs");
                }
            }
        }
    }

    if (source.contains("target")) {
        const json &target = source.at("target");
        bridge.target.x = target.value("x", 0.0);
        bridge.target.y = target.value("y", 0.0);
        bridge.target.radius = target.value("radius", 0.0);
    }

    bridge.postDetectionResponseEnabled =
            bridge.taskMode == "post-detection-response";
    bridge.jointSingleLadderGoalsEnabled =
            bridge.searchPolicy == "nearest-feasible-single-ladder"
            || bridge.postDetectionResponseEnabled;
    if (bridge.postDetectionResponseEnabled) {
        if (source.contains("search")) {
            throw std::invalid_argument(
                    "post-detection response forbids all search state");
        }
        if (!source.contains("response") || !source.at("response").is_object()) {
            throw std::invalid_argument(
                    "post-detection response requires one static response tuple");
        }
        const json &response = source.at("response");
        const std::vector<std::string> expectedResponseKeys = {
                "distance-m", "dwell-time-s", "leader-id", "mode",
                "target-radius-m", "terminal-communication-margin-m"};
        std::vector<std::string> responseKeys;
        for (const auto &[key, value] : response.items()) {
            (void) value;
            responseKeys.push_back(key);
        }
        std::sort(responseKeys.begin(), responseKeys.end());
        if (responseKeys != expectedResponseKeys
            || response.value("mode", "")
                    != "known-static-target-reach-and-dwell"
            || response.value("leader-id", 0) != 4
            || response.value("target-radius-m", 0.0)
                    != BRIDGE_R13_TARGET_RADIUS_M
            || response.value("dwell-time-s", 0.0)
                    != BRIDGE_R13_DWELL_TIME_S
            || response.value("terminal-communication-margin-m", 0.0)
                    != BRIDGE_R13_TERMINAL_COMMUNICATION_MARGIN_M) {
            throw std::invalid_argument(
                    "post-detection response must equal the frozen R13 task contract");
        }
        bridge.responseDistanceM = response.value(
                "distance-m", std::numeric_limits<double>::quiet_NaN());
        bridge.responseTargetRadiusM = response.at("target-radius-m").get<double>();
        bridge.responseDwellTimeS = response.at("dwell-time-s").get<double>();
        bridge.responseTerminalCommunicationMarginM =
                response.at("terminal-communication-margin-m").get<double>();
        const bool frozenDistance =
                bridge.responseDistanceM == BRIDGE_R13_DISTANCE_LOW_M
                || bridge.responseDistanceM == BRIDGE_R13_DISTANCE_CRITICAL_M
                || bridge.responseDistanceM == BRIDGE_R13_DISTANCE_NEGATIVE_M;
        const auto geometry = bridgeR13StaticGeometry(bridge.responseDistanceM);
        if (!frozenDistance
            || bridge.target.x != geometry.target.x
            || bridge.target.y != geometry.target.y
            || bridge.target.radius != BRIDGE_R13_TARGET_RADIUS_M) {
            throw std::invalid_argument(
                    "post-detection response target must be the exact frozen distance geometry");
        }
        if (bridge.gammaStarHomotopyIntervals != 8
            || bridge.gammaStarLookaheadSteps != 1
            || bridge.gammaStarPredictiveGate != 14.0) {
            throw std::invalid_argument(
                    "post-detection response freezes M=8, H=1, and tau=14");
        }
    }
    if (bridge.jointSingleLadderGoalsEnabled) {
        if (!bridge.enabled || bridge.topologyPolicy != "fixed"
            || config.value("num", 0) != 4) {
            throw std::invalid_argument(
                    "nearest-feasible-single-ladder requires an enabled fixed topology with four mobiles");
        }
        if (bridge.relaySupportGuardEnabled
            || bridge.supportChainGuardEnabled
            || bridge.goalDiversionEnabled) {
            throw std::invalid_argument(
                    "nearest-feasible-single-ladder forbids post-certificate goal modification");
        }
        const json bases = config.value("bases", json::array());
        if (!bases.is_array() || bases.size() != 2) {
            throw std::invalid_argument(
                    "nearest-feasible-single-ladder requires exactly two bases");
        }
        for (const auto &base : bases) {
            if (!base.is_array() || base.size() != 2
                || !base.at(0).is_number() || !base.at(1).is_number()
                || !std::isfinite(base.at(0).get<double>())
                || !std::isfinite(base.at(1).get<double>())) {
                throw std::invalid_argument(
                        "nearest-feasible-single-ladder base coordinates must be finite planar points");
            }
        }
        if (bridge.postDetectionResponseEnabled
            && bases != json({{BRIDGE_R13_BASE_X_M, BRIDGE_R13_BASE0_Y_M},
                              {BRIDGE_R13_BASE_X_M, BRIDGE_R13_BASE1_Y_M}})) {
            throw std::invalid_argument(
                    "post-detection response requires the frozen ordered bases");
        }
        const json expectedBoundary = {
                {0.0, 0.0}, {1800.0, 0.0},
                {1800.0, 2000.0}, {0.0, 2000.0}};
        const json &world = config.at("world");
        if (world.value("boundary", json::array()) != expectedBoundary
            || world.value("spacing", 0.0) != 100.0) {
            throw std::invalid_argument(
                    "nearest-feasible-single-ladder requires the frozen world and grid");
        }
        const json &search = bridge.postDetectionResponseEnabled
                ? json::object() : source.at("search");
        if (bridge.postDetectionResponseEnabled) {
            bridge.jointSingleLadderRotationDeg = 60.0;
            bridge.jointSingleLadderLeaderId = 4;
            bridge.jointSingleLadderGoalMaxRange = 849.0;
            bridge.jointSingleLadderGoalMinSeparation = 10.0;
            bridge.jointSingleLadderInitialRow = -1;
            bridge.jointSingleLadderInitialColumn = -1;
        } else {
        bridge.jointSingleLadderRotationDeg = search.value(
                "rotation-deg", std::numeric_limits<double>::quiet_NaN());
        bridge.jointSingleLadderLeaderId = search.value("leader-id", 0);
        bridge.jointSingleLadderGoalMaxRange = search.value(
                "goal-max-range-m", std::numeric_limits<double>::quiet_NaN());
        bridge.jointSingleLadderGoalMinSeparation = search.value(
                "goal-min-separation-m", std::numeric_limits<double>::quiet_NaN());
        const json initialGrid = search.value(
                "initial-leader-grid", json::array());
        if (bridge.jointSingleLadderRotationDeg != 60.0
            || bridge.jointSingleLadderLeaderId != 4
            || bridge.jointSingleLadderGoalMaxRange != 849.0
            || bridge.jointSingleLadderGoalMinSeparation != 10.0
            || initialGrid != json({4, 11})) {
            throw std::invalid_argument(
                    "nearest-feasible-single-ladder task geometry must equal the frozen r10 contract");
        }
        bridge.jointSingleLadderInitialRow = initialGrid.at(0).get<int>();
        bridge.jointSingleLadderInitialColumn = initialGrid.at(1).get<int>();
        }

        const json &topology = source.at("topology");
        const json expectedReferences = {
                {"1", {{"anchor-ids", json::array()}, {"base-ids", {0, 1}}}},
                {"2", {{"anchor-ids", {1}}, {"base-ids", {0}}}},
                {"3", {{"anchor-ids", {2, 1}}, {"base-ids", json::array()}}},
                {"4", {{"anchor-ids", {3, 2}}, {"base-ids", json::array()}}},
        };
        if (topology.value("fixed-references", json::object())
                    != expectedReferences
            || topology.value("max-range", 0.0) != 850.0) {
            throw std::invalid_argument(
                    "nearest-feasible-single-ladder requires the canonical eight-edge 850 m topology");
        }
        const json &withoutSlack = config.at("cbfs").at("without-slack");
        const json &communication = withoutSlack.at("comm-fixed");
        const json &safety = withoutSlack.at("safety");
        if (!communication.value("on", false)
            || communication.value("max-range", 0.0) != 850.0
            || communication.value("range-tightening-margin", 0.0) != 1.0
            || !safety.value("on", false)
            || safety.value("safe-distance", 0.0) != 10.0) {
                throw std::invalid_argument(
                    "single-ladder task requires 849 m task range and 10 m safety rows");
        }
    }
    return bridge;
}

inline json makeBridgeMetadata(const json &config, const BridgeExperimentConfig &bridge) {
    json metadata = {
        {"enabled", bridge.enabled},
        {"row", bridge.row},
        {"task_mode", bridge.taskMode},
        {"search_policy", bridge.searchPolicy},
        {"joint_single_ladder_goals_enabled",
                bridge.jointSingleLadderGoalsEnabled},
        {"post_detection_response_enabled",
                bridge.postDetectionResponseEnabled},
        {"response_distance_m", bridge.responseDistanceM},
        {"response_target_radius_m", bridge.responseTargetRadiusM},
        {"response_dwell_time_s", bridge.responseDwellTimeS},
        {"response_terminal_communication_margin_m",
                bridge.responseTerminalCommunicationMarginM},
        {"stop_on_detection", bridge.stopOnDetection},
        {"topology_policy", bridge.topologyPolicy},
        {"safety_filter", bridge.safetyFilter},
        {"nominal_guard_enabled", bridge.nominalGuardEnabled},
        {"nominal_guard_mode", bridge.nominalGuardMode},
        {"nominal_guard_tolerance", bridge.nominalGuardTolerance},
        {"relay_support_guard_enabled", bridge.relaySupportGuardEnabled},
        {"relay_support_guard_margin", bridge.relaySupportRobustMargin},
        {"support_chain_guard_enabled", bridge.supportChainGuardEnabled},
        {"support_chain_guard_margin", bridge.supportChainRobustMargin},
        {"support_chain_guard_scope", bridge.supportChainGuardScope},
        {"support_chain_state_reserve_enabled", bridge.supportChainStateReserveEnabled},
        {"support_chain_state_reserve_base_margin", bridge.supportChainStateReserveBaseMargin},
        {"support_chain_state_reserve_headroom", bridge.supportChainStateReserveHeadroom},
        {"support_chain_state_reserve_tighten_margin", bridge.supportChainStateReserveTightenMargin},
        {"support_chain_state_reserve_closing_rate_gain", bridge.supportChainStateReserveClosingRateGain},
        {"support_chain_state_reserve_max_margin", bridge.supportChainStateReserveMaxMargin},
        {"goal_diversion_enabled", bridge.goalDiversionEnabled},
        {"goal_diversion_distance_threshold", bridge.goalDiversionDistance},
        {"goal_diversion_radial_threshold", bridge.goalDiversionRadial},
        {"goal_diversion_separation_scale", bridge.goalDiversionSeparationScale},
        {"goal_diversion_max_offset", bridge.goalDiversionMaxOffset},
        {"goal_diversion_pair_scope", bridge.goalDiversionPairScope},
        {"goal_diversion_pair_id_a", bridge.goalDiversionPairIdA},
        {"goal_diversion_pair_id_b", bridge.goalDiversionPairIdB},
        {"goal_diversion_lookahead_steps", bridge.goalDiversionLookaheadSteps},
        {"goal_diversion_lookahead_distance_threshold", bridge.goalDiversionLookaheadDistance},
        {"goal_diversion_lookahead_radial_threshold", bridge.goalDiversionLookaheadRadial},
        {"goal_diversion_multi_pair", bridge.goalDiversionMultiPair},
        {"gamma_star_feedback_enabled", bridge.gammaStarFeedbackEnabled},
        {"gamma_star_feedback_mode", bridge.gammaStarFeedbackMode},
        {"gamma_star_feedback_analysis_role", bridge.gammaStarFeedbackAnalysisRole},
        {"gamma_star_feedback_selection_rule", bridge.gammaStarFeedbackSelectionRule},
        {"gamma_star_feedback_constraint_execution",
                bridge.gammaStarFeedbackConstraintExecution},
        {"gamma_star_feedback_homotopy_intervals", bridge.gammaStarHomotopyIntervals},
        {"gamma_star_feedback_lookahead_steps", bridge.gammaStarLookaheadSteps},
        {"gamma_star_feedback_predictive_gate", bridge.gammaStarPredictiveGate},
        {"area_width_m", bridgeWorldExtent(config, 0)},
        {"area_height_m", bridgeWorldExtent(config, 1)},
        {"horizon_s", config.at("execute").at("time-total").get<double>()},
        {"control_sample_time_s", config.at("execute").at("time-step").get<double>()},
        {"report_cadence_s", bridge.reportCadence},
        {"target", {
            {"x", bridge.target.x},
            {"y", bridge.target.y},
            {"radius", bridge.target.radius}
        }}
    };
    return metadata;
}

inline double bridgeEntropy(const std::vector<double> &weights) {
    double total = std::accumulate(weights.begin(), weights.end(), 0.0);
    if (total <= 0.0) {
        return 0.0;
    }
    double entropy = 0.0;
    for (double weight : weights) {
        if (weight <= 0.0) {
            continue;
        }
        double p = weight / total;
        entropy -= p * std::log(p);
    }
    return entropy;
}

#endif
