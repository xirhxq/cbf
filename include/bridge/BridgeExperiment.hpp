#ifndef CBF_BRIDGE_EXPERIMENT_HPP
#define CBF_BRIDGE_EXPERIMENT_HPP

#include "utils.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

struct BridgeTargetConfig {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
};

struct BridgeExperimentConfig {
    bool enabled = false;
    std::string row = "debug";
    std::string searchPolicy = "coverage";
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
    bridge.searchPolicy = source.value("search-policy", bridge.searchPolicy);
    bridge.topologyPolicy = source.value("topology-policy", bridge.topologyPolicy);
    bridge.safetyFilter = source.value("safety-filter", bridge.safetyFilter);
    bridge.reportCadence = source.value("report-cadence", bridge.reportCadence);

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
    }

    if (source.contains("target")) {
        const json &target = source.at("target");
        bridge.target.x = target.value("x", 0.0);
        bridge.target.y = target.value("y", 0.0);
        bridge.target.radius = target.value("radius", 0.0);
    }
    return bridge;
}

inline json makeBridgeMetadata(const json &config, const BridgeExperimentConfig &bridge) {
    json metadata = {
        {"enabled", bridge.enabled},
        {"row", bridge.row},
        {"search_policy", bridge.searchPolicy},
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
