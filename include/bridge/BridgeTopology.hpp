#ifndef CBF_BRIDGE_TOPOLOGY_HPP
#define CBF_BRIDGE_TOPOLOGY_HPP

#include "ComputingGeometry/Point.hpp"
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

struct BridgeTopologyConfig {
    double maxRange = 1200.0;
    Point denialCenter = Point(1500.0, 1500.0);
    Point denialHalfSize = Point(500.0, 500.0);
    double healthyQuality = 0.95;
    double deniedQuality = 0.15;
    double sigma0 = 20.0;
    double referenceVariance = 25.0;
    double uncertaintyMultiplier = 3.0;
    double physicalSwitchMargin = 0.0;
    double robustSwitchMargin = 0.0;
    double certifiedMargin = 0.0;
    bool certifiedOnly = false;
    bool failSafeHold = false;
};

struct BridgeTopologyDecision {
    bool accepted = false;
    bool certified = false;
    bool failSafe = false;
    bool relayActive = false;
    int acceptedSwitches = 0;
    int rejectedCandidates = 0;
    double minRobustMargin = 0.0;
    double minFimEigenvalue = 0.0;
    double relaySupportMargin = std::numeric_limits<double>::quiet_NaN();
    std::map<int, std::vector<int>> anchorIds;
    json log = json::object();
};

struct BridgeOneStepSupportGoal {
    Point goal;
    Point predicted;
    double margin = -std::numeric_limits<double>::infinity();
    bool improved = false;
};

inline bool bridgePointInsideDeniedZone(const Point &p, const BridgeTopologyConfig &config) {
    return std::abs(p.x - config.denialCenter.x) <= config.denialHalfSize.x
           && std::abs(p.y - config.denialCenter.y) <= config.denialHalfSize.y;
}

inline double bridgeLinkQuality(const Point &a, const Point &b, const BridgeTopologyConfig &config) {
    Point midpoint((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
    return bridgePointInsideDeniedZone(midpoint, config) ? config.deniedQuality : config.healthyQuality;
}

inline double bridgeEffectiveRangeVariance(double quality, const BridgeTopologyConfig &config) {
    double q = std::max(0.05, quality);
    return config.sigma0 * config.sigma0 / q + config.referenceVariance;
}

inline double bridgeRobustMargin(const Point &a, const Point &b, const BridgeTopologyConfig &config) {
    double quality = bridgeLinkQuality(a, b, config);
    double uncertainty = config.uncertaintyMultiplier * std::sqrt(bridgeEffectiveRangeVariance(quality, config));
    return config.maxRange - a.distance_to(b) - uncertainty;
}

inline double bridgeFimDiagnostic(double margin, const BridgeTopologyConfig &config) {
    double normalized = std::max(0.0, margin + config.maxRange) / std::max(config.maxRange, 1.0);
    return normalized * normalized;
}

inline double bridgeRelaySupportMargin(const std::map<int, Point> &positions, const BridgeTopologyConfig &config) {
    if (positions.size() < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    int beacon = positions.begin()->first;
    int relay = std::next(positions.begin())->first;
    return bridgeRobustMargin(positions.at(beacon), positions.at(relay), config);
}

inline Point bridgeRelaySupportGoal(
    const std::map<int, Point> &positions,
    const BridgeTopologyConfig &config,
    double requiredMargin
) {
    if (positions.size() < 2) {
        return positions.empty() ? Point(0.0, 0.0) : positions.begin()->second;
    }

    int beacon = positions.begin()->first;
    int relay = std::next(positions.begin())->first;
    Point beaconPoint = positions.at(beacon);
    Point relayPoint = positions.at(relay);

    Point downstream(0.0, 0.0);
    int downstreamCount = 0;
    for (const auto &[id, point] : positions) {
        if (id == beacon || id == relay) {
            continue;
        }
        downstream = downstream + point;
        ++downstreamCount;
    }
    if (downstreamCount > 0) {
        downstream = downstream / static_cast<double>(downstreamCount);
    } else {
        downstream = relayPoint;
    }

    Point direction = downstream - beaconPoint;
    if (direction.len() <= 1.0e-9) {
        direction = relayPoint - beaconPoint;
    }
    double directionLength = direction.len();
    if (directionLength <= 1.0e-9) {
        return relayPoint;
    }
    Point unit = direction / directionLength;
    double upperDistance = std::max(relayPoint.distance_to(beaconPoint), downstream.distance_to(beaconPoint));
    upperDistance = std::max(upperDistance, 1.0);

    Point best = beaconPoint;
    for (int i = 0; i <= 128; ++i) {
        double fraction = static_cast<double>(i) / 128.0;
        Point candidate = beaconPoint + unit * (upperDistance * fraction);
        if (bridgeRobustMargin(beaconPoint, candidate, config) >= requiredMargin) {
            best = candidate;
        }
    }
    return best;
}

inline Point bridgeSupportChainGoal(
    const Point &anchor,
    const Point &current,
    const Point &desiredGoal,
    const BridgeTopologyConfig &config,
    double requiredMargin
) {
    if (bridgeRobustMargin(anchor, desiredGoal, config) >= requiredMargin) {
        return desiredGoal;
    }

    Point direction = desiredGoal - anchor;
    if (direction.len() <= 1.0e-9) {
        direction = current - anchor;
    }
    double directionLength = direction.len();
    if (directionLength <= 1.0e-9) {
        return current;
    }

    Point unit = direction / directionLength;
    double upperDistance = std::max(anchor.distance_to(current), anchor.distance_to(desiredGoal));
    upperDistance = std::max(upperDistance, 1.0);

    Point best = anchor;
    for (int i = 0; i <= 128; ++i) {
        double fraction = static_cast<double>(i) / 128.0;
        Point candidate = anchor + unit * (upperDistance * fraction);
        if (bridgeRobustMargin(anchor, candidate, config) >= requiredMargin) {
            best = candidate;
        }
    }
    return best;
}

inline Point bridgePredictiveSupportChainGoal(
    const Point &anchor,
    const Point &current,
    const Point &predictedNext,
    const Point &desiredGoal,
    const BridgeTopologyConfig &config,
    double requiredMargin
) {
    if (bridgeRobustMargin(anchor, predictedNext, config) >= requiredMargin) {
        return desiredGoal;
    }
    return bridgeSupportChainGoal(anchor, current, desiredGoal, config, requiredMargin);
}

inline Point bridgePredictiveMovingAnchorSupportGoal(
    const Point &,
    const Point &anchorPredicted,
    const Point &childCurrent,
    const Point &childPredicted,
    const Point &desiredGoal,
    const BridgeTopologyConfig &config,
    double requiredMargin
) {
    if (bridgeRobustMargin(anchorPredicted, childPredicted, config) >= requiredMargin) {
        return desiredGoal;
    }
    return bridgeSupportChainGoal(anchorPredicted, childCurrent, desiredGoal, config, requiredMargin);
}

inline BridgeOneStepSupportGoal bridgeChooseBetterOneStepSupportGoal(
    const Point &anchorPredicted,
    const Point &primaryGoal,
    const Point &primaryPredicted,
    const Point &fallbackGoal,
    const Point &fallbackPredicted,
    const BridgeTopologyConfig &config
) {
    double primaryMargin = bridgeRobustMargin(anchorPredicted, primaryPredicted, config);
    double fallbackMargin = bridgeRobustMargin(anchorPredicted, fallbackPredicted, config);
    if (fallbackMargin > primaryMargin) {
        return {fallbackGoal, fallbackPredicted, fallbackMargin, true};
    }
    return {primaryGoal, primaryPredicted, primaryMargin, false};
}

inline BridgeTopologyDecision chooseBridgeTopology(
    const std::map<int, Point> &positions,
    const BridgeTopologyConfig &config,
    const std::string &policy
) {
    BridgeTopologyDecision decision;
    if (positions.empty()) {
        decision.log = {
            {"event", "empty"},
            {"accepted", false},
            {"certified", false},
            {"fail_safe", config.certifiedOnly}
        };
        return decision;
    }

    int beacon = positions.begin()->first;
    int relay = positions.size() >= 2 ? std::next(positions.begin())->first : beacon;
    decision.relaySupportMargin = bridgeRelaySupportMargin(positions, config);
    decision.minRobustMargin = std::numeric_limits<double>::infinity();
    decision.minFimEigenvalue = std::numeric_limits<double>::infinity();

    for (const auto &[id, point] : positions) {
        if (id == beacon) {
            decision.anchorIds[id] = {};
            continue;
        }
        double directMargin = bridgeRobustMargin(positions.at(beacon), point, config);
        double directDistance = positions.at(beacon).distance_to(point);
        double directPhysicalMargin = config.maxRange - directDistance;
        double acceptedMargin = directMargin;
        std::vector<int> anchors = {beacon};

        bool directPhysicallyTight = directPhysicalMargin < config.physicalSwitchMargin;
        bool reservePolicy = policy == "adaptive-relay-reserve";
        bool chainPolicy = policy == "adaptive-chain" || policy == "adaptive-relay-chain";
        bool directRobustlyTight = reservePolicy && directMargin < config.robustSwitchMargin;
        bool adaptiveRelayPolicy = policy == "adaptive-relay" || reservePolicy;
        if (chainPolicy) {
            int bestAnchor = beacon;
            double bestMargin = directMargin;
            for (const auto &[candidateId, candidatePoint] : positions) {
                if (candidateId == id) {
                    break;
                }
                if (candidateId == beacon) {
                    continue;
                }
                double candidatePhysicalMargin = config.maxRange - candidatePoint.distance_to(point);
                if (candidatePhysicalMargin < 0.0) {
                    continue;
                }
                double candidateMargin = bridgeRobustMargin(candidatePoint, point, config);
                if (candidateMargin > bestMargin) {
                    bestMargin = candidateMargin;
                    bestAnchor = candidateId;
                }
            }
            if (bestAnchor != beacon) {
                anchors = {bestAnchor};
                acceptedMargin = bestMargin;
                decision.relayActive = true;
                ++decision.acceptedSwitches;
                if (directMargin < config.certifiedMargin || directPhysicallyTight) {
                    ++decision.rejectedCandidates;
                }
            }
        } else if (adaptiveRelayPolicy && id != relay && (directMargin < 0.0 || directPhysicallyTight || directRobustlyTight)) {
            ++decision.rejectedCandidates;
            double relayFirst = bridgeRobustMargin(positions.at(beacon), positions.at(relay), config);
            double relaySecond = bridgeRobustMargin(positions.at(relay), point, config);
            double relayMargin = std::min(relayFirst, relaySecond);
            double relayPhysicalMargin = std::min(config.maxRange - positions.at(beacon).distance_to(positions.at(relay)),
                                                  config.maxRange - positions.at(relay).distance_to(point));
            bool relayWithinPhysicalRange = relayPhysicalMargin >= 0.0;
            bool relayHasPreferredMargin = relayPhysicalMargin >= config.physicalSwitchMargin;
            bool relayIsPhysicalImprovement = directPhysicallyTight && relayPhysicalMargin > directPhysicalMargin;
            if (relayWithinPhysicalRange
                && ((relayHasPreferredMargin && relayMargin >= directMargin) || relayIsPhysicalImprovement)) {
                anchors = {relay};
                acceptedMargin = relayMargin;
                decision.relayActive = true;
                ++decision.acceptedSwitches;
            }
        }

        decision.anchorIds[id] = anchors;
        decision.minRobustMargin = std::min(decision.minRobustMargin, acceptedMargin);
        decision.minFimEigenvalue = std::min(decision.minFimEigenvalue, bridgeFimDiagnostic(acceptedMargin, config));
    }

    decision.accepted = std::isfinite(decision.minRobustMargin);
    decision.certified = decision.accepted && decision.minRobustMargin >= config.certifiedMargin;
    decision.failSafe = config.certifiedOnly && !decision.certified;
    decision.log = {
        {"accepted", decision.accepted},
        {"certified", decision.certified},
        {"certified_only", config.certifiedOnly},
        {"fail_safe", decision.failSafe},
        {"fail_safe_hold", config.failSafeHold},
        {"certified_margin", config.certifiedMargin},
        {"relay_active", decision.relayActive},
        {"accepted_switches", decision.acceptedSwitches},
        {"rejected_candidates", decision.rejectedCandidates},
        {"relay_support_margin", decision.relaySupportMargin},
        {"min_robust_margin", decision.minRobustMargin},
        {"min_fim_eigenvalue", decision.minFimEigenvalue}
    };
    return decision;
}

inline BridgeTopologyConfig loadBridgeTopologyConfig(const json &config) {
    BridgeTopologyConfig topology;
    if (!config.contains("bridge") || !config.at("bridge").contains("topology")) {
        return topology;
    }
    const json &source = config.at("bridge").at("topology");
    topology.maxRange = source.value("max-range", topology.maxRange);
    topology.healthyQuality = source.value("healthy-quality", topology.healthyQuality);
    topology.deniedQuality = source.value("denied-quality", topology.deniedQuality);
    topology.sigma0 = source.value("sigma0", topology.sigma0);
    topology.referenceVariance = source.value("reference-variance", topology.referenceVariance);
    topology.uncertaintyMultiplier = source.value("uncertainty-multiplier", topology.uncertaintyMultiplier);
    topology.physicalSwitchMargin = source.value("physical-switch-margin", topology.physicalSwitchMargin);
    topology.robustSwitchMargin = source.value("robust-switch-margin", topology.robustSwitchMargin);
    topology.certifiedMargin = source.value("certified-margin", topology.certifiedMargin);
    topology.certifiedOnly = source.value("certified-only", topology.certifiedOnly);
    topology.failSafeHold = source.value("fail-safe-hold", topology.failSafeHold);
    if (source.contains("denial-center")) {
        topology.denialCenter = Point(source.at("denial-center").at(0).get<double>(),
                                      source.at("denial-center").at(1).get<double>());
    }
    if (source.contains("denial-half-size")) {
        topology.denialHalfSize = Point(source.at("denial-half-size").at(0).get<double>(),
                                        source.at("denial-half-size").at(1).get<double>());
    }
    return topology;
}

#endif
