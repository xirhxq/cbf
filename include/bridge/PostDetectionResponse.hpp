#ifndef CBF_POST_DETECTION_RESPONSE_HPP
#define CBF_POST_DETECTION_RESPONSE_HPP

#include "utils.h"
#include "bridge/SingleLadderGoalSelector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

inline constexpr double BRIDGE_R13_BASE_X_M = 250.0;
inline constexpr double BRIDGE_R13_BASE0_Y_M = 1000.0;
inline constexpr double BRIDGE_R13_BASE1_Y_M = 1510.0;
inline constexpr double BRIDGE_R13_COMMUNICATION_RANGE_M = 850.0;
inline constexpr double BRIDGE_R13_TERMINAL_COMMUNICATION_MARGIN_M = 10.0;
inline constexpr double BRIDGE_R13_TARGET_RADIUS_M = 50.0;
inline constexpr double BRIDGE_R13_DWELL_TIME_S = 30.0;

inline double bridgeR13DistanceForAssignedEdgeLimit(double edgeLimitM) {
    constexpr double baseSeparationM =
            BRIDGE_R13_BASE1_Y_M - BRIDGE_R13_BASE0_Y_M;
    return (-std::sqrt(3.0) * baseSeparationM
            + std::sqrt(16.0 * edgeLimitM * edgeLimitM
                        - baseSeparationM * baseSeparationM)) / 2.0;
}

inline const double BRIDGE_R13_DISTANCE_CRITICAL_M =
        bridgeR13DistanceForAssignedEdgeLimit(
                BRIDGE_R13_COMMUNICATION_RANGE_M
                - BRIDGE_R13_TERMINAL_COMMUNICATION_MARGIN_M);
inline const double BRIDGE_R13_DISTANCE_LOW_M =
        0.9 * bridgeR13DistanceForAssignedEdgeLimit(
                BRIDGE_R13_COMMUNICATION_RANGE_M
                - BRIDGE_R13_TERMINAL_COMMUNICATION_MARGIN_M);
inline const double BRIDGE_R13_DISTANCE_NEGATIVE_M =
        bridgeR13DistanceForAssignedEdgeLimit(
                BRIDGE_R13_COMMUNICATION_RANGE_M) + 25.0;

struct BridgeR13StaticAudit {
    bool valid = false;
    bool worldValid = false;
    bool separationValid = false;
    bool physicalCommunicationValid = false;
    double maxAssignedEdgeM = 0.0;
    int maxAssignedEdgeOwnerId = 0;
    std::string maxAssignedEdgeReferenceKind;
    int maxAssignedEdgeReferenceId = -1;
    std::vector<BridgeSingleLadderGoalEdge> overPhysicalRangeEdges;
    std::vector<std::string> rejectionReasons;
};

struct BridgeR13StaticGeometry {
    double offshoreDistanceM = 0.0;
    Point target;
    BridgeSingleLadderGoalTuple tuple;
    BridgeSingleLadderGoalCertificate exactEightEdgeCertificate;
    BridgeR13StaticAudit audit;
};

inline BridgeR13StaticGeometry bridgeR13StaticGeometry(double distanceM) {
    const Point base0(BRIDGE_R13_BASE_X_M, BRIDGE_R13_BASE0_Y_M);
    const Point base1(BRIDGE_R13_BASE_X_M, BRIDGE_R13_BASE1_Y_M);
    const Point target(
            BRIDGE_R13_BASE_X_M + distanceM,
            0.5 * (BRIDGE_R13_BASE0_Y_M + BRIDGE_R13_BASE1_Y_M));
    const auto tuple = bridgeBuildSingleLadderGoalTuple(base0, base1, target);
    const auto certificate = bridgeCertifySingleLadderGoalTuple(
            tuple, base0, base1, bridgeSingleTriangularLadderReferences(),
            Point(0.0, 0.0), Point(1800.0, 2000.0),
            BRIDGE_R13_COMMUNICATION_RANGE_M, 10.0);

    BridgeR13StaticAudit audit;
    audit.worldValid = true;
    for (const Point &goal : tuple.goals) {
        audit.worldValid = audit.worldValid
                && goal.x >= 0.0 && goal.x <= 1800.0
                && goal.y >= 0.0 && goal.y <= 2000.0;
    }
    audit.separationValid = std::all_of(
            certificate.mobileSeparations.begin(),
            certificate.mobileSeparations.end(),
            [](double distance) { return distance >= 10.0; });
    audit.physicalCommunicationValid = true;
    for (const auto &edge : certificate.edges) {
        if (edge.length > audit.maxAssignedEdgeM) {
            audit.maxAssignedEdgeM = edge.length;
            audit.maxAssignedEdgeOwnerId = edge.ownerId;
            audit.maxAssignedEdgeReferenceKind = edge.referenceKind;
            audit.maxAssignedEdgeReferenceId = edge.referenceId;
        }
        if (edge.length > BRIDGE_R13_COMMUNICATION_RANGE_M) {
            audit.physicalCommunicationValid = false;
            audit.overPhysicalRangeEdges.push_back(edge);
        }
    }
    if (!audit.worldValid) {
        audit.rejectionReasons.push_back("goal-outside-world");
    }
    if (!audit.separationValid) {
        audit.rejectionReasons.push_back("mobile-separation-below-minimum");
    }
    for (const auto &edge : audit.overPhysicalRangeEdges) {
        if (edge.ownerId == 1 && edge.referenceKind == "base"
            && edge.referenceId == 0) {
            audit.rejectionReasons.push_back("u1-base0-over-physical-range");
        } else {
            audit.rejectionReasons.push_back("other-edge-over-physical-range");
        }
    }
    audit.valid = audit.rejectionReasons.empty();
    return {distanceM, target, tuple, certificate, audit};
}

struct BridgeR13DwellSnapshot {
    bool intervalValid = false;
    bool complete = false;
    double continuousDwellS = 0.0;
    double requiredDwellS = BRIDGE_R13_DWELL_TIME_S;
    double targetRadiusM = BRIDGE_R13_TARGET_RADIUS_M;
    double maxHorizontalDistanceM = std::numeric_limits<double>::infinity();
    bool hardGatesValid = false;
};

class BridgeR13DwellTracker {
public:
    BridgeR13DwellTracker(
            double targetRadiusM = BRIDGE_R13_TARGET_RADIUS_M,
            double requiredDwellS = BRIDGE_R13_DWELL_TIME_S)
        : targetRadiusM_(targetRadiusM), requiredDwellS_(requiredDwellS) {}

    void reset() {
        continuousDwellS_ = 0.0;
        snapshot_ = {};
        snapshot_.targetRadiusM = targetRadiusM_;
        snapshot_.requiredDwellS = requiredDwellS_;
    }

    BridgeR13DwellSnapshot observeZohInterval(
            const Point &position,
            const Point &velocity,
            const Point &acceleration,
            const Point &target,
            double durationS,
            bool hardGatesValid) {
        const double maxDistance = maximumDistanceZoh(
                position - target, velocity, acceleration, durationS);
        const bool finite = std::isfinite(durationS) && durationS > 0.0
                && std::isfinite(maxDistance);
        const bool intervalValid = finite && hardGatesValid
                && maxDistance <= targetRadiusM_;
        continuousDwellS_ = intervalValid
                ? continuousDwellS_ + durationS : 0.0;
        snapshot_ = {
                intervalValid,
                intervalValid && continuousDwellS_ >= requiredDwellS_,
                continuousDwellS_, requiredDwellS_, targetRadiusM_,
                maxDistance, hardGatesValid};
        return snapshot_;
    }

    const BridgeR13DwellSnapshot &snapshot() const { return snapshot_; }

private:
    static double maximumDistanceZoh(
            const Point &relativePosition,
            const Point &velocity,
            const Point &acceleration,
            double durationS) {
        if (!(durationS >= 0.0) || !std::isfinite(durationS)
            || !std::isfinite(relativePosition.x)
            || !std::isfinite(relativePosition.y)
            || !std::isfinite(velocity.x) || !std::isfinite(velocity.y)
            || !std::isfinite(acceleration.x)
            || !std::isfinite(acceleration.y)) {
            return std::numeric_limits<double>::infinity();
        }
        auto positionAt = [&](double t) {
            return relativePosition + velocity * t
                    + acceleration * (0.5 * t * t);
        };
        auto derivativeHalf = [&](double t) {
            return positionAt(t) * (velocity + acceleration * t);
        };
        std::vector<double> partitions = {0.0, durationS};
        const double qa = 1.5 * (acceleration * acceleration);
        const double qb = 3.0 * (velocity * acceleration);
        const double qc = (velocity * velocity)
                + (relativePosition * acceleration);
        if (std::abs(qa) > 1.0e-15) {
            const double discriminant = qb * qb - 4.0 * qa * qc;
            if (discriminant >= 0.0) {
                const double root = std::sqrt(discriminant);
                for (double value : {(-qb - root) / (2.0 * qa),
                                     (-qb + root) / (2.0 * qa)}) {
                    if (value > 0.0 && value < durationS) partitions.push_back(value);
                }
            }
        } else if (std::abs(qb) > 1.0e-15) {
            const double value = -qc / qb;
            if (value > 0.0 && value < durationS) partitions.push_back(value);
        }
        std::sort(partitions.begin(), partitions.end());
        std::vector<double> candidates = partitions;
        for (std::size_t index = 1; index < partitions.size(); ++index) {
            double left = partitions.at(index - 1);
            double right = partitions.at(index);
            double leftValue = derivativeHalf(left);
            double rightValue = derivativeHalf(right);
            if (leftValue == 0.0) candidates.push_back(left);
            if (rightValue == 0.0) candidates.push_back(right);
            if ((leftValue < 0.0) == (rightValue < 0.0)) continue;
            for (int iteration = 0; iteration < 80; ++iteration) {
                const double midpoint = 0.5 * (left + right);
                const double midpointValue = derivativeHalf(midpoint);
                if ((leftValue < 0.0) == (midpointValue < 0.0)) {
                    left = midpoint;
                    leftValue = midpointValue;
                } else {
                    right = midpoint;
                }
            }
            candidates.push_back(0.5 * (left + right));
        }
        double maximumSquared = 0.0;
        for (double t : candidates) {
            maximumSquared = std::max(maximumSquared, positionAt(t).len2());
        }
        return std::sqrt(maximumSquared);
    }

    double targetRadiusM_;
    double requiredDwellS_;
    double continuousDwellS_ = 0.0;
    BridgeR13DwellSnapshot snapshot_;
};

inline std::string bridgeR13CandidateMechanism(
        double candidateMaximum,
        double regulationThreshold,
        bool feedbackEnabled,
        const std::string &selectionRule = "least-intervention") {
    if (!feedbackEnabled) return "feedback-disabled";
    if (selectionRule == "maximum-reserve") {
        return "maximum-reserve-comparator";
    }
    return candidateMaximum >= regulationThreshold
            ? "reserve-admissible-least-intervention"
            : "maximum-reserve-fallback";
}

#endif
