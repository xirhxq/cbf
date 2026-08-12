#ifndef CBF_SINGLE_LADDER_GOAL_SELECTOR_HPP
#define CBF_SINGLE_LADDER_GOAL_SELECTOR_HPP

#include "bridge/BridgeTopology.hpp"
#include "ComputingGeometry/Point.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

inline constexpr double BRIDGE_SINGLE_LADDER_GOAL_MAX_RANGE_M = 849.0;
inline constexpr double BRIDGE_SINGLE_LADDER_GOAL_MIN_SEPARATION_M = 10.0;

struct BridgeSingleLadderGoalTuple {
    Point leaderGoal;
    std::array<Point, 4> goals;
};

struct BridgeSingleLadderGoalEdge {
    int ownerId = 0;
    std::string referenceKind;
    int referenceId = -1;
    double length = 0.0;
};

struct BridgeSingleLadderGoalCertificate {
    bool valid = false;
    std::vector<std::string> rejectionReasons;
    std::array<BridgeSingleLadderGoalEdge, 8> edges;
    std::array<double, 6> mobileSeparations{};
};

struct BridgeSingleLadderCandidateAudit {
    int row = -1;
    int column = -1;
    Point leaderGoal;
    long double squaredDistance = 0.0;
    bool accepted = false;
    std::vector<std::string> rejectionReasons;
};

struct BridgeSingleLadderGridCell {
    int row = -1;
    int column = -1;
};

struct BridgeSingleLadderGoalDecision {
    BridgeSingleLadderGoalTuple tuple;
    BridgeSingleLadderGoalCertificate certificate;
    int selectedRow = -1;
    int selectedColumn = -1;
    int evaluatedCandidates = 0;
    bool reusedPrevious = false;
    bool noFeasibleGoal = false;
    std::uint64_t selectionEpoch = 0;
    std::vector<BridgeSingleLadderGridCell> unsearchedCells;
    std::vector<BridgeSingleLadderCandidateAudit> candidates;
};

inline json bridgeSingleLadderGoalDecisionJson(
        const BridgeSingleLadderGoalDecision &decision,
        const Point &leaderPosition,
        const std::array<Point, 2> &bases,
        const BridgeFixedReferenceMap &references,
        const Point &worldMin,
        const Point &worldMax) {
    json goals = json::array();
    for (std::size_t index = 0; index < decision.tuple.goals.size(); ++index) {
        const Point &goal = decision.tuple.goals.at(index);
        goals.push_back({
                {"robot", static_cast<int>(index + 1)},
                {"x", goal.x},
                {"y", goal.y},
        });
    }
    json edges = json::array();
    for (const auto &edge : decision.certificate.edges) {
        edges.push_back({
                {"owner_robot", edge.ownerId},
                {"reference_kind", edge.referenceKind},
                {"reference_id", edge.referenceId},
                {"length_m", edge.length},
        });
    }
    json separations = json::array();
    std::size_t separationIndex = 0;
    for (int first = 1; first <= 4; ++first) {
        for (int second = first + 1; second <= 4; ++second) {
            separations.push_back({
                    {"first_robot", first},
                    {"second_robot", second},
                    {"distance_m", decision.certificate.mobileSeparations.at(
                            separationIndex++)},
            });
        }
    }
    json candidates = json::array();
    for (const auto &candidate : decision.candidates) {
        candidates.push_back({
                {"row", candidate.row},
                {"column", candidate.column},
                {"leader_goal", {
                        {"x", candidate.leaderGoal.x},
                        {"y", candidate.leaderGoal.y}}},
                {"squared_distance_m2",
                        static_cast<double>(candidate.squaredDistance)},
                {"accepted", candidate.accepted},
                {"rejection_reasons", candidate.rejectionReasons},
        });
    }
    json unsearchedCells = json::array();
    for (const auto &cell : decision.unsearchedCells) {
        unsearchedCells.push_back({
                {"row", cell.row}, {"column", cell.column}});
    }
    json referenceMap = json::object();
    for (const auto &[robotId, entry] : references) {
        referenceMap[std::to_string(robotId)] = {
                {"anchor_ids", entry.anchorIds},
                {"base_ids", entry.baseIds},
        };
    }
    return {
            {"schema", "single-ladder-goal-ledger-v1"},
            {"selection_epoch", decision.selectionEpoch},
            {"leader_id", 4},
            {"leader_position", {
                    {"x", leaderPosition.x}, {"y", leaderPosition.y}}},
            {"selected_row", decision.selectedRow},
            {"selected_column", decision.selectedColumn},
            {"evaluated_candidates", decision.evaluatedCandidates},
            {"reused_previous", decision.reusedPrevious},
            {"no_feasible_goal", decision.noFeasibleGoal},
            {"certificate_valid", decision.certificate.valid},
            {"certificate_rejection_reasons",
                    decision.certificate.rejectionReasons},
            {"rotation_deg", 60.0},
            {"goal_max_range_m", BRIDGE_SINGLE_LADDER_GOAL_MAX_RANGE_M},
            {"goal_min_separation_m",
                    BRIDGE_SINGLE_LADDER_GOAL_MIN_SEPARATION_M},
            {"world_min", {{"x", worldMin.x}, {"y", worldMin.y}}},
            {"world_max", {{"x", worldMax.x}, {"y", worldMax.y}}},
            {"bases", {
                    {{"id", 0}, {"x", bases.at(0).x}, {"y", bases.at(0).y}},
                    {{"id", 1}, {"x", bases.at(1).x}, {"y", bases.at(1).y}}}},
            {"fixed_references", referenceMap},
            {"unsearched_cells", unsearchedCells},
            {"candidates", candidates},
            {"goals", goals},
            {"edges", edges},
            {"mobile_separations", separations},
    };
}

inline BridgeSingleLadderGoalTuple bridgeBuildSingleLadderGoalTuple(
        const Point &base0,
        const Point &base1,
        const Point &leaderGoal) {
    const Point midpoint(
            0.5 * (base0.x + base1.x),
            0.5 * (base0.y + base1.y));
    const Point section(
            0.5 * (leaderGoal.x - midpoint.x),
            0.5 * (leaderGoal.y - midpoint.y));
    const double sinSixty = std::sqrt(3.0) * 0.5;
    const Point rotatedSection(
            0.5 * section.x - sinSixty * section.y,
            sinSixty * section.x + 0.5 * section.y);

    return {
            leaderGoal,
            {
                    midpoint + rotatedSection,
                    midpoint + section,
                    midpoint + section + rotatedSection,
                    leaderGoal,
            },
    };
}

inline BridgeSingleLadderGoalCertificate bridgeCertifySingleLadderGoalTuple(
        const BridgeSingleLadderGoalTuple &tuple,
        const Point &base0,
        const Point &base1,
        const BridgeFixedReferenceMap &references,
        const Point &worldMin,
        const Point &worldMax,
        double maxGoalRange,
        double minGoalSeparation) {
    BridgeSingleLadderGoalCertificate certificate;
    if (references != bridgeSingleTriangularLadderReferences()) {
        certificate.rejectionReasons.push_back("reference-map-mismatch");
    }

    auto finitePoint = [](const Point &point) {
        return std::isfinite(point.x) && std::isfinite(point.y);
    };
    if (!finitePoint(base0) || !finitePoint(base1)
        || !finitePoint(tuple.leaderGoal)
        || !finitePoint(worldMin) || !finitePoint(worldMax)
        || !std::isfinite(maxGoalRange)
        || !std::isfinite(minGoalSeparation)) {
        certificate.rejectionReasons.push_back("non-finite-input");
    }

    if (finitePoint(base0) && finitePoint(base1)
        && finitePoint(tuple.leaderGoal)) {
        const auto expected = bridgeBuildSingleLadderGoalTuple(
                base0, base1, tuple.leaderGoal);
        for (std::size_t index = 0; index < tuple.goals.size(); ++index) {
            if (tuple.goals.at(index).x != expected.goals.at(index).x
                || tuple.goals.at(index).y != expected.goals.at(index).y) {
                certificate.rejectionReasons.push_back(
                        "tuple-geometry-mismatch");
                break;
            }
        }
    }

    for (const Point &goal : tuple.goals) {
        if (!finitePoint(goal)) {
            certificate.rejectionReasons.push_back("non-finite-goal");
            break;
        }
        if (goal.x < worldMin.x || goal.x > worldMax.x
            || goal.y < worldMin.y || goal.y > worldMax.y) {
            certificate.rejectionReasons.push_back("goal-outside-world");
            break;
        }
    }

    const std::array<Point, 2> bases = {base0, base1};
    std::size_t edgeIndex = 0;
    const auto canonical = bridgeSingleTriangularLadderReferences();
    for (const auto &[ownerId, ownerReferences] : canonical) {
        const Point &owner = tuple.goals.at(static_cast<std::size_t>(ownerId - 1));
        for (const int anchorId : ownerReferences.anchorIds) {
            const Point &reference = tuple.goals.at(
                    static_cast<std::size_t>(anchorId - 1));
            const double length = owner.distance_to(reference);
            certificate.edges.at(edgeIndex++) = {
                    ownerId, "mobile", anchorId, length};
            if (!std::isfinite(length) || length > maxGoalRange) {
                certificate.rejectionReasons.push_back("assigned-edge-over-range");
            }
        }
        for (const int baseId : ownerReferences.baseIds) {
            const Point &reference = bases.at(static_cast<std::size_t>(baseId));
            const double length = owner.distance_to(reference);
            certificate.edges.at(edgeIndex++) = {
                    ownerId, "base", baseId, length};
            if (!std::isfinite(length) || length > maxGoalRange) {
                certificate.rejectionReasons.push_back("assigned-edge-over-range");
            }
        }
    }

    std::size_t separationIndex = 0;
    for (std::size_t first = 0; first < tuple.goals.size(); ++first) {
        for (std::size_t second = first + 1;
             second < tuple.goals.size(); ++second) {
            const double separation = tuple.goals.at(first).distance_to(
                    tuple.goals.at(second));
            certificate.mobileSeparations.at(separationIndex++) = separation;
            if (!std::isfinite(separation)
                || separation < minGoalSeparation) {
                certificate.rejectionReasons.push_back(
                        "mobile-separation-below-minimum");
            }
        }
    }

    certificate.valid = certificate.rejectionReasons.empty();
    return certificate;
}

#endif
