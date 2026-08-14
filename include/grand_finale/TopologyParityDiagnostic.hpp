#pragma once

#include "grand_finale/TopologySolver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct ExhaustiveTopologyResult {
    bool feasible = false;
    std::vector<DirectedEdge> edges;
    LexicographicObjective objective;
};

inline std::vector<std::string> topologyEdgeIds(
    const std::vector<DirectedEdge>& edges) {
    std::vector<std::string> ids;
    for (const auto& edge : edges) ids.push_back(edge.id());
    std::sort(ids.begin(), ids.end());
    return ids;
}

inline bool topologyObjectiveEquivalent(
    const LexicographicObjective& a,
    const LexicographicObjective& b,
    double tolerance) {
    return std::abs(a.progress - b.progress) <= tolerance &&
           std::abs(a.fim_geometry - b.fim_geometry) <= tolerance &&
           a.negative_switch_count == b.negative_switch_count &&
           a.negative_edge_count == b.negative_edge_count;
}

inline ExhaustiveTopologyResult exhaustiveTopologyOracle(
    const TopologyModel& model,
    double tolerance) {
    const auto& decisions = model.edges();
    if (decisions.size() > 20)
        throw std::invalid_argument("exhaustive topology oracle limited to 20 edges");
    ExhaustiveTopologyResult best;
    const std::uint64_t count = std::uint64_t{1} << decisions.size();
    for (std::uint64_t mask = 0; mask < count; ++mask) {
        std::vector<DirectedEdge> selected;
        for (std::size_t i = 0; i < decisions.size(); ++i)
            if ((mask >> i) & 1U) selected.push_back(decisions[i].edge);
        const auto evaluation = model.evaluate(selected);
        if (!evaluation.valid) continue;
        const bool better = !best.feasible || lexicographicallyBetter(
            evaluation.objective, best.objective, tolerance);
        const bool tied = best.feasible && topologyObjectiveEquivalent(
            evaluation.objective, best.objective, tolerance);
        if (better || (tied && topologyEdgeIds(selected) <
                                topologyEdgeIds(best.edges))) {
            best = {true, selected, evaluation.objective};
        }
    }
    return best;
}

inline bool topologySelectionMatchesOracle(
    const ExhaustiveTopologyResult& oracle,
    const TopologySolution& solution) {
    return oracle.feasible && solution.status == TopologySolveStatus::Optimal &&
           topologyEdgeIds(oracle.edges) == topologyEdgeIds(solution.edges);
}

struct TopologyParityReport {
    bool exhaustive_feasible = false;
    bool gurobi_matches = false;
    bool highs_matches = false;
    bool selected_edge_tie_matches = false;
    std::string highs_detail;
    std::string reason;
};

inline TopologyParityReport compareTopologyParity(
    const TopologyModel& model,
    const TopologySolution& gurobi,
    const TopologySolution& highs,
    double tolerance) {
    TopologyParityReport report;
    const auto oracle = exhaustiveTopologyOracle(model, tolerance);
    report.exhaustive_feasible = oracle.feasible;
    report.gurobi_matches = gurobi.status == TopologySolveStatus::Optimal &&
        topologyObjectiveEquivalent(oracle.objective, gurobi.objective, tolerance);
    report.highs_matches = highs.status == TopologySolveStatus::Optimal &&
        topologyObjectiveEquivalent(oracle.objective, highs.objective, tolerance);
    const bool gurobi_tie = topologySelectionMatchesOracle(oracle, gurobi);
    const bool highs_tie = topologySelectionMatchesOracle(oracle, highs);
    report.selected_edge_tie_matches = gurobi_tie && highs_tie;
    report.highs_detail = highs.detail;
    report.reason = !oracle.feasible ? "oracle_infeasible" :
        !report.gurobi_matches ? "gurobi_objective" :
        !report.highs_matches ? "highs_objective" :
        !report.selected_edge_tie_matches ? "selected_edge_tie" : "parity";
    return report;
}

}  // namespace gf
