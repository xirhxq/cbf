#pragma once

#include <cstddef>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

namespace gf {

struct TheoryTerminalNode {
    std::size_t id = 0;
    bool full_state_certified = false;
    std::set<int> robust_sensing_cells;
};

struct TheoryReachabilityEdge {
    std::size_t from = 0;
    std::size_t to = 0;
};

struct FiniteServiceContract {
    bool frontier_selected_in_finite_wait = false;
    bool full_state_witness_certified = false;
    bool witness_completes_in_finite_time = false;
    bool no_infinite_reform_retreat_preemption = false;
};

struct IntervalSafetyPremises {
    bool uncertainty_contained_for_interval = false;
    bool hard_control_feasible_for_interval = false;
    bool feedback_regular = false;
};

struct HybridJumpAudit {
    double horizon_s = 0.0;
    double minimum_dwell_s = 0.0;
    std::size_t macro_jumps = 0;
    std::size_t maximum_atomic_jumps_per_macro = 0;
    std::size_t observed_atomic_jumps = 0;
};

namespace theory_closure_oracle_detail {

inline std::map<std::size_t, TheoryTerminalNode> validatedNodes(
    const std::vector<TheoryTerminalNode>& terminals) {
    std::map<std::size_t, TheoryTerminalNode> nodes;
    for (const auto& terminal : terminals) {
        if (!nodes.emplace(terminal.id, terminal).second)
            throw std::invalid_argument("duplicate terminal-set id");
    }
    return nodes;
}

inline std::set<std::size_t> reachable(
    std::size_t start,
    const std::map<std::size_t, TheoryTerminalNode>& nodes,
    const std::vector<TheoryReachabilityEdge>& witnesses,
    bool reverse) {
    if (nodes.count(start) == 0 || !nodes.at(start).full_state_certified)
        throw std::invalid_argument("initial terminal set must be certified");
    std::map<std::size_t, std::vector<std::size_t>> adjacency;
    for (const auto& edge : witnesses) {
        if (nodes.count(edge.from) == 0 || nodes.count(edge.to) == 0)
            throw std::invalid_argument("reachability edge has unknown terminal set");
        const std::size_t from = reverse ? edge.to : edge.from;
        const std::size_t to = reverse ? edge.from : edge.to;
        if (nodes.at(from).full_state_certified &&
            nodes.at(to).full_state_certified)
            adjacency[from].push_back(to);
    }
    std::set<std::size_t> visited{start};
    std::queue<std::size_t> frontier;
    frontier.push(start);
    while (!frontier.empty()) {
        const std::size_t current = frontier.front();
        frontier.pop();
        for (std::size_t next : adjacency[current]) {
            if (visited.insert(next).second) frontier.push(next);
        }
    }
    return visited;
}

}  // namespace theory_closure_oracle_detail

inline std::set<std::size_t> mutualTerminalReachability(
    std::size_t initial,
    const std::vector<TheoryTerminalNode>& terminals,
    const std::vector<TheoryReachabilityEdge>& witnesses) {
    const auto nodes = theory_closure_oracle_detail::validatedNodes(terminals);
    const auto forward = theory_closure_oracle_detail::reachable(
        initial, nodes, witnesses, false);
    const auto backward = theory_closure_oracle_detail::reachable(
        initial, nodes, witnesses, true);
    std::set<std::size_t> mutual;
    for (std::size_t id : forward) {
        if (backward.count(id) != 0) mutual.insert(id);
    }
    return mutual;
}

inline std::set<int> certifiedCoverageDomain(
    std::size_t initial,
    const std::vector<TheoryTerminalNode>& terminals,
    const std::vector<TheoryReachabilityEdge>& witnesses) {
    const auto nodes = theory_closure_oracle_detail::validatedNodes(terminals);
    const auto mutual = mutualTerminalReachability(initial, terminals, witnesses);
    std::set<int> cells;
    for (std::size_t id : mutual) {
        cells.insert(
            nodes.at(id).robust_sensing_cells.begin(),
            nodes.at(id).robust_sensing_cells.end());
    }
    return cells;
}

inline bool rankStrictlyDescends(
    const std::set<int>& domain,
    const std::set<int>& covered_before,
    const std::set<int>& covered_after,
    int serviced_cell) {
    if (domain.count(serviced_cell) == 0 ||
        covered_before.count(serviced_cell) != 0 ||
        covered_after.count(serviced_cell) == 0)
        return false;
    for (int cell : covered_before) {
        if (domain.count(cell) != 0 && covered_after.count(cell) == 0)
            return false;
    }
    std::size_t before_rank = 0;
    std::size_t after_rank = 0;
    for (int cell : domain) {
        before_rank += covered_before.count(cell) == 0 ? 1 : 0;
        after_rank += covered_after.count(cell) == 0 ? 1 : 0;
    }
    return after_rank < before_rank;
}

inline bool finiteServiceContractHolds(const FiniteServiceContract& contract) {
    return contract.frontier_selected_in_finite_wait &&
        contract.full_state_witness_certified &&
        contract.witness_completes_in_finite_time &&
        contract.no_infinite_reform_retreat_preemption;
}

inline bool longTermProgressClaimSupported(
    bool positive_gamma,
    bool snapshot_progress_compatible,
    const FiniteServiceContract& contract) {
    (void)positive_gamma;
    (void)snapshot_progress_compatible;
    return finiteServiceContractHolds(contract);
}

inline bool intervalSafetyClaimSupported(const IntervalSafetyPremises& premises) {
    return premises.uncertainty_contained_for_interval &&
        premises.hard_control_feasible_for_interval &&
        premises.feedback_regular;
}

inline bool boundedHybridJumpCount(const HybridJumpAudit& audit) {
    if (!std::isfinite(audit.horizon_s) || audit.horizon_s < 0.0 ||
        !std::isfinite(audit.minimum_dwell_s) ||
        audit.minimum_dwell_s <= 0.0 ||
        audit.maximum_atomic_jumps_per_macro == 0)
        return false;
    const auto maximum_macros = static_cast<std::size_t>(
        std::floor(audit.horizon_s / audit.minimum_dwell_s + 1.0e-12)) + 1;
    return audit.macro_jumps <= maximum_macros &&
        audit.observed_atomic_jumps <=
            audit.macro_jumps * audit.maximum_atomic_jumps_per_macro;
}

}  // namespace gf
