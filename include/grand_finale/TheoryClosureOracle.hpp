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
    bool fixture_label_valid = false;
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
    bool extended_initial_set_satisfied = false;
    bool hard_control_feasible_for_interval = false;
    bool reference_qualification_persists = false;
    bool feedback_regular = false;
};

struct FiniteSetTerminationPremises {
    bool finite_grid = false;
    bool current_terminal_in_mutual_class = false;
    FiniteServiceContract service;
    bool monotone_coverage_marks = false;
};

struct HybridJumpAudit {
    double horizon_s = 0.0;
    double minimum_dwell_s = 0.0;
    double sampled_period_s = 0.0;
    std::size_t external_episodes = 0;
    std::size_t maximum_atomic_jumps_per_episode = 0;
    std::size_t sampled_cycles = 0;
    std::size_t maximum_atomic_jumps_per_sample = 0;
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
    if (nodes.count(start) == 0 || !nodes.at(start).fixture_label_valid)
        throw std::invalid_argument("initial terminal fixture label must be valid");
    std::map<std::size_t, std::vector<std::size_t>> adjacency;
    for (const auto& edge : witnesses) {
        if (nodes.count(edge.from) == 0 || nodes.count(edge.to) == 0)
            throw std::invalid_argument("reachability edge has unknown terminal set");
        const std::size_t from = reverse ? edge.to : edge.from;
        const std::size_t to = reverse ? edge.from : edge.to;
        if (nodes.at(from).fixture_label_valid &&
            nodes.at(to).fixture_label_valid)
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

// Finite-label consistency oracle only. Edges are assumed witness labels; this
// helper does not verify full-state quantifiers or synthesize a policy.
inline std::set<std::size_t> finiteTerminalGraphMutualClass(
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

inline std::set<int> finiteGraphCoverageLabels(
    std::size_t initial,
    const std::vector<TheoryTerminalNode>& terminals,
    const std::vector<TheoryReachabilityEdge>& witnesses) {
    const auto nodes = theory_closure_oracle_detail::validatedNodes(terminals);
    const auto mutual = finiteTerminalGraphMutualClass(initial, terminals, witnesses);
    std::set<int> cells;
    for (std::size_t id : mutual) {
        cells.insert(
            nodes.at(id).robust_sensing_cells.begin(),
            nodes.at(id).robust_sensing_cells.end());
    }
    return cells;
}

inline bool finiteRankStrictlyDescends(
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

inline bool finiteSetTerminationPremisesHold(
    const FiniteSetTerminationPremises& premises) {
    return premises.finite_grid &&
        premises.current_terminal_in_mutual_class &&
        finiteServiceContractHolds(premises.service) &&
        premises.monotone_coverage_marks;
}

inline bool intervalSafetyClaimSupported(const IntervalSafetyPremises& premises) {
    return premises.uncertainty_contained_for_interval &&
        premises.extended_initial_set_satisfied &&
        premises.hard_control_feasible_for_interval &&
        premises.reference_qualification_persists &&
        premises.feedback_regular;
}

inline bool finiteJumpCountConsistent(const HybridJumpAudit& audit) {
    if (!std::isfinite(audit.horizon_s) || audit.horizon_s < 0.0 ||
        !std::isfinite(audit.minimum_dwell_s) ||
        audit.minimum_dwell_s <= 0.0 ||
        !std::isfinite(audit.sampled_period_s) ||
        audit.sampled_period_s <= 0.0 ||
        audit.maximum_atomic_jumps_per_episode == 0 ||
        audit.maximum_atomic_jumps_per_sample == 0)
        return false;
    const auto maximum_episodes = static_cast<std::size_t>(
        std::floor(audit.horizon_s / audit.minimum_dwell_s + 1.0e-12)) + 1;
    const auto maximum_samples = static_cast<std::size_t>(
        std::floor(audit.horizon_s / audit.sampled_period_s + 1.0e-12)) + 1;
    return audit.external_episodes <= maximum_episodes &&
        audit.sampled_cycles <= maximum_samples &&
        audit.observed_atomic_jumps <=
            audit.external_episodes * audit.maximum_atomic_jumps_per_episode +
            audit.sampled_cycles * audit.maximum_atomic_jumps_per_sample;
}

}  // namespace gf
