#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace gf {

enum class VersionSemantics {
    FreshnessAlphaInvariant,
    AbsoluteNumericValue
};

enum class PolicyObservationScope {
    EstimatorHistoryOnly,
    TruthState,
    FutureMeasurement
};

struct MacroStateMargins {
    double tube = 0.0;
    double reference_initial_set = 0.0;
    double collision_initial_set = 0.0;
    double hard_control = 0.0;
    double fim = 0.0;
    double posterior = 0.0;
    double sensing = 0.0;
};

struct MacroStateCandidate {
    std::size_t id = 0;
    std::array<double, 4> lower{};
    std::array<double, 4> upper{};
    MacroStateMargins margins;
    std::size_t minimum_effective_reference_count = 0;
    bool dag_valid = false;
    bool canonical_bookkeeping = false;
    std::set<int> robust_sensing_cells;
    VersionSemantics version_semantics =
        VersionSemantics::FreshnessAlphaInvariant;
};

struct VerifiedMacroState {
    std::size_t id = 0;
    std::set<int> robust_sensing_cells;
};

struct WitnessIntervalAudit {
    double duration_s = 0.0;
    double tube_margin = 0.0;
    double reference_h_margin = 0.0;
    double reference_psi1_margin = 0.0;
    double collision_h_margin = 0.0;
    double collision_psi1_margin = 0.0;
    double hard_control_margin = 0.0;
    double fim_margin = 0.0;
    double posterior_margin = 0.0;
    std::size_t minimum_effective_reference_count = 0;
};

struct FiniteRefreshEvent {
    double time_s = 0.0;
    std::string link_id;
    double variance_m2 = 0.0;
    double quality = 0.0;
};

struct FiniteInformationContract {
    double horizon_s = 0.0;
    double maximum_aoi_s = 0.0;
    double initial_aoi_upper_s = 0.0;
    std::vector<std::string> required_links;
    std::vector<FiniteRefreshEvent> refreshes;
};

struct PrimitiveCandidate {
    std::size_t from = 0;
    std::size_t to = 0;
    PolicyObservationScope observation_scope =
        PolicyObservationScope::EstimatorHistoryOnly;
    double duration_upper_s = 0.0;
    std::vector<WitnessIntervalAudit> intervals;
    FiniteInformationContract information;
    double terminal_inclusion_margin = 0.0;
    double actual_control_margin = 0.0;
    std::size_t maximum_atomic_jumps = 0;
    std::size_t maximum_retreat_phases = 0;
};

struct VerifiedPrimitive {
    std::size_t from = 0;
    std::size_t to = 0;
    double duration_upper_s = 0.0;
};

struct VerifiedCore {
    std::size_t initial = 0;
    std::set<std::size_t> node_ids;
    std::vector<VerifiedPrimitive> edges;
    std::set<int> coverage_cells;
    std::map<std::size_t, VerifiedMacroState> nodes;
};

struct FrozenCoverageTour {
    std::vector<std::size_t> node_visit_order;
    std::set<int> serviced_cells;
    double duration_upper_s = 0.0;
};

namespace certified_coverage_roadmap_detail {

inline bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

inline bool finiteNonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

inline std::set<std::size_t> reachable(
    std::size_t start,
    const std::set<std::size_t>& nodes,
    const std::vector<VerifiedPrimitive>& edges,
    bool reverse) {
    std::map<std::size_t, std::vector<std::size_t>> adjacency;
    for (const auto& edge : edges) {
        const std::size_t from = reverse ? edge.to : edge.from;
        const std::size_t to = reverse ? edge.from : edge.to;
        if (nodes.count(from) != 0 && nodes.count(to) != 0)
            adjacency[from].push_back(to);
    }
    for (auto& [from, next] : adjacency) {
        (void)from;
        std::sort(next.begin(), next.end());
    }
    std::set<std::size_t> visited{start};
    std::queue<std::size_t> queue;
    queue.push(start);
    while (!queue.empty()) {
        const std::size_t current = queue.front();
        queue.pop();
        for (std::size_t next : adjacency[current]) {
            if (visited.insert(next).second) queue.push(next);
        }
    }
    return visited;
}

inline std::optional<std::vector<std::size_t>> shortestPath(
    std::size_t start,
    std::size_t goal,
    const VerifiedCore& core) {
    if (start == goal) return std::vector<std::size_t>{start};
    std::map<std::size_t, std::vector<std::size_t>> adjacency;
    for (const auto& edge : core.edges)
        adjacency[edge.from].push_back(edge.to);
    for (auto& [from, next] : adjacency) {
        (void)from;
        std::sort(next.begin(), next.end());
    }
    std::queue<std::size_t> queue;
    std::map<std::size_t, std::size_t> parent;
    queue.push(start);
    parent[start] = start;
    while (!queue.empty()) {
        const std::size_t current = queue.front();
        queue.pop();
        for (std::size_t next : adjacency[current]) {
            if (parent.count(next) != 0) continue;
            parent[next] = current;
            if (next == goal) {
                std::vector<std::size_t> path{goal};
                while (path.back() != start)
                    path.push_back(parent.at(path.back()));
                std::reverse(path.begin(), path.end());
                return path;
            }
            queue.push(next);
        }
    }
    return std::nullopt;
}

inline double edgeDuration(
    const VerifiedCore& core,
    std::size_t from,
    std::size_t to) {
    for (const auto& edge : core.edges)
        if (edge.from == from && edge.to == to)
            return edge.duration_upper_s;
    return std::numeric_limits<double>::infinity();
}

}  // namespace certified_coverage_roadmap_detail

inline std::optional<VerifiedMacroState> verifyMacroState(
    const MacroStateCandidate& candidate) {
    using certified_coverage_roadmap_detail::finitePositive;
    for (std::size_t index = 0; index < candidate.lower.size(); ++index) {
        if (!std::isfinite(candidate.lower[index]) ||
            !std::isfinite(candidate.upper[index]) ||
            candidate.lower[index] > candidate.upper[index])
            return std::nullopt;
    }
    const std::array<double, 7> margins{
        candidate.margins.tube,
        candidate.margins.reference_initial_set,
        candidate.margins.collision_initial_set,
        candidate.margins.hard_control,
        candidate.margins.fim,
        candidate.margins.posterior,
        candidate.margins.sensing};
    if (!std::all_of(margins.begin(), margins.end(), finitePositive) ||
        candidate.minimum_effective_reference_count < 2 ||
        !candidate.dag_valid || !candidate.canonical_bookkeeping ||
        candidate.robust_sensing_cells.empty() ||
        candidate.version_semantics !=
            VersionSemantics::FreshnessAlphaInvariant) {
        return std::nullopt;
    }
    return VerifiedMacroState{candidate.id, candidate.robust_sensing_cells};
}

inline std::optional<VerifiedPrimitive> verifyPrimitiveContract(
    const PrimitiveCandidate& candidate) {
    using namespace certified_coverage_roadmap_detail;
    if (candidate.from == candidate.to ||
        candidate.observation_scope !=
            PolicyObservationScope::EstimatorHistoryOnly ||
        !finitePositive(candidate.duration_upper_s) ||
        candidate.intervals.empty() ||
        !finitePositive(candidate.information.horizon_s) ||
        candidate.information.horizon_s > candidate.duration_upper_s ||
        !finitePositive(candidate.information.maximum_aoi_s) ||
        !finiteNonnegative(candidate.information.initial_aoi_upper_s) ||
        candidate.information.initial_aoi_upper_s >
            candidate.information.maximum_aoi_s ||
        candidate.information.required_links.empty() ||
        !finiteNonnegative(candidate.terminal_inclusion_margin) ||
        !finiteNonnegative(candidate.actual_control_margin) ||
        candidate.maximum_atomic_jumps == 0 ||
        candidate.maximum_retreat_phases > 1) {
        return std::nullopt;
    }
    double audited_time = 0.0;
    for (const auto& interval : candidate.intervals) {
        const std::array<double, 9> margins{
            interval.tube_margin,
            interval.reference_h_margin,
            interval.reference_psi1_margin,
            interval.collision_h_margin,
            interval.collision_psi1_margin,
            interval.hard_control_margin,
            interval.fim_margin,
            interval.posterior_margin,
            static_cast<double>(interval.minimum_effective_reference_count -
                                std::min<std::size_t>(
                                    interval.minimum_effective_reference_count, 2))};
        if (!finitePositive(interval.duration_s) ||
            !std::all_of(
                margins.begin(), margins.begin() + 8, finiteNonnegative) ||
            interval.minimum_effective_reference_count < 2)
            return std::nullopt;
        audited_time += interval.duration_s;
    }
    if (audited_time + 1.0e-12 < candidate.duration_upper_s)
        return std::nullopt;

    std::set<std::string> refreshed;
    for (const auto& refresh : candidate.information.refreshes) {
        if (!finiteNonnegative(refresh.time_s) ||
            refresh.time_s > candidate.information.horizon_s ||
            refresh.link_id.empty() || !finitePositive(refresh.variance_m2) ||
            !finitePositive(refresh.quality) || refresh.quality > 1.0)
            return std::nullopt;
        refreshed.insert(refresh.link_id);
    }
    for (const auto& link : candidate.information.required_links)
        if (refreshed.count(link) == 0) return std::nullopt;
    return VerifiedPrimitive{
        candidate.from, candidate.to, candidate.duration_upper_s};
}

inline VerifiedCore buildVerifiedCore(
    std::size_t initial,
    const std::vector<VerifiedMacroState>& nodes,
    const std::vector<VerifiedPrimitive>& edges) {
    VerifiedCore result;
    result.initial = initial;
    std::set<std::size_t> ids;
    for (const auto& node : nodes) {
        if (!result.nodes.emplace(node.id, node).second)
            return result;
        ids.insert(node.id);
    }
    if (ids.count(initial) == 0) return result;
    const auto forward = certified_coverage_roadmap_detail::reachable(
        initial, ids, edges, false);
    const auto backward = certified_coverage_roadmap_detail::reachable(
        initial, ids, edges, true);
    for (std::size_t id : forward)
        if (backward.count(id) != 0) result.node_ids.insert(id);
    for (const auto& edge : edges)
        if (result.node_ids.count(edge.from) != 0 &&
            result.node_ids.count(edge.to) != 0)
            result.edges.push_back(edge);
    std::sort(result.edges.begin(), result.edges.end(), [](const auto& lhs, const auto& rhs) {
        return std::tie(lhs.from, lhs.to) < std::tie(rhs.from, rhs.to);
    });
    for (std::size_t id : result.node_ids) {
        result.coverage_cells.insert(
            result.nodes.at(id).robust_sensing_cells.begin(),
            result.nodes.at(id).robust_sensing_cells.end());
    }
    return result;
}

inline std::optional<FrozenCoverageTour> tryBuildFrozenCoverageTour(
    const VerifiedCore& core,
    std::size_t root,
    double service_time_upper_s) {
    using namespace certified_coverage_roadmap_detail;
    if (core.node_ids.count(root) == 0 || core.coverage_cells.empty() ||
        !finitePositive(service_time_upper_s))
        return std::nullopt;
    std::map<int, std::size_t> assigned;
    for (int cell : core.coverage_cells) {
        for (std::size_t id : core.node_ids) {
            if (core.nodes.at(id).robust_sensing_cells.count(cell) != 0) {
                assigned[cell] = id;
                break;
            }
        }
    }
    std::set<std::size_t> service_nodes;
    for (const auto& [cell, id] : assigned) {
        (void)cell;
        service_nodes.insert(id);
    }
    FrozenCoverageTour tour;
    tour.node_visit_order.push_back(root);
    std::size_t current = root;
    for (std::size_t target : service_nodes) {
        if (target != current) {
            const auto path = shortestPath(current, target, core);
            if (!path.has_value()) return std::nullopt;
            for (std::size_t index = 1; index < path->size(); ++index) {
                tour.duration_upper_s += edgeDuration(
                    core, (*path)[index - 1], (*path)[index]);
                tour.node_visit_order.push_back((*path)[index]);
            }
            current = target;
        }
        for (int cell : core.nodes.at(target).robust_sensing_cells)
            if (assigned[cell] == target) tour.serviced_cells.insert(cell);
        tour.duration_upper_s += service_time_upper_s;
    }
    if (current != root) {
        const auto path = shortestPath(current, root, core);
        if (!path.has_value()) return std::nullopt;
        for (std::size_t index = 1; index < path->size(); ++index) {
            tour.duration_upper_s += edgeDuration(
                core, (*path)[index - 1], (*path)[index]);
            tour.node_visit_order.push_back((*path)[index]);
        }
    }
    if (!std::isfinite(tour.duration_upper_s) ||
        tour.serviced_cells != core.coverage_cells)
        return std::nullopt;
    return tour;
}

inline FrozenCoverageTour buildFrozenCoverageTour(
    const VerifiedCore& core,
    std::size_t root,
    double service_time_upper_s) {
    const auto result = tryBuildFrozenCoverageTour(
        core, root, service_time_upper_s);
    return result.value_or(FrozenCoverageTour{});
}

inline double finiteTourUpperBound(const FrozenCoverageTour& tour) {
    return tour.duration_upper_s;
}

}  // namespace gf
