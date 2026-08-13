#pragma once

#include "grand_finale/Types.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gf {

struct LexicographicObjective {
    double progress = 0.0;
    double fim_geometry = 0.0;
    std::int32_t negative_switch_count = 0;
    std::int32_t negative_edge_count = 0;
};

inline bool lexicographicallyBetter(
    const LexicographicObjective& lhs,
    const LexicographicObjective& rhs,
    double tolerance = 1e-10) {
    if (lhs.progress > rhs.progress + tolerance) return true;
    if (rhs.progress > lhs.progress + tolerance) return false;
    if (lhs.fim_geometry > rhs.fim_geometry + tolerance) return true;
    if (rhs.fim_geometry > lhs.fim_geometry + tolerance) return false;
    if (lhs.negative_switch_count != rhs.negative_switch_count) {
        return lhs.negative_switch_count > rhs.negative_switch_count;
    }
    return lhs.negative_edge_count > rhs.negative_edge_count;
}

struct TopologyRequest {
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::vector<DirectedEdge> eligible_edges;
    std::vector<DirectedEdge> old_edges;
    std::size_t min_indegree;
    std::size_t max_indegree;
    std::map<std::string, double> progress_coefficients;
    std::map<std::string, double> fim_linear_coefficients;
    std::map<std::string, double> fim_pair_coefficients;
};

struct EdgeDecision {
    DirectedEdge edge;
    std::size_t variable_index;
};

struct TopologyEvaluation {
    bool valid = false;
    std::string reason;
    std::map<NodeId, std::size_t> levels;
    LexicographicObjective objective;
};

enum class LinearSense { LessEqual, GreaterEqual };

struct BinaryProductRow {
    std::map<std::size_t, double> coefficients;
    LinearSense sense;
    double rhs;
};

inline std::array<BinaryProductRow, 3> binaryProductRows(
    std::size_t first,
    std::size_t second,
    std::size_t product) {
    return {
        BinaryProductRow{{{product, 1.0}, {first, -1.0}},
                         LinearSense::LessEqual, 0.0},
        BinaryProductRow{{{product, 1.0}, {second, -1.0}},
                         LinearSense::LessEqual, 0.0},
        BinaryProductRow{{{product, 1.0}, {first, -1.0}, {second, -1.0}},
                         LinearSense::GreaterEqual, -1.0}};
}

inline bool binaryProductRowsHold(
    const std::array<BinaryProductRow, 3>& rows,
    const std::map<std::size_t, int>& values,
    double tolerance = 1e-12) {
    for (const BinaryProductRow& row : rows) {
        double lhs = 0.0;
        for (const auto& [index, coefficient] : row.coefficients) {
            lhs += coefficient * values.at(index);
        }
        if (row.sense == LinearSense::LessEqual && lhs > row.rhs + tolerance) {
            return false;
        }
        if (row.sense == LinearSense::GreaterEqual && lhs < row.rhs - tolerance) {
            return false;
        }
    }
    return true;
}

class TopologyModel {
public:
    explicit TopologyModel(TopologyRequest request)
        : request_(std::move(request)) {
        canonicalizeNodes(request_.mobile_ids, "mobile");
        canonicalizeNodes(request_.fixed_ids, "fixed");
        std::set<NodeId> mobile_set(
            request_.mobile_ids.begin(), request_.mobile_ids.end());
        std::set<NodeId> known = mobile_set;
        for (NodeId id : request_.fixed_ids) {
            if (!known.insert(id).second) {
                throw std::invalid_argument(
                    "mobile and fixed node sets must be disjoint");
            }
        }
        if (request_.min_indegree < 2 ||
            request_.max_indegree < request_.min_indegree) {
            throw std::invalid_argument("invalid topology indegree bounds");
        }

        std::sort(
            request_.eligible_edges.begin(), request_.eligible_edges.end(),
            [](const DirectedEdge& lhs, const DirectedEdge& rhs) {
                return lhs.id() < rhs.id();
            });
        std::string previous;
        for (std::size_t index = 0;
             index < request_.eligible_edges.size(); ++index) {
            const DirectedEdge& edge = request_.eligible_edges[index];
            if (mobile_set.count(edge.owner) == 0) {
                throw std::invalid_argument(
                    "eligible edge owner must be a mobile node");
            }
            if (known.count(edge.reference) == 0) {
                throw std::invalid_argument(
                    "eligible edge reference must be a known mobile or fixed node");
            }
            if (index > 0 && edge.id() == previous) {
                throw std::invalid_argument("duplicate eligible edge");
            }
            previous = edge.id();
            edges_.push_back(EdgeDecision{edge, index});
            eligible_ids_.insert(edge.id());
        }
        for (const DirectedEdge& edge : request_.old_edges) {
            old_ids_.insert(edge.id());
        }
    }

    const TopologyRequest& request() const { return request_; }
    const std::vector<EdgeDecision>& edges() const { return edges_; }

    TopologyEvaluation evaluate(
        const std::vector<DirectedEdge>& selected_edges) const {
        TopologyEvaluation result;
        std::set<std::string> selected_ids;
        std::map<NodeId, std::size_t> indegree;
        std::map<NodeId, std::vector<NodeId>> adjacency;
        std::map<NodeId, std::size_t> graph_indegree;
        for (NodeId id : request_.fixed_ids) {
            indegree[id] = 0;
            graph_indegree[id] = 0;
        }
        for (NodeId id : request_.mobile_ids) {
            indegree[id] = 0;
            graph_indegree[id] = 0;
        }

        for (const DirectedEdge& edge : selected_edges) {
            if (eligible_ids_.count(edge.id()) == 0 ||
                !selected_ids.insert(edge.id()).second) {
                result.reason = "not_eligible";
                return result;
            }
            ++indegree[edge.owner];
            ++graph_indegree[edge.owner];
            adjacency[edge.reference].push_back(edge.owner);
        }
        for (NodeId owner : request_.mobile_ids) {
            if (indegree[owner] < request_.min_indegree ||
                indegree[owner] > request_.max_indegree) {
                result.reason = "indegree";
                return result;
            }
        }

        std::priority_queue<NodeId, std::vector<NodeId>, std::greater<NodeId>> ready;
        for (const auto& [id, degree] : graph_indegree) {
            if (degree == 0) ready.push(id);
        }
        std::size_t visited = 0;
        while (!ready.empty()) {
            const NodeId node = ready.top();
            ready.pop();
            ++visited;
            if (std::binary_search(
                    request_.fixed_ids.begin(), request_.fixed_ids.end(), node)) {
                result.levels[node] = 0;
            } else if (result.levels.count(node) == 0) {
                result.levels[node] = 1;
            }
            std::vector<NodeId>& successors = adjacency[node];
            std::sort(successors.begin(), successors.end());
            for (NodeId successor : successors) {
                result.levels[successor] = std::max(
                    result.levels[successor], result.levels[node] + 1);
                if (--graph_indegree[successor] == 0) {
                    ready.push(successor);
                }
            }
        }
        if (visited != graph_indegree.size()) {
            result.levels.clear();
            result.reason = "cycle";
            return result;
        }

        for (const std::string& id : selected_ids) {
            result.objective.progress += coefficient(
                request_.progress_coefficients, id);
            result.objective.fim_geometry += coefficient(
                request_.fim_linear_coefficients, id);
        }
        for (const auto& [pair_key, value] : request_.fim_pair_coefficients) {
            const std::size_t delimiter = pair_key.find('|');
            if (delimiter == std::string::npos) {
                throw std::invalid_argument("invalid FIM pair key");
            }
            if (selected_ids.count(pair_key.substr(0, delimiter)) != 0 &&
                selected_ids.count(pair_key.substr(delimiter + 1)) != 0) {
                result.objective.fim_geometry += value;
            }
        }
        std::size_t switches = 0;
        for (const std::string& id : selected_ids) {
            if (old_ids_.count(id) == 0) ++switches;
        }
        for (const std::string& id : old_ids_) {
            if (selected_ids.count(id) == 0) ++switches;
        }
        result.objective.negative_switch_count =
            -static_cast<std::int32_t>(switches);
        result.objective.negative_edge_count =
            -static_cast<std::int32_t>(selected_ids.size());
        result.valid = true;
        return result;
    }

private:
    static void canonicalizeNodes(
        std::vector<NodeId>& ids,
        const char* kind) {
        std::sort(ids.begin(), ids.end());
        if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
            throw std::invalid_argument(
                std::string("duplicate ") + kind + " node id");
        }
    }

    static double coefficient(
        const std::map<std::string, double>& coefficients,
        const std::string& id) {
        const auto value = coefficients.find(id);
        return value == coefficients.end() ? 0.0 : value->second;
    }

    TopologyRequest request_;
    std::vector<EdgeDecision> edges_;
    std::set<std::string> eligible_ids_;
    std::set<std::string> old_ids_;
};

}  // namespace gf
