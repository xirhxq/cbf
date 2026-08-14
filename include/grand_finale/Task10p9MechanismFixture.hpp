#pragma once

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

namespace gf {

struct Task10p9Scenario {
    double map_width_m = 0.0;
    double map_height_m = 0.0;
    std::vector<NodeId> mobile_ids;
    std::vector<Eigen::Vector2d> mobile_positions;
    std::map<NodeId, Eigen::Vector2d> fixed_positions;
    std::vector<DirectedEdge> initial_topology;
};

struct ReferenceBallOverlap {
    bool strict = false;
    double center_distance_m = 0.0;
    double slack_m = 0.0;
};

inline ReferenceBallOverlap referenceBallOverlap(
    const Eigen::Vector2d& old_anchor,
    const Eigen::Vector2d& add_anchor,
    double keep_radius_m,
    double add_radius_m) {
    const double distance = (old_anchor - add_anchor).norm();
    const double slack = keep_radius_m + add_radius_m - distance;
    return {std::isfinite(slack) && slack > 0.0, distance, slack};
}

inline Task10p9Scenario task10p9EasyScenario() {
    return {20.0, 20.0, {1,2,3,4},
        {{7.0,7.0},{7.0,13.0},{12.0,7.0},{12.0,13.0}},
        {{10,{4.0,4.0}},{11,{4.0,16.0}},{12,{18.0,2.0}}},
        {{10,1},{11,1},{10,2},{1,2},
         {11,3},{1,3},{2,4},{3,4}}};
}

inline Task10p9Scenario task10p9ActiveScenario() {
    Task10p9Scenario result;
    result.map_width_m = 1700.0;
    result.map_height_m = 400.0;
    for (NodeId id=1; id<=14; ++id) result.mobile_ids.push_back(id);
    const std::vector<double> xs{790.0,798.0,806.0,814.0};
    const std::vector<double> ys{180.0,188.0,196.0,204.0};
    for (double y : ys) for (double x : xs) {
        if (result.mobile_positions.size() == 14) break;
        result.mobile_positions.push_back({x,y});
    }
    result.fixed_positions = {
        {100,{0.0,200.0}}, {101,{800.0,350.0}}, {102,{1600.0,200.0}}};
    for (NodeId owner : result.mobile_ids) {
        result.initial_topology.push_back({100,owner});
        result.initial_topology.push_back({101,owner});
    }
    return result;
}

inline Task10p9Scenario task10p9D1SquareScenario() {
    Task10p9Scenario result;
    result.map_width_m = 40.0;
    result.map_height_m = 40.0;
    for (NodeId id=1; id<=14; ++id) result.mobile_ids.push_back(id);
    const std::vector<double> xs{8.0,16.0,24.0,32.0};
    const std::vector<double> ys{8.0,16.0,24.0,32.0};
    for (double y : ys) for (double x : xs) {
        if (result.mobile_positions.size() == 14) break;
        result.mobile_positions.push_back({x,y});
    }
    result.fixed_positions = {
        {100,{2.0,2.0}}, {101,{2.0,38.0}}, {102,{38.0,2.0}}};
    for (NodeId owner : result.mobile_ids) {
        result.initial_topology.push_back({100,owner});
        result.initial_topology.push_back({
            static_cast<NodeId>(owner == 2 ? 1 : 101), owner});
    }
    return result;
}

inline Task10p9Scenario task10p9D1ElongatedScenario() {
    Task10p9Scenario result = task10p9D1SquareScenario();
    result.map_width_m = 80.0;
    result.fixed_positions = {
        {100,{2.0,2.0}}, {101,{2.0,38.0}}, {102,{78.0,2.0}}};
    result.mobile_positions.clear();
    const std::vector<double> xs{10.0,30.0,50.0,70.0};
    const std::vector<double> ys{8.0,16.0,24.0,32.0};
    for (double y : ys) for (double x : xs) {
        if (result.mobile_positions.size() == 14) break;
        result.mobile_positions.push_back({x,y});
    }
    return result;
}

inline bool containsEdge(
    const std::vector<DirectedEdge>& edges,
    const DirectedEdge& query) {
    return std::any_of(edges.begin(), edges.end(), [&](const auto& edge) {
        return edge.id() == query.id();
    });
}

inline double minimumReferenceBarrierH(
    const std::vector<CanonicalHardRow>& rows) {
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto& row : rows)
        if (row.kind == CanonicalHardRowKind::ReferenceDistance)
            minimum = std::min(minimum, row.barrier_h);
    return minimum;
}

inline std::vector<DirectedEdge> task10p9ActiveCandidateUniverse(
    const GrandFinaleSwarmAdapter& adapter) {
    const auto runtime = adapter.runtimeSnapshot();
    std::map<std::string,RangeLinkState> links;
    for (const auto& [id,link] : runtime.range_links)
        links[id] = {link.age_s,link.quality};
    const auto gates = buildEligibility(runtime.estimate, links,
        {adapter.config().add_reference_distance_m,
         adapter.config().reference_distance_m,
         adapter.config().maximum_range_aoi_s,
         adapter.config().maximum_reference_position_eigenvalue_m2,
         adapter.config().minimum_range_quality,
         adapter.config().uncertainty_sigma}, {});
    std::set<NodeId> current_references;
    for (const auto& edge : runtime.topology)
        if (edge.owner == 2) current_references.insert(edge.reference);
    std::vector<DirectedEdge> candidates;
    for (const auto& gate : gates.candidates)
        if (gate.edge.owner == 2 && gate.eligible &&
            !current_references.count(gate.edge.reference))
            candidates.push_back(gate.edge);
    std::sort(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b) { return a.id() < b.id(); });
    return candidates;
}

inline TopologyRequest task10p9ActiveProposal(
    const GrandFinaleSwarmAdapter& adapter) {
    const auto runtime = adapter.runtimeSnapshot();
    TopologyRequest request;
    request.mobile_ids = runtime.estimate.mobile_ids;
    for (const auto& [id, position] : runtime.estimate.fixed_positions) {
        (void)position; request.fixed_ids.push_back(id);
    }
    request.old_edges = runtime.topology;
    for (const auto& edge : runtime.topology)
        if (edge.id() != "100->2") request.eligible_edges.push_back(edge);
    const auto candidates = task10p9ActiveCandidateUniverse(adapter);
    request.eligible_edges.insert(
        request.eligible_edges.end(), candidates.begin(), candidates.end());
    request.min_indegree = 2;
    request.max_indegree = 2;
    for (const auto& edge : candidates)
        request.progress_coefficients[edge.id()] =
            edge.id() == "102->2" ? 1.0 : 0.0;
    return request;
}

}  // namespace gf
