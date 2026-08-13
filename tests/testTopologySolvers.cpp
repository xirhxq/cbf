#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/HighsTopologySolver.hpp"
#ifdef ENABLE_GUROBI
#include "grand_finale/GurobiTopologySolver.hpp"
#endif

#include <algorithm>
#include <cstdint>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

namespace {

gf::TopologyRequest oracleRequest() {
    const std::vector<gf::DirectedEdge> eligible = {
        {10, 1}, {11, 1},
        {10, 2}, {1, 2}, {3, 2},
        {11, 3}, {1, 3}, {2, 3},
        {10, 4}, {2, 4}, {3, 4}};
    const std::vector<gf::DirectedEdge> old_edges = {
        {10, 1}, {11, 1}, {10, 2}, {1, 2},
        {11, 3}, {1, 3}, {10, 4}, {2, 4}};
    return gf::TopologyRequest{
        {1, 2, 3, 4}, {10, 11}, eligible, old_edges, 2, 2,
        {{"10->1", 1}, {"11->1", 1}, {"10->2", 1}, {"1->2", 2},
         {"3->2", 8}, {"11->3", 1}, {"1->3", 2}, {"2->3", 8},
         {"10->4", 1}, {"2->4", 3}, {"3->4", 4}},
        {},
        {{"10->1|11->1", 2}, {"10->2|1->2", 2},
         {"11->3|1->3", 2}, {"10->4|3->4", 3}}};
}

bool independentValid(
    const gf::TopologyRequest& request,
    const std::vector<gf::DirectedEdge>& selected) {
    std::set<gf::NodeId> mobiles(request.mobile_ids.begin(), request.mobile_ids.end());
    std::map<gf::NodeId, int> indegree;
    std::map<gf::NodeId, int> graph_indegree;
    std::map<gf::NodeId, std::vector<gf::NodeId>> adjacency;
    for (gf::NodeId id : request.mobile_ids) indegree[id] = graph_indegree[id] = 0;
    for (gf::NodeId id : request.fixed_ids) graph_indegree[id] = 0;
    for (const gf::DirectedEdge& edge : selected) {
        if (mobiles.count(edge.owner) == 0) return false;
        ++indegree[edge.owner];
        ++graph_indegree[edge.owner];
        adjacency[edge.reference].push_back(edge.owner);
    }
    for (gf::NodeId id : request.mobile_ids) {
        if (indegree[id] < static_cast<int>(request.min_indegree) ||
            indegree[id] > static_cast<int>(request.max_indegree)) return false;
    }
    std::queue<gf::NodeId> ready;
    for (const auto& [id, degree] : graph_indegree) if (degree == 0) ready.push(id);
    std::size_t visited = 0;
    while (!ready.empty()) {
        const gf::NodeId node = ready.front(); ready.pop(); ++visited;
        for (gf::NodeId owner : adjacency[node]) {
            if (--graph_indegree[owner] == 0) ready.push(owner);
        }
    }
    return visited == graph_indegree.size();
}

gf::LexicographicObjective independentObjective(
    const gf::TopologyRequest& request,
    const std::vector<gf::DirectedEdge>& selected) {
    std::set<std::string> ids;
    std::set<std::string> old;
    gf::LexicographicObjective result;
    for (const gf::DirectedEdge& edge : selected) {
        ids.insert(edge.id());
        const auto progress = request.progress_coefficients.find(edge.id());
        if (progress != request.progress_coefficients.end()) result.progress += progress->second;
        const auto fim = request.fim_linear_coefficients.find(edge.id());
        if (fim != request.fim_linear_coefficients.end()) result.fim_geometry += fim->second;
    }
    for (const gf::DirectedEdge& edge : request.old_edges) old.insert(edge.id());
    for (const auto& [key, coefficient] : request.fim_pair_coefficients) {
        const std::size_t delimiter = key.find('|');
        if (ids.count(key.substr(0, delimiter)) && ids.count(key.substr(delimiter + 1)))
            result.fim_geometry += coefficient;
    }
    int switches = 0;
    for (const std::string& id : ids) if (!old.count(id)) ++switches;
    for (const std::string& id : old) if (!ids.count(id)) ++switches;
    result.negative_switch_count = -switches;
    result.negative_edge_count = -static_cast<int>(ids.size());
    return result;
}

struct OracleResult {
    bool feasible = false;
    gf::LexicographicObjective objective;
};

OracleResult exhaustiveOracle(const gf::TopologyRequest& request) {
    OracleResult best;
    const std::uint64_t subset_count = std::uint64_t{1} << request.eligible_edges.size();
    for (std::uint64_t mask = 0; mask < subset_count; ++mask) {
        std::vector<gf::DirectedEdge> selected;
        for (std::size_t index = 0; index < request.eligible_edges.size(); ++index)
            if ((mask >> index) & 1U) selected.push_back(request.eligible_edges[index]);
        if (!independentValid(request, selected)) continue;
        const gf::LexicographicObjective objective = independentObjective(request, selected);
        if (!best.feasible || gf::lexicographicallyBetter(objective, best.objective)) {
            best = OracleResult{true, objective};
        }
    }
    return best;
}

void checkAgainstOracle(
    gf::TopologySolver& solver,
    const gf::TopologyModel& model,
    const OracleResult& oracle) {
    const gf::TopologySolution solution = solver.solve(model);
    INFO(solution.detail);
    REQUIRE(solution.status == gf::TopologySolveStatus::Optimal);
    CHECK(independentValid(model.request(), solution.edges));
    CHECK(solution.objective.progress == doctest::Approx(oracle.objective.progress));
    CHECK(solution.objective.fim_geometry == doctest::Approx(oracle.objective.fim_geometry));
    CHECK(solution.objective.negative_switch_count == oracle.objective.negative_switch_count);
    CHECK(solution.objective.negative_edge_count == oracle.objective.negative_edge_count);
}

}  // namespace

TEST_CASE("HiGHS exact-linearized MILP matches the independent 4+2 oracle") {
    const gf::TopologyRequest request = oracleRequest();
    const gf::TopologyModel model(request);
    const OracleResult oracle = exhaustiveOracle(request);
    REQUIRE(oracle.feasible);
    gf::HighsTopologySolver solver;
    checkAgainstOracle(solver, model, oracle);
}

#ifdef ENABLE_GUROBI
TEST_CASE("Gurobi native MIQP matches the independent 4+2 oracle") {
    const gf::TopologyRequest request = oracleRequest();
    const gf::TopologyModel model(request);
    const OracleResult oracle = exhaustiveOracle(request);
    REQUIRE(oracle.feasible);
    gf::GurobiTopologySolver solver;
    checkAgainstOracle(solver, model, oracle);
}
#endif

TEST_CASE("Topology adapters report infeasible when an owner has fewer than two candidates") {
    gf::TopologyRequest request = oracleRequest();
    request.eligible_edges.erase(
        std::remove_if(request.eligible_edges.begin(), request.eligible_edges.end(),
            [](const gf::DirectedEdge& edge) { return edge.owner == 4 && edge.reference != 10; }),
        request.eligible_edges.end());
    const gf::TopologyModel model(request);
    gf::HighsTopologySolver highs;
    CHECK(highs.solve(model).status == gf::TopologySolveStatus::Infeasible);
#ifdef ENABLE_GUROBI
    gf::GurobiTopologySolver gurobi;
    CHECK(gurobi.solve(model).status == gf::TopologySolveStatus::Infeasible);
#endif
}

TEST_CASE("No-good rejection makes both proposers return a different topology") {
    gf::TopologyRequest request = oracleRequest();
    gf::HighsTopologySolver highs;
    const auto first = highs.solve(gf::TopologyModel(request));
    REQUIRE(first.status == gf::TopologySolveStatus::Optimal);
    request.forbidden_topologies.push_back(first.edges);
    const auto second = highs.solve(gf::TopologyModel(request));
    REQUIRE(second.status == gf::TopologySolveStatus::Optimal);
    std::set<std::string> first_ids, second_ids;
    for (const auto& edge : first.edges) first_ids.insert(edge.id());
    for (const auto& edge : second.edges) second_ids.insert(edge.id());
    CHECK(second_ids != first_ids);
#ifdef ENABLE_GUROBI
    gf::TopologyRequest gurobi_request = oracleRequest();
    gf::GurobiTopologySolver gurobi;
    const auto gurobi_first = gurobi.solve(gf::TopologyModel(gurobi_request));
    REQUIRE(gurobi_first.status == gf::TopologySolveStatus::Optimal);
    gurobi_request.forbidden_topologies.push_back(gurobi_first.edges);
    const auto gurobi_second = gurobi.solve(gf::TopologyModel(gurobi_request));
    REQUIRE(gurobi_second.status == gf::TopologySolveStatus::Optimal);
    std::set<std::string> gurobi_first_ids, gurobi_second_ids;
    for (const auto& edge : gurobi_first.edges) gurobi_first_ids.insert(edge.id());
    for (const auto& edge : gurobi_second.edges) gurobi_second_ids.insert(edge.id());
    CHECK(gurobi_second_ids != gurobi_first_ids);
#endif
}
