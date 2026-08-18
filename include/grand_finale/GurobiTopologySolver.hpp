#pragma once

#include "grand_finale/TopologySolver.hpp"

#ifdef ENABLE_GUROBI
#include "gurobi_c++.h"

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

class GurobiTopologySolver : public TopologySolver {
public:
    TopologySolution solve(const TopologyModel& topology) override {
        try {
            GRBEnv environment(true);
            environment.set(GRB_IntParam_OutputFlag, 0);
            environment.set(GRB_IntParam_Seed, 2027);
            environment.set(GRB_DoubleParam_FeasibilityTol,
                            kTopologyFeasibilityTolerance);
            environment.set(GRB_DoubleParam_IntFeasTol,
                            kTopologyFeasibilityTolerance);
            environment.set(GRB_DoubleParam_OptimalityTol,
                            kTopologyFeasibilityTolerance);
            environment.set(GRB_DoubleParam_MIPGap, 0.0);
            environment.set(GRB_DoubleParam_MIPGapAbs, 0.0);
            environment.start();
            GRBModel model(environment);

            const auto& decisions = topology.edges();
            const auto& request = topology.request();
            for (NodeId owner : request.mobile_ids) {
                const auto candidate_count = std::count_if(
                    decisions.begin(), decisions.end(),
                    [owner](const EdgeDecision& decision) {
                        return decision.edge.owner == owner;
                    });
                if (candidate_count <
                    static_cast<std::ptrdiff_t>(request.min_indegree)) {
                    return TopologySolution{
                        TopologySolveStatus::Infeasible, {}, {}, {},
                        "owner has fewer candidates than the minimum indegree"};
                }
            }
            std::vector<GRBVar> edge_variables;
            std::map<std::string, std::size_t> edge_index;
            for (std::size_t index = 0; index < decisions.size(); ++index) {
                edge_variables.push_back(model.addVar(
                    0.0, 1.0, 0.0, GRB_BINARY,
                    "x_" + decisions[index].edge.id()));
                edge_index[decisions[index].edge.id()] = index;
            }
            std::map<NodeId, GRBVar> levels;
            for (NodeId id : request.mobile_ids) {
                levels.emplace(id, model.addVar(
                    1.0, static_cast<double>(request.mobile_ids.size()),
                    0.0, GRB_INTEGER, "level_" + std::to_string(id)));
            }

            for (NodeId owner : request.mobile_ids) {
                GRBLinExpr indegree = 0.0;
                for (std::size_t index = 0; index < decisions.size(); ++index)
                    if (decisions[index].edge.owner == owner) indegree += edge_variables[index];
                model.addConstr(indegree >= static_cast<double>(request.min_indegree));
                model.addConstr(indegree <= static_cast<double>(request.max_indegree));
            }
            for (const auto& edge:request.required_edges)
                model.addConstr(edge_variables.at(edge_index.at(edge.id()))==1.0);
            const double big_m = static_cast<double>(request.mobile_ids.size() + 1);
            std::set<NodeId> mobiles(request.mobile_ids.begin(), request.mobile_ids.end());
            for (std::size_t index = 0; index < decisions.size(); ++index) {
                const DirectedEdge& edge = decisions[index].edge;
                GRBLinExpr row = levels.at(edge.owner) - big_m * edge_variables[index];
                if (mobiles.count(edge.reference)) row -= levels.at(edge.reference);
                model.addConstr(row >= 1.0 - big_m);
            }
            for (const auto& forbidden : request.forbidden_topologies) {
                std::set<std::string> forbidden_ids;
                for (const DirectedEdge& edge : forbidden)
                    forbidden_ids.insert(edge.id());
                GRBLinExpr difference = 0.0;
                for (std::size_t index = 0; index < decisions.size(); ++index) {
                    if (forbidden_ids.count(decisions[index].edge.id()))
                        difference += 1.0 - edge_variables[index];
                    else
                        difference += edge_variables[index];
                }
                model.addConstr(difference >= 1.0);
            }

            const auto coefficient = [](const std::map<std::string, double>& values,
                                        const std::string& id) {
                const auto value = values.find(id);
                return value == values.end() ? 0.0 : value->second;
            };
            GRBQuadExpr progress = 0.0;
            GRBQuadExpr fim = 0.0;
            GRBQuadExpr switches = 0.0;
            GRBQuadExpr edge_count = 0.0;
            std::set<std::string> old_ids;
            for (const DirectedEdge& edge : request.old_edges) old_ids.insert(edge.id());
            for (std::size_t index = 0; index < decisions.size(); ++index) {
                const std::string id = decisions[index].edge.id();
                progress += coefficient(request.progress_coefficients, id) * edge_variables[index];
                fim += coefficient(request.fim_linear_coefficients, id) * edge_variables[index];
                switches += (old_ids.count(id) ? 1.0 : -1.0) * edge_variables[index];
                edge_count -= edge_variables[index];
            }
            for (const auto& [key, value] : request.fim_pair_coefficients) {
                const std::size_t delimiter = key.find('|');
                if (delimiter == std::string::npos) throw std::invalid_argument("invalid FIM pair key");
                fim += value * edge_variables.at(edge_index.at(key.substr(0, delimiter))) *
                             edge_variables.at(edge_index.at(key.substr(delimiter + 1)));
            }

            std::vector<GRBQuadExpr> objectives =
                {progress, fim, switches, edge_count};
            // Fifth and later levels are a frozen deterministic tie rule:
            // among equal four-layer optima, prefer the lexicographically
            // smallest sorted selected-edge sequence.
            for (const GRBVar& edge : edge_variables) {
                GRBQuadExpr tie = 0.0;
                tie += edge;
                objectives.push_back(tie);
            }
            for (std::size_t pass = 0; pass < objectives.size(); ++pass) {
                model.setObjective(objectives[pass], GRB_MAXIMIZE);
                model.optimize();
                const int status = model.get(GRB_IntAttr_Status);
                if (status == GRB_INFEASIBLE) {
                    return TopologySolution{TopologySolveStatus::Infeasible};
                }
                if (status != GRB_OPTIMAL) {
                    return TopologySolution{
                        TopologySolveStatus::Error, {}, {}, {},
                        "Gurobi did not prove an optimal topology"};
                }
                if (pass + 1 < objectives.size()) {
                    const double optimum = model.get(GRB_DoubleAttr_ObjVal);
                    model.addQConstr(objectives[pass] >=
                                     optimum -
                                     kTopologyLexicographicFreezeTolerance);
                }
            }

            std::vector<DirectedEdge> selected;
            for (std::size_t index = 0; index < decisions.size(); ++index)
                if (edge_variables[index].get(GRB_DoubleAttr_X) > 0.5)
                    selected.push_back(decisions[index].edge);
            const TopologyEvaluation evaluation = topology.evaluate(selected);
            if (!evaluation.valid) {
                return TopologySolution{
                    TopologySolveStatus::Error, {}, {}, {},
                    "Gurobi returned a hard-invalid topology"};
            }
            return TopologySolution{
                TopologySolveStatus::Optimal, selected, evaluation.levels,
                evaluation.objective, "optimal"};
        } catch (const GRBException& error) {
            return TopologySolution{
                TopologySolveStatus::Error, {}, {}, {}, error.getMessage()};
        } catch (const std::exception& error) {
            return TopologySolution{
                TopologySolveStatus::Error, {}, {}, {}, error.what()};
        }
    }
};

}  // namespace gf
#endif
