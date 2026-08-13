#pragma once

#include "grand_finale/TopologySolver.hpp"

#ifdef ENABLE_HIGHS
#include "Highs.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace gf {

class HighsTopologySolver : public TopologySolver {
private:
    struct Row {
        std::map<int, double> coefficients;
        double lower;
        double upper;
    };

public:
    TopologySolution solve(const TopologyModel& topology) override {
        const auto& decisions = topology.edges();
        const auto& request = topology.request();
        const int edge_count = static_cast<int>(decisions.size());
        const int mobile_count = static_cast<int>(request.mobile_ids.size());
        for (NodeId owner : request.mobile_ids) {
            const auto candidate_count = std::count_if(
                decisions.begin(), decisions.end(),
                [owner](const EdgeDecision& decision) {
                    return decision.edge.owner == owner;
                });
            if (candidate_count < static_cast<std::ptrdiff_t>(request.min_indegree)) {
                return TopologySolution{
                    TopologySolveStatus::Infeasible, {}, {}, {},
                    "owner has fewer candidates than the minimum indegree"};
            }
        }
        std::map<std::string, int> edge_index;
        for (int index = 0; index < edge_count; ++index)
            edge_index[decisions[index].edge.id()] = index;

        struct Pair { int first; int second; double coefficient; };
        std::vector<Pair> pairs;
        for (const auto& [key, value] : request.fim_pair_coefficients) {
            const std::size_t delimiter = key.find('|');
            if (delimiter == std::string::npos)
                return TopologySolution{TopologySolveStatus::Error, {}, {}, {}, "invalid FIM pair key"};
            pairs.push_back(Pair{
                edge_index.at(key.substr(0, delimiter)),
                edge_index.at(key.substr(delimiter + 1)), value});
        }
        const int pair_offset = edge_count + mobile_count;
        const int variable_count = pair_offset + static_cast<int>(pairs.size());
        const double infinity = kHighsInf;
        std::vector<Row> base_rows;

        std::map<NodeId, int> level_index;
        for (int index = 0; index < mobile_count; ++index)
            level_index[request.mobile_ids[index]] = edge_count + index;
        for (NodeId owner : request.mobile_ids) {
            Row row{{}, static_cast<double>(request.min_indegree),
                    static_cast<double>(request.max_indegree)};
            for (int index = 0; index < edge_count; ++index)
                if (decisions[index].edge.owner == owner) row.coefficients[index] = 1.0;
            base_rows.push_back(row);
        }
        const double big_m = static_cast<double>(mobile_count + 1);
        for (int index = 0; index < edge_count; ++index) {
            const DirectedEdge& edge = decisions[index].edge;
            Row row{{{level_index.at(edge.owner), 1.0}, {index, -big_m}},
                    1.0 - big_m, infinity};
            const auto reference_level = level_index.find(edge.reference);
            if (reference_level != level_index.end())
                row.coefficients[reference_level->second] = -1.0;
            base_rows.push_back(row);
        }
        for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
            const int y = pair_offset + index;
            base_rows.push_back(Row{{{y, 1}, {pairs[index].first, -1}}, -infinity, 0});
            base_rows.push_back(Row{{{y, 1}, {pairs[index].second, -1}}, -infinity, 0});
            base_rows.push_back(Row{{{y, 1}, {pairs[index].first, -1},
                                     {pairs[index].second, -1}}, -1, infinity});
        }
        for (const auto& forbidden : request.forbidden_topologies) {
            std::set<std::string> forbidden_ids;
            for (const DirectedEdge& edge : forbidden)
                forbidden_ids.insert(edge.id());
            Row row{{}, -infinity,
                    static_cast<double>(forbidden_ids.size()) - 1.0};
            for (int index = 0; index < edge_count; ++index) {
                row.coefficients[index] =
                    forbidden_ids.count(decisions[index].edge.id()) ? 1.0 : -1.0;
            }
            base_rows.push_back(std::move(row));
        }

        const auto coefficient = [](const std::map<std::string, double>& values,
                                    const std::string& id) {
            const auto value = values.find(id);
            return value == values.end() ? 0.0 : value->second;
        };
        std::vector<std::vector<double>> objectives(
            4, std::vector<double>(variable_count, 0.0));
        std::set<std::string> old_ids;
        for (const DirectedEdge& edge : request.old_edges) old_ids.insert(edge.id());
        for (int index = 0; index < edge_count; ++index) {
            const std::string id = decisions[index].edge.id();
            objectives[0][index] = coefficient(request.progress_coefficients, id);
            objectives[1][index] = coefficient(request.fim_linear_coefficients, id);
            objectives[2][index] = old_ids.count(id) ? 1.0 : -1.0;
            objectives[3][index] = -1.0;
        }
        for (int index = 0; index < static_cast<int>(pairs.size()); ++index)
            objectives[1][pair_offset + index] = pairs[index].coefficient;

        std::vector<Row> freeze_rows;
        std::vector<double> solution;
        for (int pass = 0; pass < 4; ++pass) {
            Highs highs;
            highs.setOptionValue("output_flag", false);
            highs.setOptionValue("random_seed", 2027);
            HighsModel model;
            model.lp_.sense_ = ObjSense::kMaximize;
            model.lp_.num_col_ = variable_count;
            model.lp_.col_cost_ = objectives[pass];
            model.lp_.col_lower_.assign(variable_count, 0.0);
            model.lp_.col_upper_.assign(variable_count, 1.0);
            model.lp_.integrality_.assign(variable_count, HighsVarType::kInteger);
            for (int index = 0; index < mobile_count; ++index) {
                model.lp_.col_lower_[edge_count + index] = 1.0;
                model.lp_.col_upper_[edge_count + index] = mobile_count;
            }
            std::vector<Row> rows = base_rows;
            rows.insert(rows.end(), freeze_rows.begin(), freeze_rows.end());
            model.lp_.num_row_ = static_cast<int>(rows.size());
            model.lp_.row_lower_.reserve(rows.size());
            model.lp_.row_upper_.reserve(rows.size());
            model.lp_.a_matrix_.format_ = MatrixFormat::kRowwise;
            model.lp_.a_matrix_.start_.clear();
            model.lp_.a_matrix_.start_.push_back(0);
            for (const Row& row : rows) {
                model.lp_.row_lower_.push_back(row.lower);
                model.lp_.row_upper_.push_back(row.upper);
                for (const auto& [index, value] : row.coefficients) {
                    if (value == 0.0) continue;
                    model.lp_.a_matrix_.index_.push_back(index);
                    model.lp_.a_matrix_.value_.push_back(value);
                }
                model.lp_.a_matrix_.start_.push_back(
                    static_cast<HighsInt>(model.lp_.a_matrix_.index_.size()));
            }
            if (highs.passModel(model) != HighsStatus::kOk ||
                highs.run() != HighsStatus::kOk) {
                return TopologySolution{TopologySolveStatus::Error, {}, {}, {}, "HiGHS execution failed"};
            }
            const HighsModelStatus status = highs.getModelStatus();
            if (status == HighsModelStatus::kInfeasible)
                return TopologySolution{
                    TopologySolveStatus::Infeasible, {}, {}, {},
                    "HiGHS infeasible at lexicographic pass " +
                        std::to_string(pass)};
            if (status != HighsModelStatus::kOptimal)
                return TopologySolution{TopologySolveStatus::Error, {}, {}, {}, "HiGHS did not prove optimality"};
            solution = highs.getSolution().col_value;
            double optimum = 0.0;
            for (int index = 0; index < variable_count; ++index)
                optimum += objectives[pass][index] * solution[index];
            if (pass < 3) {
                Row freeze{{}, optimum - 1e-8, infinity};
                for (int index = 0; index < variable_count; ++index)
                    if (objectives[pass][index] != 0.0)
                        freeze.coefficients[index] = objectives[pass][index];
                freeze_rows.push_back(std::move(freeze));
            }
        }

        std::vector<DirectedEdge> selected;
        for (int index = 0; index < edge_count; ++index)
            if (solution[index] > 0.5) selected.push_back(decisions[index].edge);
        const TopologyEvaluation evaluation = topology.evaluate(selected);
        if (!evaluation.valid)
            return TopologySolution{TopologySolveStatus::Error, {}, {}, {}, "HiGHS returned a hard-invalid topology"};
        return TopologySolution{
            TopologySolveStatus::Optimal, selected, evaluation.levels,
            evaluation.objective, "optimal"};
    }
};

}  // namespace gf
#else
namespace gf {
class HighsTopologySolver : public TopologySolver {
public:
    TopologySolution solve(const TopologyModel&) override {
        return TopologySolution{TopologySolveStatus::Error, {}, {}, {}, "HiGHS is unavailable"};
    }
};
}  // namespace gf
#endif
