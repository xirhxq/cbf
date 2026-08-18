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
#include <sstream>

namespace gf {

inline bool highsStatusAllowsExecution(HighsStatus status) {
    return status != HighsStatus::kError;
}

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
        for (const auto& edge:request.required_edges)
            base_rows.push_back(Row{{{edge_index.at(edge.id()),1.0}},1.0,1.0});
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
        for (int edge = 0; edge < edge_count; ++edge) {
            std::vector<double> tie(variable_count, 0.0);
            tie[edge] = 1.0;
            objectives.push_back(std::move(tie));
        }

        std::vector<Row> freeze_rows;
        std::vector<double> solution;
        std::vector<double> pass_optima;
        std::ostringstream raw_status;
        for (int pass = 0; pass < static_cast<int>(objectives.size()); ++pass) {
            Highs highs;
            highs.setOptionValue("output_flag", false);
            highs.setOptionValue("random_seed", 2027);
            highs.setOptionValue("primal_feasibility_tolerance",
                                 kTopologyFeasibilityTolerance);
            highs.setOptionValue("dual_feasibility_tolerance",
                                 kTopologyFeasibilityTolerance);
            highs.setOptionValue("mip_feasibility_tolerance",
                                 kTopologyFeasibilityTolerance);
            highs.setOptionValue("mip_rel_gap", 0.0);
            highs.setOptionValue("mip_abs_gap", 0.0);
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
            const HighsStatus pass_status = highs.passModel(model);
            const HighsStatus run_status = highsStatusAllowsExecution(pass_status)
                ? highs.run() : HighsStatus::kError;
            const HighsModelStatus raw_model_status = highs.getModelStatus();
            if (pass > 0) raw_status << ';';
            raw_status << "pass" << pass
                       << ":passModel=" << highsStatusToString(pass_status)
                       << ",run=" << highsStatusToString(run_status)
                       << ",model=" << highs.modelStatusToString(
                              raw_model_status);
            if (!highsStatusAllowsExecution(pass_status) ||
                !highsStatusAllowsExecution(run_status)) {
                return TopologySolution{TopologySolveStatus::Error, {}, {}, {},
                    "HiGHS execution failed:" + raw_status.str()};
            }
            const HighsModelStatus status = raw_model_status;
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
            pass_optima.push_back(optimum);
            if (pass + 1 < static_cast<int>(objectives.size())) {
                Row freeze{{},
                    optimum - kTopologyLexicographicFreezeTolerance,
                    infinity};
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
        std::vector<double> recomputed{
            evaluation.objective.progress,
            evaluation.objective.fim_geometry,
            static_cast<double>(evaluation.objective.negative_switch_count) +
                static_cast<double>(old_ids.size()),
            static_cast<double>(evaluation.objective.negative_edge_count)};
        for (int edge = 0; edge < edge_count; ++edge)
            recomputed.push_back(solution[edge] > 0.5 ? 1.0 : 0.0);
        if (recomputed.size() != pass_optima.size())
            return TopologySolution{TopologySolveStatus::Error, {}, {}, {},
                "HiGHS lexicographic audit size mismatch"};
        for (std::size_t pass = 0; pass < recomputed.size(); ++pass)
            if (std::abs(recomputed[pass] - pass_optima[pass]) >
                kTopologyLexicographicFreezeTolerance) {
                return TopologySolution{TopologySolveStatus::Error, {}, {}, {},
                    "HiGHS lexicographic audit failed at pass " +
                    std::to_string(pass)};
            }
        return TopologySolution{
            TopologySolveStatus::Optimal, selected, evaluation.levels,
            evaluation.objective, raw_status.str()};
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
