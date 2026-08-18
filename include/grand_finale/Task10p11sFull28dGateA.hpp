#pragma once

#include "grand_finale/Task10p11sSnapshotCapture.hpp"
#include "json.hpp"

#ifdef ENABLE_GUROBI
#include "gurobi_c++.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct Task10p11sRow28d {
    std::string id;
    Eigen::VectorXd coefficient;
    double constant = 0.0;
    bool coupled_mobile_pair = false;
    double uncertainty_reserve_counted_once = 0.0;

    double residual(const Eigen::VectorXd& controls) const {
        return coefficient.dot(controls) + constant;
    }
};

struct Task10p11sRows28d {
    std::vector<NodeId> mobile_ids;
    std::vector<Task10p11sRow28d> rows;
    std::size_t coupled_mobile_pair_count = 0;
    std::size_t uncoupled_row_count = 0;
};

inline std::string task10p11sPairBaseId(const std::string& id) {
    const std::string marker = ":owner:";
    const std::size_t position = id.rfind(marker);
    if (position == std::string::npos) {
        throw std::invalid_argument("mobile pair row lacks owner suffix: " + id);
    }
    return id.substr(0, position);
}

inline Task10p11sRows28d buildTask10p11sRows28d(
    const std::vector<CanonicalHardRow>& canonical_rows,
    const std::vector<NodeId>& mobile_ids,
    bool couple_mobile_pairs) {
    Task10p11sRows28d result;
    result.mobile_ids = mobile_ids;
    std::map<NodeId, std::size_t> control_offset;
    for (std::size_t index = 0; index < mobile_ids.size(); ++index) {
        if (!control_offset.emplace(mobile_ids[index], 2 * index).second) {
            throw std::invalid_argument("duplicate mobile id in 28D ordering");
        }
    }
    if (mobile_ids.size() != 14) {
        throw std::invalid_argument("Task 10.11s requires exactly 14 owners");
    }

    std::map<std::string, std::vector<const CanonicalHardRow*>> mobile_pairs;
    for (const CanonicalHardRow& row : canonical_rows) {
        const bool pair_kind =
            row.kind == CanonicalHardRowKind::ReferenceDistance ||
            row.kind == CanonicalHardRowKind::Collision;
        const bool mobile_peer = row.peer.has_value() &&
            control_offset.count(*row.peer) != 0;
        if (couple_mobile_pairs && pair_kind && mobile_peer) {
            mobile_pairs[task10p11sPairBaseId(row.id)].push_back(&row);
            continue;
        }

        Task10p11sRow28d expanded;
        expanded.id = row.id;
        expanded.coefficient = Eigen::VectorXd::Zero(2 * mobile_ids.size());
        const auto owner = control_offset.find(row.owner);
        if (owner == control_offset.end()) {
            throw std::invalid_argument("row owner missing from 28D ordering");
        }
        expanded.coefficient.segment<2>(owner->second) =
            row.control_coefficient;
        expanded.constant = row.constant;
        result.rows.push_back(std::move(expanded));
        ++result.uncoupled_row_count;
    }

    for (const auto& [base_id, halves] : mobile_pairs) {
        if (halves.size() != 2) {
            throw std::invalid_argument(
                "mobile pair does not have exactly two half rows: " + base_id);
        }
        const CanonicalHardRow& first = *halves[0];
        const CanonicalHardRow& second = *halves[1];
        if (!first.peer.has_value() || !second.peer.has_value() ||
            first.owner != *second.peer || second.owner != *first.peer ||
            std::abs(first.responsibility - 0.5) > 1e-12 ||
            std::abs(second.responsibility - 0.5) > 1e-12 ||
            (first.control_coefficient + second.control_coefficient).norm() >
                1e-9 ||
            std::abs(first.constant - second.constant) > 1e-8 ||
            std::abs(first.coefficient_uncertainty_reserve -
                     second.coefficient_uncertainty_reserve) > 1e-10) {
            throw std::invalid_argument(
                "incoherent mobile pair half rows: " + base_id);
        }

        Task10p11sRow28d coupled;
        coupled.id = base_id + ":full-pair-once-reserve";
        coupled.coefficient = Eigen::VectorXd::Zero(2 * mobile_ids.size());
        coupled.coefficient.segment<2>(control_offset.at(first.owner)) =
            first.control_coefficient;
        coupled.coefficient.segment<2>(control_offset.at(*first.peer)) =
            -first.control_coefficient;
        coupled.uncertainty_reserve_counted_once =
            first.coefficient_uncertainty_reserve;
        coupled.constant = 2.0 * first.constant +
            coupled.uncertainty_reserve_counted_once;
        coupled.coupled_mobile_pair = true;
        result.rows.push_back(std::move(coupled));
        ++result.coupled_mobile_pair_count;
    }
    std::sort(result.rows.begin(), result.rows.end(),
        [](const Task10p11sRow28d& left, const Task10p11sRow28d& right) {
            return left.id < right.id;
        });
    return result;
}

inline Eigen::VectorXd task10p11sOrderedControls(
    const std::vector<NodeId>& mobile_ids,
    const std::map<NodeId, Eigen::Vector2d>& controls) {
    Eigen::VectorXd ordered(2 * mobile_ids.size());
    for (std::size_t index = 0; index < mobile_ids.size(); ++index) {
        ordered.segment<2>(2 * index) = controls.at(mobile_ids[index]);
    }
    return ordered;
}

inline std::map<NodeId, Eigen::Vector2d> task10p11sControlMap(
    const std::vector<NodeId>& mobile_ids,
    const Eigen::VectorXd& controls) {
    if (controls.size() != static_cast<Eigen::Index>(2 * mobile_ids.size())) {
        throw std::invalid_argument("28D control vector has wrong dimension");
    }
    std::map<NodeId, Eigen::Vector2d> result;
    for (std::size_t index = 0; index < mobile_ids.size(); ++index) {
        result.emplace(mobile_ids[index], controls.segment<2>(2 * index));
    }
    return result;
}

struct Task10p11sQpResult {
    std::string status = "not_solved";
    bool feasible = false;
    Eigen::VectorXd controls;
    double objective = std::numeric_limits<double>::quiet_NaN();
    double minimum_residual = std::numeric_limits<double>::quiet_NaN();
    std::string limiting_row_id;
    double solve_time_s = 0.0;
};

inline Task10p11sQpResult solveTask10p11sQp(
    const Task10p11sRows28d& problem,
    const Eigen::VectorXd& nominal) {
    Task10p11sQpResult result;
#ifdef ENABLE_GUROBI
    try {
        if (nominal.size() != static_cast<Eigen::Index>(
                2 * problem.mobile_ids.size())) {
            throw std::invalid_argument("nominal objective is not 28D");
        }
        GRBEnv environment(true);
        environment.set(GRB_IntParam_OutputFlag, 0);
        environment.set(GRB_IntParam_Seed, 2027);
        environment.set(GRB_IntParam_Threads, 1);
        environment.set(GRB_IntParam_NumericFocus, 3);
        environment.set(GRB_IntParam_DualReductions, 0);
        environment.set(GRB_DoubleParam_FeasibilityTol, 1e-9);
        environment.set(GRB_DoubleParam_OptimalityTol, 1e-9);
        environment.set(GRB_DoubleParam_BarConvTol, 1e-12);
        environment.start();
        GRBModel model(environment);
        std::vector<GRBVar> variables;
        variables.reserve(static_cast<std::size_t>(nominal.size()));
        for (Eigen::Index index = 0; index < nominal.size(); ++index) {
            variables.push_back(model.addVar(
                -GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS));
        }
        for (const Task10p11sRow28d& row : problem.rows) {
            GRBLinExpr expression = row.constant;
            for (Eigen::Index index = 0; index < row.coefficient.size(); ++index) {
                if (row.coefficient(index) != 0.0) {
                    expression += row.coefficient(index) * variables[index];
                }
            }
            model.addConstr(expression >= 0.0);
        }
        GRBQuadExpr objective = 0.0;
        for (Eigen::Index index = 0; index < nominal.size(); ++index) {
            objective += (variables[index] - nominal(index)) *
                (variables[index] - nominal(index));
        }
        model.setObjective(objective, GRB_MINIMIZE);
        model.optimize();
        result.solve_time_s = model.get(GRB_DoubleAttr_Runtime);
        const int status = model.get(GRB_IntAttr_Status);
        if (status == GRB_INFEASIBLE) {
            result.status = "infeasible";
            return result;
        }
        if (status != GRB_OPTIMAL) {
            result.status = "gurobi_status_" + std::to_string(status);
            return result;
        }
        result.status = "optimal";
        result.feasible = true;
        result.controls.resize(nominal.size());
        for (Eigen::Index index = 0; index < nominal.size(); ++index) {
            result.controls(index) = variables[index].get(GRB_DoubleAttr_X);
        }
        result.objective = model.get(GRB_DoubleAttr_ObjVal);
        result.minimum_residual = std::numeric_limits<double>::infinity();
        for (const Task10p11sRow28d& row : problem.rows) {
            const double residual = row.residual(result.controls);
            if (residual < result.minimum_residual) {
                result.minimum_residual = residual;
                result.limiting_row_id = row.id;
            }
        }
    } catch (const GRBException& error) {
        result.status = "gurobi_error_" + std::to_string(error.getErrorCode()) +
            ":" + error.getMessage();
    } catch (const std::exception& error) {
        result.status = std::string("error:") + error.what();
    }
#else
    (void)problem;
    (void)nominal;
    result.status = "gurobi_not_enabled";
#endif
    return result;
}

inline nlohmann::json task10p11sQpJson(
    const Task10p11sRows28d& problem,
    const Task10p11sQpResult& result) {
    nlohmann::json controls = nullptr;
    if (result.feasible) {
        controls = nlohmann::json::object();
        const auto mapped = task10p11sControlMap(
            problem.mobile_ids, result.controls);
        for (const auto& [id, value] : mapped) {
            controls[std::to_string(id)] = {value.x(), value.y()};
        }
    }
    return {
        {"status", result.status},
        {"feasible", result.feasible},
        {"objective", std::isfinite(result.objective)
            ? nlohmann::json(result.objective) : nlohmann::json(nullptr)},
        {"minimum_compiled_residual_mps2",
         std::isfinite(result.minimum_residual)
            ? nlohmann::json(result.minimum_residual) : nlohmann::json(nullptr)},
        {"limiting_row_id", result.limiting_row_id},
        {"solve_time_s", result.solve_time_s},
        {"row_count", problem.rows.size()},
        {"coupled_mobile_pair_count", problem.coupled_mobile_pair_count},
        {"uncoupled_row_count", problem.uncoupled_row_count},
        {"controls", controls}};
}

inline nlohmann::json runTask10p11sFull28dGateA(
    const nlohmann::json& snapshot) {
    const auto validation = validateTask10p11sSnapshot(snapshot);
    if (!validation.complete) {
        throw std::invalid_argument(
            "incomplete failure snapshot: " + validation.reason);
    }
    const auto request =
        task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
    const auto nominal_map =
        task10p11s_capture_detail::nominalFromJson(
            snapshot.at("nominal_controls"));
    const Eigen::VectorXd nominal =
        task10p11sOrderedControls(request.mobile_ids, nominal_map);
    const auto canonical_rows = buildCanonicalHardRows(request);

    nlohmann::json local = nlohmann::json::object();
    bool every_local_feasible = true;
    NodeId limiting_owner = 0;
    double limiting_gamma = std::numeric_limits<double>::infinity();
    for (NodeId owner : request.mobile_ids) {
        const auto gamma = solveCanonicalGammaStar(
            canonical_rows, owner, request.acceleration_half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma)) {
            throw std::runtime_error("invalid owner gamma for owner " +
                                     std::to_string(owner));
        }
        local[std::to_string(owner)] = {
            {"gamma_mps2", gamma.gamma},
            {"feasible", gamma.gamma >= 0.0}};
        every_local_feasible = every_local_feasible && gamma.gamma >= 0.0;
        if (gamma.gamma < limiting_gamma) {
            limiting_gamma = gamma.gamma;
            limiting_owner = owner;
        }
    }

    const auto same_half_problem = buildTask10p11sRows28d(
        canonical_rows, request.mobile_ids, false);
    const auto same_half = solveTask10p11sQp(same_half_problem, nominal);
    const bool same_half_equivalent =
        same_half.feasible == every_local_feasible;

    const auto full_pair_problem = buildTask10p11sRows28d(
        canonical_rows, request.mobile_ids, true);
    const auto full_pair = solveTask10p11sQp(full_pair_problem, nominal);

    nlohmann::json successor = {
        {"performed", false},
        {"reason", "current_full_pair_infeasible_or_not_proven"}};
    if (full_pair.feasible) {
        const auto current_controls = task10p11sControlMap(
            request.mobile_ids, full_pair.controls);
        const auto successor_request = rebuildTask10p11sSuccessorRequest(
            snapshot, current_controls);
        const auto successor_canonical =
            buildCanonicalHardRows(successor_request);
        const auto successor_problem = buildTask10p11sRows28d(
            successor_canonical, successor_request.mobile_ids, true);
        const auto successor_solution = solveTask10p11sQp(
            successor_problem, full_pair.controls);
        successor = task10p11sQpJson(
            successor_problem, successor_solution);
        successor["performed"] = true;
        successor["prediction"] = "exact_zoh_no_measurement_one_step";
        successor["claim_boundary"] =
            "one_step_successor_feasibility_not_recursive_feasibility";
    }

    return {
        {"protocol", "task10p11s-offline-gate-a-full28d-v1"},
        {"snapshot_complete", true},
        {"owner_count", validation.owner_count},
        {"canonical_row_count", canonical_rows.size()},
        {"same_half", {
            {"centralized_qp", task10p11sQpJson(
                same_half_problem, same_half)},
            {"distributed_owner_results", local},
            {"all_distributed_local_feasible", every_local_feasible},
            {"block_separable_equivalence_verified", same_half_equivalent},
            {"limiting_owner", limiting_owner},
            {"limiting_gamma_mps2", limiting_gamma}}},
        {"full_pair", task10p11sQpJson(full_pair_problem, full_pair)},
        {"successor", successor},
        {"gate_b_run_performed", false},
        {"recursive_feasibility_claimed", false}};
}

}  // namespace gf
