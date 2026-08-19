#pragma once

#include "grand_finale/Task10p11sFull28dGateA.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gf {

struct Task10p11tPairRows {
    std::string base_id;
    CanonicalHardRow first;
    CanonicalHardRow second;
    double central_constant_lower = 0.0;
    double coefficient_reserve_per_owner = 0.0;
};

inline Task10p11tPairRows makeTask10p11tPairRows(
    const CanonicalHardRow& first, const CanonicalHardRow& second) {
    const bool pair_kind =
        first.kind == CanonicalHardRowKind::ReferenceDistance ||
        first.kind == CanonicalHardRowKind::Collision;
    if (!pair_kind || first.kind != second.kind ||
        !first.peer.has_value() || !second.peer.has_value() ||
        first.owner != *second.peer || second.owner != *first.peer ||
        std::abs(first.responsibility - 0.5) > 1e-12 ||
        std::abs(second.responsibility - 0.5) > 1e-12 ||
        task10p11sPairBaseId(first.id) != task10p11sPairBaseId(second.id) ||
        (first.control_coefficient + second.control_coefficient).norm() > 1e-9 ||
        std::abs(first.constant - second.constant) > 1e-8 ||
        std::abs(first.coefficient_uncertainty_reserve -
                 second.coefficient_uncertainty_reserve) > 1e-10 ||
        !std::isfinite(first.constant) ||
        !std::isfinite(first.coefficient_uncertainty_reserve) ||
        first.coefficient_uncertainty_reserve < 0.0) {
        throw std::invalid_argument(
            "Task 10.11t requires one coherent mobile-pair half-row pair");
    }
    Task10p11tPairRows result;
    result.base_id = task10p11sPairBaseId(first.id);
    result.first = first;
    result.second = second;
    if (result.second.owner < result.first.owner) {
        std::swap(result.first, result.second);
    }
    result.coefficient_reserve_per_owner =
        first.coefficient_uncertainty_reserve;
    // Frozen robust half rows are rho*C_lower - R at rho=1/2.
    result.central_constant_lower =
        2.0 * (first.constant + result.coefficient_reserve_per_owner);
    return result;
}

inline std::pair<double, double> task10p11tDynamicConstants(
    const Task10p11tPairRows& pair, double transfer_mps2) {
    if (!std::isfinite(transfer_mps2)) {
        throw std::invalid_argument("pair burden transfer must be finite");
    }
    return {
        pair.first.constant + transfer_mps2,
        pair.second.constant - transfer_mps2};
}

inline double task10p11tFullPairResidualFromLocalSum(
    const Task10p11tPairRows& pair,
    double first_local_residual,
    double second_local_residual) {
    return first_local_residual + second_local_residual +
        pair.coefficient_reserve_per_owner;
}

struct Task10p11tInterval {
    bool valid = false;
    bool feasible = false;
    double lower = std::numeric_limits<double>::quiet_NaN();
    double upper = std::numeric_limits<double>::quiet_NaN();
};

struct Task10p11tLocalReplay {
    bool feasible = false;
    Eigen::Vector2d control = Eigen::Vector2d::Zero();
    double objective = std::numeric_limits<double>::quiet_NaN();
    double minimum_residual = std::numeric_limits<double>::quiet_NaN();
    std::string limiting_row_id;
};

struct Task10p11tDynamicPairResult {
    bool valid = false;
    bool feasible = false;
    std::string reason;
    Task10p11tPairRows pair;
    double legacy_half_first_gamma =
        -std::numeric_limits<double>::infinity();
    double legacy_half_second_gamma =
        -std::numeric_limits<double>::infinity();
    Task10p11tInterval first_owner_interval;
    Task10p11tInterval second_owner_interval;
    Task10p11tInterval shared_interval;
    bool bounded_fraction_feasible = false;
    double selected_transfer_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double equivalent_fraction = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector2d first_control = Eigen::Vector2d::Zero();
    Eigen::Vector2d second_control = Eigen::Vector2d::Zero();
    double pair_nominal_objective =
        std::numeric_limits<double>::quiet_NaN();
    bool first_local_replay_feasible = false;
    bool second_local_replay_feasible = false;
    Eigen::Vector2d first_local_replay_control = Eigen::Vector2d::Zero();
    Eigen::Vector2d second_local_replay_control = Eigen::Vector2d::Zero();
    double minimum_independent_local_residual =
        std::numeric_limits<double>::quiet_NaN();
    double dynamic_local_sum_residual =
        std::numeric_limits<double>::quiet_NaN();
    double full_pair_residual = std::numeric_limits<double>::quiet_NaN();
};

namespace task10p11t_detail {

inline const CanonicalHardRow& targetRowForOwner(
    const Task10p11tPairRows& pair, NodeId owner) {
    if (owner == pair.first.owner) return pair.first;
    if (owner == pair.second.owner) return pair.second;
    throw std::invalid_argument("owner is outside selected mobile pair");
}

inline GRBLinExpr dynamicExpression(
    const CanonicalHardRow& row,
    const Task10p11tPairRows& pair,
    const GRBVar& ux,
    const GRBVar& uy,
    const GRBVar& transfer) {
    GRBLinExpr expression =
        row.control_coefficient.x() * ux +
        row.control_coefficient.y() * uy;
    if (row.id == pair.first.id) {
        expression += row.constant + transfer;
    } else if (row.id == pair.second.id) {
        expression += row.constant - transfer;
    } else {
        expression += row.constant;
    }
    return expression;
}

inline double dynamicResidual(
    const CanonicalHardRow& row,
    const Task10p11tPairRows& pair,
    double transfer_mps2,
    const Eigen::Vector2d& control) {
    double constant = row.constant;
    const auto dynamic = task10p11tDynamicConstants(pair, transfer_mps2);
    if (row.id == pair.first.id) constant = dynamic.first;
    if (row.id == pair.second.id) constant = dynamic.second;
    return row.control_coefficient.dot(control) + constant;
}

#ifdef ENABLE_GUROBI
inline void configureEnvironment(GRBEnv& environment) {
    environment.set(GRB_IntParam_OutputFlag, 0);
    environment.set(GRB_IntParam_Seed, 2027);
    environment.set(GRB_IntParam_Threads, 1);
    environment.set(GRB_IntParam_NumericFocus, 3);
    environment.set(GRB_IntParam_DualReductions, 0);
    environment.set(GRB_DoubleParam_FeasibilityTol, 1e-9);
    environment.set(GRB_DoubleParam_OptimalityTol, 1e-9);
}

inline Task10p11tInterval ownerTransferInterval(
    const std::vector<CanonicalHardRow>& rows,
    const Task10p11tPairRows& pair,
    NodeId owner,
    double acceleration_half_box) {
    Task10p11tInterval result;
    try {
        GRBEnv environment(true);
        configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        GRBVar ux = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar uy = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar transfer = model.addVar(
            -GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
        for (const CanonicalHardRow& row : rows) {
            if (row.owner != owner) continue;
            model.addConstr(
                dynamicExpression(row, pair, ux, uy, transfer) >= 0.0);
        }
        const bool first_owner = owner == pair.first.owner;
        GRBLinExpr objective = transfer;
        model.setObjective(
            objective, first_owner ? GRB_MINIMIZE : GRB_MAXIMIZE);
        model.optimize();
        result.valid = true;
        result.feasible = model.get(GRB_IntAttr_Status) == GRB_OPTIMAL;
        if (result.feasible) {
            if (first_owner) {
                result.lower = transfer.get(GRB_DoubleAttr_X);
                result.upper = std::numeric_limits<double>::infinity();
            } else {
                result.lower = -std::numeric_limits<double>::infinity();
                result.upper = transfer.get(GRB_DoubleAttr_X);
            }
        }
    } catch (const GRBException&) {
        result.valid = false;
    }
    return result;
}

inline Task10p11tLocalReplay localReplay(
    const std::vector<CanonicalHardRow>& rows,
    const Task10p11tPairRows& pair,
    NodeId owner,
    double transfer_value,
    double acceleration_half_box,
    const Eigen::Vector2d& nominal) {
    Task10p11tLocalReplay result;
    try {
        GRBEnv environment(true);
        configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        GRBVar ux = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar uy = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar transfer = model.addVar(
            transfer_value, transfer_value, 0.0, GRB_CONTINUOUS);
        for (const CanonicalHardRow& row : rows) {
            if (row.owner != owner) continue;
            model.addConstr(
                dynamicExpression(row, pair, ux, uy, transfer) >= 0.0);
        }
        GRBQuadExpr objective =
            (ux - nominal.x()) * (ux - nominal.x()) +
            (uy - nominal.y()) * (uy - nominal.y());
        model.setObjective(objective, GRB_MINIMIZE);
        model.optimize();
        if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) return result;
        result.feasible = true;
        result.control = {ux.get(GRB_DoubleAttr_X), uy.get(GRB_DoubleAttr_X)};
        result.objective = model.get(GRB_DoubleAttr_ObjVal);
        result.minimum_residual = std::numeric_limits<double>::infinity();
        for (const CanonicalHardRow& row : rows) {
            if (row.owner != owner) continue;
            const double residual = dynamicResidual(
                row, pair, transfer_value, result.control);
            if (residual < result.minimum_residual) {
                result.minimum_residual = residual;
                result.limiting_row_id = row.id;
            }
        }
    } catch (const GRBException&) {
        result.feasible = false;
    }
    return result;
}

#endif

}  // namespace task10p11t_detail

inline Task10p11tDynamicPairResult solveTask10p11tDynamicPair(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids,
    const std::map<NodeId, Eigen::Vector2d>& nominal_controls,
    double acceleration_half_box,
    const std::string& selected_pair_base_id) {
    Task10p11tDynamicPairResult result;
    if (mobile_ids.size() != 14 || !std::isfinite(acceleration_half_box) ||
        acceleration_half_box <= 0.0) {
        result.reason = "invalid_frozen_problem_boundary";
        return result;
    }
    std::vector<const CanonicalHardRow*> halves;
    for (const CanonicalHardRow& row : rows) {
        const bool pair_kind =
            row.kind == CanonicalHardRowKind::ReferenceDistance ||
            row.kind == CanonicalHardRowKind::Collision;
        if (pair_kind && row.peer.has_value() &&
            std::find(mobile_ids.begin(), mobile_ids.end(), *row.peer) !=
                mobile_ids.end() &&
            task10p11sPairBaseId(row.id) == selected_pair_base_id) {
            halves.push_back(&row);
        }
    }
    if (halves.size() != 2) {
        result.reason = "selected_conflict_does_not_resolve_to_one_row_pair";
        return result;
    }
    try {
        result.pair = makeTask10p11tPairRows(*halves[0], *halves[1]);
    } catch (const std::exception& error) {
        result.reason = error.what();
        return result;
    }
    if (nominal_controls.count(result.pair.first.owner) == 0 ||
        nominal_controls.count(result.pair.second.owner) == 0) {
        result.reason = "selected_pair_nominal_control_missing";
        return result;
    }
    const auto first_gamma = solveCanonicalGammaStar(
        rows, result.pair.first.owner, acceleration_half_box);
    const auto second_gamma = solveCanonicalGammaStar(
        rows, result.pair.second.owner, acceleration_half_box);
    if (!first_gamma.valid || !second_gamma.valid) {
        result.reason = "legacy_half_gamma_invalid";
        return result;
    }
    result.legacy_half_first_gamma = first_gamma.gamma;
    result.legacy_half_second_gamma = second_gamma.gamma;
#ifdef ENABLE_GUROBI
    result.first_owner_interval = task10p11t_detail::ownerTransferInterval(
        rows, result.pair, result.pair.first.owner, acceleration_half_box);
    result.second_owner_interval = task10p11t_detail::ownerTransferInterval(
        rows, result.pair, result.pair.second.owner, acceleration_half_box);
    result.shared_interval.valid = result.first_owner_interval.valid &&
        result.second_owner_interval.valid;
    if (result.shared_interval.valid &&
        result.first_owner_interval.feasible &&
        result.second_owner_interval.feasible) {
        result.shared_interval.lower = std::max(
            result.first_owner_interval.lower,
            result.second_owner_interval.lower);
        result.shared_interval.upper = std::min(
            result.first_owner_interval.upper,
            result.second_owner_interval.upper);
        result.shared_interval.feasible =
            result.shared_interval.lower <= result.shared_interval.upper + 1e-10;
    }
    result.valid = result.shared_interval.valid;
    if (!result.shared_interval.feasible) {
        result.reason = "dynamic_pair_responsibility_interval_empty";
        return result;
    }
    const double bounded_fraction_half_width =
        0.5 * std::abs(result.pair.central_constant_lower);
    result.bounded_fraction_feasible =
        std::max(result.shared_interval.lower, -bounded_fraction_half_width) <=
        std::min(result.shared_interval.upper, bounded_fraction_half_width) +
            1e-10;
    try {
        GRBEnv environment(true);
        task10p11t_detail::configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        GRBVar first_x = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar first_y = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar second_x = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar second_y = model.addVar(-acceleration_half_box,
            acceleration_half_box, 0.0, GRB_CONTINUOUS);
        GRBVar transfer = model.addVar(result.shared_interval.lower,
            result.shared_interval.upper, 0.0, GRB_CONTINUOUS);
        for (const CanonicalHardRow& row : rows) {
            if (row.owner == result.pair.first.owner) {
                model.addConstr(task10p11t_detail::dynamicExpression(
                    row, result.pair, first_x, first_y, transfer) >= 0.0);
            } else if (row.owner == result.pair.second.owner) {
                model.addConstr(task10p11t_detail::dynamicExpression(
                    row, result.pair, second_x, second_y, transfer) >= 0.0);
            }
        }
        const auto& first_nominal = nominal_controls.at(result.pair.first.owner);
        const auto& second_nominal = nominal_controls.at(result.pair.second.owner);
        GRBQuadExpr objective =
            (first_x - first_nominal.x()) * (first_x - first_nominal.x()) +
            (first_y - first_nominal.y()) * (first_y - first_nominal.y()) +
            (second_x - second_nominal.x()) * (second_x - second_nominal.x()) +
            (second_y - second_nominal.y()) * (second_y - second_nominal.y());
        model.setObjective(objective, GRB_MINIMIZE);
        model.optimize();
        if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
            result.reason = "pair_only_qp_not_optimal";
            return result;
        }
        result.first_control = {
            first_x.get(GRB_DoubleAttr_X), first_y.get(GRB_DoubleAttr_X)};
        result.second_control = {
            second_x.get(GRB_DoubleAttr_X), second_y.get(GRB_DoubleAttr_X)};
        const double transfer_lower =
            -result.pair.first.margin(result.first_control);
        const double transfer_upper =
            result.pair.second.margin(result.second_control);
        result.selected_transfer_mps2 =
            std::clamp(0.0, transfer_lower, transfer_upper);
        if (std::abs(result.pair.central_constant_lower) > 1e-15) {
            result.equivalent_fraction = 0.5 +
                result.selected_transfer_mps2 /
                    result.pair.central_constant_lower;
        }
        result.pair_nominal_objective =
            (result.first_control - first_nominal).squaredNorm() +
            (result.second_control - second_nominal).squaredNorm();
    } catch (const GRBException& error) {
        result.reason = "gurobi_error_" + std::to_string(error.getErrorCode());
        return result;
    }
    const auto first_replay = task10p11t_detail::localReplay(
        rows, result.pair, result.pair.first.owner,
        result.selected_transfer_mps2,
        acceleration_half_box,
        nominal_controls.at(result.pair.first.owner));
    const auto second_replay = task10p11t_detail::localReplay(
        rows, result.pair, result.pair.second.owner,
        result.selected_transfer_mps2,
        acceleration_half_box,
        nominal_controls.at(result.pair.second.owner));
    result.first_local_replay_feasible = first_replay.feasible;
    result.second_local_replay_feasible = second_replay.feasible;
    result.first_local_replay_control = first_replay.control;
    result.second_local_replay_control = second_replay.control;
    result.minimum_independent_local_residual = std::min(
        first_replay.minimum_residual, second_replay.minimum_residual);
    const double first_pair_residual = task10p11t_detail::dynamicResidual(
        result.pair.first, result.pair, result.selected_transfer_mps2,
        first_replay.control);
    const double second_pair_residual = task10p11t_detail::dynamicResidual(
        result.pair.second, result.pair, result.selected_transfer_mps2,
        second_replay.control);
    result.dynamic_local_sum_residual =
        first_pair_residual + second_pair_residual;
    result.full_pair_residual = task10p11tFullPairResidualFromLocalSum(
        result.pair, first_pair_residual, second_pair_residual);
    result.feasible = first_replay.feasible && second_replay.feasible &&
        result.minimum_independent_local_residual >= -1e-8 &&
        result.full_pair_residual >= -1e-8;
    result.reason = result.feasible
        ? "dynamic_pair_local_replay_feasible"
        : "dynamic_pair_local_replay_failed";
#else
    result.reason = "gurobi_not_enabled";
#endif
    return result;
}

inline nlohmann::json task10p11tResultJson(
    const Task10p11tDynamicPairResult& result) {
    const auto number = [](double value) {
        return std::isfinite(value)
            ? nlohmann::json(value) : nlohmann::json(nullptr);
    };
    const auto interval = [&](const Task10p11tInterval& value) {
        return nlohmann::json{{"valid", value.valid},
            {"feasible", value.feasible}, {"lower", number(value.lower)},
            {"upper", number(value.upper)}};
    };
    return {
        {"protocol", "task10p11t-dynamic-mobile-pair-offline-v1"},
        {"valid", result.valid}, {"feasible", result.feasible},
        {"reason", result.reason}, {"selected_pair", result.pair.base_id},
        {"first_owner", result.pair.first.owner},
        {"second_owner", result.pair.second.owner},
        {"central_constant_lower_mps2",
            number(result.pair.central_constant_lower)},
        {"coefficient_reserve_per_owner_mps2",
            number(result.pair.coefficient_reserve_per_owner)},
        {"legacy_half", {
            {"first_gamma_mps2", number(result.legacy_half_first_gamma)},
            {"second_gamma_mps2", number(result.legacy_half_second_gamma)}}},
        {"burden_transfer_intervals_mps2", {
            {"first_owner", interval(result.first_owner_interval)},
            {"second_owner", interval(result.second_owner_interval)},
            {"shared", interval(result.shared_interval)}}},
        {"bounded_fraction_feasible", result.bounded_fraction_feasible},
        {"selected_transfer_mps2", number(result.selected_transfer_mps2)},
        {"equivalent_unbounded_fraction",
            number(result.equivalent_fraction)},
        {"pair_nominal_objective", number(result.pair_nominal_objective)},
        {"pair_only_controls", {
            {std::to_string(result.pair.first.owner),
                {result.first_control.x(), result.first_control.y()}},
            {std::to_string(result.pair.second.owner),
                {result.second_control.x(), result.second_control.y()}}}},
        {"independent_distributed_replay", {
            {"first_feasible", result.first_local_replay_feasible},
            {"second_feasible", result.second_local_replay_feasible},
            {"first_control", {result.first_local_replay_control.x(),
                result.first_local_replay_control.y()}},
            {"second_control", {result.second_local_replay_control.x(),
                result.second_local_replay_control.y()}},
            {"minimum_local_residual_mps2",
                number(result.minimum_independent_local_residual)},
            {"dynamic_local_sum_residual_mps2",
                number(result.dynamic_local_sum_residual)},
            {"full_pair_residual_mps2", number(result.full_pair_residual)},
            {"full_pair_implied_by_local_rows",
                result.dynamic_local_sum_residual >= -1e-8 &&
                result.full_pair_residual >=
                    result.dynamic_local_sum_residual - 1e-12}}},
        {"architecture_claim",
            "signed_pair_transfer_plus_two_local_2d_qps"},
        {"production_controller_modified", false},
        {"trajectory_run_performed", false}};
}

inline nlohmann::json runTask10p11tFailureSnapshotOffline(
    const nlohmann::json& snapshot,
    const std::string& selected_pair_base_id) {
    const auto validation = validateTask10p11sSnapshot(snapshot);
    if (!validation.complete) {
        throw std::invalid_argument(
            "incomplete failure snapshot: " + validation.reason);
    }
    const auto request = task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto nominal = task10p11s_capture_detail::nominalFromJson(
        snapshot.at("nominal_controls"));
    const auto rows = buildCanonicalHardRows(request);
    nlohmann::json output = task10p11tResultJson(
        solveTask10p11tDynamicPair(rows, request.mobile_ids, nominal,
            request.acceleration_half_box, selected_pair_base_id));
    output["snapshot_complete"] = true;
    output["input_snapshot_protocol"] = snapshot.at("protocol");
    output["canonical_row_count"] = rows.size();
    return output;
}

}  // namespace gf
