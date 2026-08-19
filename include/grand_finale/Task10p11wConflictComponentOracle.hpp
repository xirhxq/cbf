#pragma once

#include "grand_finale/Task10p11tDynamicPairResponsibility.hpp"

#include <filesystem>
#include <fstream>
#include <optional>
#include <queue>
#include <set>

namespace gf {

namespace task10p11w_detail {

constexpr double kSolverFeasibilityTolerance = 1.0e-9;
constexpr const char* kPair = "reference:2->4";

struct RestrictedResult {
    bool feasible = false;
    std::string status = "not_solved";
    Eigen::VectorXd controls;
    double objective = std::numeric_limits<double>::quiet_NaN();
    double margin = -std::numeric_limits<double>::infinity();
    double minimum_residual = std::numeric_limits<double>::quiet_NaN();
    std::string limiting_row;
};

inline nlohmann::json number(double value) {
    return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
}

inline nlohmann::json idsJson(const std::set<NodeId>& ids) {
    return std::vector<NodeId>(ids.begin(), ids.end());
}

inline std::set<NodeId> rowOwners(const Task10p11sRow28d& row,
                                  const std::vector<NodeId>& mobile_ids) {
    std::set<NodeId> owners;
    for (std::size_t index = 0; index < mobile_ids.size(); ++index) {
        if (row.coefficient.segment<2>(2 * index).norm() > 0.0)
            owners.insert(mobile_ids[index]);
    }
    return owners;
}

inline std::pair<double, std::string> minimumResidual(
    const Task10p11sRows28d& problem, const Eigen::VectorXd& controls) {
    double minimum = std::numeric_limits<double>::infinity();
    std::string id;
    for (const auto& row : problem.rows) {
        const double residual = row.residual(controls);
        if (residual < minimum) {
            minimum = residual;
            id = row.id;
        }
    }
    return {minimum, id};
}

inline RestrictedResult solveRestricted(
    const Task10p11sRows28d& problem, const Eigen::VectorXd& nominal,
    const Eigen::VectorXd& frozen, const std::set<NodeId>& component,
    bool maximize_margin) {
    RestrictedResult result;
#ifdef ENABLE_GUROBI
    try {
        if (nominal.size() != frozen.size() ||
            nominal.size() != static_cast<Eigen::Index>(2 * problem.mobile_ids.size()))
            throw std::invalid_argument("restricted objective dimension mismatch");
        GRBEnv environment(true);
        task10p11t_detail::configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        std::map<std::pair<std::size_t, int>, GRBVar> variable;
        for (std::size_t index = 0; index < problem.mobile_ids.size(); ++index)
            if (component.count(problem.mobile_ids[index]) != 0)
                for (int axis = 0; axis < 2; ++axis)
                    variable.emplace(std::make_pair(index, axis), model.addVar(
                        -GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS));
        std::optional<GRBVar> gamma;
        if (maximize_margin)
            gamma.emplace(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0,
                                       GRB_CONTINUOUS));
        for (const auto& row : problem.rows) {
            GRBLinExpr expression = row.constant;
            for (std::size_t index = 0; index < problem.mobile_ids.size(); ++index) {
                for (int axis = 0; axis < 2; ++axis) {
                    const double coefficient = row.coefficient(2 * index + axis);
                    if (coefficient == 0.0) continue;
                    const auto found = variable.find({index, axis});
                    if (found == variable.end())
                        expression += coefficient * frozen(2 * index + axis);
                    else
                        expression += coefficient * found->second;
                }
            }
            if (maximize_margin && row.participates_in_gamma)
                expression -= *gamma;
            model.addConstr(expression >= 0.0);
        }
        if (maximize_margin) {
            GRBLinExpr objective = *gamma;
            model.setObjective(objective, GRB_MAXIMIZE);
        } else {
            GRBQuadExpr objective = 0.0;
            for (const auto& [key, value] : variable) {
                const Eigen::Index offset = static_cast<Eigen::Index>(
                    2 * key.first + key.second);
                objective += (value - nominal(offset)) * (value - nominal(offset));
            }
            model.setObjective(objective, GRB_MINIMIZE);
        }
        model.optimize();
        const int status = model.get(GRB_IntAttr_Status);
        if (status != GRB_OPTIMAL) {
            result.status = status == GRB_INFEASIBLE ? "infeasible" :
                "gurobi_status_" + std::to_string(status);
            return result;
        }
        double optimized_margin = std::numeric_limits<double>::quiet_NaN();
        if (maximize_margin) {
            optimized_margin = gamma->get(GRB_DoubleAttr_X);
            GRBLinExpr fixed_margin = *gamma;
            model.addConstr(fixed_margin >= optimized_margin);
            GRBQuadExpr tie_break = 0.0;
            for (const auto& [key, value] : variable) {
                const Eigen::Index offset = static_cast<Eigen::Index>(
                    2 * key.first + key.second);
                tie_break += (value - nominal(offset)) *
                    (value - nominal(offset));
            }
            model.setObjective(tie_break, GRB_MINIMIZE);
            model.optimize();
            if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
                result.status = "maximum_margin_tie_break_failed";
                return result;
            }
        }
        result.feasible = true;
        result.status = "optimal";
        result.controls = frozen;
        for (const auto& [key, value] : variable)
            result.controls(2 * key.first + key.second) =
                value.get(GRB_DoubleAttr_X);
        const auto residual = minimumResidual(problem, result.controls);
        result.minimum_residual = residual.first;
        result.limiting_row = residual.second;
        result.margin = maximize_margin ? optimized_margin : result.minimum_residual;
        if (!maximize_margin) {
            result.objective = 0.0;
            for (const auto& [key, value] : variable) {
                const Eigen::Index offset = static_cast<Eigen::Index>(
                    2 * key.first + key.second);
                const double difference = result.controls(offset) - nominal(offset);
                result.objective += difference * difference;
            }
        }
    } catch (const GRBException& error) {
        result.status = "gurobi_error_" + std::to_string(error.getErrorCode());
    } catch (const std::exception& error) {
        result.status = std::string("error:") + error.what();
    }
#else
    (void)problem; (void)nominal; (void)frozen; (void)component;
    (void)maximize_margin;
    result.status = "gurobi_not_enabled";
#endif
    return result;
}

inline Task10p11tPairRows selectedPair(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids) {
    std::vector<const CanonicalHardRow*> halves;
    for (const auto& row : rows) {
        if (!row.peer.has_value() ||
            std::find(mobile_ids.begin(), mobile_ids.end(), *row.peer) ==
                mobile_ids.end()) continue;
        const bool pair_kind = row.kind == CanonicalHardRowKind::ReferenceDistance ||
            row.kind == CanonicalHardRowKind::Collision;
        if (pair_kind && task10p11sPairBaseId(row.id) == kPair)
            halves.push_back(&row);
    }
    if (halves.size() != 2)
        throw std::runtime_error("pair 2-4 is absent from canonical rows");
    return makeTask10p11tPairRows(*halves[0], *halves[1]);
}

inline Eigen::VectorXd frozenLocalControls(
    const std::vector<CanonicalHardRow>& rows,
    const CanonicalHardRowRequest& request,
    const std::map<NodeId, Eigen::Vector2d>& nominal) {
    const auto pair = selectedPair(rows, request.mobile_ids);
    std::map<NodeId, Eigen::Vector2d> controls;
    for (NodeId owner : request.mobile_ids) {
        const auto replay = task10p11t_detail::localReplay(
            rows, pair, owner, 0.0, request.acceleration_half_box,
            nominal.at(owner));
        if (owner != 2 && owner != 4 && !replay.feasible)
            throw std::runtime_error("non-component frozen local QP infeasible for owner " +
                                     std::to_string(owner));
        controls.emplace(owner, replay.feasible ? replay.control : nominal.at(owner));
    }
    return task10p11sOrderedControls(request.mobile_ids, controls);
}

inline nlohmann::json restrictedJson(const RestrictedResult& result,
                                     const Task10p11sRows28d& problem) {
    nlohmann::json controls = nullptr;
    if (result.feasible) {
        controls = nlohmann::json::object();
        for (const auto& [id, value] :
             task10p11sControlMap(problem.mobile_ids, result.controls))
            controls[std::to_string(id)] = {value.x(), value.y()};
    }
    return {{"status", result.status}, {"feasible", result.feasible},
        {"objective", number(result.objective)}, {"margin_mps2", number(result.margin)},
        {"minimum_full_row_residual_mps2", number(result.minimum_residual)},
        {"limiting_row_id", result.limiting_row}, {"controls", controls}};
}

inline nlohmann::json controlsJson(const std::vector<NodeId>& mobile_ids,
                                   const Eigen::VectorXd& controls) {
    nlohmann::json output = nlohmann::json::object();
    for (const auto& [id, value] : task10p11sControlMap(mobile_ids, controls))
        output[std::to_string(id)] = {value.x(), value.y()};
    return output;
}

inline nlohmann::json localIntervalJson(const Task10p11tDynamicPairResult& value) {
    return {{"valid", value.shared_interval.valid},
        {"feasible", value.shared_interval.feasible},
        {"lower_mps2", number(value.shared_interval.lower)},
        {"upper_mps2", number(value.shared_interval.upper)},
        {"reason", value.reason}};
}

struct SuccessorAudit {
    CanonicalHardRowRequest request;
    Task10p11sRows28d problem;
    RestrictedResult component;
    RestrictedResult component_margin;
    Task10p11sQpResult full;
    RestrictedResult full_margin;
    Task10p11tDynamicPairResult local;
};

inline SuccessorAudit successorAudit(
    const nlohmann::json& snapshot, const Eigen::VectorXd& controls,
    const std::set<NodeId>& component,
    const std::vector<NodeId>& mobile_ids) {
    SuccessorAudit result;
    const auto control_map = task10p11sControlMap(mobile_ids, controls);
    result.request = rebuildTask10p11sSuccessorRequest(snapshot, control_map);
    const auto rows = buildCanonicalHardRows(result.request);
    result.problem = buildTask10p11sRows28d(rows, result.request.mobile_ids, true);
    const Eigen::VectorXd successor_frozen = frozenLocalControls(
        rows, result.request, control_map);
    result.component = solveRestricted(
        result.problem, controls, successor_frozen, component, false);
    result.component_margin = solveRestricted(
        result.problem, controls, successor_frozen, component, true);
    result.full = solveTask10p11sQp(result.problem, controls);
    const std::set<NodeId> all(result.request.mobile_ids.begin(),
                               result.request.mobile_ids.end());
    result.full_margin = solveRestricted(
        result.problem, controls, controls, all, true);
    result.local = solveTask10p11tDynamicPair(
        rows, result.request.mobile_ids, control_map,
        result.request.acceleration_half_box, kPair);
    return result;
}

inline nlohmann::json successorJson(const SuccessorAudit& audit) {
    return {{"performed", true},
        {"prediction", "exact_zoh_no_measurement_one_step"},
        {"local_signed_transfer", localIntervalJson(audit.local)},
        {"component_feasibility", restrictedJson(audit.component, audit.problem)},
        {"component_maximum_margin", restrictedJson(
            audit.component_margin, audit.problem)},
        {"full_pair_feasibility", task10p11sQpJson(audit.problem, audit.full)},
        {"full_pair_maximum_margin", restrictedJson(
            audit.full_margin, audit.problem)},
        {"full_row_residual_recomputed", audit.full.feasible &&
            audit.full.minimum_residual >= -kSolverFeasibilityTolerance},
        {"recursive_feasibility_claimed", false}};
}

inline std::map<NodeId, std::set<NodeId>> activeGraph(
    const Task10p11sRows28d& problem, const Eigen::VectorXd& oracle_controls) {
    std::map<NodeId, std::set<NodeId>> graph;
    for (NodeId id : problem.mobile_ids) graph[id];
    for (const auto& row : problem.rows) {
        if (!row.coupled_mobile_pair ||
            row.residual(oracle_controls) > kSolverFeasibilityTolerance) continue;
        const auto owners = rowOwners(row, problem.mobile_ids);
        if (owners.size() != 2) continue;
        const NodeId first = *owners.begin();
        const NodeId second = *std::next(owners.begin());
        graph[first].insert(second);
        graph[second].insert(first);
    }
    return graph;
}

inline std::set<NodeId> minimumComponent(
    const Task10p11sRows28d& problem, const Eigen::VectorXd& nominal,
    const Eigen::VectorXd& frozen, const Eigen::VectorXd& full_oracle,
    nlohmann::json& attempts) {
    const auto graph = activeGraph(problem, full_oracle);
    std::queue<std::set<NodeId>> frontier;
    std::set<std::set<NodeId>> visited;
    frontier.push({2, 4});
    visited.insert({2, 4});
    while (!frontier.empty()) {
        const std::set<NodeId> component = frontier.front();
        frontier.pop();
        const auto solution = solveRestricted(
            problem, nominal, frozen, component, false);
        attempts.push_back({{"component", idsJson(component)},
            {"result", restrictedJson(solution, problem)}});
        if (solution.feasible &&
            solution.minimum_residual >= -kSolverFeasibilityTolerance)
            return component;
        std::set<NodeId> neighbors;
        for (NodeId owner : component)
            for (NodeId peer : graph.at(owner))
                if (component.count(peer) == 0) neighbors.insert(peer);
        for (NodeId peer : neighbors) {
            auto expanded = component;
            expanded.insert(peer);
            if (visited.insert(expanded).second) frontier.push(std::move(expanded));
        }
    }
    throw std::runtime_error(
        "no feasible component reachable on terminal active constraint graph");
}

struct FrameInput {
    std::filesystem::path path;
    nlohmann::json snapshot;
    CanonicalHardRowRequest request;
    std::vector<CanonicalHardRow> canonical_rows;
    Task10p11sRows28d problem;
    Eigen::VectorXd nominal;
    Eigen::VectorXd frozen;
    Task10p11sQpResult full;
    double time_s = 0.0;
};

inline FrameInput readFrame(const std::filesystem::path& path) {
    FrameInput frame;
    frame.path = path;
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot read checkpoint: " + path.string());
    input >> frame.snapshot;
    const auto validation = validateTask10p11sSnapshot(frame.snapshot);
    if (!validation.complete)
        throw std::runtime_error("checkpoint is not a complete oracle snapshot");
    frame.request = task10p11s_capture_detail::requestFromJson(
        frame.snapshot.at("canonical_request"));
    const auto nominal_map = task10p11s_capture_detail::nominalFromJson(
        frame.snapshot.at("nominal_controls"));
    frame.canonical_rows = buildCanonicalHardRows(frame.request);
    frame.problem = buildTask10p11sRows28d(
        frame.canonical_rows, frame.request.mobile_ids, true);
    frame.nominal = task10p11sOrderedControls(frame.request.mobile_ids, nominal_map);
    frame.frozen = frozenLocalControls(
        frame.canonical_rows, frame.request, nominal_map);
    frame.full = solveTask10p11sQp(frame.problem, frame.nominal);
    frame.time_s = frame.snapshot.at("runtime").at("runtime_s").get<double>();
    if (!frame.full.feasible)
        throw std::runtime_error("28D full-pair oracle unexpectedly infeasible");
    return frame;
}

}  // namespace task10p11w_detail

inline nlohmann::json runTask10p11wOfflineOracle(
    const std::vector<std::filesystem::path>& checkpoint_paths) {
    using namespace task10p11w_detail;
    if (checkpoint_paths.size() != 6)
        throw std::invalid_argument("Task 10.11w requires exactly six checkpoints");
    std::vector<FrameInput> inputs;
    for (const auto& path : checkpoint_paths) inputs.push_back(readFrame(path));
    for (std::size_t index = 0; index < inputs.size(); ++index) {
        const double expected = 132.4 + 0.1 * static_cast<double>(index);
        if (std::abs(inputs[index].time_s - expected) > 1.0e-8)
            throw std::invalid_argument("checkpoint time/order mismatch");
    }

    nlohmann::json search_attempts = nlohmann::json::array();
    const FrameInput& terminal = inputs.back();
    const std::set<NodeId> component = minimumComponent(
        terminal.problem, terminal.nominal, terminal.frozen,
        terminal.full.controls, search_attempts);
    const bool pair_4d_sufficient = component == std::set<NodeId>{2, 4};

    nlohmann::json frames = nlohmann::json::array();
    std::optional<double> earliest_prevention;
    std::optional<double> last_recovery;
    bool every_selected_successor = true;
    bool every_pair_current = true;
    bool every_pair_successor = true;
    bool every_fallback_restores_local = true;
    double maximum_selected_deviation = 0.0;
    for (const auto& frame : inputs) {
        const auto current_nominal = task10p11s_capture_detail::nominalFromJson(
            frame.snapshot.at("nominal_controls"));
        const auto current_local = solveTask10p11tDynamicPair(
            frame.canonical_rows, frame.request.mobile_ids, current_nominal,
            frame.request.acceleration_half_box, kPair);
        const std::set<NodeId> pair_component{2, 4};
        const auto pair_solution = solveRestricted(
            frame.problem, frame.nominal, frame.frozen, pair_component, false);
        const auto component_solution = solveRestricted(
            frame.problem, frame.nominal, frame.frozen, component, false);
        if (!component_solution.feasible)
            throw std::runtime_error("terminal-minimum component failed on an earlier frame");
        const auto pair_successor = pair_solution.feasible
            ? std::optional<SuccessorAudit>(successorAudit(
                frame.snapshot, pair_solution.controls, pair_component,
                frame.request.mobile_ids)) : std::nullopt;
        const auto component_successor = successorAudit(
            frame.snapshot, component_solution.controls, component,
            frame.request.mobile_ids);
        const auto full_successor = successorAudit(
            frame.snapshot, frame.full.controls,
            std::set<NodeId>(frame.request.mobile_ids.begin(),
                             frame.request.mobile_ids.end()),
            frame.request.mobile_ids);

        const auto maximum_margin = solveRestricted(
            frame.problem, frame.nominal, frame.frozen, component, true);
        if (!maximum_margin.feasible)
            throw std::runtime_error("maximum-margin component command unavailable");
        nlohmann::json candidates = nlohmann::json::array();
        std::vector<SuccessorAudit> candidate_audits;
        std::vector<Eigen::VectorXd> candidate_controls;
        for (std::size_t index = 0; index <= 8; ++index) {
            const double alpha = static_cast<double>(index) / 8.0;
            const Eigen::VectorXd controls =
                (1.0 - alpha) * component_solution.controls +
                alpha * maximum_margin.controls;
            const auto residual = minimumResidual(frame.problem, controls);
            auto audit = successorAudit(
                frame.snapshot, controls, component, frame.request.mobile_ids);
            const double deviation =
                (controls - component_solution.controls).norm();
            const double score = std::min(
                audit.component_margin.margin, audit.full_margin.margin);
            candidates.push_back({{"index", index}, {"alpha", alpha},
                {"controls", controlsJson(frame.request.mobile_ids, controls)},
                {"current_feasible", residual.first >=
                    -kSolverFeasibilityTolerance},
                {"current_minimum_full_row_residual_mps2", residual.first},
                {"current_limiting_row_id", residual.second},
                {"coverage_control_deviation_l2_mps2", deviation},
                {"predicted_component_full_margin_mps2", number(score)},
                {"successor", successorJson(audit)}});
            candidate_controls.push_back(controls);
            candidate_audits.push_back(std::move(audit));
        }

        const bool coverage_predicts_empty =
            !candidate_audits.front().local.shared_interval.feasible;
        for (const auto& audit : candidate_audits)
            if (audit.local.shared_interval.feasible)
                last_recovery = frame.time_s;
        std::size_t predicted_argmax = 0;
        double predicted_best = -std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < candidate_audits.size(); ++index) {
            const double score = std::min(
                candidate_audits[index].component_margin.margin,
                candidate_audits[index].full_margin.margin);
            if (score > predicted_best) {
                predicted_best = score;
                predicted_argmax = index;
            }
        }
        std::size_t selected = 0;
        std::string selection = "coverage_first";
        if (coverage_predicts_empty) {
            if (!earliest_prevention.has_value()) earliest_prevention = frame.time_s;
            std::optional<std::size_t> restorative;
            for (std::size_t index = 0; index < candidate_audits.size(); ++index) {
                const auto& audit = candidate_audits[index];
                if (audit.local.shared_interval.feasible &&
                    audit.component.feasible && audit.full.feasible) {
                    restorative = index;
                    break;
                }
            }
            if (restorative.has_value()) {
                selected = *restorative;
                selection = "least_coverage_deviation_restoring_local";
            } else {
                selection = "second_order_inspired_maximum_predicted_margin_fallback";
                selected = predicted_argmax;
            }
        }
        const auto& selected_audit = candidate_audits[selected];
        const bool selected_successor = selected_audit.full.feasible &&
            selected_audit.component.feasible;
        const bool fallback_selected = selection ==
            "second_order_inspired_maximum_predicted_margin_fallback";
        if (fallback_selected)
            every_fallback_restores_local = every_fallback_restores_local &&
                selected_audit.local.shared_interval.feasible;
        maximum_selected_deviation = std::max(maximum_selected_deviation,
            (candidate_controls[selected] - component_solution.controls).norm());
        every_selected_successor = every_selected_successor && selected_successor;
        every_pair_current = every_pair_current && pair_solution.feasible;
        every_pair_successor = every_pair_successor &&
            pair_successor.has_value() && pair_successor->component.feasible &&
            pair_successor->full.feasible;
        candidates[selected]["selected"] = true;

        frames.push_back({{"time_s", frame.time_s},
            {"checkpoint", frame.path.filename().string()},
            {"current_local_signed_transfer", localIntervalJson(current_local)},
            {"full_28d_oracle", task10p11sQpJson(frame.problem, frame.full)},
            {"full_28d_successor", successorJson(full_successor)},
            {"component_search", {{"pair_2_4_tested", true},
                {"pair_2_4_4d", restrictedJson(pair_solution, frame.problem)},
                {"pair_2_4_successor", pair_successor.has_value()
                    ? successorJson(*pair_successor)
                    : nlohmann::json{{"performed", false},
                        {"reason", "current_4d_infeasible"}}},
                {"minimum_component", idsJson(component)},
                {"minimum_component_solution", restrictedJson(
                    component_solution, frame.problem)},
                {"minimum_component_successor", successorJson(
                    component_successor)}}},
            {"preventive_homotopy", {{"segments", 8},
                {"candidate_count", 9},
                {"endpoint_semantics",
                 "coverage_projection_to_current_full_row_maximum_margin"},
                {"current_maximum_margin_endpoint", restrictedJson(
                    maximum_margin, frame.problem)},
                {"coverage_predicts_next_local_interval_empty",
                    coverage_predicts_empty},
                {"maximum_predicted_full_pair_margin_candidate_index",
                    predicted_argmax},
                {"maximum_predicted_component_full_margin_mps2",
                    number(predicted_best)},
                {"selection", selection}, {"selected_index", selected},
                {"selected_alpha", static_cast<double>(selected) / 8.0},
                {"selected_successor_feasible", selected_successor},
                {"candidates", std::move(candidates)}}}});
    }
    nlohmann::json earliest = earliest_prevention.has_value()
        ? nlohmann::json(*earliest_prevention) : nlohmann::json(nullptr);
    nlohmann::json latest = last_recovery.has_value()
        ? nlohmann::json(*last_recovery) : nlohmann::json(nullptr);
    return {{"protocol", "task10p11w-offline-oracle-v1"},
        {"checkpoint_count", checkpoint_paths.size()},
        {"frozen_pair", kPair},
        {"active_graph_numeric_boundary_mps2", kSolverFeasibilityTolerance},
        {"active_graph_boundary_source", "frozen_gurobi_feasibility_tolerance"},
        {"component_search_at_132p9", {{"attempts", search_attempts},
            {"pair_2_4_4d_sufficient", pair_4d_sufficient},
            {"minimum_component", idsJson(component)}}},
        {"frames", std::move(frames)},
        {"summary", {{"pair_2_4_4d_sufficient", pair_4d_sufficient},
            {"minimum_component", idsJson(component)},
            {"earliest_preventive_intervention_s", earliest},
            {"last_local_recovery_s", latest},
            {"pair_2_4_current_feasible_all_frames", every_pair_current},
            {"pair_2_4_successor_feasible_all_frames", every_pair_successor},
            {"fallback_restores_local_feasibility", every_fallback_restores_local},
            {"maximum_selected_coverage_control_deviation_l2_mps2",
                maximum_selected_deviation},
            {"selected_fallback_successor_feasible_all_frames",
                every_selected_successor}}},
        {"trajectory_run_performed", false},
        {"production_controller_modified", false},
        {"fixed_topology_modified", false},
        {"parameter_or_threshold_scan_performed", false},
        {"claim_boundary", {{"one_step_only", true},
            {"recursive_feasibility_claimed", false},
            {"task_11_entered", false}}}};
}

}  // namespace gf
