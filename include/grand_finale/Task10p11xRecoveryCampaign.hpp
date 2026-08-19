#pragma once

#include "grand_finale/Task10p11nGainSynthesis.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"
#include "grand_finale/Task10p11wConflictComponentOracle.hpp"

#include <filesystem>
#include <fstream>
#include <optional>

namespace gf {

// Immutable protocol choices fixed before any checkpoint continuation.
struct Task10p11xPreregisteredParameters {
    std::vector<double> predictive_tau_mps2{16.0, 18.0};
    std::optional<LinearHocbfGains> collision_gains =
        LinearHocbfGains{0.125, 0.5};
    DirectedEdge topology_addition{1, 4};
    DirectedEdge topology_removal{2, 4};
};

inline Task10p11xPreregisteredParameters task10p11xPreregisteredParameters() {
    return {};
}

namespace task10p11x_detail {

constexpr double kTolerance = 1.0e-8;
constexpr const char* kPair = "reference:2->4";

inline nlohmann::json number(double value) {
    return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
}

inline std::vector<DirectedEdge> replacedTopology(
    std::vector<DirectedEdge> edges, const DirectedEdge& addition,
    const DirectedEdge& removal, bool keep_old) {
    if (!keep_old) {
        edges.erase(std::remove_if(edges.begin(), edges.end(),
            [&](const auto& edge) { return edge.id() == removal.id(); }),
            edges.end());
    }
    if (std::none_of(edges.begin(), edges.end(),
            [&](const auto& edge) { return edge.id() == addition.id(); }))
        edges.push_back(addition);
    return edges;
}

inline CanonicalHardRowRequest requestAtEstimate(
    const nlohmann::json& snapshot, const JointEstimateSnapshot& estimate,
    const std::optional<LinearHocbfGains>& gains = std::nullopt,
    const std::optional<std::vector<DirectedEdge>>& topology = std::nullopt) {
    using namespace task10p11s_capture_detail;
    CanonicalHardRowRequest result = requestFromJson(
        snapshot.at("canonical_request"));
    if (estimate.mobile_ids != result.mobile_ids)
        throw std::invalid_argument("estimate owner identity mismatch");
    if (gains.has_value()) {
        result.collision_spec.lambda1 = gains->lambda1_per_s;
        result.collision_spec.lambda2 = gains->lambda2_per_s;
    }
    if (topology.has_value()) result.reference_edges = *topology;
    for (std::size_t index = 0; index < estimate.mobile_ids.size(); ++index) {
        const Eigen::Vector4d state = estimate.mean.segment<4>(4 * index);
        result.states.at(estimate.mobile_ids[index]) = {
            Point(state.x(), state.y()), state.tail<2>(), Eigen::Vector2d::Zero()};
    }
    const auto& parameters = snapshot.at("successor_parameters");
    const double sigma = parameters.at("uncertainty_sigma").get<double>();
    const double single_position_support =
        parameters.at("single_position_support_m").get<double>();
    const double single_velocity_support =
        parameters.at("single_velocity_support_mps").get<double>();
    const double relative_position_support =
        parameters.at("relative_position_support_m").get<double>();
    const double relative_velocity_support =
        parameters.at("relative_velocity_support_mps").get<double>();
    const auto pair_tube = [&](NodeId first, NodeId second) {
        const double fp = std::sqrt(std::max(
            0.0, detail::maximumPositionEigenvalue(estimate, first)));
        const double sp = std::sqrt(std::max(
            0.0, detail::maximumPositionEigenvalue(estimate, second)));
        const double fv = std::sqrt(std::max(
            0.0, detail::maximumVelocityEigenvalue(estimate, first)));
        const double sv = std::sqrt(std::max(
            0.0, detail::maximumVelocityEigenvalue(estimate, second)));
        return PairwiseSnapshotTube{
            sigma * (fp + sp) + relative_position_support,
            sigma * (fv + sv) + relative_velocity_support,
            SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    };
    result.reference_snapshot_tubes.clear();
    for (const auto& edge : result.reference_edges)
        result.reference_snapshot_tubes.emplace(
            edge.id(), pair_tube(edge.owner, edge.reference));
    result.collision_snapshot_tubes.clear();
    for (const auto& edge : result.collision_pairs)
        result.collision_snapshot_tubes.emplace(
            edge.id(), pair_tube(edge.first, edge.second));
    for (NodeId id : result.mobile_ids) {
        const double position_sigma = std::sqrt(std::max(
            0.0, detail::maximumPositionEigenvalue(estimate, id)));
        const double velocity_sigma = std::sqrt(std::max(
            0.0, detail::maximumVelocityEigenvalue(estimate, id)));
        if (!result.workspace_snapshot_tubes.empty())
            result.workspace_snapshot_tubes[id] = {
                sigma * position_sigma + single_position_support,
                sigma * velocity_sigma,
                SnapshotTubeProvenance::CovarianceSigmaDevelopment};
        if (!result.plant_speed_snapshot_tubes.empty())
            result.plant_speed_snapshot_tubes[id] = {0.0,
                sigma * velocity_sigma + single_velocity_support,
                SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    }
    result.plant_speed_dt_s =
        parameters.at("dt_s").get<double>();
    return result;
}

inline std::map<NodeId, Eigen::Vector2d> localProjection(
    const std::vector<CanonicalHardRow>& rows,
    const CanonicalHardRowRequest& request,
    const std::map<NodeId, Eigen::Vector2d>& nominal) {
    const auto pair = task10p11w_detail::selectedPair(rows, request.mobile_ids);
    std::map<NodeId, Eigen::Vector2d> controls;
    for (NodeId owner : request.mobile_ids) {
        const auto replay = task10p11t_detail::localReplay(
            rows, pair, owner, 0.0, request.acceleration_half_box,
            nominal.at(owner));
        if (!replay.feasible)
            throw std::runtime_error("local projection infeasible:" +
                                     std::to_string(owner));
        controls.emplace(owner, replay.control);
    }
    return controls;
}

inline std::map<NodeId, Eigen::Vector2d> localMaximumMargin(
    const std::vector<CanonicalHardRow>& rows,
    const CanonicalHardRowRequest& request) {
    std::map<NodeId, Eigen::Vector2d> controls;
    for (NodeId owner : request.mobile_ids) {
        const auto gamma = solveCanonicalGammaStar(
            rows, owner, request.acceleration_half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma) || gamma.gamma < -kTolerance)
            throw std::runtime_error("local maximum margin unavailable:" +
                                     std::to_string(owner));
        controls.emplace(owner, Eigen::Vector2d(gamma.accelX, gamma.accelY));
    }
    return controls;
}

inline double minimumLocalResidual(
    const std::vector<CanonicalHardRow>& rows,
    const std::map<NodeId, Eigen::Vector2d>& controls) {
    double value = std::numeric_limits<double>::infinity();
    for (const auto& row : rows)
        value = std::min(value, row.margin(controls.at(row.owner)));
    return value;
}

inline double minimumLocalGamma(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& owners, double half_box) {
    double value = std::numeric_limits<double>::infinity();
    for (NodeId owner : owners) {
        const auto gamma = solveCanonicalGammaStar(rows, owner, half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma))
            return -std::numeric_limits<double>::infinity();
        value = std::min(value, gamma.gamma);
    }
    return value;
}

struct SuccessorResult {
    bool local_interval = false;
    bool all_local_qps = false;
    bool component = false;
    bool full_pair = false;
    double local_margin = -std::numeric_limits<double>::infinity();
    double component_residual = -std::numeric_limits<double>::infinity();
    double full_residual = -std::numeric_limits<double>::infinity();
    Task10p11tDynamicPairResult pair;
    Task10p11sQpResult full;
};

inline SuccessorResult successor(
    const nlohmann::json& snapshot,
    const std::map<NodeId, Eigen::Vector2d>& controls,
    const std::optional<LinearHocbfGains>& gains = std::nullopt,
    const std::optional<std::vector<DirectedEdge>>& topology = std::nullopt) {
    SuccessorResult result;
    const auto current = task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    const auto& parameters = snapshot.at("successor_parameters");
    const auto predicted = predictNoMeasurementSnapshot(
        current, controls, parameters.at("dt_s").get<double>(),
        parameters.at("estimator_acceleration_variance").get<double>());
    const auto request = requestAtEstimate(snapshot, predicted, gains, topology);
    const auto rows = buildCanonicalHardRows(request);
    result.local_margin = minimumLocalGamma(
        rows, request.mobile_ids, request.acceleration_half_box);
    const auto pair = solveTask10p11tDynamicPair(
        rows, request.mobile_ids, controls,
        request.acceleration_half_box, kPair);
    result.pair = pair;
    result.local_interval = pair.shared_interval.feasible;
    try {
        const auto local = localProjection(rows, request, controls);
        result.all_local_qps = minimumLocalResidual(rows, local) >= -kTolerance;
    } catch (...) {
        result.all_local_qps = false;
    }
    const auto problem = buildTask10p11sRows28d(rows, request.mobile_ids, true);
    const auto ordered = task10p11sOrderedControls(request.mobile_ids, controls);
    const auto full = solveTask10p11sQp(problem, ordered);
    result.full = full;
    result.full_pair = full.feasible && full.minimum_residual >= -kTolerance;
    result.full_residual = full.minimum_residual;
    try {
        const auto frozen = task10p11w_detail::frozenLocalControls(
            rows, request, controls);
        const auto component = task10p11w_detail::solveRestricted(
            problem, ordered, frozen, {2, 4}, false);
        result.component = component.feasible &&
            component.minimum_residual >= -kTolerance;
        result.component_residual = component.minimum_residual;
    } catch (...) {
        result.component = false;
    }
    return result;
}

inline nlohmann::json successorJson(const SuccessorResult& value) {
    return {{"local_signed_transfer_feasible", value.local_interval},
        {"all_local_qps_feasible", value.all_local_qps},
        {"minimum_local_gamma_mps2", number(value.local_margin)},
        {"component_4d_feasible", value.component},
        {"component_minimum_full_row_residual_mps2",
            number(value.component_residual)},
        {"full_pair_28d_feasible", value.full_pair},
        {"full_pair_minimum_residual_mps2", number(value.full_residual)},
        {"exact_zoh_one_step", true},
        {"recursive_feasibility_claimed", false}};
}

inline nlohmann::json distributedPreventiveFrame(
    const nlohmann::json& snapshot) {
    const auto request = task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto rows = buildCanonicalHardRows(request);
    const auto nominal = task10p11s_capture_detail::nominalFromJson(
        snapshot.at("nominal_controls"));
    std::map<NodeId, Eigen::Vector2d> coverage;
    std::map<NodeId, Eigen::Vector2d> maximum;
    try {
        coverage = localProjection(rows, request, nominal);
        maximum = localMaximumMargin(rows, request);
    } catch (const std::exception& error) {
        return {{"valid", false}, {"reason", error.what()},
            {"triggered", true}, {"selection", "current_local_infeasible"},
            {"offline_gate_passed", false},
            {"candidates", nlohmann::json::array()}};
    }
    nlohmann::json candidates = nlohmann::json::array();
    std::vector<SuccessorResult> audits;
    std::vector<std::map<NodeId, Eigen::Vector2d>> commands;
    for (std::size_t index = 0; index <= 8; ++index) {
        const double alpha = static_cast<double>(index) / 8.0;
        std::map<NodeId, Eigen::Vector2d> command;
        for (NodeId owner : request.mobile_ids)
            command[owner] = (1.0 - alpha) * coverage.at(owner) +
                             alpha * maximum.at(owner);
        const auto audit = successor(snapshot, command);
        const auto problem = buildTask10p11sRows28d(
            rows, request.mobile_ids, true);
        const auto ordered = task10p11sOrderedControls(
            request.mobile_ids, command);
        const auto residual = task10p11w_detail::minimumResidual(problem, ordered);
        candidates.push_back({{"index", index}, {"alpha", alpha},
            {"current_local_residual_mps2", minimumLocalResidual(rows, command)},
            {"current_full_row_residual_mps2", residual.first},
            {"coverage_control_deviation_l2_mps2",
                (ordered - task10p11sOrderedControls(
                    request.mobile_ids, coverage)).norm()},
            {"successor", successorJson(audit)}});
        commands.push_back(command);
        audits.push_back(audit);
    }
    const bool trigger = !audits.front().local_interval ||
                         !audits.front().all_local_qps;
    std::size_t selected = 0;
    std::string selection = "coverage_first";
    if (trigger) {
        std::optional<std::size_t> restoring;
        for (std::size_t index = 0; index < audits.size(); ++index)
            if (audits[index].local_interval && audits[index].all_local_qps &&
                audits[index].full_pair) {
                restoring = index;
                break;
            }
        if (restoring.has_value()) {
            selected = *restoring;
            selection = "least_coverage_deviation_restoring_local";
        } else {
            selection = "maximum_predicted_local_margin";
            for (std::size_t index = 1; index < audits.size(); ++index)
                if (audits[index].local_margin > audits[selected].local_margin)
                    selected = index;
        }
    }
    candidates[selected]["selected"] = true;
    return {{"valid", true}, {"reason", "evaluated"},
        {"triggered", trigger}, {"selection", selection},
        {"selected_index", selected},
        {"selected_successor", successorJson(audits[selected])},
        {"offline_gate_passed", audits[selected].local_interval &&
            audits[selected].all_local_qps && audits[selected].full_pair},
        {"candidates", std::move(candidates)}};
}

inline nlohmann::json parameterFrame(
    const nlohmann::json& snapshot, double tau,
    const std::optional<LinearHocbfGains>& gains = std::nullopt) {
    const auto estimate = task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    const auto nominal = task10p11s_capture_detail::nominalFromJson(
        snapshot.at("nominal_controls"));
    const auto& parameters = snapshot.at("successor_parameters");
    CanonicalGammaFeedbackEvaluationContext context;
    const CanonicalGammaFeedbackConfig config{
        snapshot.at("canonical_request").at("acceleration_half_box").get<double>(),
        8, GammaFeedbackSelectionMode::LeastIntervention, tau, 1.0e-10};
    const auto selected = evaluateCanonicalGammaFeedbackBatchReference(
        estimate, nominal, config,
        parameters.at("dt_s").get<double>(),
        parameters.at("estimator_acceleration_variance").get<double>(),
        [&](const JointEstimateSnapshot& value) {
            return requestAtEstimate(snapshot, value, gains);
        }, context);
    if (!selected.valid)
        return {{"valid", false}, {"reason", selected.reason},
            {"offline_gate_passed", false}};
    const auto audit = successor(snapshot, selected.selected_controls, gains);
    std::size_t interventions = 0;
    for (const auto& [owner, value] : selected.selections) {
        (void)owner;
        if (value.intervened) ++interventions;
    }
    return {{"valid", true}, {"reason", selected.reason},
        {"tau_mps2", tau}, {"intervened_owner_count", interventions},
        {"successor", successorJson(audit)},
        {"offline_gate_passed", audit.local_interval &&
            audit.all_local_qps && audit.full_pair}};
}

struct ComponentFallbackDecision {
    bool valid = false;
    bool use_component = false;
    bool local_current_feasible = false;
    bool local_successor_feasible = false;
    std::string reason;
    std::map<NodeId,Eigen::Vector2d> controls;
    double current_full_row_residual_mps2 =
        -std::numeric_limits<double>::infinity();
    SuccessorResult successor_audit;
};

inline std::optional<std::map<NodeId,Eigen::Vector2d>>
selectedDistributedControls(const nlohmann::json& snapshot, double tau) {
    const auto estimate = task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    const auto nominal = task10p11s_capture_detail::nominalFromJson(
        snapshot.at("nominal_controls"));
    const auto& parameters = snapshot.at("successor_parameters");
    CanonicalGammaFeedbackEvaluationContext context;
    const CanonicalGammaFeedbackConfig config{
        snapshot.at("canonical_request").at("acceleration_half_box").get<double>(),
        8, GammaFeedbackSelectionMode::LeastIntervention, tau, 1.0e-10};
    const auto selected = evaluateCanonicalGammaFeedbackBatchReference(
        estimate, nominal, config,
        parameters.at("dt_s").get<double>(),
        parameters.at("estimator_acceleration_variance").get<double>(),
        [&](const JointEstimateSnapshot& value) {
            return requestAtEstimate(snapshot, value);
        }, context);
    if (!selected.valid) return std::nullopt;
    const auto request = task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    try {
        return localProjection(buildCanonicalHardRows(request), request,
            selected.selected_controls);
    } catch (...) {
        return std::nullopt;
    }
}

inline ComponentFallbackDecision decidePairComponentFallback(
    const nlohmann::json& snapshot, bool component_was_active) {
    ComponentFallbackDecision result;
    const auto request = task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto rows = buildCanonicalHardRows(request);
    const auto nominal = task10p11s_capture_detail::nominalFromJson(
        snapshot.at("nominal_controls"));
    const auto distributed = selectedDistributedControls(snapshot, 14.0);
    if (distributed.has_value()) {
        result.local_current_feasible = true;
        const auto local_next = successor(snapshot, *distributed);
        result.local_successor_feasible = local_next.local_interval &&
            local_next.all_local_qps && local_next.full_pair;
        if (!component_was_active || result.local_successor_feasible) {
            if (result.local_successor_feasible) {
                result.valid = true;
                result.use_component = false;
                result.reason = component_was_active
                    ? "local_current_and_successor_recovered"
                    : "distributed_local_successor_feasible";
                result.controls = *distributed;
                result.successor_audit = local_next;
                const auto problem = buildTask10p11sRows28d(
                    rows, request.mobile_ids, true);
                result.current_full_row_residual_mps2 =
                    task10p11w_detail::minimumResidual(problem,
                        task10p11sOrderedControls(
                            request.mobile_ids, result.controls)).first;
                return result;
            }
        }
    }
    Eigen::VectorXd frozen;
    try {
        frozen = task10p11w_detail::frozenLocalControls(
            rows, request, nominal);
    } catch (const std::exception& error) {
        result.reason = std::string("outside_local_projection_failed:") +
            error.what();
        return result;
    }
    const auto problem = buildTask10p11sRows28d(
        rows, request.mobile_ids, true);
    const auto ordered_nominal = task10p11sOrderedControls(
        request.mobile_ids, nominal);
    const auto component = task10p11w_detail::solveRestricted(
        problem, ordered_nominal, frozen, {2, 4}, false);
    if (!component.feasible ||
        component.minimum_residual < -kTolerance) {
        result.reason = "pair_2_4_component_current_infeasible";
        return result;
    }
    result.controls = task10p11sControlMap(
        request.mobile_ids, component.controls);
    result.successor_audit = successor(snapshot, result.controls);
    result.current_full_row_residual_mps2 = component.minimum_residual;
    result.use_component = true;
    result.valid = result.successor_audit.component &&
        result.successor_audit.full_pair;
    result.reason = result.valid
        ? "pair_2_4_component_selected"
        : "pair_2_4_component_successor_infeasible";
    return result;
}

inline nlohmann::json componentFallbackDecisionJson(
    const ComponentFallbackDecision& value) {
    return {{"valid", value.valid}, {"use_component", value.use_component},
        {"local_current_feasible", value.local_current_feasible},
        {"local_successor_feasible", value.local_successor_feasible},
        {"reason", value.reason},
        {"current_full_row_residual_mps2",
            number(value.current_full_row_residual_mps2)},
        {"successor", successorJson(value.successor_audit)}};
}

inline nlohmann::json topologyFrame(
    const std::filesystem::path& checkpoint,
    const Task10p11xPreregisteredParameters& protocol) {
    const auto snapshot = readTask10p11vJson(checkpoint);
    auto fixture = makeTask10p11rFixedBaselineFixture(
        GammaFeedbackSelectionMode::LeastIntervention, 14.0);
    if (!fixture->adapter.initializeStageZero().initialized)
        throw std::runtime_error("topology fixture stage zero failed");
    restoreTask10p11vRestartState(*fixture, snapshot.at("restart_checkpoint"));
    const auto certificate = fixture->adapter.auditReplacement(
        protocol.topology_addition, protocol.topology_removal);
    const auto old = task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request")).reference_edges;
    const auto union_edges = replacedTopology(old, protocol.topology_addition,
        protocol.topology_removal, true);
    const auto successor_edges = replacedTopology(old, protocol.topology_addition,
        protocol.topology_removal, false);
    const auto nominal = task10p11s_capture_detail::nominalFromJson(
        snapshot.at("nominal_controls"));
    const auto evaluate = [&](const std::vector<DirectedEdge>& edges) {
        const auto estimate = task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        const auto request = requestAtEstimate(snapshot, estimate,
            std::nullopt, edges);
        const auto rows = buildCanonicalHardRows(request);
        const auto problem = buildTask10p11sRows28d(
            rows, request.mobile_ids, true);
        const auto ordered = task10p11sOrderedControls(
            request.mobile_ids, nominal);
        const auto current = solveTask10p11sQp(problem, ordered);
        const auto next = current.feasible
            ? successor(snapshot,
                task10p11sControlMap(request.mobile_ids, current.controls),
                std::nullopt, edges)
            : SuccessorResult{};
        return nlohmann::json{{"current_full_pair_feasible", current.feasible},
            {"current_minimum_residual_mps2", number(current.minimum_residual)},
            {"successor", successorJson(next)}};
    };
    const auto union_audit = evaluate(union_edges);
    const auto successor_audit = evaluate(successor_edges);
    const bool gate = certificate.valid &&
        union_audit.at("current_full_pair_feasible").get<bool>() &&
        union_audit.at("successor").at("full_pair_28d_feasible").get<bool>() &&
        successor_audit.at("current_full_pair_feasible").get<bool>() &&
        successor_audit.at("successor").at("full_pair_28d_feasible").get<bool>();
    return {{"addition", protocol.topology_addition.id()},
        {"removal", protocol.topology_removal.id()},
        {"certificate", {{"valid", certificate.valid},
            {"reason", certificate.reason},
            {"forward_valid", certificate.forward_valid},
            {"old_gamma_mps2", number(certificate.old_state.minimum_gamma)},
            {"union_gamma_mps2", number(certificate.union_state.minimum_gamma)},
            {"successor_gamma_mps2",
                number(certificate.successor_state.minimum_gamma)}}},
        {"union", union_audit}, {"successor_topology", successor_audit},
        {"information_fim_reference_tube_epoch_gate", certificate.valid},
        {"offline_gate_passed", gate}};
}

}  // namespace task10p11x_detail

inline nlohmann::json runTask10p11xStageAPreregistration(
    const std::vector<std::filesystem::path>& checkpoint_paths) {
    using namespace task10p11x_detail;
    if (checkpoint_paths.size() != 6)
        throw std::invalid_argument("Stage A requires exactly six checkpoints");
    std::vector<nlohmann::json> snapshots;
    std::optional<std::uint64_t> digest;
    for (std::size_t index = 0; index < checkpoint_paths.size(); ++index) {
        const auto snapshot = readTask10p11vJson(checkpoint_paths[index]);
        const auto validation = validateTask10p11sSnapshot(snapshot);
        if (!validation.complete ||
            std::abs(snapshot.at("runtime").at("runtime_s").get<double>() -
                (132.4 + 0.1 * static_cast<double>(index))) > 1.0e-8)
            throw std::runtime_error("checkpoint identity/order failed");
        const auto current_digest = snapshot.at("restart_checkpoint")
            .at("config_digest").get<std::uint64_t>();
        if (digest.has_value() && *digest != current_digest)
            throw std::runtime_error("checkpoint config digest mismatch");
        digest = current_digest;
        snapshots.push_back(snapshot);
    }
    const auto protocol = task10p11xPreregisteredParameters();
    nlohmann::json distributed = nlohmann::json::array();
    nlohmann::json tau_routes = nlohmann::json::array();
    nlohmann::json gain = nlohmann::json::array();
    for (const auto& snapshot : snapshots) {
        distributed.push_back(distributedPreventiveFrame(snapshot));
        nlohmann::json tau_frame = nlohmann::json::array();
        for (double tau : protocol.predictive_tau_mps2)
            tau_frame.push_back(parameterFrame(snapshot, tau));
        tau_routes.push_back(std::move(tau_frame));
        gain.push_back(parameterFrame(snapshot, 14.0, protocol.collision_gains));
    }
    const auto component = runTask10p11wOfflineOracle(checkpoint_paths);
    const auto topology = topologyFrame(checkpoint_paths.at(4), protocol);
    nlohmann::json tau_summary = nlohmann::json::array();
    for (std::size_t candidate = 0;
         candidate < protocol.predictive_tau_mps2.size(); ++candidate)
        tau_summary.push_back({{"tau_mps2",
                protocol.predictive_tau_mps2[candidate]},
            {"offline_gate_passed",
                tau_routes.at(4).at(candidate).at("offline_gate_passed")}});
    const bool distributed_gate =
        distributed.at(4).at("offline_gate_passed").get<bool>();
    const bool component_gate = component.at("frames").at(4)
        .at("component_search").at("minimum_component_successor")
        .at("full_pair_feasibility").at("feasible").get<bool>();
    const bool gain_gate = gain.at(4).at("offline_gate_passed").get<bool>();
    return {{"protocol", "fixed-baseline-multi-path-recovery-stage-a-v1"},
        {"checkpoint_count", checkpoint_paths.size()},
        {"config_digest", *digest},
        {"trajectory_run_performed", false},
        {"frozen", {{"dt_s", 0.1}, {"collision_distance_m", 10.0},
            {"reference_keep_m", 850.0}, {"reference_add_m", 849.0},
            {"acceleration_half_box_mps2", 4.0},
            {"plant_speed_limit_mps", 30.0}, {"stage_zero", true},
            {"hard_gates", true}}},
        {"routes", {
            {"distributed_preventive", {{"frames", distributed},
                {"offline_gate_passed", distributed_gate}}},
            {"pair_2_4_component", {{"oracle", component},
                {"offline_gate_passed", component_gate}}},
            {"parameter", {{"tau_frames", tau_routes},
                {"tau_candidates", tau_summary}, {"gain_frames", gain},
                {"gain_candidate", {{"lambda1_per_s", 0.125},
                    {"lambda2_per_s", 0.5},
                    {"offline_gate_passed", gain_gate}}},
                {"gamma_star_is_diagnostic_not_parameter", true}}},
            {"dynamic_topology", topology}}},
        {"stage_b_order", {"distributed_preventive",
            "pair_2_4_component", "tau_16", "tau_18",
            "gain_0.125_0.5", "dynamic_topology_1_to_4_replace_2_to_4"}},
        {"claim_boundary", {{"one_step_only", true},
            {"recursive_feasibility_claimed", false},
            {"task_11_entered", false}, {"parameter_scan_performed", false}}}};
}

}  // namespace gf
