#pragma once

#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

inline nlohmann::json runTask10p11uFrozenFailureAudit(
    const nlohmann::json& frozen) {
    constexpr double expected_time_s = 132.89999999999674;
    constexpr double tolerance_s = 1e-9;
    const std::string expected_reason =
        "pair_coordination_failed:dynamic_pair_responsibility_interval_empty";
    const std::string expected_pair = "reference:2->4";
    if (!frozen.is_object() ||
        frozen.value("protocol", std::string{}) !=
            "task10p11t-online-pair-v1" ||
        frozen.value("outcome", std::string{}) != "failed" ||
        frozen.value("reason", std::string{}) != expected_reason ||
        !frozen.contains("simulated_time_s") ||
        !frozen.at("simulated_time_s").is_number() ||
        std::abs(frozen.at("simulated_time_s").get<double>() -
                 expected_time_s) > tolerance_s ||
        frozen.value("active_pair_base_id", std::string{}) != expected_pair ||
        !frozen.value("topology_frozen", false)) {
        throw std::invalid_argument(
            "input is not the frozen Task 10.11t 132.9 s failure summary");
    }
    if (!frozen.contains("source") || !frozen.at("source").is_object() ||
        !frozen.contains("conflict_events") ||
        !frozen.at("conflict_events").is_array() ||
        frozen.at("conflict_events").empty() ||
        !frozen.contains("responsibility_trace") ||
        !frozen.at("responsibility_trace").is_array() ||
        frozen.at("responsibility_trace").empty()) {
        throw std::invalid_argument(
            "frozen Task 10.11t failure summary is structurally incomplete");
    }
    const auto& terminal_conflict = frozen.at("conflict_events").back();
    if (terminal_conflict.value("pair_step_reason", std::string{}) !=
            expected_reason ||
        !terminal_conflict.contains("time_s") ||
        std::abs(terminal_conflict.at("time_s").get<double>() -
                 expected_time_s) > tolerance_s ||
        terminal_conflict.value(
            "mobile_pair_base_ids", std::vector<std::string>{}) !=
            std::vector<std::string>{expected_pair}) {
        throw std::invalid_argument(
            "terminal conflict does not certify the signed-transfer stop");
    }

    const std::vector<std::string> required_snapshot_fields{
        "runtime", "estimator", "dekf_internal", "canonical_request",
        "actual_rows", "owner_row_counts", "nominal_controls",
        "objective_28d", "successor_parameters", "preflight"};
    std::vector<std::string> missing;
    for (const std::string& field : required_snapshot_fields) {
        if (!frozen.contains(field)) missing.push_back(field);
    }
    if (missing.empty()) {
        throw std::invalid_argument(
            "Task 10.11u received an unexpected complete snapshot; "
            "use the registered full-28D oracle rather than this summary stop");
    }

    const nlohmann::json undetermined_current = {
        {"performed", false},
        {"status", "undetermined"},
        {"reason", "complete_14_owner_rows_and_objective_missing"}};
    return {
        {"protocol", "task10p11u-frozen-132p9-evidence-gate-v1"},
        {"input_summary_valid", true},
        {"frozen_time_s", expected_time_s},
        {"pair_base_id", expected_pair},
        {"local_signed_transfer", {
            {"determined", true},
            {"feasible", false},
            {"status", "infeasible"},
            {"reason", "dynamic_pair_responsibility_interval_empty"},
            {"evidence", "online_same_snapshot_pair_solver"}}},
        {"snapshot_preflight", {
            {"complete", false},
            {"required_fields", required_snapshot_fields},
            {"missing_fields", missing},
            {"owner_states_available", false},
            {"actual_rows_available", false},
            {"nominal_objective_available", false},
            {"successor_reconstruction_available", false}}},
        {"current_full_pair_28d", undetermined_current},
        {"independent_full_row_residual_recompute", {
            {"performed", false},
            {"status", "not_authorized_by_incomplete_snapshot"},
            {"reason", "rows_and_28d_control_witness_missing"}}},
        {"successor", {
            {"performed", false},
            {"status", "undetermined"},
            {"reason", "current_full_pair_feasibility_not_established"}}},
        {"pair_2_4_component_4d", {
            {"performed", false},
            {"status", "undetermined"},
            {"equivalent_reduction_established", false},
            {"reason",
             "actual_rows_external_controls_and_nominal_objective_missing"}}},
        {"reason", "frozen_132p9_snapshot_incomplete_for_full_28d"},
        {"claim_boundary",
         "local_signed_transfer_infeasible_full_pair_and_successor_undetermined"},
        {"trajectory_run_performed", false},
        {"production_controller_modified", false},
        {"task11_performed", false}};
}

}  // namespace gf
