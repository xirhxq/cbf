#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11qStandardSafetyOn.hpp"
#include "grand_finale/Task10p11tOnlinePairResponsibility.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <vector>

namespace {

using json = nlohmann::json;

json edgeJson(const gf::DirectedEdge& edge) {
    return {{"reference", edge.reference}, {"owner", edge.owner},
            {"id", edge.id()}};
}

json vectorJson(const Eigen::Vector2d& value) {
    return json::array({value.x(), value.y()});
}

json targetLedgerJson(
    const std::map<gf::NodeId, Eigen::Vector2d>& values) {
    json result = json::object();
    for (const auto& [owner, value] : values)
        result[std::to_string(owner)] = vectorJson(value);
    return result;
}

struct TruthMargins {
    double mobile_mobile = std::numeric_limits<double>::infinity();
    double mobile_fixed = std::numeric_limits<double>::infinity();
    double speed_margin = std::numeric_limits<double>::infinity();
    double maximum_reference_distance = 0.0;
};

TruthMargins truthMargins(
    const gf::Task10p11rFixedBaselineFixture& fixture) {
    TruthMargins result;
    std::map<gf::NodeId, Eigen::Vector2d> positions;
    for (const auto& robot : fixture.swarm.robots) {
        const Point point = robot->model->xy();
        positions[robot->id] = {point.x, point.y};
        const Eigen::Vector2d velocity(
            robot->model->getStateVariable("vx"),
            robot->model->getStateVariable("vy"));
        result.speed_margin = std::min(
            result.speed_margin, 30.0 - velocity.norm());
    }
    for (std::size_t index = 0;
         index < fixture.scenario.mobile_ids.size(); ++index) {
        const auto owner = fixture.scenario.mobile_ids[index];
        for (std::size_t peer = index + 1;
             peer < fixture.scenario.mobile_ids.size(); ++peer) {
            result.mobile_mobile = std::min(
                result.mobile_mobile,
                (positions.at(owner) - positions.at(
                    fixture.scenario.mobile_ids[peer])).norm());
        }
        for (const auto& [fixed, position] :
             fixture.scenario.fixed_positions) {
            (void)fixed;
            result.mobile_fixed = std::min(
                result.mobile_fixed,
                (positions.at(owner) - position).norm());
        }
    }
    for (const auto& edge : fixture.frozen_topology) {
        const Eigen::Vector2d reference = edge.reference >= 100
            ? fixture.scenario.fixed_positions.at(edge.reference)
            : positions.at(edge.reference);
        result.maximum_reference_distance = std::max(
            result.maximum_reference_distance,
            (positions.at(edge.owner) - reference).norm());
    }
    return result;
}

bool hexDigest(const std::string& value, std::size_t length) {
    return value.size() == length && std::all_of(
        value.begin(), value.end(), [](unsigned char character) {
            return std::isxdigit(character) != 0;
        });
}

json run(
    const std::string& parent_commit,
    const std::string& parent_tree,
    const std::string& cbf_commit,
    const std::string& cbf_tree,
    const std::string& binary_sha256,
    const std::optional<std::filesystem::path>& checkpoint_directory) {
    constexpr std::size_t maximum_cycles = 5000;
    constexpr std::size_t coverage_stride_cycles = 10;
    constexpr std::size_t sparse_checkpoint_stride_cycles = 100;
    auto fixture = gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention, 14.0);
    const auto initialized = fixture->adapter.initializeStageZero();
    if (!initialized.initialized) {
        return {{"protocol", "task10p11t-online-pair-v1"},
                {"outcome", "initialization_failed"},
                {"reason", initialized.reason}};
    }
    if (fixture->adapter.config().range_noise_std_m!=0.0 ||
        fixture->adapter.config().range_dropout_probability!=0.0)
        throw std::runtime_error("restart_contract_requires_zero_range_rng");

    const auto initial_topology = fixture->adapter.runtimeSnapshot().topology;
    std::string topology_canonical;
    json topology_json = json::array();
    for (const auto& edge : gf::canonicalTargetEdges(initial_topology))
        topology_canonical += edge.id() + ";";
    for (const auto& edge : initial_topology)
        topology_json.push_back(edgeJson(edge));

    json coverage_progress = json::array();
    coverage_progress.push_back({
        {"time_s", 0.0},
        {"covered_cells", fixture->adapter.coverage().truthCoveredCount()},
        {"truth_coverage", fixture->adapter.coverage().truthFraction()}});
    json responsibility_trace = json::array();
    json conflict_events = json::array();
    json multi_pair_events = json::array();
    std::optional<std::string> active_pair;
    std::optional<double> first_intervention_time;
    std::optional<double> t95;
    std::optional<double> t100;
    std::string failure;
    double minimum_legacy_gamma = std::numeric_limits<double>::infinity();
    double minimum_hard_residual = std::numeric_limits<double>::infinity();
    double minimum_dynamic_residual = std::numeric_limits<double>::infinity();
    double minimum_once_reserve_full_pair =
        std::numeric_limits<double>::infinity();
    double minimum_collision_h = std::numeric_limits<double>::infinity();
    double minimum_collision_psi1 = std::numeric_limits<double>::infinity();
    double minimum_collision_residual =
        std::numeric_limits<double>::infinity();
    double minimum_reference_h = std::numeric_limits<double>::infinity();
    double minimum_reference_psi1 = std::numeric_limits<double>::infinity();
    double minimum_reference_residual =
        std::numeric_limits<double>::infinity();
    double minimum_information_fim = std::numeric_limits<double>::infinity();
    double maximum_posterior = 0.0;
    double minimum_aoi = std::numeric_limits<double>::infinity();
    std::size_t minimum_effective_references =
        std::numeric_limits<std::size_t>::max();
    double minimum_mobile_mobile = std::numeric_limits<double>::infinity();
    double minimum_mobile_fixed = std::numeric_limits<double>::infinity();
    double minimum_speed_margin = std::numeric_limits<double>::infinity();
    double maximum_reference_distance = 0.0;
    std::size_t responsibility_cycles = 0;
    std::size_t stagnant_cycles = 0;
    int previous_covered = fixture->adapter.coverage().truthCoveredCount();
    const auto wall_start = std::chrono::steady_clock::now();
    std::size_t checkpoint_index=0;
    std::size_t sparse_checkpoint_count=0;

    for (std::size_t cycle = 0; cycle < maximum_cycles; ++cycle) {
        if (!fixture->topologyFrozen()) {
            failure = "fixed_topology_mutated";
            break;
        }
        const auto runtime = fixture->adapter.runtimeSnapshot();
        const double decision_time = runtime.runtime_s;
        if (checkpoint_directory.has_value() &&
            cycle%sparse_checkpoint_stride_cycles==0) {
            auto sparse=gf::makeTask10p11vSparseRestartCheckpoint(
                *fixture,active_pair,cycle);
            sparse["source"]={{"parent_commit",parent_commit},
                {"parent_tree",parent_tree},{"cbf_commit",cbf_commit},
                {"cbf_tree",cbf_tree},{"binary_sha256",binary_sha256}};
            sparse["config_digest"]=
                gf::task10p11qConfigDigest(fixture->adapter.config());
            sparse["frozen_restart_contract"]={{"range_noise_std_m",0.0},
                {"range_dropout_probability",0.0},
                {"fixed_topology",true},{"selection_mode",
                    "least_intervention"},{"tau_mps2",14.0}};
            std::ostringstream name;
            name<<"sparse-"<<std::setw(4)<<std::setfill('0')<<cycle
                <<"-t"<<std::fixed<<std::setprecision(1)<<decision_time
                <<".json";
            gf::writeTask10p11vJson(
                *checkpoint_directory/name.str(),sparse);
            ++sparse_checkpoint_count;
        }
        if (!active_pair.has_value()) {
            const auto rows = fixture->adapter.currentSnapshotHardRows(
                runtime.topology);
            const auto diagnostic = gf::diagnoseTask10p11tOnlineConflicts(
                rows, runtime.estimate.mobile_ids, decision_time,
                runtime.mode, runtime.topology,
                fixture->adapter.config().acceleration_half_box);
            if (!diagnostic.valid) {
                failure = diagnostic.reason;
                break;
            }
            minimum_legacy_gamma = std::min(
                minimum_legacy_gamma,
                diagnostic.minimum_legacy_gamma_mps2);
            if (diagnostic.infeasible) {
                conflict_events.push_back({
                    {"time_s", decision_time},
                    {"infeasible_owners", diagnostic.infeasible_owners},
                    {"mobile_pair_base_ids",
                     diagnostic.mobile_pair_base_ids},
                    {"minimum_legacy_gamma_mps2",
                     diagnostic.minimum_legacy_gamma_mps2}});
                const auto selected =
                    gf::task10p11tUniqueOnlinePair(diagnostic);
                if (!selected.has_value()) {
                    failure = diagnostic.mobile_pair_base_ids.size() > 1
                        ? "multiple_mobile_pair_conflict"
                        : "dynamic_pair_conflict_not_unique";
                    if (diagnostic.mobile_pair_base_ids.size() > 1)
                        multi_pair_events.push_back(conflict_events.back());
                    break;
                }
                active_pair = *selected;
                first_intervention_time = decision_time;
            }
        }

        std::optional<gf::CanonicalHardRowRequest> checkpoint_request;
        std::optional<json> checkpoint_fields;
        if (active_pair.has_value() && checkpoint_directory.has_value()) {
            checkpoint_request=fixture->adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology);
            checkpoint_fields=gf::captureTask10p11vRestartFields(*fixture);
        }

        const auto control = active_pair.has_value()
            ? fixture->controller.advanceWithDynamicPairResponsibility(
                  *active_pair)
            : fixture->controller.advance();
        if (checkpoint_request.has_value() && checkpoint_fields.has_value()) {
            if (fixture->controller.lastNominalControls().size()!=14)
                throw std::runtime_error(
                    "checkpoint_nominal_controls_incomplete");
            const auto base=gf::makeTask10p11sSnapshot(runtime,
                *checkpoint_request,
                fixture->controller.lastNominalControls(),
                fixture->adapter.config());
            const std::string event=control.step.advanced
                ?(checkpoint_index==0?"first_dynamic_intervention":
                    "dynamic_intervention")
                :"fail_closed";
            auto checkpoint=gf::makeTask10p11vRestartCheckpoint(
                base,*checkpoint_fields,event);
            checkpoint["restart_checkpoint"]["source"]={
                {"parent_commit",parent_commit},{"parent_tree",parent_tree},
                {"cbf_commit",cbf_commit},{"cbf_tree",cbf_tree},
                {"binary_sha256",binary_sha256}};
            checkpoint["restart_checkpoint"]["config_digest"]=
                gf::task10p11qConfigDigest(fixture->adapter.config());
            std::ostringstream name;
            name<<"checkpoint-"<<std::setw(3)<<std::setfill('0')
                <<checkpoint_index<<"-t"<<std::fixed<<std::setprecision(1)
                <<decision_time<<"-"<<event<<".json";
            gf::writeTask10p11vJson(
                *checkpoint_directory/name.str(),checkpoint);
            ++checkpoint_index;
        }
        if (!control.step.advanced) {
            failure = control.reason.empty()
                ? control.step.reason : control.reason;
            if (active_pair.has_value()) {
                const auto failed_runtime = fixture->adapter.runtimeSnapshot();
                const auto failed_rows =
                    fixture->adapter.currentSnapshotHardRows(
                        failed_runtime.topology);
                const auto diagnostic = gf::diagnoseTask10p11tOnlineConflicts(
                    failed_rows, failed_runtime.estimate.mobile_ids,
                    failed_runtime.runtime_s, failed_runtime.mode,
                    failed_runtime.topology,
                    fixture->adapter.config().acceleration_half_box);
                if (diagnostic.valid) {
                    conflict_events.push_back({
                        {"time_s", failed_runtime.runtime_s},
                        {"infeasible_owners", diagnostic.infeasible_owners},
                        {"mobile_pair_base_ids",
                         diagnostic.mobile_pair_base_ids},
                        {"minimum_legacy_gamma_mps2",
                         diagnostic.minimum_legacy_gamma_mps2},
                        {"pair_step_reason", failure}});
                    if (diagnostic.mobile_pair_base_ids.size() > 1) {
                        multi_pair_events.push_back(conflict_events.back());
                        failure = "multiple_mobile_pair_conflict";
                    }
                }
            }
            break;
        }
        if (!fixture->topologyFrozen()) {
            failure = "fixed_topology_mutated";
            break;
        }

        minimum_hard_residual = std::min(
            minimum_hard_residual, control.step.minimum_hard_residual);
        minimum_collision_h = std::min(
            minimum_collision_h, control.step.minimum_collision_h);
        minimum_collision_psi1 = std::min(
            minimum_collision_psi1, control.step.minimum_collision_psi1);
        minimum_collision_residual = std::min(
            minimum_collision_residual,
            control.step.minimum_collision_residual);
        minimum_reference_h = std::min(
            minimum_reference_h, control.step.minimum_reference_h);
        minimum_reference_psi1 = std::min(
            minimum_reference_psi1, control.step.minimum_reference_psi1);
        minimum_reference_residual = std::min(
            minimum_reference_residual,
            control.step.minimum_reference_residual);
        if (control.step.dynamic_pair.applied) {
            ++responsibility_cycles;
            const auto& pair = control.step.dynamic_pair;
            minimum_dynamic_residual = std::min(
                minimum_dynamic_residual,
                pair.minimum_local_residual_mps2);
            minimum_once_reserve_full_pair = std::min(
                minimum_once_reserve_full_pair,
                pair.once_reserve_full_pair_residual_mps2);
            responsibility_trace.push_back({
                {"decision_time_s", decision_time},
                {"applied_time_s", fixture->swarm.robots.front()->runtime},
                {"pair_base_id", pair.pair_base_id},
                {"first_owner", pair.first_owner},
                {"second_owner", pair.second_owner},
                {"interval_lower_mps2",
                 pair.transfer_interval_lower_mps2},
                {"interval_upper_mps2",
                 pair.transfer_interval_upper_mps2},
                {"selected_transfer_mps2",
                 pair.selected_transfer_mps2},
                {"minimum_local_residual_mps2",
                 pair.minimum_local_residual_mps2},
                {"limiting_owner", pair.limiting_owner},
                {"limiting_row_id", pair.limiting_row_id},
                {"dynamic_pair_local_sum_residual_mps2",
                 pair.dynamic_pair_local_sum_residual_mps2},
                {"once_reserve_full_pair_residual_mps2",
                 pair.once_reserve_full_pair_residual_mps2},
                {"truth_coverage", control.step.truth_coverage}});
        }

        const auto information = fixture->adapter.currentReferenceAudit();
        minimum_information_fim = std::min(
            minimum_information_fim,
            information.minimum_robust_fim_cone_lower_bound);
        maximum_posterior = std::max(
            maximum_posterior, information.maximum_posterior_eigenvalue);
        minimum_aoi = std::min(
            minimum_aoi, information.minimum_range_aoi_margin_s);
        minimum_effective_references = std::min(
            minimum_effective_references,
            information.minimum_effective_reference_count);
        if (information.minimum_effective_reference_count < 2)
            failure = "information_effective_reference_failure";
        else if (information.minimum_robust_fim_cone_lower_bound < 1e-6)
            failure = "information_robust_fim_failure";
        else if (information.maximum_posterior_eigenvalue >
                 fixture->adapter.config().maximum_posterior_eigenvalue_m2)
            failure = "information_posterior_failure";
        else if (information.minimum_range_aoi_margin_s < 0.0)
            failure = "information_aoi_failure";

        const auto truth = truthMargins(*fixture);
        minimum_mobile_mobile = std::min(
            minimum_mobile_mobile, truth.mobile_mobile);
        minimum_mobile_fixed = std::min(
            minimum_mobile_fixed, truth.mobile_fixed);
        minimum_speed_margin = std::min(
            minimum_speed_margin, truth.speed_margin);
        maximum_reference_distance = std::max(
            maximum_reference_distance, truth.maximum_reference_distance);
        if (truth.mobile_mobile < 10.0 - 1e-9 ||
            truth.mobile_fixed < 10.0 - 1e-9)
            failure = "truth_collision";
        if (truth.speed_margin < -1e-9)
            failure = "plant_speed_violation";
        if (!failure.empty()) break;

        const double time_s = fixture->swarm.robots.front()->runtime;
        const double coverage = fixture->adapter.coverage().truthFraction();
        if (!t95.has_value() && coverage >= 0.95 - 1e-12) t95 = time_s;
        if (!t100.has_value() && coverage >= 1.0 - 1e-12) t100 = time_s;
        if ((cycle + 1) % coverage_stride_cycles == 0 ||
            t95.has_value() || t100.has_value()) {
            coverage_progress.push_back({
                {"time_s", time_s},
                {"covered_cells",
                 fixture->adapter.coverage().truthCoveredCount()},
                {"truth_coverage", coverage},
                {"dynamic_pair_active", active_pair.has_value()}});
        }
        if (t100.has_value()) break;
        const int covered = fixture->adapter.coverage().truthCoveredCount();
        stagnant_cycles = covered == previous_covered
            ? stagnant_cycles + 1 : 0;
        previous_covered = covered;
        if (stagnant_cycles >= 1000 && coverage < 1.0 - 1e-12) {
            failure = "coverage_stagnation_100s";
            break;
        }
    }

    const double wall_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wall_start).count();
    const double final_time = fixture->swarm.robots.front()->runtime;
    const double final_coverage = fixture->adapter.coverage().truthFraction();
    if (coverage_progress.empty() ||
        coverage_progress.back().at("time_s").get<double>() != final_time) {
        coverage_progress.push_back({
            {"time_s", final_time},
            {"covered_cells", fixture->adapter.coverage().truthCoveredCount()},
            {"truth_coverage", final_coverage},
            {"dynamic_pair_active", active_pair.has_value()}});
    }
    std::map<gf::NodeId, Eigen::Vector2d> targets;
    for (const auto& [owner, target] : fixture->controller.committedTargets())
        targets[owner] = target.center;
    const std::string final_reason = t100.has_value()
        ? "t100_reached"
        : failure.empty() ? "registered_500s_limit" : failure;
    const std::string outcome = t100.has_value()
        ? "t100" : failure.empty() ? "bounded_pass" : "failed";

    return {
        {"protocol", "task10p11t-online-pair-v1"},
        {"case", "fixed_stage_zero_dynamic_pair_unique_run"},
        {"unique_run_index", 1},
        {"source", {
            {"parent_commit", parent_commit},
            {"parent_tree", parent_tree},
            {"cbf_commit", cbf_commit},
            {"cbf_tree", cbf_tree},
            {"binary_sha256", binary_sha256}}},
        {"authority_commit",
         gf::task10p11rAuthorityContract().source_commit},
        {"legacy_selection_mode_before_intervention",
         "least_intervention"},
        {"predictive_tau_mps2", 14.0},
        {"dynamic_pair_policy",
         "unique_exact_conflict_pair_then_recompute_signed_transfer_each_cycle"},
        {"frozen_model_config_digest", gf::task10p11qConfigDigest(
            fixture->adapter.config())},
        {"registered_maximum_cycles", maximum_cycles},
        {"control_dt_s", fixture->adapter.config().dt_s},
        {"outcome", outcome},
        {"reason", final_reason},
        {"simulated_time_s", final_time},
        {"wall_time_s", wall_time},
        {"truth_coverage", final_coverage},
        {"covered_cells", fixture->adapter.coverage().truthCoveredCount()},
        {"denominator_cells", 90000},
        {"t95_true_s", t95.has_value() ? json(*t95) : json(nullptr)},
        {"t100_true_s", t100.has_value() ? json(*t100) : json(nullptr)},
        {"first_intervention_time_s", first_intervention_time.has_value()
            ? json(*first_intervention_time) : json(nullptr)},
        {"active_pair_base_id", active_pair.has_value()
            ? json(*active_pair) : json(nullptr)},
        {"responsibility_cycles", responsibility_cycles},
        {"responsibility_trace", responsibility_trace},
        {"conflict_events", conflict_events},
        {"multi_pair_conflict_events", multi_pair_events},
        {"minimum_legacy_gamma_before_intervention_mps2",
         minimum_legacy_gamma},
        {"minimum_dynamic_local_residual_mps2",
         responsibility_cycles > 0
            ? json(minimum_dynamic_residual) : json(nullptr)},
        {"minimum_once_reserve_full_pair_residual_mps2",
         responsibility_cycles > 0
            ? json(minimum_once_reserve_full_pair) : json(nullptr)},
        {"minimum_robust_hard_residual_mps2", minimum_hard_residual},
        {"minimum_collision_h_m", minimum_collision_h},
        {"minimum_collision_psi1_mps", minimum_collision_psi1},
        {"minimum_collision_residual_mps2", minimum_collision_residual},
        {"minimum_reference_h_m", minimum_reference_h},
        {"minimum_reference_psi1_mps", minimum_reference_psi1},
        {"minimum_reference_residual_mps2", minimum_reference_residual},
        {"minimum_accepted_information_robust_fim_cone_lower_bound",
         minimum_information_fim},
        {"maximum_posterior_eigenvalue_m2", maximum_posterior},
        {"minimum_aoi_margin_s", minimum_aoi},
        {"minimum_effective_reference_count", minimum_effective_references},
        {"minimum_truth_mobile_mobile_distance_m", minimum_mobile_mobile},
        {"minimum_truth_mobile_fixed_distance_m", minimum_mobile_fixed},
        {"minimum_truth_speed_margin_mps", minimum_speed_margin},
        {"maximum_truth_reference_distance_m", maximum_reference_distance},
        {"fixed_topology", topology_json},
        {"topology_canonical", topology_canonical},
        {"topology_frozen", fixture->topologyFrozen()},
        {"target_ledger", targetLedgerJson(targets)},
        {"target_epoch", fixture->controller.targetEpoch()},
        {"coverage_progress", coverage_progress},
        {"parameters_scanned", false},
        {"restart_checkpoint_capture_enabled",
         checkpoint_directory.has_value()},
        {"restart_checkpoint_count",checkpoint_index},
        {"sparse_restart_checkpoint_count",sparse_checkpoint_count},
        {"second_run_performed", false},
        {"task11_performed", false},
        {"recursive_feasibility_claimed", false}};
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7 && argc != 8) {
        std::cerr << "usage: GrandFinaleTask10p11tOnline OUTPUT_JSON "
                     "PARENT_COMMIT PARENT_TREE CBF_COMMIT CBF_TREE "
                     "BINARY_SHA256 [CHECKPOINT_DIRECTORY]\n";
        return 2;
    }
    if (!hexDigest(argv[2], 40) || !hexDigest(argv[3], 40) ||
        !hexDigest(argv[4], 40) || !hexDigest(argv[5], 40) ||
        !hexDigest(argv[6], 64)) {
        std::cerr << "provenance_preflight_failed\n";
        return 3;
    }
    try {
        const std::filesystem::path output_path = argv[1];
        const std::optional<std::filesystem::path> checkpoint_directory=
            argc==8?std::optional<std::filesystem::path>{argv[7]}:
                std::nullopt;
        const json output = run(argv[2], argv[3], argv[4], argv[5], argv[6],
            checkpoint_directory);
        gf::writeTask10p11vJson(output_path,output);
        std::cout << output.dump(2) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Task 10.11t online run failed: " << error.what() << '\n';
        return 4;
    }
}
