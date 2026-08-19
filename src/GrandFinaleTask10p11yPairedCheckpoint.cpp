#include "grand_finale/Task10p11qStandardSafetyOn.hpp"
#include "grand_finale/Task10p11tOnlinePairResponsibility.hpp"
#include "grand_finale/Task10p11yEarlyPrevention.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>

namespace {

using json=nlohmann::json;

constexpr double kEndpointS=133.0;
constexpr std::size_t kMaximumCycles=30;
constexpr double kTolerance=1.0e-8;

struct TruthMargins {
    double mobile_mobile=std::numeric_limits<double>::infinity();
    double mobile_fixed=std::numeric_limits<double>::infinity();
    double speed=std::numeric_limits<double>::infinity();
};

TruthMargins truthMargins(const gf::Task10p11rFixedBaselineFixture& fixture) {
    TruthMargins result;
    std::map<gf::NodeId,Eigen::Vector2d> positions;
    for (const auto& robot:fixture.swarm.robots) {
        const Point point=robot->model->xy();
        positions[robot->id]={point.x,point.y};
        result.speed=std::min(result.speed,30.0-Eigen::Vector2d(
            robot->model->getStateVariable("vx"),
            robot->model->getStateVariable("vy")).norm());
    }
    for (std::size_t index=0;index<fixture.scenario.mobile_ids.size();++index) {
        const auto owner=fixture.scenario.mobile_ids[index];
        for (std::size_t peer=index+1;
             peer<fixture.scenario.mobile_ids.size();++peer)
            result.mobile_mobile=std::min(result.mobile_mobile,
                (positions.at(owner)-positions.at(
                    fixture.scenario.mobile_ids[peer])).norm());
        for (const auto& [fixed,position]:fixture.scenario.fixed_positions) {
            (void)fixed;
            result.mobile_fixed=std::min(result.mobile_fixed,
                (positions.at(owner)-position).norm());
        }
    }
    return result;
}

std::string checkpointName(std::size_t index,double time,
    const std::string& event) {
    std::ostringstream name;
    name<<"checkpoint-"<<std::setw(3)<<std::setfill('0')<<index
        <<"-t"<<std::fixed<<std::setprecision(1)<<time<<"-"<<event<<".json";
    return name.str();
}

json controlsJson(const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    json result=json::object();
    for (const auto& [owner,control]:controls)
        result[std::to_string(owner)]={control.x(),control.y()};
    return result;
}

double controlDifference(
    const std::map<gf::NodeId,Eigen::Vector2d>& first,
    const std::map<gf::NodeId,Eigen::Vector2d>& second) {
    double squared=0.0;
    for (const auto& [owner,control]:first)
        squared+=(control-second.at(owner)).squaredNorm();
    return std::sqrt(squared);
}

json hardGateJson(gf::Task10p11rFixedBaselineFixture& fixture,
    const TruthMargins& truth) {
    const auto information=fixture.adapter.currentReferenceAudit();
    return {{"fixed_topology",fixture.topologyFrozen()},
        {"minimum_effective_reference_count",
            information.minimum_effective_reference_count},
        {"minimum_robust_fim",information.minimum_robust_fim_cone_lower_bound},
        {"maximum_posterior_eigenvalue_m2",
            information.maximum_posterior_eigenvalue},
        {"minimum_aoi_margin_s",information.minimum_range_aoi_margin_s},
        {"minimum_mobile_mobile_distance_m",truth.mobile_mobile},
        {"minimum_mobile_fixed_distance_m",truth.mobile_fixed},
        {"minimum_speed_margin_mps",truth.speed}};
}

std::string failedPostAdvanceGate(
    gf::Task10p11rFixedBaselineFixture& fixture,
    const TruthMargins& truth) {
    if (!fixture.topologyFrozen()) return "fixed_topology_mutated";
    const auto information=fixture.adapter.currentReferenceAudit();
    if (information.minimum_effective_reference_count<2)
        return "information_effective_reference_failure";
    if (information.minimum_robust_fim_cone_lower_bound<1.0e-6)
        return "information_robust_fim_failure";
    if (information.maximum_posterior_eigenvalue>
        fixture.adapter.config().maximum_posterior_eigenvalue_m2)
        return "information_posterior_failure";
    if (information.minimum_range_aoi_margin_s<0.0)
        return "information_aoi_failure";
    if (truth.mobile_mobile<10.0-1.0e-9 ||
        truth.mobile_fixed<10.0-1.0e-9)
        return "truth_collision";
    if (truth.speed<-1.0e-9) return "plant_speed_violation";
    return {};
}

json metric(double value,const std::string& reason="observed") {
    return gf::task10p11yMetric(std::isfinite(value)
        ?std::optional<double>(value):std::nullopt,
        std::isfinite(value)?reason:"not_applicable");
}

struct RunState {
    std::optional<std::string> active_pair;
    std::optional<double> first_prediction;
    std::optional<double> first_intervention;
    gf::Task10p11yCycleStatistics statistics;
    std::size_t checkpoint_index=0;
    std::size_t pair_cycles=0;
    std::string stop_reason;
    json trace=json::array();
};

void decorateCheckpoint(json& checkpoint,const std::string& branch,
    const json& source,const json& decision) {
    checkpoint["task10p11y"]={{"protocol",
        "task10p11y-early-entry-paired-checkpoint-v1"},
        {"branch",branch},{"source",source},{"decision",decision}};
}

json run(const std::string& branch,const std::filesystem::path& restore_path,
    const std::filesystem::path& checkpoint_directory,const json& source) {
    if (branch!="C0" && branch!="T1")
        throw std::invalid_argument("branch must be C0 or T1");
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized)
        throw std::runtime_error("stage_zero_initialization_failed:"+
            initialized.reason);
    const auto sparse=gf::readTask10p11vJson(restore_path);
    const auto restored=gf::restoreTask10p11vSparseRestartCheckpoint(
        *fixture,sparse);
    if (restored.cycle!=1300 || restored.active_pair.has_value() ||
        sparse.at("config_digest").get<std::uint64_t>()!=
            gf::task10p11qConfigDigest(fixture->adapter.config()) ||
        fixture->adapter.config().range_noise_std_m!=0.0 ||
        fixture->adapter.config().range_dropout_probability!=0.0 ||
        !fixture->topologyFrozen() ||
        std::abs(fixture->swarm.robots.front()->runtime-130.0)>1.0e-9)
        throw std::runtime_error("restore_or_frozen_identity_mismatch");
    std::filesystem::create_directories(checkpoint_directory);
    RunState state;
    const double initial_coverage=fixture->adapter.coverage().truthFraction();
    const int initial_cells=fixture->adapter.coverage().truthCoveredCount();
    const auto wall_start=std::chrono::steady_clock::now();

    for (std::size_t cycle=0;cycle<kMaximumCycles;++cycle) {
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const double decision_time=runtime.runtime_s;
        if (decision_time>=kEndpointS-1.0e-9) {
            state.stop_reason="fixed_endpoint_133p0";
            break;
        }
        if (!fixture->topologyFrozen()) {
            state.stop_reason="fixed_topology_mutated";
            break;
        }
        const auto request=fixture->adapter.snapshotHardRowRequest(
            runtime.estimate,runtime.topology);
        const auto restart_fields=gf::captureTask10p11vRestartFields(*fixture);
        const auto rows=gf::buildCanonicalHardRows(request);
        std::optional<gf::task10p11y_detail::Decision> t1_decision;
        bool dynamic_used=false;
        bool preventive_intervention=false;
        double selected_transfer=std::numeric_limits<double>::quiet_NaN();
        double preventive_deviation=0.0;

        gf::SimpleCoverageControlStep control;
        if (branch=="C0") {
            if (!state.active_pair.has_value()) {
                const auto diagnostic=gf::diagnoseTask10p11tOnlineConflicts(
                    rows,runtime.estimate.mobile_ids,decision_time,
                    runtime.mode,runtime.topology,
                    fixture->adapter.config().acceleration_half_box);
                if (!diagnostic.valid) {
                    state.stop_reason=diagnostic.reason;
                    break;
                }
                if (diagnostic.infeasible) {
                    const auto selected=gf::task10p11tUniqueOnlinePair(
                        diagnostic);
                    if (!selected.has_value()) {
                        state.stop_reason=diagnostic.mobile_pair_base_ids.size()>1
                            ?"multiple_mobile_pair_conflict":
                            "dynamic_pair_conflict_not_unique";
                        break;
                    }
                    state.active_pair=*selected;
                }
            }
            dynamic_used=state.active_pair.has_value();
            control=dynamic_used
                ?fixture->controller.advanceWithDynamicPairResponsibility(
                    *state.active_pair)
                :fixture->controller.advance();
            if (control.step.dynamic_pair.applied)
                selected_transfer=
                    control.step.dynamic_pair.selected_transfer_mps2;
        } else {
            control=fixture->controller.advanceWithDevelopmentControlOverride(
                [&](const gf::GrandFinaleRuntimeSnapshot& callback_runtime,
                    const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
                    const std::map<gf::NodeId,double>& yaw_rates) {
                    auto snapshot=gf::makeTask10p11sSnapshot(
                        callback_runtime,fixture->adapter.snapshotHardRowRequest(
                            callback_runtime.estimate,
                            callback_runtime.topology),nominal,
                        fixture->adapter.config());
                    t1_decision=gf::task10p11y_detail::decide(snapshot);
                    if (!t1_decision->valid) {
                        gf::GrandFinaleSwarmStep rejected;
                        rejected.reason=t1_decision->reason;
                        return rejected;
                    }
                    if (!t1_decision->preventive_trigger &&
                        t1_decision->baseline_fixed_half)
                        return fixture->adapter.stepWithNominalAndYawRates(
                            nominal,yaw_rates);
                    dynamic_used=true;
                    std::map<gf::NodeId,Eigen::Vector2d> selected_controls=
                        t1_decision->baseline_controls;
                    selected_transfer=t1_decision->baseline_transfer;
                    if (t1_decision->preventive_trigger) {
                        const auto& candidate=t1_decision->candidates.at(
                            t1_decision->selected_index);
                        selected_controls=candidate.controls;
                        selected_transfer=candidate.transfer;
                        preventive_deviation=candidate.coverage_deviation;
                        preventive_intervention=
                            t1_decision->preventive_intervention;
                    }
                    gf::DynamicPairCertifiedControlRequest applied;
                    applied.pair_base_id=gf::task10p11y_detail::kPair;
                    applied.first_owner=
                        t1_decision->current_pair.pair.first.owner;
                    applied.second_owner=
                        t1_decision->current_pair.pair.second.owner;
                    applied.transfer_interval_lower_mps2=
                        t1_decision->current_pair.shared_interval.lower;
                    applied.transfer_interval_upper_mps2=
                        t1_decision->current_pair.shared_interval.upper;
                    applied.transfer_mps2=selected_transfer;
                    applied.controls=std::move(selected_controls);
                    return fixture->adapter.stepWithDynamicPairCertifiedControls(
                        applied,yaw_rates,callback_runtime.estimator_token,
                        callback_runtime.topology_token);
                });
            if (!t1_decision.has_value())
                throw std::runtime_error("T1 decision callback missing");
        }

        const auto nominal=fixture->controller.lastNominalControls();
        if (nominal.size()!=14)
            throw std::runtime_error("current nominal control map incomplete");
        auto snapshot=gf::makeTask10p11sSnapshot(
            runtime,request,nominal,fixture->adapter.config());
        const auto current_pair=gf::solveTask10p11tDynamicPair(
            rows,request.mobile_ids,nominal,request.acceleration_half_box,
            gf::task10p11y_detail::kPair);
        gf::task10p11y_detail::SuccessorAudit successor;
        double current_full=std::numeric_limits<double>::quiet_NaN();
        if (control.step.advanced) {
            current_full=gf::task10p11y_detail::fullRowResidual(
                rows,request,control.step.applied_controls);
            successor=gf::task10p11y_detail::successorAudit(
                snapshot,control.step.applied_controls);
            if (!successor.signed_transfer_interval &&
                !state.first_prediction.has_value())
                state.first_prediction=decision_time;
            if (branch=="T1" && t1_decision->preventive_trigger &&
                !state.first_prediction.has_value())
                state.first_prediction=decision_time;
            if (branch=="T1" && !dynamic_used &&
                controlDifference(control.step.applied_controls,
                    t1_decision->baseline_controls)>1.0e-8)
                throw std::runtime_error(
                    "reconstructed_baseline_control_mismatch");
        } else if (branch=="T1" && t1_decision->preventive_trigger &&
            !state.first_prediction.has_value()) {
            state.first_prediction=decision_time;
        }

        const bool intervention=branch=="C0"
            ?control.step.dynamic_pair.applied:preventive_intervention;
        if (control.step.advanced && dynamic_used) ++state.pair_cycles;
        if (control.step.advanced && intervention &&
            !state.first_intervention.has_value())
            state.first_intervention=decision_time;
        state.statistics.observe(control.step.advanced,intervention,
            preventive_deviation);

        json decision_json=branch=="T1"
            ?gf::task10p11y_detail::decisionJson(*t1_decision)
            :json{{"mode",dynamic_used?"dynamic_signed_transfer":
                    "original_tau14_distributed"},
                {"current_signed_transfer_interval",
                    gf::task10p11y_detail::intervalJson(
                        current_pair.shared_interval.lower,
                        current_pair.shared_interval.upper,
                        current_pair.shared_interval.feasible)},
                {"successor",gf::task10p11y_detail::successorJson(successor)}};
        const bool save_event=!control.step.advanced || dynamic_used ||
            preventive_intervention;
        if (save_event) {
            const std::string event=!control.step.advanced?"fail_closed":
                preventive_intervention?"preventive_intervention":
                "dynamic_signed_transfer";
            auto checkpoint=gf::makeTask10p11vRestartCheckpoint(
                snapshot,restart_fields,event);
            decorateCheckpoint(checkpoint,branch,source,decision_json);
            gf::writeTask10p11vJson(checkpoint_directory/checkpointName(
                state.checkpoint_index++,decision_time,event),checkpoint);
        }

        json trace_entry={{"decision_time_s",decision_time},
            {"advanced",control.step.advanced},{"dynamic_pair_used",dynamic_used},
            {"preventive_intervention",preventive_intervention},
            {"current_signed_transfer_interval",
                gf::task10p11y_detail::intervalJson(
                    current_pair.shared_interval.lower,
                    current_pair.shared_interval.upper,
                    current_pair.shared_interval.feasible)},
            {"selected_transfer_mps2",metric(selected_transfer)},
            {"current_full_row_residual_mps2",metric(current_full)},
            {"successor",gf::task10p11y_detail::successorJson(successor)},
            {"coverage_after",control.step.advanced
                ?gf::task10p11yMetric(control.step.truth_coverage,"observed")
                :gf::task10p11yMetric(std::nullopt,
                    "not_applicable_not_advanced")},
            {"preventive_control_deviation_l2_mps2",
                metric(preventive_deviation)},
            {"reason",control.reason},
            {"decision",decision_json}};
        if (!control.step.advanced) {
            state.trace.push_back(std::move(trace_entry));
            state.stop_reason=control.reason.empty()
                ?control.step.reason:control.reason;
            break;
        }

        const auto truth=truthMargins(*fixture);
        trace_entry["hard_gates_after"]=hardGateJson(*fixture,truth);
        state.trace.push_back(std::move(trace_entry));
        const std::string gate_failure=failedPostAdvanceGate(*fixture,truth);
        if (!gate_failure.empty()) {
            gf::writeTask10p11yPostAdvanceFailureCheckpoint(
                checkpoint_directory/checkpointName(state.checkpoint_index++,
                    fixture->swarm.robots.front()->runtime,
                    "post_advance_fail_closed"),*fixture,gate_failure,
                {{"branch",branch},{"source",source}});
            state.stop_reason=gate_failure;
            break;
        }
        if (fixture->swarm.robots.front()->runtime>=kEndpointS-1.0e-9) {
            auto endpoint=gf::makeTask10p11yCurrentPackedCheckpoint(
                *fixture,"fixed_endpoint_133p0");
            decorateCheckpoint(endpoint,branch,source,
                {{"endpoint",true}});
            gf::writeTask10p11vJson(checkpoint_directory/checkpointName(
                state.checkpoint_index++,
                fixture->swarm.robots.front()->runtime,
                "fixed_endpoint"),endpoint);
            state.stop_reason="fixed_endpoint_133p0";
            break;
        }
    }
    if (state.stop_reason.empty())
        state.stop_reason="maximum_3s_cycle_limit";
    const double final_time=fixture->swarm.robots.front()->runtime;
    const double final_coverage=fixture->adapter.coverage().truthFraction();
    const int final_cells=fixture->adapter.coverage().truthCoveredCount();
    const double wall=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-wall_start).count();
    const bool endpoint=std::abs(final_time-kEndpointS)<=1.0e-9;
    return {{"protocol","task10p11y-early-entry-paired-checkpoint-v1"},
        {"branch",branch},{"source",source},
        {"frozen",{{"start_time_s",130.0},{"endpoint_time_s",133.0},
            {"maximum_simulated_seconds",3.0},{"fixed_topology",true},
            {"tau_mps2",14.0},{"candidate_count",branch=="T1"?9:0},
            {"component_4d_endpoint_used",false},
            {"gamma_star_role","diagnostic_not_parameter"}}},
        {"initial_coverage",initial_coverage},
        {"initial_covered_cells",initial_cells},
        {"final_time_s",final_time},{"reached_fixed_endpoint",endpoint},
        {"final_coverage",final_coverage},{"final_covered_cells",final_cells},
        {"coverage_increment",final_coverage-initial_coverage},
        {"first_prediction_time_s",gf::task10p11yMetric(
            state.first_prediction,"observed_or_not_applicable")},
        {"first_intervention_time_s",gf::task10p11yMetric(
            state.first_intervention,"observed_or_not_applicable")},
        {"advanced_cycles",state.statistics.advanced_cycles},
        {"pair_cycles",state.pair_cycles},
        {"preventive_intervention_cycles",
            state.statistics.intervention_cycles},
        {"cumulative_preventive_control_deviation_l2_mps2",
            state.statistics.cumulative_control_deviation},
        {"stop_reason",state.stop_reason},{"wall_time_s",wall},
        {"checkpoint_count",state.checkpoint_index},
        {"trace",std::move(state.trace)},
        {"claim_boundary",{{"finite_suffix_only",true},
            {"recursive_feasibility_claimed",false},
            {"stage_zero_trajectory_run",false},{"task11_entered",false},
            {"mpc_entered",false},{"backup_cbf_entered",false}}}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=10) {
        std::cerr<<"usage: GrandFinaleTask10p11yPairedCheckpoint C0|T1 "
            "SPARSE_130 CHECKPOINT_DIR RESULT_JSON PARENT_COMMIT PARENT_TREE "
            "CBF_COMMIT CBF_TREE BINARY_SHA256\n";
        return 2;
    }
    try {
        const json source={{"parent_commit",argv[5]},
            {"parent_tree",argv[6]},{"cbf_commit",argv[7]},
            {"cbf_tree",argv[8]},{"binary_sha256",argv[9]}};
        const auto result=run(argv[1],argv[2],argv[3],source);
        gf::writeTask10p11vJson(argv[4],result);
        std::cout<<result.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11y failed: "<<error.what()<<'\n';
        return 3;
    }
}
