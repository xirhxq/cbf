#include "grand_finale/Task10p11zCampaign.hpp"
#include "grand_finale/Task10p11qStandardSafetyOn.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>

namespace {

using json=nlohmann::json;

constexpr std::size_t kMaximumCycles=5000;
constexpr std::size_t kSparseStride=100;
constexpr std::size_t kCoverageStride=10;
constexpr std::size_t kStagnationCycles=1000;
constexpr double kTolerance=1.0e-8;

struct SourceIdentity {
    std::string parent_commit;
    std::string parent_tree;
    std::string cbf_commit;
    std::string cbf_tree;
    std::string binary_sha256;
};

struct TruthMargins {
    double mobile_mobile=std::numeric_limits<double>::infinity();
    double mobile_fixed=std::numeric_limits<double>::infinity();
    double speed=std::numeric_limits<double>::infinity();
};

struct RunState {
    std::optional<std::string> active_pair;
    std::optional<double> first_prediction;
    std::optional<double> first_intervention;
    std::optional<double> t95;
    std::optional<double> t100;
    std::optional<double> coverage_132p4;
    std::optional<double> coverage_133p0;
    std::optional<std::set<gf::NodeId>> component;
    std::size_t advanced_cycles=0;
    std::size_t intervention_cycles=0;
    std::size_t intervention_duration_cycles=0;
    std::size_t component_changes=0;
    std::size_t maximum_component_size=0;
    std::size_t checkpoint_index=0;
    std::size_t sparse_count=0;
    std::size_t stagnant_cycles=0;
    int previous_covered=0;
    double cumulative_deviation=0.0;
    double minimum_current_residual=std::numeric_limits<double>::infinity();
    double minimum_successor_residual=std::numeric_limits<double>::infinity();
    std::string stop_reason;
    json coverage_progress=json::array();
    json intervention_trace=json::array();
};

bool hexDigest(const std::string& value,std::size_t length) {
    return value.size()==length && std::all_of(value.begin(),value.end(),
        [](unsigned char value) { return std::isxdigit(value)!=0; });
}

double profileTau(const std::string& profile) {
    if (profile=="R2") return 18.0;
    if (profile=="R0" || profile=="R1" || profile=="R3") return 14.0;
    throw std::invalid_argument("profile must be R0, R1, R2, or R3");
}

json controlsJson(const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    json result=json::object();
    for (const auto& [owner,control]:controls)
        result[std::to_string(owner)]={control.x(),control.y()};
    return result;
}

double componentDifference(
    const std::map<gf::NodeId,Eigen::Vector2d>& selected,
    const std::map<gf::NodeId,Eigen::Vector2d>& baseline,
    const std::set<gf::NodeId>& component) {
    double squared=0.0;
    for (gf::NodeId owner:component)
        squared+=(selected.at(owner)-baseline.at(owner)).squaredNorm();
    return std::sqrt(squared);
}

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

std::string failedHardGate(gf::Task10p11rFixedBaselineFixture& fixture,
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

std::string checkpointName(std::size_t index,double time,
    const std::string& event) {
    std::ostringstream name;
    name<<"checkpoint-"<<std::setw(4)<<std::setfill('0')<<index
        <<"-t"<<std::fixed<<std::setprecision(1)<<time<<"-"<<event<<".json";
    return name.str();
}

json sourceJson(const SourceIdentity& source) {
    return {{"parent_commit",source.parent_commit},
        {"parent_tree",source.parent_tree},{"cbf_commit",source.cbf_commit},
        {"cbf_tree",source.cbf_tree},{"binary_sha256",source.binary_sha256}};
}

void validatePreregistration(const json& prereg,const std::string& profile,
    double tau,const SourceIdentity& source) {
    const auto frozen=gf::task10p11zPreregisteredCampaign();
    if (prereg.at("protocol")!="task10p11z-preregistration-v1" ||
        prereg.at("profile_order").get<std::vector<std::string>>()!=
            frozen.profile_order ||
        prereg.at("profiles").at(profile).at("tau_mps2").get<double>()!=tau ||
        prereg.at("candidate_count").get<std::size_t>()!=
            frozen.candidate_count ||
        prereg.at("component_size_limit").get<std::size_t>()!=
            frozen.component_size_limit ||
        prereg.at("r4_status")!=frozen.r4_status ||
        prereg.at("source").at("cbf_commit")!=source.cbf_commit ||
        prereg.at("source").at("cbf_tree")!=source.cbf_tree ||
        prereg.at("source").at("binary_sha256")!=source.binary_sha256)
        throw std::runtime_error("preregistration_identity_mismatch");
}

void decorate(json& checkpoint,const std::string& profile,
    const SourceIdentity& source,const json& decision) {
    checkpoint["task10p11z"]={{"protocol","task10p11z-stage-zero-v1"},
        {"profile",profile},{"source",sourceJson(source)},
        {"decision",decision}};
}

void saveSparse(const std::filesystem::path& directory,
    const gf::Task10p11rFixedBaselineFixture& fixture,const RunState& state,
    const std::string& profile,const SourceIdentity& source,double tau) {
    auto sparse=gf::makeTask10p11vSparseRestartCheckpoint(
        fixture,state.active_pair,state.advanced_cycles);
    sparse["task10p11z"]={{"protocol","task10p11z-stage-zero-v1"},
        {"profile",profile},{"source",sourceJson(source)},
        {"tau_mps2",tau},{"fixed_topology",true}};
    std::ostringstream name;
    name<<"sparse-"<<std::setw(4)<<std::setfill('0')
        <<state.advanced_cycles<<"-t"<<std::fixed<<std::setprecision(1)
        <<fixture.swarm.robots.front()->runtime<<".json";
    gf::writeTask10p11vJson(directory/name.str(),sparse);
}

void savePreBoundary(const std::filesystem::path& directory,RunState& state,
    const std::string& profile,const SourceIdentity& source,double time,
    const std::string& event,const json& snapshot,const json& restart,
    const json& decision) {
    auto checkpoint=gf::makeTask10p11vRestartCheckpoint(snapshot,restart,event);
    decorate(checkpoint,profile,source,decision);
    gf::writeTask10p11vJson(directory/checkpointName(
        state.checkpoint_index++,time,event),checkpoint);
}

void saveCurrent(const std::filesystem::path& directory,RunState& state,
    const std::string& profile,const SourceIdentity& source,
    gf::Task10p11rFixedBaselineFixture& fixture,const std::string& event,
    const json& decision) {
    auto checkpoint=gf::makeTask10p11yCurrentPackedCheckpoint(fixture,event);
    decorate(checkpoint,profile,source,decision);
    gf::writeTask10p11vJson(directory/checkpointName(state.checkpoint_index++,
        fixture.swarm.robots.front()->runtime,event),checkpoint);
}

struct CycleEvidence {
    double decision_time=0.0;
    json snapshot;
    json restart;
    json decision=json::object();
    std::map<gf::NodeId,Eigen::Vector2d> baseline_controls;
    std::map<gf::NodeId,Eigen::Vector2d> selected_controls;
    double current_residual=std::numeric_limits<double>::quiet_NaN();
    double successor_residual=std::numeric_limits<double>::quiet_NaN();
    double deviation=0.0;
    bool predicted=false;
    bool intervention=false;
    bool component_change=false;
    std::set<gf::NodeId> component;
};

gf::SimpleCoverageControlStep advanceR0R2(
    gf::Task10p11rFixedBaselineFixture& fixture,RunState& state,
    CycleEvidence& evidence) {
    const auto runtime=fixture.adapter.runtimeSnapshot();
    const auto request=fixture.adapter.snapshotHardRowRequest(
        runtime.estimate,runtime.topology);
    const auto restart=gf::captureTask10p11vRestartFields(fixture);
    const auto rows=gf::buildCanonicalHardRows(request);
    if (!state.active_pair.has_value()) {
        const auto diagnostic=gf::diagnoseTask10p11tOnlineConflicts(
            rows,runtime.estimate.mobile_ids,runtime.runtime_s,runtime.mode,
            runtime.topology,fixture.adapter.config().acceleration_half_box);
        if (!diagnostic.valid) throw std::runtime_error(diagnostic.reason);
        if (diagnostic.infeasible) {
            state.active_pair=gf::task10p11tUniqueOnlinePair(diagnostic);
            if (!state.active_pair.has_value())
                throw std::runtime_error(
                    diagnostic.mobile_pair_base_ids.size()>1
                    ?"multiple_mobile_pair_conflict":
                    "dynamic_pair_conflict_not_unique");
        }
    }
    auto control=state.active_pair.has_value()
        ?fixture.controller.advanceWithDynamicPairResponsibility(
            *state.active_pair):fixture.controller.advance();
    if (fixture.controller.lastNominalControls().size()!=14)
        throw std::runtime_error("native_nominal_controls_incomplete");
    evidence.decision_time=runtime.runtime_s;
    evidence.snapshot=gf::makeTask10p11sSnapshot(runtime,request,
        fixture.controller.lastNominalControls(),fixture.adapter.config());
    evidence.restart=restart;
    evidence.selected_controls=control.step.applied_controls;
    evidence.intervention=control.step.dynamic_pair.applied;
    if (control.step.advanced) {
        evidence.current_residual=gf::task10p11y_detail::fullRowResidual(
            rows,request,control.step.applied_controls);
        const auto successor=gf::task10p11y_detail::successorAudit(
            evidence.snapshot,control.step.applied_controls);
        evidence.successor_residual=successor.full_pair_28d_residual;
        evidence.predicted=!successor.signed_transfer_interval;
        evidence.decision={{"mode",state.active_pair.has_value()
                ?"dynamic_signed_transfer":"native_distributed"},
            {"active_pair",state.active_pair.has_value()
                ?json(*state.active_pair):json(nullptr)},
            {"applied_controls",controlsJson(control.step.applied_controls)},
            {"successor",gf::task10p11y_detail::successorJson(successor)}};
    } else {
        evidence.decision={{"mode","native_fail_closed"},
            {"active_pair",state.active_pair.has_value()
                ?json(*state.active_pair):json(nullptr)},
            {"reason",control.reason}};
    }
    return control;
}

gf::SimpleCoverageControlStep advanceR1(
    gf::Task10p11rFixedBaselineFixture& fixture,RunState& state,double tau,
    CycleEvidence& evidence) {
    const auto prepared=gf::task10p11zPrepareNativeBaseline(fixture,
        gf::GammaFeedbackSelectionMode::LeastIntervention,tau,
        state.active_pair);
    if (!prepared.valid)
        throw std::runtime_error("prepared_native_baseline_failed:"+
            prepared.reason);
    state.active_pair=prepared.active_pair;
    if (!prepared.control.step.advanced) {
        const auto runtime=fixture.adapter.runtimeSnapshot();
        const auto request=fixture.adapter.snapshotHardRowRequest(
            runtime.estimate,runtime.topology);
        evidence.decision_time=runtime.runtime_s;
        evidence.restart=gf::captureTask10p11vRestartFields(fixture);
        evidence.snapshot=gf::makeTask10p11sSnapshot(runtime,request,
            prepared.nominal_controls,fixture.adapter.config());
        evidence.decision={{"valid",false},
            {"mode","prepared_native_fail_closed"},
            {"reason",prepared.reason},
            {"actual_control_advance_attempted",false}};
        return prepared.control;
    }
    std::optional<gf::task10p11y_detail::Decision> decision;
    auto control=fixture.controller.advanceWithDevelopmentControlOverride(
        [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
            const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
            const std::map<gf::NodeId,double>& yaw_rates) {
            if (!gf::task10p11zAppliedControlsMatch(
                    nominal,prepared.nominal_controls,1.0e-12)) {
                gf::GrandFinaleSwarmStep rejected;
                rejected.reason="prepared_native_nominal_mismatch";
                return rejected;
            }
            const auto request=fixture.adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology);
            evidence.decision_time=runtime.runtime_s;
            evidence.restart=gf::captureTask10p11vRestartFields(fixture);
            evidence.snapshot=gf::makeTask10p11sSnapshot(runtime,request,
                nominal,fixture.adapter.config());
            const bool fixed_half=!prepared.control.step.dynamic_pair.applied;
            const double transfer=fixed_half?0.0:
                prepared.control.step.dynamic_pair.selected_transfer_mps2;
            decision=gf::task10p11y_detail::decideWithPreparedBaseline(
                evidence.snapshot,prepared.control.step.applied_controls,
                transfer,fixed_half);
            if (!decision->valid) {
                gf::GrandFinaleSwarmStep rejected;
                rejected.reason=decision->reason;
                return rejected;
            }
            evidence.predicted=decision->preventive_trigger;
            evidence.intervention=decision->preventive_intervention;
            evidence.baseline_controls=decision->baseline_controls;
            evidence.selected_controls=decision->baseline_controls;
            double selected_transfer=decision->baseline_transfer;
            if (decision->preventive_trigger) {
                const auto& selected=decision->candidates.at(
                    decision->selected_index);
                evidence.selected_controls=selected.controls;
                selected_transfer=selected.transfer;
                evidence.deviation=selected.coverage_deviation;
                state.active_pair=gf::task10p11y_detail::kPair;
            }
            if (!decision->preventive_trigger && decision->baseline_fixed_half)
                return fixture.adapter.stepWithNominalAndYawRates(
                    nominal,yaw_rates);
            gf::DynamicPairCertifiedControlRequest request_controls;
            request_controls.pair_base_id=gf::task10p11y_detail::kPair;
            request_controls.first_owner=decision->current_pair.pair.first.owner;
            request_controls.second_owner=decision->current_pair.pair.second.owner;
            request_controls.transfer_interval_lower_mps2=
                decision->current_pair.shared_interval.lower;
            request_controls.transfer_interval_upper_mps2=
                decision->current_pair.shared_interval.upper;
            request_controls.transfer_mps2=selected_transfer;
            request_controls.controls=evidence.selected_controls;
            return fixture.adapter.stepWithDynamicPairCertifiedControls(
                request_controls,yaw_rates,runtime.estimator_token,
                runtime.topology_token);
        });
    if (!decision.has_value())
        throw std::runtime_error("R1_decision_callback_missing");
    evidence.decision=gf::task10p11y_detail::decisionJson(*decision);
    if (control.step.advanced && !gf::task10p11zAppliedControlsMatch(
            control.step.applied_controls,evidence.selected_controls,1.0e-10))
        throw std::runtime_error("committed_preventive_control_mismatch");
    evidence.current_residual=control.step.advanced
        ?gf::task10p11y_detail::fullRowResidual(
            gf::buildCanonicalHardRows(
                gf::task10p11s_capture_detail::requestFromJson(
                    evidence.snapshot.at("canonical_request"))),
            gf::task10p11s_capture_detail::requestFromJson(
                evidence.snapshot.at("canonical_request")),
            control.step.applied_controls)
        :std::numeric_limits<double>::quiet_NaN();
    if (control.step.advanced) {
        const auto successor=gf::task10p11y_detail::successorAudit(
            evidence.snapshot,control.step.applied_controls);
        evidence.successor_residual=successor.full_pair_28d_residual;
    }
    return control;
}

gf::SimpleCoverageControlStep advanceR3(
    gf::Task10p11rFixedBaselineFixture& fixture,RunState& state,double tau,
    CycleEvidence& evidence) {
    const auto prepared=gf::task10p11zPrepareNativeBaseline(fixture,
        gf::GammaFeedbackSelectionMode::LeastIntervention,tau,
        state.active_pair);
    if (!prepared.valid)
        throw std::runtime_error("prepared_native_baseline_invalid:"+
            prepared.reason);
    state.active_pair=prepared.active_pair;
    std::optional<gf::Task10p11zAdaptiveComponentDecision> decision;
    auto control=fixture.controller.advanceWithDevelopmentControlOverride(
        [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
            const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
            const std::map<gf::NodeId,double>& yaw_rates) {
            if (!gf::task10p11zAppliedControlsMatch(
                    nominal,prepared.nominal_controls,1.0e-12)) {
                gf::GrandFinaleSwarmStep rejected;
                rejected.reason="prepared_native_nominal_mismatch";
                return rejected;
            }
            const auto request=fixture.adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology);
            const auto rows=gf::buildCanonicalHardRows(request);
            evidence.decision_time=runtime.runtime_s;
            evidence.restart=gf::captureTask10p11vRestartFields(fixture);
            evidence.snapshot=gf::makeTask10p11sSnapshot(runtime,request,
                nominal,fixture.adapter.config());
            if (prepared.control.step.advanced)
                evidence.baseline_controls=prepared.control.step.applied_controls;
            else
                evidence.baseline_controls=gf::task10p11sControlMap(
                    request.mobile_ids,gf::task10p11w_detail::frozenLocalControls(
                        rows,request,nominal));
            decision=gf::task10p11zDecideAdaptiveComponent(
                evidence.snapshot,evidence.baseline_controls,3);
            if (!decision->valid) {
                gf::GrandFinaleSwarmStep rejected;
                rejected.reason=decision->reason;
                return rejected;
            }
            evidence.predicted=!decision->local_successor_feasible;
            evidence.intervention=decision->use_component;
            evidence.selected_controls=decision->controls;
            evidence.component=decision->component;
            evidence.current_residual=decision->current_full_residual_mps2;
            evidence.successor_residual=decision->successor_full_residual_mps2;
            if (decision->use_component) {
                evidence.deviation=componentDifference(
                    decision->controls,evidence.baseline_controls,
                    decision->component);
                return fixture.adapter.stepWithDevelopmentFullPairCertifiedControls(
                    decision->controls,yaw_rates,runtime.estimator_token,
                    runtime.topology_token,true,
                    decision->successor_full_residual_mps2);
            }
            if (!prepared.control.step.dynamic_pair.applied)
                return fixture.adapter.stepWithNominalAndYawRates(
                    nominal,yaw_rates);
            const auto pair=gf::solveTask10p11tDynamicPair(
                rows,request.mobile_ids,nominal,
                request.acceleration_half_box,gf::task10p11y_detail::kPair);
            gf::DynamicPairCertifiedControlRequest pair_request;
            pair_request.pair_base_id=gf::task10p11y_detail::kPair;
            pair_request.first_owner=pair.pair.first.owner;
            pair_request.second_owner=pair.pair.second.owner;
            pair_request.transfer_interval_lower_mps2=pair.shared_interval.lower;
            pair_request.transfer_interval_upper_mps2=pair.shared_interval.upper;
            pair_request.transfer_mps2=
                prepared.control.step.dynamic_pair.selected_transfer_mps2;
            pair_request.controls=evidence.baseline_controls;
            return fixture.adapter.stepWithDynamicPairCertifiedControls(
                pair_request,yaw_rates,runtime.estimator_token,
                runtime.topology_token);
        });
    if (!decision.has_value())
        throw std::runtime_error("R3_decision_callback_missing");
    evidence.decision=gf::task10p11zAdaptiveComponentDecisionJson(*decision);
    if (control.step.advanced && !gf::task10p11zAppliedControlsMatch(
            control.step.applied_controls,evidence.selected_controls,1.0e-10))
        throw std::runtime_error("committed_component_control_mismatch");
    const std::optional<std::set<gf::NodeId>> selected=
        decision->use_component
            ?std::optional<std::set<gf::NodeId>>(decision->component)
            :std::nullopt;
    evidence.component_change=selected!=state.component;
    if (evidence.component_change) {
        ++state.component_changes;
        state.component=selected;
    }
    state.maximum_component_size=std::max(
        state.maximum_component_size,decision->component.size());
    return control;
}

json runProfile(const std::string& profile,
    gf::Task10p11rFixedBaselineFixture& fixture,
    const std::filesystem::path& checkpoint_directory,
    const SourceIdentity& source,const json& prereg) {
    const double tau=profileTau(profile);
    validatePreregistration(prereg,profile,tau,source);
    const auto initialized=fixture.adapter.initializeStageZero();
    if (!initialized.initialized)
        throw std::runtime_error("stage_zero_initialization_failed:"+
            initialized.reason);
    if (fixture.adapter.config().range_noise_std_m!=0.0 ||
        fixture.adapter.config().range_dropout_probability!=0.0 ||
        !fixture.topologyFrozen())
        throw std::runtime_error("frozen_stage_zero_identity_mismatch");
    std::filesystem::create_directories(checkpoint_directory);
    RunState state;
    state.previous_covered=fixture.adapter.coverage().truthCoveredCount();
    state.coverage_progress.push_back({{"time_s",0.0},
        {"covered_cells",state.previous_covered},
        {"truth_coverage",fixture.adapter.coverage().truthFraction()}});
    const auto wall_start=std::chrono::steady_clock::now();
    for (std::size_t cycle=0;cycle<kMaximumCycles;++cycle) {
        if (!fixture.topologyFrozen()) {
            state.stop_reason="fixed_topology_mutated";
            break;
        }
        if (cycle%kSparseStride==0) {
            saveSparse(checkpoint_directory,fixture,state,profile,source,tau);
            ++state.sparse_count;
        }
        CycleEvidence evidence;
        gf::SimpleCoverageControlStep control;
        if (profile=="R0" || profile=="R2")
            control=advanceR0R2(fixture,state,evidence);
        else if (profile=="R1")
            control=advanceR1(fixture,state,tau,evidence);
        else
            control=advanceR3(fixture,state,tau,evidence);

        const bool first_prediction=evidence.predicted &&
            !state.first_prediction.has_value();
        const bool first_intervention=evidence.intervention &&
            !state.first_intervention.has_value();
        if (first_prediction) state.first_prediction=evidence.decision_time;
        if (first_intervention && control.step.advanced)
            state.first_intervention=evidence.decision_time;
        const bool milestone=std::abs(evidence.decision_time-132.4)<1.0e-9 ||
            std::abs(evidence.decision_time-132.8)<1.0e-9 ||
            std::abs(evidence.decision_time-132.9)<1.0e-9 ||
            std::abs(evidence.decision_time-133.0)<1.0e-9;
        if (first_prediction || first_intervention ||
            evidence.component_change || milestone || !control.step.advanced) {
            const std::string event=!control.step.advanced?"fail_closed":
                evidence.component_change?"component_change":
                first_intervention?"first_intervention":
                first_prediction?"first_prediction":"frozen_time";
            savePreBoundary(checkpoint_directory,state,profile,source,
                evidence.decision_time,event,evidence.snapshot,
                evidence.restart,evidence.decision);
        }
        if (!control.step.advanced) {
            state.stop_reason=control.reason.empty()
                ?control.step.reason:control.reason;
            break;
        }
        ++state.advanced_cycles;
        if (evidence.intervention) {
            ++state.intervention_cycles;
            ++state.intervention_duration_cycles;
            state.cumulative_deviation+=evidence.deviation;
        }
        if (std::isfinite(evidence.current_residual))
            state.minimum_current_residual=std::min(
                state.minimum_current_residual,evidence.current_residual);
        if (std::isfinite(evidence.successor_residual))
            state.minimum_successor_residual=std::min(
                state.minimum_successor_residual,evidence.successor_residual);
        state.intervention_trace.push_back({
            {"decision_time_s",evidence.decision_time},
            {"predicted_local_successor_infeasible",evidence.predicted},
            {"intervention",evidence.intervention},
            {"component",gf::task10p11w_detail::idsJson(evidence.component)},
            {"current_full_row_residual_mps2",gf::task10p11yMetric(
                std::isfinite(evidence.current_residual)
                    ?std::optional<double>(evidence.current_residual):
                     std::nullopt,"observed_or_not_applicable")},
            {"successor_full_pair_residual_mps2",gf::task10p11yMetric(
                std::isfinite(evidence.successor_residual)
                    ?std::optional<double>(evidence.successor_residual):
                     std::nullopt,"observed_or_not_applicable")},
            {"control_deviation_l2_mps2",evidence.deviation},
            {"coverage_after",control.step.truth_coverage},
            {"reason",control.reason}});

        const auto truth=truthMargins(fixture);
        const std::string gate=failedHardGate(fixture,truth);
        if (!gate.empty()) {
            saveCurrent(checkpoint_directory,state,profile,source,fixture,
                "post_advance_hard_gate_failure",{{"reason",gate},
                    {"advanced",true}});
            state.stop_reason=gate;
            break;
        }
        const double time=fixture.swarm.robots.front()->runtime;
        const double coverage=fixture.adapter.coverage().truthFraction();
        if (std::abs(time-132.4)<1.0e-9) state.coverage_132p4=coverage;
        if (std::abs(time-133.0)<1.0e-9) state.coverage_133p0=coverage;
        if (!state.t95.has_value() && coverage>=0.95-1.0e-12) {
            state.t95=time;
            saveCurrent(checkpoint_directory,state,profile,source,fixture,
                "t95",{{"coverage",coverage}});
        }
        if (!state.t100.has_value() && coverage>=1.0-1.0e-12) {
            state.t100=time;
            saveCurrent(checkpoint_directory,state,profile,source,fixture,
                "t100",{{"coverage",coverage}});
        }
        if ((cycle+1)%kCoverageStride==0 || state.t95.has_value() ||
            state.t100.has_value())
            state.coverage_progress.push_back({{"time_s",time},
                {"covered_cells",fixture.adapter.coverage().truthCoveredCount()},
                {"truth_coverage",coverage}});
        if (state.t100.has_value()) {
            state.stop_reason="t100_reached";
            break;
        }
        const int covered=fixture.adapter.coverage().truthCoveredCount();
        state.stagnant_cycles=covered==state.previous_covered
            ?state.stagnant_cycles+1:0;
        state.previous_covered=covered;
        if (state.stagnant_cycles>=kStagnationCycles) {
            state.stop_reason="coverage_stagnation_100s";
            break;
        }
    }
    if (state.stop_reason.empty()) state.stop_reason="registered_500s_limit";
    const double wall=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-wall_start).count();
    const double final_time=fixture.swarm.robots.front()->runtime;
    const double final_coverage=fixture.adapter.coverage().truthFraction();
    if (state.coverage_progress.empty() ||
        state.coverage_progress.back().at("time_s").get<double>()!=final_time)
        state.coverage_progress.push_back({{"time_s",final_time},
            {"covered_cells",fixture.adapter.coverage().truthCoveredCount()},
            {"truth_coverage",final_coverage}});
    const bool hard_failure=state.stop_reason!="t100_reached" &&
        state.stop_reason!="coverage_stagnation_100s" &&
        state.stop_reason!="registered_500s_limit";
    return {{"protocol","task10p11z-stage-zero-result-v1"},
        {"valid",true},{"profile",profile},{"source",sourceJson(source)},
        {"config_digest",gf::task10p11qConfigDigest(fixture.adapter.config())},
        {"frozen",{{"dt_s",0.1},{"collision_distance_m",10.0},
            {"reference_distance_m",850.0},{"strict_reference_distance_m",849.0},
            {"acceleration_half_box_mps2",4.0},{"plant_speed_limit_mps",30.0},
            {"tau_mps2",tau},{"fixed_topology",true},
            {"component_size_limit",3},{"candidate_count",9},
            {"gamma_star_role","diagnostic_not_parameter"}}},
        {"outcome",state.t100.has_value()?"t100":
            hard_failure?"scientific_hard_failure":"bounded_without_t100"},
        {"stop_reason",state.stop_reason},{"hard_failure",hard_failure},
        {"simulated_time_s",final_time},{"wall_time_s",wall},
        {"truth_coverage",final_coverage},
        {"covered_cells",fixture.adapter.coverage().truthCoveredCount()},
        {"coverage_at_132p4",gf::task10p11yMetric(
            state.coverage_132p4,"observed_or_not_applicable")},
        {"coverage_at_133p0",gf::task10p11yMetric(
            state.coverage_133p0,"observed_or_not_applicable")},
        {"t95_s",gf::task10p11yMetric(state.t95,"observed_or_not_applicable")},
        {"t100_s",gf::task10p11yMetric(state.t100,"observed_or_not_applicable")},
        {"first_prediction_s",gf::task10p11yMetric(
            state.first_prediction,"observed_or_not_applicable")},
        {"first_intervention_s",gf::task10p11yMetric(
            state.first_intervention,"observed_or_not_applicable")},
        {"advanced_cycles",state.advanced_cycles},
        {"intervention_cycles",state.intervention_cycles},
        {"intervention_duration_s",0.1*state.intervention_duration_cycles},
        {"cumulative_coverage_control_deviation_l2_mps2",
            state.cumulative_deviation},
        {"minimum_current_full_row_residual_mps2",gf::task10p11yMetric(
            std::isfinite(state.minimum_current_residual)
                ?std::optional<double>(state.minimum_current_residual):
                 std::nullopt,"observed_or_not_applicable")},
        {"minimum_successor_full_pair_residual_mps2",gf::task10p11yMetric(
            std::isfinite(state.minimum_successor_residual)
                ?std::optional<double>(state.minimum_successor_residual):
                 std::nullopt,"observed_or_not_applicable")},
        {"maximum_component_size",state.maximum_component_size},
        {"component_changes",state.component_changes},
        {"topology_switch_count",0},{"checkpoint_count",state.checkpoint_index},
        {"sparse_checkpoint_count",state.sparse_count},
        {"coverage_progress",std::move(state.coverage_progress)},
        {"intervention_trace",std::move(state.intervention_trace)},
        {"claim_boundary",{{"finite_trajectory_only",true},
            {"recursive_feasibility_claimed",false},{"task11_entered",false},
            {"mpc_entered",false},{"backup_cbf_entered",false},
            {"production_14_owner_centralized_controller",false}}}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=10) {
        std::cerr<<"usage: GrandFinaleTask10p11zCampaign R0|R1|R2|R3 "
            "RESULT_JSON CHECKPOINT_DIR PREREG_JSON PARENT_COMMIT "
            "PARENT_TREE CBF_COMMIT CBF_TREE BINARY_SHA256\n";
        return 2;
    }
    const std::string profile=argv[1];
    const std::filesystem::path result_path=argv[2];
    const std::filesystem::path checkpoint_directory=argv[3];
    const SourceIdentity source{argv[5],argv[6],argv[7],argv[8],argv[9]};
    if (!hexDigest(source.parent_commit,40) ||
        !hexDigest(source.parent_tree,40) ||
        !hexDigest(source.cbf_commit,40) || !hexDigest(source.cbf_tree,40) ||
        !hexDigest(source.binary_sha256,64)) {
        std::cerr<<"provenance_preflight_failed\n";
        return 3;
    }
    std::unique_ptr<gf::Task10p11rFixedBaselineFixture> fixture;
    bool advanced=false;
    try {
        const double tau=profileTau(profile);
        fixture=gf::makeTask10p11rFixedBaselineFixture(
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau);
        const auto result=runProfile(profile,*fixture,checkpoint_directory,
            source,gf::readTask10p11vJson(argv[4]));
        advanced=result.at("advanced_cycles").get<std::size_t>()>0;
        gf::writeTask10p11vJson(result_path,result);
        std::cout<<result.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11z campaign failed: "<<error.what()<<'\n';
        if (fixture) {
            try {
                advanced=advanced || fixture->controller.successfulControlCycles()>0;
                gf::writeTask10p11zTerminationEvidence(result_path,
                    checkpoint_directory/"termination-exception.json",*fixture,
                    {gf::Task10p11zTerminationBoundary::Exception,false,
                        advanced,error.what(),{{"profile",profile},
                            {"source",sourceJson(source)}}});
            } catch (const std::exception& publication_error) {
                std::cerr<<"termination evidence publication failed: "
                    <<publication_error.what()<<'\n';
            }
        }
        return 4;
    }
}
