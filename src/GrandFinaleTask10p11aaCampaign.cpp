#include "grand_finale/Task10p11aaCampaign.hpp"
#include "grand_finale/Task10p11aaGraphOracle.hpp"
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
#include <sstream>

namespace {

using json=nlohmann::json;
constexpr std::size_t kMaximumCycles=5000;
constexpr std::size_t kSparseStride=100;
constexpr std::size_t kCoverageStride=10;
constexpr std::size_t kStagnationCycles=1000;
constexpr double kTolerance=1.0e-8;

struct SourceIdentity {
    std::string parent_commit,parent_tree,cbf_commit,cbf_tree,binary_sha256;
};

struct TruthMargins {
    double mobile_mobile=std::numeric_limits<double>::infinity();
    double mobile_fixed=std::numeric_limits<double>::infinity();
    double speed=std::numeric_limits<double>::infinity();
};

struct CycleEvidence {
    double decision_time=0.0;
    json snapshot;
    json restart;
    json decision=json::object();
    std::map<gf::NodeId,Eigen::Vector2d> baseline;
    std::map<gf::NodeId,Eigen::Vector2d> selected;
    bool local_feedback_intervened=false;
    bool dynamic_pair_intervened=false;
    bool graph_intervened=false;
    double current_local_gamma=std::numeric_limits<double>::quiet_NaN();
    double current_global_gamma=std::numeric_limits<double>::quiet_NaN();
    double successor_global_gamma=std::numeric_limits<double>::quiet_NaN();
    double current_residual=std::numeric_limits<double>::quiet_NaN();
    double successor_residual=std::numeric_limits<double>::quiet_NaN();
    double deviation=0.0;
};

struct RunState {
    std::optional<std::string> active_pair;
    std::optional<double> first_local_feedback;
    std::optional<double> first_dynamic_pair;
    std::optional<double> first_graph;
    std::optional<double> first_margin_divergence;
    std::optional<double> t95,t100,coverage132p4,coverage147p2,coverage162p8;
    std::size_t advanced=0,local_feedback_cycles=0,dynamic_pair_cycles=0;
    std::size_t graph_cycles=0,checkpoint_index=0,sparse_count=0;
    std::size_t stagnant=0;
    int previous_covered=0;
    double cumulative_deviation=0.0;
    double minimum_current_residual=std::numeric_limits<double>::infinity();
    double minimum_successor_residual=std::numeric_limits<double>::infinity();
    double minimum_local_gamma=std::numeric_limits<double>::infinity();
    double minimum_global_gamma=std::numeric_limits<double>::infinity();
    double minimum_successor_gamma=std::numeric_limits<double>::infinity();
    double maximum_reference_stretch=0.0;
    std::string stop_reason;
    json coverage_progress=json::array();
    json margin_trace=json::array();
};

bool hexDigest(const std::string& value,std::size_t length) {
    return value.size()==length && std::all_of(value.begin(),value.end(),
        [](unsigned char item){ return std::isxdigit(item)!=0; });
}

double tauFor(const std::string& profile) {
    if (profile=="D20") return 20.0;
    if (profile=="D22") return 22.0;
    if (profile=="G1" || profile=="G2") return 14.0;
    throw std::invalid_argument("profile must be D20, D22, G1, or G2");
}

json sourceJson(const SourceIdentity& value) {
    return {{"parent_commit",value.parent_commit},{"parent_tree",value.parent_tree},
        {"cbf_commit",value.cbf_commit},{"cbf_tree",value.cbf_tree},
        {"binary_sha256",value.binary_sha256}};
}

json metric(const std::optional<double>& value,const std::string& reason) {
    return value.has_value()?json{{"status","valid"},{"value",*value},
        {"reason","observed"}}:json{{"status","not_applicable"},
        {"value",nullptr},{"reason",reason}};
}

json number(double value) {
    return std::isfinite(value)?json(value):json(nullptr);
}

void validatePrereg(const json& prereg,const std::string& profile,
    const SourceIdentity& source) {
    const auto frozen=gf::task10p11aaPreregisteredCampaign();
    if (prereg.at("protocol")!="task10p11aa-preregistration-v1" ||
        prereg.at("frozen_before_first_trajectory")!=true ||
        prereg.at("profile_order").get<std::vector<std::string>>()!=
            frozen.profile_order ||
        prereg.at("profiles").at(profile).at("tau_mps2")!=tauFor(profile) ||
        prereg.at("candidate_count")!=frozen.candidate_count ||
        prereg.at("g2_successor_witness_count")!=
            frozen.g2_successor_witness_count ||
        prereg.at("source").at("cbf_commit")!=source.cbf_commit ||
        prereg.at("source").at("cbf_tree")!=source.cbf_tree ||
        prereg.at("source").at("binary_sha256")!=source.binary_sha256)
        throw std::runtime_error("preregistration_identity_mismatch");
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
        for (std::size_t peer=index+1;peer<fixture.scenario.mobile_ids.size();++peer)
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

std::string hardGate(gf::Task10p11rFixedBaselineFixture& fixture,
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
        truth.mobile_fixed<10.0-1.0e-9) return "truth_collision";
    if (truth.speed<-1.0e-9) return "plant_speed_violation";
    return {};
}

double referenceStretch(const gf::Task10p11rFixedBaselineFixture& fixture) {
    std::map<gf::NodeId,Eigen::Vector2d> positions;
    for (const auto& robot:fixture.swarm.robots) {
        const Point point=robot->model->xy();
        positions[robot->id]={point.x,point.y};
    }
    for (const auto& [fixed,position]:fixture.scenario.fixed_positions)
        positions[fixed]=position;
    double value=0.0;
    for (const auto& edge:fixture.frozen_topology)
        value=std::max(value,(positions.at(edge.owner)-
                              positions.at(edge.reference)).norm());
    return value;
}

std::string checkpointName(std::size_t index,double time,
    const std::string& event) {
    std::ostringstream stream;
    stream<<"checkpoint-"<<std::setw(4)<<std::setfill('0')<<index
        <<"-t"<<std::fixed<<std::setprecision(1)<<time<<"-"<<event<<".json";
    return stream.str();
}

void decorate(json& checkpoint,const std::string& profile,
    const SourceIdentity& source,const json& decision) {
    checkpoint["task10p11aa"]={{"protocol","task10p11aa-stage-zero-v1"},
        {"profile",profile},{"source",sourceJson(source)},
        {"decision",decision}};
}

void saveSparse(const std::filesystem::path& directory,
    const gf::Task10p11rFixedBaselineFixture& fixture,const RunState& state,
    const std::string& profile,const SourceIdentity& source,double tau) {
    auto value=gf::makeTask10p11vSparseRestartCheckpoint(
        fixture,state.active_pair,state.advanced);
    value["task10p11aa"]={{"protocol","task10p11aa-stage-zero-v1"},
        {"profile",profile},{"source",sourceJson(source)},
        {"tau_mps2",tau},{"fixed_topology",true}};
    std::ostringstream name;
    name<<"sparse-"<<std::setw(4)<<std::setfill('0')<<state.advanced
        <<"-t"<<std::fixed<<std::setprecision(1)
        <<fixture.swarm.robots.front()->runtime<<".json";
    gf::writeTask10p11vJson(directory/name.str(),value);
}

void savePre(const std::filesystem::path& directory,RunState& state,
    const std::string& profile,const SourceIdentity& source,
    const CycleEvidence& evidence,const std::string& event) {
    auto value=gf::makeTask10p11vRestartCheckpoint(
        evidence.snapshot,evidence.restart,event);
    decorate(value,profile,source,evidence.decision);
    gf::writeTask10p11vJson(directory/checkpointName(state.checkpoint_index++,
        evidence.decision_time,event),value);
}

void saveCurrent(const std::filesystem::path& directory,RunState& state,
    const std::string& profile,const SourceIdentity& source,
    gf::Task10p11rFixedBaselineFixture& fixture,const std::string& event,
    const json& decision) {
    auto value=gf::makeTask10p11yCurrentPackedCheckpoint(fixture,event);
    decorate(value,profile,source,decision);
    gf::writeTask10p11vJson(directory/checkpointName(state.checkpoint_index++,
        fixture.swarm.robots.front()->runtime,event),value);
}

double localGamma(const std::vector<gf::CanonicalHardRow>& rows,
    const gf::CanonicalHardRowRequest& request) {
    double value=std::numeric_limits<double>::infinity();
    for (gf::NodeId owner:request.mobile_ids) {
        const auto gamma=gf::solveCanonicalGammaStar(
            rows,owner,request.acceleration_half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma))
            return -std::numeric_limits<double>::infinity();
        value=std::min(value,gamma.gamma);
    }
    return value;
}

gf::task10p11aa_detail::FullStateAudit successorGlobal(
    const json& snapshot,const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    const auto estimate0=gf::task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    const auto estimate1=gf::task10p11aa_detail::predictEstimate(
        snapshot,estimate0,controls);
    const auto request1=gf::task10p11x_detail::requestAtEstimate(
        snapshot,estimate1);
    return gf::task10p11aa_detail::fullStateAudit(
        request1,gf::task10p11sOrderedControls(request1.mobile_ids,controls));
}

void computeMargins(CycleEvidence& evidence) {
    const auto request=gf::task10p11s_capture_detail::requestFromJson(
        evidence.snapshot.at("canonical_request"));
    const auto rows=gf::buildCanonicalHardRows(request);
    evidence.current_local_gamma=localGamma(rows,request);
    const auto ordered=gf::task10p11sOrderedControls(
        request.mobile_ids,evidence.selected);
    const auto current=gf::task10p11aa_detail::fullStateAudit(request,ordered);
    evidence.current_global_gamma=current.recomputed_gamma;
    evidence.current_residual=gf::task10p11w_detail::minimumResidual(
        current.problem,ordered).first;
    const auto successor=successorGlobal(evidence.snapshot,evidence.selected);
    evidence.successor_global_gamma=successor.recomputed_gamma;
    evidence.successor_residual=successor.minimum_residual;
}

gf::SimpleCoverageControlStep advanceDistributed(
    gf::Task10p11rFixedBaselineFixture& fixture,RunState& state,
    CycleEvidence& evidence) {
    const auto runtime=fixture.adapter.runtimeSnapshot();
    const auto request=fixture.adapter.snapshotHardRowRequest(
        runtime.estimate,runtime.topology);
    evidence.decision_time=runtime.runtime_s;
    evidence.restart=gf::captureTask10p11vRestartFields(fixture);
    const auto rows=gf::buildCanonicalHardRows(request);
    if (!state.active_pair.has_value()) {
        const auto diagnostic=gf::diagnoseTask10p11tOnlineConflicts(
            rows,runtime.estimate.mobile_ids,runtime.runtime_s,runtime.mode,
            runtime.topology,fixture.adapter.config().acceleration_half_box);
        if (!diagnostic.valid) throw std::runtime_error(diagnostic.reason);
        if (diagnostic.infeasible) {
            state.active_pair=gf::task10p11tUniqueOnlinePair(diagnostic);
            if (!state.active_pair.has_value())
                throw std::runtime_error(diagnostic.mobile_pair_base_ids.size()>1
                    ?"multiple_mobile_pair_conflict":
                     "dynamic_pair_conflict_not_unique");
        }
    }
    auto control=state.active_pair.has_value()
        ?fixture.controller.advanceWithDynamicPairResponsibility(*state.active_pair)
        :fixture.controller.advance();
    if (fixture.controller.lastNominalControls().size()!=14)
        throw std::runtime_error("native_nominal_controls_incomplete");
    evidence.snapshot=gf::makeTask10p11sSnapshot(runtime,request,
        fixture.controller.lastNominalControls(),fixture.adapter.config());
    evidence.selected=control.step.applied_controls;
    evidence.baseline=fixture.controller.lastNominalControls();
    evidence.dynamic_pair_intervened=control.step.dynamic_pair.applied;
    for (const auto& [owner,diagnostic]:control.step.gamma_feedback) {
        (void)owner;
        evidence.local_feedback_intervened=
            evidence.local_feedback_intervened || diagnostic.intervened;
    }
    if (control.step.advanced) {
        computeMargins(evidence);
        const auto nominal=gf::task10p11sOrderedControls(
            request.mobile_ids,evidence.baseline);
        const auto selected=gf::task10p11sOrderedControls(
            request.mobile_ids,evidence.selected);
        evidence.deviation=(selected-nominal).norm();
    }
    evidence.decision={{"mode",state.active_pair.has_value()
            ?"dynamic_signed_transfer":"native_distributed"},
        {"active_pair",state.active_pair.has_value()?json(*state.active_pair):json(nullptr)},
        {"local_gamma_feedback_intervened",evidence.local_feedback_intervened},
        {"dynamic_pair_intervened",evidence.dynamic_pair_intervened},
        {"current_owner_local_gamma_mps2",number(evidence.current_local_gamma)},
        {"current_full_pair_global_gamma_mps2",number(evidence.current_global_gamma)},
        {"successor_full_pair_global_gamma_mps2",number(evidence.successor_global_gamma)}};
    return control;
}

gf::SimpleCoverageControlStep advanceGraph(
    gf::Task10p11rFixedBaselineFixture& fixture,RunState& state,
    CycleEvidence& evidence,gf::Task10p11aaGraphDepth depth) {
    const auto boundary=gf::task10p11zCaptureBeforeOverride(fixture);
    const auto prepared=gf::task10p11zPrepareNativeBaseline(fixture,
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0,
        state.active_pair);
    if (!prepared.valid)
        throw std::runtime_error("prepared_native_baseline_failed:"+
                                 prepared.reason);
    state.active_pair=prepared.active_pair;
    if (!prepared.control.step.advanced)
        throw std::runtime_error("prepared_native_baseline_fail_closed:"+
                                 prepared.reason);
    std::optional<gf::Task10p11aaGraphDecision> decision;
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
            evidence.decision_time=runtime.runtime_s;
            evidence.restart=boundary.restart_fields;
            evidence.snapshot=gf::makeTask10p11sSnapshot(
                runtime,boundary.request,nominal,fixture.adapter.config());
            evidence.baseline=prepared.control.step.applied_controls;
            decision=gf::task10p11aaDecideGraphOracle(
                evidence.snapshot,evidence.baseline,depth);
            if (!decision->valid) {
                gf::GrandFinaleSwarmStep rejected;
                rejected.reason=decision->reason;
                return rejected;
            }
            evidence.selected=decision->selected_controls;
            evidence.graph_intervened=decision->selected_index>0;
            const auto ordered0=gf::task10p11sOrderedControls(
                boundary.request.mobile_ids,evidence.baseline);
            const auto ordered=gf::task10p11sOrderedControls(
                boundary.request.mobile_ids,evidence.selected);
            evidence.deviation=(ordered-ordered0).norm();
            const auto& selected=decision->candidates.at(decision->selected_index);
            evidence.current_local_gamma=localGamma(
                gf::buildCanonicalHardRows(boundary.request),boundary.request);
            evidence.current_global_gamma=
                decision->current_endpoint.recomputed_gamma_mps2;
            evidence.successor_global_gamma=selected.successor_gamma_mps2;
            evidence.current_residual=selected.current_full_row_residual_mps2;
            evidence.successor_residual=
                selected.successor_full_row_residual_mps2;
            for (const auto& [owner,diagnostic]:prepared.control.step.gamma_feedback) {
                (void)owner;
                evidence.local_feedback_intervened=
                    evidence.local_feedback_intervened || diagnostic.intervened;
            }
            evidence.dynamic_pair_intervened=
                prepared.control.step.dynamic_pair.applied;
            return fixture.adapter.stepWithDevelopmentFullPairCertifiedControls(
                evidence.selected,yaw_rates,runtime.estimator_token,
                runtime.topology_token,true,evidence.successor_residual);
        });
    if (!decision.has_value())
        throw std::runtime_error("graph_decision_callback_missing");
    evidence.decision=gf::task10p11aaGraphDecisionJson(*decision);
    if (control.step.advanced && !gf::task10p11zAppliedControlsMatch(
            control.step.applied_controls,evidence.selected,1.0e-10))
        throw std::runtime_error("committed_graph_control_mismatch");
    return control;
}

json runProfile(const std::string& profile,
    gf::Task10p11rFixedBaselineFixture& fixture,
    const std::filesystem::path& directory,const SourceIdentity& source,
    const json& prereg) {
    const double tau=tauFor(profile);
    validatePrereg(prereg,profile,source);
    const auto initialized=fixture.adapter.initializeStageZero();
    if (!initialized.initialized)
        throw std::runtime_error("stage_zero_initialization_failed:"+
                                 initialized.reason);
    if (fixture.adapter.config().range_noise_std_m!=0.0 ||
        fixture.adapter.config().range_dropout_probability!=0.0 ||
        !fixture.topologyFrozen())
        throw std::runtime_error("frozen_stage_zero_identity_mismatch");
    std::filesystem::create_directories(directory);
    RunState state;
    state.previous_covered=fixture.adapter.coverage().truthCoveredCount();
    state.coverage_progress.push_back({{"time_s",0.0},
        {"truth_coverage",fixture.adapter.coverage().truthFraction()}});
    const auto wall_start=std::chrono::steady_clock::now();
    for (std::size_t cycle=0;cycle<kMaximumCycles;++cycle) {
        if (cycle%kSparseStride==0) {
            saveSparse(directory,fixture,state,profile,source,tau);
            ++state.sparse_count;
        }
        CycleEvidence evidence;
        gf::SimpleCoverageControlStep control;
        if (profile=="D20" || profile=="D22")
            control=advanceDistributed(fixture,state,evidence);
        else
            control=advanceGraph(fixture,state,evidence,
                profile=="G1"?gf::Task10p11aaGraphDepth::H1:
                              gf::Task10p11aaGraphDepth::H2FiniteWitness);

        const bool first_local=evidence.local_feedback_intervened &&
            !state.first_local_feedback.has_value();
        const bool first_pair=evidence.dynamic_pair_intervened &&
            !state.first_dynamic_pair.has_value();
        const bool first_graph=evidence.graph_intervened &&
            !state.first_graph.has_value();
        const bool signs_differ=std::isfinite(evidence.current_local_gamma) &&
            std::isfinite(evidence.current_global_gamma) &&
            ((evidence.current_local_gamma>=0.0)!=
             (evidence.current_global_gamma>=0.0) ||
             (std::isfinite(evidence.successor_global_gamma) &&
              (evidence.current_global_gamma>=0.0)!=
              (evidence.successor_global_gamma>=0.0)));
        const bool first_divergence=signs_differ &&
            !state.first_margin_divergence.has_value();
        if (first_local) state.first_local_feedback=evidence.decision_time;
        if (first_pair) state.first_dynamic_pair=evidence.decision_time;
        if (first_graph) state.first_graph=evidence.decision_time;
        if (first_divergence) state.first_margin_divergence=evidence.decision_time;
        const bool milestone=std::abs(evidence.decision_time-132.4)<1e-9 ||
            std::abs(evidence.decision_time-147.2)<1e-9 ||
            std::abs(evidence.decision_time-162.8)<1e-9;
        if (first_local || first_pair || first_graph || first_divergence ||
            milestone || !control.step.advanced) {
            const std::string event=!control.step.advanced?"fail_closed":
                first_graph?"first_graph_intervention":
                first_local?"first_local_gamma_intervention":
                first_pair?"first_dynamic_pair_intervention":
                first_divergence?"first_margin_layer_divergence":"frozen_time";
            savePre(directory,state,profile,source,evidence,event);
        }
        if (!control.step.advanced) {
            state.stop_reason=control.reason.empty()?control.step.reason:
                                                    control.reason;
            break;
        }
        ++state.advanced;
        if (evidence.local_feedback_intervened) ++state.local_feedback_cycles;
        if (evidence.dynamic_pair_intervened) ++state.dynamic_pair_cycles;
        if (evidence.graph_intervened) ++state.graph_cycles;
        state.cumulative_deviation+=evidence.deviation;
        if (std::isfinite(evidence.current_residual))
            state.minimum_current_residual=std::min(
                state.minimum_current_residual,evidence.current_residual);
        if (std::isfinite(evidence.successor_residual))
            state.minimum_successor_residual=std::min(
                state.minimum_successor_residual,evidence.successor_residual);
        if (std::isfinite(evidence.current_local_gamma))
            state.minimum_local_gamma=std::min(
                state.minimum_local_gamma,evidence.current_local_gamma);
        if (std::isfinite(evidence.current_global_gamma))
            state.minimum_global_gamma=std::min(
                state.minimum_global_gamma,evidence.current_global_gamma);
        if (std::isfinite(evidence.successor_global_gamma))
            state.minimum_successor_gamma=std::min(
                state.minimum_successor_gamma,evidence.successor_global_gamma);
        state.maximum_reference_stretch=std::max(
            state.maximum_reference_stretch,referenceStretch(fixture));
        state.margin_trace.push_back({{"time_s",evidence.decision_time},
            {"owner_local_gamma_mps2",number(evidence.current_local_gamma)},
            {"full_pair_global_gamma_mps2",number(evidence.current_global_gamma)},
            {"successor_full_pair_global_gamma_mps2",number(
                evidence.successor_global_gamma)},
            {"local_feedback_intervened",evidence.local_feedback_intervened},
            {"graph_intervened",evidence.graph_intervened},
            {"coverage_after",control.step.truth_coverage}});
        const auto truth=truthMargins(fixture);
        const auto gate=hardGate(fixture,truth);
        if (!gate.empty()) {
            saveCurrent(directory,state,profile,source,fixture,
                "post_advance_hard_gate_failure",{{"reason",gate},{"advanced",true}});
            state.stop_reason=gate;
            break;
        }
        const double time=fixture.swarm.robots.front()->runtime;
        const double coverage=fixture.adapter.coverage().truthFraction();
        if (std::abs(time-132.4)<1e-9) state.coverage132p4=coverage;
        if (std::abs(time-147.2)<1e-9) state.coverage147p2=coverage;
        if (std::abs(time-162.8)<1e-9) state.coverage162p8=coverage;
        if (!state.t95.has_value() && coverage>=0.95-1e-12) {
            state.t95=time;
            saveCurrent(directory,state,profile,source,fixture,"t95",
                {{"coverage",coverage}});
        }
        if (!state.t100.has_value() && coverage>=1.0-1e-12) {
            state.t100=time;
            saveCurrent(directory,state,profile,source,fixture,"t100",
                {{"coverage",coverage}});
        }
        if ((cycle+1)%kCoverageStride==0 || state.t95.has_value() ||
            state.t100.has_value())
            state.coverage_progress.push_back({{"time_s",time},
                {"truth_coverage",coverage},
                {"covered_cells",fixture.adapter.coverage().truthCoveredCount()}});
        if (state.t100.has_value()) { state.stop_reason="t100_reached"; break; }
        const int covered=fixture.adapter.coverage().truthCoveredCount();
        state.stagnant=covered==state.previous_covered?state.stagnant+1:0;
        state.previous_covered=covered;
        if (state.stagnant>=kStagnationCycles) {
            state.stop_reason="coverage_stagnation_100s";
            break;
        }
    }
    if (state.stop_reason.empty()) state.stop_reason="registered_500s_limit";
    const double wall=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-wall_start).count();
    const double final_time=fixture.swarm.robots.front()->runtime;
    const double final_coverage=fixture.adapter.coverage().truthFraction();
    const bool hard_failure=state.stop_reason!="t100_reached" &&
        state.stop_reason!="coverage_stagnation_100s" &&
        state.stop_reason!="registered_500s_limit";
    return {{"protocol","task10p11aa-stage-zero-result-v1"},{"valid",true},
        {"profile",profile},{"source",sourceJson(source)},
        {"config_digest",gf::task10p11qConfigDigest(fixture.adapter.config())},
        {"frozen",{{"dt_s",0.1},{"collision_distance_m",10.0},
            {"reference_distance_m",850.0},{"strict_reference_distance_m",849.0},
            {"acceleration_half_box_mps2",4.0},{"plant_speed_limit_mps",30.0},
            {"tau_mps2",tau},{"hocbf_gains_changed",false},
            {"fixed_topology",true},{"candidate_count",9},
            {"gamma_star_role","state_diagnostic_feedback_signal_not_parameter"}}},
        {"outcome",state.t100.has_value()?"t100":hard_failure?
            "scientific_hard_failure":"bounded_without_t100"},
        {"stop_reason",state.stop_reason},{"hard_failure",hard_failure},
        {"simulated_time_s",final_time},{"wall_time_s",wall},
        {"truth_coverage",final_coverage},
        {"coverage_at_132p4",metric(state.coverage132p4,"not_reached")},
        {"coverage_at_147p2",metric(state.coverage147p2,"not_reached")},
        {"coverage_at_162p8",metric(state.coverage162p8,"not_reached")},
        {"t95_s",metric(state.t95,"not_attained")},
        {"t100_s",metric(state.t100,"not_attained")},
        {"first_local_gamma_feedback_intervention_s",metric(
            state.first_local_feedback,"not_observed")},
        {"first_dynamic_pair_intervention_s",metric(
            state.first_dynamic_pair,"not_observed")},
        {"first_graph_intervention_s",metric(state.first_graph,"not_applicable")},
        {"first_margin_layer_divergence_s",metric(
            state.first_margin_divergence,"not_observed")},
        {"advanced_cycles",state.advanced},
        {"local_gamma_feedback_intervention_cycles",state.local_feedback_cycles},
        {"dynamic_pair_intervention_cycles",state.dynamic_pair_cycles},
        {"graph_intervention_cycles",state.graph_cycles},
        {"cumulative_coverage_control_deviation_l2_mps2",
            state.cumulative_deviation},
        {"maximum_reference_stretch_m",state.maximum_reference_stretch},
        {"minimum_owner_local_gamma_mps2",number(state.minimum_local_gamma)},
        {"minimum_current_full_pair_global_gamma_mps2",number(
            state.minimum_global_gamma)},
        {"minimum_successor_full_pair_global_gamma_mps2",number(
            state.minimum_successor_gamma)},
        {"minimum_current_full_row_residual_mps2",number(
            state.minimum_current_residual)},
        {"minimum_successor_full_row_residual_mps2",number(
            state.minimum_successor_residual)},
        {"checkpoint_count",state.checkpoint_index},
        {"sparse_checkpoint_count",state.sparse_count},
        {"coverage_progress",std::move(state.coverage_progress)},
        {"margin_trace",std::move(state.margin_trace)},
        {"claim_boundary",{{"finite_trajectory_only",true},
            {"recursive_feasibility_claimed",false},
            {"g1_development_centralized_oracle",profile=="G1"},
            {"g2_finite_9x9_witness_not_unrestricted_predecessor",profile=="G2"},
            {"production_14_owner_centralized_controller",false},
            {"task11_entered",false},{"mpc_entered",false},
            {"backup_cbf_entered",false}}}};
}

void publishInvalid(const std::filesystem::path& result_path,
    const std::filesystem::path& directory,const std::string& profile,
    const SourceIdentity& source,gf::Task10p11rFixedBaselineFixture& fixture,
    const std::string& reason) {
    std::filesystem::create_directories(directory);
    const bool advanced=fixture.controller.successfulControlCycles()>0;
    auto checkpoint=fixture.controller.lastNominalControls().size()==14
        ?gf::makeTask10p11yCurrentPackedCheckpoint(fixture,"exception")
        :gf::makeTask10p11vSparseRestartCheckpoint(
            fixture,std::nullopt,fixture.controller.successfulControlCycles());
    decorate(checkpoint,profile,source,{{"valid",false},{"reason",reason},
        {"advanced",advanced}});
    const auto checkpoint_path=directory/"termination-exception.json";
    gf::writeTask10p11vJson(checkpoint_path,checkpoint);
    gf::writeTask10p11vJson(result_path,{{"protocol",
        "task10p11aa-stage-zero-result-v1"},{"valid",false},
        {"profile",profile},{"advanced",advanced},{"reason",reason},
        {"checkpoint_published",true},
        {"checkpoint_path",checkpoint_path.filename().string()},
        {"source",sourceJson(source)}});
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=10) {
        std::cerr<<"usage: GrandFinaleTask10p11aaCampaign "
            "D20|D22|G1|G2 RESULT_JSON CHECKPOINT_DIR PREREG_JSON "
            "PARENT_COMMIT PARENT_TREE CBF_COMMIT CBF_TREE BINARY_SHA256\n";
        return 2;
    }
    const std::string profile=argv[1];
    const std::filesystem::path result_path=argv[2];
    const std::filesystem::path directory=argv[3];
    const SourceIdentity source{argv[5],argv[6],argv[7],argv[8],argv[9]};
    if (!hexDigest(source.parent_commit,40) || !hexDigest(source.parent_tree,40) ||
        !hexDigest(source.cbf_commit,40) || !hexDigest(source.cbf_tree,40) ||
        !hexDigest(source.binary_sha256,64)) {
        std::cerr<<"provenance_preflight_failed\n";
        return 3;
    }
    std::unique_ptr<gf::Task10p11rFixedBaselineFixture> fixture;
    try {
        fixture=gf::makeTask10p11rFixedBaselineFixture(
            gf::GammaFeedbackSelectionMode::LeastIntervention,tauFor(profile));
        const auto result=runProfile(profile,*fixture,directory,source,
            gf::readTask10p11vJson(argv[4]));
        gf::writeTask10p11vJson(result_path,result);
        std::cout<<result.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11aa campaign failed: "<<error.what()<<'\n';
        if (fixture) {
            try { publishInvalid(result_path,directory,profile,source,
                                  *fixture,error.what()); }
            catch (const std::exception& publication) {
                std::cerr<<"termination publication failed: "
                         <<publication.what()<<'\n';
            }
        }
        return 4;
    }
}
