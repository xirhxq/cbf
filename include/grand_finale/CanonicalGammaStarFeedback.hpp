#pragma once

#include "grand_finale/CentralizedEkfOracle.hpp"
#include "grand_finale/ProgressCompatibility.hpp"
#include "grand_finale/Task10p11ComputeProfile.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

enum class GammaFeedbackSelectionMode {
    DiagnosticsOnly,
    LeastIntervention,
    MaximumPredictedMargin
};

struct CanonicalGammaFeedbackConfig {
    double acceleration_half_box = 0.0;
    std::size_t homotopy_segments = 8;
    GammaFeedbackSelectionMode selection_mode =
        GammaFeedbackSelectionMode::DiagnosticsOnly;
    std::optional<double> predictive_tau_mps2;
    double feasibility_tolerance = 1.0e-10;
    // Task 11b V-b (prereg v1.1 section 11.1): pure efficiency gate - skip
    // the per-candidate predictive rollouts when the owner's current gamma
    // already clears the frozen threshold.  Safety is unaffected: the
    // full hard-QP path still runs on the task projection.
    bool margin_gate_enabled = false;
    double margin_gate_threshold_mps2 = 1.0;
    // Task 11b V-c (prereg): restrictive predictive scoring - the predicted
    // gamma* for an owner is solved over the limiting row family recorded on
    // the previous tick (base id of the previous dominant row) instead of
    // the full row set.  Null pointer = baseline semantics.
    const std::map<NodeId,std::string>* limiting_family_filter = nullptr;
    // Task 11b V-a (prereg): first-order analytic prediction - the predicted
    // gamma for a candidate is its exact current-state margin plus dt times
    // the analytic row-margin rate along the ZOH trajectory, replacing the
    // per-candidate propagation + rebuild + solve.  Validated against
    // central finite differences by focused tests (section 11.3 guardrail).
    bool analytic_first_order_enabled = false;
    // Task 12 Phase 1 (approved 2026-08-31): margin-aware nominal throttle.
    // Purely soft-task: scales the coverage nominal velocity by the frozen
    // piecewise rule of the min-owner current gamma*.  Hard ledger, tau and
    // predictive evaluation untouched.  Zero extra solves (reuses the
    // per-owner current gamma* already computed each tick).
    bool nominal_throttle_enabled = false;
    double throttle_gamma_th_mps2 = 2.0;
    double throttle_gamma_floor_mps2 = 0.5;
    // Task 11b S1-v4: nominal speed saturation (0 disables).
    double nominal_speed_saturation_mps = 0.0;
    // Task 12 Phase 1 v2.2: endpoint in-flight margin signal.
    bool throttle_v2_enabled = false;
    std::map<NodeId,std::string> throttle_v2_endpoint_family;
    // Task 12 Phase 1 v2.2: signal replacement - per-endpoint in-flight
    // margin on the eroding row family (selected-control basis), instead of
    // the exact box-optimal gamma*.  Endpoints frozen from the Phase 0
    // derivation: owner 2 <-> reference:101->2, owner 4 <-> reference:2->4.
};

inline double task12NominalThrottleScale(
    double min_owner_current_gamma,double gamma_th,double gamma_floor) {
    if (min_owner_current_gamma>=gamma_th) return 1.0;
    if (min_owner_current_gamma<=gamma_floor) return 0.0;
    return (min_owner_current_gamma-gamma_floor)/(gamma_th-gamma_floor);
}

// Task 11b V-a: analytic time derivative of a canonical row margin along
// the ZOH trajectory with the candidate control held.  Derived per row kind
// from the builder formulas (CanonicalHardRows / SnapshotRobustPairRow /
// PlantSpeedAppliedControl); validated against central finite differences
// by focused tests (prereg v1.1 section 11.3 guardrail).
inline double task11bAnalyticRowRate(
    const CanonicalHardRow& row,NodeId owner,
    const Eigen::Vector2d& control,
    const std::map<NodeId,PairwiseSecondOrderState2D>& states,
    const std::map<NodeId,Eigen::Vector2d>& all_controls,
    const PairwiseSecondOrderRowSpec& collision_spec,
    const PairwiseSecondOrderRowSpec& reference_spec,
    double acceleration_half_box,double dt_s) {
    const Eigen::Vector2d u=control;
    switch (row.kind) {
    case CanonicalHardRowKind::InputBox:
    case CanonicalHardRowKind::Auxiliary:
        return 0.0;
    case CanonicalHardRowKind::Workspace: {
        const auto& state=states.at(owner);
        // row.normal equals the stored control coefficient (-facet normal);
        // margin = normal.u + h + 2*hdot with h,hdot affine in (p,v).
        return row.normal.dot(state.velocity+2.0*u);
    }
    case CanonicalHardRowKind::PlantSpeedAppliedControl: {
        const auto& state=states.at(owner);
        const Eigen::Vector2d n=-row.control_coefficient;
        const Eigen::Vector2d n_perp(-n.y(),n.x());
        const double speed2=state.velocity.squaredNorm();
        const double theta_dot=speed2>1e-12?
            (state.velocity.x()*u.y()-state.velocity.y()*u.x())/speed2:
            0.0;
        return -theta_dot*n_perp.dot(u)+
            (-theta_dot*n_perp.dot(state.velocity)-n.dot(u))/dt_s;
    }
    case CanonicalHardRowKind::SpeedLimit: {
        const auto& state=states.at(owner);
        const Eigen::Vector2d c=row.control_coefficient;
        const double gain=1.0;  // formal speed_cbf_gain
        return c.dot(u)-2.0*gain*state.velocity.dot(u);
    }
    case CanonicalHardRowKind::ReferenceDistance:
    case CanonicalHardRowKind::Collision: {
        if (!row.peer.has_value()) return 0.0;
        const NodeId peer=*row.peer;
        const auto& first=states.at(owner);
        const auto& second=states.at(peer);
        Eigen::Vector2d u_second=Eigen::Vector2d::Zero();
        const auto found=all_controls.find(peer);
        if (found!=all_controls.end()) u_second=found->second;
        const Eigen::Vector2d rel_p(first.position.x-second.position.x,
            first.position.y-second.position.y);
        const Eigen::Vector2d rel_v=first.velocity-second.velocity;
        const Eigen::Vector2d rel_u=u-u_second;
        const double d=rel_p.norm();
        if (d<1e-9) return 0.0;
        const Eigen::Vector2d n_hat=rel_p/d;
        const double radial=n_hat.dot(rel_v);
        const double speed=rel_v.norm();
        const double speed_rate=speed>1e-9?
            rel_v.dot(rel_u)/speed:0.0;
        const double n_hat_rate_dot_u_owner=
            (rel_v.dot(u)-radial*n_hat.dot(u))/d;
        const double radial_rate=
            (rel_v.dot(rel_v)-radial*radial)/d+n_hat.dot(rel_u);
        const double pr=row.position_uncertainty_reserve_m;
        const double q=pr/d;
        if (!(q<1.0)) return 0.0;
        const double s_q=std::sqrt(1.0-q*q);
        const double dir=std::sqrt(2.0*(1.0-s_q));
        const double q_dot=-q*radial/d;
        const double dir_dot=q>1e-12?
            -q*q*radial/(d*s_q*dir):0.0;
        const double reserve_rate=acceleration_half_box*std::sqrt(2.0)*
            dir_dot;
        const bool collision=row.kind==CanonicalHardRowKind::Collision;
        const auto& spec=collision?collision_spec:reference_spec;
        const double tv=row.velocity_uncertainty_reserve_mps;
        double central_rate=0.0;
        if (collision) {
            central_rate=spec.lambda1*spec.lambda2*radial+
                (spec.lambda1+spec.lambda2)*
                (radial_rate-dir_dot*speed-dir*speed_rate);
        } else {
            const double speed_upper=speed+tv;
            const double d_lower=d-pr;
            if (d_lower<=0.0) return 0.0;
            const double ratio_rate=
                (2.0*speed_upper*speed_rate*d_lower-
                 speed_upper*speed_upper*radial)/(d_lower*d_lower);
            central_rate=spec.lambda1*spec.lambda2*(-radial)+
                (spec.lambda1+spec.lambda2)*
                (-(radial_rate+dir_dot*speed+dir*speed_rate))-
                ratio_rate;
        }
        const double control_term_rate=
            (collision?1.0:-1.0)*n_hat_rate_dot_u_owner;
        return control_term_rate+row.responsibility*central_rate-
            reserve_rate;
    }
    }
    return 0.0;
}

inline std::string task11bRowFamilyBase(const std::string& row_id) {
    std::string base=row_id;
    const std::string owner_suffix=":owner:";
    auto position=base.rfind(owner_suffix);
    if (position!=std::string::npos) base=base.substr(0,position);
    const std::string facet_suffix=":facet:";
    position=base.rfind(facet_suffix);
    if (position!=std::string::npos) base=base.substr(0,position);
    return base;
}

struct CanonicalGammaFeedbackStage {
    bool valid = false;
    std::string reason;
    NodeId owner = 0;
    double current_gamma = -std::numeric_limits<double>::infinity();
    Eigen::Vector2d task_projection = Eigen::Vector2d::Zero();
    Eigen::Vector2d maximum_margin_control = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> candidates;
    std::vector<std::string> current_row_ids;
    std::vector<CanonicalHardRow> current_rows;
};

struct CanonicalPredictedGammaScore {
    bool valid = false;
    double gamma = -std::numeric_limits<double>::infinity();
    std::string dominant_row;
};

struct CanonicalGammaFeedbackResult {
    bool valid = false;
    std::string reason;
    Eigen::Vector2d selected_control = Eigen::Vector2d::Zero();
    double nominal_predicted_gamma =
        -std::numeric_limits<double>::infinity();
    double maximum_margin_candidate_predicted_gamma =
        -std::numeric_limits<double>::infinity();
    double selected_predicted_gamma =
        -std::numeric_limits<double>::infinity();
    double controllable_predicted_gamma_range = 0.0;
    bool intervened = false;
    bool actual_next_gamma_guaranteed = false;
    std::size_t selected_candidate_index = 0;
    bool tau_attainment_valid = false;
    bool tau_attained = false;
    std::string dominant_row;
    std::string fallback_reason;
    // Task 12 Phase 1 throttle telemetry (activation = s<1).
};

struct CanonicalGammaFeedbackWork {
    std::size_t policy_evaluations = 0;
    std::size_t canonical_row_rebuilds = 0;
    std::size_t exact_gamma_solves = 0;
    std::size_t estimator_propagations = 0;
    std::size_t focused_owner_row_rebuilds = 0;
};

struct CanonicalGammaFeedbackEvaluationContext {
    bool active = false;
    std::size_t entries = 0;
};

struct CanonicalGammaFeedbackBatchResult {
    bool valid = false;
    std::string reason;
    std::vector<CanonicalHardRow> current_rows;
    std::map<NodeId,CanonicalGammaFeedbackStage> stages;
    std::map<NodeId,CanonicalGammaFeedbackResult> selections;
    std::map<NodeId,Eigen::Vector2d> selected_controls;
    CanonicalGammaFeedbackWork work;
    Task10p11ComputeProfile compute_profile;
    // Task 12 Phase 1 throttle telemetry (activation = s<1).
    bool throttle_active=false;
    double throttle_s=1.0;
    double throttle_min_gamma=-std::numeric_limits<double>::infinity();
    NodeId throttle_limiting_owner=0;
    // v2.2 per-endpoint telemetry
    std::map<NodeId,double> throttle_v2_endpoint_s;
    std::map<NodeId,double> throttle_v2_endpoint_signal;
    bool throttle_v2_active=false;
};

inline void validateCanonicalGammaFeedbackConfig(
    const CanonicalGammaFeedbackConfig& config) {
    if (!std::isfinite(config.acceleration_half_box) ||
        config.acceleration_half_box <= 0.0 ||
        config.homotopy_segments == 0 ||
        !std::isfinite(config.feasibility_tolerance) ||
        config.feasibility_tolerance < 0.0 ||
        (config.predictive_tau_mps2.has_value() &&
         (!std::isfinite(*config.predictive_tau_mps2) ||
          *config.predictive_tau_mps2 < 0.0))) {
        throw std::invalid_argument("invalid canonical gamma feedback configuration");
    }
    if (config.selection_mode == GammaFeedbackSelectionMode::LeastIntervention &&
        !config.predictive_tau_mps2.has_value()) {
        throw std::invalid_argument(
            "least-intervention mode requires an explicit predictive tau");
    }
}

inline CanonicalGammaFeedbackStage buildCanonicalGammaFeedbackStage(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,
    const Eigen::Vector2d& task_nominal,
    const CanonicalGammaFeedbackConfig& config,
    Task10p11ComputeProfile* profile=nullptr) {
    validateCanonicalGammaFeedbackConfig(config);
    CanonicalGammaFeedbackStage result;
    result.owner=owner;
    if (!task_nominal.allFinite()) {
        result.reason="invalid_task_nominal";
        return result;
    }
    for (const auto& row:rows) {
        if (row.owner==owner) {
            result.current_row_ids.push_back(row.id);
            result.current_rows.push_back(row);
        }
    }
    const auto gamma_started=std::chrono::steady_clock::now();
    const auto current=solveCanonicalGammaStar(
        rows,owner,config.acceleration_half_box);
    if (profile!=nullptr) profile->record(
        Task10p11ComputePhase::CurrentGamma,
        std::chrono::duration<double>(
            std::chrono::steady_clock::now()-gamma_started).count(),true);
    if (!current.valid || !std::isfinite(current.gamma)) {
        result.reason="current_gamma_invalid";
        return result;
    }
    result.current_gamma=current.gamma;
    result.maximum_margin_control={current.accelX,current.accelY};
    if (current.gamma < 0.0) {
        result.reason="current_gamma_negative";
        return result;
    }
    const auto projection_started=std::chrono::steady_clock::now();
    const auto projected=evaluateProgressCompatibility(
        rows,owner,task_nominal,config.acceleration_half_box,
        {std::numeric_limits<double>::max(),0.0,
         config.feasibility_tolerance,true});
    if (profile!=nullptr) profile->record(
        Task10p11ComputePhase::ExactHardProjection,
        std::chrono::duration<double>(
            std::chrono::steady_clock::now()-projection_started).count(),true);
    if (!projected.polytope_nonempty ||
        projected.minimum_hard_residual < -config.feasibility_tolerance) {
        result.reason="current_hard_projection_failed";
        return result;
    }
    result.task_projection=projected.projection;
    result.candidates.reserve(config.homotopy_segments+1);
    for (std::size_t index=0;index<=config.homotopy_segments;++index) {
        const double alpha=static_cast<double>(index)/
            static_cast<double>(config.homotopy_segments);
        const Eigen::Vector2d candidate=(1.0-alpha)*result.task_projection+
            alpha*result.maximum_margin_control;
        if (minimumCanonicalOwnerResidual(rows,owner,candidate)<
            -config.feasibility_tolerance) {
            result.reason="homotopy_candidate_not_current_feasible";
            result.candidates.clear();
            return result;
        }
        result.candidates.push_back(candidate);
    }
    result.valid=true;
    result.reason="current_stage_valid";
    return result;
}

template<class ScoreFn>
CanonicalGammaFeedbackResult selectCanonicalGammaFeedback(
    const CanonicalGammaFeedbackStage& stage,
    const CanonicalGammaFeedbackConfig& config,ScoreFn&& score) {
    validateCanonicalGammaFeedbackConfig(config);
    CanonicalGammaFeedbackResult result;
    if (!stage.valid || stage.candidates.empty()) {
        result.reason="invalid_current_stage";
        return result;
    }
    if (config.margin_gate_enabled &&
        stage.current_gamma>=config.margin_gate_threshold_mps2) {
        // V-b (prereg v1.1 section 11.1): skip the predictive rollouts and
        // score the candidates with exact current-state margins instead.
        // "沿用 current gamma" semantics - least-intervention selection on
        // current-state values, no estimator propagation or row rebuild.
        std::vector<CanonicalPredictedGammaScore> current_scores;
        current_scores.reserve(stage.candidates.size());
        for (const auto& candidate:stage.candidates) {
            CanonicalPredictedGammaScore value;
            value.valid=true;
            value.gamma=std::numeric_limits<double>::infinity();
            for (const auto& row:stage.current_rows)
                value.gamma=std::min(value.gamma,row.margin(candidate));
            if (!std::isfinite(value.gamma)) value.gamma=
                -std::numeric_limits<double>::infinity();
            current_scores.push_back(value);
        }
        result.valid=true;
        result.reason="margin_gate_current_state_scoring";
        std::size_t selected=0;
        if (config.selection_mode==
            GammaFeedbackSelectionMode::LeastIntervention) {
            result.tau_attainment_valid=true;
            const auto threshold=std::find_if(
                current_scores.begin(),current_scores.end(),
                [&](const auto& value) {
                    return value.gamma+config.feasibility_tolerance>=
                        *config.predictive_tau_mps2;
                });
            if (threshold==current_scores.end()) {
                selected=static_cast<std::size_t>(std::distance(
                    current_scores.begin(),std::max_element(
                        current_scores.begin(),current_scores.end(),
                        [](const auto& lhs,const auto& rhs) {
                            return lhs.gamma<rhs.gamma;
                        })));
                result.tau_attained=false;
                result.fallback_reason=
                    "margin_gate_tau_unattained_maximum_current_margin";
            } else {
                selected=static_cast<std::size_t>(std::distance(
                    current_scores.begin(),threshold));
                result.tau_attained=true;
            }
        }
        result.selected_control=stage.candidates[selected];
        result.selected_candidate_index=selected;
        result.selected_predicted_gamma=current_scores[selected].gamma;
        result.nominal_predicted_gamma=current_scores.front().gamma;
        result.maximum_margin_candidate_predicted_gamma=
            current_scores.back().gamma;
        result.controllable_predicted_gamma_range=
            current_scores.back().gamma-current_scores.front().gamma;
        result.intervened=selected!=0;
        result.dominant_row=current_scores[selected].dominant_row;
        return result;
    }
    std::vector<CanonicalPredictedGammaScore> scores;
    scores.reserve(stage.candidates.size());
    for (const auto& candidate:stage.candidates) {
        const auto value=score(candidate);
        if (!value.valid || !std::isfinite(value.gamma)) {
            result.valid=true;
            result.reason="current_projection_fallback";
            result.selected_control=stage.task_projection;
            result.selected_candidate_index=0;
            result.tau_attainment_valid=false;
            result.fallback_reason="invalid_prediction_use_current_projection";
            return result;
        }
        scores.push_back(value);
    }
    result.nominal_predicted_gamma=scores.front().gamma;
    result.maximum_margin_candidate_predicted_gamma=scores.back().gamma;
    const auto best=std::max_element(
        scores.begin(),scores.end(),[](const auto& lhs,const auto& rhs) {
            return lhs.gamma<rhs.gamma;
        });
    const std::size_t best_index=static_cast<std::size_t>(
        std::distance(scores.begin(),best));
    std::size_t selected=0;
    if (config.selection_mode==GammaFeedbackSelectionMode::MaximumPredictedMargin) {
        selected=best_index;
    } else if (config.selection_mode==GammaFeedbackSelectionMode::LeastIntervention) {
        result.tau_attainment_valid=true;
        const auto threshold=std::find_if(
            scores.begin(),scores.end(),[&](const auto& value) {
                return value.gamma+config.feasibility_tolerance>=
                    *config.predictive_tau_mps2;
            });
        if (threshold==scores.end()) {
            selected=best_index;
            result.tau_attained=false;
            result.fallback_reason="tau_unattained_maximum_predicted_margin";
        } else {
            selected=static_cast<std::size_t>(
                std::distance(scores.begin(),threshold));
            result.tau_attained=true;
        }
    }
    result.valid=true;
    result.reason="selected";
    result.selected_control=stage.candidates[selected];
    result.selected_candidate_index=selected;
    result.selected_predicted_gamma=scores[selected].gamma;
    result.controllable_predicted_gamma_range=
        best->gamma-scores.front().gamma;
    result.intervened=selected!=0;
    result.dominant_row=scores[selected].dominant_row;
    return result;
}

inline JointEstimateSnapshot predictNoMeasurementSnapshot(
    const JointEstimateSnapshot& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& controls,double dt_s,
    double acceleration_variance) {
    if (!std::isfinite(dt_s) || dt_s<=0.0 ||
        !std::isfinite(acceleration_variance) || acceleration_variance<0.0 ||
        snapshot.mean.size()!=static_cast<Eigen::Index>(4*snapshot.mobile_ids.size()) ||
        snapshot.covariance.rows()!=snapshot.mean.size() ||
        snapshot.covariance.cols()!=snapshot.mean.size()) {
        throw std::invalid_argument("invalid no-measurement prediction input");
    }
    JointEstimateSnapshot result=snapshot;
    Eigen::Matrix4d transition=Eigen::Matrix4d::Identity();
    transition(0,2)=dt_s;
    transition(1,3)=dt_s;
    Eigen::Matrix<double,4,2> input;
    input<<0.5*dt_s*dt_s,0.0,
           0.0,0.5*dt_s*dt_s,
           dt_s,0.0,
           0.0,dt_s;
    Eigen::MatrixXd joint_transition=Eigen::MatrixXd::Identity(
        snapshot.mean.size(),snapshot.mean.size());
    Eigen::MatrixXd process=Eigen::MatrixXd::Zero(
        snapshot.mean.size(),snapshot.mean.size());
    for (std::size_t index=0;index<snapshot.mobile_ids.size();++index) {
        const auto control=controls.find(snapshot.mobile_ids[index]);
        if (control==controls.end() || !control->second.allFinite())
            throw std::invalid_argument("prediction requires every mobile control");
        const Eigen::Index offset=static_cast<Eigen::Index>(4*index);
        result.mean.segment<4>(offset)=
            transition*snapshot.mean.segment<4>(offset)+input*control->second;
        joint_transition.block<4,4>(offset,offset)=transition;
        process.block<4,4>(offset,offset)=
            acceleration_variance*input*input.transpose();
    }
    result.covariance=joint_transition*snapshot.covariance*
        joint_transition.transpose()+process;
    result.covariance=0.5*(result.covariance+result.covariance.transpose());
    return result;
}

inline std::string dominantCanonicalOwnerRow(
    const std::vector<CanonicalHardRow>& rows,NodeId owner,
    const Eigen::Vector2d& control) {
    const CanonicalHardRow* dominant=nullptr;
    for (const auto& row:rows) {
        if (row.owner!=owner || !row.participates_in_gamma) continue;
        if (dominant==nullptr || row.margin(control)<dominant->margin(control))
            dominant=&row;
    }
    return dominant==nullptr ? std::string{} : dominant->id;
}

template<class BuildRequestFn>
CanonicalGammaFeedbackBatchResult evaluateCanonicalGammaFeedbackBatchReference(
    const JointEstimateSnapshot& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& task_nominals,
    const CanonicalGammaFeedbackConfig& config,double dt_s,
    double acceleration_variance,BuildRequestFn&& build_request,
    CanonicalGammaFeedbackEvaluationContext& context) {
    CanonicalGammaFeedbackBatchResult result;
    if (context.active) {
        result.reason="recursive_policy_evaluation";
        return result;
    }
    struct ActiveGuard {
        explicit ActiveGuard(CanonicalGammaFeedbackEvaluationContext& value)
            : context(value) {
            context.active=true;
            ++context.entries;
        }
        ~ActiveGuard() { context.active=false; }
        CanonicalGammaFeedbackEvaluationContext& context;
    } guard(context);
    result.work.policy_evaluations=1;
    try {
        result.current_rows=buildCanonicalHardRows(build_request(snapshot));
        ++result.work.canonical_row_rebuilds;
    } catch (...) {
        result.reason="current_canonical_row_rebuild_failed";
        return result;
    }
    std::map<NodeId,Eigen::Vector2d> projected_controls;
    for (NodeId owner:snapshot.mobile_ids) {
        const auto nominal=task_nominals.find(owner);
        if (nominal==task_nominals.end()) {
            result.reason="task_nominal_missing";
            return result;
        }
        ++result.work.exact_gamma_solves;
        auto stage=buildCanonicalGammaFeedbackStage(
            result.current_rows,owner,nominal->second,config,
            &result.compute_profile);
        if (!stage.valid) {
            result.reason=stage.reason;
            return result;
        }
        projected_controls.emplace(owner,stage.task_projection);
        result.stages.emplace(owner,std::move(stage));
    }
    // Shared V-a state table: current states for analytic row rates.
    const auto base_request=build_request(snapshot);
    const auto collision_spec=base_request.collision_spec;
    const auto reference_spec=base_request.reference_spec;
    std::map<NodeId,PairwiseSecondOrderState2D> current_states;
    if (config.analytic_first_order_enabled)
        for (std::size_t index=0;index<snapshot.mobile_ids.size();++index) {
            const Eigen::Vector4d state=snapshot.mean.segment<4>(
                static_cast<Eigen::Index>(4*index));
            PairwiseSecondOrderState2D value;
            value.position=Point(state.x(),state.y());
            value.velocity=state.tail<2>();
            value.acceleration=Eigen::Vector2d::Zero();
            current_states.emplace(snapshot.mobile_ids[index],value);
        }
    for (NodeId owner:snapshot.mobile_ids) {
        const auto& stage=result.stages.at(owner);
        const auto selected=selectCanonicalGammaFeedback(
            stage,config,
            config.analytic_first_order_enabled?
                static_cast<std::function<CanonicalPredictedGammaScore(
                    const Eigen::Vector2d&)>>(
                [&](const Eigen::Vector2d& candidate) {
                    CanonicalPredictedGammaScore value;
                    value.valid=true;
                    value.gamma=std::numeric_limits<double>::infinity();
                    const CanonicalHardRow* limiting=nullptr;
                    for (const auto& row:stage.current_rows) {
                        const double margin=row.margin(candidate);
                        if (margin<value.gamma) {
                            value.gamma=margin;
                            limiting=&row;
                        }
                    }
                    if (limiting==nullptr)
                        return CanonicalPredictedGammaScore{};
                    const double rate=task11bAnalyticRowRate(
                        *limiting,owner,candidate,current_states,
                        projected_controls,collision_spec,reference_spec,
                        config.acceleration_half_box,dt_s);
                    if (!std::isfinite(rate))
                        return CanonicalPredictedGammaScore{};
                    value.gamma+=dt_s*rate;
                    value.dominant_row=limiting->id;
                    return value;
                }):
                static_cast<std::function<CanonicalPredictedGammaScore(
                    const Eigen::Vector2d&)>>(
                [&](const Eigen::Vector2d& candidate) {
                try {
                    auto surrogate_controls=projected_controls;
                    surrogate_controls.at(owner)=candidate;
                    ++result.work.estimator_propagations;
                    const auto predicted=predictNoMeasurementSnapshot(
                        snapshot,surrogate_controls,dt_s,
                        acceleration_variance);
                    ++result.work.canonical_row_rebuilds;
                    auto predicted_rows=buildCanonicalHardRows(
                        build_request(predicted));
                    if (config.limiting_family_filter!=nullptr) {
                        const auto family=
                            config.limiting_family_filter->find(owner);
                        if (family!=config.limiting_family_filter->end() &&
                            !family->second.empty()) {
                            std::vector<CanonicalHardRow> filtered;
                            for (auto& row:predicted_rows) {
                                if (row.owner!=owner) continue;
                                const std::string base=task11bRowFamilyBase(
                                    row.id);
                                if (base==family->second ||
                                    row.kind==
                                        CanonicalHardRowKind::InputBox)
                                    filtered.push_back(row);
                            }
                            if (!filtered.empty())
                                predicted_rows=std::move(filtered);
                        }
                    }
                    ++result.work.exact_gamma_solves;
                    const auto gamma=solveCanonicalGammaStar(
                        predicted_rows,owner,config.acceleration_half_box);
                    if (!gamma.valid || !std::isfinite(gamma.gamma))
                        return CanonicalPredictedGammaScore{};
                    const Eigen::Vector2d witness(gamma.accelX,gamma.accelY);
                    return CanonicalPredictedGammaScore{
                        true,gamma.gamma,dominantCanonicalOwnerRow(
                            predicted_rows,owner,witness)};
                } catch (...) {
                    return CanonicalPredictedGammaScore{};
                }
            }));
        if (!selected.valid) {
            result.reason=selected.reason;
            return result;
        }
        result.selected_controls.emplace(owner,selected.selected_control);
        result.selections.emplace(owner,selected);
    }
    result.valid=true;
    result.reason="selected";
    return result;
}

template<class BuildRequestFn>
CanonicalGammaFeedbackBatchResult evaluateCanonicalGammaFeedbackBatchOptimized(
    const JointEstimateSnapshot& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& task_nominals,
    const CanonicalGammaFeedbackConfig& config,double dt_s,
    double acceleration_variance,BuildRequestFn&& build_request,
    CanonicalGammaFeedbackEvaluationContext& context) {
    CanonicalGammaFeedbackBatchResult result;
    if (context.active) {
        result.reason="recursive_policy_evaluation";
        return result;
    }
    struct ActiveGuard {
        explicit ActiveGuard(CanonicalGammaFeedbackEvaluationContext& value)
            : context(value) { context.active=true; ++context.entries; }
        ~ActiveGuard() { context.active=false; }
        CanonicalGammaFeedbackEvaluationContext& context;
    } guard(context);
    result.work.policy_evaluations=1;
    try {
        const auto row_started=std::chrono::steady_clock::now();
        result.current_rows=buildCanonicalHardRows(build_request(snapshot));
        result.compute_profile.record(
            Task10p11ComputePhase::CurrentCanonicalRowRebuild,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()-row_started).count(),true);
        result.compute_profile.record(
            Task10p11ComputePhase::CanonicalRowRebuild,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()-row_started).count(),true);
        ++result.work.canonical_row_rebuilds;
    } catch (...) {
        result.reason="current_canonical_row_rebuild_failed";
        return result;
    }
    // Task 11b S1-v4: nominal speed saturation - scale each owner's nominal
    // acceleration so the projected velocity |v + dt*n| stays <= the
    // saturation limit (29.9 m/s).  Purely soft-task; hard rows untouched.
    std::map<NodeId,Eigen::Vector2d> effective_nominals;
    if (config.nominal_speed_saturation_mps>0.0) {
        for (NodeId owner:snapshot.mobile_ids) {
            const auto nominal=task_nominals.find(owner);
            if (nominal==task_nominals.end()) continue;
            const Eigen::Index off=4*static_cast<Eigen::Index>(
                std::distance(snapshot.mobile_ids.begin(),
                    std::find(snapshot.mobile_ids.begin(),
                        snapshot.mobile_ids.end(),owner)));
            const Eigen::Vector2d v=snapshot.mean.segment<2>(off);
            const Eigen::Vector2d& n=nominal->second;
            const Eigen::Vector2d w=v+dt_s*n;
            const double wspeed=w.norm();
            effective_nominals.emplace(owner,n);
            const double limit=config.nominal_speed_saturation_mps;
            if (wspeed>limit) {
                // |v + dt*k*n| = limit  ->  k from the quadratic, smaller
                // nonnegative root, clamped to [0,1].
                const double a=dt_s*dt_s*n.squaredNorm();
                const double b=2.0*dt_s*v.dot(n);
                const double c=v.squaredNorm()-limit*limit;
                double k=0.0;
                if (a>1e-12) {
                    const double disc=b*b-4.0*a*c;
                    if (disc>=0.0)
                        k=std::clamp((-b+std::sqrt(disc))/(2.0*a),0.0,1.0);
                }
                effective_nominals[owner]=k*n;
            }
        }
    }
    std::map<NodeId,Eigen::Vector2d> projected_controls;
    // Task 12 Phase 1: margin-aware nominal throttle - scale the coverage
    // nominal by s(min-owner current gamma*) BEFORE stage construction
    // (purely soft-task; hard rows, tau, candidates machinery unchanged).
    double throttle_min_gamma=std::numeric_limits<double>::infinity();
    NodeId throttle_owner=0;
    double throttle_s=1.0;
    for (NodeId owner:snapshot.mobile_ids) {
        const auto nominal=task_nominals.find(owner);
        if (nominal==task_nominals.end()) {
            result.reason="task_nominal_missing";
            return result;
        }
        Eigen::Vector2d effective_nominal=nominal->second;
        if (const auto it=effective_nominals.find(owner);
            it!=effective_nominals.end())
            effective_nominal=it->second;
        if (config.nominal_throttle_enabled ||
            config.throttle_v2_enabled) {
            double signal_gamma=std::numeric_limits<double>::infinity();
            std::string signal_family;
            if (config.throttle_v2_enabled) {
                // v2.2: in-flight margin on the owner's eroding row family
                // (selected-control basis), exact per endpoint.
                const auto family_it=
                    config.throttle_v2_endpoint_family.find(owner);
                if (family_it!=config.throttle_v2_endpoint_family.end()) {
                    signal_family=family_it->second;
                    for (const auto& row:result.current_rows) {
                        if (row.owner!=owner ||
                            !row.participates_in_gamma) continue;
                        if (task11bRowFamilyBase(row.id)!=signal_family)
                            continue;
                        const double m=row.margin(nominal->second);
                        if (m<signal_gamma) signal_gamma=m;
                    }
                }
                if (!std::isfinite(signal_gamma))
                    signal_gamma=std::numeric_limits<double>::infinity();
            } else {
                const auto gamma_now_sol=solveCanonicalGammaStar(
                    result.current_rows,owner,config.acceleration_half_box);
                signal_gamma=gamma_now_sol.valid&&
                    std::isfinite(gamma_now_sol.gamma)?gamma_now_sol.gamma:
                    std::numeric_limits<double>::infinity();
            }
            const double s_now=task12NominalThrottleScale(signal_gamma,
                config.throttle_gamma_th_mps2,
                config.throttle_gamma_floor_mps2);
            if (s_now<result.throttle_s ||
                !std::isfinite(result.throttle_s)) {
                result.throttle_s=s_now;
                result.throttle_min_gamma=signal_gamma;
                result.throttle_limiting_owner=owner;
                result.throttle_active=s_now<1.0;
            }
            result.throttle_v2_endpoint_s[owner]=s_now;
            result.throttle_v2_endpoint_signal[owner]=signal_gamma;
            if (s_now<1.0) {
                result.throttle_v2_active=true;
                effective_nominal*=s_now;
            }
        }

        ++result.work.exact_gamma_solves;
        auto stage=buildCanonicalGammaFeedbackStage(
            result.current_rows,owner,effective_nominal,config,
            &result.compute_profile);
        if (!stage.valid) {
            result.reason=stage.reason;
            return result;
        }
        projected_controls.emplace(owner,stage.task_projection);
        result.stages.emplace(owner,std::move(stage));
    }
    JointEstimateSnapshot baseline_predicted;
    try {
        const auto propagation_started=std::chrono::steady_clock::now();
        baseline_predicted=predictNoMeasurementSnapshot(
            snapshot,projected_controls,dt_s,acceleration_variance);
        result.compute_profile.record(
            Task10p11ComputePhase::EstimatorPropagation,
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()-propagation_started).count(),true);
        ++result.work.estimator_propagations;
    } catch (...) {
        result.reason="prediction_baseline_failed";
        return result;
    }
    Eigen::Matrix<double,4,2> input;
    input<<0.5*dt_s*dt_s,0.0,
           0.0,0.5*dt_s*dt_s,
           dt_s,0.0,
           0.0,dt_s;
    Eigen::Matrix4d transition=Eigen::Matrix4d::Identity();
    transition(0,2)=dt_s;
    transition(1,3)=dt_s;
    // Shared V-a tables (prereg v1.1 section 11.1): current states, specs,
    // and per-owner current rows for the analytic first-order predictor.
    std::map<NodeId,PairwiseSecondOrderState2D> current_states;
    if (config.analytic_first_order_enabled)
        for (std::size_t index=0;index<snapshot.mobile_ids.size();++index) {
            const Eigen::Vector4d state=snapshot.mean.segment<4>(
                static_cast<Eigen::Index>(4*index));
            PairwiseSecondOrderState2D value;
            value.position=Point(state.x(),state.y());
            value.velocity=state.tail<2>();
            value.acceleration=Eigen::Vector2d::Zero();
            current_states.emplace(snapshot.mobile_ids[index],value);
        }
    const auto base_request=config.analytic_first_order_enabled
        ?std::optional<CanonicalHardRowRequest>(build_request(snapshot))
        :std::nullopt;
    for (std::size_t owner_index=0;owner_index<snapshot.mobile_ids.size();
         ++owner_index) {
        const NodeId owner=snapshot.mobile_ids[owner_index];
        const auto& stage=result.stages.at(owner);
        std::vector<CanonicalHardRow> va_owner_rows;
        if (config.analytic_first_order_enabled && base_request.has_value())
            va_owner_rows=buildCanonicalOwnerHardRows(*base_request,owner);
        const auto selected=selectCanonicalGammaFeedback(
            stage,config,[&](const Eigen::Vector2d& candidate) {
                if (config.analytic_first_order_enabled) {
                    CanonicalPredictedGammaScore value;
                    value.valid=true;
                    value.gamma=std::numeric_limits<double>::infinity();
                    const CanonicalHardRow* limiting=nullptr;
                    for (const auto& row:va_owner_rows) {
                        const double margin=row.margin(candidate);
                        if (margin<value.gamma) {
                            value.gamma=margin;
                            limiting=&row;
                        }
                    }
                    if (limiting==nullptr)
                        return CanonicalPredictedGammaScore{};
                    const double rate=task11bAnalyticRowRate(
                        *limiting,owner,candidate,current_states,
                        projected_controls,base_request->collision_spec,
                        base_request->reference_spec,
                        config.acceleration_half_box,dt_s);
                    if (!std::isfinite(rate))
                        return CanonicalPredictedGammaScore{};
                    value.gamma+=dt_s*rate;
                    value.dominant_row=limiting->id;
                    return value;
                }
                try {
                    auto predicted=baseline_predicted;
                    predicted.mean.segment<4>(4*owner_index)=
                        transition*snapshot.mean.segment<4>(4*owner_index)+
                        input*candidate;
                    ++result.work.canonical_row_rebuilds;
                    ++result.work.focused_owner_row_rebuilds;
                    const auto row_started=std::chrono::steady_clock::now();
                    const auto predicted_rows=buildCanonicalOwnerHardRows(
                        build_request(predicted),owner);
                    result.compute_profile.record(
                        Task10p11ComputePhase::PredictedCanonicalRowRebuild,
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now()-row_started).count(),true);
                    result.compute_profile.record(
                        Task10p11ComputePhase::CanonicalRowRebuild,
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now()-row_started).count(),true);
                    ++result.work.exact_gamma_solves;
                    const auto gamma_started=std::chrono::steady_clock::now();
                    const auto gamma=solveCanonicalGammaStar(
                        predicted_rows,owner,config.acceleration_half_box);
                    result.compute_profile.record(
                        Task10p11ComputePhase::PredictedGamma,
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now()-gamma_started).count(),true);
                    if (!gamma.valid || !std::isfinite(gamma.gamma))
                        return CanonicalPredictedGammaScore{};
                    const Eigen::Vector2d witness(gamma.accelX,gamma.accelY);
                    return CanonicalPredictedGammaScore{
                        true,gamma.gamma,dominantCanonicalOwnerRow(
                            predicted_rows,owner,witness)};
                } catch (...) {
                    return CanonicalPredictedGammaScore{};
                }
            });
        if (!selected.valid) {
            result.reason=selected.reason;
            return result;
        }
        result.selected_controls.emplace(owner,selected.selected_control);
        result.selections.emplace(owner,selected);
    }
    result.valid=true;
    result.reason="selected";
    return result;
}

template<class BuildRequestFn>
CanonicalGammaFeedbackBatchResult evaluateCanonicalGammaFeedbackBatch(
    const JointEstimateSnapshot& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& task_nominals,
    const CanonicalGammaFeedbackConfig& config,double dt_s,
    double acceleration_variance,BuildRequestFn&& build_request,
    CanonicalGammaFeedbackEvaluationContext& context) {
    return evaluateCanonicalGammaFeedbackBatchOptimized(
        snapshot,task_nominals,config,dt_s,acceleration_variance,
        std::forward<BuildRequestFn>(build_request),context);
}

}  // namespace gf
