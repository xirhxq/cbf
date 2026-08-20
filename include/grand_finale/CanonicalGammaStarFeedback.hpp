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
};

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
    for (NodeId owner:snapshot.mobile_ids) {
        const auto& stage=result.stages.at(owner);
        const auto selected=selectCanonicalGammaFeedback(
            stage,config,[&](const Eigen::Vector2d& candidate) {
                try {
                    auto surrogate_controls=projected_controls;
                    surrogate_controls.at(owner)=candidate;
                    ++result.work.estimator_propagations;
                    const auto predicted=predictNoMeasurementSnapshot(
                        snapshot,surrogate_controls,dt_s,
                        acceleration_variance);
                    ++result.work.canonical_row_rebuilds;
                    const auto predicted_rows=buildCanonicalHardRows(
                        build_request(predicted));
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
    for (std::size_t owner_index=0;owner_index<snapshot.mobile_ids.size();
         ++owner_index) {
        const NodeId owner=snapshot.mobile_ids[owner_index];
        const auto& stage=result.stages.at(owner);
        const auto selected=selectCanonicalGammaFeedback(
            stage,config,[&](const Eigen::Vector2d& candidate) {
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
