#pragma once

#include "grand_finale/CentralizedEkfOracle.hpp"
#include "grand_finale/ProgressCompatibility.hpp"

#include <algorithm>
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
    std::string dominant_row;
    std::string fallback_reason;
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
    const CanonicalGammaFeedbackConfig& config) {
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
    const auto current=solveCanonicalGammaStar(
        rows,owner,config.acceleration_half_box);
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
    const auto projected=evaluateProgressCompatibility(
        rows,owner,task_nominal,config.acceleration_half_box,
        {std::numeric_limits<double>::max(),0.0,
         config.feasibility_tolerance,true});
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
        const auto threshold=std::find_if(
            scores.begin(),scores.end(),[&](const auto& value) {
                return value.gamma+config.feasibility_tolerance>=
                    *config.predictive_tau_mps2;
            });
        if (threshold==scores.end()) {
            selected=best_index;
            result.fallback_reason="tau_unattained_maximum_predicted_margin";
        } else {
            selected=static_cast<std::size_t>(
                std::distance(scores.begin(),threshold));
        }
    }
    result.valid=true;
    result.reason="selected";
    result.selected_control=stage.candidates[selected];
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

}  // namespace gf
