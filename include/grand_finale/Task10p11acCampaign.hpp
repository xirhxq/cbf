#pragma once

#include "grand_finale/Task10p11rFixedBaseline.hpp"

#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

enum class Task10p11acFeedbackBranch {
    NominalRetention,
    LeastInterventionTauAttained,
    TauUnattainedMaximumMarginFallback,
    InvalidPredictionProjectionFallback
};

inline std::string task10p11acFeedbackBranchName(
    Task10p11acFeedbackBranch branch) {
    switch (branch) {
        case Task10p11acFeedbackBranch::NominalRetention:
            return "nominal_retention";
        case Task10p11acFeedbackBranch::LeastInterventionTauAttained:
            return "least_intervention_tau_attained";
        case Task10p11acFeedbackBranch::TauUnattainedMaximumMarginFallback:
            return "tau_unattained_maximum_margin_fallback";
        case Task10p11acFeedbackBranch::InvalidPredictionProjectionFallback:
            return "invalid_prediction_projection_fallback";
    }
    throw std::logic_error("unknown Task 10.11ac feedback branch");
}

inline Task10p11acFeedbackBranch task10p11acClassifyFeedback(
    const GrandFinaleGammaFeedbackDiagnostic& diagnostic) {
    if (diagnostic.fallback_reason=="invalid_prediction_use_current_projection")
        return Task10p11acFeedbackBranch::InvalidPredictionProjectionFallback;
    if (!diagnostic.tau_attainment_valid)
        throw std::invalid_argument(
            "Task 10.11ac requires valid tau-attainment diagnostics");
    if (!diagnostic.tau_attained) {
        if (diagnostic.fallback_reason!=
            "tau_unattained_maximum_predicted_margin")
            throw std::invalid_argument(
                "Task 10.11ac tau-unattained diagnostic lacks fallback reason");
        return Task10p11acFeedbackBranch::
            TauUnattainedMaximumMarginFallback;
    }
    if (!diagnostic.fallback_reason.empty())
        throw std::invalid_argument(
            "Task 10.11ac tau-attained diagnostic has fallback reason");
    return diagnostic.selected_candidate_index==0
        ?Task10p11acFeedbackBranch::NominalRetention
        :Task10p11acFeedbackBranch::LeastInterventionTauAttained;
}

inline nlohmann::json task10p11acVectorJson(const Eigen::Vector2d& value) {
    if (!value.allFinite())
        throw std::invalid_argument("Task 10.11ac vector is non-finite");
    return nlohmann::json::array({value.x(),value.y()});
}

inline nlohmann::json task10p11acDiagnosticMetricJson(
    double value,bool valid,const std::string& invalid_reason) {
    if (valid) {
        if (!std::isfinite(value))
            throw std::invalid_argument(
                "Task 10.11ac valid diagnostic metric is non-finite");
        return {{"status","valid"},{"value",value},{"reason","observed"}};
    }
    return {{"status","invalid"},{"value",nullptr},
        {"reason",invalid_reason}};
}

inline nlohmann::json task10p11acOwnerDecisionJson(
    NodeId owner,const GrandFinaleGammaFeedbackDiagnostic& diagnostic,
    const Eigen::Vector2d& coverage_nominal,
    const Eigen::Vector2d& applied_control,std::size_t candidate_count) {
    if (candidate_count<2 ||
        diagnostic.selected_candidate_index>=candidate_count ||
        !std::isfinite(diagnostic.current_gamma))
        throw std::invalid_argument(
            "Task 10.11ac owner diagnostic is non-finite or out of range");
    const auto branch=task10p11acClassifyFeedback(diagnostic);
    const bool prediction_valid=branch!=Task10p11acFeedbackBranch::
        InvalidPredictionProjectionFallback;
    const double alpha=static_cast<double>(
        diagnostic.selected_candidate_index)/
        static_cast<double>(candidate_count-1);
    const double feedback_deviation=(
        diagnostic.selected_nominal-coverage_nominal).norm();
    const double final_deviation=(applied_control-coverage_nominal).norm();
    if (!std::isfinite(feedback_deviation) || !std::isfinite(final_deviation))
        throw std::invalid_argument(
            "Task 10.11ac owner deviation is non-finite");
    return {{"owner",owner},{"branch",task10p11acFeedbackBranchName(branch)},
        {"current_gamma_mps2",task10p11acDiagnosticMetricJson(
            diagnostic.current_gamma,true,{})},
        {"nominal_predicted_gamma_mps2",task10p11acDiagnosticMetricJson(
            diagnostic.nominal_predicted_gamma,prediction_valid,
            diagnostic.fallback_reason)},
        {"maximum_margin_candidate_predicted_gamma_mps2",
            task10p11acDiagnosticMetricJson(
                diagnostic.maximum_margin_candidate_predicted_gamma,
                prediction_valid,diagnostic.fallback_reason)},
        {"selected_predicted_gamma_mps2",
            task10p11acDiagnosticMetricJson(
                diagnostic.selected_predicted_gamma,prediction_valid,
                diagnostic.fallback_reason)},
        {"tau_attainment_valid",diagnostic.tau_attainment_valid},
        {"tau_attained",diagnostic.tau_attained},
        {"selected_candidate_index",diagnostic.selected_candidate_index},
        {"selected_alpha",alpha},
        {"fallback_reason",diagnostic.fallback_reason},
        {"coverage_nominal",task10p11acVectorJson(coverage_nominal)},
        {"current_hard_projection",task10p11acVectorJson(
            diagnostic.current_hard_projection)},
        {"maximum_margin_endpoint",task10p11acVectorJson(
            diagnostic.maximum_margin_control)},
        {"selected_nominal",task10p11acVectorJson(
            diagnostic.selected_nominal)},
        {"final_applied_control",task10p11acVectorJson(applied_control)},
        {"gamma_feedback_deviation_l2_mps2",feedback_deviation},
        {"final_control_deviation_l2_mps2",final_deviation},
        {"dominant_row",diagnostic.dominant_row}};
}

struct Task10p11acCycleStatistics {
    std::size_t advanced_cycles=0;
    std::map<std::string,std::size_t> owner_branch_counts;
    std::map<std::size_t,std::size_t> owner_alpha_counts;
    std::map<std::string,double> branch_feedback_deviation;
    std::map<std::string,double> branch_final_deviation;

    void observe(bool advanced,const nlohmann::json& owner_decisions) {
        if (!advanced) return;
        if (!owner_decisions.is_array() || owner_decisions.size()!=14)
            throw std::invalid_argument(
                "Task 10.11ac advanced cycle requires fourteen owner decisions");
        ++advanced_cycles;
        for (const auto& decision:owner_decisions) {
            const auto branch=decision.at("branch").get<std::string>();
            const auto index=decision.at("selected_candidate_index").
                get<std::size_t>();
            const double feedback=decision.at(
                "gamma_feedback_deviation_l2_mps2").get<double>();
            const double final=decision.at(
                "final_control_deviation_l2_mps2").get<double>();
            if (!std::isfinite(feedback) || !std::isfinite(final))
                throw std::invalid_argument(
                    "Task 10.11ac statistics reject non-finite deviation");
            ++owner_branch_counts[branch];
            ++owner_alpha_counts[index];
            branch_feedback_deviation[branch]+=feedback;
            branch_final_deviation[branch]+=final;
        }
    }
};

struct Task10p11acInitialization {
    std::string id;
    std::vector<Eigen::Vector2d> positions;
    std::vector<Eigen::Vector2d> velocities;
    std::string record_sha256;
};

inline Task10p11acInitialization task10p11acInitializationFromManifest(
    const nlohmann::json& manifest,const std::string& id) {
    const auto& value=manifest.at("initializations").at(id);
    Task10p11acInitialization result;
    result.id=id;
    result.record_sha256=value.at("record_sha256").get<std::string>();
    const auto& states=value.at("mobile_states");
    if (states.size()!=14)
        throw std::invalid_argument(
            "Task 10.11ac initialization must contain fourteen mobiles");
    NodeId expected=1;
    for (const auto& state:states) {
        if (state.at("owner").get<NodeId>()!=expected++)
            throw std::invalid_argument(
                "Task 10.11ac initialization owner order mismatch");
        const auto& p=state.at("position_m");
        const auto& v=state.at("velocity_mps");
        Eigen::Vector2d position(p.at(0).get<double>(),p.at(1).get<double>());
        Eigen::Vector2d velocity(v.at(0).get<double>(),v.at(1).get<double>());
        if (!position.allFinite() || !velocity.allFinite())
            throw std::invalid_argument(
                "Task 10.11ac initialization contains non-finite state");
        result.positions.push_back(position);
        result.velocities.push_back(velocity);
    }
    return result;
}

inline nlohmann::json task10p11acSwarmSettings(
    const Task10p10Scenario& scenario,
    const std::vector<Eigen::Vector2d>& velocities) {
    if (velocities.size()!=scenario.mobile_ids.size())
        throw std::invalid_argument(
            "Task 10.11ac velocity count does not match mobile count");
    auto settings=task10p11pSwarmSettings(scenario,SolverProfile::Gurobi);
    settings["initial"]["velocity"]["values"]=nlohmann::json::array();
    for (const auto& velocity:velocities) {
        if (!velocity.allFinite())
            throw std::invalid_argument(
                "Task 10.11ac velocity is non-finite");
        settings["initial"]["velocity"]["values"].push_back(
            {velocity.x(),velocity.y()});
    }
    return settings;
}

inline nlohmann::json task10p11acBaseIdentityJson(
    const Task10p11rFixedBaselineFixture& fixture) {
    const auto& c=fixture.adapter.config();
    nlohmann::json polygon=nlohmann::json::array();
    for (const auto& point:c.boundary.explicit_flight_polygon)
        polygon.push_back(task10p11acVectorJson(point));
    nlohmann::json topology=nlohmann::json::array();
    for (const auto& edge:fixture.frozen_topology)
        topology.push_back({{"reference",edge.reference},{"owner",edge.owner}});
    nlohmann::json branches=nlohmann::json::array();
    for (const auto& branch:task10p11rAuthorityContract().branches) {
        branches.push_back({{"members",branch.members},{"leader",branch.leader},
            {"coverage_origin",task10p11acVectorJson(branch.coverage_origin)},
            {"rotation_rad",branch.rotation_rad},
            {"preferred_fixed_roots",branch.preferred_fixed_roots}});
    }
    return {{"protocol","task10p11ac-base-identity-v1"},
        {"adapter_config_without_tau",{
            {"solver_profile",static_cast<int>(c.solver_profile)},
            {"dt_s",c.dt_s},{"minimum_dwell_s",c.minimum_dwell_s},
            {"acceleration_half_box",c.acceleration_half_box},
            {"speed_limit_mps",c.speed_limit_mps},
            {"speed_cbf_gain",c.speed_cbf_gain},
            {"plant_speed_facet_count",c.plant_speed_facet_count},
            {"maximum_yaw_rate_radps",c.maximum_yaw_rate_radps},
            {"position_gain",c.position_gain},{"velocity_gain",c.velocity_gain},
            {"estimator_acceleration_variance",c.estimator_acceleration_variance},
            {"initial_position_variance_m2",c.initial_position_variance_m2},
            {"initial_velocity_variance_m2",c.initial_velocity_variance_m2},
            {"certified_error_bound_m",c.certified_error_bound_m},
            {"certified_shadow_single_position_support_m",
                c.certified_shadow_single_position_support_m},
            {"certified_shadow_single_velocity_support_mps",
                c.certified_shadow_single_velocity_support_mps},
            {"certified_shadow_relative_position_support_m",
                c.certified_shadow_relative_position_support_m},
            {"certified_shadow_relative_velocity_support_mps",
                c.certified_shadow_relative_velocity_support_mps},
            {"maximum_accepted_range_innovation_m",
                c.maximum_accepted_range_innovation_m},
            {"range_random_seed",c.range_random_seed},
            {"range_noise_std_m",c.range_noise_std_m},
            {"range_dropout_probability",c.range_dropout_probability},
            {"sensor_radius_m",c.sensor_radius_m},
            {"coverage_footprint_kind",static_cast<int>(
                c.coverage_footprint_kind)},
            {"coverage_inner_radius_m",c.coverage_inner_radius_m},
            {"coverage_half_angle_rad",c.coverage_half_angle_rad},
            {"reference_distance_m",c.reference_distance_m},
            {"add_reference_distance_m",c.add_reference_distance_m},
            {"reference_uncertainty_m",c.reference_uncertainty_m},
            {"uncertainty_sigma",c.uncertainty_sigma},
            {"maximum_reference_position_eigenvalue_m2",
                c.maximum_reference_position_eigenvalue_m2},
            {"maximum_posterior_eigenvalue_m2",
                c.maximum_posterior_eigenvalue_m2},
            {"maximum_range_aoi_s",c.maximum_range_aoi_s},
            {"minimum_range_quality",c.minimum_range_quality},
            {"collision_distance_m",c.collision_distance_m},
            {"collision_lambda1",c.collision_lambda1},
            {"collision_lambda2",c.collision_lambda2},
            {"residual_tolerance",c.residual_tolerance},
            {"qp_oracle_tolerance",c.qp_oracle_tolerance},
            {"boundary_policy",static_cast<int>(c.boundary.policy)},
            {"flight_polygon_source",static_cast<int>(
                c.boundary.flight_polygon_source)},
            {"explicit_flight_polygon",polygon},
            {"boundary_soft_slack_weight",c.boundary.soft_slack_weight},
            {"gamma_feedback_selection",static_cast<int>(
                c.gamma_feedback_selection)},
            {"gamma_feedback_homotopy_segments",
                c.gamma_feedback_homotopy_segments},
            {"gamma_feedback_tolerance",c.gamma_feedback_tolerance},
            {"progress_max_projection_norm",
                c.progress_compatibility.max_projection_norm},
            {"progress_min_direction_ratio",
                c.progress_compatibility.min_direction_ratio},
            {"progress_comparison_tolerance",
                c.progress_compatibility.comparison_tolerance},
            {"progress_zero_nominal_is_compatible",
                c.progress_compatibility.zero_nominal_is_compatible}}},
        {"world",{{"scenario_id",fixture.scenario.id},
            {"width_m",fixture.scenario.width_m},
            {"height_m",fixture.scenario.height_m},
            {"grid_spacing_m",fixture.settings.at("world").at("spacing")},
            {"searching",fixture.settings.at("searching")},
            {"initial_yaw_deg",fixture.settings.at("initial").at("yawDeg")}}},
        {"fixed_topology",topology},{"authority_branches",branches},
        {"candidate_count",c.gamma_feedback_homotopy_segments+1},
        {"exact_zoh",true},{"fixed_buav",true},
        {"hard_gates",{{"collision_distance_m",10.0},
            {"reference_distance_m",850.0},
            {"strict_reference_distance_m",849.0},
            {"plant_speed_limit_mps",30.0},
            {"minimum_effective_references",2},
            {"minimum_robust_fim",1.0e-6}}}};
}

}  // namespace gf
