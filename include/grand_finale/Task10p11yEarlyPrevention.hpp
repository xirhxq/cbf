#pragma once

#include "grand_finale/Task10p11xRecoveryCampaign.hpp"
#include "grand_finale/Task10p11yEvidence.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gf {

namespace task10p11y_detail {

constexpr double kTolerance=1.0e-8;
constexpr const char* kPair="reference:2->4";

inline double fullRowResidual(const std::vector<CanonicalHardRow>& rows,
    const CanonicalHardRowRequest& request,
    const std::map<NodeId,Eigen::Vector2d>& controls) {
    const auto problem=buildTask10p11sRows28d(
        rows,request.mobile_ids,true);
    return task10p11w_detail::minimumResidual(problem,
        task10p11sOrderedControls(request.mobile_ids,controls)).first;
}

inline double dynamicLocalResidual(const std::vector<CanonicalHardRow>& rows,
    const Task10p11tPairRows& pair,double transfer,
    const std::map<NodeId,Eigen::Vector2d>& controls) {
    double minimum=std::numeric_limits<double>::infinity();
    for (const auto& row:rows)
        minimum=std::min(minimum,task10p11t_detail::dynamicResidual(
            row,pair,transfer,controls.at(row.owner)));
    return minimum;
}

struct SuccessorAudit {
    bool valid=false;
    bool signed_transfer_interval=false;
    bool all_local_qps=false;
    bool signed_transfer_full_rows=false;
    bool full_pair_28d=false;
    double interval_lower=std::numeric_limits<double>::quiet_NaN();
    double interval_upper=std::numeric_limits<double>::quiet_NaN();
    double interval_slack=-std::numeric_limits<double>::infinity();
    double signed_transfer_full_residual=
        -std::numeric_limits<double>::infinity();
    double full_pair_28d_residual=-std::numeric_limits<double>::infinity();
    std::string reason;
};

inline SuccessorAudit successorAudit(const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& controls) {
    SuccessorAudit result;
    try {
        const auto current=task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        const auto& parameters=snapshot.at("successor_parameters");
        const auto predicted=predictNoMeasurementSnapshot(
            current,controls,parameters.at("dt_s").get<double>(),
            parameters.at("estimator_acceleration_variance").get<double>());
        const auto request=task10p11x_detail::requestAtEstimate(
            snapshot,predicted);
        const auto rows=buildCanonicalHardRows(request);
        const auto local=solveTask10p11tDistributedLocalStep(
            rows,request.mobile_ids,controls,request.acceleration_half_box,
            kPair);
        result.signed_transfer_interval=
            local.pair.shared_interval.feasible;
        result.interval_lower=local.pair.shared_interval.lower;
        result.interval_upper=local.pair.shared_interval.upper;
        if (std::isfinite(result.interval_lower) &&
            std::isfinite(result.interval_upper))
            result.interval_slack=result.interval_upper-result.interval_lower;
        result.all_local_qps=local.feasible;
        if (local.feasible) {
            const auto audit=auditDevelopmentFullPairCertifiedControls(
                rows,request.mobile_ids,request.acceleration_half_box,
                kTolerance,local.controls);
            result.signed_transfer_full_rows=audit.valid;
            result.signed_transfer_full_residual=
                audit.minimum_residual_mps2;
        }
        const auto problem=buildTask10p11sRows28d(
            rows,request.mobile_ids,true);
        const auto ordered=task10p11sOrderedControls(
            request.mobile_ids,controls);
        const auto full=solveTask10p11sQp(problem,ordered);
        result.full_pair_28d=full.feasible &&
            full.minimum_residual>=-kTolerance;
        result.full_pair_28d_residual=full.minimum_residual;
        result.valid=true;
        result.reason=result.signed_transfer_interval &&
            result.all_local_qps && result.signed_transfer_full_rows &&
            result.full_pair_28d
            ?"successor_signed_transfer_and_full_rows_feasible"
            :"successor_gate_not_all_feasible";
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    return result;
}

struct MarginEndpoint {
    bool valid=false;
    std::string reason;
    std::map<NodeId,Eigen::Vector2d> controls;
    double transfer=std::numeric_limits<double>::quiet_NaN();
    double gamma=std::numeric_limits<double>::quiet_NaN();
};

inline MarginEndpoint maximumDynamicLocalMargin(
    const std::vector<CanonicalHardRow>& rows,
    const CanonicalHardRowRequest& request,
    const Task10p11tPairRows& pair) {
    MarginEndpoint result;
#ifdef ENABLE_GUROBI
    try {
        for (NodeId owner:request.mobile_ids) {
            if (owner==pair.first.owner || owner==pair.second.owner) continue;
            const auto gamma=solveCanonicalGammaStar(
                rows,owner,request.acceleration_half_box);
            if (!gamma.valid || !std::isfinite(gamma.gamma)) {
                result.reason="nonpair_local_margin_unavailable:"+
                    std::to_string(owner);
                return result;
            }
            result.controls.emplace(owner,
                Eigen::Vector2d(gamma.accelX,gamma.accelY));
        }
        GRBEnv environment(true);
        task10p11t_detail::configureEnvironment(environment);
        environment.start();
        GRBModel model(environment);
        const double bound=request.acceleration_half_box;
        GRBVar first_x=model.addVar(-bound,bound,0.0,GRB_CONTINUOUS);
        GRBVar first_y=model.addVar(-bound,bound,0.0,GRB_CONTINUOUS);
        GRBVar second_x=model.addVar(-bound,bound,0.0,GRB_CONTINUOUS);
        GRBVar second_y=model.addVar(-bound,bound,0.0,GRB_CONTINUOUS);
        GRBVar transfer=model.addVar(
            -GRB_INFINITY,GRB_INFINITY,0.0,GRB_CONTINUOUS);
        GRBVar gamma=model.addVar(
            -GRB_INFINITY,GRB_INFINITY,0.0,GRB_CONTINUOUS);
        for (const auto& row:rows) {
            if (row.owner!=pair.first.owner &&
                row.owner!=pair.second.owner) continue;
            const bool first=row.owner==pair.first.owner;
            auto expression=task10p11t_detail::dynamicExpression(
                row,pair,first?first_x:second_x,
                first?first_y:second_y,transfer);
            if (row.participates_in_gamma)
                model.addConstr(expression>=gamma);
            else
                model.addConstr(expression>=0.0);
        }
        GRBLinExpr objective=gamma;
        model.setObjective(objective,GRB_MAXIMIZE);
        model.optimize();
        if (model.get(GRB_IntAttr_Status)!=GRB_OPTIMAL) {
            result.reason="pair_dynamic_margin_not_optimal";
            return result;
        }
        result.controls[pair.first.owner]={
            first_x.get(GRB_DoubleAttr_X),first_y.get(GRB_DoubleAttr_X)};
        result.controls[pair.second.owner]={
            second_x.get(GRB_DoubleAttr_X),second_y.get(GRB_DoubleAttr_X)};
        result.transfer=transfer.get(GRB_DoubleAttr_X);
        result.gamma=gamma.get(GRB_DoubleAttr_X);
        result.valid=result.controls.size()==request.mobile_ids.size() &&
            std::isfinite(result.transfer) && std::isfinite(result.gamma);
        result.reason=result.valid?"maximum_dynamic_local_margin":
            "maximum_dynamic_local_margin_nonfinite";
    } catch (const GRBException& error) {
        result.reason="gurobi_error_"+std::to_string(error.getErrorCode());
    }
#else
    (void)rows;(void)request;(void)pair;
    result.reason="gurobi_not_enabled";
#endif
    return result;
}

struct Candidate {
    std::size_t index=0;
    double alpha=0.0;
    bool current_feasible=false;
    std::map<NodeId,Eigen::Vector2d> controls;
    double transfer=std::numeric_limits<double>::quiet_NaN();
    double coverage_deviation=std::numeric_limits<double>::quiet_NaN();
    double current_local_residual=-std::numeric_limits<double>::infinity();
    double current_full_residual=-std::numeric_limits<double>::infinity();
    SuccessorAudit successor;
};

struct Decision {
    bool valid=false;
    bool preventive_trigger=false;
    bool preventive_intervention=false;
    bool baseline_fixed_half=false;
    std::string reason;
    std::string selection;
    Task10p11tDynamicPairResult current_pair;
    std::map<NodeId,Eigen::Vector2d> baseline_controls;
    double baseline_transfer=std::numeric_limits<double>::quiet_NaN();
    SuccessorAudit baseline_successor;
    std::vector<Candidate> candidates;
    std::size_t selected_index=0;
};

inline Decision decideWithPreparedBaseline(const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& baseline_controls,
    double baseline_transfer,bool baseline_fixed_half) {
    Decision result;
    try {
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto rows=buildCanonicalHardRows(request);
        const auto nominal=task10p11s_capture_detail::nominalFromJson(
            snapshot.at("nominal_controls"));
        result.current_pair=solveTask10p11tDynamicPair(
            rows,request.mobile_ids,nominal,request.acceleration_half_box,kPair);
        if (!result.current_pair.valid ||
            !result.current_pair.shared_interval.feasible) {
            result.reason="current_signed_transfer_interval_empty";
            return result;
        }
        if (baseline_controls.size()!=request.mobile_ids.size() ||
            !std::isfinite(baseline_transfer)) {
            result.reason="prepared_baseline_control_identity_invalid";
            return result;
        }
        result.baseline_fixed_half=baseline_fixed_half;
        result.baseline_controls=baseline_controls;
        result.baseline_transfer=baseline_transfer;
        result.baseline_successor=successorAudit(
            snapshot,result.baseline_controls);
        result.preventive_trigger=
            !result.baseline_successor.signed_transfer_interval;
        if (!result.preventive_trigger) {
            result.valid=result.baseline_successor.valid &&
                result.baseline_successor.all_local_qps &&
                result.baseline_successor.signed_transfer_full_rows &&
                result.baseline_successor.full_pair_28d;
            result.selection="coverage_first";
            result.reason=result.valid?"coverage_successor_feasible":
                "coverage_successor_hard_gate_failed";
            return result;
        }
        const auto endpoint=maximumDynamicLocalMargin(
            rows,request,result.current_pair.pair);
        if (!endpoint.valid) {
            result.reason="maximum_local_margin_endpoint_failed:"+
                endpoint.reason;
            return result;
        }
        const auto baseline_ordered=task10p11sOrderedControls(
            request.mobile_ids,result.baseline_controls);
        for (std::size_t index=0;index<=8;++index) {
            Candidate candidate;
            candidate.index=index;
            candidate.alpha=static_cast<double>(index)/8.0;
            candidate.transfer=(1.0-candidate.alpha)*result.baseline_transfer+
                candidate.alpha*endpoint.transfer;
            for (NodeId owner:request.mobile_ids)
                candidate.controls[owner]=(1.0-candidate.alpha)*
                    result.baseline_controls.at(owner)+candidate.alpha*
                    endpoint.controls.at(owner);
            candidate.coverage_deviation=(task10p11sOrderedControls(
                request.mobile_ids,candidate.controls)-baseline_ordered).norm();
            candidate.current_local_residual=dynamicLocalResidual(
                rows,result.current_pair.pair,candidate.transfer,
                candidate.controls);
            candidate.current_full_residual=fullRowResidual(
                rows,request,candidate.controls);
            candidate.current_feasible=
                candidate.current_local_residual>=-kTolerance &&
                candidate.current_full_residual>=-kTolerance;
            if (candidate.current_feasible)
                candidate.successor=successorAudit(
                    snapshot,candidate.controls);
            result.candidates.push_back(std::move(candidate));
        }
        std::optional<std::size_t> restoring;
        for (std::size_t index=0;index<result.candidates.size();++index) {
            const auto& candidate=result.candidates[index];
            const bool restores=candidate.current_feasible &&
                candidate.successor.valid &&
                candidate.successor.signed_transfer_interval &&
                candidate.successor.all_local_qps &&
                candidate.successor.signed_transfer_full_rows &&
                candidate.successor.full_pair_28d;
            if (!restores) continue;
            if (!restoring.has_value() ||
                candidate.coverage_deviation<
                    result.candidates[*restoring].coverage_deviation-1e-12)
                restoring=index;
        }
        if (restoring.has_value()) {
            result.selected_index=*restoring;
            result.selection="least_coverage_deviation_restoring_local";
        } else {
            std::optional<std::size_t> maximum;
            for (std::size_t index=0;index<result.candidates.size();++index) {
                const auto& candidate=result.candidates[index];
                if (!candidate.current_feasible ||
                    !candidate.successor.valid ||
                    !candidate.successor.full_pair_28d) continue;
                if (!maximum.has_value() || candidate.successor.interval_slack>
                    result.candidates[*maximum].successor.interval_slack+1e-12)
                    maximum=index;
            }
            if (!maximum.has_value()) {
                result.reason="no_current_and_successor_full_row_candidate";
                return result;
            }
            result.selected_index=*maximum;
            result.selection="maximum_predicted_local_margin";
        }
        result.preventive_intervention=result.selected_index!=0;
        result.valid=result.candidates.at(result.selected_index).current_feasible;
        result.reason=result.valid?"preventive_candidate_selected":
            "selected_candidate_current_infeasible";
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    return result;
}

inline Decision decide(const nlohmann::json& snapshot) {
    try {
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto rows=buildCanonicalHardRows(request);
        const auto nominal=task10p11s_capture_detail::nominalFromJson(
            snapshot.at("nominal_controls"));
        const auto fixed=task10p11x_detail::selectedDistributedControls(
            snapshot,14.0);
        if (fixed.has_value())
            return decideWithPreparedBaseline(snapshot,*fixed,0.0,true);
        const auto dynamic=solveTask10p11tDistributedLocalStep(
            rows,request.mobile_ids,nominal,
            request.acceleration_half_box,kPair);
        if (!dynamic.feasible) {
            Decision result;
            result.reason=dynamic.reason;
            return result;
        }
        return decideWithPreparedBaseline(snapshot,dynamic.controls,
            dynamic.pair.selected_transfer_mps2,false);
    } catch (const std::exception& error) {
        Decision result;
        result.reason=error.what();
        return result;
    }
}

inline nlohmann::json intervalJson(double lower,double upper,bool feasible) {
    return {{"feasible",feasible},
        {"lower_mps2",task10p11yMetric(
            std::isfinite(lower)?std::optional<double>(lower):std::nullopt,
            std::isfinite(lower)?"observed":"not_applicable")},
        {"upper_mps2",task10p11yMetric(
            std::isfinite(upper)?std::optional<double>(upper):std::nullopt,
            std::isfinite(upper)?"observed":"not_applicable")}};
}

inline nlohmann::json successorJson(const SuccessorAudit& value) {
    return {{"valid",value.valid},{"reason",value.reason},
        {"signed_transfer_interval",intervalJson(
            value.interval_lower,value.interval_upper,
            value.signed_transfer_interval)},
        {"interval_slack_mps2",task10p11yMetric(
            std::isfinite(value.interval_slack)
                ?std::optional<double>(value.interval_slack):std::nullopt,
            std::isfinite(value.interval_slack)
                ?"observed":"not_applicable_interval_unavailable")},
        {"all_local_qps_feasible",value.all_local_qps},
        {"signed_transfer_full_rows_feasible",
            value.signed_transfer_full_rows},
        {"signed_transfer_full_residual_mps2",task10p11yMetric(
            std::isfinite(value.signed_transfer_full_residual)
                ?std::optional<double>(value.signed_transfer_full_residual)
                :std::nullopt,"observed_or_not_applicable")},
        {"full_pair_28d_feasible",value.full_pair_28d},
        {"full_pair_28d_residual_mps2",task10p11yMetric(
            std::isfinite(value.full_pair_28d_residual)
                ?std::optional<double>(value.full_pair_28d_residual)
                :std::nullopt,"observed_or_not_applicable")},
        {"exact_zoh_one_step",true},
        {"recursive_feasibility_claimed",false}};
}

inline nlohmann::json decisionJson(const Decision& value) {
    nlohmann::json candidates=nlohmann::json::array();
    for (const auto& candidate:value.candidates)
        candidates.push_back({{"index",candidate.index},
            {"alpha",candidate.alpha},{"current_feasible",
                candidate.current_feasible},
            {"transfer_mps2",task10p11yMetric(
                std::isfinite(candidate.transfer)
                    ?std::optional<double>(candidate.transfer):std::nullopt,
                "observed_or_not_applicable")},
            {"coverage_control_deviation_l2_mps2",task10p11yMetric(
                std::isfinite(candidate.coverage_deviation)
                    ?std::optional<double>(candidate.coverage_deviation)
                    :std::nullopt,"observed_or_not_applicable")},
            {"current_local_residual_mps2",task10p11yMetric(
                std::isfinite(candidate.current_local_residual)
                    ?std::optional<double>(candidate.current_local_residual)
                    :std::nullopt,"observed_or_not_applicable")},
            {"current_full_row_residual_mps2",task10p11yMetric(
                std::isfinite(candidate.current_full_residual)
                    ?std::optional<double>(candidate.current_full_residual)
                    :std::nullopt,"observed_or_not_applicable")},
            {"successor",successorJson(candidate.successor)}});
    return {{"valid",value.valid},{"reason",value.reason},
        {"selection",value.selection},
        {"preventive_trigger",value.preventive_trigger},
        {"preventive_intervention",value.preventive_intervention},
        {"baseline_fixed_half",value.baseline_fixed_half},
        {"current_signed_transfer_interval",intervalJson(
            value.current_pair.shared_interval.lower,
            value.current_pair.shared_interval.upper,
            value.current_pair.shared_interval.feasible)},
        {"baseline_transfer_mps2",task10p11yMetric(
            std::isfinite(value.baseline_transfer)
                ?std::optional<double>(value.baseline_transfer):std::nullopt,
            "observed_or_not_applicable")},
        {"baseline_successor",successorJson(value.baseline_successor)},
        {"selected_index",value.selected_index},
        {"candidates",std::move(candidates)}};
}

}  // namespace task10p11y_detail

}  // namespace gf
