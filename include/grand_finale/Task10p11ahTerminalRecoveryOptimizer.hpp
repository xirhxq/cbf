#pragma once

#include "grand_finale/Task10p11ahEarlyH2Recovery.hpp"

#include <chrono>
#include <functional>

namespace gf {

struct Task10p11ahPosthocOptimizerProtocol {
    bool post_hoc_development_extension=true;
    std::set<NodeId> component{2,9};
    std::size_t horizon_steps=2;
    double tau_mps2=22.0;
    double feasibility_tolerance_mps2=1.0e-8;
    bool fixed_topology=true;
    std::string external_controls=
        "same_frame_canonical_distributed_controls";
    std::string optimizer="deterministic_continuous_pattern_search";
    bool component_expansion_allowed=false;
    bool depth_expansion_allowed=false;
};

inline Task10p11ahPosthocOptimizerProtocol
task10p11ahPosthocOptimizerProtocol() { return {}; }

struct Task10p11ahPlanScore {
    bool full_rows_feasible=false;
    double minimum_full_row_residual_mps2=
        -std::numeric_limits<double>::infinity();
    bool terminal_recovered=false;
    double terminal_recovery_margin_mps2=
        -std::numeric_limits<double>::infinity();
    double cumulative_coverage_deviation_l2_mps2=
        std::numeric_limits<double>::infinity();
};

inline bool task10p11ahBetterPlan(const Task10p11ahPlanScore& candidate,
                                  const Task10p11ahPlanScore& incumbent) {
    constexpr double kNumericalOrderingTolerance=1.0e-10;
    if (candidate.full_rows_feasible!=incumbent.full_rows_feasible)
        return candidate.full_rows_feasible;
    if (!candidate.full_rows_feasible) {
        return candidate.minimum_full_row_residual_mps2>
            incumbent.minimum_full_row_residual_mps2+
                kNumericalOrderingTolerance;
    }
    if (candidate.terminal_recovered!=incumbent.terminal_recovered)
        return candidate.terminal_recovered;
    if (!candidate.terminal_recovered) {
        return candidate.terminal_recovery_margin_mps2>
            incumbent.terminal_recovery_margin_mps2+
                kNumericalOrderingTolerance;
    }
    return candidate.cumulative_coverage_deviation_l2_mps2+
        kNumericalOrderingTolerance<
            incumbent.cumulative_coverage_deviation_l2_mps2;
}

enum class Task10p11ahOptimizerClassification {
    Witness,
    StrictlyInfeasible,
    Undetermined,
    LocalNoWitness
};

inline std::string task10p11ahOptimizerClassificationName(
    Task10p11ahOptimizerClassification value) {
    switch (value) {
    case Task10p11ahOptimizerClassification::Witness:
        return "terminal_recovery_witness";
    case Task10p11ahOptimizerClassification::StrictlyInfeasible:
        return "strictly_infeasible";
    case Task10p11ahOptimizerClassification::Undetermined:
        return "undetermined";
    case Task10p11ahOptimizerClassification::LocalNoWitness:
        return "local_optimizer_no_witness";
    }
    return "undetermined";
}

struct Task10p11ahComponentPlan {
    Eigen::Vector2d owner2_u0=Eigen::Vector2d::Zero();
    Eigen::Vector2d owner9_u0=Eigen::Vector2d::Zero();
    Eigen::Vector2d owner2_u1=Eigen::Vector2d::Zero();
    Eigen::Vector2d owner9_u1=Eigen::Vector2d::Zero();
};

struct Task10p11ahPlanEvaluation {
    bool valid=false;
    std::string reason;
    Task10p11ahOptimizerClassification classification=
        Task10p11ahOptimizerClassification::Undetermined;
    Task10p11ahPlanScore score;
    std::map<NodeId,Eigen::Vector2d> distributed_u0;
    std::map<NodeId,Eigen::Vector2d> selected_u0;
    std::map<NodeId,Eigen::Vector2d> distributed_u1;
    std::map<NodeId,Eigen::Vector2d> selected_u1;
    std::map<NodeId,Eigen::Vector2d> distributed_u2;
    double x0_realized_minimum_residual_mps2=
        -std::numeric_limits<double>::infinity();
    double x1_exact_minimum_residual_mps2=
        -std::numeric_limits<double>::infinity();
    double x1_realized_minimum_residual_mps2=
        -std::numeric_limits<double>::infinity();
    double x2_exact_minimum_residual_mps2=
        -std::numeric_limits<double>::infinity();
    double x2_realized_minimum_residual_mps2=
        -std::numeric_limits<double>::infinity();
    double terminal_owner_local_gamma_mps2=
        -std::numeric_limits<double>::infinity();
    double terminal_signed_transfer_width_mps2=
        -std::numeric_limits<double>::infinity();
    double terminal_native_successor_gamma_mps2=
        -std::numeric_limits<double>::infinity();
    double terminal_native_successor_residual_mps2=
        -std::numeric_limits<double>::infinity();
    bool terminal_signed_transfer_feasible=false;
    bool terminal_native_current_feasible=false;
    bool terminal_native_successor_feasible=false;
    bool terminal_h1_witness=false;
    bool terminal_h2_witness=false;
    std::size_t full_row_count_x0=0;
    std::size_t full_row_count_x1=0;
    std::size_t full_row_count_x2=0;
    nlohmann::json x0_estimator=nlohmann::json::object();
    nlohmann::json x1_estimator=nlohmann::json::object();
    nlohmann::json x2_estimator=nlohmann::json::object();
};

namespace task10p11ah_optimizer_detail {

constexpr double kTolerance=1.0e-8;
const std::set<NodeId> kComponent{2,9};

inline std::map<NodeId,Eigen::Vector2d> projectedComponentControls(
    const CanonicalHardRowRequest& request,
    const std::map<NodeId,Eigen::Vector2d>& distributed,
    const Eigen::Vector2d& owner2,const Eigen::Vector2d& owner9,
    double* minimum_residual=nullptr) {
    const auto problem=buildTask10p11sRows28d(
        buildCanonicalHardRows(request),request.mobile_ids,true);
    const Eigen::VectorXd frozen=task10p11sOrderedControls(
        request.mobile_ids,distributed);
    Eigen::VectorXd desired=frozen;
    const auto ownerOffset=[&](NodeId owner) {
        const auto found=std::find(request.mobile_ids.begin(),
                                   request.mobile_ids.end(),owner);
        if (found==request.mobile_ids.end())
            throw std::runtime_error("component owner absent");
        return 2*std::distance(request.mobile_ids.begin(),found);
    };
    const Eigen::Index offset2=ownerOffset(2);
    const Eigen::Index offset9=ownerOffset(9);
    desired.segment<2>(offset2)=owner2;
    desired.segment<2>(offset9)=owner9;
    const auto projection=task10p11w_detail::solveRestricted(
        problem,desired,frozen,kComponent,false);
    if (!projection.feasible || projection.controls.size()!=frozen.size())
        throw std::runtime_error("component_current_projection_failed:"+
                                 projection.status);
    const double independent=task10p11af_detail::independentMinimumResidual(
        problem,projection.controls);
    if (minimum_residual!=nullptr) *minimum_residual=independent;
    if (independent<-kTolerance)
        throw std::runtime_error("component_current_projection_infeasible");
    const auto result=task10p11sControlMap(
        request.mobile_ids,projection.controls);
    if (!task10p11ahOnlyComponentDiffers(
            distributed,result,kComponent,1.0e-12))
        throw std::runtime_error("outside_component_control_identity_failed");
    return result;
}

inline std::unique_ptr<Task10p11rFixedBaselineFixture> cloneFixture(
    const Task10p11rFixedBaselineFixture& source,
    const std::optional<std::string>& active_pair) {
    auto shadow=makeTask10p11rFixedBaselineFixture(
        GammaFeedbackSelectionMode::LeastIntervention,22.0);
    if (!shadow->adapter.initializeStageZero().initialized)
        throw std::runtime_error("terminal optimizer shadow init failed");
    const auto restart=makeTask10p11vSparseRestartCheckpoint(
        source,active_pair,source.controller.successfulControlCycles());
    const auto restored=restoreTask10p11vSparseRestartCheckpoint(
        *shadow,restart);
    if (restored.active_pair!=active_pair)
        throw std::runtime_error("terminal optimizer active pair mismatch");
    if (!shadow->topologyFrozen())
        throw std::runtime_error("terminal optimizer topology not frozen");
    return shadow;
}

struct NativeFrame {
    Task10p11zPreOverrideCapture boundary;
    Task10p11zPreparedBaseline prepared;
    nlohmann::json snapshot;
    Task10p11sRows28d problem;
};

inline NativeFrame nativeFrame(Task10p11rFixedBaselineFixture& fixture,
                               std::optional<std::string> active_pair) {
    NativeFrame frame;
    frame.boundary=task10p11zCaptureBeforeOverride(fixture);
    frame.prepared=task10p11zPrepareNativeBaseline(
        fixture,GammaFeedbackSelectionMode::LeastIntervention,22.0,
        std::move(active_pair));
    if (!frame.prepared.valid || !frame.prepared.control.step.advanced)
        throw std::runtime_error("native proposal unavailable:"+
                                 frame.prepared.reason);
    frame.snapshot=makeTask10p11sSnapshot(
        frame.boundary.runtime,frame.boundary.request,
        frame.prepared.nominal_controls,fixture.adapter.config());
    frame.problem=buildTask10p11sRows28d(
        buildCanonicalHardRows(frame.boundary.request),
        frame.boundary.request.mobile_ids,true);
    if (frame.problem.rows.size()!=1113)
        throw std::runtime_error("terminal optimizer row count mismatch");
    return frame;
}

inline SimpleCoverageControlStep applyVerified(
    Task10p11rFixedBaselineFixture& fixture,const NativeFrame& frame,
    const std::map<NodeId,Eigen::Vector2d>& controls,
    double successor_residual) {
    return fixture.controller.advanceWithDevelopmentControlOverride(
        [&](const GrandFinaleRuntimeSnapshot& runtime,
            const std::map<NodeId,Eigen::Vector2d>& nominal,
            const std::map<NodeId,double>& yaw_rates) {
            if (!task10p11zAppliedControlsMatch(
                    nominal,frame.prepared.nominal_controls,1.0e-12)) {
                GrandFinaleSwarmStep rejected;
                rejected.reason="terminal_optimizer_native_nominal_mismatch";
                return rejected;
            }
            return fixture.adapter.stepWithDevelopmentFullPairCertifiedControls(
                controls,yaw_rates,runtime.estimator_token,
                runtime.topology_token,true,successor_residual);
        });
}

inline double independentResidualAtEstimate(
    const nlohmann::json& snapshot,const JointEstimateSnapshot& estimate,
    const std::map<NodeId,Eigen::Vector2d>& controls,
    std::size_t* row_count=nullptr) {
    const auto request=task10p11x_detail::requestAtEstimate(snapshot,estimate);
    const auto problem=buildTask10p11sRows28d(
        buildCanonicalHardRows(request),request.mobile_ids,true);
    if (row_count!=nullptr) *row_count=problem.rows.size();
    return task10p11af_detail::independentMinimumResidual(
        problem,task10p11sOrderedControls(request.mobile_ids,controls));
}

inline double finiteMinimum(std::initializer_list<double> values) {
    double result=std::numeric_limits<double>::infinity();
    for (double value:values) result=std::min(result,value);
    return result;
}

inline double deviation(
    const std::map<NodeId,Eigen::Vector2d>& selected,
    const std::map<NodeId,Eigen::Vector2d>& distributed) {
    double squared=0.0;
    for (NodeId owner:kComponent)
        squared+=(selected.at(owner)-distributed.at(owner)).squaredNorm();
    return std::sqrt(squared);
}

}  // namespace task10p11ah_optimizer_detail

inline Task10p11ahPlanEvaluation evaluateTask10p11ahTerminalRecoveryPlan(
    const Task10p11rFixedBaselineFixture& source,
    std::optional<std::string> active_pair,
    const Task10p11ahComponentPlan& plan) {
    using namespace task10p11ah_optimizer_detail;
    Task10p11ahPlanEvaluation result;
    try {
        auto fixture=cloneFixture(source,active_pair);
        auto frame0=nativeFrame(*fixture,active_pair);
        active_pair=frame0.prepared.active_pair;
        result.distributed_u0=frame0.prepared.control.step.applied_controls;
        result.selected_u0=projectedComponentControls(
            frame0.boundary.request,result.distributed_u0,
            plan.owner2_u0,plan.owner9_u0,
            &result.x0_realized_minimum_residual_mps2);
        result.full_row_count_x0=frame0.problem.rows.size();
        result.x0_estimator=task10p11s_capture_detail::estimateJson(
            frame0.boundary.runtime.estimate);
        const auto exact_x1=task10p11aa_detail::predictEstimate(
            frame0.snapshot,frame0.boundary.runtime.estimate,
            result.selected_u0);
        const auto successor0=task10p11af_detail::successorFullPair(
            frame0.snapshot,result.selected_u0);
        result.score.minimum_full_row_residual_mps2=finiteMinimum({
            result.x0_realized_minimum_residual_mps2,
            successor0.minimum_residual});
        if (!successor0.feasible) {
            result.valid=true;
            result.reason="x0_control_has_no_full_pair_successor";
            result.classification=
                Task10p11ahOptimizerClassification::LocalNoWitness;
            return result;
        }
        const auto applied0=applyVerified(
            *fixture,frame0,result.selected_u0,successor0.minimum_residual);
        if (!applied0.step.advanced)
            throw std::runtime_error("x0 actual advance failed:"+
                (applied0.reason.empty()?applied0.step.reason:applied0.reason));

        auto frame1=nativeFrame(*fixture,active_pair);
        active_pair=frame1.prepared.active_pair;
        result.distributed_u1=frame1.prepared.control.step.applied_controls;
        result.selected_u1=projectedComponentControls(
            frame1.boundary.request,result.distributed_u1,
            plan.owner2_u1,plan.owner9_u1,
            &result.x1_realized_minimum_residual_mps2);
        result.x1_exact_minimum_residual_mps2=independentResidualAtEstimate(
            frame0.snapshot,exact_x1,result.selected_u1,
            &result.full_row_count_x1);
        result.x1_estimator=task10p11s_capture_detail::estimateJson(
            frame1.boundary.runtime.estimate);
        const auto exact_x2=task10p11aa_detail::predictEstimate(
            frame0.snapshot,exact_x1,result.selected_u1);
        const auto successor1=task10p11af_detail::successorFullPair(
            frame1.snapshot,result.selected_u1);
        result.score.minimum_full_row_residual_mps2=finiteMinimum({
            result.score.minimum_full_row_residual_mps2,
            result.x1_exact_minimum_residual_mps2,
            result.x1_realized_minimum_residual_mps2,
            successor1.minimum_residual});
        if (!successor1.feasible) {
            result.valid=true;
            result.reason="x1_control_has_no_full_pair_successor";
            result.classification=
                Task10p11ahOptimizerClassification::LocalNoWitness;
            return result;
        }
        const auto applied1=applyVerified(
            *fixture,frame1,result.selected_u1,successor1.minimum_residual);
        if (!applied1.step.advanced)
            throw std::runtime_error("x1 actual advance failed:"+
                (applied1.reason.empty()?applied1.step.reason:applied1.reason));

        auto frame2=nativeFrame(*fixture,active_pair);
        result.distributed_u2=frame2.prepared.control.step.applied_controls;
        const Eigen::VectorXd native2=task10p11sOrderedControls(
            frame2.boundary.request.mobile_ids,result.distributed_u2);
        result.x2_realized_minimum_residual_mps2=
            task10p11af_detail::independentMinimumResidual(
                frame2.problem,native2);
        result.x2_exact_minimum_residual_mps2=independentResidualAtEstimate(
            frame0.snapshot,exact_x2,result.distributed_u2,
            &result.full_row_count_x2);
        result.x2_estimator=task10p11s_capture_detail::estimateJson(
            frame2.boundary.runtime.estimate);
        const auto rows2=buildCanonicalHardRows(frame2.boundary.request);
        result.terminal_owner_local_gamma_mps2=
            task10p11af_detail::minimumOwnerLocalGamma(
                rows2,frame2.boundary.request);
        const auto transfer=solveTask10p11tDynamicPair(
            rows2,frame2.boundary.request.mobile_ids,
            frame2.prepared.nominal_controls,
            frame2.boundary.request.acceleration_half_box,
            "collision:2--9");
        result.terminal_signed_transfer_feasible=transfer.feasible;
        if (transfer.shared_interval.valid)
            result.terminal_signed_transfer_width_mps2=
                transfer.shared_interval.upper-transfer.shared_interval.lower;
        result.terminal_native_current_feasible=
            result.x2_realized_minimum_residual_mps2>=-kTolerance &&
            result.x2_exact_minimum_residual_mps2>=-kTolerance;
        const auto successor2=task10p11af_detail::successorFullPair(
            frame2.snapshot,result.distributed_u2);
        result.terminal_native_successor_gamma_mps2=
            successor2.recomputed_gamma;
        result.terminal_native_successor_residual_mps2=
            successor2.minimum_residual;
        result.terminal_native_successor_feasible=successor2.feasible;
        const auto continuation=task10p11ah_detail::continuationWitness(
            frame2.snapshot,frame2.boundary.runtime.estimate,
            frame2.boundary.request,native2);
        result.terminal_h1_witness=continuation.h1;
        result.terminal_h2_witness=continuation.h2;
        result.score.full_rows_feasible=finiteMinimum({
            result.score.minimum_full_row_residual_mps2,
            result.x2_exact_minimum_residual_mps2,
            result.x2_realized_minimum_residual_mps2})>=-kTolerance;
        result.score.minimum_full_row_residual_mps2=finiteMinimum({
            result.score.minimum_full_row_residual_mps2,
            result.x2_exact_minimum_residual_mps2,
            result.x2_realized_minimum_residual_mps2});
        result.score.terminal_recovered=result.score.full_rows_feasible &&
            result.terminal_owner_local_gamma_mps2>=-kTolerance &&
            result.terminal_signed_transfer_feasible &&
            result.terminal_native_current_feasible &&
            result.terminal_native_successor_feasible;
        result.score.terminal_recovery_margin_mps2=finiteMinimum({
            result.terminal_owner_local_gamma_mps2,
            result.terminal_signed_transfer_width_mps2,
            result.x2_realized_minimum_residual_mps2,
            result.x2_exact_minimum_residual_mps2,
            result.terminal_native_successor_gamma_mps2,
            result.terminal_native_successor_residual_mps2});
        result.score.cumulative_coverage_deviation_l2_mps2=
            deviation(result.selected_u0,result.distributed_u0)+
            deviation(result.selected_u1,result.distributed_u1);
        result.valid=true;
        result.classification=result.score.terminal_recovered
            ?Task10p11ahOptimizerClassification::Witness
            :Task10p11ahOptimizerClassification::LocalNoWitness;
        result.reason=result.score.terminal_recovered
            ?"terminal_distributed_recovery_witness_rebuilt"
            :"full_rows_hold_but_terminal_distributed_recovery_missing";
    } catch (const std::exception& error) {
        result.reason=error.what();
        result.classification=Task10p11ahOptimizerClassification::Undetermined;
    }
    return result;
}

inline nlohmann::json task10p11ahControlsJson(
    const std::map<NodeId,Eigen::Vector2d>& controls) {
    nlohmann::json result=nlohmann::json::object();
    for (const auto& [owner,control]:controls)
        result[std::to_string(owner)]={control.x(),control.y()};
    return result;
}

inline nlohmann::json task10p11ahPlanEvaluationJson(
    const Task10p11ahPlanEvaluation& value) {
    return {{"valid",value.valid},{"reason",value.reason},
        {"classification",task10p11ahOptimizerClassificationName(
            value.classification)},
        {"full_rows_feasible",value.score.full_rows_feasible},
        {"terminal_recovered",value.score.terminal_recovered},
        {"minimum_full_row_residual_mps2",task10p11w_detail::number(
            value.score.minimum_full_row_residual_mps2)},
        {"terminal_recovery_margin_mps2",task10p11w_detail::number(
            value.score.terminal_recovery_margin_mps2)},
        {"cumulative_coverage_deviation_l2_mps2",
            task10p11w_detail::number(
                value.score.cumulative_coverage_deviation_l2_mps2)},
        {"rows",{{"x0",value.full_row_count_x0},
            {"x1",value.full_row_count_x1},{"x2",value.full_row_count_x2}}},
        {"residuals_mps2",{
            {"x0_realized",task10p11w_detail::number(
                value.x0_realized_minimum_residual_mps2)},
            {"x1_exact",task10p11w_detail::number(
                value.x1_exact_minimum_residual_mps2)},
            {"x1_realized",task10p11w_detail::number(
                value.x1_realized_minimum_residual_mps2)},
            {"x2_exact",task10p11w_detail::number(
                value.x2_exact_minimum_residual_mps2)},
            {"x2_realized",task10p11w_detail::number(
                value.x2_realized_minimum_residual_mps2)}}},
        {"terminal",{
            {"owner_local_gamma_mps2",task10p11w_detail::number(
                value.terminal_owner_local_gamma_mps2)},
            {"signed_transfer_feasible",
                value.terminal_signed_transfer_feasible},
            {"signed_transfer_width_mps2",task10p11w_detail::number(
                value.terminal_signed_transfer_width_mps2)},
            {"native_current_full_pair_feasible",
                value.terminal_native_current_feasible},
            {"native_successor_full_pair_feasible",
                value.terminal_native_successor_feasible},
            {"native_successor_gamma_mps2",task10p11w_detail::number(
                value.terminal_native_successor_gamma_mps2)},
            {"native_successor_minimum_residual_mps2",
                task10p11w_detail::number(
                    value.terminal_native_successor_residual_mps2)},
            {"H1_witness",value.terminal_h1_witness},
            {"H2_witness",value.terminal_h2_witness}}},
        {"controls",{{"distributed_U0",task10p11ahControlsJson(
                value.distributed_u0)},
            {"selected_U0",task10p11ahControlsJson(value.selected_u0)},
            {"distributed_U1",task10p11ahControlsJson(
                value.distributed_u1)},
            {"selected_U1",task10p11ahControlsJson(value.selected_u1)},
            {"distributed_U2",task10p11ahControlsJson(
                value.distributed_u2)}}},
        {"estimator",{{"x0",value.x0_estimator},
            {"x1",value.x1_estimator},{"x2",value.x2_estimator}}},
        {"claim_boundary",{{"positive_witness_proves_existence",true},
            {"negative_local_search_proves_infeasibility",false},
            {"post_hoc_development_extension",true},
            {"recursive_feasibility_claimed",false}}}};
}

inline Eigen::Matrix<double,8,1> task10p11ahPlanVector(
    const Task10p11ahComponentPlan& plan) {
    Eigen::Matrix<double,8,1> value;
    value<<plan.owner2_u0.x(),plan.owner2_u0.y(),
        plan.owner9_u0.x(),plan.owner9_u0.y(),
        plan.owner2_u1.x(),plan.owner2_u1.y(),
        plan.owner9_u1.x(),plan.owner9_u1.y();
    return value;
}

inline Task10p11ahComponentPlan task10p11ahPlanFromVector(
    const Eigen::Matrix<double,8,1>& value) {
    Task10p11ahComponentPlan plan;
    plan.owner2_u0={value(0),value(1)};
    plan.owner9_u0={value(2),value(3)};
    plan.owner2_u1={value(4),value(5)};
    plan.owner9_u1={value(6),value(7)};
    return plan;
}

struct Task10p11ahPatternSearchSettings {
    std::vector<double> step_schedule_mps2{
        1.0,0.5,0.25,0.125,0.0625,0.03125,0.015625,0.0078125};
    std::size_t maximum_sweeps_per_step=2;
    std::size_t maximum_evaluations=260;
    double wall_clock_limit_s=3600.0;
    double input_half_box_mps2=4.0;
};

struct Task10p11ahOptimizerResult {
    bool valid=false;
    std::string reason;
    Task10p11ahOptimizerClassification classification=
        Task10p11ahOptimizerClassification::Undetermined;
    bool terminal_recovery_witness_found=false;
    bool timed_out=false;
    std::size_t evaluations=0;
    double solve_time_s=0.0;
    Task10p11ahComponentPlan plan;
    Task10p11ahPlanEvaluation evaluation;
    nlohmann::json trace=nlohmann::json::array();
};

using Task10p11ahPlanEvaluator=std::function<Task10p11ahPlanEvaluation(
    const Task10p11ahComponentPlan&)>;

inline Task10p11ahOptimizerResult solveTask10p11ahTerminalRecovery(
    const std::vector<Task10p11ahComponentPlan>& registered_starts,
    const Task10p11ahPlanEvaluator& evaluator,
    const Task10p11ahPatternSearchSettings& settings={}) {
    Task10p11ahOptimizerResult result;
    const auto started=std::chrono::steady_clock::now();
    const auto deadline=started+std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(std::chrono::duration<double>(
            settings.wall_clock_limit_s));
    if (!evaluator || registered_starts.empty() ||
        settings.step_schedule_mps2.empty() ||
        settings.maximum_evaluations==0 ||
        !std::isfinite(settings.input_half_box_mps2) ||
        settings.input_half_box_mps2<=0.0) {
        result.reason="invalid_frozen_pattern_search_settings";
        return result;
    }
    std::optional<Task10p11ahPlanEvaluation> best;
    Eigen::Matrix<double,8,1> center=Eigen::Matrix<double,8,1>::Zero();
    auto evaluate=[&](const Eigen::Matrix<double,8,1>& vector,
                      const std::string& source,double step,
                      std::size_t dimension,int direction) {
        const auto value=evaluator(task10p11ahPlanFromVector(vector));
        ++result.evaluations;
        result.trace.push_back({{"evaluation",result.evaluations},
            {"source",source},{"step_mps2",step},
            {"dimension",dimension},{"direction",direction},
            {"valid",value.valid},
            {"classification",task10p11ahOptimizerClassificationName(
                value.classification)},
            {"full_rows_feasible",value.score.full_rows_feasible},
            {"terminal_recovered",value.score.terminal_recovered},
            {"minimum_full_row_residual_mps2",task10p11w_detail::number(
                value.score.minimum_full_row_residual_mps2)},
            {"terminal_recovery_margin_mps2",task10p11w_detail::number(
                value.score.terminal_recovery_margin_mps2)},
            {"coverage_deviation_l2_mps2",task10p11w_detail::number(
                value.score.cumulative_coverage_deviation_l2_mps2)}});
        return value;
    };
    for (std::size_t index=0;index<registered_starts.size() &&
            result.evaluations<settings.maximum_evaluations &&
            std::chrono::steady_clock::now()<deadline;++index) {
        auto vector=task10p11ahPlanVector(registered_starts[index]);
        vector=vector.cwiseMax(-settings.input_half_box_mps2)
                     .cwiseMin(settings.input_half_box_mps2);
        auto value=evaluate(vector,"registered_start",0.0,index,0);
        if (value.valid && (!best.has_value() ||
                task10p11ahBetterPlan(value.score,best->score))) {
            best=std::move(value);
            center=vector;
        }
    }
    if (!best.has_value()) {
        result.reason="all_registered_starts_undetermined";
        result.classification=Task10p11ahOptimizerClassification::Undetermined;
        result.solve_time_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return result;
    }
    for (double step:settings.step_schedule_mps2) {
        if (!std::isfinite(step) || step<=0.0) {
            result.reason="invalid_frozen_step_schedule";
            result.classification=
                Task10p11ahOptimizerClassification::Undetermined;
            return result;
        }
        for (std::size_t sweep=0;
             sweep<settings.maximum_sweeps_per_step;++sweep) {
            bool improved=false;
            Eigen::Matrix<double,8,1> sweep_best_vector=center;
            Task10p11ahPlanEvaluation sweep_best=*best;
            for (Eigen::Index dimension=0;dimension<8;++dimension) {
                for (int direction:{-1,1}) {
                    if (result.evaluations>=settings.maximum_evaluations ||
                        std::chrono::steady_clock::now()>=deadline) break;
                    auto candidate=center;
                    candidate(dimension)=std::clamp(
                        candidate(dimension)+direction*step,
                        -settings.input_half_box_mps2,
                        settings.input_half_box_mps2);
                    if (candidate(dimension)==center(dimension)) continue;
                    auto value=evaluate(candidate,"coordinate_poll",step,
                        static_cast<std::size_t>(dimension),direction);
                    if (value.valid && task10p11ahBetterPlan(
                            value.score,sweep_best.score)) {
                        sweep_best=std::move(value);
                        sweep_best_vector=candidate;
                        improved=true;
                    }
                }
            }
            if (!improved) break;
            *best=std::move(sweep_best);
            center=sweep_best_vector;
        }
        if (result.evaluations>=settings.maximum_evaluations ||
            std::chrono::steady_clock::now()>=deadline) break;
    }
    result.timed_out=std::chrono::steady_clock::now()>=deadline;
    result.plan=task10p11ahPlanFromVector(center);
    result.evaluation=*best;
    result.terminal_recovery_witness_found=
        best->score.terminal_recovered;
    result.classification=result.terminal_recovery_witness_found
        ?Task10p11ahOptimizerClassification::Witness
        :Task10p11ahOptimizerClassification::LocalNoWitness;
    result.valid=true;
    result.reason=result.terminal_recovery_witness_found
        ?"verified_terminal_recovery_witness_found"
        :(result.timed_out
            ?"local_search_timed_out_without_witness"
            :"deterministic_local_search_found_no_terminal_recovery_witness");
    result.solve_time_s=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-started).count();
    return result;
}

inline nlohmann::json task10p11ahOptimizerResultJson(
    const Task10p11ahOptimizerResult& value) {
    const auto vector=task10p11ahPlanVector(value.plan);
    return {{"protocol","task10p11ah-posthoc-terminal-recovery-optimizer-v1"},
        {"valid",value.valid},{"reason",value.reason},
        {"classification",task10p11ahOptimizerClassificationName(
            value.classification)},
        {"terminal_recovery_witness_found",
            value.terminal_recovery_witness_found},
        {"timed_out",value.timed_out},{"evaluations",value.evaluations},
        {"solve_time_s",value.solve_time_s},
        {"component",nlohmann::json::array({2,9})},
        {"horizon_steps",2},{"tau_mps2",22.0},
        {"selected_plan_vector",std::vector<double>(
            vector.data(),vector.data()+vector.size())},
        {"evaluation",task10p11ahPlanEvaluationJson(value.evaluation)},
        {"optimization_trace",value.trace},
        {"claim_boundary",{
            {"post_hoc_development_method_extension",true},
            {"local_no_witness_is_not_infeasibility",true},
            {"fixed_topology",true},{"component_expansion",false},
            {"horizon_expansion",false},{"recursive_feasibility",false}}}};
}

}  // namespace gf
