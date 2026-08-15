#pragma once

#include "grand_finale/BrakingSnapshotOracle.hpp"
#include "grand_finale/CanonicalGammaStarFeedback.hpp"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "models/DoubleIntegrate2D.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct FrontierRolloutRequest {
    SolverProfile profile = SolverProfile::OpenSource;
    CanonicalHardRowRequest hard_row_request;
    std::map<NodeId,Eigen::Vector2d> targets;
    double position_gain = 0.0;
    double velocity_gain = 0.0;
    double dt_s = 0.0;
    std::size_t cycles = 0;
    double residual_tolerance = 1e-7;
    JointEstimateSnapshot estimator_snapshot;
    double estimator_acceleration_variance = 0.0;
    CanonicalGammaFeedbackConfig gamma_feedback_config;
    std::function<CanonicalHardRowRequest(
        const JointEstimateSnapshot&)> canonical_request_builder;
};

struct FrontierRolloutResult {
    bool accepted = false;
    std::string reason;
    std::size_t completed_cycles = 0;
    std::size_t failed_cycle = std::numeric_limits<std::size_t>::max();
    double duration_s = 0.0;
    double minimum_robust_residual =
        std::numeric_limits<double>::infinity();
    CanonicalHardRowRequest final_request;
    struct BrakingTraceEntry {
        std::size_t cycle = 0;
        double available_horizon_s = 0.0;
        BrakingSnapshotResult snapshot;
    };
    std::vector<BrakingTraceEntry> braking_trace;
    std::string first_negative_source;
    CanonicalGammaFeedbackWork gamma_policy_work;
    std::size_t qp_solves = 0;
    std::vector<CanonicalGammaFeedbackBatchResult> policy_trace;
    std::vector<double> policy_cycle_wall_s;
    Task10p11ComputeProfile compute_profile;
};

inline FrontierRolloutResult evaluateFrontierRollout(
    FrontierRolloutRequest request) {
    if (request.cycles == 0 || !std::isfinite(request.dt_s) ||
        request.dt_s <= 0.0 || !std::isfinite(request.position_gain) ||
        request.position_gain < 0.0 ||
        !std::isfinite(request.velocity_gain) ||
        request.velocity_gain < 0.0 ||
        request.targets.size() != request.hard_row_request.mobile_ids.size() ||
        request.estimator_snapshot.mobile_ids !=
            request.hard_row_request.mobile_ids ||
        request.estimator_snapshot.mean.size()!=static_cast<Eigen::Index>(
            4*request.estimator_snapshot.mobile_ids.size()) ||
        request.estimator_snapshot.covariance.rows()!=
            request.estimator_snapshot.mean.size() ||
        request.estimator_snapshot.covariance.cols()!=
            request.estimator_snapshot.mean.size() ||
        !std::isfinite(request.estimator_acceleration_variance) ||
        request.estimator_acceleration_variance<0.0 ||
        !request.canonical_request_builder) {
        throw std::invalid_argument("invalid frontier rollout request");
    }
    validateCanonicalGammaFeedbackConfig(request.gamma_feedback_config);
    FrontierRolloutResult result;
    CanonicalHocbfQpController controller;
    for (std::size_t cycle=0;cycle<request.cycles;++cycle) {
        const double available_horizon=
            static_cast<double>(request.cycles-cycle)*request.dt_s;
        auto braking=evaluateBrakingSnapshot(
            request.hard_row_request,available_horizon,
            request.residual_tolerance);
        result.braking_trace.push_back({
            cycle,available_horizon,std::move(braking)});
        const auto& snapshot=result.braking_trace.back().snapshot;
        if (!snapshot.hard_polytope_nonempty ||
            !snapshot.snapshot_braking_admissible) {
            result.reason=snapshot.reason;
            result.first_negative_source=snapshot.first_negative_source;
            result.failed_cycle=cycle;
            result.final_request=request.hard_row_request;
            return result;
        }
        std::map<NodeId,Eigen::Vector2d> task_nominals;
        for (std::size_t index=0;
             index<request.estimator_snapshot.mobile_ids.size();++index) {
            const NodeId owner=request.estimator_snapshot.mobile_ids[index];
            const Eigen::Vector4d state=
                request.estimator_snapshot.mean.segment<4>(4*index);
            Eigen::Vector2d nominal=request.position_gain*
                (request.targets.at(owner)-state.head<2>())-
                request.velocity_gain*state.tail<2>();
            nominal.x()=std::clamp(nominal.x(),
                -request.hard_row_request.acceleration_half_box,
                request.hard_row_request.acceleration_half_box);
            nominal.y()=std::clamp(nominal.y(),
                -request.hard_row_request.acceleration_half_box,
                request.hard_row_request.acceleration_half_box);
            task_nominals.emplace(owner,nominal);
        }
        CanonicalGammaFeedbackEvaluationContext policy_context;
        const auto policy_start=std::chrono::steady_clock::now();
        auto policy=evaluateCanonicalGammaFeedbackBatch(
            request.estimator_snapshot,task_nominals,
            request.gamma_feedback_config,request.dt_s,
            request.estimator_acceleration_variance,
            request.canonical_request_builder,policy_context);
        result.policy_cycle_wall_s.push_back(std::chrono::duration<double>(
            std::chrono::steady_clock::now()-policy_start).count());
        result.gamma_policy_work.policy_evaluations+=
            policy.work.policy_evaluations;
        result.gamma_policy_work.canonical_row_rebuilds+=
            policy.work.canonical_row_rebuilds;
        result.gamma_policy_work.exact_gamma_solves+=
            policy.work.exact_gamma_solves;
        result.compute_profile.merge(policy.compute_profile);
        if (!policy.valid) {
            result.reason=policy.reason;
            result.failed_cycle=cycle;
            result.final_request=request.hard_row_request;
            result.policy_trace.push_back(std::move(policy));
            return result;
        }
        const auto& rows=policy.current_rows;
        std::map<NodeId,Eigen::Vector2d> controls;
        for (NodeId owner : request.hard_row_request.mobile_ids) {
            const auto solved=controller.solve({
                request.profile,owner,cycle+1,1,SupervisorMode::Search,
                policy.selected_controls.at(owner),
                request.hard_row_request.acceleration_half_box,
                rows,request.residual_tolerance});
            result.compute_profile.record(
                Task10p11ComputePhase::SolverInitialization,
                solved.solver_initialization_wall_s,
                !solved.solver_cold_start);
            result.compute_profile.record(
                Task10p11ComputePhase::SolverModelUpdate,
                solved.solver_model_update_wall_s,true);
            result.compute_profile.record(
                Task10p11ComputePhase::RobustQpSolve,
                solved.solver_solve_wall_s,true);
            result.compute_profile.record(
                Task10p11ComputePhase::ResidualTokenAudit,
                solved.residual_token_audit_wall_s,true);
            ++result.qp_solves;
            if (!controlMayBeApplied(
                    solved,cycle+1,1,SupervisorMode::Search)) {
                result.reason=solved.failure_reason;
                result.failed_cycle=cycle;
                result.final_request=request.hard_row_request;
                return result;
            }
            const double independent=minimumCanonicalOwnerResidual(
                rows,owner,solved.control);
            if (independent < -request.residual_tolerance) {
                result.reason="residual_verification_failed";
                result.failed_cycle=cycle;
                result.final_request=request.hard_row_request;
                return result;
            }
            result.minimum_robust_residual=std::min(
                result.minimum_robust_residual,independent);
            controls[owner]=solved.control;
        }
        request.estimator_snapshot=predictNoMeasurementSnapshot(
            request.estimator_snapshot,controls,request.dt_s,
            request.estimator_acceleration_variance);
        request.hard_row_request=
            request.canonical_request_builder(request.estimator_snapshot);
        result.policy_trace.push_back(std::move(policy));
        ++result.completed_cycles;
    }
    result.accepted=true;
    result.reason="accepted";
    result.duration_s=request.dt_s*request.cycles;
    result.final_request=std::move(request.hard_row_request);
    return result;
}

}  // namespace gf
