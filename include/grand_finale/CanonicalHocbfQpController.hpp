#pragma once

#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/HybridSupervisor.hpp"
#include "grand_finale/ProgressCompatibility.hpp"
#include "grand_finale/Types.hpp"
#include "optimisers/optimisers"

#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gf {

struct CanonicalQpRequest {
    SolverProfile profile = SolverProfile::OpenSource;
    NodeId owner = 0;
    std::uint64_t estimator_version = 0;
    std::uint64_t topology_version = 0;
    SupervisorMode mode = SupervisorMode::Search;
    Eigen::Vector2d nominal = Eigen::Vector2d::Zero();
    double acceleration_half_box = 0.0;
    std::vector<CanonicalHardRow> rows;
    double residual_tolerance = 1.0e-7;
    // Task 11b rung B': the estimate-side SpeedLimit initial-set precheck
    // trips on ~1e-9-scale boundary numerics when the row binds at cruise
    // (S1-v3 death).  Under the truth-gate flag the row stays binding as a
    // QP constraint, and the initial-set judgment moves to truth telemetry
    // in the runner (hard gate at 30.01 m/s, fuse at 31).
    bool speed_initial_set_truth_gate = false;
};

struct CanonicalQpResult {
    NodeId owner = 0;
    std::uint64_t estimator_version = 0;
    std::uint64_t topology_version = 0;
    SupervisorMode mode = SupervisorMode::Search;
    bool hard_polytope_nonempty = false;
    bool solver_succeeded = false;
    bool residual_verified = false;
    bool control_available = false;
    Eigen::Vector2d control = Eigen::Vector2d::Zero();
    Eigen::Vector2d exact_projection = Eigen::Vector2d::Zero();
    double exact_oracle_error = std::numeric_limits<double>::infinity();
    double minimum_hard_residual =
        -std::numeric_limits<double>::infinity();
    std::string solver_status = "not_run";
    std::string failure_reason;
    bool solver_cold_start = false;
    bool solver_instance_reused = false;
    double exact_projection_wall_s = 0.0;
    double solver_initialization_wall_s = 0.0;
    double solver_model_update_wall_s = 0.0;
    double solver_solve_wall_s = 0.0;
    double residual_token_audit_wall_s = 0.0;
    int solver_iteration_count = -1;
    double solver_primal_residual =
        std::numeric_limits<double>::infinity();
    double solver_dual_residual =
        std::numeric_limits<double>::infinity();
};

inline bool controlMayBeApplied(
    const CanonicalQpResult& result,
    std::uint64_t estimator_version,
    std::uint64_t topology_version,
    SupervisorMode mode) {
    return result.control_available && result.solver_succeeded &&
           result.residual_verified &&
           result.estimator_version == estimator_version &&
           result.topology_version == topology_version &&
           result.mode == mode;
}

class CanonicalHocbfQpController {
public:
    CanonicalQpResult solve(const CanonicalQpRequest& request) const {
        CanonicalQpResult result;
        result.owner = request.owner;
        result.estimator_version = request.estimator_version;
        result.topology_version = request.topology_version;
        result.mode = request.mode;

        if (!request.nominal.allFinite() ||
            !std::isfinite(request.acceleration_half_box) ||
            request.acceleration_half_box <= 0.0 ||
            !std::isfinite(request.residual_tolerance) ||
            request.residual_tolerance < 0.0) {
            result.failure_reason = "invalid_request";
            return result;
        }
        if (!request.speed_initial_set_truth_gate) {
            for (const auto& row : request.rows) {
                if (row.owner==request.owner &&
                    row.kind==CanonicalHardRowKind::SpeedLimit &&
                    (!std::isfinite(row.barrier_h) ||
                     row.barrier_h < -request.residual_tolerance)) {
                    result.failure_reason="speed_initial_set_violated";
                    return result;
                }
            }
        }

        const auto exact_started=std::chrono::steady_clock::now();
        const auto exact = evaluateProgressCompatibility(
            request.rows, request.owner, request.nominal,
            request.acceleration_half_box,
            ProgressCompatibilityConfig{
                std::numeric_limits<double>::max(), 0.0,
                request.residual_tolerance, true});
        result.exact_projection_wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-exact_started).count();
        result.hard_polytope_nonempty = exact.polytope_nonempty;
        if (!exact.polytope_nonempty) {
            result.failure_reason = "hard_polytope_empty";
            return result;
        }
        result.exact_projection = exact.projection;

        json settings = {
            {"k_delta", 1.0},
            {"absolute-tolerance", 1.0e-8},
            {"relative-tolerance", 1.0e-8},
            {"primal-infeasibility-tolerance", 1.0e-8},
            {"dual-infeasibility-tolerance", 1.0e-8},
            {"maximum-iterations", 1000000},
            {"scaling-iterations", 0},
            {"explicit-row-scaling", true},
        };
        try {
            const std::string optimiser_name =
                request.profile == SolverProfile::Gurobi ? "Gurobi" : "OSQP";
            auto found=optimisers_.find(request.profile);
            if (found==optimisers_.end()) {
                const auto initialization_started=
                    std::chrono::steady_clock::now();
                found=optimisers_.emplace(
                    request.profile,createOptimiser(optimiser_name,settings)).first;
                result.solver_initialization_wall_s=
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now()-initialization_started).count();
                result.solver_cold_start=true;
            } else {
                result.solver_instance_reused=true;
                found->second->clear();
            }
            OptimiserBase& optimiser=*found->second;
            const auto update_started=std::chrono::steady_clock::now();
            optimiser.start(2, 2);
            optimiser.setObjective(request.nominal);
            for (const CanonicalHardRow& row : request.rows) {
                if (row.owner != request.owner) continue;
                optimiser.addLinearConstraint(
                    row.control_coefficient, -row.constant);
            }
            result.solver_model_update_wall_s=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-update_started).count();
            const auto solve_started=std::chrono::steady_clock::now();
            const Eigen::VectorXd solution = optimiser.solve();
            result.solver_solve_wall_s=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-solve_started).count();
            const json status = optimiser.getStatus();
            result.solver_status = status.value("status", "unknown");
            result.solver_iteration_count = status.value("iteration_count", -1);
            result.solver_primal_residual = status.value(
                "primal_residual",std::numeric_limits<double>::infinity());
            result.solver_dual_residual = status.value(
                "dual_residual",std::numeric_limits<double>::infinity());
            result.solver_succeeded =
                result.solver_status == "optimal" ||
                result.solver_status == "optimal_inaccurate";
            if (!result.solver_succeeded || solution.size() < 2 ||
                !solution.head<2>().allFinite()) {
                result.failure_reason = "solver_failure";
                return result;
            }
            const auto audit_started=std::chrono::steady_clock::now();
            result.control = solution.head<2>();
            result.minimum_hard_residual = minimumCanonicalOwnerResidual(
                request.rows, request.owner, result.control);
            result.residual_verified =
                result.minimum_hard_residual >= -request.residual_tolerance;
            result.exact_oracle_error =
                (result.control - result.exact_projection).norm();
            result.residual_token_audit_wall_s=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-audit_started).count();
            if (!result.residual_verified) {
                result.failure_reason = "residual_verification_failed";
                return result;
            }
            result.control_available = true;
            return result;
        } catch (...) {
            result.failure_reason = "solver_failure";
            return result;
        }
    }

private:
    mutable std::map<SolverProfile,std::unique_ptr<OptimiserBase>> optimisers_;
};

}  // namespace gf
