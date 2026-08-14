#pragma once

#include "grand_finale/BrakingSnapshotOracle.hpp"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "models/DoubleIntegrate2D.hpp"

#include <algorithm>
#include <cmath>
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
};

inline FrontierRolloutResult evaluateFrontierRollout(
    FrontierRolloutRequest request) {
    if (request.cycles == 0 || !std::isfinite(request.dt_s) ||
        request.dt_s <= 0.0 || !std::isfinite(request.position_gain) ||
        request.position_gain < 0.0 ||
        !std::isfinite(request.velocity_gain) ||
        request.velocity_gain < 0.0 ||
        request.targets.size() != request.hard_row_request.mobile_ids.size()) {
        throw std::invalid_argument("invalid frontier rollout request");
    }
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
        const auto rows=buildCanonicalHardRows(request.hard_row_request);
        std::map<NodeId,Eigen::Vector2d> controls;
        for (NodeId owner : request.hard_row_request.mobile_ids) {
            const auto& state=request.hard_row_request.states.at(owner);
            const Eigen::Vector2d position(state.position.x,state.position.y);
            Eigen::Vector2d nominal=request.position_gain*
                (request.targets.at(owner)-position)-
                request.velocity_gain*state.velocity;
            nominal.x()=std::clamp(nominal.x(),
                -request.hard_row_request.acceleration_half_box,
                request.hard_row_request.acceleration_half_box);
            nominal.y()=std::clamp(nominal.y(),
                -request.hard_row_request.acceleration_half_box,
                request.hard_row_request.acceleration_half_box);
            const auto solved=controller.solve({
                request.profile,owner,cycle+1,1,SupervisorMode::Search,
                nominal,request.hard_row_request.acceleration_half_box,
                rows,request.residual_tolerance});
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
        for (NodeId owner : request.hard_row_request.mobile_ids) {
            auto& state=request.hard_row_request.states.at(owner);
            const Eigen::Vector2d position(state.position.x,state.position.y);
            const auto next=propagateDoubleIntegratorPlanarZoh(
                position,state.velocity,controls.at(owner),request.dt_s);
            state.position=Point(next.position.x(),next.position.y());
            state.velocity=next.velocity;
        }
        for (auto& [id,tube] : request.hard_row_request.reference_snapshot_tubes) {
            (void)id;
            tube.position_radius_m += request.dt_s*tube.velocity_radius_mps;
        }
        for (auto& [id,tube] : request.hard_row_request.collision_snapshot_tubes) {
            (void)id;
            tube.position_radius_m += request.dt_s*tube.velocity_radius_mps;
        }
        for (auto& [id,tube] : request.hard_row_request.workspace_snapshot_tubes) {
            (void)id;
            tube.position_radius_m += request.dt_s*tube.velocity_radius_mps;
        }
        ++result.completed_cycles;
    }
    result.accepted=true;
    result.reason="accepted";
    result.duration_s=request.dt_s*request.cycles;
    result.final_request=std::move(request.hard_row_request);
    return result;
}

}  // namespace gf
