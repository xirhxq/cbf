#pragma once

#include "grand_finale/BoundaryPolicy.hpp"
#include "grand_finale/Types.hpp"
#include "optimisers/optimisers"

#include <map>
#include <memory>

namespace gf {

struct BoundarySoftRow {
    std::string id;
    NodeId owner=0;
    Eigen::Vector2d control_coefficient=Eigen::Vector2d::Zero();
    double constant=0.0;
    double barrier_h=0.0;
    double barrier_psi1=0.0;
    double position_reserve_m=0.0;
    double velocity_reserve_mps=0.0;

    double margin(const Eigen::Vector2d& control,double slack) const {
        return control_coefficient.dot(control)+constant+slack;
    }
};

struct BoundarySoftRowRequest {
    std::vector<NodeId> mobile_ids;
    std::map<NodeId,PairwiseSecondOrderState2D> states;
    std::vector<BoundarySoftFacet2D> facets;
    std::map<NodeId,SingleSnapshotTube2D> snapshot_tubes;
};

inline std::vector<BoundarySoftRow> buildBoundarySoftRows(
    const BoundarySoftRowRequest& request,
    std::optional<NodeId> owner_filter=std::nullopt) {
    std::vector<BoundarySoftRow> rows;
    std::set<std::string> ids;
    for (const auto& raw : request.facets) {
        if (raw.id.empty() || !ids.insert(raw.id).second ||
            !raw.outward_normal.allFinite() ||
            raw.outward_normal.norm()<=1e-12 ||
            !std::isfinite(raw.offset_m))
            throw std::invalid_argument("invalid soft boundary facet");
        const double norm=raw.outward_normal.norm();
        const Eigen::Vector2d normal=raw.outward_normal/norm;
        const double offset=raw.offset_m/norm;
        for (NodeId owner : request.mobile_ids) {
            if (owner_filter.has_value() && owner!=*owner_filter) continue;
            const auto state=request.states.find(owner);
            const auto tube=request.snapshot_tubes.find(owner);
            if (state==request.states.end() || tube==request.snapshot_tubes.end())
                throw std::invalid_argument("missing soft boundary snapshot state");
            if (!std::isfinite(tube->second.position_radius_m) ||
                tube->second.position_radius_m<0.0 ||
                !std::isfinite(tube->second.velocity_radius_mps) ||
                tube->second.velocity_radius_mps<0.0)
                throw std::invalid_argument("invalid soft boundary snapshot tube");
            const Eigen::Vector2d position(
                state->second.position.x,state->second.position.y);
            const double h=offset-normal.dot(position)-
                tube->second.position_radius_m;
            const double hdot=-normal.dot(state->second.velocity)-
                tube->second.velocity_radius_mps;
            rows.push_back({
                "soft-workspace:"+std::to_string(owner)+":"+raw.id,
                owner,-normal,h+2.0*hdot,h,h+hdot,
                tube->second.position_radius_m,
                tube->second.velocity_radius_mps});
        }
    }
    return rows;
}

struct BoundarySoftQpRequest {
    SolverProfile profile=SolverProfile::OpenSource;
    NodeId owner=0;
    Eigen::Vector2d nominal=Eigen::Vector2d::Zero();
    double acceleration_half_box=0.0;
    double slack_weight=1.0;
    std::vector<BoundarySoftRow> rows;
    double residual_tolerance=1e-7;
};

struct BoundarySoftQpResult {
    bool solver_succeeded=false;
    bool nominal_available=false;
    Eigen::Vector2d selected_nominal=Eigen::Vector2d::Zero();
    std::vector<double> slacks;
    double minimum_soft_residual=std::numeric_limits<double>::infinity();
    std::string solver_status="not_run";
    std::string failure_reason;
};

class BoundarySoftNominalSelector {
public:
    BoundarySoftQpResult solve(const BoundarySoftQpRequest& request) const {
        BoundarySoftQpResult result;
        if (request.owner<=0 || !request.nominal.allFinite() ||
            !std::isfinite(request.acceleration_half_box) ||
            request.acceleration_half_box<=0.0 ||
            !std::isfinite(request.slack_weight) ||
            request.slack_weight<=0.0 ||
            !std::isfinite(request.residual_tolerance) ||
            request.residual_tolerance<0.0) {
            result.failure_reason="invalid_soft_boundary_request";
            return result;
        }
        std::vector<BoundarySoftRow> rows;
        for (const auto& row : request.rows) {
            if (row.owner!=request.owner) continue;
            if (row.id.empty() || !row.control_coefficient.allFinite() ||
                !std::isfinite(row.constant)) {
                result.failure_reason="invalid_soft_boundary_row";
                return result;
            }
            rows.push_back(row);
        }
        if (rows.empty()) {
            result.solver_succeeded=true;
            result.nominal_available=true;
            result.selected_nominal=request.nominal.cwiseMax(
                Eigen::Vector2d::Constant(-request.acceleration_half_box))
                .cwiseMin(Eigen::Vector2d::Constant(request.acceleration_half_box));
            return result;
        }
        json settings={{"k_delta",request.slack_weight},
                       {"absolute-tolerance",1e-8},
                       {"relative-tolerance",1e-8},
                       {"primal-infeasibility-tolerance",1e-8},
                       {"dual-infeasibility-tolerance",1e-8}};
        try {
            const std::string name=request.profile==SolverProfile::Gurobi
                ?"Gurobi":"OSQP";
            auto found=optimisers_.find(request.profile);
            if (found==optimisers_.end())
                found=optimisers_.emplace(
                    request.profile,createOptimiser(name,settings)).first;
            else
                found->second->clear();
            auto& optimiser=*found->second;
            const int slack_count=static_cast<int>(rows.size());
            optimiser.start(2+slack_count,2);
            optimiser.setObjective(request.nominal);
            for (int axis=0;axis<2;++axis) {
                Eigen::VectorXd positive=Eigen::VectorXd::Zero(2+slack_count);
                Eigen::VectorXd negative=positive;
                positive(axis)=1.0;
                negative(axis)=-1.0;
                optimiser.addLinearConstraint(
                    positive,-request.acceleration_half_box);
                optimiser.addLinearConstraint(
                    negative,-request.acceleration_half_box);
            }
            for (int index=0;index<slack_count;++index) {
                const auto coefficient=makeSlackConstraintCoefficients(
                    rows[index].control_coefficient,slack_count,index);
                optimiser.addLinearConstraint(coefficient,-rows[index].constant);
            }
            const Eigen::VectorXd solution=optimiser.solve();
            result.solver_status=optimiser.getStatus().value("status","unknown");
            result.solver_succeeded=result.solver_status=="optimal" ||
                result.solver_status=="optimal_inaccurate";
            if (!result.solver_succeeded || solution.size()!=2+slack_count ||
                !solution.allFinite()) {
                result.failure_reason="soft_boundary_solver_failure";
                return result;
            }
            result.selected_nominal=solution.head<2>();
            for (int index=0;index<slack_count;++index) {
                const double slack=solution(2+index);
                result.slacks.push_back(slack);
                result.minimum_soft_residual=std::min(
                    result.minimum_soft_residual,
                    rows[index].margin(result.selected_nominal,slack));
                if (slack<-request.residual_tolerance) {
                    result.failure_reason="soft_boundary_negative_slack";
                    return result;
                }
            }
            if (result.minimum_soft_residual<-request.residual_tolerance) {
                result.failure_reason="soft_boundary_residual_failure";
                return result;
            }
            result.nominal_available=true;
            return result;
        } catch (...) {
            result.failure_reason="soft_boundary_solver_failure";
            return result;
        }
    }

private:
    mutable std::map<SolverProfile,std::unique_ptr<OptimiserBase>> optimisers_;
};

}  // namespace gf
