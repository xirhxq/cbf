#pragma once

#include "grand_finale/Task15ForwardCoveragePolicy.hpp"
#include "grand_finale/Task16CoverageTypes.hpp"

#include <chrono>
#include <optional>
#include <sstream>

namespace gf {

inline const char* task16Cbf2026SourceCommit() {
    return "47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d";
}

struct Task16SearchBounds {
    Eigen::Vector2d minimum=Eigen::Vector2d::Zero();
    Eigen::Vector2d maximum=Eigen::Vector2d::Zero();
};

struct Task16CoverageAgentState {
    NodeId id=0;
    Eigen::Vector2d position=Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity=Eigen::Vector2d::Zero();
    double yaw_rad=0.0;
    double position_error_bound_m=0.0;
};

struct Task16CoverageConfig {
    double forward_focus_distance_m=400.0;
    double sensor_inner_radius_m=0.0;
    double sensor_outer_radius_m=400.0;
    double sensor_half_angle_rad=M_PI/3.0;
    double cell_half_diagonal_m=5.0*std::sqrt(2.0);
    double nominal_speed_mps=30.0;
    double successor_dt_s=0.1;
    double comparison_epsilon=1.0e-9;
};

struct Task16CoverageAssignment {
    std::string squad;
    FrontierCell task;
    Eigen::Vector2d endpoint=Eigen::Vector2d::Zero();
    NodeId responsible_member=0;
    std::map<NodeId,Eigen::Vector2d> target_centers;
    double service_time_s=std::numeric_limits<double>::infinity();
};

struct Task16CoverageRequest {
    Task16CoverageArm arm=Task16CoverageArm::HistoricalClipped;
    std::vector<Task16CoverageAgentState> agents;
    std::vector<Task16CoverageAgentState> approved_successor_agents;
    std::vector<FrontierCell> uncovered_cells;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    Eigen::Vector2d search_min=Eigen::Vector2d::Zero();
    Eigen::Vector2d search_max=Eigen::Vector2d::Zero();
    Task16CoverageConfig config;
};

struct Task16CoverageResult {
    bool valid=false;
    std::string reason;
    std::map<std::string,Task16CoverageAssignment> assignments;
    std::map<NodeId,FrontierCell> targets;
    std::map<std::string,NodeId> voronoi_owner;
    std::map<std::string,NodeId> service_owner;
    std::size_t active_squads=0;
    std::size_t scanned_member_cell_pairs=0;
    std::uint64_t request_digest=0;
    double allocation_wall_s=0.0;
};

inline std::map<NodeId,Eigen::Vector2d> task16ForwardTargets(
    const Task13UnifiedCoverageSquad& squad,const Eigen::Vector2d& base,
    const Eigen::Vector2d& endpoint,
    std::optional<Task16SearchBounds> clip_bounds) {
    auto targets=task15ForwardTargets(squad,base,endpoint);
    if (clip_bounds.has_value())
        for (auto& [owner,target]:targets) {
            (void)owner;
            target.x()=std::clamp(target.x(),clip_bounds->minimum.x(),
                clip_bounds->maximum.x());
            target.y()=std::clamp(target.y(),clip_bounds->minimum.y(),
                clip_bounds->maximum.y());
        }
    return targets;
}

namespace task16_coverage_detail {

inline const Task16CoverageAgentState& agent(
    const std::vector<Task16CoverageAgentState>& agents,NodeId id) {
    const auto found=std::find_if(agents.begin(),agents.end(),
        [&](const auto& item) { return item.id==id; });
    if (found==agents.end())
        throw std::invalid_argument("missing Task 16 agent");
    return *found;
}

inline std::uint64_t hashText(const std::string& text) {
    std::uint64_t hash=1469598103934665603ULL;
    for (const unsigned char value:text) {
        hash^=value;
        hash*=1099511628211ULL;
    }
    return hash==0?1:hash;
}

inline double serviceTime(
    const Task13UnifiedCoverageSquad& squad,const FrontierCell& cell,
    const Task16CoverageRequest& request) {
    double best=std::numeric_limits<double>::infinity();
    for (NodeId member:squad.members) {
        const auto& current=agent(request.agents,member);
        if (task15CertifiedSectorContains(current.position,current.yaw_rad,
                current.position_error_bound_m,cell,
                {32,16,250.0,request.config.sensor_inner_radius_m,
                 request.config.sensor_outer_radius_m,
                 request.config.sensor_half_angle_rad,
                 request.config.cell_half_diagonal_m}))
            best=0.0;
        const auto& successor=agent(request.approved_successor_agents,member);
        if (task15CertifiedSectorContains(successor.position,
                successor.yaw_rad,successor.position_error_bound_m,cell,
                {32,16,250.0,request.config.sensor_inner_radius_m,
                 request.config.sensor_outer_radius_m,
                 request.config.sensor_half_angle_rad,
                 request.config.cell_half_diagonal_m}))
            best=std::min(best,request.config.successor_dt_s);
        const double effective_radius=std::max(0.0,
            request.config.sensor_outer_radius_m-
            successor.position_error_bound_m-
            request.config.cell_half_diagonal_m);
        const double travel=std::max(0.0,
            (cell.center-successor.position).norm()-effective_radius);
        best=std::min(best,request.config.successor_dt_s+
            travel/request.config.nominal_speed_mps);
    }
    return best;
}

inline const FrontierCell& nearestToFocus(
    const std::vector<const FrontierCell*>& cells,
    const Eigen::Vector2d& focus,double epsilon) {
    const FrontierCell* best=cells.front();
    double best_distance=(best->center-focus).squaredNorm();
    for (const FrontierCell* cell:cells) {
        const double distance=(cell->center-focus).squaredNorm();
        if (distance<best_distance-epsilon ||
            (std::abs(distance-best_distance)<=epsilon &&
             cell->id()<best->id())) {
            best=cell;
            best_distance=distance;
        }
    }
    return *best;
}

}  // namespace task16_coverage_detail

inline Task16CoverageResult allocateTask16Cbf2026Coverage(
    Task16CoverageRequest request) {
    const auto started=std::chrono::steady_clock::now();
    Task16CoverageResult result;
    const auto finish=[&](Task16CoverageResult value) {
        value.allocation_wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return value;
    };
    if (request.agents.size()!=14 || request.uncovered_cells.empty() ||
        !request.search_min.allFinite() || !request.search_max.allFinite() ||
        (request.search_min.array()>request.search_max.array()).any() ||
        request.fixed_positions.find(101)==request.fixed_positions.end() ||
        request.config.forward_focus_distance_m<0.0 ||
        request.config.sensor_outer_radius_m<=0.0 ||
        request.config.nominal_speed_mps<=0.0 ||
        request.config.successor_dt_s<=0.0) {
        result.reason="invalid_task16_coverage_request";
        return finish(std::move(result));
    }
    std::sort(request.uncovered_cells.begin(),request.uncovered_cells.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
    const auto squads=task13UnifiedCoverageSquads();
    if (request.arm==Task16CoverageArm::FormationAware &&
        request.approved_successor_agents.size()!=14) {
        result.reason="missing_task16_approved_successor";
        return finish(std::move(result));
    }
    std::map<std::string,std::vector<const FrontierCell*>> shares;
    for (const auto& cell:request.uncovered_cells) {
        std::size_t selected=0;
        if (request.arm==Task16CoverageArm::FormationAware) {
            const double a=task16_coverage_detail::serviceTime(
                squads[0],cell,request);
            const double b=task16_coverage_detail::serviceTime(
                squads[1],cell,request);
            selected=b<a-request.config.comparison_epsilon?1:0;
            result.service_owner[cell.id()]=squads[selected].leader;
            result.scanned_member_cell_pairs+=14;
        } else {
            const auto& a=task16_coverage_detail::agent(
                request.agents,squads[0].leader);
            const auto& b=task16_coverage_detail::agent(
                request.agents,squads[1].leader);
            const double da=(cell.center-a.position).squaredNorm();
            const double db=(cell.center-b.position).squaredNorm();
            selected=db<da-request.config.comparison_epsilon?1:0;
            result.voronoi_owner[cell.id()]=squads[selected].leader;
        }
        shares[squads[selected].name].push_back(&cell);
    }
    const Eigen::Vector2d base=request.fixed_positions.at(101);
    for (const auto& squad:squads) {
        const auto share=shares.find(squad.name);
        if (share==shares.end() || share->second.empty()) continue;
        const auto& leader=task16_coverage_detail::agent(
            request.agents,squad.leader);
        const Eigen::Vector2d focus=leader.position+
            request.config.forward_focus_distance_m*
            Eigen::Vector2d(std::cos(leader.yaw_rad),
                            std::sin(leader.yaw_rad));
        const auto& task=task16_coverage_detail::nearestToFocus(
            share->second,focus,request.config.comparison_epsilon);
        Task16CoverageAssignment assignment;
        assignment.squad=squad.name;
        assignment.task=task;
        assignment.endpoint=task.center;
        assignment.target_centers=task16ForwardTargets(squad,base,
            assignment.endpoint,
            request.arm==Task16CoverageArm::HistoricalClipped
                ?std::optional<Task16SearchBounds>{{request.search_min,
                                                    request.search_max}}
                :std::nullopt);
        double responsible_distance=std::numeric_limits<double>::infinity();
        for (const auto& [owner,target]:assignment.target_centers) {
            result.targets[owner]={task.x_index,task.y_index,target};
            const double distance=(target-task.center).squaredNorm();
            if (distance<responsible_distance-1e-12 ||
                (std::abs(distance-responsible_distance)<=1e-12 &&
                 owner<assignment.responsible_member)) {
                responsible_distance=distance;
                assignment.responsible_member=owner;
            }
        }
        if (request.arm==Task16CoverageArm::FormationAware)
            assignment.service_time_s=task16_coverage_detail::serviceTime(
                squad,task,request);
        result.assignments[squad.name]=std::move(assignment);
    }
    std::ostringstream canonical;
    canonical.precision(17);
    canonical<<static_cast<int>(request.arm)<<'|';
    for (const auto& agent:request.agents)
        canonical<<agent.id<<':'<<agent.position.transpose()<<':'
                 <<agent.yaw_rad<<';';
    canonical<<'|';
    for (const auto& cell:request.uncovered_cells)
        canonical<<cell.id()<<':'<<cell.center.transpose()<<';';
    result.request_digest=task16_coverage_detail::hashText(canonical.str());
    result.active_squads=result.assignments.size();
    result.valid=!result.assignments.empty();
    result.reason=result.valid?"task16_cbf2026_targets":
        "task16_no_assigned_squad";
    return finish(std::move(result));
}

}  // namespace gf
