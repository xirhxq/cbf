#pragma once

#include "grand_finale/Task16Cbf2026CoveragePolicy.hpp"

namespace gf {

enum class Task17PeriodicArm {
    Voronoi,
    SuccessorServiceTime,
    CurrentMemberDistance
};

inline std::size_t task17UpdatePeriodCycles(const std::string& policy) {
    if (policy.find("-p20")!=std::string::npos) return 20;
    if (policy.find("-p10")!=std::string::npos) return 10;
    if (policy.find("-p2")!=std::string::npos) return 2;
    return 5;
}

struct Task17PeriodicCoverageRequest {
    Task17PeriodicArm arm=Task17PeriodicArm::Voronoi;
    std::vector<Task16CoverageAgentState> agents;
    std::vector<Task16CoverageAgentState> approved_successor_agents;
    std::vector<FrontierCell> uncovered_cells;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    Eigen::Vector2d search_min=Eigen::Vector2d::Zero();
    Eigen::Vector2d search_max=Eigen::Vector2d::Zero();
    Task16CoverageConfig config;
    bool reference_compatible_formation=false;
    bool member_aware_wide_formation=false;
    bool coherent_service_wide_formation=false;
};

struct Task17PeriodicCoverageResult {
    bool valid=false;
    bool complete=false;
    std::string reason;
    std::map<std::string,Task16CoverageAssignment> assignments;
    std::map<NodeId,FrontierCell> targets;
    std::map<std::string,NodeId> partition_owner;
    std::size_t active_squads=0;
    std::size_t scanned_member_cell_pairs=0;
    std::size_t duplicate_task_count=0;
    std::uint64_t request_digest=0;
    double allocation_wall_s=0.0;
};

namespace task17_periodic_detail {

inline double currentMemberDistanceSquared(
    const Task13UnifiedCoverageSquad& squad,const FrontierCell& cell,
    const Task17PeriodicCoverageRequest& request) {
    double best=std::numeric_limits<double>::infinity();
    for (const NodeId member:squad.members)
        best=std::min(best,(task16_coverage_detail::agent(
            request.agents,member).position-cell.center).squaredNorm());
    return best;
}

inline double successorServiceTime(
    const Task13UnifiedCoverageSquad& squad,const FrontierCell& cell,
    const Task17PeriodicCoverageRequest& request) {
    Task16CoverageRequest compatible;
    compatible.agents=request.agents;
    compatible.approved_successor_agents=request.approved_successor_agents;
    compatible.config=request.config;
    return task16_coverage_detail::serviceTime(squad,cell,compatible);
}

inline std::optional<task13_unified_detail::FastWitnessGeometry>
referenceCompatibleGeometry(const Task13UnifiedCoverageSquad& squad,
    const FrontierCell& cell,const Task17PeriodicCoverageRequest& request) {
    Task13UnifiedCoverageConfig config;
    config.minimum_half_width_m=7.0;
    config.fan_ratio=0.0075;
    config.reference_limit_m=850.0;
    config.separation_limit_m=10.0;
    config.certified_service_standoff_m=0.0;
    config.compute_nominal_fim_proxy=false;
    std::optional<task13_unified_detail::FastWitnessGeometry> best;
    double best_distance=std::numeric_limits<double>::infinity();
    for (const NodeId member:squad.members) {
        const auto geometry=task13_unified_detail::fastTaperedGeometry(
            squad,member,cell,request.fixed_positions,config);
        if (!geometry.has_value()) continue;
        const double distance=(task16_coverage_detail::agent(
            request.agents,member).position-cell.center).squaredNorm();
        if (!best.has_value()||distance<best_distance-1e-12||
            (std::abs(distance-best_distance)<=1e-12&&
             member<best->responsible_member)) {
            best=geometry;
            best_distance=distance;
        }
    }
    return best;
}

inline std::optional<task13_unified_detail::FastWitnessGeometry>
memberAwareWideGeometry(const Task13UnifiedCoverageSquad& squad,
    const FrontierCell& cell,const Task17PeriodicCoverageRequest& request) {
    Task13UnifiedCoverageConfig config;
    config.separation_limit_m=10.0;
    config.certified_service_standoff_m=0.0;
    config.compute_nominal_fim_proxy=false;
    config.cbf2026_wide_virtual_formation=true;
    std::optional<task13_unified_detail::FastWitnessGeometry> best;
    double best_distance=std::numeric_limits<double>::infinity();
    for (const NodeId member:squad.members) {
        const auto geometry=task13_unified_detail::fastCbf2026WideGeometry(
            squad,member,cell,request.fixed_positions,config);
        if (!geometry.has_value()) continue;
        const double distance=(task16_coverage_detail::agent(
            request.agents,member).position-cell.center).squaredNorm();
        if (!best.has_value()||distance<best_distance-1e-12||
            (std::abs(distance-best_distance)<=1e-12&&
             member<best->responsible_member)) {
            best=geometry;
            best_distance=distance;
        }
    }
    return best;
}

inline const FrontierCell* nearestToFocusExcluding(
    const std::vector<const FrontierCell*>& cells,
    const Eigen::Vector2d& focus,double epsilon,
    const std::optional<std::string>& excluded) {
    const FrontierCell* best=nullptr;
    double best_distance=std::numeric_limits<double>::infinity();
    for (const FrontierCell* cell:cells) {
        if (excluded.has_value()&&cell->id()==*excluded) continue;
        const double distance=(cell->center-focus).squaredNorm();
        if (best==nullptr||distance<best_distance-epsilon||
            (std::abs(distance-best_distance)<=epsilon&&
             cell->id()<best->id())) {
            best=cell;
            best_distance=distance;
        }
    }
    return best;
}

struct FocusedWideChoice {
    const FrontierCell* cell=nullptr;
    NodeId member=0;
    double score=std::numeric_limits<double>::infinity();
};

inline FocusedWideChoice focusedWideChoice(
    const Task13UnifiedCoverageSquad& squad,
    const std::vector<const FrontierCell*>& cells,
    const Task17PeriodicCoverageRequest& request,
    const std::optional<std::string>& excluded) {
    FocusedWideChoice best;
    Task13UnifiedCoverageConfig config;
    config.separation_limit_m=10.0;
    config.certified_service_standoff_m=0.0;
    config.compute_nominal_fim_proxy=false;
    config.cbf2026_wide_virtual_formation=true;
    for (const FrontierCell* cell:cells) {
        if (excluded.has_value()&&cell->id()==*excluded) continue;
        for (const NodeId member:squad.members) {
            if (!task13_unified_detail::fastCbf2026WideGeometry(
                    squad,member,*cell,request.fixed_positions,config).has_value())
                continue;
            const auto& agent=task16_coverage_detail::agent(
                request.agents,member);
            const Eigen::Vector2d focus=agent.position+
                request.config.forward_focus_distance_m*
                Eigen::Vector2d(std::cos(agent.yaw_rad),
                                std::sin(agent.yaw_rad));
            const double score=(cell->center-focus).squaredNorm();
            if (best.cell==nullptr||score<best.score-
                    request.config.comparison_epsilon||
                (std::abs(score-best.score)<=request.config.comparison_epsilon&&
                 std::tuple(cell->id(),member)<
                 std::tuple(best.cell->id(),best.member)))
                best={cell,member,score};
        }
    }
    return best;
}

}  // namespace task17_periodic_detail

inline Task17PeriodicCoverageResult allocateTask17PeriodicCoverage(
    Task17PeriodicCoverageRequest request) {
    const auto started=std::chrono::steady_clock::now();
    Task17PeriodicCoverageResult result;
    const auto finish=[&](Task17PeriodicCoverageResult value) {
        value.allocation_wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return value;
    };
    if (request.agents.size()!=14||
        request.fixed_positions.find(101)==request.fixed_positions.end()||
        !request.search_min.allFinite()||!request.search_max.allFinite()||
        (request.search_min.array()>request.search_max.array()).any()||
        request.config.forward_focus_distance_m<0.0||
        request.config.sensor_outer_radius_m<=0.0||
        request.config.nominal_speed_mps<=0.0||
        request.config.successor_dt_s<=0.0||
        (request.arm==Task17PeriodicArm::SuccessorServiceTime&&
         request.approved_successor_agents.size()!=14)) {
        result.reason="invalid_task17_periodic_request";
        return finish(std::move(result));
    }
    const int formation_modes=
        static_cast<int>(request.reference_compatible_formation)+
        static_cast<int>(request.member_aware_wide_formation)+
        static_cast<int>(request.coherent_service_wide_formation);
    if (formation_modes>1) {
        result.reason="task17_formation_modes_mutually_exclusive";
        return finish(std::move(result));
    }
    std::sort(request.uncovered_cells.begin(),request.uncovered_cells.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
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
    if (request.uncovered_cells.empty()) {
        result.valid=true;
        result.complete=true;
        result.reason="task17_certified_complete";
        return finish(std::move(result));
    }

    const auto squads=task13UnifiedCoverageSquads();
    std::map<std::string,std::vector<const FrontierCell*>> shares;
    std::vector<const FrontierCell*> all;
    all.reserve(request.uncovered_cells.size());
    for (const auto& cell:request.uncovered_cells) {
        all.push_back(&cell);
        double costs[2]{};
        for (std::size_t index=0;index<2;++index) {
            if (request.arm==Task17PeriodicArm::Voronoi) {
                const auto& leader=task16_coverage_detail::agent(
                    request.agents,squads[index].leader);
                costs[index]=(cell.center-leader.position).squaredNorm();
            } else if (request.arm==
                    Task17PeriodicArm::SuccessorServiceTime) {
                costs[index]=task17_periodic_detail::successorServiceTime(
                    squads[index],cell,request);
                result.scanned_member_cell_pairs+=7;
            } else {
                costs[index]=task17_periodic_detail::
                    currentMemberDistanceSquared(squads[index],cell,request);
                result.scanned_member_cell_pairs+=7;
            }
        }
        const std::size_t selected=costs[1]<costs[0]-
            request.config.comparison_epsilon?1:0;
        shares[squads[selected].name].push_back(&cell);
        result.partition_owner[cell.id()]=squads[selected].leader;
    }

    struct Selection {
        const FrontierCell* task=nullptr;
        const FrontierCell* alternative=nullptr;
        Eigen::Vector2d focus=Eigen::Vector2d::Zero();
        NodeId responsible_member=0;
        NodeId alternative_responsible_member=0;
        double score=std::numeric_limits<double>::infinity();
        double alternative_score=std::numeric_limits<double>::infinity();
    };
    std::array<Selection,2> selected;
    for (std::size_t index=0;index<2;++index) {
        const auto& squad=squads[index];
        const auto share=shares.find(squad.name);
        const auto& raw_pool=share==shares.end()||share->second.empty()
            ?all:share->second;
        if (request.coherent_service_wide_formation) {
            auto best=task17_periodic_detail::focusedWideChoice(
                squad,raw_pool,request,std::nullopt);
            if (best.cell==nullptr)
                best=task17_periodic_detail::focusedWideChoice(
                    squad,all,request,std::nullopt);
            if (best.cell==nullptr) {
                result.reason="task17_no_coherent_service_wide_cell";
                return finish(std::move(result));
            }
            const auto alternative=task17_periodic_detail::focusedWideChoice(
                squad,raw_pool,request,best.cell->id());
            selected[index].task=best.cell;
            selected[index].responsible_member=best.member;
            selected[index].score=best.score;
            selected[index].alternative=alternative.cell;
            selected[index].alternative_responsible_member=
                alternative.member;
            selected[index].alternative_score=alternative.score;
            result.scanned_member_cell_pairs+=7*raw_pool.size();
            continue;
        }
        std::vector<const FrontierCell*> compatible_pool;
        if (request.reference_compatible_formation||
            request.member_aware_wide_formation) {
            const auto has_geometry=[&](const FrontierCell& cell) {
                return request.reference_compatible_formation
                    ?task17_periodic_detail::referenceCompatibleGeometry(
                        squad,cell,request).has_value()
                    :task17_periodic_detail::memberAwareWideGeometry(
                        squad,cell,request).has_value();
            };
            for (const FrontierCell* cell:raw_pool)
                if (has_geometry(*cell))
                    compatible_pool.push_back(cell);
            if (compatible_pool.empty())
                for (const FrontierCell* cell:all)
                    if (has_geometry(*cell))
                        compatible_pool.push_back(cell);
        }
        const auto& pool=(request.reference_compatible_formation||
                          request.member_aware_wide_formation)
            ?compatible_pool:raw_pool;
        if (pool.empty()) {
            result.reason="task17_no_reference_compatible_cell";
            return finish(std::move(result));
        }
        const auto& leader=task16_coverage_detail::agent(
            request.agents,squad.leader);
        selected[index].focus=leader.position+
            request.config.forward_focus_distance_m*
            Eigen::Vector2d(std::cos(leader.yaw_rad),
                            std::sin(leader.yaw_rad));
        selected[index].task=task17_periodic_detail::nearestToFocusExcluding(
            pool,selected[index].focus,request.config.comparison_epsilon,
            std::nullopt);
        selected[index].alternative=
            task17_periodic_detail::nearestToFocusExcluding(
                pool,selected[index].focus,request.config.comparison_epsilon,
                selected[index].task->id());
    }
    if (selected[0].task->id()==selected[1].task->id()) {
        const auto penalty=[&](std::size_t index) {
            if (selected[index].alternative==nullptr)
                return std::numeric_limits<double>::infinity();
            if (request.coherent_service_wide_formation)
                return selected[index].alternative_score-
                    selected[index].score;
            return (selected[index].alternative->center-
                    selected[index].focus).squaredNorm()-
                   (selected[index].task->center-
                    selected[index].focus).squaredNorm();
        };
        const double a=penalty(0),b=penalty(1);
        if (a<std::numeric_limits<double>::infinity()||
            b<std::numeric_limits<double>::infinity()) {
            const std::size_t replace=b<a-request.config.comparison_epsilon
                ?1:0;
            if (selected[replace].alternative!=nullptr) {
                selected[replace].task=selected[replace].alternative;
                selected[replace].responsible_member=
                    selected[replace].alternative_responsible_member;
            } else
                selected[1-replace].task=selected[1-replace].alternative;
        }
    }
    result.duplicate_task_count=
        selected[0].task->id()==selected[1].task->id()?1:0;

    const Eigen::Vector2d base=request.fixed_positions.at(101);
    for (std::size_t index=0;index<2;++index) {
        const auto& squad=squads[index];
        Task16CoverageAssignment assignment;
        assignment.squad=squad.name;
        assignment.task=*selected[index].task;
        assignment.endpoint=assignment.task.center;
        std::optional<task13_unified_detail::FastWitnessGeometry> compatible;
        if (request.reference_compatible_formation)
            compatible=task17_periodic_detail::referenceCompatibleGeometry(
                squad,assignment.task,request);
        else if (request.member_aware_wide_formation)
            compatible=task17_periodic_detail::memberAwareWideGeometry(
                squad,assignment.task,request);
        else if (request.coherent_service_wide_formation) {
            Task13UnifiedCoverageConfig config;
            config.separation_limit_m=10.0;
            config.certified_service_standoff_m=0.0;
            config.compute_nominal_fim_proxy=false;
            config.cbf2026_wide_virtual_formation=true;
            compatible=task13_unified_detail::fastCbf2026WideGeometry(
                squad,selected[index].responsible_member,
                assignment.task,request.fixed_positions,config);
        }
        if ((request.reference_compatible_formation||
             request.member_aware_wide_formation||
             request.coherent_service_wide_formation)&&
            !compatible.has_value()) {
            result.reason="task17_reference_compatible_materialization_failed";
            return finish(std::move(result));
        }
        if (compatible.has_value()) {
            assignment.responsible_member=compatible->responsible_member;
            for (std::size_t local=0;local<squad.members.size();++local)
                assignment.target_centers[squad.members[local]]=
                    compatible->targets[local];
        } else assignment.target_centers=task16ForwardTargets(
            squad,base,assignment.endpoint,std::nullopt);
        double responsible_distance=std::numeric_limits<double>::infinity();
        for (const auto& [owner,target]:assignment.target_centers) {
            result.targets[owner]={assignment.task.x_index,
                assignment.task.y_index,target};
            const double distance=(target-assignment.task.center).
                squaredNorm();
            if (!compatible.has_value()&&(
                distance<responsible_distance-1e-12||
                (std::abs(distance-responsible_distance)<=1e-12&&
                 owner<assignment.responsible_member))) {
                responsible_distance=distance;
                assignment.responsible_member=owner;
            }
        }
        if (request.arm==Task17PeriodicArm::SuccessorServiceTime)
            assignment.service_time_s=
                task17_periodic_detail::successorServiceTime(
                    squad,assignment.task,request);
        result.assignments[squad.name]=std::move(assignment);
    }
    result.active_squads=result.assignments.size();
    result.valid=result.assignments.size()==2&&result.targets.size()==14;
    result.reason=result.valid?"task17_periodic_targets":
        "task17_incomplete_periodic_ledger";
    return finish(std::move(result));
}

}  // namespace gf
