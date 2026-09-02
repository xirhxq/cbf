#pragma once

#include "grand_finale/Task13UnifiedCoveragePolicy.hpp"

#include <array>
#include <map>
#include <set>
#include <vector>

namespace gf {

struct Task15ForwardGeometry {
    std::map<NodeId,Eigen::Vector2d> targets;
    double maximum_reference_edge_m=0.0;
    double minimum_target_separation_m=
        std::numeric_limits<double>::infinity();
    double nominal_fim_proxy=0.0;
};

struct Task15ForwardCoverageConfig {
    std::size_t shortlist_capacity=32;
    std::size_t bearing_bins=16;
    double endpoint_standoff_m=250.0;
    double sensor_inner_radius_m=0.0;
    double sensor_outer_radius_m=400.0;
    double sensor_half_angle_rad=M_PI/3.0;
    double cell_half_diagonal_m=5.0*std::sqrt(2.0);
    double level_a_reference_margin_m=1.0;
    double reference_limit_m=850.0;
    double target_separation_m=10.0;
    double nominal_speed_mps=30.0;
    double braking_acceleration_mps2=4.0;
    double movement_time_epsilon_s=0.1;
    double comparison_epsilon=1.0e-9;
};

struct Task15ForwardAgentState {
    NodeId id=0;
    Eigen::Vector2d position=Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity=Eigen::Vector2d::Zero();
    double yaw_rad=0.0;
    double position_error_bound_m=0.0;
};

struct Task15ForwardCoverageCandidate {
    std::string squad;
    FrontierCell task;
    FrontierCell endpoint;
    NodeId responsible_member=0;
    std::map<NodeId,Eigen::Vector2d> targets;
    std::map<NodeId,std::string> target_ids;
    std::vector<std::string> predicted_cell_ids;
    std::size_t predicted_new_certified_cells=0;
    double predicted_yaw_rad=0.0;
    double movement_time_s=0.0;
    double utility=0.0;
    double maximum_reference_edge_m=0.0;
    double reference_overrun_m=0.0;
    double minimum_target_separation_m=
        std::numeric_limits<double>::infinity();
    double nominal_fim_proxy=0.0;
    double maximum_target_displacement_m=0.0;
    bool level_a=false;
};

struct Task15ForwardCoverageRequest {
    std::vector<Task15ForwardAgentState> agents;
    std::vector<FrontierCell> uncovered_cells;
    std::vector<FrontierCell> domain_cells;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    std::map<std::string,Task15ForwardCoverageCandidate> retained;
    Task15ForwardCoverageConfig config;
};

struct Task15ForwardCoverageAssignment {
    Task15ForwardCoverageCandidate candidate;
    bool active=false;
    std::string reason;
    bool persistent=false;
};

struct Task15ForwardCoverageResult {
    bool valid=false;
    std::string reason;
    std::map<std::string,Task15ForwardCoverageAssignment> assignments;
    std::map<NodeId,FrontierCell> targets;
    std::size_t active_squads=0;
    std::size_t level_a_squads=0;
    std::size_t evaluated_candidates=0;
    std::size_t evaluated_joint_combinations=0;
    double combined_utility=0.0;
    double minimum_cross_target_separation_m=
        std::numeric_limits<double>::infinity();
};

inline std::map<NodeId,Eigen::Vector2d> task15ForwardTargets(
    const Task13UnifiedCoverageSquad& squad,const Eigen::Vector2d& base,
    const Eigen::Vector2d& endpoint) {
    static constexpr std::array<double,7> axial{1,1,2,2,3,3,4};
    static constexpr std::array<double,7> triangular{0,1,0,1,0,1,0};
    const Eigen::Vector2d section=(endpoint-base)/4.0;
    const Eigen::Rotation2Dd rotate(squad.mirror_sign*M_PI/3.0);
    std::map<NodeId,Eigen::Vector2d> targets;
    for (std::size_t local=0;local<squad.members.size();++local)
        targets[squad.members[local]]=base+axial[local]*section+
            triangular[local]*(rotate*section);
    return targets;
}

inline Task15ForwardGeometry task15EvaluateForwardGeometry(
    const Task13UnifiedCoverageSquad& squad,const Eigen::Vector2d& base,
    const Eigen::Vector2d& endpoint,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions) {
    Task15ForwardGeometry value;
    value.targets=task15ForwardTargets(squad,base,endpoint);
    for (const auto& edge:squad.edges) {
        const auto owner=value.targets.at(edge.owner);
        const auto mobile=value.targets.find(edge.reference);
        const auto reference=mobile!=value.targets.end()
            ?mobile->second:fixed_positions.at(edge.reference);
        value.maximum_reference_edge_m=std::max(
            value.maximum_reference_edge_m,(owner-reference).norm());
    }
    for (const auto& [owner,target]:value.targets) {
        for (const auto& [other,other_target]:value.targets)
            if (other>owner) value.minimum_target_separation_m=std::min(
                value.minimum_target_separation_m,
                (target-other_target).norm());
        for (const auto& [fixed_id,fixed_target]:fixed_positions) {
            (void)fixed_id;
            value.minimum_target_separation_m=std::min(
                value.minimum_target_separation_m,
                (target-fixed_target).norm());
        }
    }
    value.nominal_fim_proxy=task13_unified_detail::nominalFimProxy(
        squad,value.targets,fixed_positions);
    return value;
}

inline bool task15CertifiedSectorContains(
    const Eigen::Vector2d& center,double yaw_rad,double error_bound_m,
    const FrontierCell& cell,const Task15ForwardCoverageConfig& config) {
    const double reserve=error_bound_m+config.cell_half_diagonal_m;
    const Eigen::Vector2d delta=cell.center-center;
    const double distance=delta.norm();
    if (distance+reserve>config.sensor_outer_radius_m+1e-12 ||
        (config.sensor_inner_radius_m>1e-12 &&
         distance-reserve<config.sensor_inner_radius_m-1e-12)) return false;
    if (config.sensor_half_angle_rad>=M_PI-1e-12) return true;
    if (distance<=reserve) return false;
    const double bearing=std::atan2(delta.y(),delta.x());
    const double error=std::abs(std::atan2(
        std::sin(bearing-yaw_rad),std::cos(bearing-yaw_rad)));
    return error+std::asin(std::min(1.0,reserve/distance))<=
        config.sensor_half_angle_rad+1e-12;
}

inline const Task15ForwardAgentState& task15Agent(
    const Task15ForwardCoverageRequest& request,NodeId id) {
    const auto found=std::find_if(request.agents.begin(),request.agents.end(),
        [&](const auto& value) { return value.id==id; });
    if (found==request.agents.end())
        throw std::invalid_argument("missing task15 agent");
    return *found;
}

inline FrontierCell task15EndpointForTask(
    const FrontierCell& task,const Eigen::Vector2d& base,
    const Task15ForwardCoverageRequest& request) {
    if (request.domain_cells.empty())
        throw std::invalid_argument("empty task15 endpoint domain");
    const Eigen::Vector2d radial=task.center-base;
    const Eigen::Vector2d desired=radial.norm()>1e-12
        ?task.center-request.config.endpoint_standoff_m*radial.normalized()
        :task.center;
    const FrontierCell* best=&request.domain_cells.front();
    double best_distance=(best->center-desired).squaredNorm();
    for (const auto& cell:request.domain_cells) {
        const double distance=(cell.center-desired).squaredNorm();
        if (distance<best_distance-1e-12 ||
            (std::abs(distance-best_distance)<=1e-12 &&
             std::tie(cell.x_index,cell.y_index)<
             std::tie(best->x_index,best->y_index))) {
            best=&cell;
            best_distance=distance;
        }
    }
    return *best;
}

inline std::optional<Task15ForwardCoverageCandidate>
task15ForwardCandidateForEndpoint(
    const Task13UnifiedCoverageSquad& squad,const FrontierCell& task,
    const FrontierCell& endpoint,
    const Task15ForwardCoverageRequest& request) {
    const auto base_it=request.fixed_positions.find(101);
    if (base_it==request.fixed_positions.end() || request.agents.size()!=14 ||
        request.uncovered_cells.empty()) return std::nullopt;
    Task15ForwardCoverageCandidate value;
    value.squad=squad.name;
    value.task=task;
    value.endpoint=endpoint;
    const auto geometry=task15EvaluateForwardGeometry(
        squad,base_it->second,value.endpoint.center,request.fixed_positions);
    value.targets=geometry.targets;
    value.maximum_reference_edge_m=geometry.maximum_reference_edge_m;
    value.minimum_target_separation_m=geometry.minimum_target_separation_m;
    value.nominal_fim_proxy=geometry.nominal_fim_proxy;
    if (!(value.minimum_target_separation_m>
          request.config.target_separation_m+
          request.config.comparison_epsilon)) return std::nullopt;
    const Eigen::Vector2d direction=value.endpoint.center-base_it->second;
    value.predicted_yaw_rad=direction.norm()>1e-12
        ?std::atan2(direction.y(),direction.x()):M_PI/2.0;
    const double level_a_limit=request.config.reference_limit_m-
        request.config.level_a_reference_margin_m;
    value.level_a=value.maximum_reference_edge_m<
        level_a_limit-request.config.comparison_epsilon;
    value.reference_overrun_m=std::max(
        0.0,value.maximum_reference_edge_m-level_a_limit);
    double maximum_speed=0.0;
    for (NodeId member:squad.members) {
        const auto& agent=task15Agent(request,member);
        value.maximum_target_displacement_m=std::max(
            value.maximum_target_displacement_m,
            (value.targets.at(member)-agent.position).norm());
        maximum_speed=std::max(maximum_speed,agent.velocity.norm());
        value.target_ids[member]=task.id();
    }
    value.movement_time_s=value.maximum_target_displacement_m/
        request.config.nominal_speed_mps+maximum_speed/
        request.config.braking_acceleration_mps2;
    double responsible_distance=std::numeric_limits<double>::infinity();
    for (const auto& cell:request.uncovered_cells) {
        bool serviced=false;
        for (NodeId member:squad.members) {
            const auto& agent=task15Agent(request,member);
            if (!task15CertifiedSectorContains(value.targets.at(member),
                    value.predicted_yaw_rad,agent.position_error_bound_m,
                    cell,request.config)) continue;
            serviced=true;
            if (cell.id()==task.id()) {
                const double distance=(cell.center-
                    value.targets.at(member)).norm();
                if (distance<responsible_distance-1e-12 ||
                    (std::abs(distance-responsible_distance)<=1e-12 &&
                     member<value.responsible_member)) {
                    responsible_distance=distance;
                    value.responsible_member=member;
                }
            }
        }
        if (serviced) value.predicted_cell_ids.push_back(cell.id());
    }
    value.predicted_new_certified_cells=value.predicted_cell_ids.size();
    if (value.responsible_member==0 ||
        value.predicted_new_certified_cells==0) return std::nullopt;
    value.utility=static_cast<double>(value.predicted_new_certified_cells)/
        (value.movement_time_s+request.config.movement_time_epsilon_s);
    return value;
}

inline std::optional<Task15ForwardCoverageCandidate>
task15ForwardCandidateForTask(
    const Task13UnifiedCoverageSquad& squad,const FrontierCell& task,
    const Task15ForwardCoverageRequest& request) {
    const auto base_it=request.fixed_positions.find(101);
    if (base_it==request.fixed_positions.end()) return std::nullopt;
    return task15ForwardCandidateForEndpoint(squad,task,
        task15EndpointForTask(task,base_it->second,request),request);
}

inline bool task15RetainWithinUtilityBand(
    const Task15ForwardCoverageCandidate& retained,
    const Task15ForwardCoverageCandidate& proposed,double fraction) {
    return retained.predicted_new_certified_cells>0 &&
        retained.level_a==proposed.level_a && fraction>=0.0 &&
        retained.utility+1e-12>=proposed.utility*(1.0-fraction);
}

inline std::size_t task15ReselectionAttemptLimit(
    const Task15ForwardCoverageConfig& config) {
    return config.shortlist_capacity;
}

inline bool task15CandidateLess(
    const Task15ForwardCoverageCandidate& lhs,
    const Task15ForwardCoverageCandidate& rhs) {
    if (lhs.level_a!=rhs.level_a) return lhs.level_a;
    if (!lhs.level_a && std::abs(lhs.reference_overrun_m-
            rhs.reference_overrun_m)>1e-12)
        return lhs.reference_overrun_m<rhs.reference_overrun_m;
    if (std::abs(lhs.utility-rhs.utility)>1e-12)
        return lhs.utility>rhs.utility;
    if (std::abs(lhs.nominal_fim_proxy-rhs.nominal_fim_proxy)>1e-12)
        return lhs.nominal_fim_proxy>rhs.nominal_fim_proxy;
    if (std::abs(lhs.maximum_target_displacement_m-
            rhs.maximum_target_displacement_m)>1e-12)
        return lhs.maximum_target_displacement_m<
            rhs.maximum_target_displacement_m;
    return std::tie(lhs.task.x_index,lhs.task.y_index,
                    lhs.endpoint.x_index,lhs.endpoint.y_index,lhs.squad)<
           std::tie(rhs.task.x_index,rhs.task.y_index,
                    rhs.endpoint.x_index,rhs.endpoint.y_index,rhs.squad);
}

inline std::vector<FrontierCell> task15EndpointShortlist(
    const std::vector<FrontierCell>& uncovered,const Eigen::Vector2d& base,
    const Task15ForwardCoverageConfig& config) {
    std::vector<FrontierCell> sorted=uncovered;
    std::sort(sorted.begin(),sorted.end(),[](const auto& lhs,const auto& rhs) {
        return std::tie(lhs.x_index,lhs.y_index)<
               std::tie(rhs.x_index,rhs.y_index);
    });
    if (sorted.size()<=config.shortlist_capacity) return sorted;
    struct BinValue {
        const FrontierCell* nearest=nullptr;
        const FrontierCell* farthest=nullptr;
        double nearest_r2=std::numeric_limits<double>::infinity();
        double farthest_r2=-1.0;
    };
    std::vector<BinValue> bins(config.bearing_bins);
    const auto identity=[](const FrontierCell* value) {
        return std::tie(value->x_index,value->y_index);
    };
    for (const auto& cell:sorted) {
        const Eigen::Vector2d delta=cell.center-base;
        const double angle=std::atan2(delta.y(),delta.x());
        const double scaled=(angle+M_PI)/(2.0*M_PI)*
            static_cast<double>(config.bearing_bins);
        const std::size_t index=std::min(config.bearing_bins-1,
            static_cast<std::size_t>(std::max(0.0,std::floor(scaled))));
        const double r2=delta.squaredNorm();
        auto& bin=bins[index];
        if (!bin.nearest || r2<bin.nearest_r2-1e-12 ||
            (std::abs(r2-bin.nearest_r2)<=1e-12 &&
             identity(&cell)<identity(bin.nearest))) {
            bin.nearest=&cell;
            bin.nearest_r2=r2;
        }
        if (!bin.farthest || r2>bin.farthest_r2+1e-12 ||
            (std::abs(r2-bin.farthest_r2)<=1e-12 &&
             identity(&cell)<identity(bin.farthest))) {
            bin.farthest=&cell;
            bin.farthest_r2=r2;
        }
    }
    std::vector<FrontierCell> result;
    std::set<std::string> seen;
    for (const auto& bin:bins) for (const FrontierCell* cell:
         {bin.nearest,bin.farthest})
        if (cell && seen.insert(cell->id()).second) result.push_back(*cell);
    if (result.size()>config.shortlist_capacity)
        result.resize(config.shortlist_capacity);
    return result;
}

inline double task15CrossTargetSeparation(
    const Task15ForwardCoverageCandidate& first,
    const Task15ForwardCoverageCandidate& second) {
    double minimum=std::numeric_limits<double>::infinity();
    for (const auto& [first_id,a]:first.targets) {
        (void)first_id;
        for (const auto& [second_id,b]:second.targets) {
            (void)second_id;
            minimum=std::min(minimum,(a-b).norm());
        }
    }
    return minimum;
}

inline double task15TargetLedgerMinimumSeparation(
    const std::map<NodeId,Eigen::Vector2d>& targets,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions) {
    double minimum=std::numeric_limits<double>::infinity();
    for (auto first=targets.begin();first!=targets.end();++first) {
        for (auto second=std::next(first);second!=targets.end();++second)
            minimum=std::min(minimum,
                (first->second-second->second).norm());
        for (const auto& [fixed_id,fixed]:fixed_positions) {
            (void)fixed_id;
            minimum=std::min(minimum,(first->second-fixed).norm());
        }
    }
    return minimum;
}

inline Task15ForwardCoverageResult allocateTask15ForwardCoverage(
    Task15ForwardCoverageRequest request) {
    Task15ForwardCoverageResult result;
    if (request.agents.size()!=14 || request.domain_cells.empty() ||
        request.fixed_positions.count(100)==0 ||
        request.fixed_positions.count(101)==0 ||
        request.fixed_positions.count(102)==0 ||
        request.config.shortlist_capacity==0 ||
        request.config.bearing_bins==0) {
        result.reason="invalid_task15_forward_request";
        return result;
    }
    std::set<std::string> task_ids;
    for (const auto& cell:request.uncovered_cells)
        if (cell.x_index<0 || cell.y_index<0 || !cell.center.allFinite() ||
            !task_ids.insert(cell.id()).second) {
            result.reason="invalid_task15_uncovered_cells";
            return result;
        }
    const auto squads=task13UnifiedCoverageSquads();
    std::map<std::string,std::vector<Task15ForwardCoverageAssignment>> choices;
    const auto seeds=task15EndpointShortlist(request.uncovered_cells,
        request.fixed_positions.at(101),request.config);
    for (const auto& squad:squads) {
        std::vector<Task15ForwardCoverageCandidate> active;
        for (const auto& task:seeds) {
            const auto candidate=task15ForwardCandidateForTask(
                squad,task,request);
            ++result.evaluated_candidates;
            if (candidate.has_value()) active.push_back(*candidate);
        }
        const bool has_level_a=std::any_of(active.begin(),active.end(),
            [](const auto& value) { return value.level_a; });
        if (has_level_a) active.erase(std::remove_if(
            active.begin(),active.end(),[](const auto& value) {
                return !value.level_a;
            }),active.end());
        std::sort(active.begin(),active.end(),task15CandidateLess);
        for (auto& candidate:active)
            choices[squad.name].push_back(
                {std::move(candidate),true,"forward_positive_gain"});
        const auto retained=request.retained.find(squad.name);
        if (retained!=request.retained.end()) {
            const auto task=std::find_if(request.uncovered_cells.begin(),
                request.uncovered_cells.end(),[&](const auto& cell) {
                    return cell.id()==retained->second.task.id();
                });
            if (task!=request.uncovered_cells.end() &&
                !choices[squad.name].empty()) {
                const auto refreshed=task15ForwardCandidateForEndpoint(
                    squad,*task,retained->second.endpoint,request);
                if (refreshed.has_value() &&
                    task15RetainWithinUtilityBand(*refreshed,
                        choices[squad.name].front().candidate,0.05))
                    choices[squad.name].insert(choices[squad.name].begin(),
                        {*refreshed,true,"retain_within_5pct",true});
            }
            choices[squad.name].push_back(
                {retained->second,false,"retain_last_legal_target"});
        }
    }
    struct Joint {
        Task15ForwardCoverageAssignment a,b;
        std::size_t active=0;
        std::size_t level_a=0;
        std::size_t persistent=0;
        double utility=0.0;
        double fim=0.0;
        double displacement=0.0;
        double cross=std::numeric_limits<double>::infinity();
    };
    std::optional<Joint> best;
    const auto joint_identity=[](const Joint& value) {
        return std::tuple(value.a.candidate.task.x_index,
            value.a.candidate.task.y_index,value.a.candidate.endpoint.x_index,
            value.a.candidate.endpoint.y_index,
            value.b.candidate.task.x_index,value.b.candidate.task.y_index,
            value.b.candidate.endpoint.x_index,
            value.b.candidate.endpoint.y_index);
    };
    const auto better=[&](const Joint& lhs,const Joint& rhs) {
        if (lhs.active!=rhs.active) return lhs.active>rhs.active;
        if (lhs.level_a!=rhs.level_a) return lhs.level_a>rhs.level_a;
        if (lhs.persistent!=rhs.persistent)
            return lhs.persistent>rhs.persistent;
        if (std::abs(lhs.utility-rhs.utility)>1e-12)
            return lhs.utility>rhs.utility;
        if (std::abs(lhs.fim-rhs.fim)>1e-12) return lhs.fim>rhs.fim;
        if (std::abs(lhs.displacement-rhs.displacement)>1e-12)
            return lhs.displacement<rhs.displacement;
        return joint_identity(lhs)<joint_identity(rhs);
    };
    for (const auto& a:choices["A"]) for (const auto& b:choices["B"]) {
        ++result.evaluated_joint_combinations;
        if (a.active && b.active && a.candidate.task.id()==
            b.candidate.task.id()) continue;
        const double cross=task15CrossTargetSeparation(
            a.candidate,b.candidate);
        if (!(cross>request.config.target_separation_m+
              request.config.comparison_epsilon)) continue;
        Joint value{a,b,static_cast<std::size_t>(a.active)+
            static_cast<std::size_t>(b.active),
            static_cast<std::size_t>(a.active&&a.candidate.level_a)+
            static_cast<std::size_t>(b.active&&b.candidate.level_a),
            static_cast<std::size_t>(a.persistent)+
            static_cast<std::size_t>(b.persistent),
            (a.active?a.candidate.utility:0.0)+
            (b.active?b.candidate.utility:0.0),
            a.candidate.nominal_fim_proxy+b.candidate.nominal_fim_proxy,
            a.candidate.maximum_target_displacement_m+
            b.candidate.maximum_target_displacement_m,cross};
        if (!best.has_value() || better(value,*best)) best=std::move(value);
    }
    if (!best.has_value()) {
        result.reason=request.uncovered_cells.empty()
            ?"no_retained_task15_configuration"
            :"no_joint_task15_forward_assignment";
        return result;
    }
    result.assignments["A"]=best->a;
    result.assignments["B"]=best->b;
    result.active_squads=best->active;
    result.level_a_squads=best->level_a;
    result.combined_utility=best->utility;
    result.minimum_cross_target_separation_m=best->cross;
    for (const auto* assignment:{&best->a,&best->b})
        for (const auto& [member,target]:assignment->candidate.targets)
            result.targets[member]={assignment->candidate.task.x_index,
                assignment->candidate.task.y_index,target};
    result.valid=result.targets.size()==14;
    result.reason=result.valid?"task15_forward_assignment_selected":
        "incomplete_task15_target_ledger";
    return result;
}

}  // namespace gf
