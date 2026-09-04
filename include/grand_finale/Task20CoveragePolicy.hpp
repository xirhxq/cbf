#pragma once

#include "grand_finale/Task16Cbf2026CoveragePolicy.hpp"
#include "grand_finale/Task20DagLatticeContract.hpp"

#include <chrono>
#include <set>

namespace gf {

enum class Task20TargetPolicy {
    Cbf2026Voronoi,
    LatticeFootprint,
    ResidualShape,
    CoverageWavefront
};

struct Task20CoverageConfig {
    double forward_focus_distance_m=400.0;
    double comparison_epsilon=1.0e-9;
};

struct Task20CoverageRequest {
    Task20DagLatticeContract contract;
    Task20TargetPolicy policy=Task20TargetPolicy::Cbf2026Voronoi;
    std::vector<Task16CoverageAgentState> agents;
    std::vector<FrontierCell> uncovered_cells;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    std::vector<double> initial_distance_m;
    double wavefront_band_width_m=190.0;
    Task20CoverageConfig config;
};

struct Task20CoverageAssignment {
    std::string coverage_unit;
    FrontierCell task;
    Eigen::Vector2d front=Eigen::Vector2d::Zero();
};

struct Task20CoverageResult {
    bool valid=false;
    bool complete=false;
    std::string reason;
    std::map<std::string,Task20CoverageAssignment> assignments;
    std::map<std::string,Eigen::Vector2d> fronts;
    std::map<NodeId,FrontierCell> targets;
    std::size_t active_band=0;
    std::size_t scanned_cells=0;
    double allocation_wall_s=0.0;
};

namespace task20_policy_detail {

inline const Task16CoverageAgentState& agent(
    const std::vector<Task16CoverageAgentState>& agents,NodeId id) {
    const auto found=std::find_if(agents.begin(),agents.end(),
        [&](const auto& value) { return value.id==id; });
    if (found==agents.end()) throw std::invalid_argument("missing Task 20 agent");
    return *found;
}

struct UnitFrontState {
    Eigen::Vector2d position=Eigen::Vector2d::Zero();
    Eigen::Vector2d direction=Eigen::Vector2d{0.0,1.0};
};

inline UnitFrontState unitFront(
    const Task20CoverageUnit& unit,
    const std::vector<Task16CoverageAgentState>& agents) {
    const std::vector<NodeId> members=unit.front_members.empty()
        ?std::vector<NodeId>{unit.leader}:unit.front_members;
    UnitFrontState result;
    Eigen::Vector2d yaw_direction=Eigen::Vector2d::Zero();
    for (NodeId member:members) {
        const auto& state=agent(agents,member);
        result.position+=state.position;
        yaw_direction+=Eigen::Vector2d{std::cos(state.yaw_rad),
                                      std::sin(state.yaw_rad)};
    }
    result.position/=static_cast<double>(members.size());
    if (yaw_direction.norm()>1.0e-9)
        result.direction=yaw_direction.normalized();
    else {
        const auto& leader=agent(agents,unit.leader);
        result.direction={std::cos(leader.yaw_rad),
                          std::sin(leader.yaw_rad)};
    }
    return result;
}

inline const FrontierCell* nearest(const std::vector<const FrontierCell*>& cells,
    const Eigen::Vector2d& focus,double epsilon,
    const std::set<std::string>& used) {
    const FrontierCell* best=nullptr;
    double best_distance=std::numeric_limits<double>::infinity();
    for (const FrontierCell* cell:cells) {
        if (used.count(cell->id())) continue;
        const double distance=(cell->center-focus).squaredNorm();
        if (best==nullptr||distance<best_distance-epsilon||
            (std::abs(distance-best_distance)<=epsilon&&cell->id()<best->id())) {
            best=cell;
            best_distance=distance;
        }
    }
    return best;
}

inline Eigen::Vector2d residualShapeFocus(
    const std::vector<const FrontierCell*>& cells,std::size_t unit_index,
    std::size_t unit_count) {
    Eigen::Vector2d mean=Eigen::Vector2d::Zero();
    for (const auto* cell:cells) mean+=cell->center;
    mean/=static_cast<double>(cells.size());
    Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
    for (const auto* cell:cells) {
        const Eigen::Vector2d delta=cell->center-mean;
        covariance+=delta*delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    Eigen::Vector2d axis=solver.eigenvectors().col(1);
    if (axis.x()<0.0) axis=-axis;
    const double centered=static_cast<double>(unit_index)-
        0.5*static_cast<double>(unit_count-1);
    return mean+centered*400.0*axis;
}

}  // namespace task20_policy_detail

inline std::vector<double> task20InitialEuclideanDistanceField(
    const GridWorld& initial_certified) {
    std::vector<std::pair<int,int>> sources;
    for (int x=0;x<initial_certified.xNum;++x)
        for (int y=0;y<initial_certified.yNum;++y)
            if (initial_certified.vis[static_cast<std::size_t>(
                    x*initial_certified.yNum+y)])
                sources.push_back({x,y});
    if (sources.empty())
        throw std::invalid_argument("Task 20 initial certified set is empty");
    const double dx=(initial_certified.xLim.second-
        initial_certified.xLim.first)/initial_certified.xNum;
    const double dy=(initial_certified.yLim.second-
        initial_certified.yLim.first)/initial_certified.yNum;
    std::vector<double> result(static_cast<std::size_t>(
        initial_certified.xNum*initial_certified.yNum));
    for (int x=0;x<initial_certified.xNum;++x)
        for (int y=0;y<initial_certified.yNum;++y) {
            double best=std::numeric_limits<double>::infinity();
            for (const auto& source:sources) {
                const double sx=dx*(x-source.first);
                const double sy=dy*(y-source.second);
                best=std::min(best,sx*sx+sy*sy);
            }
            result[static_cast<std::size_t>(x*initial_certified.yNum+y)]=
                std::sqrt(best);
        }
    return result;
}

inline Task20CoverageResult allocateTask20Coverage(Task20CoverageRequest request) {
    const auto started=std::chrono::steady_clock::now();
    Task20CoverageResult result;
    const auto finish=[&](Task20CoverageResult value) {
        value.allocation_wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return value;
    };
    if (!request.contract.valid||request.agents.size()!=14||
        request.fixed_positions.size()<3||
        request.config.forward_focus_distance_m<0.0) {
        result.reason="invalid_task20_request";
        return finish(std::move(result));
    }
    if (request.uncovered_cells.empty()) {
        result.valid=true;
        result.complete=true;
        result.reason="certified_t100";
        return finish(std::move(result));
    }
    std::sort(request.uncovered_cells.begin(),request.uncovered_cells.end(),
        [](const auto& first,const auto& second) { return first.id()<second.id(); });
    std::vector<const FrontierCell*> eligible;
    if (request.policy==Task20TargetPolicy::CoverageWavefront) {
        if (request.initial_distance_m.size()!=90000||
            !(request.wavefront_band_width_m>0.0)) {
            result.reason="missing_wavefront_distance_field";
            return finish(std::move(result));
        }
        result.active_band=std::numeric_limits<std::size_t>::max();
        for (const auto& cell:request.uncovered_cells) {
            const std::size_t index=static_cast<std::size_t>(
                cell.x_index*300+cell.y_index);
            result.active_band=std::min(result.active_band,
                static_cast<std::size_t>(std::floor(
                    request.initial_distance_m.at(index)/
                    request.wavefront_band_width_m)));
        }
        for (const auto& cell:request.uncovered_cells) {
            const std::size_t index=static_cast<std::size_t>(
                cell.x_index*300+cell.y_index);
            const auto band=static_cast<std::size_t>(std::floor(
                request.initial_distance_m.at(index)/
                request.wavefront_band_width_m));
            if (band==result.active_band) eligible.push_back(&cell);
        }
    } else {
        for (const auto& cell:request.uncovered_cells) eligible.push_back(&cell);
    }
    result.scanned_cells=eligible.size();
    std::set<std::string> used;
    if (request.policy==Task20TargetPolicy::LatticeFootprint) {
        constexpr int bin_cells=10;
        constexpr int bins_per_axis=30;
        std::array<int,bins_per_axis*bins_per_axis> counts{};
        std::array<const FrontierCell*,bins_per_axis*bins_per_axis> catalog{};
        for (const FrontierCell* cell:eligible) {
            const int bin=(cell->x_index/bin_cells)*bins_per_axis+
                cell->y_index/bin_cells;
            ++counts[bin];
            if (catalog[bin]==nullptr||cell->id()<catalog[bin]->id())
                catalog[bin]=cell;
        }
        std::set<int> claimed_bins;
        for (const auto& unit:request.contract.coverage_units) {
            const auto front=task20_policy_detail::unitFront(
                unit,request.agents);
            const Eigen::Vector2d focus=front.position+
                request.config.forward_focus_distance_m*front.direction;
            const FrontierCell* best=nullptr;
            std::set<int> best_bins;
            int best_gain=-1;
            double best_cost=std::numeric_limits<double>::infinity();
            for (const FrontierCell* candidate:catalog) {
                if (candidate==nullptr||used.count(candidate->id())) continue;
                std::map<std::string,Eigen::Vector2d> fronts;
                for (const auto& value:request.contract.coverage_units)
                    fronts[value.id]=candidate->center;
                const auto lifted=task20LiftTargets(
                    request.contract,request.fixed_positions,fronts);
                if (!lifted.valid) continue;
                Eigen::Vector2d base=Eigen::Vector2d::Zero();
                for (NodeId anchor:unit.base_anchors)
                    base+=request.fixed_positions.at(anchor);
                base/=static_cast<double>(unit.base_anchors.size());
                Eigen::Vector2d heading=candidate->center-base;
                if (heading.norm()<=1.0e-12) heading={0.0,1.0};
                heading.normalize();
                std::set<int> service_bins;
                for (NodeId member:unit.members) {
                    const Eigen::Vector2d center=lifted.targets.at(member);
                    const int bx=static_cast<int>(std::floor(center.x()/100.0));
                    const int by=static_cast<int>(std::floor(center.y()/100.0));
                    for (int x=std::max(0,bx-4);
                         x<=std::min(bins_per_axis-1,bx+4);++x)
                        for (int y=std::max(0,by-4);
                             y<=std::min(bins_per_axis-1,by+4);++y) {
                            const Eigen::Vector2d delta{
                                50.0+100.0*x-center.x(),
                                50.0+100.0*y-center.y()};
                            const double distance=delta.norm();
                            constexpr double reserve=70.71067811865476+0.05;
                            if (distance<=reserve||distance+reserve>400.0)
                                continue;
                            const double cosine=std::clamp(
                                heading.dot(delta/distance),-1.0,1.0);
                            if (std::acos(cosine)+std::asin(reserve/distance)
                                <=M_PI/3.0+1.0e-12)
                                service_bins.insert(x*bins_per_axis+y);
                        }
                }
                int gain=0;
                for (int bin:service_bins)
                    if (!claimed_bins.count(bin)) gain+=counts[bin];
                const double cost=(candidate->center-focus).squaredNorm();
                if (best==nullptr||gain>best_gain||
                    (gain==best_gain&&(cost<best_cost-
                        request.config.comparison_epsilon||
                     (std::abs(cost-best_cost)<=
                        request.config.comparison_epsilon&&
                      candidate->id()<best->id())))) {
                    best=candidate;
                    best_bins=std::move(service_bins);
                    best_gain=gain;
                    best_cost=cost;
                }
            }
            if (best==nullptr) continue;
            used.insert(best->id());
            claimed_bins.insert(best_bins.begin(),best_bins.end());
            result.assignments[unit.id]={unit.id,*best,best->center};
            result.fronts[unit.id]=best->center;
        }
    }
    std::map<std::string,std::vector<const FrontierCell*>> shares;
    if (request.policy==Task20TargetPolicy::ResidualShape) {
        for (const auto& unit:request.contract.coverage_units)
            shares[unit.id]=eligible;
    } else {
        for (const FrontierCell* cell:eligible) {
            const Task20CoverageUnit* selected=nullptr;
            double selected_distance=std::numeric_limits<double>::infinity();
            for (const auto& unit:request.contract.coverage_units) {
                const auto front=task20_policy_detail::unitFront(
                    unit,request.agents);
                const double distance=(cell->center-front.position).squaredNorm();
                if (selected==nullptr||
                    distance<selected_distance-request.config.comparison_epsilon||
                    (std::abs(distance-selected_distance)<=
                        request.config.comparison_epsilon&&unit.id<selected->id)) {
                    selected=&unit;
                    selected_distance=distance;
                }
            }
            shares[selected->id].push_back(cell);
        }
    }
    if (request.policy!=Task20TargetPolicy::LatticeFootprint)
    for (std::size_t unit_index=0;
         unit_index<request.contract.coverage_units.size();++unit_index) {
        const auto& unit=request.contract.coverage_units[unit_index];
        std::vector<const FrontierCell*> candidates=shares[unit.id];
        if (candidates.empty()) candidates=eligible;
        const auto front=task20_policy_detail::unitFront(unit,request.agents);
        Eigen::Vector2d focus=front.position+
            request.config.forward_focus_distance_m*front.direction;
        if (request.policy==Task20TargetPolicy::ResidualShape)
            focus=task20_policy_detail::residualShapeFocus(eligible,unit_index,
                request.contract.coverage_units.size());
        const FrontierCell* task=task20_policy_detail::nearest(candidates,focus,
            request.config.comparison_epsilon,used);
        if (task==nullptr)
            task=task20_policy_detail::nearest(eligible,focus,
                request.config.comparison_epsilon,used);
        if (task==nullptr) continue;
        used.insert(task->id());
        result.assignments[unit.id]={unit.id,*task,task->center};
        result.fronts[unit.id]=task->center;
    }
    if (result.assignments.empty()) {
        result.reason="no_task20_assignment";
        return finish(std::move(result));
    }
    auto complete_fronts=result.fronts;
    for (const auto& unit:request.contract.coverage_units)
        if (!complete_fronts.count(unit.id))
            complete_fronts[unit.id]=result.assignments.begin()->second.front;
    const auto lifted=task20LiftTargets(
        request.contract,request.fixed_positions,complete_fronts);
    if (!lifted.valid) {
        result.reason=lifted.reason;
        return finish(std::move(result));
    }
    for (const auto& [unit_id,assignment]:result.assignments) {
        const std::string sought_unit_id=unit_id;
        const auto unit=std::find_if(request.contract.coverage_units.begin(),
            request.contract.coverage_units.end(),[&](const auto& value) {
                return value.id==sought_unit_id;
            });
        for (NodeId member:unit->members) {
            FrontierCell target=assignment.task;
            target.center=lifted.targets.at(member);
            result.targets[member]=target;
        }
    }
    result.valid=true;
    result.reason="task20_real_cell_assignments";
    return finish(std::move(result));
}

}  // namespace gf
