#pragma once

// Task 23: DAG-agnostic Persistent Deadline Ribbon (PDR).
//
// This header owns a research-only, default-off planning path.  The online
// PDR core intentionally has a very small state surface; offline witness
// classification and margins are evidence, not online scoring terms.

#include "grand_finale/Task22FootprintInsetSweep.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace gf {

struct Task23ContractAudit {
    bool valid=false;
    std::string reason;
    std::vector<std::vector<NodeId>> mobile_components;
};

namespace task23_detail {

inline NodeId findRoot(std::map<NodeId,NodeId>& parent,NodeId node) {
    NodeId root=node;
    while (parent.at(root)!=root) root=parent.at(root);
    while (parent.at(node)!=node) {
        const NodeId next=parent.at(node);
        parent[node]=root;
        node=next;
    }
    return root;
}

inline void unite(std::map<NodeId,NodeId>& parent,NodeId first,NodeId second) {
    NodeId a=findRoot(parent,first);
    NodeId b=findRoot(parent,second);
    if (a==b) return;
    if (a>b) std::swap(a,b);
    parent[b]=a;
}

inline std::set<std::set<NodeId>> asSets(
    const std::vector<std::vector<NodeId>>& groups) {
    std::set<std::set<NodeId>> result;
    for (const auto& group:groups) result.emplace(group.begin(),group.end());
    return result;
}

}  // namespace task23_detail

// A coverage unit is not a planner declaration: it is exactly a connected
// component of the undirected mobile--mobile projection of the reference
// DAG.  This prevents an allocator from commanding independently coupled
// vehicles (the failure exposed by Task 22 P8).
inline Task23ContractAudit task23AuditCoverageContract(
    const Task20DagLatticeContract& contract) {
    Task23ContractAudit result;
    if (!contract.valid) {
        result.reason="invalid_dag_lattice_contract";
        return result;
    }
    std::map<NodeId,NodeId> parent;
    for (NodeId member=1;member<=14;++member) parent[member]=member;
    for (const auto& edge:contract.reference_edges)
        if (edge.reference>=1&&edge.reference<=14&&
            edge.owner>=1&&edge.owner<=14)
            task23_detail::unite(parent,edge.reference,edge.owner);
    std::map<NodeId,std::vector<NodeId>> components;
    for (NodeId member=1;member<=14;++member)
        components[task23_detail::findRoot(parent,member)].push_back(member);
    for (auto& [root,members]:components) {
        (void)root;
        std::sort(members.begin(),members.end());
        result.mobile_components.push_back(std::move(members));
    }
    std::sort(result.mobile_components.begin(),result.mobile_components.end());

    std::vector<std::vector<NodeId>> declared;
    for (const auto& unit:contract.coverage_units) {
        auto members=unit.members;
        std::sort(members.begin(),members.end());
        declared.push_back(std::move(members));
    }
    if (task23_detail::asSets(declared)!=
        task23_detail::asSets(result.mobile_components)) {
        result.reason="declared_units_do_not_match_mobile_connectivity";
        return result;
    }
    result.valid=true;
    result.reason="coverage_units_match_mobile_connectivity";
    return result;
}

// Frozen Task 23 one-unit 5-4-3-2 contract.  Kept behind the generic
// contract interface; PDR has no pinball-specific online branch.
inline Task20DagLatticeContract task23Pinball5432Contract() {
    return task22Pinball5432Contract();
}

enum class Task23WitnessTier {
    NominalCompatible,
    VirtualDependent
};

struct Task23CellWitness {
    std::string cell_id;
    std::string coverage_unit;
    NodeId service_member=0;
    std::size_t pass_index=0;
    double first_service_s=0.0;
    double last_service_s=0.0;
    double canonical_s=0.0;
    Eigen::Vector2d canonical_front=Eigen::Vector2d::Zero();
    Eigen::Vector2d route_tangent=Eigen::Vector2d::Zero();
    double sensing_margin_m=0.0;
    double yaw_margin_rad=0.0;
    double maximum_target_reference_m=0.0;
    double minimum_target_separation_m=0.0;
    double nominal_fim_proxy=0.0;
    Task23WitnessTier tier=Task23WitnessTier::VirtualDependent;
};

struct Task23DeadlinePlan {
    bool valid=false;
    std::string reason;
    Task22SweepPlan route_geometry;
    std::map<std::string,std::vector<Task23CellWitness>> queues;
    std::map<std::string,Task23CellWitness> witnesses;
    std::map<std::string,std::string> canonical_owner;
    std::set<std::string> initially_certified;
    std::vector<std::string> no_witness_cells;
    std::size_t nominal_compatible_count=0;
    std::size_t virtual_dependent_count=0;
    // One plan-level route-validity predicate.  These geometry-derived
    // constants, unlike per-cell witness margins/tier, are the only offline
    // evidence fields consumed by the online PDR core.
    double route_validity_lateral_m=0.0;
    double route_validity_yaw_rad=0.0;
    std::string deadline_rule="earliest_feasible_pass_compatible_interval";
};

struct Task23CoreUnitState {
    std::size_t route_segment=0;
    double cursor_s=0.0;
    std::size_t deadline_queue_index=0;
    Eigen::Vector2d shared_front_target=Eigen::Vector2d::Zero();
    bool has_shared_front_target=false;
    bool active=false;
    FrontierCell last_real_task;
    bool has_last_real_task=false;
};

struct Task23CoreRequest {
    const Task23DeadlinePlan* plan=nullptr;
    std::vector<FrontierCell> certified_uncovered;
    std::map<std::string,Eigen::Vector2d> actual_fronts;
    std::map<std::string,double> actual_front_yaws;
    std::map<std::string,Task23CoreUnitState> states;
    double forward_focus_distance_m=400.0;
    double comparison_epsilon=1.0e-9;
};

struct Task23CoreAssignment {
    std::string coverage_unit;
    FrontierCell task;
    Eigen::Vector2d shared_front_target=Eigen::Vector2d::Zero();
    Eigen::Vector2d route_tangent=Eigen::Vector2d::Zero();
    double actual_front_s=0.0;
    double target_s=0.0;
    double deadline_s=0.0;
    bool route_valid=false;
    bool holding=false;
    bool active=false;
};

struct Task23CoreResult {
    bool valid=false;
    bool complete=false;
    std::string reason;
    std::map<std::string,Task23CoreAssignment> assignments;
    std::map<std::string,Task23CoreUnitState> states;
    std::size_t covered_deadlines_skipped=0;
    double allocation_wall_s=0.0;
};

namespace task23_detail {

inline double wrapAngle(double value) {
    while (value>M_PI) value-=2.0*M_PI;
    while (value<-M_PI) value+=2.0*M_PI;
    return value;
}

struct NominalGeometry {
    double maximum_reference_m=0.0;
    double minimum_separation_m=std::numeric_limits<double>::infinity();
    double fim_proxy=std::numeric_limits<double>::infinity();
    bool compatible=false;
};

inline NominalGeometry nominalGeometry(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const std::map<NodeId,Eigen::Vector2d>& targets) {
    NominalGeometry result;
    const auto position=[&](NodeId node)->const Eigen::Vector2d& {
        const auto mobile=targets.find(node);
        if (mobile!=targets.end()) return mobile->second;
        return fixed_positions.at(node);
    };
    for (const auto& edge:contract.reference_edges)
        result.maximum_reference_m=std::max(result.maximum_reference_m,
            (position(edge.owner)-position(edge.reference)).norm());
    for (NodeId mobile=1;mobile<=14;++mobile) {
        for (NodeId other=mobile+1;other<=14;++other)
            result.minimum_separation_m=std::min(
                result.minimum_separation_m,
                (targets.at(mobile)-targets.at(other)).norm());
        for (const auto& [fixed_id,fixed]:fixed_positions) {
            (void)fixed_id;
            result.minimum_separation_m=std::min(
                result.minimum_separation_m,
                (targets.at(mobile)-fixed).norm());
        }
        Eigen::Matrix2d information=Eigen::Matrix2d::Zero();
        for (const auto& edge:contract.reference_edges) {
            if (edge.owner!=mobile) continue;
            const Eigen::Vector2d delta=position(edge.reference)-
                targets.at(mobile);
            if (delta.norm()<=1.0e-12) continue;
            const Eigen::Vector2d unit=delta.normalized();
            information+=unit*unit.transpose();
        }
        result.fim_proxy=std::min(result.fim_proxy,
            information.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff());
    }
    result.compatible=result.maximum_reference_m<850.0&&
        result.minimum_separation_m>10.0;
    return result;
}

struct SectorMargin {
    bool covered=false;
    double spatial_m=-std::numeric_limits<double>::infinity();
    double yaw_rad=-std::numeric_limits<double>::infinity();
};

inline SectorMargin sectorMargin(const Eigen::Vector2d& pose,double yaw,
    const Eigen::Vector2d& center) {
    constexpr double reserve=0.05+5.0*1.4142135623730950488;
    constexpr double maximum=400.0-reserve;
    const Eigen::Vector2d delta=center-pose;
    const double distance=delta.norm();
    SectorMargin result;
    if (!(distance>reserve&&distance<=maximum)) return result;
    const double error=std::abs(wrapAngle(
        std::atan2(delta.y(),delta.x())-yaw));
    const double angular=M_PI/3.0-error-std::asin(reserve/distance);
    if (angular<0.0) return result;
    result.covered=true;
    result.yaw_rad=angular;
    result.spatial_m=std::min({distance-reserve,maximum-distance,
        distance*angular});
    return result;
}

inline std::size_t sampleAtS(const Task22UnitRoute& route,double s) {
    const auto found=std::lower_bound(route.samples.begin(),route.samples.end(),
        s,[](const Task22RouteSample& sample,double value) {
            return sample.s<value;
        });
    if (found==route.samples.end()) return route.samples.size()-1;
    return static_cast<std::size_t>(found-route.samples.begin());
}

inline bool ownsCross(const Task22SweepPlan& plan,
    const std::string& unit,double cross) {
    std::vector<std::string> units;
    for (const auto& [id,corridor]:plan.corridors) {
        (void)corridor;
        units.push_back(id);
    }
    std::sort(units.begin(),units.end());
    const auto it=std::find(units.begin(),units.end(),unit);
    if (it==units.end()) return false;
    const auto& corridor=plan.corridors.at(unit);
    const bool final=std::next(it)==units.end();
    return cross>=corridor.cross_min-1.0e-9&&
        (final?cross<=corridor.cross_max+1.0e-9:
               cross<corridor.cross_max-1.0e-9);
}

inline void rebuildPdrRouteSamples(Task22UnitRoute& route,
    const Task21CoordinateField& field,double pitch,
    const std::optional<Eigen::Vector2d>& initial_front) {
    route.samples.clear();
    const auto point=[&](double progress,double cross) {
        return field.origin+progress*field.progress_axis+
            cross*field.cross_axis;
    };
    auto push=[&](const Eigen::Vector2d& position,
        const Eigen::Vector2d& tangent,std::size_t pass,bool transition) {
        if (!route.samples.empty()&&
            (route.samples.back().position-position).norm()<1.0e-9) {
            // A pass endpoint is a mandatory straight-pass pose even when
            // the entry/vertical connector ends at the same coordinate.
            if (route.samples.back().on_fillet&&!transition) {
                route.samples.back().tangent=tangent.normalized();
                route.samples.back().pass_index=pass;
                route.samples.back().on_fillet=false;
            }
            return;
        }
        route.samples.push_back({0.0,position,tangent.normalized(),pass,
            transition});
    };
    auto line=[&](const Eigen::Vector2d& from,const Eigen::Vector2d& to,
        const Eigen::Vector2d& tangent,std::size_t pass,bool transition,
        bool skip_first) {
        const double length=(to-from).norm();
        const std::size_t steps=std::max<std::size_t>(1,
            static_cast<std::size_t>(std::ceil(length/pitch)));
        for (std::size_t step=skip_first?1:0;step<=steps;++step)
            push(from+static_cast<double>(step)/static_cast<double>(steps)*
                (to-from),tangent,pass,transition);
    };
    if (route.passes.empty()) return;
    const auto& first=route.passes.front();
    const Eigen::Vector2d first_point=point(first.progress,first.cross_begin);
    if (initial_front&&(*initial_front-first_point).norm()>1.0e-9) {
        const Eigen::Vector2d delta=first_point-*initial_front;
        line(*initial_front,first_point,delta.normalized(),first.pass_index,
            true,false);
    }
    for (std::size_t pass=0;pass<route.passes.size();++pass) {
        const auto& value=route.passes[pass];
        const Eigen::Vector2d begin=point(value.progress,value.cross_begin);
        const Eigen::Vector2d end=point(value.progress,value.cross_end);
        line(begin,end,static_cast<double>(value.direction)*field.cross_axis,
            value.pass_index,false,false);
        if (pass+1==route.passes.size()) continue;
        const auto& next=route.passes[pass+1];
        const Eigen::Vector2d next_begin=point(next.progress,next.cross_begin);
        line(end,next_begin,field.progress_axis,next.pass_index,true,true);
    }
    double s=0.0;
    for (std::size_t index=0;index<route.samples.size();++index) {
        if (index>0) s+=(route.samples[index].position-
            route.samples[index-1].position).norm();
        route.samples[index].s=s;
    }
    route.total_length=s;
}

inline Task22SweepPlan buildPdrRouteGeometry(
    std::vector<FrontierCell> cells,
    std::vector<std::string> coverage_units,
    const Task21CoordinateField& field,double pass_spacing_m,
    std::size_t local_window_cells,
    const std::map<std::string,Eigen::Vector2d>& initial_front_positions) {
    Task22SweepPlan result;
    result.field=field;
    result.pass_spacing_m=pass_spacing_m;
    result.local_window_cells=local_window_cells;
    if (!field.valid||cells.empty()||coverage_units.empty()||
        !(pass_spacing_m>0.0)||local_window_cells==0) {
        result.reason="invalid_task23_route_request";
        return result;
    }
    std::sort(coverage_units.begin(),coverage_units.end());
    if (std::adjacent_find(coverage_units.begin(),coverage_units.end())!=
        coverage_units.end()) {
        result.reason="duplicate_coverage_unit";
        return result;
    }
    std::sort(cells.begin(),cells.end(),[](const auto& first,
        const auto& second) { return first.id()<second.id(); });
    result.cells=std::move(cells);
    for (std::size_t index=0;index<result.cells.size();++index)
        if (!result.cell_lookup.emplace(result.cells[index].id(),index).second) {
            result.reason="duplicate_route_cell";
            return result;
        }
    result.sample_pitch_m=task22_detail::cellPitch(result.cells);
    double minimum_progress=std::numeric_limits<double>::infinity();
    double maximum_progress=-std::numeric_limits<double>::infinity();
    std::vector<double> unique_cross;
    for (const auto& cell:result.cells) {
        const Eigen::Vector2d coordinate=field.coordinates(cell.center);
        minimum_progress=std::min(minimum_progress,coordinate.x());
        maximum_progress=std::max(maximum_progress,coordinate.x());
        unique_cross.push_back(coordinate.y());
    }
    std::sort(unique_cross.begin(),unique_cross.end());
    unique_cross.erase(std::unique(unique_cross.begin(),unique_cross.end(),
        [](double first,double second) {
            return std::abs(first-second)<1.0e-9;
        }),unique_cross.end());
    if (unique_cross.size()<coverage_units.size()) {
        result.reason="insufficient_cross_track_support";
        return result;
    }
    std::vector<double> boundaries{
        unique_cross.front()-0.5*result.sample_pitch_m};
    for (std::size_t unit=1;unit<coverage_units.size();++unit) {
        const std::size_t cut=unit*unique_cross.size()/coverage_units.size();
        boundaries.push_back(0.5*(unique_cross[cut-1]+unique_cross[cut]));
    }
    boundaries.push_back(unique_cross.back()+0.5*result.sample_pitch_m);
    const std::size_t pass_count=std::max<std::size_t>(1,
        static_cast<std::size_t>(std::ceil(
            (maximum_progress-minimum_progress)/pass_spacing_m)));
    for (std::size_t unit_index=0;unit_index<coverage_units.size();++unit_index) {
        const std::string& unit_id=coverage_units[unit_index];
        Task21RibbonCorridor corridor{unit_id,boundaries[unit_index],
            boundaries[unit_index+1],0};
        for (const auto& cell:result.cells) {
            const double cross=field.coordinates(cell.center).y();
            const bool final=unit_index+1==coverage_units.size();
            if (cross>=corridor.cross_min-1.0e-9&&
                (final?cross<=corridor.cross_max+1.0e-9:
                       cross<corridor.cross_max-1.0e-9))
                ++corridor.workload;
        }
        // Discovery-only extent: wide enough to enumerate every certified
        // straight-pass service interval.  It is replaced below by the
        // canonical-witness envelope before the plan becomes an online asset.
        constexpr double reserve=0.05+5.0*1.4142135623730950488;
        constexpr double certified_range=400.0-reserve;
        const double low=corridor.cross_min-certified_range;
        const double high=corridor.cross_max+certified_range;
        int first_direction=1;
        const auto initial=initial_front_positions.find(unit_id);
        if (initial!=initial_front_positions.end()&&
            field.coordinates(initial->second).y()>
                0.5*(corridor.cross_min+corridor.cross_max))
            first_direction=-1;
        Task22UnitRoute route;
        route.coverage_unit=unit_id;
        for (std::size_t pass=0;pass<pass_count;++pass) {
            Task22SweepPass value;
            value.pass_index=pass;
            value.direction=pass%2==0?first_direction:-first_direction;
            value.progress=minimum_progress+
                (static_cast<double>(pass)+0.5)*pass_spacing_m;
            value.cross_begin=value.direction>0?low:high;
            value.cross_end=value.direction>0?high:low;
            value.inset_low=corridor.cross_min-value.cross_begin;
            value.inset_high=value.cross_end-corridor.cross_max;
            route.passes.push_back(value);
        }
        rebuildPdrRouteSamples(route,field,result.sample_pitch_m,
            initial==initial_front_positions.end()
                ?std::optional<Eigen::Vector2d>{}
                :std::optional<Eigen::Vector2d>{initial->second});
        route.first_service_s.assign(result.cells.size(),-1.0);
        route.last_service_s.assign(result.cells.size(),-1.0);
        result.corridors.emplace(unit_id,corridor);
        result.routes.emplace(unit_id,std::move(route));
    }
    result.valid=true;
    result.reason="task23_discovery_route";
    return result;
}

}  // namespace task23_detail

// Offline-only witness construction.  Classification and margins are
// frozen into the asset; allocateTask23PdrCore never scores them.
inline Task23DeadlinePlan task23BuildDeadlinePlan(
    const std::vector<FrontierCell>& cells,
    const Task21CoordinateField& field,double pass_spacing_m,
    std::size_t local_window_cells,
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const std::map<std::string,Eigen::Vector2d>& initial_front_positions,
    const std::set<std::string>& initially_certified={}) {
    Task23DeadlinePlan result;
    result.initially_certified=initially_certified;
    const auto audit=task23AuditCoverageContract(contract);
    if (!audit.valid) {
        result.reason=audit.reason;
        return result;
    }
    std::vector<std::string> units;
    for (const auto& unit:contract.coverage_units) units.push_back(unit.id);
    result.route_geometry=task23_detail::buildPdrRouteGeometry(cells,units,
        field,pass_spacing_m,local_window_cells,initial_front_positions);
    if (!result.route_geometry.valid) {
        result.reason=result.route_geometry.reason;
        return result;
    }
    // Staying within half the gap between adjacent pass centre-lines minus
    // half a cell prevents a folding-route projection from changing passes.
    // The yaw tube is the certified 60-degree sector remaining after the
    // formal position reserve at maximum certified sensing range.
    constexpr double reserve=0.05+5.0*1.4142135623730950488;
    result.route_validity_lateral_m=std::max(0.0,
        0.5*(pass_spacing_m-result.route_geometry.sample_pitch_m));
    result.route_validity_yaw_rad=M_PI/3.0-
        std::asin(reserve/(400.0-reserve));

    struct CachedSample {
        bool valid=false;
        Task20LiftResult lifted;
        task23_detail::NominalGeometry nominal;
    };
    std::map<std::string,std::vector<CachedSample>> cache;
    std::map<std::string,std::pair<Eigen::Vector2d,Eigen::Vector2d>>
        interval_fronts;
    for (const auto& unit:contract.coverage_units) {
        const auto& route=result.route_geometry.routes.at(unit.id);
        auto& values=cache[unit.id];
        values.resize(route.samples.size());
        for (std::size_t index=0;index<route.samples.size();++index) {
            if (route.samples[index].on_fillet) continue;
            std::map<std::string,Eigen::Vector2d> fronts=
                initial_front_positions;
            fronts[unit.id]=route.samples[index].position;
            values[index].lifted=task20LiftTargets(contract,fixed_positions,
                fronts);
            if (!values[index].lifted.valid) continue;
            values[index].nominal=task23_detail::nominalGeometry(contract,
                fixed_positions,values[index].lifted.targets);
            values[index].valid=true;
        }
    }

    for (std::size_t cell_index=0;
         cell_index<result.route_geometry.cells.size();++cell_index) {
        const auto& cell=result.route_geometry.cells[cell_index];
        if (initially_certified.count(cell.id())) continue;
        const double cross=field.coordinates(cell.center).y();
        const Task20CoverageUnit* owner=nullptr;
        for (const auto& unit:contract.coverage_units)
            if (task23_detail::ownsCross(result.route_geometry,unit.id,cross)) {
                owner=&unit;
                break;
            }
        if (owner==nullptr) {
            result.no_witness_cells.push_back(cell.id());
            continue;
        }
        result.canonical_owner[cell.id()]=owner->id;
        const auto& route=result.route_geometry.routes.at(owner->id);
        struct Candidate {
            std::size_t sample=0;
            NodeId member=0;
            double margin=-std::numeric_limits<double>::infinity();
            double yaw_margin=-std::numeric_limits<double>::infinity();
        };
        std::vector<Candidate> candidates;
        for (std::size_t sample=0;sample<route.samples.size();++sample) {
            const auto& geometry=cache.at(owner->id)[sample];
            if (!geometry.valid) continue;
            const auto& route_sample=route.samples[sample];
            const double yaw=std::atan2(route_sample.tangent.y(),
                route_sample.tangent.x());
            Candidate best;
            best.sample=sample;
            for (NodeId member:owner->members) {
                const auto margin=task23_detail::sectorMargin(
                    geometry.lifted.targets.at(member),yaw,cell.center);
                if (!margin.covered) continue;
                if (best.member==0||margin.spatial_m>best.margin+1.0e-9||
                    (std::abs(margin.spatial_m-best.margin)<=1.0e-9&&
                     member<best.member)) {
                    best.member=member;
                    best.margin=margin.spatial_m;
                    best.yaw_margin=margin.yaw_rad;
                }
            }
            if (best.member==0) continue;
            candidates.push_back(best);
        }
        if (candidates.empty()) {
            result.no_witness_cells.push_back(cell.id());
            continue;
        }
        std::size_t selected_pass=std::numeric_limits<std::size_t>::max();
        for (const auto& candidate:candidates)
            selected_pass=std::min(selected_pass,
                route.samples[candidate.sample].pass_index);
        bool has_compatible=false;
        for (const auto& candidate:candidates)
            if (route.samples[candidate.sample].pass_index==selected_pass)
                has_compatible=has_compatible||
                    cache.at(owner->id)[candidate.sample].nominal.compatible;
        std::vector<Candidate> selected;
        for (const auto& candidate:candidates) {
            const auto& geometry=cache.at(owner->id)[candidate.sample];
            if (route.samples[candidate.sample].pass_index==selected_pass&&
                (!has_compatible||geometry.nominal.compatible))
                selected.push_back(candidate);
        }
        if (selected.empty()) {
            result.no_witness_cells.push_back(cell.id());
            continue;
        }
        std::size_t interval_begin=0,chosen_begin=0,chosen_end=0;
        bool in_interval=false;
        for (std::size_t index=0;index<selected.size();++index) {
            const bool contiguous=index==0||selected[index].sample==
                selected[index-1].sample+1;
            if (!in_interval||!contiguous) {
                interval_begin=index;
                in_interval=true;
            }
            if (index+1==selected.size()||
                selected[index+1].sample!=selected[index].sample+1) {
                chosen_begin=interval_begin;
                chosen_end=index;
                in_interval=false;
            }
        }
        const Candidate* canonical=nullptr;
        for (std::size_t index=chosen_begin;index<=chosen_end;++index) {
            const auto& candidate=selected[index];
            if (canonical==nullptr||candidate.margin>canonical->margin+1.0e-9||
                (std::abs(candidate.margin-canonical->margin)<=1.0e-9&&
                 candidate.sample>canonical->sample)) canonical=&candidate;
        }
        if (canonical==nullptr) {
            result.no_witness_cells.push_back(cell.id());
            continue;
        }
        const auto& sample=route.samples[canonical->sample];
        const auto& nominal=cache.at(owner->id)[canonical->sample].nominal;
        Task23CellWitness witness;
        witness.cell_id=cell.id();
        witness.coverage_unit=owner->id;
        witness.service_member=canonical->member;
        witness.pass_index=sample.pass_index;
        witness.first_service_s=route.samples[
            selected[chosen_begin].sample].s;
        witness.last_service_s=route.samples[
            selected[chosen_end].sample].s;
        witness.canonical_s=sample.s;
        witness.canonical_front=sample.position;
        witness.route_tangent=sample.tangent;
        witness.sensing_margin_m=canonical->margin;
        witness.yaw_margin_rad=canonical->yaw_margin;
        witness.maximum_target_reference_m=nominal.maximum_reference_m;
        witness.minimum_target_separation_m=nominal.minimum_separation_m;
        witness.nominal_fim_proxy=nominal.fim_proxy;
        witness.tier=has_compatible?Task23WitnessTier::NominalCompatible:
            Task23WitnessTier::VirtualDependent;
        result.witnesses.emplace(cell.id(),witness);
        interval_fronts[cell.id()]={
            route.samples[selected[chosen_begin].sample].position,
            route.samples[selected[chosen_end].sample].position};
        if (has_compatible) ++result.nominal_compatible_count;
        else ++result.virtual_dependent_count;
    }

    // Freeze each straight pass to the canonical-witness cross-track
    // envelope.  Adjacent passes share the outward endpoint on their turn
    // side, yielding exactly: horizontal sweep -> same-side progress move ->
    // reversed horizontal sweep.  Discovery extents and service margins are
    // discarded here and never reach the online allocator.
    for (const auto& unit:contract.coverage_units) {
        auto& route=result.route_geometry.routes.at(unit.id);
        const auto& corridor=result.route_geometry.corridors.at(unit.id);
        std::vector<double> low(route.passes.size(),
            std::numeric_limits<double>::infinity());
        std::vector<double> high(route.passes.size(),
            -std::numeric_limits<double>::infinity());
        for (const auto& [cell_id,witness]:result.witnesses) {
            if (witness.coverage_unit!=unit.id) continue;
            const double cross=field.coordinates(witness.canonical_front).y();
            low[witness.pass_index]=std::min(low[witness.pass_index],cross);
            high[witness.pass_index]=std::max(high[witness.pass_index],cross);
        }
        for (std::size_t pass=0;pass<route.passes.size();++pass) {
            if (!std::isfinite(low[pass]))
                low[pass]=high[pass]=0.5*(corridor.cross_min+
                    corridor.cross_max);
            route.passes[pass].inset_low=low[pass];
            route.passes[pass].inset_high=high[pass];
        }
        const auto turn_cross=[&](std::size_t pass) {
            const int side=route.passes[pass].direction;
            return side>0?std::max(high[pass],high[pass+1]):
                std::min(low[pass],low[pass+1]);
        };
        for (std::size_t pass=0;pass<route.passes.size();++pass) {
            auto& value=route.passes[pass];
            value.cross_begin=pass==0
                ?(value.direction>0?low[pass]:high[pass])
                :turn_cross(pass-1);
            value.cross_end=pass+1==route.passes.size()
                ?(value.direction>0?high[pass]:low[pass])
                :turn_cross(pass);
        }
        const auto initial=initial_front_positions.find(unit.id);
        task23_detail::rebuildPdrRouteSamples(route,field,
            result.route_geometry.sample_pitch_m,
            initial==initial_front_positions.end()
                ?std::optional<Eigen::Vector2d>{}
                :std::optional<Eigen::Vector2d>{initial->second});
        const auto nearest=[&](std::size_t pass,
            const Eigen::Vector2d& position) {
            std::size_t best=0;
            double distance=std::numeric_limits<double>::infinity();
            for (std::size_t index=0;index<route.samples.size();++index) {
                const auto& sample=route.samples[index];
                if (sample.on_fillet||sample.pass_index!=pass) continue;
                const double candidate=(sample.position-position).norm();
                if (candidate<distance-1.0e-9) {
                    distance=candidate;
                    best=index;
                }
            }
            return best;
        };
        for (auto& [cell_id,witness]:result.witnesses) {
            if (witness.coverage_unit!=unit.id) continue;
            const auto canonical=nearest(witness.pass_index,
                witness.canonical_front);
            const auto first=nearest(witness.pass_index,
                interval_fronts.at(cell_id).first);
            const auto last=nearest(witness.pass_index,
                interval_fronts.at(cell_id).second);
            witness.canonical_s=route.samples[canonical].s;
            witness.canonical_front=route.samples[canonical].position;
            witness.route_tangent=route.samples[canonical].tangent;
            witness.first_service_s=route.samples[first].s;
            witness.last_service_s=route.samples[last].s;
            std::map<std::string,Eigen::Vector2d> fronts=
                initial_front_positions;
            fronts[unit.id]=witness.canonical_front;
            const auto lifted=task20LiftTargets(contract,fixed_positions,
                fronts);
            const double yaw=std::atan2(witness.route_tangent.y(),
                witness.route_tangent.x());
            const auto margin=task23_detail::sectorMargin(
                lifted.targets.at(witness.service_member),yaw,
                result.route_geometry.cells.at(
                    result.route_geometry.cell_lookup.at(cell_id)).center);
            const auto nominal=task23_detail::nominalGeometry(contract,
                fixed_positions,lifted.targets);
            witness.sensing_margin_m=margin.spatial_m;
            witness.yaw_margin_rad=margin.yaw_rad;
            witness.maximum_target_reference_m=nominal.maximum_reference_m;
            witness.minimum_target_separation_m=nominal.minimum_separation_m;
            witness.nominal_fim_proxy=nominal.fim_proxy;
            witness.tier=nominal.compatible
                ?Task23WitnessTier::NominalCompatible
                :Task23WitnessTier::VirtualDependent;
            result.queues[unit.id].push_back(witness);
        }
    }
    result.nominal_compatible_count=0;
    result.virtual_dependent_count=0;
    for (const auto& [cell_id,witness]:result.witnesses) {
        (void)cell_id;
        if (witness.tier==Task23WitnessTier::NominalCompatible)
            ++result.nominal_compatible_count;
        else ++result.virtual_dependent_count;
    }
    for (const auto& unit:contract.coverage_units) {
        auto& queue=result.queues[unit.id];
        std::sort(queue.begin(),queue.end(),[](const auto& first,
            const auto& second) {
            if (std::abs(first.canonical_s-second.canonical_s)>1.0e-9)
                return first.canonical_s<second.canonical_s;
            return first.cell_id<second.cell_id;
        });
    }
    result.valid=result.no_witness_cells.empty()&&
        result.witnesses.size()+initially_certified.size()>=cells.size();
    result.reason=result.valid?"task23_deadline_plan_valid":
        "task23_straight_pass_service_holes";
    if (result.valid) result.route_geometry.reason=
        "task23_canonical_witness_envelope_route";
    return result;
}

inline Task23CoreResult allocateTask23PdrCore(
    const Task23CoreRequest& request) {
    const auto started=std::chrono::steady_clock::now();
    Task23CoreResult result;
    const auto finish=[&](Task23CoreResult value) {
        value.allocation_wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return value;
    };
    if (request.plan==nullptr||!request.plan->valid||
        request.forward_focus_distance_m<0.0) {
        result.reason="invalid_task23_core_request";
        return finish(std::move(result));
    }
    std::map<std::string,FrontierCell> uncovered;
    for (const auto& cell:request.certified_uncovered)
        if (!uncovered.emplace(cell.id(),cell).second) {
            result.reason="duplicate_uncovered_cell";
            return finish(std::move(result));
        }
    result.states=request.states;
    if (uncovered.empty()) {
        result.valid=true;
        result.complete=true;
        result.reason="certified_t100";
        return finish(std::move(result));
    }
    for (const auto& [unit,queue]:request.plan->queues) {
        auto& state=result.states[unit];
        while (state.deadline_queue_index<queue.size()&&
            !uncovered.count(queue[state.deadline_queue_index].cell_id)) {
            ++state.deadline_queue_index;
            ++result.covered_deadlines_skipped;
        }
        Task23CoreAssignment assignment;
        assignment.coverage_unit=unit;
        if (state.deadline_queue_index>=queue.size()) {
            state.active=false;
            if (state.has_last_real_task&&state.has_shared_front_target) {
                assignment.task=state.last_real_task;
                assignment.shared_front_target=state.shared_front_target;
            }
            result.assignments[unit]=assignment;
            continue;
        }
        const auto& deadline=queue[state.deadline_queue_index];
        const auto front=request.actual_fronts.find(unit);
        const auto yaw=request.actual_front_yaws.find(unit);
        if (front==request.actual_fronts.end()||yaw==request.actual_front_yaws.end()||
            !front->second.allFinite()||!std::isfinite(yaw->second)) {
            result.reason="missing_actual_front_state:"+unit;
            return finish(std::move(result));
        }
        const auto& route=request.plan->route_geometry.routes.at(unit);
        const std::size_t previous_target_index=std::min(
            state.route_segment,route.samples.size()-1);
        const double search_begin=std::max(0.0,
            state.cursor_s-request.forward_focus_distance_m);
        const double search_end=std::min(deadline.canonical_s,
            state.cursor_s+request.forward_focus_distance_m);
        std::size_t projection_index=
            task23_detail::sampleAtS(route,search_begin);
        double projection_distance=(front->second-
            route.samples[projection_index].position).norm();
        for (std::size_t index=projection_index+1;
             index<route.samples.size()&&route.samples[index].s<=search_end;
             ++index) {
            const double distance=(front->second-
                route.samples[index].position).norm();
            if (distance<projection_distance-request.comparison_epsilon) {
                projection_distance=distance;
                projection_index=index;
            }
        }
        const auto& projection_sample=route.samples[projection_index];
        const double yaw_error=std::abs(task23_detail::wrapAngle(
            yaw->second-std::atan2(projection_sample.tangent.y(),
                projection_sample.tangent.x())));
        // One predicate only.  It reads plan-level route geometry, never
        // per-cell witness tier, interval or margin.
        assignment.route_valid=
            projection_distance<=request.plan->route_validity_lateral_m+
                1.0e-9&&
            yaw_error<=request.plan->route_validity_yaw_rad+1.0e-9;
        double actual_front_s=state.cursor_s;
        if (assignment.route_valid)
            actual_front_s=std::max(state.cursor_s,
                std::min(deadline.canonical_s,projection_sample.s));
        double target_s=route.samples[previous_target_index].s;
        if (assignment.route_valid)
            target_s=std::min(deadline.canonical_s,
                actual_front_s+request.forward_focus_distance_m);
        target_s=std::max(route.samples[previous_target_index].s,target_s);
        const std::size_t target_index=
            task23_detail::sampleAtS(route,target_s);
        state.cursor_s=actual_front_s;
        state.route_segment=target_index;
        state.shared_front_target=route.samples[target_index].position;
        state.has_shared_front_target=true;
        state.active=true;
        state.last_real_task=uncovered.at(deadline.cell_id);
        state.has_last_real_task=true;

        assignment.task=state.last_real_task;
        assignment.shared_front_target=state.shared_front_target;
        assignment.route_tangent=route.samples[target_index].tangent;
        assignment.actual_front_s=actual_front_s;
        assignment.target_s=target_s;
        assignment.deadline_s=deadline.canonical_s;
        assignment.holding=!assignment.route_valid||
            target_s>=deadline.canonical_s-request.comparison_epsilon;
        assignment.active=true;
        result.assignments[unit]=assignment;
    }
    result.valid=true;
    result.reason="task23_pdr_core_real_deadline_assignments";
    return finish(std::move(result));
}

}  // namespace gf
