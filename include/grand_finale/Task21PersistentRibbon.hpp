#pragma once

#include "grand_finale/Task20CoveragePolicy.hpp"

#include <chrono>
#include <set>

namespace gf {

struct Task21CoordinateField {
    bool valid=false;
    Eigen::Vector2d origin=Eigen::Vector2d::Zero();
    Eigen::Vector2d progress_axis=Eigen::Vector2d::Zero();
    Eigen::Vector2d cross_axis=Eigen::Vector2d::Zero();

    Eigen::Vector2d coordinates(const Eigen::Vector2d& point) const {
        const Eigen::Vector2d delta=point-origin;
        return {progress_axis.dot(delta),cross_axis.dot(delta)};
    }
};

struct Task21RibbonCorridor {
    std::string coverage_unit;
    double cross_min=0.0;
    double cross_max=0.0;
    std::size_t workload=0;
};

struct Task21RibbonSegment {
    std::size_t band_index=0;
    int direction=1;
    double progress_min=0.0;
    double progress_max=0.0;
    double route_progress=0.0;
    Eigen::Vector2d tangent=Eigen::Vector2d::Zero();
    std::size_t route_begin=0;
    std::size_t route_end=0;
};

struct Task21UnitRoute {
    std::string coverage_unit;
    std::vector<std::string> cell_ids;
    std::vector<Task21RibbonSegment> segments;
};

struct Task21RibbonPlan {
    bool valid=false;
    std::string reason;
    Task21CoordinateField field;
    double route_spacing_m=0.0;
    std::size_t local_window_cells=0;
    double endpoint_tolerance_m=0.0;
    std::vector<FrontierCell> cells;
    std::map<std::string,std::size_t> cell_lookup;
    std::vector<Task21RibbonCorridor> corridors;
    std::map<std::string,Task21UnitRoute> routes;
};

struct Task21AllocationRequest {
    Task21RibbonPlan plan;
    std::vector<FrontierCell> uncovered_cells;
    std::map<std::string,Eigen::Vector2d> front_positions;
    std::map<std::string,std::size_t> cursors;
    std::map<std::string,std::size_t> active_segments;
    double forward_focus_distance_m=400.0;
    double comparison_epsilon=1.0e-9;
};

struct Task21RibbonAssignment {
    std::string coverage_unit;
    FrontierCell task;
    std::size_t route_index=0;
    std::size_t band_index=0;
    Eigen::Vector2d route_tangent=Eigen::Vector2d::Zero();
    Eigen::Vector2d front=Eigen::Vector2d::Zero();
    bool transition_hold=false;
};

struct Task21AllocationResult {
    bool valid=false;
    bool complete=false;
    std::string reason;
    std::map<std::string,Task21RibbonAssignment> assignments;
    std::map<std::string,std::size_t> cursors;
    std::map<std::string,std::size_t> active_segments;
    std::size_t scanned_cells=0;
    double allocation_wall_s=0.0;
};

inline Task21CoordinateField task21AffineCoordinateField(
    Eigen::Vector2d origin,Eigen::Vector2d progress_axis,
    Eigen::Vector2d cross_axis) {
    Task21CoordinateField result;
    if (!origin.allFinite()||!progress_axis.allFinite()||
        !cross_axis.allFinite()||progress_axis.norm()<1.0e-12||
        cross_axis.norm()<1.0e-12) return result;
    progress_axis.normalize();
    cross_axis.normalize();
    if (std::abs(progress_axis.dot(cross_axis))>1.0e-9) return result;
    result.valid=true;
    result.origin=origin;
    result.progress_axis=progress_axis;
    result.cross_axis=cross_axis;
    return result;
}

namespace task21_detail {

inline std::size_t bandFor(double progress,double minimum,double spacing) {
    return static_cast<std::size_t>(std::max(0.0,
        std::floor((progress-minimum)/spacing+1.0e-12)));
}

inline Eigen::Vector2d tangentFor(const Task21CoordinateField& field,
    int direction) {
    return static_cast<double>(direction)*field.cross_axis;
}

inline std::size_t bandAt(const Task21UnitRoute& route,std::size_t index) {
    for (const auto& segment:route.segments)
        if (index>=segment.route_begin&&index<segment.route_end)
            return segment.band_index;
    return route.segments.empty()?0:route.segments.back().band_index;
}

inline Eigen::Vector2d tangentAt(const Task21UnitRoute& route,
    std::size_t index) {
    for (const auto& segment:route.segments)
        if (index>=segment.route_begin&&index<segment.route_end)
            return segment.tangent;
    return route.segments.empty()?Eigen::Vector2d::UnitX():
        route.segments.back().tangent;
}

}  // namespace task21_detail

inline Task21RibbonPlan task21BuildRibbonPlan(
    std::vector<FrontierCell> cells,std::vector<std::string> coverage_units,
    const Task21CoordinateField& field,double route_spacing_m,
    std::size_t local_window_cells,
    const std::map<std::string,Eigen::Vector2d>& initial_front_positions={}) {
    Task21RibbonPlan result;
    result.field=field;
    result.route_spacing_m=route_spacing_m;
    result.local_window_cells=local_window_cells;
    if (!field.valid||cells.empty()||coverage_units.empty()||
        !(route_spacing_m>0.0)||local_window_cells==0) {
        result.reason="invalid_ribbon_plan_request";
        return result;
    }
    std::sort(coverage_units.begin(),coverage_units.end());
    if (std::adjacent_find(coverage_units.begin(),coverage_units.end())!=
        coverage_units.end()) {
        result.reason="duplicate_coverage_unit";
        return result;
    }
    std::sort(cells.begin(),cells.end(),[](const auto& first,const auto& second) {
        return first.id()<second.id();
    });
    for (std::size_t index=0;index<cells.size();++index) {
        if (!result.cell_lookup.emplace(cells[index].id(),index).second) {
            result.reason="duplicate_route_cell";
            return result;
        }
    }
    result.cells=std::move(cells);
    std::vector<double> unique_cross;
    for (const auto& cell:result.cells)
        unique_cross.push_back(field.coordinates(cell.center).y());
    std::sort(unique_cross.begin(),unique_cross.end());
    unique_cross.erase(std::unique(unique_cross.begin(),unique_cross.end(),
        [](double first,double second) { return std::abs(first-second)<1.0e-9; }),
        unique_cross.end());
    if (unique_cross.size()<coverage_units.size()) {
        result.reason="insufficient_cross_track_support";
        return result;
    }
    double cell_pitch=std::numeric_limits<double>::infinity();
    for (std::size_t index=1;index<unique_cross.size();++index)
        if (unique_cross[index]-unique_cross[index-1]>1.0e-9)
            cell_pitch=std::min(cell_pitch,
                unique_cross[index]-unique_cross[index-1]);
    if (!std::isfinite(cell_pitch)) cell_pitch=route_spacing_m;
    result.endpoint_tolerance_m=1.5*cell_pitch;
    std::vector<double> boundaries{unique_cross.front()-1.0e-6};
    for (std::size_t unit=1;unit<coverage_units.size();++unit) {
        const std::size_t cut=unit*unique_cross.size()/coverage_units.size();
        boundaries.push_back(0.5*(unique_cross[cut-1]+unique_cross[cut]));
    }
    boundaries.push_back(unique_cross.back()+1.0e-6);
    double minimum_progress=std::numeric_limits<double>::infinity();
    for (const auto& cell:result.cells)
        minimum_progress=std::min(minimum_progress,
            field.coordinates(cell.center).x());
    for (std::size_t unit_index=0;unit_index<coverage_units.size();++unit_index) {
        const std::string& unit=coverage_units[unit_index];
        Task21RibbonCorridor corridor{unit,boundaries[unit_index],
            boundaries[unit_index+1],0};
        int first_direction=1;
        const auto initial_front=initial_front_positions.find(unit);
        if (initial_front!=initial_front_positions.end()) {
            if (!initial_front->second.allFinite()) {
                result.reason="invalid_initial_unit_front";
                return result;
            }
            const double initial_cross=
                field.coordinates(initial_front->second).y();
            const double midpoint=0.5*(corridor.cross_min+corridor.cross_max);
            first_direction=initial_cross<=midpoint?1:-1;
        }
        struct Ordered { std::string id; std::size_t band; double cross; double progress; };
        std::vector<Ordered> ordered;
        for (const auto& cell:result.cells) {
            const Eigen::Vector2d coordinate=field.coordinates(cell.center);
            const bool final=unit_index+1==coverage_units.size();
            if (coordinate.y()+1.0e-9<corridor.cross_min||
                (final?coordinate.y()>corridor.cross_max+1.0e-9:
                       coordinate.y()>=corridor.cross_max-1.0e-9)) continue;
            ordered.push_back({cell.id(),task21_detail::bandFor(
                coordinate.x(),minimum_progress,route_spacing_m),
                coordinate.y(),coordinate.x()});
        }
        std::sort(ordered.begin(),ordered.end(),[first_direction](
            const Ordered& first,const Ordered& second) {
            if (first.band!=second.band) return first.band<second.band;
            if (std::abs(first.cross-second.cross)>1.0e-9)
                return (first.band%2==0?first_direction:-first_direction)>0
                    ?first.cross<second.cross:first.cross>second.cross;
            if (std::abs(first.progress-second.progress)>1.0e-9)
                return first.progress<second.progress;
            return first.id<second.id;
        });
        corridor.workload=ordered.size();
        result.corridors.push_back(corridor);
        Task21UnitRoute route;
        route.coverage_unit=unit;
        for (const auto& value:ordered) route.cell_ids.push_back(value.id);
        for (std::size_t begin=0;begin<ordered.size();) {
            std::size_t end=begin+1;
            while (end<ordered.size()&&ordered[end].band==ordered[begin].band) ++end;
            double pmin=ordered[begin].progress,pmax=ordered[begin].progress;
            for (std::size_t index=begin+1;index<end;++index) {
                pmin=std::min(pmin,ordered[index].progress);
                pmax=std::max(pmax,ordered[index].progress);
            }
            const int direction=ordered[begin].band%2==0
                ?first_direction:-first_direction;
            const double route_progress=minimum_progress+
                (static_cast<double>(ordered[begin].band)+0.5)*route_spacing_m;
            route.segments.push_back({ordered[begin].band,direction,pmin,pmax,
                route_progress,
                task21_detail::tangentFor(field,direction),begin,end});
            begin=end;
        }
        if (route.cell_ids.empty()) {
            result.reason="empty_unit_corridor";
            return result;
        }
        result.routes.emplace(unit,std::move(route));
    }
    result.valid=true;
    return result;
}

inline Task21AllocationResult allocateTask21Ribbon(
    const Task21AllocationRequest& request) {
    const auto started=std::chrono::steady_clock::now();
    Task21AllocationResult result;
    const auto finish=[&](Task21AllocationResult value) {
        value.allocation_wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return value;
    };
    if (!request.plan.valid||request.forward_focus_distance_m<0.0) {
        result.reason="invalid_ribbon_allocation_request";
        return finish(std::move(result));
    }
    std::map<std::string,FrontierCell> uncovered;
    for (const auto& cell:request.uncovered_cells) {
        if (!request.plan.cell_lookup.count(cell.id())||
            !uncovered.emplace(cell.id(),cell).second) {
            result.reason="invalid_or_duplicate_uncovered_cell";
            return finish(std::move(result));
        }
    }
    result.cursors=request.cursors;
    result.active_segments=request.active_segments;
    for (const auto& [unit,route]:request.plan.routes)
        if (!result.cursors.count(unit)) result.cursors[unit]=0;
    for (const auto& [unit,route]:request.plan.routes)
        if (!result.active_segments.count(unit)) result.active_segments[unit]=0;
    if (uncovered.empty()) {
        result.valid=true;
        result.complete=true;
        result.reason="certified_t100";
        return finish(std::move(result));
    }
    std::set<std::string> used;
    for (const auto& [unit,route]:request.plan.routes) {
        const auto front=request.front_positions.find(unit);
        if (front==request.front_positions.end()||!front->second.allFinite()) {
            result.reason="missing_unit_front:"+unit;
            return finish(std::move(result));
        }
        std::size_t cursor=std::min(result.cursors.at(unit),route.cell_ids.size());
        while (cursor<route.cell_ids.size()&&!uncovered.count(route.cell_ids[cursor]))
            ++cursor;
        result.cursors[unit]=cursor;
        if (cursor==route.cell_ids.size()) continue;
        std::size_t active=std::min(result.active_segments.at(unit),
            route.segments.size()-1);
        while (active<route.segments.size()&&
               cursor>=route.segments[active].route_end) {
            const auto& segment=route.segments[active];
            const auto& endpoint_cell=request.plan.cells.at(request.plan.cell_lookup.at(
                route.cell_ids.at(segment.route_end-1)));
            const double endpoint_cross=
                request.plan.field.coordinates(endpoint_cell.center).y();
            const Eigen::Vector2d endpoint=request.plan.field.origin+
                segment.route_progress*request.plan.field.progress_axis+
                endpoint_cross*request.plan.field.cross_axis;
            if ((front->second-endpoint).norm()>
                request.plan.endpoint_tolerance_m) {
                const auto& task=request.plan.cells.at(
                    request.plan.cell_lookup.at(route.cell_ids[cursor]));
                used.insert(task.id());
                result.assignments[unit]={unit,task,segment.route_end-1,
                    segment.band_index,segment.tangent,endpoint,true};
                break;
            }
            ++active;
        }
        result.active_segments[unit]=active;
        if (result.assignments.count(unit)) continue;
        if (active>=route.segments.size()) continue;
        const auto& active_segment=route.segments[active];
        const Eigen::Vector2d tangent=active_segment.tangent;
        const Eigen::Vector2d focus=front->second+
            request.forward_focus_distance_m*tangent;
        const std::size_t begin=std::max(cursor,active_segment.route_begin);
        const std::size_t end=std::min(active_segment.route_end,
            begin+request.plan.local_window_cells);
        const FrontierCell* best=nullptr;
        std::size_t best_index=cursor;
        double best_distance=std::numeric_limits<double>::infinity();
        for (std::size_t index=begin;index<end;++index) {
            const auto found=uncovered.find(route.cell_ids[index]);
            if (found==uncovered.end()||used.count(found->first)) continue;
            ++result.scanned_cells;
            const double distance=(found->second.center-focus).squaredNorm();
            if (best==nullptr||distance<best_distance-request.comparison_epsilon||
                (std::abs(distance-best_distance)<=request.comparison_epsilon&&
                 found->first<best->id())) {
                best=&found->second;
                best_index=index;
                best_distance=distance;
            }
        }
        if (best==nullptr) {
            // The cursor points at a real uncovered cell, so this is reachable
            // only if a prior unit consumed it; corridors are disjoint.
            result.reason="empty_active_segment_window:"+unit;
            return finish(std::move(result));
        }
        used.insert(best->id());
        const std::size_t band=task21_detail::bandAt(route,best_index);
        const auto segment=std::find_if(route.segments.begin(),
            route.segments.end(),[&](const auto& value) {
                return value.band_index==band;
            });
        const double cross=request.plan.field.coordinates(best->center).y();
        const Eigen::Vector2d route_pose=request.plan.field.origin+
            segment->route_progress*request.plan.field.progress_axis+
            cross*request.plan.field.cross_axis;
        result.assignments[unit]={unit,*best,best_index,band,tangent,route_pose,false};
    }
    if (result.assignments.empty()) {
        result.reason="uncovered_outside_active_corridors";
        return finish(std::move(result));
    }
    result.valid=true;
    result.reason="task21_real_cell_assignments";
    return finish(std::move(result));
}

inline Task20DagLatticeContract task21PinballContract() {
    Task20DagLatticeContract result;
    result.id="pinball-four-layer";
    result.structural_signature="units=14;rows=4-4-4-2;pinball-cross-braced";
    result.reference_edges={
        {100,1},{101,1},{100,2},{101,2},{101,3},{102,3},{101,4},{102,4},
        {1,5},{2,5},{2,6},{3,6},{2,7},{3,7},{3,8},{4,8},
        {5,9},{6,9},{6,10},{7,10},{6,11},{7,11},{7,12},{8,12},
        {9,13},{10,13},{11,14},{12,14}};
    result.coverage_units={{"P",{1,2,3,4,5,6,7,8,9,10,11,12,13,14},
        {100,101,102},14,{13,14}}};
    const std::vector<std::pair<double,double>> roles{
        {0.25,-0.24},{0.25,-0.08},{0.25,0.08},{0.25,0.24},
        {0.50,-0.18},{0.50,-0.06},{0.50,0.06},{0.50,0.18},
        {0.75,-0.24},{0.75,-0.08},{0.75,0.08},{0.75,0.24},
        {1.00,-0.08},{1.00,0.08}};
    task20_lattice_detail::addRoles(result,result.coverage_units.front(),roles);
    task20_lattice_detail::finish(result);
    return result;
}

}  // namespace gf
