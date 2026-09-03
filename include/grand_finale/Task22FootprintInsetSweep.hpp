#pragma once

// Task 22 (P5): continuous footprint-inset sweep on the DAG-conditioned
// front manifold.  Replaces the Task 21 persistent-ribbon rectangular
// corridor endpoints and discrete band switching with:
//   * sweep passes whose lateral endpoints are inset to the extreme poses
//     that the unit's certified footprint needs in order to service every
//     cell the pass uniquely serves (footprint-derived, no standoff
//     constant);
//   * exact semicircle U-turns of radius half the pass spacing between
//     adjacent passes (position and tangent continuous, no discrete jump);
//   * one monotone arclength cursor per coverage unit;
//   * real certified-uncovered cell IDs for tasks, typed separately from
//     the navigation/front pose used for target lifting.
// All geometry derives from the certified sensing sector semantics and the
// frozen Task 20 DAG contract; there are no tunable constants.

#include "grand_finale/Task20DagLatticeContract.hpp"
#include "grand_finale/Task21PersistentRibbon.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>

namespace gf {

// Certified sensing sector predicate, identical to the Task 21 route
// oracle: reserve tube 0.05 + half cell diagonal, range (reserve, 400 m],
// bearing cone of 60 degrees including the asin(reserve/d) widening.
// Algebraically identical to the Task 21 oracle's atan2/asin form: with
// r=reserve, d=distance, lateral=|delta x tangent|, the cone condition
// error+asin(r/d)<=pi/3 equals forward>0 AND
// lateral<=sin(pi/3)*sqrt(d^2-r^2)-r/2 (the forward gate keeps the sine
// map monotone).  The Python independent verifier keeps the original
// trigonometric form as a cross-check; it caught a missing forward-branch
// in the first version of this rewrite.
inline bool task22CellInCertifiedSector(const Eigen::Vector2d& pose,double yaw,
    const Eigen::Vector2d& center) {
    constexpr double reserve=0.05+5.0*1.4142135623730950488;
    const double cosine=std::cos(yaw);
    const double sine=std::sin(yaw);
    const double dx=center.x()-pose.x();
    const double dy=center.y()-pose.y();
    const double distance_squared=dx*dx+dy*dy;
    if (distance_squared<=reserve*reserve||
        distance_squared>(400.0-reserve)*(400.0-reserve)) return false;
    const double forward=cosine*dx+sine*dy;
    if (forward<=0.0) return false;  // the cone is strictly forward; this
    // restores the branch that the monotone-sine step needs for exact
    // equivalence with error+asin(r/d)<=pi/3.
    const double lateral=std::abs(-sine*dx+cosine*dy);
    constexpr double sin60=0.86602540378443864676;
    const double half_r=0.5*reserve;
    return lateral<=sin60*std::sqrt(distance_squared-reserve*reserve)-
        half_r+1.0e-9;
}

struct Task22SweepPass {
    std::size_t pass_index=0;
    int direction=1;              // cross-axis travel sense
    double progress=0.0;          // progress coordinate of the pass line
    double cross_begin=0.0;       // travel from cross_begin to cross_end
    double cross_end=0.0;
    double inset_low=0.0;         // footprint-derived coverage insets
    double inset_high=0.0;
};

struct Task22RouteSample {
    double s=0.0;                 // monotone arclength
    Eigen::Vector2d position=Eigen::Vector2d::Zero();
    Eigen::Vector2d tangent=Eigen::Vector2d::Zero();
    std::size_t pass_index=0;
    bool on_fillet=false;
};

struct Task22CellService {
    std::string cell_id;
    double first_service_s=0.0;
};

struct Task22UnitRoute {
    std::string coverage_unit;
    std::vector<Task22SweepPass> passes;
    std::vector<Task22RouteSample> samples;
    double total_length=0.0;
    std::vector<Task22CellService> cell_order;
    std::size_t full_extent_service_count=0;
};

struct Task22SweepPlan {
    bool valid=false;
    std::string reason;
    Task21CoordinateField field;
    double pass_spacing_m=0.0;
    double sample_pitch_m=10.0;
    std::size_t local_window_cells=64;
    std::vector<FrontierCell> cells;
    std::map<std::string,std::size_t> cell_lookup;
    std::map<std::string,Task21RibbonCorridor> corridors;
    std::map<std::string,Task22UnitRoute> routes;
};

struct Task22AllocationRequest {
    Task22SweepPlan plan;
    std::vector<FrontierCell> uncovered_cells;
    std::map<std::string,Eigen::Vector2d> front_positions;
    std::map<std::string,double> cursors;
    double forward_focus_distance_m=400.0;
    double comparison_epsilon=1.0e-9;
};

struct Task22SweepAssignment {
    std::string coverage_unit;
    FrontierCell task;
    double cursor_s=0.0;
    Eigen::Vector2d route_pose=Eigen::Vector2d::Zero();
    Eigen::Vector2d route_tangent=Eigen::Vector2d::Zero();
    std::size_t route_sample_index=0;
    bool on_fillet=false;
    bool window_empty=false;       // task came from the route-order fallback
};

struct Task22AllocationResult {
    bool valid=false;
    bool complete=false;
    std::string reason;
    std::map<std::string,Task22SweepAssignment> assignments;
    std::map<std::string,double> cursors;
    std::size_t scanned_cells=0;
    double allocation_wall_s=0.0;
};

namespace task22_detail {

using CellBits=std::vector<std::uint64_t>;

inline void setBit(CellBits& bits,std::size_t index) {
    bits[index>>6]|=std::uint64_t{1}<<(index&63);
}
inline bool getBit(const CellBits& bits,std::size_t index) {
    return (bits[index>>6]>>(index&63))&1;
}
inline void orInto(CellBits& target,const CellBits& source) {
    for (std::size_t word=0;word<target.size();++word)
        target[word]|=source[word];
}
inline std::size_t countBits(const CellBits& bits) {
    std::size_t count=0;
    for (std::uint64_t word:bits) count+=__builtin_popcountll(word);
    return count;
}

inline double cellPitch(const std::vector<FrontierCell>& cells) {
    std::vector<double> xs;
    xs.reserve(cells.size());
    for (const auto& cell:cells) xs.push_back(cell.center.x());
    std::sort(xs.begin(),xs.end());
    for (std::size_t index=1;index<xs.size();++index)
        if (xs[index]-xs[index-1]>1.0e-9) return xs[index]-xs[index-1];
    return 10.0;
}

struct UnitGeometry {
    const Task20DagLatticeContract* contract=nullptr;
    const std::map<NodeId,Eigen::Vector2d>* fixed_positions=nullptr;
    std::map<std::string,Eigen::Vector2d> other_fronts;
    std::string unit_id;
    Task21CoordinateField field;
};

// Lifts all 14 member targets with the unit's front at (progress, cross).
inline bool liftAt(const UnitGeometry& geometry,double progress,double cross,
    std::map<NodeId,Eigen::Vector2d>& targets) {
    std::map<std::string,Eigen::Vector2d> fronts=geometry.other_fronts;
    fronts[geometry.unit_id]=geometry.field.origin+
        progress*geometry.field.progress_axis+
        cross*geometry.field.cross_axis;
    const auto lifted=task20LiftTargets(*geometry.contract,
        *geometry.fixed_positions,fronts);
    if (!lifted.valid) return false;
    targets=std::move(lifted.targets);
    return true;
}

// Credits the unit's members' certified sector coverage at the front pose
// (progress, cross) travelling along `direction` into `bits`.
inline bool serviceAt(const UnitGeometry& geometry,double progress,
    double cross,int direction,const Task22SweepPlan& plan,
    const Task20CoverageUnit& unit,CellBits& bits) {
    std::map<NodeId,Eigen::Vector2d> targets;
    if (!liftAt(geometry,progress,cross,targets)) return false;
    const Eigen::Vector2d tangent=static_cast<double>(direction)*
        geometry.field.cross_axis;
    const double yaw=std::atan2(tangent.y(),tangent.x());
    double min_x=1.0e18,max_x=-1.0e18,min_y=1.0e18,max_y=-1.0e18;
    for (NodeId member:unit.members) {
        const Eigen::Vector2d pose=targets.at(member);
        min_x=std::min(min_x,pose.x()); max_x=std::max(max_x,pose.x());
        min_y=std::min(min_y,pose.y()); max_y=std::max(max_y,pose.y());
    }
    for (std::size_t index=0;index<plan.cells.size();++index) {
        if (getBit(bits,index)) continue;
        const FrontierCell& cell=plan.cells[index];
        if (cell.center.x()<min_x-400.0||cell.center.x()>max_x+400.0||
            cell.center.y()<min_y-400.0||cell.center.y()>max_y+400.0)
            continue;
        for (NodeId member:unit.members)
            if (task22CellInCertifiedSector(targets.at(member),yaw,
                cell.center)) {
                setBit(bits,index);
                break;
            }
    }
    return true;
}

}  // namespace task22_detail

inline Task22SweepPlan task22BuildSweepPlan(
    const std::vector<FrontierCell>& cells,
    const std::vector<std::string>& coverage_units,
    const Task21CoordinateField& field,double pass_spacing_m,
    std::size_t local_window_cells,
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const std::map<std::string,Eigen::Vector2d>& initial_front_positions) {
    Task22SweepPlan result;
    result.field=field;
    result.pass_spacing_m=pass_spacing_m;
    result.local_window_cells=local_window_cells;
    if (!field.valid||cells.empty()||coverage_units.empty()||
        !(pass_spacing_m>0.0)||local_window_cells==0||!contract.valid) {
        result.reason="invalid_sweep_plan_request";
        return result;
    }
    result.sample_pitch_m=task22_detail::cellPitch(cells);
    std::vector<std::string> units=coverage_units;
    std::sort(units.begin(),units.end());
    if (std::adjacent_find(units.begin(),units.end())!=units.end()) {
        result.reason="duplicate_coverage_unit";
        return result;
    }
    result.cells=cells;
    std::sort(result.cells.begin(),result.cells.end(),
        [](const auto& first,const auto& second) {
            return first.id()<second.id(); });
    for (std::size_t index=0;index<result.cells.size();++index)
        if (!result.cell_lookup.emplace(result.cells[index].id(),index)
                .second) {
            result.reason="duplicate_route_cell";
            return result;
        }
    const std::size_t cell_count=result.cells.size();
    const double pitch=result.sample_pitch_m;
    const std::size_t words=(cell_count+63)/64;

    std::vector<double> unique_cross;
    for (const auto& cell:result.cells)
        unique_cross.push_back(field.coordinates(cell.center).y());
    std::sort(unique_cross.begin(),unique_cross.end());
    unique_cross.erase(std::unique(unique_cross.begin(),unique_cross.end(),
        [](double first,double second) {
            return std::abs(first-second)<1.0e-9; }),
        unique_cross.end());
    if (unique_cross.size()<units.size()) {
        result.reason="insufficient_cross_track_support";
        return result;
    }
    std::vector<double> boundaries{unique_cross.front()-1.0e-6};
    for (std::size_t unit=1;unit<units.size();++unit) {
        const std::size_t cut=unit*unique_cross.size()/units.size();
        boundaries.push_back(0.5*(unique_cross[cut-1]+unique_cross[cut]));
    }
    boundaries.push_back(unique_cross.back()+1.0e-6);
    double minimum_progress=1.0e18,maximum_progress=-1.0e18;
    for (const auto& cell:result.cells) {
        const double progress=field.coordinates(cell.center).x();
        minimum_progress=std::min(minimum_progress,progress);
        maximum_progress=std::max(maximum_progress,progress);
    }
    if (!(maximum_progress>minimum_progress)) {
        result.reason="degenerate_progress_extent";
        return result;
    }
    std::map<std::string,const Task20CoverageUnit*> unit_by_id;
    for (const auto& unit:contract.coverage_units)
        unit_by_id[unit.id]=&unit;
    for (const auto& unit:units)
        if (!unit_by_id.count(unit)) {
            result.reason="unknown_coverage_unit:"+unit;
            return result;
        }
    std::vector<double> pose_cross;
    for (double cross=unique_cross.front();
         cross<=unique_cross.back()+1.0e-9;cross+=pitch)
        pose_cross.push_back(cross);
    auto corridor_poses=[&pose_cross](double cross_min,double cross_max) {
        std::vector<double> result;
        for (double cross:pose_cross)
            if (cross>=cross_min-1.0e-9&&cross<=cross_max+1.0e-9)
                result.push_back(cross);
        return result;
    };

    for (std::size_t unit_index=0;unit_index<units.size();++unit_index) {
        const std::string& unit_id=units[unit_index];
        const Task20CoverageUnit& unit=*unit_by_id.at(unit_id);
        Task21RibbonCorridor corridor{unit_id,boundaries[unit_index],
            boundaries[unit_index+1],0};
        const bool final_unit=unit_index+1==units.size();
        for (const auto& cell:result.cells) {
            const double cross=field.coordinates(cell.center).y();
            if (cross+1.0e-9<corridor.cross_min||
                (final_unit?cross>corridor.cross_max+1.0e-9:
                    cross>=corridor.cross_max-1.0e-9)) continue;
            ++corridor.workload;
        }
        if (corridor.workload==0) {
            result.reason="empty_unit_corridor:"+unit_id;
            return result;
        }
        const std::vector<double> unit_pose_cross=corridor_poses(
            corridor.cross_min,corridor.cross_max);
        std::vector<double> pass_progress;
        for (double progress=minimum_progress+0.5*pass_spacing_m;
             progress<maximum_progress+0.5*pass_spacing_m;
             progress+=pass_spacing_m)
            pass_progress.push_back(progress);
        if (pass_progress.empty()) {
            result.reason="empty_pass_family:"+unit_id;
            return result;
        }

        task22_detail::UnitGeometry geometry;
        geometry.contract=&contract;
        geometry.fixed_positions=&fixed_positions;
        geometry.unit_id=unit_id;
        geometry.field=field;
        const Eigen::Vector2d other_front=field.origin+
            0.5*(minimum_progress+maximum_progress)*field.progress_axis+
            0.5*(boundaries[0]+boundaries.back())*field.cross_axis;
        for (const auto& other:contract.coverage_units)
            if (other.id!=unit_id) geometry.other_fronts[other.id]=other_front;

        // Forward sweep per pass: cumulative union records each cell's
        // lowest covering pose cross (first_cover); the final state is the
        // pass's full-extent union.
        std::vector<task22_detail::CellBits> pass_union(
            pass_progress.size(),task22_detail::CellBits(words,0));
        std::vector<std::map<std::size_t,double>> first_cover(
            pass_progress.size()),last_cover(pass_progress.size());
        for (std::size_t pass=0;pass<pass_progress.size();++pass) {
            task22_detail::CellBits cumulative(words,0);
            for (double cross:unit_pose_cross) {
                task22_detail::CellBits bits(words,0);
                if (!task22_detail::serviceAt(geometry,
                    pass_progress[pass],cross,1,result,unit,bits)) {
                    result.reason="lifting_failed:"+unit_id;
                    return result;
                }
                for (std::size_t word=0;word<words;++word) {
                    std::uint64_t fresh=bits[word]&~cumulative[word];
                    cumulative[word]|=bits[word];
                    while (fresh) {
                        const int bit=__builtin_ctzll(fresh);
                        fresh&=fresh-1;
                        first_cover[pass][word*64+bit]=cross;
                    }
                }
            }
            pass_union[pass]=std::move(cumulative);
            task22_detail::CellBits backward(words,0);
            for (std::size_t index=unit_pose_cross.size();index-->0;) {
                task22_detail::CellBits bits(words,0);
                if (!task22_detail::serviceAt(geometry,
                    pass_progress[pass],unit_pose_cross[index],1,result,
                    unit,bits)) {
                    result.reason="lifting_failed:"+unit_id;
                    return result;
                }
                for (std::size_t word=0;word<words;++word) {
                    std::uint64_t fresh=bits[word]&~backward[word];
                    backward[word]|=bits[word];
                    while (fresh) {
                        const int bit=__builtin_ctzll(fresh);
                        fresh&=fresh-1;
                        last_cover[pass][word*64+bit]=unit_pose_cross[index];
                    }
                }
            }
        }

        // Responsibility slabs: every corridor cell belongs to the pass
        // whose progress line is nearest (ties to the lower index), so the
        // unit's passes tile the corridor.  The insets are then the tightest
        // cross range that still services every slab cell: inset_high = max
        // over slab cells of their lowest covering pose cross, inset_low =
        // min over slab cells of their highest covering pose cross.  This
        // makes the trimmed route's service a superset of the full-extent
        // family's by construction.
        Task22UnitRoute route;
        route.coverage_unit=unit_id;
        std::size_t full_extent_count=0;
        std::vector<std::pair<double,double>> insets(pass_progress.size(),
            {corridor.cross_min,corridor.cross_max});
        std::vector<std::size_t> slab(cell_count,
            std::numeric_limits<std::size_t>::max());
        for (std::size_t index=0;index<cell_count;++index) {
            const double cross=field.coordinates(
                result.cells[index].center).y();
            if (cross+1.0e-9<corridor.cross_min||
                cross>corridor.cross_max+1.0e-9) continue;
            const double progress=field.coordinates(
                result.cells[index].center).x();
            std::size_t best=0;
            double best_distance=std::numeric_limits<double>::infinity();
            for (std::size_t pass=0;pass<pass_progress.size();++pass) {
                const double distance=std::abs(progress-pass_progress[pass]);
                if (distance<best_distance-1.0e-9) {
                    best_distance=distance;
                    best=pass;
                }
            }
            slab[index]=best;
        }
        for (std::size_t pass=0;pass<pass_progress.size();++pass) {
            full_extent_count+=task22_detail::countBits(pass_union[pass]);
            double low_bound=1.0e18,high_bound=-1.0e18;
            bool any_responsibility=false;
            for (std::size_t index=0;index<cell_count;++index) {
                if (slab[index]!=pass) continue;
                any_responsibility=true;
                const auto first=first_cover[pass].find(index);
                const auto last=last_cover[pass].find(index);
                if (first==first_cover[pass].end()||
                    last==last_cover[pass].end()) {
                    // The pass never services a cell it is responsible for;
                    // keep the corridor extreme and let the oracle gate fail.
                    low_bound=corridor.cross_min;
                    high_bound=corridor.cross_max;
                    break;
                }
                low_bound=std::min(low_bound,last->second);
                high_bound=std::max(high_bound,first->second);
            }
            if (any_responsibility&&low_bound<=high_bound)
                insets[pass]={low_bound,high_bound};
        }
        route.full_extent_service_count=full_extent_count;

        const double initial_cross_value=0.5*
            (corridor.cross_min+corridor.cross_max);
        const auto initial_front=initial_front_positions.find(unit_id);
        const double observed_cross=
            initial_front==initial_front_positions.end()||
            !initial_front->second.allFinite()
            ?initial_cross_value
            :field.coordinates(initial_front->second).y();
        const int first_direction=
            observed_cross<=initial_cross_value?1:-1;

        // Shared U-turn cross for the pair (pass, pass+1) at the side where
        // `pass` ends: outward extreme of the two passes' own insets so
        // neither pass loses unique coverage.
        const auto pair_turn_cross=[&](std::size_t pass) {
            const int side=pass%2==0?first_direction:-first_direction;
            if (side>0)
                return std::max(insets[pass].second,insets[pass+1].second);
            return std::min(insets[pass].first,insets[pass+1].first);
        };

        std::vector<Task22SweepPass> passes;
        for (std::size_t pass=0;pass<pass_progress.size();++pass) {
            const int direction=pass%2==0?first_direction:-first_direction;
            Task22SweepPass value;
            value.pass_index=pass;
            value.direction=direction;
            value.progress=pass_progress[pass];
            value.inset_low=insets[pass].first;
            value.inset_high=insets[pass].second;
            const double begin=pass==0
                ?(direction>0?insets[pass].first:insets[pass].second)
                :pair_turn_cross(pass-1);
            const double end=pass+1==pass_progress.size()
                ?(direction>0?insets[pass].second:insets[pass].first)
                :pair_turn_cross(pass);
            value.cross_begin=begin;
            value.cross_end=end;
            passes.push_back(value);
        }
        route.passes=passes;

        // Dense monotone sampling: pass poses at the cell pitch and exact
        // semicircle fillets; arclength is recomputed from geometry so the
        // samples stay consistent.
        auto push=[&](const Eigen::Vector2d& position,
            const Eigen::Vector2d& tangent,std::size_t pass,bool on_fillet) {
            Task22RouteSample sample;
            sample.position=position;
            sample.tangent=tangent;
            sample.pass_index=pass;
            sample.on_fillet=on_fillet;
            route.samples.push_back(sample);
        };
        auto push_pass=[&](const Task22SweepPass& pass,bool skip_first) {
            const double length=std::abs(pass.cross_end-pass.cross_begin);
            const Eigen::Vector2d tangent=static_cast<double>(
                pass.direction)*field.cross_axis;
            if (!(length>1.0e-9)) {
                push(field.origin+pass.progress*field.progress_axis+
                    pass.cross_begin*field.cross_axis,tangent,
                    pass.pass_index,false);
                return;
            }
            const std::size_t count=std::max<std::size_t>(2,
                static_cast<std::size_t>(std::ceil(length/pitch)));
            for (std::size_t index=skip_first?1:0;index<=count;++index) {
                const double fraction=static_cast<double>(index)/
                    static_cast<double>(count);
                const double cross=pass.cross_begin+
                    fraction*(pass.cross_end-pass.cross_begin);
                push(field.origin+pass.progress*field.progress_axis+
                    cross*field.cross_axis,tangent,pass.pass_index,false);
            }
        };
        // Route-start convention: an approach arc from the unit's initial
        // front to the first pass begin, tangent-continuous with the pass
        // direction.  Radius comes from the closed-form circle through the
        // initial front tangent to the pass line (no parameter); the
        // approach poses service cells like any other route pose.
        {
            const Task22SweepPass& first=passes.front();
            const Eigen::Vector2d begin=field.origin+
                first.progress*field.progress_axis+
                first.cross_begin*field.cross_axis;
            const Eigen::Vector2d travel=static_cast<double>(
                first.direction)*field.cross_axis;
            const auto initial_front_entry=initial_front_positions.find(
                unit_id);
            if (initial_front_entry!=initial_front_positions.end()&&
                initial_front_entry->second.allFinite()&&
                (initial_front_entry->second-begin).norm()>pitch) {
                const Eigen::Vector2d from=initial_front_entry->second;
                const Eigen::Vector2d offset=from-begin;
                const double along=offset.dot(travel);
                const double across=offset.dot(field.progress_axis);
                if (std::abs(across)<1.0e-9) {
                    // On the pass line: straight approach along the pass.
                    const std::size_t count=std::max<std::size_t>(2,
                        static_cast<std::size_t>(
                            std::ceil(std::abs(along)/pitch)));
                    for (std::size_t index=0;index<count;++index) {
                        const double fraction=static_cast<double>(index)/
                            static_cast<double>(count);
                        push(begin+fraction*offset,travel,
                            first.pass_index,true);
                    }
                } else {
                    const double radius=(along*along+across*across)/
                        (2.0*across);
                    const Eigen::Vector2d center=begin+
                        radius*field.progress_axis;
                    const double theta_begin=std::atan2(
                        field.cross_axis.dot(from-center),
                        field.progress_axis.dot(from-center));
                    const double theta_end=std::atan2(
                        field.cross_axis.dot(begin-center),
                        field.progress_axis.dot(begin-center));
                    // Sweep from theta_begin to theta_end choosing the
                    // direction whose tangent at begin matches travel.
                    const int dtheta=travel.dot(field.cross_axis)>=0.0?1:-1;
                    double delta=theta_end-theta_begin;
                    while (delta> M_PI) delta-=2.0*M_PI;
                    while (delta<-M_PI) delta+=2.0*M_PI;
                    if (delta*dtheta<0.0) delta+=2.0*M_PI*dtheta;
                    const auto tangent_at=[&](double theta) {
                        return static_cast<double>(dtheta)*
                            (-std::sin(theta)*field.progress_axis+
                             std::cos(theta)*field.cross_axis);
                    };
                    const double arc=std::abs(delta)*std::abs(radius);
                    const std::size_t steps=std::max<std::size_t>(2,
                        static_cast<std::size_t>(std::ceil(arc/pitch)));
                    for (std::size_t step=0;step<steps;++step) {
                        const double theta=theta_begin+delta*
                            static_cast<double>(step)/
                            static_cast<double>(steps);
                        push(center+std::abs(radius)*
                            (std::cos(theta)*field.progress_axis+
                             std::sin(theta)*field.cross_axis),
                            tangent_at(theta),first.pass_index,true);
                    }
                }
            }
        }
        push_pass(passes.front(),false);
        for (std::size_t pass=1;pass<passes.size();++pass) {
            const Task22SweepPass& from=passes[pass-1];
            const Task22SweepPass& to=passes[pass];
            const double dp=to.progress-from.progress;
            if (!(std::abs(dp)>1.0e-9)) {
                result.reason="degenerate_pass_spacing:"+unit_id;
                return result;
            }
            // Preregistration section 1(iii): R = min(kappa_max^-1, dp/2)
            // with kappa_max = a_box/v_max^2 = 4/30^2 (no hand constant).
            // When 2R < |dp| the remaining |dp|-2R of progress is crossed
            // by a straight segment along the cross direction, keeping the
            // connector position- and tangent-continuous.
            constexpr double kappa_max=4.0/(30.0*30.0);
            const double radius=std::min(1.0/kappa_max,0.5*std::abs(dp));
            const double straight=std::max(0.0,std::abs(dp)-2.0*radius);
            const double turn=from.cross_end;
            const Eigen::Vector2d start=field.origin+
                from.progress*field.progress_axis+turn*field.cross_axis;
            const double travel_sign=static_cast<double>(from.direction);
            // Arc A: quarter turn from the travel tangent into the
            // progress direction; its centre sits one radius inward of the
            // turn line toward the route interior.
            const int progress_sign=dp>0.0?1:-1;
            const Eigen::Vector2d center_a=start+
                progress_sign*radius*field.progress_axis;
            const double theta_a0=std::atan2(
                field.cross_axis.dot(start-center_a),
                field.progress_axis.dot(start-center_a));
            // dtheta sign: the start tangent must equal travel*cross.
            const int dtheta_a=travel_sign*progress_sign>=0.0?-1:1;
            const auto arc_tangent=[&](const Eigen::Vector2d& center,
                double theta,int dsign) {
                return static_cast<double>(dsign)*
                    (-std::sin(theta)*field.progress_axis+
                     std::cos(theta)*field.cross_axis);
            };
            const double quarter=M_PI/2.0;
            const std::size_t steps_a=std::max<std::size_t>(4,
                static_cast<std::size_t>(std::ceil(quarter*radius/pitch))+1);
            Eigen::Vector2d junction=start;
            for (std::size_t step=1;step<=steps_a;++step) {
                const double theta=theta_a0+dtheta_a*quarter*
                    static_cast<double>(step)/static_cast<double>(steps_a);
                junction=center_a+radius*(std::cos(theta)*
                    field.progress_axis+std::sin(theta)*field.cross_axis);
                push(junction,arc_tangent(center_a,theta,dtheta_a),
                    to.pass_index,true);
            }
            if (straight>1.0e-9) {
                const Eigen::Vector2d straight_tangent=
                    progress_sign*field.progress_axis;
                const std::size_t steps_s=std::max<std::size_t>(1,
                    static_cast<std::size_t>(std::ceil(straight/pitch)));
                for (std::size_t step=1;step<=steps_s;++step) {
                    const double fraction=static_cast<double>(step)/
                        static_cast<double>(steps_s);
                    push(junction+fraction*straight*straight_tangent,
                        straight_tangent,to.pass_index,true);
                }
            }
            // Arc B: quarter turn from the progress direction back to the
            // opposite travel tangent, centred one radius inward of the
            // destination pass line.  The arc STARTS at the straight
            // segment's end (pure cross offset from the centre), not at
            // the pass-arrival point.
            const Eigen::Vector2d end=field.origin+
                to.progress*field.progress_axis+to.cross_begin*
                field.cross_axis;
            const double to_travel=static_cast<double>(to.direction);
            const Eigen::Vector2d center_b=end-
                progress_sign*radius*field.progress_axis;
            const Eigen::Vector2d straight_end=junction+
                straight*progress_sign*field.progress_axis;
            const double theta_b0=std::atan2(
                field.cross_axis.dot(straight_end-center_b),
                field.progress_axis.dot(straight_end-center_b));
            const int dtheta_b=to_travel*progress_sign>=0.0?1:-1;
            const std::size_t steps_b=std::max<std::size_t>(4,
                static_cast<std::size_t>(std::ceil(quarter*radius/pitch))+1);
            for (std::size_t step=1;step<=steps_b;++step) {
                const double theta=theta_b0+dtheta_b*quarter*
                    static_cast<double>(step)/static_cast<double>(steps_b);
                push(center_b+radius*(std::cos(theta)*
                    field.progress_axis+std::sin(theta)*field.cross_axis),
                    arc_tangent(center_b,theta,dtheta_b),to.pass_index,true);
            }
            push_pass(to,true);
        }
        double running=0.0;
        for (std::size_t index=0;index<route.samples.size();++index) {
            if (index>0) running+=(route.samples[index].position-
                route.samples[index-1].position).norm();
            route.samples[index].s=running;
        }
        route.total_length=running;
        if (route.samples.size()<2||!(route.total_length>0.0)) {
            result.reason="degenerate_route:"+unit_id;
            return result;
        }

        // Route-ordered first-service ledger for online windowing.
        task22_detail::CellBits serviced(words,0);
        for (const auto& sample:route.samples) {
            std::map<std::string,Eigen::Vector2d> fronts=
                geometry.other_fronts;
            fronts[unit_id]=sample.position;
            const auto lifted=task20LiftTargets(contract,fixed_positions,
                fronts);
            if (!lifted.valid) {
                result.reason="lifting_failed:"+unit_id;
                return result;
            }
            const double yaw=std::atan2(sample.tangent.y(),
                sample.tangent.x());
            double min_x=1.0e18,max_x=-1.0e18,min_y=1.0e18,max_y=-1.0e18;
            for (NodeId member:unit.members) {
                const Eigen::Vector2d pose=lifted.targets.at(member);
                min_x=std::min(min_x,pose.x());
                max_x=std::max(max_x,pose.x());
                min_y=std::min(min_y,pose.y());
                max_y=std::max(max_y,pose.y());
            }
            for (std::size_t index=0;index<cell_count;++index) {
                if (task22_detail::getBit(serviced,index)) continue;
                const FrontierCell& cell=result.cells[index];
                const double cell_cross=field.coordinates(cell.center).y();
                if (cell_cross+1.0e-9<corridor.cross_min||
                    cell_cross>corridor.cross_max+1.0e-9) continue;
                if (cell.center.x()<min_x-400.0||cell.center.x()>max_x+400.0||
                    cell.center.y()<min_y-400.0||
                    cell.center.y()>max_y+400.0) continue;
                for (NodeId member:unit.members)
                    if (task22CellInCertifiedSector(
                        lifted.targets.at(member),yaw,cell.center)) {
                        task22_detail::setBit(serviced,index);
                        route.cell_order.push_back({cell.id(),sample.s});
                        break;
                    }
            }
        }
        if (route.cell_order.empty()) {
            result.reason="route_services_no_cells:"+unit_id;
            return result;
        }
        result.corridors.emplace(unit_id,corridor);
        result.routes.emplace(unit_id,std::move(route));
    }
    result.valid=true;
    return result;
}

// Full-corridor pass poses (no insets, no fillets) for oracle cross-checks.
inline std::vector<std::pair<Eigen::Vector2d,double>>
task22FullExtentPoses(const Task22SweepPlan& plan,
    const std::string& coverage_unit) {
    std::vector<std::pair<Eigen::Vector2d,double>> result;
    const auto route=plan.routes.find(coverage_unit);
    if (route==plan.routes.end()) return result;
    const auto corridor=plan.corridors.at(coverage_unit);
    const double pitch=plan.sample_pitch_m;
    std::vector<double> pose_cross;
    for (double cross=corridor.cross_min+0.5*pitch;
         cross<=corridor.cross_max;cross+=pitch)
        pose_cross.push_back(cross);
    for (const auto& pass:route->second.passes)
        for (double cross:pose_cross) {
            const Eigen::Vector2d position=plan.field.origin+
                pass.progress*plan.field.progress_axis+
                cross*plan.field.cross_axis;
            const Eigen::Vector2d tangent=static_cast<double>(
                pass.direction)*plan.field.cross_axis;
            result.push_back({position,std::atan2(tangent.y(),tangent.x())});
        }
    return result;
}

inline Task22AllocationResult allocateTask22Sweep(
    const Task22AllocationRequest& request) {
    const auto started=std::chrono::steady_clock::now();
    Task22AllocationResult result;
    const auto finish=[&](Task22AllocationResult value) {
        value.allocation_wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return value;
    };
    if (!request.plan.valid||request.forward_focus_distance_m<0.0) {
        result.reason="invalid_sweep_allocation_request";
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
    for (const auto& [unit,route]:request.plan.routes)
        if (!result.cursors.count(unit)) result.cursors[unit]=0.0;
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
        // Monotone projection of the actual front onto the route ahead.
        const double previous=std::min(std::max(0.0,
            result.cursors.at(unit)),route.total_length);
        std::size_t index=0;
        while (index+1<route.samples.size()&&
               route.samples[index].s<previous-1.0e-9) ++index;
        double best=(front->second-route.samples[index].position).norm();
        std::size_t best_index=index;
        // Global projection over the remaining route (the earlier
        // first-increase walk stopped at approach-arc local minima and
        // froze the cursor at zero for whole runs).
        for (std::size_t probe=index+1;probe<route.samples.size();++probe) {
            const double distance=(front->second-
                route.samples[probe].position).norm();
            if (distance<best-request.comparison_epsilon) {
                best=distance;
                best_index=probe;
            }
        }
        const double cursor=route.samples[best_index].s;
        result.cursors[unit]=cursor;
        const double window_span=static_cast<double>(
            request.plan.local_window_cells)*request.plan.sample_pitch_m;
        // First pending work in route order: the navigation pose anchors to
        // the service point of the oldest uncovered window cell, so the
        // front only advances as certification clears pending work (the
        // arclength form of P0's nearest-real-cell pacing).  With no
        // pending cell in the window the pose advances freely along the
        // connecting arc by the forward-focus arclength.
        // Cell selection first: nearest real uncovered window cell to the
        // 400 m forward focus at the projected front (exact P0 pacing).
        const Task22RouteSample& projected=route.samples[best_index];
        const Eigen::Vector2d focus=projected.position+
            request.forward_focus_distance_m*projected.tangent;
        const FrontierCell* best_cell=nullptr;
        double best_distance=std::numeric_limits<double>::infinity();
        double best_cell_s=0.0;
        double first_pending_s=
            std::numeric_limits<double>::infinity();
        bool any_pending=false;
        for (const auto& service:route.cell_order) {
            if (service.first_service_s<previous-request.plan.sample_pitch_m)
                continue;
            if (service.first_service_s>cursor+window_span) break;
            const auto found=uncovered.find(service.cell_id);
            if (found==uncovered.end()||used.count(found->first)) continue;
            ++result.scanned_cells;
            any_pending=true;
            first_pending_s=std::min(first_pending_s,
                service.first_service_s);
            const double distance=(found->second.center-focus).squaredNorm();
            if (best_cell==nullptr||
                distance<best_distance-request.comparison_epsilon||
                (std::abs(distance-best_distance)<=
                     request.comparison_epsilon&&
                 found->first<best_cell->id())) {
                best_cell=&found->second;
                best_distance=distance;
                best_cell_s=service.first_service_s;
            }
        }
        // Navigation pose, frozen P5 anchor convention: the selected
        // cell's own first-service witness pose from the route ledger --
        // the exact configuration whose members' certified sectors cover
        // the cell, so parking there certifies it by construction.  (The
        // earlier pass-line-at-cell-cross reconstruction deadlocked on
        // cells whose witness lies on a fillet: no member's sector faced
        // the cell from the reconstructed pose.)  With no pending cell in
        // the window the pose advances freely along the connecting arc by
        // the forward-focus arclength.
        const bool window_empty=!any_pending;
        std::size_t nav_index=best_index;
        if (!window_empty) {
            // Nearest-to-focus witness pursuit (best-attested anchor,
            // 2026-09-04 P6 comparison): the front chases the witness pose
            // of the uncovered window cell nearest the 400 m forward
            // focus.  The oldest-pending variant (P6 addendum) was
            // dominated in the 300 s smoke (18.0% vs 39.6% certified) and
            // is recorded as adjudicated without a formal run.
            while (nav_index+1<route.samples.size()&&
                   route.samples[nav_index].s<best_cell_s)
                ++nav_index;
        } else {
            const double nav_target_s=
                cursor+request.forward_focus_distance_m;
            while (nav_index+1<route.samples.size()&&
                   route.samples[nav_index].s<nav_target_s)
                ++nav_index;
        }
        const Task22RouteSample& sample=route.samples[nav_index];
        const Eigen::Vector2d nav_pose=sample.position;
        const Eigen::Vector2d nav_tangent=sample.tangent;
        const bool nav_on_fillet=sample.on_fillet;
        Task22SweepAssignment assignment;
        assignment.coverage_unit=unit;
        assignment.cursor_s=cursor;
        assignment.route_pose=nav_pose;
        assignment.route_tangent=nav_tangent;
        assignment.route_sample_index=nav_index;
        assignment.on_fillet=nav_on_fillet;
        if (best_cell!=nullptr) {
            assignment.task=*best_cell;
        } else {
            // Route-order fallback: the next real uncovered cell along the
            // remaining route, else the globally nearest real cell to the
            // route pose (uniform completion rule; no tail mode, the cell
            // remains a real certified-uncovered ID and the navigation
            // pose stays the typed route pose).
            bool fallback=false;
            for (const auto& service:route.cell_order) {
                if (service.first_service_s<previous) continue;
                const auto found=uncovered.find(service.cell_id);
                if (found==uncovered.end()||used.count(found->first))
                    continue;
                assignment.task=found->second;
                fallback=true;
                break;
            }
            if (!fallback) {
                // Completion boundary: when only shared residual cells
                // remain, units may target the same real cell ID (the
                // recorded cross-squad duplicate-task efficiency boundary);
                // the window-based primary selection above stays deduped.
                double nearest=std::numeric_limits<double>::infinity();
                std::string nearest_id;
                for (const auto& [id,cell]:uncovered) {
                    const double distance=(cell.center-
                        sample.position).squaredNorm();
                    if (distance<nearest-request.comparison_epsilon||
                        (std::abs(distance-nearest)<=
                             request.comparison_epsilon&&
                         (nearest_id.empty()||id<nearest_id))) {
                        assignment.task=cell;
                        nearest=distance;
                        nearest_id=id;
                    }
                }
                if (nearest_id.empty()) {
                    result.reason="no_real_uncovered_cell:"+unit;
                    return finish(std::move(result));
                }
            }
            assignment.window_empty=true;
        }
        used.insert(assignment.task.id());
        result.assignments.emplace(unit,std::move(assignment));
    }
    result.valid=true;
    result.reason="task22_real_cell_assignments";
    return finish(std::move(result));
}

// Task 22 preregistration section 3: pinball 5-4-3-2 static contract.
// Mobile layers {1..5}/{6..9}/{10..12}/{13,14}, fixed {100,101,102}, 28
// edges with exactly two references per owner, layered pinball parents.
// Lifting is the shared affine triangular role map with axial fractions
// 0.25/0.50/0.75/1.00 by layer depth and interleaved triangular fractions
// per layer.  Offline oracle only; no closed loop is authorized by its
// existence.
inline Task20DagLatticeContract task22Pinball5432Contract() {
    Task20DagLatticeContract result;
    result.id="pinball-5-4-3-2";
    result.structural_signature="units=14;rows=5-4-3-2;pinball-layered";
    result.reference_edges={
        {100,1},{101,1},{100,2},{101,2},{101,3},{102,3},{101,4},{102,4},
        {100,5},{102,5},
        {1,6},{2,6},{2,7},{3,7},{3,8},{4,8},{4,9},{5,9},
        {6,10},{7,10},{7,11},{8,11},{8,12},{9,12},
        {10,13},{11,13},{11,14},{12,14}};
    result.coverage_units={{"P",{1,2,3,4,5,6,7,8,9,10,11,12,13,14},
        {100,101,102},14,{13,14}}};
    const std::vector<std::pair<double,double>> roles{
        {0.25,-0.24},{0.25,-0.12},{0.25,0.0},{0.25,0.12},{0.25,0.24},
        {0.50,-0.18},{0.50,-0.06},{0.50,0.06},{0.50,0.18},
        {0.75,-0.16},{0.75,0.0},{0.75,0.16},
        {1.00,-0.08},{1.00,0.08}};
    task20_lattice_detail::addRoles(result,result.coverage_units.front(),
        roles);
    task20_lattice_detail::finish(result);
    return result;
}

}  // namespace gf
