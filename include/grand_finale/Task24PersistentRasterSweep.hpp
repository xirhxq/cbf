#pragma once

#include "grand_finale/Task20DagLatticeContract.hpp"
#include "grand_finale/Task21PersistentRibbon.hpp"

#include <chrono>
#include <optional>
#include <set>

namespace gf {

enum class Task24LatticeMode {
    H0DualLadder,
    Pinball5432,
    LongTriangleSingleLadder
};

struct Task24DagContract {
    bool valid=false;
    std::string reason;
    Task24LatticeMode mode=Task24LatticeMode::H0DualLadder;
    std::string id;
    double width_m=0.0;
    double height_m=0.0;
    std::vector<DirectedEdge> reference_edges;
    std::vector<Task20CoverageUnit> coverage_units;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    Eigen::Vector2d entrance_center=Eigen::Vector2d::Zero();
};

struct Task24LiftResult {
    bool valid=false;
    std::string reason;
    std::map<NodeId,Eigen::Vector2d> targets;
};

namespace task24_detail {

inline bool validate(Task24DagContract& contract) {
    if (contract.reference_edges.size()!=28||
        contract.coverage_units.empty()) {
        contract.reason="contract_cardinality";
        return false;
    }
    std::map<NodeId,std::size_t> indegree;
    std::set<std::string> edges;
    std::set<NodeId> members;
    for (const auto& edge:contract.reference_edges) {
        if (edge.owner<1||edge.owner>14||
            !edges.insert(edge.id()).second) {
            contract.reason="invalid_or_duplicate_edge";
            return false;
        }
        ++indegree[edge.owner];
    }
    for (NodeId id=1;id<=14;++id)
        if (indegree[id]!=2) {
            contract.reason="owner_reference_count";
            return false;
        }
    for (const auto& unit:contract.coverage_units) {
        if (unit.members.empty()||unit.front_members.empty()) {
            contract.reason="invalid_coverage_unit";
            return false;
        }
        for (NodeId id:unit.members)
            if (!members.insert(id).second) {
                contract.reason="duplicate_coverage_member";
                return false;
            }
    }
    if (members.size()!=14) {
        contract.reason="missing_coverage_member";
        return false;
    }
    contract.valid=true;
    return true;
}

inline std::vector<DirectedEdge> longTriangleEdges() {
    std::vector<DirectedEdge> edges{{100,1},{101,1},{100,2},{101,2},
        {1,3},{2,3}};
    for (NodeId owner=4;owner<=14;++owner) {
        edges.emplace_back(owner-2,owner);
        edges.emplace_back(owner-1,owner);
    }
    return edges;
}

inline double maximumLongTriangleHalfWidth() {
    const Eigen::Vector2d center{1000.0,-50.0};
    double maximum=0.0;
    for (const Eigen::Vector2d corner:std::array<Eigen::Vector2d,4>{
        Eigen::Vector2d{0.0,0.0},Eigen::Vector2d{2000.0,0.0},
        Eigen::Vector2d{0.0,4500.0},Eigen::Vector2d{2000.0,4500.0}}) {
        const double h=(corner-center).norm()/13.0;
        maximum=std::max(maximum,std::sqrt(3.0)*h/2.0);
    }
    return maximum;
}

inline std::size_t nearestPass(double progress,
    const std::vector<double>& passes) {
    std::size_t best=0;
    double distance=std::abs(progress-passes.front());
    for (std::size_t index=1;index<passes.size();++index) {
        const double candidate=std::abs(progress-passes[index]);
        if (candidate<distance-1.0e-9) {
            best=index;
            distance=candidate;
        }
    }
    return best;
}

}  // namespace task24_detail

inline Task24DagContract task24Contract(Task24LatticeMode mode) {
    Task24DagContract result;
    result.mode=mode;
    if (mode==Task24LatticeMode::H0DualLadder) {
        const auto h0=task20DagLatticeContract(Task20LatticeMode::DualLadder);
        result.id="h0-dual-ladder";
        result.width_m=3000.0;
        result.height_m=3000.0;
        result.reference_edges=h0.reference_edges;
        result.coverage_units=h0.coverage_units;
        result.fixed_positions=task10p11pStandardCoastalAnchors();
        result.entrance_center={1500.0,-50.0};
    } else if (mode==Task24LatticeMode::Pinball5432) {
        result.id="pinball-5-4-3-2";
        result.width_m=4500.0;
        result.height_m=2000.0;
        for (NodeId k=0;k<6;++k)
            result.fixed_positions[100+k]={900.0*k,-50.0};
        result.entrance_center={2250.0,-50.0};
        result.reference_edges={
            {100,1},{101,1},{101,2},{102,2},{102,3},{103,3},
            {103,4},{104,4},{104,5},{105,5},
            {1,6},{2,6},{2,7},{3,7},{3,8},{4,8},{4,9},{5,9},
            {6,10},{7,10},{7,11},{8,11},{8,12},{9,12},
            {10,13},{11,13},{11,14},{12,14}};
        result.coverage_units={{"P",task10p10MobileIds(14),
            {100,101,102,103,104,105},14,{13,14}}};
    } else {
        result.id="long-triangle-single-ladder";
        result.width_m=2000.0;
        result.height_m=4500.0;
        result.entrance_center={1000.0,-50.0};
        const double d=task24_detail::maximumLongTriangleHalfWidth();
        result.fixed_positions={{100,result.entrance_center+Eigen::Vector2d{-d,0.0}},
                                {101,result.entrance_center+Eigen::Vector2d{d,0.0}}};
        result.reference_edges=task24_detail::longTriangleEdges();
        result.coverage_units={{"T",task10p10MobileIds(14),
            {100,101},14,{13,14}}};
    }
    task24_detail::validate(result);
    return result;
}

inline Task24LiftResult task24LiftTargets(
    const Task24DagContract& contract,
    const std::map<std::string,Eigen::Vector2d>& fronts) {
    Task24LiftResult result;
    if (!contract.valid) {
        result.reason="invalid_contract";
        return result;
    }
    const auto required=contract.coverage_units.front().id;
    if (contract.mode==Task24LatticeMode::H0DualLadder) {
        const auto lifted=task20LiftTargets(
            task20DagLatticeContract(Task20LatticeMode::DualLadder),
            contract.fixed_positions,fronts);
        result.valid=lifted.valid;
        result.reason=lifted.reason;
        result.targets=lifted.targets;
        return result;
    }
    const auto found=fronts.find(required);
    if (found==fronts.end()||!found->second.allFinite()) {
        result.reason="missing_front";
        return result;
    }
    const Eigen::Vector2d g=found->second;
    if (contract.mode==Task24LatticeMode::Pinball5432) {
        constexpr double width=4500.0;
        constexpr double spacing=900.0;
        NodeId id=1;
        for (int row=1;row<=4;++row) {
            const int count=6-row;
            const double alpha=static_cast<double>(row)/4.0;
            for (int slot=0;slot<count;++slot,++id)
                result.targets[id]={
                    (slot+0.5*row)*spacing+alpha*(g.x()-0.5*width),
                    -50.0+alpha*(g.y()+50.0)};
        }
    } else {
        const Eigen::Vector2d center=contract.entrance_center;
        const Eigen::Vector2d v=g-center;
        const double length=v.norm();
        if (length<1.0e-12) {
            result.reason="degenerate_triangle_front";
            return result;
        }
        const double h=length/13.0;
        const Eigen::Vector2d tangent=v/length;
        const Eigen::Vector2d normal{-tangent.y(),tangent.x()};
        const double d=std::sqrt(3.0)*h/2.0;
        result.targets[1]=center+h*tangent-d*normal;
        result.targets[2]=center+h*tangent+d*normal;
        for (NodeId id=3;id<=14;++id)
            result.targets[id]=center+(id-1)*h*tangent+
                (id%2==0?d:-d)*normal;
    }
    result.valid=result.targets.size()==14;
    if (!result.valid) result.reason="incomplete_lifting";
    return result;
}

inline std::optional<Eigen::Vector2d> task24ActualFront(
    const Task24DagContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& actual_positions,
    const std::string& unit_id) {
    const auto unit=std::find_if(contract.coverage_units.begin(),
        contract.coverage_units.end(),[&](const auto& value) {
            return value.id==unit_id;
        });
    if (unit==contract.coverage_units.end()) return std::nullopt;
    Eigen::Vector2d midpoint=Eigen::Vector2d::Zero();
    for (NodeId id:unit->front_members) {
        const auto found=actual_positions.find(id);
        if (found==actual_positions.end()) return std::nullopt;
        midpoint+=found->second;
    }
    midpoint/=static_cast<double>(unit->front_members.size());
    if (contract.mode==Task24LatticeMode::LongTriangleSingleLadder)
        return contract.entrance_center+(26.0/25.0)*
            (midpoint-contract.entrance_center);
    return midpoint;
}

struct Task24CellAssignment {
    std::string coverage_unit;
    std::size_t band_index=0;
};

struct Task24RasterPlan {
    bool valid=false;
    std::string reason;
    Task21CoordinateField field;
    double pass_spacing_m=0.0;
    std::vector<double> pass_progress_m;
    std::vector<FrontierCell> cells;
    std::map<std::string,FrontierCell> cell_lookup;
    std::vector<Task21RibbonCorridor> corridors;
    std::map<std::string,Task24CellAssignment> cell_assignments;
};

inline Task24RasterPlan task24BuildRasterPlan(
    std::vector<FrontierCell> cells,std::vector<std::string> units,
    const Task21CoordinateField& field,double spacing_m,
    const std::set<std::string>& initially_certified) {
    Task24RasterPlan result;
    result.field=field;
    result.pass_spacing_m=spacing_m;
    if (!field.valid||cells.empty()||units.empty()||!(spacing_m>0.0)) {
        result.reason="invalid_raster_plan_request";
        return result;
    }
    std::sort(units.begin(),units.end());
    if (std::adjacent_find(units.begin(),units.end())!=units.end()) {
        result.reason="duplicate_coverage_unit";
        return result;
    }
    std::sort(cells.begin(),cells.end(),[](const auto& a,const auto& b) {
        return a.id()<b.id();
    });
    result.cells=cells;
    double pmin=1.0e18,pmax=-1.0e18;
    std::map<double,std::size_t> cross_workload;
    for (const auto& cell:cells) {
        result.cell_lookup.emplace(cell.id(),cell);
        const Eigen::Vector2d q=field.coordinates(cell.center);
        pmin=std::min(pmin,q.x());
        pmax=std::max(pmax,q.x());
        if (!initially_certified.count(cell.id())) ++cross_workload[q.y()];
        else cross_workload.try_emplace(q.y(),0);
    }
    for (double p=pmin+0.5*spacing_m;p<=pmax+0.5*spacing_m+1e-9;
         p+=spacing_m)
        result.pass_progress_m.push_back(std::min(p,pmax));
    if (result.pass_progress_m.empty()) result.pass_progress_m.push_back(pmin);
    result.pass_progress_m.erase(std::unique(result.pass_progress_m.begin(),
        result.pass_progress_m.end()),result.pass_progress_m.end());
    std::vector<double> crosses;
    std::vector<std::size_t> weights;
    for (const auto& [cross,weight]:cross_workload) {
        crosses.push_back(cross);
        weights.push_back(weight);
    }
    std::size_t total=0;
    for (auto value:weights) total+=value;
    std::vector<std::size_t> cuts{0};
    std::size_t cumulative=0,index=0;
    for (std::size_t unit=1;unit<units.size();++unit) {
        const double desired=static_cast<double>(total)*unit/units.size();
        while (index+1<crosses.size()&&cumulative+weights[index]<desired) {
            cumulative+=weights[index++];
        }
        cuts.push_back(std::max(cuts.back()+1,index+1));
    }
    cuts.push_back(crosses.size());
    if (cuts[cuts.size()-2]>=crosses.size()) {
        result.reason="insufficient_cross_track_columns";
        return result;
    }
    for (std::size_t unit=0;unit<units.size();++unit) {
        const double low=unit==0?crosses.front()-1e-6:
            0.5*(crosses[cuts[unit]-1]+crosses[cuts[unit]]);
        const double high=unit+1==units.size()?crosses.back()+1e-6:
            0.5*(crosses[cuts[unit+1]-1]+crosses[cuts[unit+1]]);
        result.corridors.push_back({units[unit],low,high,0});
    }
    for (const auto& cell:cells) {
        const auto q=field.coordinates(cell.center);
        std::size_t unit_index=0;
        while (unit_index+1<result.corridors.size()&&
               q.y()>=result.corridors[unit_index].cross_max-1e-9)
            ++unit_index;
        auto& corridor=result.corridors[unit_index];
        if (!initially_certified.count(cell.id())) ++corridor.workload;
        result.cell_assignments[cell.id()]={corridor.coverage_unit,
            task24_detail::nearestPass(q.x(),result.pass_progress_m)};
    }
    result.valid=result.cell_assignments.size()==cells.size();
    if (!result.valid) result.reason="incomplete_cell_assignment";
    return result;
}

struct Task24RasterState {
    std::size_t band_index=0;
    int direction=1;
    double pass_cursor_m=0.0;
    double route_cursor_m=0.0;
    std::size_t pass_epoch=0;
    std::size_t same_band_rescans=0;
    std::optional<FrontierCell> last_real_task;
    bool has_last_continuous_front=false;
    Eigen::Vector2d last_continuous_front=Eigen::Vector2d::Zero();
};

inline Task24RasterState task24InitialRasterState(
    const Task24RasterPlan& plan,const std::string& unit) {
    if (!plan.valid||std::none_of(plan.corridors.begin(),plan.corridors.end(),
        [&](const auto& value) { return value.coverage_unit==unit; }))
        throw std::invalid_argument("unknown Task24 raster unit");
    return {};
}

struct Task24RasterRequest {
    Task24RasterPlan plan;
    std::vector<FrontierCell> uncovered_cells;
    std::map<std::string,Eigen::Vector2d> actual_fronts;
    std::map<std::string,Task24RasterState> states;
};

struct Task24RasterAssignment {
    bool active=false;
    FrontierCell task;
    Eigen::Vector2d continuous_front=Eigen::Vector2d::Zero();
    Task24RasterState state;
};

struct Task24RasterResult {
    bool valid=false;
    bool complete=false;
    std::string reason;
    double allocation_wall_s=0.0;
    std::size_t scanned_cells=0;
    std::map<std::string,Task24RasterAssignment> assignments;
};

inline Task24RasterResult allocateTask24Raster(Task24RasterRequest request) {
    const auto started=std::chrono::steady_clock::now();
    Task24RasterResult result;
    if (!request.plan.valid) {
        result.reason="invalid_plan";
        return result;
    }
    result.complete=request.uncovered_cells.empty();
    std::set<std::string> used;
    for (const auto& corridor:request.plan.corridors) {
        const std::string& unit=corridor.coverage_unit;
        const auto front=request.actual_fronts.find(unit);
        const auto state_it=request.states.find(unit);
        if (front==request.actual_fronts.end()||state_it==request.states.end()) {
            result.reason="missing_unit_runtime";
            return result;
        }
        Task24RasterState state=state_it->second;
        state.band_index=std::min(state.band_index,
            request.plan.pass_progress_m.size()-1);
        const double length=corridor.cross_max-corridor.cross_min;
        const Eigen::Vector2d q=request.plan.field.coordinates(front->second);
        const double projection=state.direction>0
            ?q.y()-corridor.cross_min:corridor.cross_max-q.y();
        const double old_cursor=state.pass_cursor_m;
        state.pass_cursor_m=std::max(state.pass_cursor_m,
            std::clamp(projection,0.0,length));
        state.route_cursor_m+=state.pass_cursor_m-old_cursor;
        auto band_pending=[&](std::size_t band) {
            return std::any_of(request.uncovered_cells.begin(),
                request.uncovered_cells.end(),[&](const auto& cell) {
                    const auto found=request.plan.cell_assignments.find(cell.id());
                    return found!=request.plan.cell_assignments.end()&&
                        found->second.coverage_unit==unit&&
                        found->second.band_index==band;
                });
        };
        constexpr double endpoint_tolerance_m=5.0+1e-9;
        if (state.pass_cursor_m>=length-endpoint_tolerance_m) {
            if (band_pending(state.band_index)) {
                state.direction=-state.direction;
                state.pass_cursor_m=0.0;
                ++state.pass_epoch;
                ++state.same_band_rescans;
            } else if (state.band_index+1<request.plan.pass_progress_m.size()) {
                ++state.band_index;
                state.direction=-state.direction;
                state.pass_cursor_m=0.0;
                ++state.pass_epoch;
            }
        }
        const double goal_cursor=std::min(length,state.pass_cursor_m+400.0);
        const double goal_cross=state.direction>0
            ?corridor.cross_min+goal_cursor:corridor.cross_max-goal_cursor;
        const double goal_progress=request.plan.pass_progress_m[state.band_index];
        Task24RasterAssignment assignment;
        assignment.continuous_front=request.plan.field.origin+
            goal_progress*request.plan.field.progress_axis+
            goal_cross*request.plan.field.cross_axis;
        const FrontierCell* best=nullptr;
        std::tuple<std::size_t,int,double,std::string> best_key;
        for (const auto& cell:request.uncovered_cells) {
            ++result.scanned_cells;
            const auto plan_cell=request.plan.cell_assignments.find(cell.id());
            if (plan_cell==request.plan.cell_assignments.end()||
                plan_cell->second.coverage_unit!=unit||used.count(cell.id()))
                continue;
            const double cross=request.plan.field.coordinates(cell.center).y();
            const double along=state.direction>0
                ?cross-corridor.cross_min:corridor.cross_max-cross;
            const bool ahead=plan_cell->second.band_index>state.band_index||
                (plan_cell->second.band_index==state.band_index&&
                 along+1e-9>=state.pass_cursor_m);
            const auto key=std::make_tuple(plan_cell->second.band_index,
                ahead?0:1,ahead?along:length-along,cell.id());
            if (!best||key<best_key) {
                best=&cell;
                best_key=key;
            }
        }
        if (best) {
            assignment.active=true;
            assignment.task=*best;
            state.last_real_task=*best;
            state.has_last_continuous_front=true;
            state.last_continuous_front=assignment.continuous_front;
            used.insert(best->id());
        } else if (state.last_real_task.has_value()) {
            assignment.task=*state.last_real_task;
            if (state.has_last_continuous_front)
                assignment.continuous_front=state.last_continuous_front;
        }
        assignment.state=state;
        result.assignments[unit]=assignment;
    }
    result.valid=true;
    result.reason=result.complete?"task24_complete":"task24_real_cell_ledger";
    result.allocation_wall_s=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-started).count();
    return result;
}

}  // namespace gf
