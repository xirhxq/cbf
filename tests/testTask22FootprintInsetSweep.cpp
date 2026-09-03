#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task22FootprintInsetSweep.hpp"
#include "grand_finale/Task20DagLatticeContract.hpp"

#include <cmath>

namespace {

std::vector<gf::FrontierCell> grid(int x_count,int y_count) {
    std::vector<gf::FrontierCell> result;
    for (int x=0;x<x_count;++x) for (int y=0;y<y_count;++y)
        result.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    return result;
}

gf::Task21CoordinateField field() {
    // progress along +y, cross along +x (Task 21 formal field).
    return gf::task21AffineCoordinateField({0.0,0.0},{0.0,1.0},{1.0,0.0});
}

gf::Task20DagLatticeContract dualLadder() {
    return gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
}

gf::Task20DagLatticeContract splitThree() {
    return gf::task20DagLatticeContract(gf::Task20LatticeMode::SplitThreeFront);
}

gf::Task20DagLatticeContract mergedStrip() {
    return gf::task20DagLatticeContract(gf::Task20LatticeMode::MergedStrip);
}

std::map<gf::NodeId,Eigen::Vector2d> fixedAnchors() {
    return {{100,{-50.0,75.0}},{101,{150.0,25.0}},{102,{350.0,125.0}}};
}

std::map<std::string,Eigen::Vector2d> initialFronts(
    const gf::Task20DagLatticeContract& contract,
    const std::map<gf::NodeId,Eigen::Vector2d>& mobile) {
    std::map<std::string,Eigen::Vector2d> fronts;
    for (const auto& unit:contract.coverage_units) {
        Eigen::Vector2d front=Eigen::Vector2d::Zero();
        const auto& members=unit.front_members.empty()
            ?std::vector<gf::NodeId>{unit.leader}:unit.front_members;
        for (gf::NodeId member:members) front+=mobile.at(member);
        fronts[unit.id]=front/static_cast<double>(members.size());
    }
    return fronts;
}

std::map<gf::NodeId,Eigen::Vector2d> mobileAt(
    const gf::Task20DagLatticeContract& contract,
    const Eigen::Vector2d& seed) {
    std::map<gf::NodeId,Eigen::Vector2d> mobile;
    for (const auto& unit:contract.coverage_units)
        for (std::size_t index=0;index<unit.members.size();++index)
            mobile[unit.members[index]]=seed+
                10.0*static_cast<double>(index)*Eigen::Vector2d::UnitX();
    return mobile;
}

}  // namespace

TEST_CASE("Task 22 builds continuous routes for one two and three units with complete corridors") {
    const std::array<std::function<gf::Task20DagLatticeContract()>,3> contracts{
        mergedStrip,dualLadder,splitThree};
    for (std::size_t index=0;index<contracts.size();++index) {
        const auto contract=contracts[index]();
        REQUIRE(contract.valid);
        const auto cells=grid(90,90);
        std::vector<std::string> units;
        for (const auto& unit:contract.coverage_units) units.push_back(unit.id);
        const auto plan=gf::task22BuildSweepPlan(cells,units,field(),220.0,64,
            contract,fixedAnchors(),initialFronts(contract,mobileAt(contract,{45.0,15.0})));
        CAPTURE(index);
        REQUIRE(plan.valid);
        REQUIRE(plan.routes.size()==units.size());
        std::set<std::string> seen;
        for (const auto& [unit,route]:plan.routes) {
            REQUIRE(!route.samples.empty());
            REQUIRE(route.total_length>0.0);
            for (std::size_t sample=1;sample<route.samples.size();++sample)
                CHECK(route.samples[sample].s>route.samples[sample-1].s);
            for (const auto& service:route.cell_order)
                CHECK(seen.insert(service.cell_id).second);
        }
        CHECK(seen.size()==cells.size());
    }
}

TEST_CASE("Task 22 route tangent is continuous with no discrete endpoint reversal") {
    const auto contract=dualLadder();
    const auto plan=gf::task22BuildSweepPlan(grid(90,90),{"A","B"},field(),
        220.0,64,contract,fixedAnchors(),
        initialFronts(contract,mobileAt(contract,{45.0,15.0})));
    REQUIRE(plan.valid);
    for (const auto& [unit,route]:plan.routes) {
        double maximum_turn_deg=0.0;
        for (std::size_t sample=1;sample<route.samples.size();++sample) {
            const double dot=std::max(-1.0,std::min(1.0,
                route.samples[sample-1].tangent.dot(route.samples[sample].tangent)));
            maximum_turn_deg=std::max(maximum_turn_deg,
                std::acos(dot)*180.0/M_PI);
        }
        CAPTURE(unit);
        CAPTURE(maximum_turn_deg);
        // P4 turns 180 degrees between adjacent band samples; P5 must stay
        // bounded by the fillet curvature over one sample pitch.
        CHECK(maximum_turn_deg<=30.0);
        for (std::size_t sample=0;sample<route.samples.size();++sample)
            CHECK(std::abs(route.samples[sample].tangent.norm()-1.0)<1.0e-9);
    }
}

TEST_CASE("Task 22 insets are footprint-derived and trim no service from the unit union") {
    const auto contract=dualLadder();
    const auto cells=grid(60,140);
    const auto plan=gf::task22BuildSweepPlan(cells,{"A","B"},field(),220.0,64,
        contract,fixedAnchors(),
        initialFronts(contract,mobileAt(contract,{45.0,15.0})));
    REQUIRE(plan.valid);
    for (const auto& [unit,route]:plan.routes) {
        const auto& corridor=plan.corridors.at(unit);
        double maximum_cross=0.0;
        double minimum_cross=1.0e9;
        for (const auto& sample:route.samples) {
            const double cross=plan.field.coordinates(sample.position).y();
            maximum_cross=std::max(maximum_cross,cross);
            minimum_cross=std::min(minimum_cross,cross);
        }
        CAPTURE(unit);
        // With a 1400 m corridor and a <=400 m certified footprint the
        // inward-most endpoint must sit strictly inside the corridor extreme.
        const double fillet_radius=0.5*plan.pass_spacing_m;
        CHECK(maximum_cross<corridor.cross_max+fillet_radius+1.0e-6);
        CHECK(minimum_cross>corridor.cross_min-fillet_radius-1.0e-6);
        // Trimming the extreme poses must not drop any cell that the full
        // extent of this unit's own route would service.
        std::set<std::string> full_service;
        for (const auto& [pose,yaw]:task22FullExtentPoses(plan,unit))
            for (const auto& cell:plan.cells) {
                const double cross=plan.field.coordinates(cell.center).y();
                if (cross+1.0e-9<corridor.cross_min||
                    cross>corridor.cross_max+1.0e-9) continue;
                if (gf::task22CellInCertifiedSector(pose,yaw,cell.center))
                    full_service.insert(cell.id());
            }
        std::set<std::string> route_service;
        for (const auto& service:route.cell_order)
            route_service.insert(service.cell_id);
        // Trimming must never drop a cell; fillet poses may add coverage.
        CHECK(std::includes(route_service.begin(),route_service.end(),
            full_service.begin(),full_service.end()));
    }
}

TEST_CASE("Task 22 allocator keeps cursors monotone and one real-ID rule for zero through three residuals") {
    const auto contract=dualLadder();
    const auto cells=grid(40,60);
    const auto plan=gf::task22BuildSweepPlan(cells,{"A","B"},field(),220.0,32,
        contract,fixedAnchors(),
        initialFronts(contract,mobileAt(contract,{45.0,15.0})));
    REQUIRE(plan.valid);
    gf::Task22AllocationRequest request;
    request.plan=plan;
    request.uncovered_cells=cells;
    request.front_positions=initialFronts(contract,mobileAt(contract,{45.0,15.0}));
    std::map<std::string,double> cursors;
    for (const auto& [unit,route]:plan.routes) cursors[unit]=0.0;
    std::map<std::string,double> previous=cursors;
    std::map<std::string,std::set<std::string>> consumed;
    for (int step=0;step<8;++step) {
        request.cursors=cursors;
        const auto result=gf::allocateTask22Sweep(request);
        REQUIRE(result.valid);
        CHECK(result.assignments.size()<=plan.routes.size());
        for (const auto& [unit,assignment]:result.assignments) {
            const auto id=assignment.task.id();
            CAPTURE(unit);
            CAPTURE(id);
            CHECK(!id.empty());
            CHECK(id.find('-')==std::string::npos);
            CHECK(id.find(':')!=std::string::npos);
            CHECK(assignment.cursor_s>=previous.at(unit)-1.0e-12);
            CHECK(assignment.route_pose.allFinite());
            CHECK(std::abs(assignment.route_tangent.norm()-1.0)<1.0e-9);
        }
        cursors=result.cursors;
        previous=cursors;
        // Advance the synthetic fronts along the assigned route poses so the
        // cursor projection can move forward.
        for (const auto& [unit,assignment]:result.assignments)
            request.front_positions[unit]=assignment.route_pose+
                40.0*assignment.route_tangent;
    }
    // Same rule for 0..3 residual cells after consuming the whole grid.
    request.uncovered_cells.clear();
    request.cursors=cursors;
    const auto drained=gf::allocateTask22Sweep(request);
    REQUIRE(drained.valid);
    CHECK(drained.complete);
    for (int residuals=0;residuals<=3;++residuals) {
        request.uncovered_cells.clear();
        for (int index=0;index<residuals;++index)
            request.uncovered_cells.push_back(cells[7*index+3]);
        request.cursors=cursors;
        const auto result=gf::allocateTask22Sweep(request);
        CAPTURE(result.reason);
        REQUIRE(result.valid);
        for (const auto& [unit,assignment]:result.assignments)
            CHECK(assignment.task.id().find(':')!=std::string::npos);
    }
}

TEST_CASE("Task 22 route is coordinate-field equivariant") {
    const auto contract=dualLadder();
    const auto fronts=initialFronts(contract,mobileAt(contract,{45.0,15.0}));
    const auto original=gf::task22BuildSweepPlan(grid(90,90),{"A","B"},field(),
        220.0,64,contract,fixedAnchors(),fronts);
    const auto rotated_field=gf::task21AffineCoordinateField(
        {0.0,0.0},{1.0,0.0},{0.0,1.0});
    std::vector<gf::FrontierCell> rotated;
    for (const auto& cell:grid(90,90))
        rotated.push_back({cell.y_index,cell.x_index,
            {cell.center.y(),cell.center.x()}});
    std::map<gf::NodeId,Eigen::Vector2d> rotated_anchors;
    std::map<std::string,Eigen::Vector2d> rotated_fronts;
    for (const auto& [id,pose]:fixedAnchors())
        rotated_anchors[id]={pose.y(),pose.x()};
    for (const auto& [id,pose]:fronts) rotated_fronts[id]={pose.y(),pose.x()};
    const auto rotated_plan=gf::task22BuildSweepPlan(rotated,{"A","B"},
        rotated_field,220.0,64,contract,rotated_anchors,rotated_fronts);
    REQUIRE(original.valid);
    REQUIRE(rotated_plan.valid);
    for (const auto& [unit,route]:original.routes) {
        const auto& other=rotated_plan.routes.at(unit);
        REQUIRE(other.samples.size()==route.samples.size());
        CHECK(std::abs(other.total_length-route.total_length)<1.0e-6);
        for (std::size_t sample=0;sample<route.samples.size();++sample) {
            CHECK(std::abs(route.samples[sample].position.x()-
                other.samples[sample].position.y())<1.0e-9);
            CHECK(std::abs(route.samples[sample].position.y()-
                other.samples[sample].position.x())<1.0e-9);
        }
    }
}

TEST_CASE("Task 22 first pass starts on the side nearest the initial front") {
    const auto contract=dualLadder();
    std::map<gf::NodeId,Eigen::Vector2d> low_mobile,high_mobile;
    for (const auto& unit:contract.coverage_units)
        for (std::size_t index=0;index<unit.members.size();++index) {
            low_mobile[unit.members[index]]=Eigen::Vector2d(15.0+10.0*index,15.0);
            high_mobile[unit.members[index]]=
                Eigen::Vector2d(885.0-10.0*index,15.0);
        }
    const auto cells=grid(90,90);
    const auto low=gf::task22BuildSweepPlan(cells,{"A","B"},field(),220.0,64,
        contract,fixedAnchors(),initialFronts(contract,low_mobile));
    const auto high=gf::task22BuildSweepPlan(cells,{"A","B"},field(),220.0,64,
        contract,fixedAnchors(),initialFronts(contract,high_mobile));
    REQUIRE(low.valid);
    REQUIRE(high.valid);
    for (const auto& [unit,route]:low.routes)
        CHECK(route.passes.front().direction==1);
    for (const auto& [unit,route]:high.routes)
        CHECK(route.passes.front().direction==-1);
}

TEST_CASE("Task 22 pinball-5-4-3-2 static contract is valid and parameter-free") {
    const auto contract=gf::task22Pinball5432Contract();
    REQUIRE(contract.valid);
    REQUIRE(contract.reference_edges.size()==28);
    REQUIRE(contract.member_roles.size()==14);
    REQUIRE(contract.coverage_units.size()==1);
    REQUIRE(contract.coverage_units.front().members.size()==14);
    const std::vector<gf::NodeId> expected_front{13,14};
    CHECK(contract.coverage_units.front().front_members==expected_front);
    // Layered mobile parents: rows {1..5}, {6..9}, {10..12}, {13,14}.
    std::map<gf::NodeId,std::vector<gf::NodeId>> parents;
    for (const auto& edge:contract.reference_edges)
        parents[edge.owner].push_back(edge.reference);
    REQUIRE(parents.size()==14);
    for (const auto& entry:parents) CHECK(entry.second.size()==2);
    for (gf::NodeId owner=1;owner<=5;++owner)
        for (gf::NodeId ref:parents[owner]) CHECK(ref>=100);
    for (gf::NodeId owner=6;owner<=9;++owner)
        for (gf::NodeId ref:parents[owner]) { CHECK(ref>=1); CHECK(ref<=5); }
    for (gf::NodeId owner=10;owner<=12;++owner)
        for (gf::NodeId ref:parents[owner]) { CHECK(ref>=6); CHECK(ref<=9); }
    for (gf::NodeId owner=13;owner<=14;++owner)
        for (gf::NodeId ref:parents[owner]) { CHECK(ref>=10); CHECK(ref<=12); }
    // Closed-form lifting is the shared triangular affine map; the oracle
    // gate on the full map is exercised by GrandFinaleTask22RouteOracle.
    const auto plan=gf::task22BuildSweepPlan(grid(60,60),{"P"},
        gf::task21AffineCoordinateField({0.0,0.0},{0.0,1.0},{1.0,0.0}),
        220.0,64,contract,fixedAnchors(),
        std::map<std::string,Eigen::Vector2d>{{"P",{45.0,15.0}}});
    REQUIRE(plan.valid);
}
