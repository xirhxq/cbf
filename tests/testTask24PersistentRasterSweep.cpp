#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task24PersistentRasterSweep.hpp"

namespace {

std::vector<gf::FrontierCell> grid(int nx=12,int ny=12) {
    std::vector<gf::FrontierCell> cells;
    for (int x=0;x<nx;++x) for (int y=0;y<ny;++y)
        cells.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    return cells;
}

gf::Task21CoordinateField field() {
    return gf::task21AffineCoordinateField(
        {0.0,0.0},{0.0,1.0},{1.0,0.0});
}

}  // namespace

TEST_CASE("Task24 frozen DAG contracts have 28 edges and exact coverage units") {
    const auto h0=gf::task24Contract(gf::Task24LatticeMode::H0DualLadder);
    const auto pinball=gf::task24Contract(gf::Task24LatticeMode::Pinball5432);
    const auto triangle=gf::task24Contract(
        gf::Task24LatticeMode::LongTriangleSingleLadder);
    for (const auto* contract:{&h0,&pinball,&triangle}) {
        REQUIRE(contract->valid);
        CHECK(contract->reference_edges.size()==28);
        std::set<gf::NodeId> owners;
        for (const auto& edge:contract->reference_edges)
            owners.insert(edge.owner);
        CHECK(owners.size()==14);
    }
    CHECK(h0.coverage_units.size()==2);
    CHECK(pinball.coverage_units.size()==1);
    CHECK(triangle.coverage_units.size()==1);
    CHECK(pinball.fixed_positions.size()==6);
    CHECK(pinball.coverage_units.front().front_members==
        std::vector<gf::NodeId>{13,14});
    CHECK(triangle.coverage_units.front().front_members==
        std::vector<gf::NodeId>{13,14});
}

TEST_CASE("Task24 H0 lifting exactly reproduces Task20 dual ladder") {
    const auto contract=gf::task24Contract(gf::Task24LatticeMode::H0DualLadder);
    const std::map<std::string,Eigen::Vector2d> fronts{
        {"A",{370.0,1420.0}},{"B",{2530.0,1730.0}}};
    const auto lifted=gf::task24LiftTargets(contract,fronts);
    const auto reference=gf::task20LiftTargets(
        gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder),
        contract.fixed_positions,fronts);
    REQUIRE(lifted.valid);
    REQUIRE(reference.valid);
    for (gf::NodeId id=1;id<=14;++id)
        CHECK((lifted.targets.at(id)-reference.targets.at(id)).norm()<1e-12);
}

TEST_CASE("Task24 Pinball lifting and actual front inverse follow frozen formulas") {
    const auto contract=gf::task24Contract(gf::Task24LatticeMode::Pinball5432);
    const Eigen::Vector2d g{2600.0,1750.0};
    const auto lifted=gf::task24LiftTargets(contract,{{"P",g}});
    REQUIRE(lifted.valid);
    CHECK(lifted.targets.at(1).isApprox(Eigen::Vector2d{537.5,400.0},1e-10));
    CHECK(lifted.targets.at(5).isApprox(Eigen::Vector2d{4137.5,400.0},1e-10));
    CHECK(lifted.targets.at(13).isApprox(Eigen::Vector2d{2150.0,1750.0},1e-10));
    CHECK(lifted.targets.at(14).isApprox(Eigen::Vector2d{3050.0,1750.0},1e-10));
    const auto front=gf::task24ActualFront(contract,lifted.targets,"P");
    REQUIRE(front.has_value());
    CHECK(front->isApprox(Eigen::Vector2d{2600.0,1750.0},1e-10));
}

TEST_CASE("Task24 long triangle lift and front inverse are exact and mirrored") {
    const auto contract=gf::task24Contract(
        gf::Task24LatticeMode::LongTriangleSingleLadder);
    const Eigen::Vector2d g{1000.0,4500.0};
    const auto lifted=gf::task24LiftTargets(contract,{{"T",g}});
    REQUIRE(lifted.valid);
    const auto front=gf::task24ActualFront(contract,lifted.targets,"T");
    REQUIRE(front.has_value());
    CHECK(front->isApprox(g,1e-9));
    const Eigen::Vector2d center=contract.entrance_center;
    CHECK(std::abs((lifted.targets.at(1)-center).x()+
                   (lifted.targets.at(2)-center).x())<1e-9);
    CHECK((contract.fixed_positions.at(100)+contract.fixed_positions.at(101)-
           2.0*center).norm()<1e-9);
}

TEST_CASE("Task24 builds disjoint complete workload-balanced corridors for 1 2 3 units") {
    for (std::size_t count=1;count<=3;++count) {
        std::vector<std::string> units;
        for (std::size_t i=0;i<count;++i) units.push_back("U"+std::to_string(i));
        const auto plan=gf::task24BuildRasterPlan(
            grid(),units,field(),40.0,{});
        CAPTURE(count);
        REQUIRE(plan.valid);
        std::set<std::string> assigned;
        std::size_t minimum=100000,maximum=0;
        for (const auto& corridor:plan.corridors) {
            minimum=std::min(minimum,corridor.workload);
            maximum=std::max(maximum,corridor.workload);
        }
        for (const auto& [id,assignment]:plan.cell_assignments) {
            (void)assignment;
            CHECK(assigned.insert(id).second);
        }
        CHECK(assigned.size()==grid().size());
        CHECK(maximum-minimum<=12);
    }
}

TEST_CASE("Task24 cursor advances by actual-front projection with fixed 400m lookahead") {
    const auto plan=gf::task24BuildRasterPlan(grid(100,12),{"U"},field(),40.0,{});
    REQUIRE(plan.valid);
    gf::Task24RasterState state=gf::task24InitialRasterState(plan,"U");
    const auto first=gf::allocateTask24Raster({plan,grid(100,12),
        {{"U",{105.0,25.0}}},{{"U",state}}});
    REQUIRE(first.valid);
    const auto& assignment=first.assignments.at("U");
    CHECK(assignment.state.pass_cursor_m==doctest::Approx(100.0));
    CHECK(assignment.continuous_front.x()==doctest::Approx(505.0));
    CHECK(assignment.continuous_front.y()==doctest::Approx(25.0));
    CHECK(assignment.task.x_index>=0);
    CHECK(assignment.task.y_index>=0);
}

TEST_CASE("Task24 only reverses at endpoint and rescans incomplete band") {
    const auto cells=grid(12,12);
    const auto plan=gf::task24BuildRasterPlan(cells,{"U"},field(),40.0,{});
    REQUIRE(plan.valid);
    auto state=gf::task24InitialRasterState(plan,"U");
    const double endpoint=plan.corridors.front().cross_max;
    const auto before=gf::allocateTask24Raster({plan,cells,
        {{"U",{endpoint-20.0,25.0}}},{{"U",state}}});
    REQUIRE(before.valid);
    CHECK(before.assignments.at("U").state.direction==1);
    const auto at=gf::allocateTask24Raster({plan,cells,
        {{"U",{endpoint,25.0}}},
        {{"U",before.assignments.at("U").state}}});
    REQUIRE(at.valid);
    CHECK(at.assignments.at("U").state.band_index==0);
    CHECK(at.assignments.at("U").state.direction==-1);
    CHECK(at.assignments.at("U").state.same_band_rescans==1);
}

TEST_CASE("Task24 completed band advances independently and real IDs use one residual rule") {
    const auto cells=grid(24,12);
    const auto plan=gf::task24BuildRasterPlan(cells,{"A","B"},field(),40.0,{});
    REQUIRE(plan.valid);
    std::vector<gf::FrontierCell> uncovered;
    for (std::size_t count=0;count<=3;++count) {
        uncovered.assign(cells.begin(),cells.begin()+count);
        std::map<std::string,gf::Task24RasterState> states;
        std::map<std::string,Eigen::Vector2d> fronts;
        for (const auto& corridor:plan.corridors) {
            states[corridor.coverage_unit]=gf::task24InitialRasterState(
                plan,corridor.coverage_unit);
            fronts[corridor.coverage_unit]={corridor.cross_min,
                plan.pass_progress_m.front()};
        }
        const auto result=gf::allocateTask24Raster(
            {plan,uncovered,fronts,states});
        CAPTURE(count);
        REQUIRE(result.valid);
        CHECK(result.complete==(count==0));
        std::set<std::string> ids;
        for (const auto& [unit,assignment]:result.assignments) {
            (void)unit;
            if (!assignment.active) continue;
            CHECK(assignment.task.id().find('-')==std::string::npos);
            CHECK(ids.insert(assignment.task.id()).second);
        }
    }
}

TEST_CASE("Task24 drained unit retains its last legal task and front") {
    const auto cells=grid(12,12);
    const auto plan=gf::task24BuildRasterPlan(cells,{"U"},field(),40.0,{});
    REQUIRE(plan.valid);
    auto state=gf::task24InitialRasterState(plan,"U");
    const auto assigned=gf::allocateTask24Raster({plan,{cells.front()},
        {{"U",{25.0,25.0}}},{{"U",state}}});
    REQUIRE(assigned.valid);
    REQUIRE(assigned.assignments.at("U").active);
    state=assigned.assignments.at("U").state;
    const Eigen::Vector2d retained_front=
        assigned.assignments.at("U").continuous_front;
    const std::string retained_id=assigned.assignments.at("U").task.id();
    const auto drained=gf::allocateTask24Raster({plan,{},
        {{"U",{115.0,105.0}}},{{"U",state}}});
    REQUIRE(drained.valid);
    CHECK(drained.complete);
    CHECK_FALSE(drained.assignments.at("U").active);
    CHECK(drained.assignments.at("U").task.id()==retained_id);
    CHECK(drained.assignments.at("U").continuous_front.isApprox(
        retained_front,1e-12));
}
