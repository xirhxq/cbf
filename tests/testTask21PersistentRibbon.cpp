#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task21PersistentRibbon.hpp"

namespace {

std::vector<gf::FrontierCell> grid(int x_count=12,int y_count=9) {
    std::vector<gf::FrontierCell> result;
    for (int x=0;x<x_count;++x) for (int y=0;y<y_count;++y)
        result.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    return result;
}

gf::Task21CoordinateField field() {
    return gf::task21AffineCoordinateField({0.0,0.0},{0.0,1.0},{1.0,0.0});
}

}  // namespace

TEST_CASE("Task 21 builds complete contiguous corridors for one two and three units") {
    for (std::size_t unit_count=1;unit_count<=3;++unit_count) {
        std::vector<std::string> units;
        for (std::size_t index=0;index<unit_count;++index)
            units.push_back("U"+std::to_string(index));
        const auto plan=gf::task21BuildRibbonPlan(grid(),units,field(),30.0,4);
        CAPTURE(unit_count);
        REQUIRE(plan.valid);
        REQUIRE(plan.cells.size()==108);
        std::set<std::string> ids;
        for (const auto& [unit,route]:plan.routes) {
            const std::string captured_unit=unit;
            CAPTURE(captured_unit);
            REQUIRE(!route.cell_ids.empty());
            for (const auto& id:route.cell_ids) CHECK(ids.insert(id).second);
        }
        CHECK(ids.size()==108);
        for (std::size_t first=0;first<plan.corridors.size();++first)
            for (std::size_t second=first+1;second<plan.corridors.size();++second)
                CHECK((plan.corridors[first].cross_max<=plan.corridors[second].cross_min||
                       plan.corridors[second].cross_max<=plan.corridors[first].cross_min));
    }
}

TEST_CASE("Task 21 route is coordinate-field equivariant and turns only at band endpoints") {
    const auto original=gf::task21BuildRibbonPlan(grid(),{"U"},field(),30.0,4);
    const auto rotated_field=gf::task21AffineCoordinateField(
        {0.0,0.0},{1.0,0.0},{0.0,1.0});
    std::vector<gf::FrontierCell> rotated;
    for (const auto& cell:grid())
        rotated.push_back({cell.y_index,cell.x_index,{cell.center.y(),cell.center.x()}});
    const auto rotated_plan=gf::task21BuildRibbonPlan(
        rotated,{"U"},rotated_field,30.0,4);
    REQUIRE(original.valid);
    REQUIRE(rotated_plan.valid);
    REQUIRE(original.routes.at("U").segments.size()==rotated_plan.routes.at("U").segments.size());
    for (const auto& segment:original.routes.at("U").segments)
        CHECK(segment.direction==((segment.band_index%2)==0?1:-1));
}

TEST_CASE("Task 21 starts each corridor from the endpoint nearest its initial front") {
    const std::map<std::string,Eigen::Vector2d> fronts{
        {"A",{105.0,15.0}},{"B",{125.0,15.0}}};
    const auto plan=gf::task21BuildRibbonPlan(
        grid(24,9),{"A","B"},field(),30.0,32,fronts);
    REQUIRE(plan.valid);
    REQUIRE(!plan.routes.at("A").segments.empty());
    REQUIRE(!plan.routes.at("B").segments.empty());
    CHECK(plan.routes.at("A").segments.front().direction==-1);
    CHECK(plan.routes.at("B").segments.front().direction==1);
    for (const auto& [unit,route]:plan.routes) {
        const int first=route.segments.front().direction;
        for (const auto& segment:route.segments)
            CHECK(segment.direction==(segment.band_index%2==0?first:-first));
    }
}

TEST_CASE("Task 21 allocator advances monotone cursors and uses one real-ID rule for zero through three residuals") {
    const auto plan=gf::task21BuildRibbonPlan(grid(),{"U"},field(),30.0,6);
    REQUIRE(plan.valid);
    std::size_t previous_cursor=0;
    for (std::size_t count=0;count<=3;++count) {
        gf::Task21AllocationRequest request;
        request.plan=plan;
        request.uncovered_cells.assign(plan.cells.begin(),plan.cells.begin()+count);
        request.front_positions={{"U",{5.0,5.0}}};
        request.cursors={{"U",previous_cursor}};
        const auto result=gf::allocateTask21Ribbon(request);
        CAPTURE(count);
        REQUIRE(result.valid);
        CHECK(result.complete==(count==0));
        CHECK(result.cursors.at("U")>=previous_cursor);
        previous_cursor=result.cursors.at("U");
        if (count>0) {
            REQUIRE(result.assignments.count("U")==1);
            const auto& target=result.assignments.at("U").task;
            CHECK(target.x_index>=0);
            CHECK(target.y_index>=0);
            CHECK(target.id().find('-')==std::string::npos);
        }
    }
}

TEST_CASE("Task 21 independent unit cursors do not impose a global band barrier") {
    const auto plan=gf::task21BuildRibbonPlan(grid(),{"A","B"},field(),30.0,8);
    REQUIRE(plan.valid);
    gf::Task21AllocationRequest request;
    request.plan=plan;
    request.front_positions={{"A",{5.0,5.0}},{"B",{115.0,5.0}}};
    request.cursors={{"A",0},{"B",0}};
    const auto& a_route=plan.routes.at("A").cell_ids;
    const auto& b_route=plan.routes.at("B").cell_ids;
    const auto a_cell=plan.cells.at(plan.cell_lookup.at(a_route.front()));
    const auto b_cell=plan.cells.at(plan.cell_lookup.at(b_route.back()));
    request.uncovered_cells={a_cell,b_cell};
    const auto result=gf::allocateTask21Ribbon(request);
    REQUIRE(result.valid);
    CHECK(result.assignments.at("A").task.id()==a_cell.id());
    CHECK(result.assignments.at("B").task.id()==b_cell.id());
    CHECK(result.cursors.at("B")>result.cursors.at("A"));
}

TEST_CASE("Task 21 holds a segment endpoint before the only planned serpentine turn") {
    const auto plan=gf::task21BuildRibbonPlan(grid(),{"U"},field(),30.0,16,
        {{"U",{5.0,5.0}}});
    REQUIRE(plan.valid);
    REQUIRE(plan.routes.at("U").segments.size()>=2);
    const auto& route=plan.routes.at("U");
    const auto& first=route.segments.front();
    const auto next_id=route.cell_ids.at(first.route_end);
    const auto next=plan.cells.at(plan.cell_lookup.at(next_id));
    gf::Task21AllocationRequest request;
    request.plan=plan;
    request.uncovered_cells={next};
    request.front_positions={{"U",{25.0,first.route_progress}}};
    request.cursors={{"U",first.route_end}};
    request.active_segments={{"U",0}};
    const auto held=gf::allocateTask21Ribbon(request);
    REQUIRE(held.valid);
    REQUIRE(held.assignments.count("U")==1);
    CHECK(held.assignments.at("U").task.id()==next_id);
    CHECK(held.assignments.at("U").transition_hold);
    CHECK(held.active_segments.at("U")==0);
    request.front_positions["U"]=held.assignments.at("U").front;
    request.active_segments=held.active_segments;
    const auto advanced=gf::allocateTask21Ribbon(request);
    REQUIRE(advanced.valid);
    CHECK(!advanced.assignments.at("U").transition_hold);
    CHECK(advanced.active_segments.at("U")==1);
}

TEST_CASE("Task 21 pinball contract is one wide front and has two references per mobile") {
    const auto contract=gf::task21PinballContract();
    REQUIRE(contract.valid);
    REQUIRE(contract.coverage_units.size()==1);
    CHECK(contract.coverage_units.front().members.size()==14);
    CHECK(contract.coverage_units.front().front_members==std::vector<gf::NodeId>{13,14});
    std::map<gf::NodeId,int> indegree;
    for (const auto& edge:contract.reference_edges) ++indegree[edge.owner];
    for (gf::NodeId owner=1;owner<=14;++owner) CHECK(indegree[owner]==2);
}
