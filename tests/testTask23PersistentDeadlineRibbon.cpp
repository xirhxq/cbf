#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task23PersistentDeadlineRibbon.hpp"

#include <set>

namespace {

std::set<std::set<gf::NodeId>> componentSets(
    const gf::Task23ContractAudit& audit) {
    std::set<std::set<gf::NodeId>> result;
    for (const auto& component:audit.mobile_components)
        result.emplace(component.begin(),component.end());
    return result;
}

gf::Task23DeadlinePlan corePlan(std::size_t count,double first_deadline=100.0) {
    gf::Task23DeadlinePlan plan;
    plan.valid=true;
    plan.route_geometry.valid=true;
    plan.route_geometry.field=gf::task21AffineCoordinateField(
        {0.0,0.0},{0.0,1.0},{1.0,0.0});
    plan.route_validity_lateral_m=50.0;
    plan.route_validity_yaw_rad=0.5;
    gf::Task22UnitRoute route;
    route.coverage_unit="U";
    for (std::size_t index=0;index<=10;++index)
        route.samples.push_back({100.0*index,{100.0*index,0.0},
            {1.0,0.0},0,false});
    route.total_length=1000.0;
    plan.route_geometry.routes["U"]=route;
    for (std::size_t index=0;index<count;++index) {
        gf::Task23CellWitness witness;
        witness.cell_id=std::to_string(index)+":0";
        witness.coverage_unit="U";
        witness.canonical_s=first_deadline+100.0*index;
        witness.canonical_front={witness.canonical_s,0.0};
        witness.route_tangent={1.0,0.0};
        witness.sensing_margin_m=50.0;
        witness.yaw_margin_rad=0.5;
        plan.queues["U"].push_back(witness);
    }
    return plan;
}

std::vector<gf::FrontierCell> remaining(std::size_t begin,std::size_t end) {
    std::vector<gf::FrontierCell> cells;
    for (std::size_t index=begin;index<end;++index)
        cells.push_back({static_cast<int>(index),0,{10.0*index,5.0}});
    return cells;
}

std::vector<gf::FrontierCell> grid(int width,int height) {
    std::vector<gf::FrontierCell> cells;
    for (int x=0;x<width;++x) for (int y=0;y<height;++y)
        cells.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    return cells;
}

std::map<gf::NodeId,Eigen::Vector2d> fixedAnchors() {
    return {{100,{-50.0,75.0}},{101,{150.0,25.0}},{102,{350.0,125.0}}};
}

std::map<std::string,Eigen::Vector2d> initialFronts(
    const gf::Task20DagLatticeContract& contract) {
    std::map<std::string,Eigen::Vector2d> result;
    for (std::size_t index=0;index<contract.coverage_units.size();++index)
        result[contract.coverage_units[index].id]={
            75.0+150.0*index,15.0};
    return result;
}

}  // namespace

TEST_CASE("Task23 coverage units are exactly mobile-reference components") {
    const auto one=gf::task23AuditCoverageContract(
        gf::task20DagLatticeContract(gf::Task20LatticeMode::MergedStrip));
    const auto two=gf::task23AuditCoverageContract(
        gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder));
    const auto three=gf::task23AuditCoverageContract(
        gf::task20DagLatticeContract(
            gf::Task20LatticeMode::SplitThreeFront));

    CHECK(one.valid);
    CHECK(two.valid);
    CHECK(three.valid);
    CHECK(one.mobile_components.size()==1);
    CHECK(two.mobile_components.size()==2);
    CHECK(three.mobile_components.size()==3);
}

TEST_CASE("Task23 rejects a declared unit split across a mobile edge") {
    auto contract=
        gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
    contract.coverage_units.clear();
    for (gf::NodeId member=1;member<=14;++member)
        contract.coverage_units.push_back({std::to_string(member),{member},
            {101},member,{member}});

    const auto audit=gf::task23AuditCoverageContract(contract);
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason=="declared_units_do_not_match_mobile_connectivity");
}

TEST_CASE("Task23 pinball contract is the frozen 5-4-3-2 DAG") {
    const auto contract=gf::task23Pinball5432Contract();
    const auto audit=gf::task23AuditCoverageContract(contract);
    CHECK(contract.valid);
    CHECK(audit.valid);
    REQUIRE(contract.coverage_units.size()==1);
    CHECK(contract.coverage_units.front().front_members==
        std::vector<gf::NodeId>{13,14});
    CHECK(componentSets(audit)==std::set<std::set<gf::NodeId>>{
        {1,2,3,4,5,6,7,8,9,10,11,12,13,14}});
    CHECK(contract.reference_edges==std::vector<gf::DirectedEdge>{
        {100,1},{101,1},{100,2},{101,2},{101,3},{102,3},
        {101,4},{102,4},{100,5},{102,5},
        {1,6},{2,6},{2,7},{3,7},{3,8},{4,8},{4,9},{5,9},
        {6,10},{7,10},{7,11},{8,11},{8,12},{9,12},
        {10,13},{11,13},{11,14},{12,14}});
}

TEST_CASE("Task23 PDR core uses one rule for zero through three residuals") {
    for (std::size_t count=0;count<=3;++count) {
        const auto plan=corePlan(count);
        gf::Task23CoreRequest request;
        request.plan=&plan;
        request.certified_uncovered=remaining(0,count);
        request.actual_fronts["U"]={0.0,0.0};
        request.actual_front_yaws["U"]=0.0;
        const auto result=gf::allocateTask23PdrCore(request);
        CHECK(result.valid);
        CHECK(result.complete==(count==0));
        if (count>0) {
            REQUIRE(result.assignments.at("U").active);
            CHECK(result.assignments.at("U").task.id()=="0:0");
            CHECK(result.assignments.at("U").deadline_s==
                doctest::Approx(100.0));
        }
    }
}

TEST_CASE("Task23 offline plan retains a straight-pass canonical witness") {
    const auto contract=
        gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
    const auto cells=grid(30,30);
    std::set<std::string> initial;
    for (const auto& cell:cells) initial.insert(cell.id());
    initial.erase("15:15");
    const auto plan=gf::task23BuildDeadlinePlan(cells,
        gf::task21AffineCoordinateField({0.0,0.0},{0.0,1.0},{1.0,0.0}),
        100.0,64,contract,fixedAnchors(),initialFronts(contract),initial);
    CAPTURE(plan.reason);
    CAPTURE(plan.no_witness_cells);
    REQUIRE(plan.valid);
    CHECK(plan.deadline_rule==
        "earliest_feasible_pass_compatible_interval");
    CHECK(plan.route_geometry.reason==
        "task23_canonical_witness_envelope_route");
    REQUIRE(plan.witnesses.count("15:15")==1);
    const auto& witness=plan.witnesses.at("15:15");
    CHECK(witness.cell_id=="15:15");
    CHECK(witness.canonical_s>=witness.first_service_s);
    CHECK(witness.canonical_s<=witness.last_service_s);
    CHECK(witness.sensing_margin_m>=0.0);
    CHECK(witness.pass_index==0);
    const auto& route=plan.route_geometry.routes.at(witness.coverage_unit);
    const auto sample=gf::task23_detail::sampleAtS(route,witness.canonical_s);
    CHECK_FALSE(route.samples[sample].on_fillet);
}

TEST_CASE("Task23 route endpoints are canonical-witness envelopes") {
    const auto contract=
        gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
    const auto cells=grid(30,30);
    const auto plan=gf::task23BuildDeadlinePlan(cells,
        gf::task21AffineCoordinateField({0.0,0.0},{0.0,1.0},{1.0,0.0}),
        100.0,64,contract,fixedAnchors(),initialFronts(contract));
    CAPTURE(plan.reason);
    REQUIRE(plan.valid);
    for (const auto& unit:contract.coverage_units) {
        const auto& route=plan.route_geometry.routes.at(unit.id);
        for (std::size_t pass=0;pass<route.passes.size();++pass) {
            double low=std::numeric_limits<double>::infinity();
            double high=-std::numeric_limits<double>::infinity();
            for (const auto& [cell,witness]:plan.witnesses) {
                (void)cell;
                if (witness.coverage_unit!=unit.id||
                    witness.pass_index!=pass) continue;
                const double cross=plan.route_geometry.field.coordinates(
                    witness.canonical_front).y();
                low=std::min(low,cross);
                high=std::max(high,cross);
            }
            if (!std::isfinite(low)) continue;
            CHECK(route.passes[pass].inset_low==doctest::Approx(low));
            CHECK(route.passes[pass].inset_high==doctest::Approx(high));
            if (pass+1<route.passes.size())
                CHECK(route.passes[pass].cross_end==
                    doctest::Approx(route.passes[pass+1].cross_begin));
        }
    }
}

TEST_CASE("Task23 queue skips covered obligations without losing real IDs") {
    const auto plan=corePlan(3);
    gf::Task23CoreRequest request;
    request.plan=&plan;
    request.certified_uncovered=remaining(1,3);
    request.actual_fronts["U"]={0.0,0.0};
    request.actual_front_yaws["U"]=0.0;
    const auto result=gf::allocateTask23PdrCore(request);
    CHECK(result.valid);
    CHECK(result.covered_deadlines_skipped==1);
    CHECK(result.assignments.at("U").task.id()=="1:0");
    CHECK(result.states.at("U").deadline_queue_index==1);
}

TEST_CASE("Task23 shared front is bounded, monotone, and holds off route") {
    const auto plan=corePlan(1,600.0);
    gf::Task23CoreRequest request;
    request.plan=&plan;
    request.certified_uncovered=remaining(0,1);
    request.actual_fronts["U"]={0.0,0.0};
    request.actual_front_yaws["U"]=0.0;
    const auto first=gf::allocateTask23PdrCore(request);
    CHECK(first.assignments.at("U").target_s==doctest::Approx(400.0));
    CHECK(first.states.at("U").cursor_s==doctest::Approx(0.0));

    request.states=first.states;
    request.actual_fronts["U"]={200.0,100.0};
    const auto off_route=gf::allocateTask23PdrCore(request);
    CHECK_FALSE(off_route.assignments.at("U").route_valid);
    CHECK(off_route.assignments.at("U").holding);
    CHECK(off_route.states.at("U").cursor_s==doctest::Approx(0.0));
    CHECK(off_route.assignments.at("U").target_s==doctest::Approx(400.0));

    request.states=off_route.states;
    request.actual_fronts["U"]={200.0,0.0};
    const auto resumed=gf::allocateTask23PdrCore(request);
    CHECK(resumed.assignments.at("U").route_valid);
    CHECK(resumed.states.at("U").cursor_s==doctest::Approx(200.0));
    CHECK(resumed.assignments.at("U").target_s==doctest::Approx(600.0));
}

TEST_CASE("Task23 drained unit retains its last real ledger") {
    const auto plan=corePlan(1);
    gf::Task23CoreRequest request;
    request.plan=&plan;
    request.certified_uncovered=remaining(0,1);
    request.actual_fronts["U"]={0.0,0.0};
    request.actual_front_yaws["U"]=0.0;
    const auto active=gf::allocateTask23PdrCore(request);
    request.states=active.states;
    request.certified_uncovered={{9,9,{95.0,95.0}}};
    const auto drained=gf::allocateTask23PdrCore(request);
    CHECK(drained.valid);
    CHECK_FALSE(drained.assignments.at("U").active);
    CHECK(drained.assignments.at("U").task.id()=="0:0");
    CHECK(drained.assignments.at("U").shared_front_target==
        active.assignments.at("U").shared_front_target);
}
