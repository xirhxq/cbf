#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32UnitFrontLedger.hpp"

TEST_CASE("Certified task status does not introduce a second stop rule in continuous front motion") {
    const auto contract=gf::task20DagLatticeContract(gf::Task20LatticeMode::MergedStrip);
    REQUIRE(contract.valid);
    REQUIRE(contract.coverage_units.size()==1);
    REQUIRE(contract.coverage_units.front().members.size()==14);
    const auto unit=contract.coverage_units.front().id;
    const std::map<gf::NodeId,Eigen::Vector2d> anchors{
        {100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const gf::FrontierCell initial{100,100,{1005,1005}};
    std::map<std::string,gf::Task20CoverageAssignment> assignments{
        {unit,{unit,initial,initial.center}}};
    const auto initialized=gf::task32AdvanceUnitFrontLedger(
        contract,anchors,assignments,{initial.id()},{},true,std::nullopt);
    REQUIRE(initialized.valid);
    const gf::FrontierCell next{200,100,{2005,1005}};
    assignments.at(unit)={unit,next,next.center};
    const auto moving=gf::task32AdvanceUnitFrontLedger(
        contract,anchors,assignments,{next.id()},initialized.units,true,2.99);
    REQUIRE(moving.valid);
    CHECK(moving.units.at(unit).applied_front.x()==doctest::Approx(1007.99));

    // Another aircraft certifies the real task before the next allocation.
    // A different real task remains globally, so this is not a post-T100 call.
    // ID is now historical, but C must still filter the same inherited goal.
    const std::set<std::string> remaining{"300:100"};
    const auto covered_one=gf::task32AdvanceUnitFrontLedger(
        contract,anchors,{},remaining,moving.units,false,2.99);
    REQUIRE(covered_one.valid);
    CHECK_FALSE(covered_one.units.at(unit).active);
    CHECK(covered_one.units.at(unit).task.id()=="200:100");
    CHECK(covered_one.units.at(unit).applied_front.x()==doctest::Approx(1010.98));
    CHECK(covered_one.units.at(unit).applied_front.y()==doctest::Approx(1005.0));
    const auto covered_two=gf::task32AdvanceUnitFrontLedger(
        contract,anchors,{},remaining,covered_one.units,false,2.99);
    REQUIRE(covered_two.valid);
    CHECK_FALSE(covered_two.units.at(unit).active);
    CHECK(covered_two.units.at(unit).task.id()=="200:100");
    CHECK(covered_two.units.at(unit).applied_front.x()==doctest::Approx(1013.97));
    CHECK(covered_two.units.at(unit).applied_front.y()==doctest::Approx(1005.0));
    REQUIRE(covered_two.targets.size()==14);
    for(const auto member:contract.coverage_units.front().members) {
        CHECK(covered_two.targets.at(member).id()=="200:100");
        CHECK(covered_two.targets.at(member).center.allFinite());
    }

    // B's inactive contract is different: no front filter and no new task,
    // therefore preserve the applied reference exactly, without parking IDs.
    const auto inactive_b=gf::task32AdvanceUnitFrontLedger(
        contract,anchors,{},remaining,moving.units,false,std::nullopt);
    REQUIRE(inactive_b.valid);
    CHECK_FALSE(inactive_b.units.at(unit).active);
    CHECK(inactive_b.units.at(unit).task.id()=="200:100");
    CHECK((inactive_b.units.at(unit).applied_front-
           moving.units.at(unit).applied_front).norm()==0.0);
    const auto inactive_b_two=gf::task32AdvanceUnitFrontLedger(
        contract,anchors,{},remaining,inactive_b.units,false,std::nullopt);
    REQUIRE(inactive_b_two.valid);
    CHECK_FALSE(inactive_b_two.units.at(unit).active);
    CHECK(inactive_b_two.units.at(unit).task.id()=="200:100");
    CHECK((inactive_b_two.units.at(unit).applied_front-
           moving.units.at(unit).applied_front).norm()==0.0);
}
