#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32ReconstructionFrontResume.hpp"

TEST_CASE("One two and three units resume the last external reference without a target jump") {
    const std::map<gf::NodeId,Eigen::Vector2d> anchors{
        {100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    for(auto mode:{gf::Task20LatticeMode::MergedStrip,gf::Task20LatticeMode::DualLadder,
                   gf::Task20LatticeMode::SplitThreeFront}) {
        const auto c=gf::task20DagLatticeContract(mode);REQUIRE(c.valid);
        std::map<std::string,Eigen::Vector2d> fronts;
        std::map<std::string,gf::FrontierCell> historical;
        int n=0;
        for(const auto& u:c.coverage_units) {
            fronts[u.id]={2000.+100*n,1200.};
            historical[u.id]={20+n,30,{205.+10*n,305.}};++n;
        }
        const auto lifted=gf::task20LiftTargets(c,anchors,fronts);REQUIRE(lifted.valid);
        const auto resumed=gf::task32ResumeFrontLedger(c,anchors,lifted.targets,historical);
        REQUIRE(resumed.valid);CHECK(resumed.targets.size()==14);
        for(const auto& u:c.coverage_units) {
            CHECK((resumed.units.at(u.id).applied_front-fronts.at(u.id)).norm()<1e-9);
            CHECK_FALSE(resumed.units.at(u.id).active);
            CHECK(resumed.units.at(u.id).task.id()==historical.at(u.id).id());
            for(auto id:u.members)CHECK((resumed.targets.at(id).center-lifted.targets.at(id)).norm()<1e-9);
        }
        // First fresh P0 assignment can be far away: C takes only one frozen step.
        std::map<std::string,gf::Task20CoverageAssignment> assignments;
        std::set<std::string> uncovered;
        for(const auto& u:c.coverage_units) {
            auto task=historical.at(u.id);task.center=fronts.at(u.id)+Eigen::Vector2d(1000,0);
            assignments[u.id]={u.id,task,task.center};uncovered.insert(task.id());
        }
        const auto next=gf::task32AdvanceUnitFrontLedger(c,anchors,assignments,uncovered,resumed.units,true,2.99);
        REQUIRE(next.valid);
        for(const auto& u:c.coverage_units)
            CHECK((next.units.at(u.id).applied_front-fronts.at(u.id)).norm()==doctest::Approx(2.99));
        auto bad=lifted.targets;bad.begin()->second.x()+=1.;
        CHECK_FALSE(gf::task32ResumeFrontLedger(c,anchors,bad,historical).valid);
        bad=lifted.targets;bad.erase(bad.begin());
        CHECK_FALSE(gf::task32ResumeFrontLedger(c,anchors,bad,historical).valid);
        auto invalid=historical;invalid.begin()->second.x_index=-1;
        CHECK_FALSE(gf::task32ResumeFrontLedger(c,anchors,lifted.targets,invalid).valid);
    }
}
