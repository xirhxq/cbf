#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task20CoveragePolicy.hpp"
#include "grand_finale/Task32UnitFrontLedger.hpp"
#include "grand_finale/Task25P0MultiDag.hpp"

static gf::Task20CoverageRequest request() {
    gf::Task20CoverageRequest r;
    r.contract=gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
    r.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    for(gf::NodeId i=1;i<=14;++i)r.agents.push_back({i,{i<=7?10.:3600.,1400.},{12.,i%2?3.:-3.},0.,.3});
    r.uncovered_cells={{432,224,{4325,2245}},{433,223,{4335,2235}}};
    return r;
}

TEST_CASE("Empty observed responsibility remains inactive without cross-pool assignment") {
    auto r=request();const auto legacy=gf::allocateTask20Coverage(r);
    REQUIRE(legacy.valid);REQUIRE(legacy.assignments.size()==2);
    r.config.empty_share_inactive=true;
    const auto candidate=gf::allocateTask20Coverage(r);
    REQUIRE(candidate.valid);CHECK(candidate.assignments.size()==1);
    CHECK(candidate.assignments.count("A")==0);CHECK(candidate.assignments.count("B")==1);
    CHECK(candidate.inactive_units.count("A")==1);
    CHECK(candidate.targets.size()==7);
    // No target for A is a request to preserve its existing historical ledger,
    // never a synthetic task or an instruction to stop sensing/control.
    for(const auto& [id,target]:candidate.targets)CHECK(id>=8);
}

TEST_CASE("Inactive unit inherits real historical ledger and exact shared front") {
    const auto r=request();std::map<std::string,gf::Task20CoverageAssignment> both;
    const gf::FrontierCell left{11,179,{115,1795}},right{432,224,{4325,2245}};
    both["A"]={"A",left,left.center};both["B"]={"B",right,right.center};
    const std::set<std::string> pool{left.id(),right.id()};
    auto first=gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,both,pool,{},true,std::nullopt);
    REQUIRE(first.valid);CHECK(first.targets.size()==14);CHECK(first.units.at("A").active);
    both.erase("A");
    const auto next=gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,both,{right.id()},first.units,true,std::nullopt);
    REQUIRE(next.valid);CHECK_FALSE(next.units.at("A").active);
    CHECK(next.units.at("A").task.id()==left.id());
    CHECK((next.units.at("A").applied_front-left.center).norm()==0);
    for(int i=1;i<=7;++i) {
        CHECK(next.targets.at(i)==first.targets.at(i));
        CHECK((next.targets.at(i).center-first.targets.at(i).center).norm()==0);
    }
    // Missing an initialized historical front is rejected, not borrowed from B.
    CHECK_FALSE(gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,both,{right.id()},{},true,std::nullopt).valid);
}

TEST_CASE("One two three units use the same zero one two three residual ledger rule") {
    for(const auto mode:{gf::Task20LatticeMode::MergedStrip,gf::Task20LatticeMode::DualLadder,gf::Task20LatticeMode::SplitThreeFront}) {
        auto r=request();r.contract=gf::task20DagLatticeContract(mode);REQUIRE(r.contract.valid);
        std::map<std::string,gf::Task20CoverageAssignment> all;
        std::set<std::string> initial_pool;int n=0;
        for(const auto& u:r.contract.coverage_units) {
            gf::FrontierCell c{100+n,100,{1005.+10*n,1005}};all[u.id]={u.id,c,c.center};initial_pool.insert(c.id());++n;
        }
        const auto first=gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,all,initial_pool,{},true,std::nullopt);
        REQUIRE(first.valid);REQUIRE(first.targets.size()==14);
        for(int remaining=0;remaining<=3;++remaining) {
            std::map<std::string,gf::Task20CoverageAssignment> a;std::set<std::string> pool;
            for(int k=0;k<remaining;++k)pool.insert(std::to_string(200+k)+":100");
            int k=0;for(const auto& u:r.contract.coverage_units) {
                if(k<remaining) {gf::FrontierCell c{200+k,100,{2005.+10*k,1005}};a[u.id]={u.id,c,c.center};}++k;
            }
            const auto x=gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,a,pool,first.units,true,std::nullopt);
            REQUIRE(x.valid);CHECK(x.targets.size()==14);
            std::size_t active=0;for(const auto& [id,s]:x.units) {
                active+=s.active;CHECK(s.task.x_index>=0);CHECK(s.task.y_index>=0);
                if(s.active)CHECK(pool.count(s.task.id())==1);
                else CHECK((s.applied_front-first.units.at(id).applied_front).norm()==0);
            }
            CHECK(active==std::min(static_cast<std::size_t>(remaining),r.contract.coverage_units.size()));
        }
        auto bad=all;bad.begin()->second.task.x_index=-1;
        CHECK_FALSE(gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,bad,initial_pool,first.units,true,std::nullopt).valid);
        CHECK_FALSE(gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,all,{},first.units,true,std::nullopt).valid);
        if(all.size()>1) {
            auto duplicate=all;auto one=duplicate.begin(),two=std::next(one);two->second.task=one->second.task;two->second.front=one->second.front;
            CHECK_FALSE(gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,duplicate,initial_pool,first.units,true,std::nullopt).valid);
        }
    }
}

TEST_CASE("Task ID can change immediately while the shared front advances continuously") {
    auto r=request();std::map<std::string,gf::Task20CoverageAssignment> a;
    const gf::FrontierCell left{11,179,{115,1795}},right{432,224,{4325,2245}};
    a["A"]={"A",left,left.center};a["B"]={"B",right,right.center};
    const auto first=gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,a,{left.id(),right.id()},{},true,std::nullopt);REQUIRE(first.valid);
    const gf::FrontierCell next{431,224,{4315,2245}};a["A"]={"A",next,next.center};
    const auto x=gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,a,{next.id(),right.id()},first.units,true,2.99);
    REQUIRE(x.valid);CHECK(x.units.at("A").task.id()==next.id());
    CHECK((x.units.at("A").applied_front-left.center).norm()==doctest::Approx(2.99));
    const auto y=gf::task32AdvanceUnitFrontLedger(r.contract,r.fixed_positions,{}, {next.id(),right.id()},x.units,false,2.99);
    REQUIRE(y.valid);CHECK(y.units.at("A").active);
    CHECK((y.units.at("A").applied_front-x.units.at("A").applied_front).norm()==doctest::Approx(2.99));
    std::map<std::string,Eigen::Vector2d> fronts;for(const auto& [id,s]:y.units)fronts[id]=s.applied_front;
    const auto expected=gf::task20LiftTargets(r.contract,r.fixed_positions,fronts);REQUIRE(expected.valid);
    for(const auto& [id,p]:expected.targets)CHECK((y.targets.at(id).center-p).norm()==0);
}
