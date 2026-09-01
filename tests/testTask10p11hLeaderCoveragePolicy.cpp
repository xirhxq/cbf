#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11hLeaderCoveragePolicy.hpp"

namespace {

gf::FrontierCell cell(int x,int y,double px,double py) {
    return {x,y,{px,py}};
}

gf::LeaderCoverageRequest request() {
    gf::LeaderCoverageRequest value;
    for (int id=1;id<=14;++id) {
        const double x=id<=7?125.0:375.0;
        value.agents.push_back({static_cast<gf::NodeId>(id),
            {x,50.0},Eigen::Vector2d::Zero()});
    }
    value.domain_cells={cell(0,0,125.0,300.0),
                        cell(1,0,375.0,300.0)};
    return value;
}

gf::LeaderCoverageRequest v5RequestWithLeftCandidates(int candidate_count) {
    auto value=request();
    value.agents.clear();
    for (int id=1;id<=14;++id) {
        const double x=id<=7?0.0:1000.0;
        value.agents.push_back({static_cast<gf::NodeId>(id),
            {x,0.0},Eigen::Vector2d::Zero()});
    }
    const std::vector<gf::FrontierCell> left={
        cell(0,0,10.0,0.0),cell(1,0,20.0,0.0),cell(2,0,30.0,0.0)};
    value.domain_cells=left;
    value.domain_cells.push_back(cell(100,0,1000.0,0.0));
    value.uncovered_cells.assign(left.begin(),left.begin()+candidate_count);
    value.leader_centroid_primary=true;
    value.leader_reachability_filter=true;
    value.leader_reference_disks[7]={
        {Eigen::Vector2d(30.0,0.0),5.0}};
    return value;
}

}

TEST_CASE("Authoritative leader and branch identity is immutable") {
    const auto spec=gf::task10p11hLeaderCoverageSpec();
    REQUIRE(spec.size()==2);
    CHECK(spec[0].leader==7);
    CHECK(spec[0].members==std::vector<gf::NodeId>{1,2,3,4,5,6,7});
    CHECK(spec[0].rotation_rad==doctest::Approx(-M_PI/3.0));
    CHECK(spec[1].leader==14);
    CHECK(spec[1].members==std::vector<gf::NodeId>{8,9,10,11,12,13,14});
    CHECK(spec[1].rotation_rad==doctest::Approx(M_PI/3.0));
    CHECK(spec[0].coverage_origin.isApprox(Eigen::Vector2d(250.0,-50.0)));
    CHECK(spec[1].coverage_origin.isApprox(Eigen::Vector2d(250.0,-50.0)));
}

TEST_CASE("Coverage branch and leader identities enter through one config interface") {
    auto value=request();
    value.uncovered_cells=value.domain_cells;
    value.branches={{{1,2,3,4,5,6,7},1,{250.0,-50.0},-M_PI/3.0,{101,100}},
                    {{8,9,10,11,12,13,14},8,{250.0,-50.0},M_PI/3.0,{101,102}}};
    const auto result=gf::allocateLeaderCoverageTargets(value);
    REQUIRE(result.valid);
    CHECK(result.leader_targets.count(1)==1);
    CHECK(result.leader_targets.count(8)==1);
    CHECK(result.leader_targets.count(7)==0);
    CHECK(result.leader_targets.count(14)==0);
    CHECK(result.targets.size()==14);
}

TEST_CASE("Only leaders partition the complete denominator") {
    auto value=request();
    value.uncovered_cells=value.domain_cells;
    const auto result=gf::allocateLeaderCoverageTargets(value);
    REQUIRE(result.valid);
    CHECK(result.voronoi_owner.size()==value.domain_cells.size());
    CHECK(result.voronoi_owner.at("0:0")==7);
    CHECK(result.voronoi_owner.at("1:0")==14);
    CHECK(result.leader_targets.at(7).id()=="0:0");
    CHECK(result.leader_targets.at(14).id()=="1:0");
    CHECK(result.targets.size()==14);
}

TEST_CASE("Followers use exact four-section alternating CBF2026 targets") {
    const auto result=gf::allocateLeaderCoverageTargets(request());
    REQUIRE(result.valid);
    const Eigen::Vector2d q7(125.0,300.0);
    const Eigen::Vector2d d=(q7-Eigen::Vector2d(250.0,-50.0))/4.0;
    const Eigen::Rotation2Dd rotate(-M_PI/3.0);
    CHECK(result.targets.at(1).center.isApprox(q7-3.0*d));
    CHECK(result.targets.at(2).center.isApprox(q7-3.0*d+rotate*d));
    CHECK(result.targets.at(3).center.isApprox(q7-2.0*d));
    CHECK(result.targets.at(4).center.isApprox(q7-2.0*d+rotate*d));
    CHECK(result.targets.at(5).center.isApprox(q7-d));
    CHECK(result.targets.at(6).center.isApprox(q7-d+rotate*d));
    CHECK(result.targets.at(7).center.isApprox(q7));
}

TEST_CASE("Main leader ledger does not project follower targets to search polygon") {
    auto value=request();
    value.domain_cells={cell(0,0,0.0,0.0),cell(1,0,500.0,600.0)};
    value.uncovered_cells=value.domain_cells;
    const auto result=gf::allocateLeaderCoverageTargets(value);
    REQUIRE(result.valid);
    CHECK(result.leader_targets.at(7).center.isApprox(Eigen::Vector2d(0.0,0.0)));
    CHECK(result.targets.at(1).center.isApprox(Eigen::Vector2d(187.5,-37.5)));
    CHECK(result.targets.at(1).center.y()<0.0);
}

TEST_CASE("Legacy search polygon clipping remains a named negative ablation") {
    auto value=request();
    value.domain_cells={cell(0,0,0.0,0.0),cell(1,0,500.0,600.0)};
    value.uncovered_cells=value.domain_cells;
    const auto unprojected=gf::allocateLeaderCoverageTargets(value);
    REQUIRE(unprojected.valid);
    const auto legacy=gf::legacyProjectLeaderCoverageTargetsToSearchPolygon(
        unprojected,Eigen::Vector2d(0.0,0.0),Eigen::Vector2d(500.0,600.0));
    CHECK(legacy.targets.at(1).center.isApprox(Eigen::Vector2d(187.5,0.0)));
    CHECK(unprojected.targets.at(1).center.isApprox(
        Eigen::Vector2d(187.5,-37.5)));
}

TEST_CASE("Leader without uncovered cells returns its own density centroid") {
    auto value=request();
    value.domain_cells={cell(0,0,100.0,100.0),cell(1,0,150.0,300.0),
                        cell(2,0,350.0,100.0),cell(3,0,400.0,300.0)};
    value.uncovered_cells={cell(0,0,100.0,100.0)};
    const auto result=gf::allocateLeaderCoverageTargets(value);
    REQUIRE(result.valid);
    CHECK(result.leader_targets.at(7).id()=="0:0");
    CHECK(result.leader_targets.at(14).x_index==-14);
    CHECK(result.leader_targets.at(14).center.isApprox(
        Eigen::Vector2d(375.0,200.0)));
    CHECK(result.centroid_fallback_leaders==std::set<gf::NodeId>{14});
}

TEST_CASE("V5 zero-candidate branch returns its region centroid identity") {
    const auto result=gf::allocateLeaderCoverageTargets(
        v5RequestWithLeftCandidates(0));
    REQUIRE(result.valid);
    CHECK(result.leader_targets.at(7).id()=="-7:-1");
    CHECK(result.leader_targets.at(7).center.isApprox(
        Eigen::Vector2d(20.0,0.0)));
}

TEST_CASE("V5 one-candidate branch preserves the real nearest target ID") {
    const auto result=gf::allocateLeaderCoverageTargets(
        v5RequestWithLeftCandidates(1));
    REQUIRE(result.valid);
    CHECK(result.leader_targets.at(7).id()=="0:0");
}

TEST_CASE("V5 two-candidate branch preserves the real nearest target ID") {
    const auto result=gf::allocateLeaderCoverageTargets(
        v5RequestWithLeftCandidates(2));
    REQUIRE(result.valid);
    CHECK(result.leader_targets.at(7).id()=="0:0");
}

TEST_CASE("V5 three-candidate branch retains reachability filtering") {
    const auto result=gf::allocateLeaderCoverageTargets(
        v5RequestWithLeftCandidates(3));
    REQUIRE(result.valid);
    CHECK(result.leader_targets.at(7).id()=="2:0");
    CHECK(result.leader_reachability_fallback_leaders.empty());
}

TEST_CASE("Input order cannot change the target ledger or digest") {
    auto first=request();
    first.uncovered_cells=first.domain_cells;
    auto second=first;
    std::reverse(second.agents.begin(),second.agents.end());
    std::reverse(second.uncovered_cells.begin(),second.uncovered_cells.end());
    std::reverse(second.domain_cells.begin(),second.domain_cells.end());
    const auto a=gf::allocateLeaderCoverageTargets(first);
    const auto b=gf::allocateLeaderCoverageTargets(second);
    REQUIRE(a.valid);
    REQUIRE(b.valid);
    CHECK(a.targets==b.targets);
    CHECK(a.request_digest==b.request_digest);
}
