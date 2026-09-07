#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31CommonBridge.hpp"
#include "grand_finale/Task25P0MultiDag.hpp"

TEST_CASE("Task31 bridge is a generic union-geometry asset, not an executable union graph") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto bridge=gf::task31CommonBridge(old,goal,fixed,{0,1});
    REQUIRE(bridge.valid);CHECK(bridge.geometry_edges.size()==51);CHECK(bridge.targets.size()==14);
    CHECK(bridge.lattice.contract.coverage_units.size()==1);
    CHECK(bridge.spacing_m==doctest::Approx(150));
    CHECK(bridge.lattice.cells.at(1).row==1);CHECK(bridge.lattice.cells.at(14).row==12);
    CHECK(bridge.targets.at(1).x()==doctest::Approx(2250));
    CHECK(bridge.targets.at(1).y()==doctest::Approx(79.9038105676658));
    CHECK_FALSE(bridge.targets.count(100));CHECK_FALSE(bridge.targets.count(101));CHECK_FALSE(bridge.targets.count(102));
    CHECK(gf::task31CommonBridge(old,goal,fixed,{0,0}).valid==false);
    for(auto code:{0,2,12,13}) {
        const auto c=gf::task25DagContractFromCode(code);
        const auto b=gf::task31CommonBridge(c,c,fixed,{0,1});REQUIRE(b.valid);
        CHECK(b.lattice.contract.coverage_units.size()==c.coverage_units.size());
        for(const auto& [id,p]:b.targets)CHECK(p.allFinite());
    }
    nlohmann::json output={{"positions",nlohmann::json::object()},{"cells",nlohmann::json::object()}};
    for(const auto& [id,p]:bridge.targets){output["positions"][std::to_string(id)]={p.x(),p.y()};const auto c=bridge.lattice.cells.at(id);output["cells"][std::to_string(id)]={c.row,c.slot};}
    std::cout<<"TASK31_COMMON_BRIDGE "<<output.dump()<<'\n';
}

TEST_CASE("Task31 common bridge keeps identities under relabelling and rigid frame changes") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto original=gf::task31CommonBridge(old,goal,fixed,{0,1});REQUIRE(original.valid);
    std::map<gf::NodeId,gf::NodeId> ids;
    for(const auto& [id,r]:goal.member_roles)ids[id]=600-7*id;
    for(const auto& [id,p]:fixed)ids[id]=1200-id;
    auto relabel=[&](auto c) {
        for(auto& e:c.reference_edges){e.reference=ids.at(e.reference);e.owner=ids.at(e.owner);}
        const auto roles=c.member_roles;c.member_roles.clear();
        for(auto [id,r]:roles){r.member=ids.at(id);c.member_roles[r.member]=r;}
        for(auto& u:c.coverage_units){for(auto& id:u.members)id=ids.at(id);for(auto& id:u.base_anchors)id=ids.at(id);for(auto& id:u.front_members)id=ids.at(id);u.leader=ids.at(u.leader);}
        return c;
    };
    const Eigen::Rotation2Dd R(.41);const Eigen::Vector2d shift(-823,276);
    std::map<gf::NodeId,Eigen::Vector2d> anchors;
    for(const auto& [id,p]:fixed)anchors[ids.at(id)]=R*p+shift;
    const auto transformed=gf::task31CommonBridge(relabel(old),relabel(goal),anchors,R*Eigen::Vector2d(0,1));
    REQUIRE(transformed.valid);CHECK(transformed.spacing_m==doctest::Approx(original.spacing_m));
    for(const auto& [id,p]:original.targets)CHECK((transformed.targets.at(ids.at(id))-(R*p+shift)).norm()<1e-8);
    auto cycle=goal;cycle.reference_edges.push_back({13,1});
    CHECK_FALSE(gf::task31CommonBridge(old,cycle,fixed,{0,1}).valid);
}
