#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31CommonBridge.hpp"
#include "grand_finale/Task25P0MultiDag.hpp"

TEST_CASE("Explicit lattice frame and scale do not drift when physical anchors are added") {
    const std::map<gf::NodeId,Eigen::Vector2d> a3{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    auto a6=a3;a6[103]={2249,-48};a6[104]={8000,3000};a6[105]={-400,700};
    auto h0=gf::task25DagContractFromCode(0),pin=gf::task25DagContractFromCode(12);
    pin.coverage_units[0].frame_origin=Eigen::Vector2d(2250,-50);
    const auto l3=gf::task31TriangularLattice(pin,a3,{0,1},450);
    const auto l6=gf::task31TriangularLattice(pin,a6,{0,1},450);
    REQUIRE(l3.valid);REQUIRE(l6.valid);
    const std::map<std::string,Eigen::Vector2d> front{{"P",{2250,950}}};
    const auto q3=gf::task20LiftTargets(l3.contract,a3,front),q6=gf::task20LiftTargets(l6.contract,a6,front);
    REQUIRE(q3.valid);REQUIRE(q6.valid);
    for(const auto& [id,p]:q3.targets) {
        CHECK((p-q6.targets.at(id)).norm()==0);
        const auto inv=gf::task20FrontForMemberPose(l6.contract,a6,id,p);
        REQUIRE(inv.valid);CHECK((inv.front-front.at("P")).norm()<1e-9);
    }
    auto shifted=pin;shifted.coverage_units[0].frame_origin=Eigen::Vector2d(2300,20);
    const auto q=gf::task20LiftTargets(shifted,a6,front);REQUIRE(q.valid);
    CHECK((q.targets.at(1)-gf::task20LiftTargets(pin,a6,front).targets.at(1)).norm()>1);
    const auto b3=gf::task31CommonBridge(h0,pin,a3,{0,1},150,450,Eigen::Vector2d(2250,-50));
    const auto b6=gf::task31CommonBridge(h0,pin,a6,{0,1},150,450,Eigen::Vector2d(2250,-50));
    REQUIRE(b3.valid);REQUIRE(b6.valid);CHECK(b6.spacing_m==150);
    for(const auto& [id,p]:b3.targets)CHECK((p-b6.targets.at(id)).norm()==0);
    CHECK_FALSE(gf::task31TriangularLattice(pin,a3,{0,1},-1).valid);
    CHECK_FALSE(gf::task31CommonBridge(h0,pin,a3,{0,1},-1,450,Eigen::Vector2d(2250,-50)).valid);
}
