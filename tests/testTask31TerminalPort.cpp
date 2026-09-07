#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31AnchorScene.hpp"
#include <fstream>
#include "grand_finale/Task29RoleCenterPath.hpp"

TEST_CASE("explicit terminal port binds a real task using the whole lattice") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");
    REQUIRE(f.good());nlohmann::json asset;f>>asset;
    const auto original=gf::task31AnchorScene(asset);REQUIRE(original.valid);
    asset["front_port_binding"]="positive_cross_terminal";
    const auto bound=gf::task31AnchorScene(asset);REQUIRE(bound.valid);
    const Eigen::Vector2d task(5,2245);
    const auto targets=gf::task20LiftTargets(bound.goal,bound.fixed,{{"P",task}});
    REQUIRE(targets.valid);
    // Frozen independent row/slot export: positive terminal slot is13.
    CHECK((targets.targets.at(13)-task).norm()<1e-8);
    CHECK(bound.fixed==original.fixed);
    CHECK(bound.goal.reference_edges==original.goal.reference_edges);
    const auto baseline=gf::task20LiftTargets(original.goal,original.fixed,{{"P",task}});
    REQUIRE(baseline.valid);
    const double scale=std::sqrt(48./49.);
    for(const auto& [i,p]:targets.targets)for(const auto& [j,q]:targets.targets)
        CHECK((p-q).norm()==doctest::Approx((baseline.targets.at(i)-baseline.targets.at(j)).norm()*scale).epsilon(1e-10));
    nlohmann::json points=nlohmann::json::object(),roles=nlohmann::json::object();
    for(const auto& [id,p]:targets.targets)points[std::to_string(id)]={p.x(),p.y()};
    for(const auto& [id,r]:bound.goal.member_roles)roles[std::to_string(id)]={r.axial_fraction,r.triangular_fraction};
    std::cout<<"TASK31_PORT_SCENE "<<nlohmann::json({{"task",{task.x(),task.y()}},
        {"ports",bound.terminal_ports},{"targets",points},{"roles",roles}}).dump()<<'\n';
}

TEST_CASE("disabled mapping is exact and canonical continuation is unchanged") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");
    REQUIRE(f.good());nlohmann::json asset;f>>asset;
    const auto original=gf::task31AnchorScene(asset);REQUIRE(original.valid);
    asset["front_port_binding"]="none";
    const auto disabled=gf::task31AnchorScene(asset);REQUIRE(disabled.valid);
    const std::map<std::string,Eigen::Vector2d> front{{"P",{2362.5,950}}};
    const auto q0=gf::task20LiftTargets(original.goal,original.fixed,front);
    const auto qd=gf::task20LiftTargets(disabled.goal,disabled.fixed,front);
    REQUIRE(q0.valid);REQUIRE(qd.valid);CHECK(q0.targets==qd.targets);
    CHECK(original.goal.structural_signature==disabled.goal.structural_signature);
    auto from=q0.targets;for(auto& [id,p]:from)p-=Eigen::Vector2d(120,300);
    const auto rates=gf::task31UnscaledContinuationRates(original,from);
    asset["front_port_binding"]="positive_cross_terminal";
    const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
    CHECK(scene.terminal_ports.at("P")==13);
    CHECK(gf::task31UnscaledContinuationRates(scene,from)==rates);
    const auto to=gf::task20LiftTargets(scene.goal,scene.fixed,front);REQUIRE(to.valid);
    const gf::Task29RoleCenterPath path(scene.goal,from,to.targets);
    CHECK(path.evaluateContinuingFrontWithRates(0,rates)==from);
    const auto end=path.evaluateContinuingFrontWithRates(1,rates);
    const auto after=path.evaluateContinuingFrontWithRates(1+1e-8,rates);
    for(const auto& [id,p]:to.targets) {
        CHECK((end.at(id)-p).norm()<1e-9);
        CHECK((after.at(id)-end.at(id)).norm()<1e-4);
    }
    asset["front_similarity_gain"]=.7;
    CHECK_FALSE(gf::task31AnchorScene(asset).valid);
    asset["front_similarity_gain"]=1.;asset["front_port_binding"]="unspecified";
    CHECK_FALSE(gf::task31AnchorScene(asset).valid);
}

TEST_CASE("port transformation is deterministic and frame covariant for one two three units") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    for(int code:{0,2,12}) {
        const auto c=gf::task25DagContractFromCode(code);
        const auto l=gf::task31TriangularLattice(c,fixed,{0,1},450);REQUIRE(l.valid);
        const auto bound=gf::task31PositiveTerminalPort(l);REQUIRE(bound.valid);
        const auto repeat=gf::task31PositiveTerminalPort(l);REQUIRE(repeat.valid);
        CHECK(bound.ports==repeat.ports);
        CHECK(bound.contract.reference_edges==c.reference_edges);
        const Eigen::Rotation2Dd rotation(.73);const Eigen::Vector2d shift(93,-117);
        auto rotated=bound.contract;auto rf=fixed;
        for(auto& [id,p]:rf)p=rotation*p+shift;
        for(auto& u:rotated.coverage_units)if(u.frame_origin)u.frame_origin=rotation*(*u.frame_origin)+shift;
        std::map<std::string,Eigen::Vector2d> fronts,moved;
        for(const auto& u:c.coverage_units){fronts[u.id]={50,2000};moved[u.id]=rotation*fronts.at(u.id)+shift;}
        const auto q=gf::task20LiftTargets(bound.contract,fixed,fronts);
        const auto qr=gf::task20LiftTargets(rotated,rf,moved);REQUIRE(q.valid);REQUIRE(qr.valid);
        for(const auto& [unit,port]:bound.ports)CHECK((q.targets.at(port)-fronts.at(unit)).norm()<1e-8);
        for(const auto& [id,p]:q.targets)CHECK((qr.targets.at(id)-(rotation*p+shift)).norm()<1e-8);
        // Relabel all mobile identities, carrying semantic member order.
        auto renamed=l;renamed.cells.clear();renamed.contract.member_roles.clear();
        auto ren=[](gf::NodeId id){return id+1000;};
        for(const auto& [id,cell]:l.cells)renamed.cells[ren(id)]=cell;
        for(const auto& [id,role]:l.contract.member_roles){auto r=role;r.member=ren(id);renamed.contract.member_roles[ren(id)]=r;}
        for(auto& u:renamed.contract.coverage_units){u.leader=ren(u.leader);for(auto& i:u.members)i=ren(i);for(auto& i:u.front_members)i=ren(i);}
        for(auto& e:renamed.contract.reference_edges){e.owner=ren(e.owner);if(l.contract.member_roles.count(e.reference))e.reference=ren(e.reference);}
        for(auto& i:renamed.contract.topological_order)i=ren(i);
        const auto rb=gf::task31PositiveTerminalPort(renamed);REQUIRE(rb.valid);
        for(const auto& [unit,port]:bound.ports)CHECK(rb.ports.at(unit)==ren(port));
        for(const auto& [id,r]:bound.contract.member_roles) {
            CHECK(rb.contract.member_roles.at(ren(id)).axial_fraction==r.axial_fraction);
            CHECK(rb.contract.member_roles.at(ren(id)).triangular_fraction==r.triangular_fraction);
        }
        nlohmann::json roles=nlohmann::json::object();
        for(const auto& [id,r]:bound.contract.member_roles)roles[std::to_string(id)]={r.axial_fraction,r.triangular_fraction};
        std::cout<<"TASK31_PORT_PROPERTY "<<nlohmann::json({{"code",code},{"units",c.coverage_units.size()},
            {"ports",bound.ports},{"roles",roles}}).dump()<<'\n';
    }
}

TEST_CASE("ambiguous or singular existing terminal roles cannot become implicit fallbacks") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto l=gf::task31TriangularLattice(gf::task25DagContractFromCode(12),fixed,{0,1},450);REQUIRE(l.valid);
    auto bad=l;bad.contract.coverage_units.front().front_members.clear();CHECK_FALSE(gf::task31PositiveTerminalPort(bad).valid);
    bad=l;for(auto id:bad.contract.coverage_units.front().front_members)bad.cells.at(id).slot=0.;
    CHECK_FALSE(gf::task31PositiveTerminalPort(bad).valid);
    bad=l;for(auto id:bad.contract.coverage_units.front().front_members){auto& r=bad.contract.member_roles.at(id);r.axial_fraction=0;r.triangular_fraction=0;}
    CHECK_FALSE(gf::task31PositiveTerminalPort(bad).valid);
}
