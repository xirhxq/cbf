#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32ModeTransition.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include <fstream>

namespace {
using json=nlohmann::json;
json read(const std::string& name) {
    std::ifstream stream(std::string(TASK32_EVIDENCE_ROOT)+"/"+name);
    json value;stream>>value;return value;
}
json plan(int start,std::vector<int> targets) {
    json modes=json::array();
    for(const auto& name:{"mode-contract-0-4500x2250-r1.json",
        "mode-contract-11-4500x2250-preparation-r1.json",
        "mode-contract-12-4500x2250-r1.json",
        "mode-contract-13-4500x2250-preparation-r1.json"})modes.push_back(read(name));
    return {{"schema","task32-mode-transition-plan-v1"},{"arm","A"},
        {"start_mode",start},{"target_modes",targets},{"first_request_s",60.0},
        {"after_restore_s",60.0},{"modes",modes}};
}
gf::Task32ModeTransitionPlan parse(const json& j) {
    return gf::task32ModeTransitionPlan(j,read("observation-r1-anchor-asset.json"));
}
}

TEST_CASE("Mode registry selects the requested DAG and complete target mapping") {
    auto p=parse(plan(12,{0,13}));REQUIRE(p.valid);
    CHECK(p.start().mode_code==12);
    CHECK(p.mode(0).contract.coverage_units.size()==2);
    CHECK(p.mode(12).contract.coverage_units.size()==1);
    CHECK(p.mode(13).mode_code==13);
    CHECK(gf::task25_detail::edgeSet(p.mode(0).contract.reference_edges)!=
          gf::task25_detail::edgeSet(p.mode(12).contract.reference_edges));
    CHECK(p.start().contract.id=="pinball-5-4-3-2-triangular-v1-positive-terminal-port-port-observation-anchor-asset");
    CHECK_THROWS(p.mode(2));
}

TEST_CASE("Request queue ends exactly once and schedules only after actual completion") {
    gf::Task32RequestSequence q({12,0},60.0,60.0);
    CHECK_FALSE(q.due(59.9));CHECK(q.due(60));CHECK(q.begin(60)==12);
    CHECK_FALSE(q.due(1000));CHECK_THROWS(q.begin(61));
    q.complete(230);CHECK_FALSE(q.due(289.9));CHECK(q.due(290));
    CHECK(q.begin(290)==0);q.complete(410);
    CHECK_FALSE(q.due(10000));CHECK(q.completed()==2);CHECK(q.planned()==2);
    CHECK(q.untriggered()==0);CHECK_FALSE(q.pending());CHECK_THROWS(q.complete(411));
    gf::Task32RequestSequence failed({12,0},60,60);
    failed.begin(60);failed.stop();CHECK_FALSE(failed.due(1000));
    CHECK(failed.completed()==0);CHECK(failed.untriggered()==1);
}

TEST_CASE("Complete directed mappings retain their own endpoints and physical anchors") {
    const auto p=parse(plan(0,{12,0,11}));REQUIRE(p.valid);
    for(const auto [old_mode,new_mode]:std::vector<std::pair<int,int>>{{0,12},{12,0},{0,11},{11,0},{0,13},{13,0}}) {
        const auto& old=p.mode(old_mode);const auto& goal=p.mode(new_mode);
        std::vector<gf::NodeId> anchors;
        for(const auto& [id,point]:old.fixed)anchors.push_back(id);
        const auto order=gf::task26ReplacementPlan(old.contract.reference_edges,goal.contract.reference_edges,anchors);
        REQUIRE(order.valid);
        auto graph=old.contract.reference_edges;
        for(const auto& [add,remove]:order.replacements) {
            graph.push_back(add);CHECK(gf::task26ValidDag(graph,3,anchors));
            graph=gf::transition_certifier_detail::without(graph,remove);
            CHECK(gf::task26ValidDag(graph,2,anchors));
        }
        CHECK(gf::task25_detail::edgeSet(graph)==gf::task25_detail::edgeSet(goal.contract.reference_edges));
        const auto fronts=gf::task32ModeCompactFronts(p,new_mode);
        const auto targets=gf::task20LiftTargets(goal.contract,goal.fixed,fronts);
        REQUIRE(targets.valid);CHECK(targets.targets.size()==14);
        if(new_mode!=12) {
            const auto legacy=gf::task26CompactFronts(goal.contract,goal.fixed);
            CHECK(fronts==legacy);
        }
    }
}

TEST_CASE("Arm identity and shared physical scene fail closed") {
    auto value=plan(0,{12});auto a=parse(value);REQUIRE(a.valid);CHECK(a.targetMechanism()==0);
    value["arm"]="C";auto c=parse(value);REQUIRE(c.valid);CHECK(c.targetMechanism()==2);
    value["arm"]="B";CHECK_FALSE(parse(value).valid);
    value=plan(0,{12});value["target_modes"]={12,12};CHECK_FALSE(parse(value).valid);
    value=plan(0,{12});value["first_request_s"]=30;CHECK_FALSE(parse(value).valid);
    value=plan(0,{12});value["modes"].push_back(value["modes"][0]);CHECK_FALSE(parse(value).valid);
}

TEST_CASE("Unregistered transition startup and missing final mappings are rejected before control") {
    auto p=parse(plan(12,{0}));REQUIRE(p.valid);
    CHECK_THROWS(gf::task32ValidateTransitionStartup(p,0,p.mode(0).contract.reference_edges,
        p.start().fixed,0));
    CHECK_THROWS(gf::task32ValidateTransitionStartup(p,12,p.mode(0).contract.reference_edges,
        p.start().fixed,0));
    CHECK_NOTHROW(gf::task32ValidateTransitionStartup(p,12,p.start().contract.reference_edges,
        p.start().fixed,0));
    CHECK_THROWS(gf::task32ValidateTransitionStartup(p,12,p.start().contract.reference_edges,
        p.start().fixed,2));
}

TEST_CASE("Fourteen-member common bridge and bow stay continuous in both directions") {
    const auto p=parse(plan(0,{12}));REQUIRE(p.valid);
    for(const auto [old_mode,new_mode]:std::vector<std::pair<int,int>>{{0,12},{12,0},{0,11},{11,0},{0,13},{13,0}}) {
        const auto& old=p.mode(old_mode).contract;const auto& goal=p.mode(new_mode).contract;
        const auto& scene=p.pinball_scene;
        const auto bridge=gf::task31CommonBridge(old,goal,scene.fixed,scene.direction,
            scene.bridge_spacing_m,scene.ranking_span_m,scene.frame_origin);
        REQUIRE(bridge.valid);REQUIRE(bridge.targets.size()==14);
        auto fronts=gf::task32ModeCompactFronts(p,old_mode);
        for(auto& [id,front]:fronts)front+=Eigen::Vector2d(175,500);
        auto initial=gf::task20LiftTargets(old,scene.fixed,fronts).targets;
        // Continuity must not depend on a perfect inverse/lift image.
        for(auto& [id,point]:initial)point+=Eigen::Vector2d(.1*id,-.2*id);
        const gf::Task32ContractionPath path(initial,bridge.targets);
        const auto start=path.evaluate(0),end=path.evaluate(1);
        for(const auto& [id,point]:initial) {
            CHECK((start.at(id)-point).norm()<1e-12);
            CHECK((end.at(id)-bridge.targets.at(id)).norm()<1e-12);
            CHECK((path.evaluate(1e-9).at(id)-point).norm()<1e-4);
        }
        for(int k=0;k<=100;++k)for(const auto& [id,point]:path.evaluate(k/100.))CHECK(point.allFinite());
        const auto target=gf::task20LiftTargets(goal,scene.fixed,gf::task32ModeCompactFronts(p,new_mode)).targets;
        const auto admitted=path.evaluate(.45);
        const gf::Task29RoleCenterPath expand(goal,admitted,target);
        for(const auto& [id,point]:target) {
            CHECK((expand.evaluate(0).at(id)-admitted.at(id)).norm()<1e-8);
            CHECK((expand.evaluate(1).at(id)-point).norm()<1e-8);
        }
    }
}
