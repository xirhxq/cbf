#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31AnchorScene.hpp"
#include "grand_finale/Task20CoveragePolicy.hpp"
#include <fstream>

static nlohmann::json asset() {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-terminal-port-r1.json");
    if(!f.good())throw std::runtime_error("missing frozen asset");
    nlohmann::json j;f>>j;return j;
}

TEST_CASE("search observation can follow registered port without changing the moving front") {
    auto j=asset();const auto old=gf::task31AnchorScene(j);REQUIRE(old.valid);
    j["search_observation"]="terminal_port";
    const auto next=gf::task31AnchorScene(j);REQUIRE(next.valid);
    const auto& u=next.goal.coverage_units.front();
    CHECK(u.front_members==old.goal.coverage_units.front().front_members);
    CHECK(next.goal.reference_edges==old.goal.reference_edges);
    CHECK(next.fixed==old.fixed);
    const auto port=next.terminal_ports.at(u.id);
    std::vector<gf::Task16CoverageAgentState> agents;
    for(auto id:u.members)agents.push_back({id,{id==port?1005.:2005.,1005.},{2,3},0,0});
    gf::Task20CoverageRequest request;request.contract=next.goal;request.fixed_positions=next.fixed;
    request.agents=agents;request.uncovered_cells={{140,100,{1405,1005}},{190,100,{1905,1005}}};
    const auto selected=gf::allocateTask20Coverage(request);REQUIRE(selected.valid);
    // Worked fixture: port focus=(1405,1005); legacy mean focus=(1905,1005).
    CHECK(selected.assignments.at(u.id).task.id()=="140:100");
    for(const auto& [id,target]:selected.targets)CHECK(target.id()=="140:100");
    std::map<std::string,Eigen::Vector2d> fronts{{u.id,{5,2245}}};
    const auto qo=gf::task20LiftTargets(old.goal,old.fixed,fronts);
    const auto qn=gf::task20LiftTargets(next.goal,next.fixed,fronts);
    REQUIRE(qo.valid);REQUIRE(qn.valid);CHECK(qo.targets==qn.targets);
    std::map<gf::NodeId,gf::Task29MotionState> states;
    for(const auto& [id,p]:qo.targets)states[id]={p,gf::task29RoleMatrix(old.goal.member_roles.at(id))*Eigen::Vector2d(8,4),.05,.01};
    const auto a=gf::task29MovingCompletion(old.goal,old.fixed,qo.targets,states,true,true);
    const auto b=gf::task29MovingCompletion(next.goal,next.fixed,qn.targets,states,true,true);
    CHECK(a.valid);CHECK(b.valid);CHECK(a.front_positions==b.front_positions);
    CHECK(a.front_velocities==b.front_velocities);CHECK(a.coordinated_speed_bounds==b.coordinated_speed_bounds);
    CHECK(a.moving_instant_ready==b.moving_instant_ready);
    CHECK(gf::task31UnscaledContinuationRates(old,qo.targets)==gf::task31UnscaledContinuationRates(next,qo.targets));
}

TEST_CASE("empty observation configuration is exact and unknown contracts are rejected") {
    auto j=asset();const auto old=gf::task31AnchorScene(j);REQUIRE(old.valid);
    j["search_observation"]="legacy_front";const auto same=gf::task31AnchorScene(j);REQUIRE(same.valid);
    std::vector<gf::Task16CoverageAgentState> agents;
    const auto port=old.terminal_ports.begin()->second;
    for(auto id:old.goal.coverage_units.front().members)agents.push_back({id,{id==port?1005.:2005.,1005.},{0,0},0,0});
    gf::Task20CoverageRequest request;request.contract=old.goal;request.fixed_positions=old.fixed;
    request.agents=agents;request.uncovered_cells={{140,100,{1405,1005}},{190,100,{1905,1005}}};
    const auto a=gf::allocateTask20Coverage(request);request.contract=same.goal;
    const auto b=gf::allocateTask20Coverage(request);REQUIRE(a.valid);REQUIRE(b.valid);
    CHECK(a.assignments.begin()->second.task.id()=="190:100");
    CHECK(a.fronts==b.fronts);
    for(const auto& [id,target]:a.targets) {
        CHECK(target.id()==b.targets.at(id).id());CHECK(target.center==b.targets.at(id).center);
    }
    j["search_observation"]="unsupported";CHECK_FALSE(gf::task31AnchorScene(j).valid);
    j["search_observation"]="terminal_port";j["front_port_binding"]="none";
    CHECK_FALSE(gf::task31AnchorScene(j).valid);
}

TEST_CASE("observation contract is unit-count independent deterministic and frame covariant") {
    for(int code:{12,0,2}) {
        auto contract=gf::task25DagContractFromCode(code);REQUIRE(contract.valid);
        gf::Task20CoverageRequest r;r.contract=contract;
        r.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
        for(auto& u:r.contract.coverage_units)u.search_observation_members={u.front_members.front()};
        for(gf::NodeId id=1;id<=14;++id)r.agents.push_back({id,{100.+100*id,500.+10*id},{3,4},.3,0});
        const std::vector<gf::FrontierCell> cells{{10,80,{105,805}},{120,110,{1205,1105}},{240,90,{2405,905}}};
        for(int n=0;n<=3;++n) {
            r.uncovered_cells.assign(cells.begin(),cells.begin()+n);
            const auto a=gf::allocateTask20Coverage(r),repeat=gf::allocateTask20Coverage(r);
            REQUIRE(a.valid);REQUIRE(repeat.valid);CHECK(a.fronts==repeat.fronts);CHECK(a.complete==(n==0));
            std::set<std::string> selected;
            for(const auto& [u,x]:a.assignments)CHECK(selected.insert(x.task.id()).second);
            CHECK(selected.size()==std::min(std::size_t(n),r.contract.coverage_units.size()));
            auto rotated=r;const Eigen::Rotation2Dd q(.7);const Eigen::Vector2d shift(73,-31);
            for(auto& [id,p]:rotated.fixed_positions)p=q*p+shift;
            for(auto& a:rotated.agents){a.position=q*a.position+shift;a.velocity=q*a.velocity;a.yaw_rad+=.7;}
            for(auto& c:rotated.uncovered_cells)c.center=q*c.center+shift;
            const auto b=gf::allocateTask20Coverage(rotated);REQUIRE(b.valid);
            for(const auto& [u,x]:a.assignments)CHECK(x.task.id()==b.assignments.at(u).task.id());
            for(const auto& [id,c]:a.targets) {
                CHECK(c.id()==b.targets.at(id).id());CHECK((q*c.center+shift-b.targets.at(id).center).norm()<1e-8);
            }
            // A bijection of mobile identities carries all semantic roles.
            auto renamed=r;auto ren=[](gf::NodeId id){return 15-id;};
            renamed.contract.member_roles.clear();
            for(const auto& [id,role]:r.contract.member_roles){auto v=role;v.member=ren(id);renamed.contract.member_roles[ren(id)]=v;}
            for(auto& u:renamed.contract.coverage_units) {
                u.leader=ren(u.leader);for(auto& id:u.members)id=ren(id);
                for(auto& id:u.front_members)id=ren(id);for(auto& id:u.search_observation_members)id=ren(id);
            }
            for(auto& e:renamed.contract.reference_edges){e.owner=ren(e.owner);if(e.reference<100)e.reference=ren(e.reference);}
            for(auto& a:renamed.agents)a.id=ren(a.id);
            const auto b2=gf::allocateTask20Coverage(renamed);REQUIRE(b2.valid);
            for(const auto& [u,x]:a.assignments)CHECK(x.task.id()==b2.assignments.at(u).task.id());
            for(const auto& [id,c]:a.targets){CHECK(c.id()==b2.targets.at(ren(id)).id());CHECK(c.center==b2.targets.at(ren(id)).center);}
        }
        auto bad=r.contract;bad.coverage_units[0].search_observation_members={100};
        gf::task20_lattice_detail::finish(bad);CHECK_FALSE(bad.valid);
        bad=r.contract;const auto id=bad.coverage_units[0].members.front();
        bad.coverage_units[0].search_observation_members={id,id};
        gf::task20_lattice_detail::finish(bad);CHECK_FALSE(bad.valid);
        if(bad.coverage_units.size()>1) {
            bad=r.contract;bad.coverage_units[0].search_observation_members={bad.coverage_units[1].members.front()};
            gf::task20_lattice_detail::finish(bad);CHECK_FALSE(bad.valid);
        }
    }
}
