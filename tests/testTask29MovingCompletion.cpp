#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task29MovingCompletion.hpp"
#include "grand_finale/Task25P0MultiDag.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

namespace {
const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
auto movingState(const gf::Task20DagLatticeContract& c,const Eigen::Vector2d& w) {
    std::map<std::string,Eigen::Vector2d> fronts;
    for (const auto& u:c.coverage_units) fronts[u.id]={2250,950};
    const auto lifted=gf::task20LiftTargets(c,fixed,fronts);
    std::map<gf::NodeId,gf::Task29MotionState> state;
    for (const auto& [id,p]:lifted.targets)
        state[id]={p,gf::task29RoleMatrix(c.member_roles.at(id))*w,0,0};
    return state;
}
}

TEST_CASE("Task29 moving contract rejects graph-only and accepts coordinated fourteen-member motion") {
    const auto c=gf::task25DagContractFromCode(12);auto state=movingState(c,{12,4});
    std::map<gf::NodeId,Eigen::Vector2d> q;for (const auto& [id,s]:state)q[id]=s.position;
    auto x=gf::task29MovingCompletion(c,fixed,q,state,true,true);
    REQUIRE(x.valid);CHECK(x.moving_instant_ready);CHECK_FALSE(x.legacy_instant_ready);
    CHECK(x.maximum_coordinated_speed_bound<1e-12);
    CHECK((x.front_positions.at("P")-Eigen::Vector2d(2250,950)).norm()<1e-10);
    CHECK((x.front_velocities.at("P")-Eigen::Vector2d(12,4)).norm()<1e-10);
    CHECK_FALSE(gf::task29MovingCompletion(c,fixed,q,state,false,true).moving_instant_ready);
    CHECK_FALSE(gf::task29MovingCompletion(c,fixed,q,state,true,false).moving_instant_ready);
    state.at(5).position.x()+=200;
    CHECK_FALSE(gf::task29MovingCompletion(c,fixed,q,state,true,true).moving_instant_ready);
}

TEST_CASE("Task29 instantaneous shape is insufficient for incoherent velocities or uncertain motion") {
    const auto c=gf::task25DagContractFromCode(12);auto state=movingState(c,{8,3});
    std::map<gf::NodeId,Eigen::Vector2d> q;for (const auto& [id,s]:state)q[id]=s.position;
    for (const auto& [id,s]:state) {
        auto bad=state;bad[id].velocity+=Eigen::Vector2d(8,-8);
        CHECK_FALSE(gf::task29MovingCompletion(c,fixed,q,bad,true,true).moving_instant_ready);
    }
    for (auto& [id,s]:state)s.velocity_support=4;
    CHECK_FALSE(gf::task29MovingCompletion(c,fixed,q,state,true,true).moving_instant_ready);
    state.erase(14);CHECK_FALSE(gf::task29MovingCompletion(c,fixed,q,state,true,true).valid);
}

TEST_CASE("Task29 fixed-front estimator has no number or coverage-unit-count special branch") {
    for (int code:{0,11,12,13,2}) {
        const auto c=gf::task25DagContractFromCode(code);auto state=movingState(c,{7,-2});
        std::map<gf::NodeId,Eigen::Vector2d> q;for (const auto& [id,s]:state)q[id]=s.position;
        const auto x=gf::task29MovingCompletion(c,fixed,q,state,true,true);
        REQUIRE(x.valid);CHECK(x.moving_instant_ready);CHECK(x.maximum_coordinated_speed_bound<1e-12);
        for (const auto& u:c.coverage_units)CHECK((x.front_velocities.at(u.id)-Eigen::Vector2d(7,-2)).norm()<1e-10);
    }
}

TEST_CASE("Task29 explicit opt-in reaches coordinator without changing production defaults") {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto config=gf::task19ProductionAdapterConfig();
    CHECK(config.target_policy_task18_cbf2026_outer);
    config.target_policy_task18_cbf2026_outer=false;config.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,config);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    REQUIRE(adapter.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor legacy(adapter,controller,"pinball-qualified-layered-centeredframe");
    CHECK_FALSE(legacy.telemetry().contains("task29"));
    const auto before=adapter.runtimeSnapshot();
    gf::Task26ExternalReconstructor moving(adapter,controller,"pinball-qualified-layered-centeredframe-moving");
    CHECK(moving.telemetry().at("task29").at("completion_contract")=="moving-v1");
    CHECK_FALSE(moving.telemetry().at("task29").at("applicable").get<bool>());
    CHECK((before.estimate.mean-adapter.runtimeSnapshot().estimate.mean).norm()==0);
    CHECK(before.topology_token==adapter.runtimeSnapshot().topology_token);
}

TEST_CASE("Task29 uncertainty bound contains deterministic joint errors without independence") {
    const auto c=gf::task25DagContractFromCode(12);auto states=movingState(c,{9,2});
    std::map<gf::NodeId,Eigen::Vector2d> q;for (auto& [id,s]:states){q[id]=s.position;s.velocity_support=.7;}
    const auto bounds=gf::task29MovingCompletion(c,fixed,q,states,true,true);REQUIRE(bounds.valid);
    for (int k=0;k<100;++k) {
        auto actual=states;
        for (auto& [id,s]:actual){const double theta=.173*k+.371*id;s.velocity+=.7*Eigen::Vector2d(std::cos(theta),std::sin(theta));s.velocity_support=0;}
        const auto truth=gf::task29MovingCompletion(c,fixed,q,actual,true,true);
        for (const auto& [id,b]:bounds.coordinated_speed_bounds)CHECK(truth.coordinated_speed_bounds.at(id)<=b+1e-10);
    }
    auto bad=states;bad.begin()->second.velocity_support=-1;CHECK_FALSE(gf::task29MovingCompletion(c,fixed,q,bad,true,true).valid);
    auto singular=c;for(auto& [id,r]:singular.member_roles){r.axial_fraction=0;r.triangular_fraction=0;}
    CHECK_FALSE(gf::task29MovingCompletion(singular,fixed,q,states,true,true).valid);
}
