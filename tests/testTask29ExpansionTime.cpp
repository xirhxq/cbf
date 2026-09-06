#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task29ExpansionTime.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task29 linear common expansion phase is finite clamped monotone and leaves geometry unchanged") {
    CHECK(gf::task29LinearExpansionPhase(-1)==0);
    CHECK(gf::task29LinearExpansionPhase(0)==0);
    CHECK(gf::task29LinearExpansionPhase(15)==.25);
    CHECK(gf::task29LinearExpansionPhase(60)==1);
    CHECK(gf::task29LinearExpansionPhase(61)==1);
    CHECK_THROWS(gf::task29LinearExpansionPhase(std::numeric_limits<double>::quiet_NaN()));
    CHECK_THROWS(gf::task29LinearExpansionPhase(std::numeric_limits<double>::infinity()));
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto a=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    const auto b=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
    gf::Task28LayerPath path(goal,a,b,gf::Task28LayerPath::Kind::CenteredFrame);
    double previous=-1;
    for(int k=0;k<=10000;++k) {
        const double time=60.*k/10000,h=gf::task29LinearExpansionPhase(time);
        CHECK(h>=previous);previous=h;
        const auto q=path.evaluate(h),same=path.evaluate(time/60.);
        for(const auto& [id,p]:q)CHECK((p-same.at(id)).norm()==0);
    }
    for(const auto& [id,p]:a)CHECK((path.evaluate(0).at(id)-p).norm()==0);
    for(const auto& [id,p]:b)CHECK((path.evaluate(1).at(id)-p).norm()==0);
}
TEST_CASE("Task29 linear phase opt in does not enable projected compact or change production") {
    auto scenario=gf::task10p11rFixedBaselineScenario();auto cfg=gf::task19ProductionAdapterConfig();
    CHECK(cfg.target_policy_task18_cbf2026_outer);
    cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor b(adapter,controller,"pinball-qualified-layered-centeredframe-moving");
    CHECK_FALSE(b.telemetry().contains("task29_timing"));
    gf::Task26ExternalReconstructor c(adapter,controller,"pinball-qualified-layered-centeredframe-moving-linearphase");
    CHECK(c.telemetry().at("task29_timing").at("common_phase")=="linear");
    CHECK(c.telemetry().at("task29_timing").at("duration_s")==60);
    CHECK(c.telemetry().contains("task29"));CHECK_FALSE(c.telemetry().contains("task29_compact"));
}
TEST_CASE("Task29 uniform terminal translation is not automatically coordinated lifting motion") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto a=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    const auto b=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
    Eigen::Vector2d velocity=Eigen::Vector2d::Zero();
    for(const auto& [id,p]:a)velocity+=b.at(id)-p;
    velocity/=60.*a.size();
    std::map<gf::NodeId,gf::Task29MotionState> states;
    for(const auto& [id,p]:b)states[id]={p,velocity,0,0};
    const auto gate=gf::task29MovingCompletion(goal,fixed,b,states,true,true);
    REQUIRE(gate.valid);CHECK_FALSE(gate.moving_instant_ready);
    CHECK(gate.maximum_coordinated_speed_bound>3);
    // Exact terminal positions and equal velocities do not waive moving-v1.
    // Fixed anchors imply different member/front velocity matrices.
}
