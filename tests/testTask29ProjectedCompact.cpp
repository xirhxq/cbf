#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task29ProjectedCompact.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

namespace {
const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
auto make(double height) {
    auto c=gf::task25DagContractFromCode(0);auto fronts=gf::task26CompactFronts(c,fixed);
    for(auto& [u,p]:fronts)p.y()=fixed.at(101).y()+height;
    return gf::task20LiftTargets(c,fixed,fronts).targets;
}
auto projected(const std::map<gf::NodeId,Eigen::Vector2d>& p,double limit=850) {
    const auto c=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    auto zero=gf::task26CompactFronts(c,fixed);for(auto& [u,q]:zero)q.y()=fixed.at(101).y();
    auto edges=c.reference_edges;edges.insert(edges.end(),goal.reference_edges.begin(),goal.reference_edges.end());
    return gf::task29ProjectedCompact(c,fixed,zero,{0,1},p,edges,limit,3*std::sqrt(.1),10);
}
}
TEST_CASE("Task29 projected compact uses a closed scalar height without new meter reserve") {
    const auto x=projected(make(1400));REQUIRE(x.valid);
    CHECK(x.height_m==doctest::Approx(847.9852078304).epsilon(1e-10));
    CHECK(x.unconstrained_height_m==doctest::Approx(1400));
    CHECK(x.maximum_supported_edge_m<=850+1e-9);CHECK(x.minimum_separation_m>10);
    CHECK(x.target_count==14);CHECK(x.height_m>x.baseline_zero_height_m);
    const auto inside=projected(make(650));REQUIRE(inside.valid);CHECK(inside.height_m==doctest::Approx(650));
    const auto config=gf::task19ProductionAdapterConfig();
    CHECK(config.reference_distance_m==850);CHECK(config.uncertainty_sigma==3);
    CHECK(config.maximum_posterior_eigenvalue_m2==.1);CHECK(config.certified_shadow_single_position_support_m==0);
    std::cout<<"TASK29_COMPACT "<<nlohmann::json({{"height_m",x.height_m},{"maximum_supported_edge_m",x.maximum_supported_edge_m},
        {"minimum_separation_m",x.minimum_separation_m},{"active_edge",x.active_edge}}).dump()<<'\n';
}
TEST_CASE("Task29 all fourteen estimated positions influence compact projection; no truth input exists") {
    auto p=make(650);const auto a=projected(p);REQUIRE(a.valid);
    for(const auto& [id,q]:p){auto v=p;v[id].y()+=1;const auto b=projected(v);REQUIRE(b.valid);CHECK(b.height_m>a.height_m);}
    for(double h:{600.,700.,800.,1000.,1600.}) {
        const auto x=projected(make(h));
        if(!x.valid){CHECK(x.reason=="nominal_separation_failed");CHECK(x.minimum_separation_m<=10);continue;}
        for(int k=0;k<100;++k) {
            const double v=x.minimum_height_m+(x.maximum_height_m-x.minimum_height_m)*(k+.5)/100;
            double trial=0;for(const auto& [id,p]:make(h))trial+=(make(v).at(id)-p).squaredNorm();
            CHECK(x.squared_displacement_m2<=trial+1e-6);
        }
    }
}
TEST_CASE("Task29 projected compact fails closed on infeasible or malformed geometry") {
    CHECK_FALSE(projected(make(700),100).valid);
    auto p=make(700);p.erase(14);CHECK_FALSE(projected(p).valid);
    p=make(700);p.at(3).x()=std::numeric_limits<double>::quiet_NaN();CHECK_FALSE(projected(p).valid);
    const auto c=gf::task25DagContractFromCode(0);auto zero=gf::task26CompactFronts(c,fixed);
    for(auto& [u,q]:zero)q=fixed.at(101); // coincident terminal slots for every height.
    CHECK_FALSE(gf::task29ProjectedCompact(c,fixed,zero,{0,1},make(700),c.reference_edges,850,3*std::sqrt(.1),10).valid);
    zero=gf::task26CompactFronts(c,fixed);zero["unused"]={std::numeric_limits<double>::quiet_NaN(),0};
    CHECK_FALSE(gf::task29ProjectedCompact(c,fixed,zero,{0,1},make(700),c.reference_edges,850,3*std::sqrt(.1),10).valid);
}
TEST_CASE("Task29 C is explicitly opted in, all old and moving defaults remain distinct") {
    auto s=gf::task10p11rFixedBaselineScenario();auto c=gf::task19ProductionAdapterConfig();
    CHECK(c.target_policy_task18_cbf2026_outer);c.target_policy_task18_cbf2026_outer=false;c.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter a(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,c);
    gf::Task10p11hSimpleCoverageController control(swarm,a);REQUIRE(a.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor b(a,control,"pinball-qualified-layered-centeredframe-moving");
    CHECK_FALSE(b.telemetry().contains("task29_compact"));
    gf::Task26ExternalReconstructor opt(a,control,"pinball-qualified-layered-centeredframe-moving-projected");
    CHECK(opt.telemetry().at("task29_compact").at("enabled")==true);
}
