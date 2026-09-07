#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("The same physical scene has the same acquisition and EKF initialization in H0 and Pinball") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");REQUIRE(f.good());
    nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    nlohmann::json initial;
    for(int mode:{0,12}) {
        auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
        for(auto& p:s.mobile_positions)p.x()+=750;
        if(mode==12)s.initial_topology=scene.goal.reference_edges;
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
        cfg.distance_range_availability={true,131211};
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);REQUIRE(x.adapter.initializeStageZero().initialized);
        const auto info=gf::task31InformationTelemetry(x.adapter);
        if(mode==0)initial=info;
        else for(const auto* key:{"range_acquisition","accepted_batch","member_position_velocity_posterior_max_eigenvalues","qualified_information","posterior_max_m2"})CHECK(info[key]==initial[key]);
        gf::Task26ExternalReconstructor observer(x.adapter,x.controller,
            "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,false,scene,true,mode);
        for(int k=0;k<20;++k) {
            observer.beforeStep();REQUIRE(x.controller.advance().step.advanced);
            const auto now=gf::task31InformationTelemetry(x.adapter);
            CHECK(now["qualified_information"]["robust_fim_min"].get<double>()>=1e-6);
            CHECK(now["posterior_max_m2"].get<double>()<=.1);CHECK(now["aoi_margin_min_s"].get<double>()>=0);
        }
        CHECK(observer.report()["requests"].empty());
        std::cout<<"TASK31_RANGE_MODE_FIXTURE "<<nlohmann::json({{"mode",mode},{"safe_ticks",20},{"requests",0},{"initial_info",info["qualified_information"]}}).dump()<<'\n';
    }
}
