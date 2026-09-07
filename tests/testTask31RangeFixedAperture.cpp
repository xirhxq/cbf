#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("Existing aperture fixed mode preserves real ledger and full14 launch qualifications") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-aperture-r1.json");
    REQUIRE(f.good());nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    for(int variant:{0,1,2}) {
        auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;
        s.fixed_positions=scene.fixed;s.initial_topology=scene.goal.reference_edges;
        for(size_t k=0;k<s.mobile_positions.size();++k)
            s.mobile_positions[k]+=Eigen::Vector2d(750+variant*std::sin(k),variant*std::cos(k));
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;
        cfg.target_policy_task20_dag_lattice=true;cfg.distance_range_availability={true,131211};
        REQUIRE(cfg.range_noise_std_m==0);REQUIRE(cfg.range_dropout_probability==0);
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:s.mobile_ids)settings["initial"]["velocity"]["values"].push_back({variant*std::cos(id),variant*std::sin(id)});
        Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
        gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
        const auto init=adapter.initializeStageZero();INFO(variant<<" "<<init.reason);REQUIRE(init.initialized);
        const auto token=adapter.runtimeSnapshot().estimator_token;
        gf::Task26ExternalReconstructor observer(adapter,controller,
            "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,false,scene,true,12);
        CHECK(adapter.runtimeSnapshot().estimator_token==token);
        for(int tick=0;tick<20;++tick) {
            observer.beforeStep();const auto step=controller.advance();INFO(tick<<" "<<step.step.reason);REQUIRE(step.step.advanced);
            const auto info=gf::task31InformationTelemetry(adapter);const auto ext=observer.telemetry();
            CHECK(info["qualified_information"]["robust_fim_min"].get<double>()>=1e-6);
            CHECK(info["posterior_max_m2"].get<double>()<=.1);CHECK(info["aoi_margin_min_s"].get<double>()>=0);
            CHECK(ext["active_mode"]==12);CHECK(ext["stage"]=="search");CHECK(ext["request_count"]==0);
            REQUIRE(controller.committedTargets().size()==14);
            for(const auto& [id,cell]:controller.committedTargets()){CHECK(cell.x_index>=0);CHECK(cell.y_index>=0);}
            CHECK(gf::task25_detail::edgeSet(adapter.runtimeSnapshot().topology)==gf::task25_detail::edgeSet(scene.goal.reference_edges));
        }
        CHECK(observer.report()["planned_request_count"]==0);CHECK(observer.report()["requests"].empty());
        std::cout<<"TASK31_FIXED_APERTURE_FIXTURE "<<nlohmann::json({{"variant",variant},{"safe_ticks",20},{"requests",0},{"gain",scene.front_similarity_gain}}).dump()<<'\n';
    }
}
