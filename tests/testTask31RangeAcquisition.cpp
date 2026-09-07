#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31AnchorScene.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/Task31InformationTelemetry.hpp"
#include <fstream>

TEST_CASE("The physical range generator applies distance availability at initialization with zero range noise") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");REQUIRE(f.good());
    nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
    auto cfg=gf::task19ProductionAdapterConfig();cfg.range_noise_std_m=0;cfg.range_dropout_probability=0;
    cfg.distance_range_availability.enabled=true;cfg.distance_range_availability.link_seed=131211;
    Swarm swarm(settings);gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,cfg);
    const auto initialized=adapter.initializeStageZero();INFO(initialized.reason);CHECK(initialized.initialized);
    const auto generated=adapter.lastRangeGenerationAudit();REQUIRE(generated.size()==175);
    int near=0,dropped=0;std::set<std::string> acquired;
    for(const auto& x:generated) {
        CHECK(x.batch==0);
        if(x.true_distance_m<=850){++near;CHECK(x.acquired);CHECK(x.probability==1);}
        dropped+=!x.acquired;if(x.acquired)acquired.insert(x.edge.id());
    }
    CHECK(near>0);CHECK(dropped>0);CHECK(acquired.size()<175);
    for(const auto& x:adapter.lastAcceptedRangeBatchAudit())CHECK(acquired.count(x.measurement.edge.id())==1);
    const auto info=gf::task31InformationTelemetry(adapter);
    REQUIRE(info.contains("range_acquisition"));
    CHECK(info["range_acquisition"]["generated_batch"].size()==175);
    CHECK(info["control_edges"].size()==28);
    CHECK(info["qualified_information"]["minimum_count"].get<int>()>=2);
    CHECK(info["qualified_information"]["robust_fim_min"].get<double>()>=1e-6);
    CHECK(info["posterior_max_m2"].get<double>()<=.1);
    CHECK(info["aoi_margin_min_s"].get<double>()>=0);
    std::cout<<"TASK31_RANGE_INITIAL "<<nlohmann::json({{"opportunities",175},{"within850",near},{"generation_dropped",dropped},
        {"accepted",adapter.lastAcceptedRangeBatchAudit().size()},{"qualified_information",info["qualified_information"]}}).dump()<<'\n';
    auto conflict=cfg;conflict.range_dropout_probability=.02;Swarm other(settings);
    CHECK_THROWS(gf::GrandFinaleSwarmAdapter(other,s.mobile_ids,s.fixed_positions,s.initial_topology,conflict));
}
