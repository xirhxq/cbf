#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31AnchorScene.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/Task31InformationTelemetry.hpp"
#include <fstream>

TEST_CASE("An explicit launch configuration installs the distance model before estimator initialization") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");REQUIRE(f.good());
    nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto cfg=gf::task19ProductionAdapterConfig();cfg.distance_range_availability={true,131211};
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
    gf::Task10p11rFixedBaselineFixture fixture(s,settings,cfg);
    REQUIRE(fixture.adapter.initializeStageZero().initialized);
    CHECK(fixture.adapter.config().distance_range_availability.enabled);
    CHECK(fixture.adapter.config().range_noise_std_m==0);
    CHECK(fixture.adapter.config().range_dropout_probability==0);
    CHECK(fixture.adapter.lastRangeGenerationAudit().size()==175);
    CHECK(fixture.controller.targetEpoch()==0);
}

TEST_CASE("Read only edge qualification does not consume the paired physical acquisition field") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");REQUIRE(f.good());
    nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
    for(auto& p:s.mobile_positions)p.x()+=750;
    auto cfg=gf::task19ProductionAdapterConfig();cfg.distance_range_availability={true,131211};
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
    gf::Task10p11rFixedBaselineFixture a(s,settings,cfg);
    // Zero noise: changing the Gaussian seed cannot change the independent
    // acquisition field, nor the actual motion/estimates.
    cfg.range_random_seed=40009;
    gf::Task10p11rFixedBaselineFixture b(s,settings,cfg);
    REQUIRE(a.adapter.initializeStageZero().initialized);REQUIRE(b.adapter.initializeStageZero().initialized);
    for(int k=0;k<20;++k) {
        const auto before=gf::task31InformationTelemetry(a.adapter);
        a.adapter.auditReplacementPlan({{{103,1},{101,1}}});
        CHECK(gf::task31InformationTelemetry(a.adapter)==before);
        REQUIRE(a.controller.advance().step.advanced);REQUIRE(b.controller.advance().step.advanced);
        const auto xa=a.adapter.runtimeSnapshot(),xb=b.adapter.runtimeSnapshot();
        CHECK(xa.estimate.mean.isApprox(xb.estimate.mean,1e-13));
        CHECK(gf::task31InformationTelemetry(a.adapter)==gf::task31InformationTelemetry(b.adapter));
        const auto gen=a.adapter.lastRangeGenerationAudit();REQUIRE(gen.size()==175);
        for(const auto& item:gen)CHECK(item.batch==std::size_t(k+1));
    }
}
