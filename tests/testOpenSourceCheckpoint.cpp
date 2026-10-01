#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/FullStateCheckpoint.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/CloudSolverProfile.hpp"

TEST_CASE("cloud solver selection is opt-in and does not replace the Gurobi build default") {
#ifdef GF_CLOUD_OSQP
    CHECK(gf::cloudSolverProfile()==gf::SolverProfile::OpenSource);
#else
    CHECK(gf::cloudSolverProfile()==gf::SolverProfile::Gurobi);
#endif
}

static auto openSourceFixture() {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::OpenSource);
    auto config=gf::task19ProductionAdapterConfig(0.5,0.0,217011);
    config.solver_profile=gf::SolverProfile::OpenSource;
    config.target_policy_task18_cbf2026_outer=false;
    config.target_policy_task20_dag_lattice=true;
    config.task32_target_mechanism=2;
    config.distance_range_availability.enabled=true;
    config.distance_range_availability.link_seed=218011;
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(scenario,settings,config);
}

TEST_CASE("OpenSource complete checkpoint captures and restores the same public control boundary") {
    auto a=openSourceFixture();
    REQUIRE(a->adapter.initializeStageZero().initialized);
    const nlohmann::json identity={{"binary","OSQP-test-same-binary"},{"solver","OpenSource"}};
    const auto saved=gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity);
    auto b=openSourceFixture();
    REQUIRE(b->adapter.initializeStageZero().initialized);
    gf::FullStateCheckpoint::restore(b->swarm,b->adapter,b->controller,
        nlohmann::json::from_cbor(nlohmann::json::to_cbor(saved)),identity);
    CHECK(gf::FullStateCheckpoint::capture(b->swarm,b->adapter,b->controller,identity)==saved);
}

TEST_CASE("OSQP successors remain identical after a complete checkpoint roundtrip") {
    auto a=openSourceFixture();REQUIRE(a->adapter.initializeStageZero().initialized);
    for(int k=0;k<3;++k)REQUIRE(a->controller.advance().step.advanced);
    const nlohmann::json identity={{"binary","OSQP-test-same-binary"},{"solver","OpenSource"}};
    const auto saved=gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity);
    auto b=openSourceFixture();REQUIRE(b->adapter.initializeStageZero().initialized);
    gf::FullStateCheckpoint::restore(b->swarm,b->adapter,b->controller,saved,identity);
    for(int k=0;k<3;++k) {
        REQUIRE(a->controller.advance().step.advanced);
        REQUIRE(b->controller.advance().step.advanced);
        CHECK(a->controller.lastNominalControls()==b->controller.lastNominalControls());
        CHECK(gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity)==
              gf::FullStateCheckpoint::capture(b->swarm,b->adapter,b->controller,identity));
    }
}

TEST_CASE("OSQP checkpoint cannot migrate solver identity or admit unsupported policies") {
    auto a=openSourceFixture();REQUIRE(a->adapter.initializeStageZero().initialized);
    const nlohmann::json identity={{"binary","OSQP-test-same-binary"},{"solver","OpenSource"}};
    const auto before=gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity);
    auto changed=before;
    changed["immutable"]["adapter_config"]["solver_profile"]=gf::checkpoint::encode(gf::SolverProfile::Gurobi);
    CHECK_THROWS(gf::FullStateCheckpoint::restore(a->swarm,a->adapter,a->controller,changed,identity));
    CHECK(gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity)==before);
    auto config=a->adapter.config();
    config.target_homotopy_enabled=true;
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::OpenSource);
    gf::Task10p11rFixedBaselineFixture unsupported(scenario,settings,config);
    REQUIRE(unsupported.adapter.initializeStageZero().initialized);
    CHECK_THROWS(gf::FullStateCheckpoint::capture(unsupported.swarm,unsupported.adapter,unsupported.controller,identity));
}
