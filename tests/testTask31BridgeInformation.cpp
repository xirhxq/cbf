#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31CommonBridge.hpp"
#include "grand_finale/Task31InformationTelemetry.hpp"
#include "grand_finale/Task25P0MultiDag.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task31 reference-only degeneration does not substitute for the formal qualified information gate") {
    auto scenario=gf::task10p11rFixedBaselineScenario();scenario.width_m=4500;scenario.height_m=2250;
    scenario.fixed_positions={{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto bridge=gf::task31CommonBridge(old,goal,scenario.fixed_positions,{0,1});REQUIRE(bridge.valid);
    scenario.mobile_positions.clear();for(auto id:scenario.mobile_ids)scenario.mobile_positions.push_back(bridge.targets.at(id));
    for(int code:{0,12}) {
        scenario.initial_topology=gf::task25DagContractFromCode(code).reference_edges;
        auto cfg=gf::task19ProductionAdapterConfig();
        auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);Swarm swarm(settings);
        gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,cfg);
        REQUIRE(adapter.initializeStageZero().initialized);
        const auto j=gf::task31InformationTelemetry(adapter);
        CHECK(j.at("current_time_s")==0);CHECK(j.at("control_edges").size()==28);
        CHECK(j.at("reference_only").at("robust_fim_min").get<double>()<0);
        CHECK(j.at("qualified_information").at("minimum_count")==16);
        CHECK(j.at("qualified_information").at("robust_fim_min").get<double>()>1e-6);
        CHECK(j.at("posterior_max_m2").get<double>()<.1);
        std::cout<<"TASK31_BRIDGE_INFORMATION "<<nlohmann::json({{"mode",code},{"actual_initialized_geometry",true},{"plant_advanced",false},{"observation",j}}).dump()<<'\n';
    }
}
