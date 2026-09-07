#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31InformationTelemetry.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task31 diagnostic separates accepted ranging graph from control DAG without mutating estimator") {
    auto s=gf::task10p11rFixedBaselineScenario();auto c=gf::task19ProductionAdapterConfig();
    auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,s.mobile_ids,s.fixed_positions,s.initial_topology,c);
    REQUIRE(adapter.initializeStageZero().initialized);
    const auto before=adapter.runtimeSnapshot();const auto batch=adapter.lastAcceptedRangeBatchAudit();
    const auto j=gf::task31InformationTelemetry(adapter);
    const auto after=adapter.runtimeSnapshot();
    CHECK(before.estimator_token==after.estimator_token);CHECK(before.topology_token==after.topology_token);
    CHECK((before.estimate.mean.array()==after.estimate.mean.array()).all());
    CHECK(j.at("accepted_batch").size()==batch.size());CHECK(j.at("control_edges").size()==28);
    CHECK(j.at("accepted_batch").size()>j.at("control_edges").size());
    REQUIRE(j.at("accepted_columns").size()==8);
    for(size_t i=0;i<batch.size();++i){CHECK(j.at("accepted_batch").at(i).at(2)==batch[i].measurement.range_m);CHECK(j.at("accepted_batch").at(i).at(5)==batch[i].innovation);}
    CHECK(j.at("qualified_information").at("minimum_count")>=2);
    CHECK(j.at("qualified_information").at("robust_fim_min")>0);
    std::cout<<"TASK31_INFORMATION_STAGE_ZERO "<<j.dump()<<'\n';
}
