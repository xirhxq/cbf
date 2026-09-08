#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/Task32SourceCvtYaw.hpp"
#include <fstream>

TEST_CASE("research source-yaw dispatch reaches actual yaw output for both outer policy families") {
    for(bool lattice:{false,true}) {
        auto s=gf::task10p11rFixedBaselineScenario();
        auto cfg=gf::task19ProductionAdapterConfig();
        cfg.task18_yaw_objective=static_cast<gf::Task18YawObjective>(5);
        cfg.target_policy_task18_cbf2026_outer=!lattice;cfg.target_policy_task20_dag_lattice=lattice;
        cfg.distance_range_availability={true,134001};cfg.range_noise_std_m=0.;cfg.range_dropout_probability=0.;
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:s.mobile_ids)settings["initial"]["velocity"]["values"].push_back({std::cos(id),std::sin(id)});
        gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);REQUIRE(x.adapter.initializeStageZero().initialized);
        const auto pre=x.adapter.runtimeSnapshot();std::map<gf::NodeId,double> yaw;
        for(const auto& robot:x.swarm.robots)yaw[robot->id]=robot->model->getStateVariable("yawRad");
        const auto step=x.controller.advance();REQUIRE(step.step.advanced);
        for(std::size_t k=0;k<s.mobile_ids.size();++k) {
            const auto id=s.mobile_ids[k];const Eigen::Vector4d state=pre.estimate.mean.segment<4>(4*k);
            const auto expected=gf::task32SourceCvtYaw(state.head<2>(),state.tail<2>(),yaw.at(id),
                step.applied_target_centers.at(id),cfg.maximum_yaw_rate_radps);
            REQUIRE(expected.valid);
            CHECK(step.step.applied_yaw_rates_radps.at(id)==doctest::Approx(expected.rate).epsilon(1e-10));
        }
    }
}
