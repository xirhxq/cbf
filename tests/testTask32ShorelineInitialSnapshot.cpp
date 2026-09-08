#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("Export shoreline initializer and exact first control cycle without changing the runner") {
    std::ifstream input("docs/evidence/task32-fixed-library/shoreline-left-r1-anchor-asset.json");
    REQUIRE(input.good()); nlohmann::json asset; input>>asset;
    const auto scene=gf::task31AnchorScene(asset); REQUIRE(scene.valid);
    const std::vector<unsigned> gaussian={2027};
    const std::vector<unsigned> link={134001};
    for(std::size_t k=0;k<gaussian.size();++k) for(int mode:{0,12}) {
        // No new noisy H0 advance after its formal information counterexample.
        if(k!=0&&mode==0)continue;
        auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
        for(auto& p:s.mobile_positions)p.x()+=750;
        if(mode==12)s.initial_topology=scene.goal.reference_edges;
        auto cfg=gf::task19ProductionAdapterConfig();
        cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
        cfg.distance_range_availability={true,link[k]};cfg.range_noise_std_m=k?.5:0.;
        cfg.range_dropout_probability=0.;cfg.range_random_seed=gaussian[k];
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);
        REQUIRE(x.adapter.initializeStageZero().initialized);
        gf::Task26ExternalReconstructor observer(x.adapter,x.controller,
            "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",
            60,1,false,true,false,scene,true,mode);
        const auto snapshot=[&]() {
            const auto state=x.adapter.runtimeSnapshot();nlohmann::json owners=nlohmann::json::array(),cov=nlohmann::json::array();
            for(Eigen::Index i=0;i<state.estimate.covariance.rows();++i) {
                nlohmann::json row=nlohmann::json::array();
                for(Eigen::Index j=0;j<state.estimate.covariance.cols();++j)row.push_back(state.estimate.covariance(i,j));
                cov.push_back(row);
            }
            for(std::size_t i=0;i<s.mobile_ids.size();++i) {
                const auto id=s.mobile_ids[i];const Robot* robot=nullptr;
                for(const auto& r:x.swarm.robots)if(r->id==static_cast<int>(id))robot=r.get();
                REQUIRE(robot!=nullptr);const Eigen::Index off=4*static_cast<Eigen::Index>(i);
                owners.push_back({{"id",id},{"truth",{robot->model->getStateVariable("x"),robot->model->getStateVariable("y"),
                    robot->model->getStateVariable("vx"),robot->model->getStateVariable("vy")}},
                    {"yaw",robot->model->getStateVariable("yawRad")},
                    {"est",{state.estimate.mean(off),state.estimate.mean(off+1),state.estimate.mean(off+2),state.estimate.mean(off+3)}}});
            }
            return nlohmann::json({{"runtime_s",state.runtime_s},{"owners",owners},{"covariance",cov},
                {"information",gf::task31InformationTelemetry(x.adapter)}});
        };
        const auto initial=snapshot();observer.beforeStep();const auto step=x.controller.advance();REQUIRE(step.step.advanced);
        const auto first=snapshot();CHECK(initial["runtime_s"]==0.);CHECK(first["runtime_s"]==.1);
        std::cout<<"SHORELINE_INITIAL_SNAPSHOT "<<nlohmann::json({{"mode",mode},{"gaussian_seed",gaussian[k]},
            {"link_seed",link[k]},{"sigma",cfg.range_noise_std_m},{"initial",initial},{"first",first}}).dump()<<'\n';
    }
}
