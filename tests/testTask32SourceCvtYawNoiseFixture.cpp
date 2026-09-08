#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/Task32SourceCvtYaw.hpp"
#include <fstream>

TEST_CASE("source yaw noisy development complete14 uses estimated-state yaw and all safety information gates") {
    std::ifstream f("docs/evidence/task32-fixed-library/observation-r1-anchor-asset.json");
    REQUIRE(f.good());nlohmann::json asset;f>>asset;const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
    const std::vector<unsigned> gaussian={133021,133037,133053}, link={134021,134037,134053};
    for(std::size_t sample=0;sample<gaussian.size();++sample) for(int variant=0;variant<3;++variant) {
        auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;s.initial_topology=scene.goal.reference_edges;
        for(std::size_t k=0;k<s.mobile_positions.size();++k)s.mobile_positions[k]+=Eigen::Vector2d(750+variant*std::sin(k),variant*std::cos(k));
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
        cfg.task18_yaw_objective=static_cast<gf::Task18YawObjective>(5);
        cfg.distance_range_availability={true,link[sample]};cfg.range_noise_std_m=.5;cfg.range_dropout_probability=0;cfg.range_random_seed=gaussian[sample];
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        settings["initial"]["velocity"]["values"]=nlohmann::json::array();
        for(auto id:s.mobile_ids)settings["initial"]["velocity"]["values"].push_back({variant*std::cos(id),variant*std::sin(id)});
        gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);REQUIRE(x.adapter.initializeStageZero().initialized);
        gf::Task26ExternalReconstructor observer(x.adapter,x.controller,
            "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,false,scene,true,12);
        double min_fim=1e30,max_post=0,max_yaw_error=0,max_slack=0;
        for(int tick=0;tick<100;++tick) {
            const auto pre=x.adapter.runtimeSnapshot();std::map<gf::NodeId,double> yaw;std::map<gf::NodeId,Eigen::Vector2d> velocity;
            for(const auto& robot:x.swarm.robots) {
                yaw[robot->id]=robot->model->getStateVariable("yawRad");
                velocity[robot->id]=robot->model->getVelocity().head<2>();
            }
            observer.beforeStep();const auto step=x.controller.advance();REQUIRE(step.step.advanced);
            CHECK(step.step.minimum_hard_residual>=-1e-9);
            const auto state=x.adapter.runtimeSnapshot();auto positions=scene.fixed;
            for(std::size_t k=0;k<s.mobile_ids.size();++k) {
                const auto id=s.mobile_ids[k];const Eigen::Vector4d est=pre.estimate.mean.segment<4>(4*k);
                const auto expected=gf::task32SourceCvtYaw(est.head<2>(),est.tail<2>(),yaw.at(id),step.applied_target_centers.at(id),1);
                REQUIRE(expected.valid);const auto actual=step.step.applied_yaw_rates_radps.at(id);
                CHECK(actual==doctest::Approx(expected.rate).epsilon(1e-10));
                max_yaw_error=std::max(max_yaw_error,std::abs(actual-expected.rate));max_slack=std::max(max_slack,expected.slack);
                const auto u=step.step.applied_controls.at(id);CHECK(u.cwiseAbs().maxCoeff()<=4+1e-6);
                CHECK(gf::auditPlantSpeedExactZoh(velocity.at(id),u,.1,30,1e-9).maximum_interval_speed_mps<=30+1e-9);
            }
            for(const auto& robot:x.swarm.robots) {
                positions[robot->id]={robot->model->getStateVariable("x"),robot->model->getStateVariable("y")};
                const auto expected=gf::wrapYawRad(yaw.at(robot->id)+.1*step.step.applied_yaw_rates_radps.at(robot->id));
                CHECK(std::abs(gf::wrapYawRad(robot->model->getStateVariable("yawRad")-expected))<1e-12);
            }
            for(const auto& edge:state.topology)CHECK((positions.at(edge.owner)-positions.at(edge.reference)).norm()<850);
            for(const auto& [id,p]:positions)if(id<=14)for(const auto& [other,q]:positions)if(other>id)CHECK((p-q).norm()>10);
            for(const auto& row:x.adapter.currentSnapshotHardRows(state.topology)) {
                if(row.kind==gf::CanonicalHardRowKind::ReferenceDistance||row.kind==gf::CanonicalHardRowKind::Collision)CHECK(row.barrier_h>0);
                CHECK(row.kind!=gf::CanonicalHardRowKind::Workspace);
            }
            const auto info=gf::task31InformationTelemetry(x.adapter);const double fim=info["qualified_information"]["robust_fim_min"],post=info["posterior_max_m2"];
            CHECK(fim>=1e-6);CHECK(post<=.1);CHECK(info["aoi_margin_min_s"].get<double>()>=0);
            min_fim=std::min(min_fim,fim);max_post=std::max(max_post,post);
            CHECK(observer.telemetry()["active_mode"]==12);CHECK(observer.telemetry()["request_count"]==0);
            REQUIRE(x.controller.committedTargets().size()==14);
            for(const auto& [id,c]:x.controller.committedTargets()){CHECK(c.x_index>=0);CHECK(c.x_index<450);CHECK(c.y_index>=0);CHECK(c.y_index<225);}
        }
        std::cout<<"SOURCE_YAW_NOISE14_FIXTURE "<<nlohmann::json({{"variant",variant},{"ticks",100},{"minimum_formal_fim",min_fim},
            {"maximum_posterior",max_post},{"maximum_yaw_dispatch_error",max_yaw_error},{"maximum_slack",max_slack},
            {"gaussian_seed",gaussian[sample]},{"link_seed",link[sample]},{"noise_std",.5}}).dump()<<'\n';
    }
}
