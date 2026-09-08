#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("frozen development fields preserve initialization and all14 port observation gates") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-terminal-port-r1.json");
    REQUIRE(f.good());nlohmann::json original;f>>original;
    const std::vector<unsigned> seeds={133021,133037,133053},links={134021,134037,134053};
    for(std::size_t key=0;key<seeds.size();++key)for(int variant=0;variant<3;++variant) {
        Eigen::VectorXd mean;Eigen::MatrixXd covariance;
        for(bool enabled:{false,true}) {
            auto asset=original;if(enabled)asset["search_observation"]="terminal_port";
            auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
            auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;
            s.fixed_positions=scene.fixed;s.initial_topology=scene.goal.reference_edges;
            for(std::size_t k=0;k<s.mobile_positions.size();++k)
                s.mobile_positions[k]+=Eigen::Vector2d(750+variant*std::sin(k),variant*std::cos(k));
            auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
            cfg.distance_range_availability={true,links[key]};cfg.range_noise_std_m=.5;
            cfg.range_dropout_probability=0;cfg.range_random_seed=seeds[key];
            auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
            settings["initial"]["velocity"]["values"]=nlohmann::json::array();
            for(auto id:s.mobile_ids)settings["initial"]["velocity"]["values"].push_back({variant*std::cos(id),variant*std::sin(id)});
            gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);REQUIRE(x.adapter.initializeStageZero().initialized);
            const auto initial=x.adapter.runtimeSnapshot();
            if(!enabled){mean=initial.estimate.mean;covariance=initial.estimate.covariance;}
            else{CHECK(mean==initial.estimate.mean);CHECK(covariance==initial.estimate.covariance);}
            gf::Task26ExternalReconstructor observer(x.adapter,x.controller,
                "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",60,1,false,true,false,scene,true,12);
            double minimum_fim=1e30,maximum_posterior=0;
            for(int tick=0;tick<100;++tick) {
                std::map<gf::NodeId,Eigen::Vector2d> before_velocity;
                for(const auto& robot:x.swarm.robots)before_velocity[static_cast<gf::NodeId>(robot->id)]=robot->model->getVelocity().head<2>();
                observer.beforeStep();auto step=x.controller.advance();REQUIRE(step.step.advanced);
                CHECK(step.step.minimum_hard_residual>=-1e-9);
                auto state=x.adapter.runtimeSnapshot();auto positions=scene.fixed;
                for(const auto& robot:x.swarm.robots) {
                    auto id=static_cast<gf::NodeId>(robot->id);auto control=step.step.applied_controls.at(id);
                    positions[id]={robot->model->getStateVariable("x"),robot->model->getStateVariable("y")};
                    CHECK(control.cwiseAbs().maxCoeff()<=4+1e-6);
                    CHECK(gf::auditPlantSpeedExactZoh(before_velocity.at(id),control,.1,30,1e-9).maximum_interval_speed_mps<=30+1e-9);
                }
                for(const auto& edge:state.topology)CHECK((positions.at(edge.owner)-positions.at(edge.reference)).norm()<850);
                for(const auto& [id,p]:positions)if(id<=14)for(const auto& [other,q]:positions)if(other>id)CHECK((p-q).norm()>10);
                for(const auto& row:x.adapter.currentSnapshotHardRows(state.topology)) {
                    if(row.kind==gf::CanonicalHardRowKind::ReferenceDistance||row.kind==gf::CanonicalHardRowKind::Collision)CHECK(row.barrier_h>0);
                    CHECK(row.kind!=gf::CanonicalHardRowKind::Workspace);
                }
                auto info=gf::task31InformationTelemetry(x.adapter);auto ext=observer.telemetry();
                double fim=info["qualified_information"]["robust_fim_min"],posterior=info["posterior_max_m2"];
                CHECK(fim>=1e-6);CHECK(posterior<=.1);CHECK(info["aoi_margin_min_s"].get<double>()>=0);
                minimum_fim=std::min(minimum_fim,fim);maximum_posterior=std::max(maximum_posterior,posterior);
                CHECK(ext["active_mode"]==12);CHECK(ext["stage"]=="search");CHECK(ext["request_count"]==0);
                if(enabled)CHECK(ext["task31"]["search_observation_members"]["P"]==nlohmann::json::array({scene.terminal_ports.at("P")}));
                REQUIRE(x.controller.committedTargets().size()==14);
                for(const auto& [id,c]:x.controller.committedTargets()){CHECK(c.x_index>=0);CHECK(c.x_index<450);CHECK(c.y_index>=0);CHECK(c.y_index<225);}
            }
            std::cout<<"TASK32_OBSERVATION_NOISE_FIXTURE "<<nlohmann::json({{"enabled",enabled},{"variant",variant},{"ticks",100},
                {"gaussian_seed",seeds[key]},{"link_seed",links[key]},{"sigma",.5},
                {"minimum_formal_fim",minimum_fim},{"maximum_posterior",maximum_posterior}}).dump()<<'\n';
        }
    }
}
