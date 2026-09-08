#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("port observation full14 startup preserves estimator and moving contract") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-terminal-port-r1.json");
    REQUIRE(f.good());nlohmann::json original;f>>original;
    for(int variant=0;variant<3;++variant) {
        Eigen::VectorXd initial_mean;Eigen::MatrixXd initial_covariance;
        for(bool enabled:{false,true}) {
            auto asset=original;if(enabled)asset["search_observation"]="terminal_port";
            const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
            auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
            s.initial_topology=scene.goal.reference_edges;
            for(std::size_t k=0;k<s.mobile_positions.size();++k)s.mobile_positions[k]+=Eigen::Vector2d(750+variant*std::sin(k),variant*std::cos(k));
            auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
            cfg.distance_range_availability={true,134001};cfg.range_noise_std_m=0;cfg.range_dropout_probability=0;cfg.range_random_seed=2027;
            auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
            settings["initial"]["velocity"]["values"]=nlohmann::json::array();
            for(auto id:s.mobile_ids)settings["initial"]["velocity"]["values"].push_back({variant*std::cos(id),variant*std::sin(id)});
            gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);REQUIRE(x.adapter.initializeStageZero().initialized);
            const auto before=x.adapter.runtimeSnapshot();
            if(!enabled){initial_mean=before.estimate.mean;initial_covariance=before.estimate.covariance;}
            else{CHECK(initial_mean==before.estimate.mean);CHECK(initial_covariance==before.estimate.covariance);}
            gf::Task26ExternalReconstructor observer(x.adapter,x.controller,
                "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",
                60,1,false,true,false,scene,true,12);
            // Registered observation is part of the mode contract, not a mutable side channel.
            auto changed=scene.goal;changed.coverage_units[0].search_observation_members={scene.goal.coverage_units[0].members.front()};
            CHECK_THROWS(x.controller.commitExternalCoverageMode(12,changed));
            double min_fim=1e30,max_posterior=0;
            for(int tick=0;tick<100;++tick) {
                std::map<gf::NodeId,Eigen::Vector2d> pre_velocity;
                for(const auto& robot:x.swarm.robots)
                    pre_velocity[static_cast<gf::NodeId>(robot->id)]=robot->model->getVelocity().head<2>();
                observer.beforeStep();const auto step=x.controller.advance();REQUIRE(step.step.advanced);
                CHECK(step.step.minimum_hard_residual>=-1e-9);
                const auto state=x.adapter.runtimeSnapshot();auto positions=scene.fixed;
                for(const auto& robot:x.swarm.robots) {
                    const auto id=static_cast<gf::NodeId>(robot->id);const auto control=step.step.applied_controls.at(id);
                    positions[id]={robot->model->getStateVariable("x"),robot->model->getStateVariable("y")};
                    CHECK(control.cwiseAbs().maxCoeff()<=4+1e-6);
                    const auto speed=gf::auditPlantSpeedExactZoh(pre_velocity.at(id),control,.1,30,1e-9);
                    CHECK(speed.maximum_interval_speed_mps<=30+1e-9);
                }
                for(const auto& edge:state.topology)CHECK((positions.at(edge.owner)-positions.at(edge.reference)).norm()<850);
                for(const auto& [id,p]:positions)if(id<=14)
                    for(const auto& [other,q]:positions)if(other>id)CHECK((p-q).norm()>10);
                for(const auto& row:x.adapter.currentSnapshotHardRows(state.topology)) {
                    if(row.kind==gf::CanonicalHardRowKind::ReferenceDistance||row.kind==gf::CanonicalHardRowKind::Collision)
                        CHECK(row.barrier_h>0);
                    CHECK(row.kind!=gf::CanonicalHardRowKind::Workspace);
                }
                const auto info=gf::task31InformationTelemetry(x.adapter);const auto ext=observer.telemetry();
                const double fim=info["qualified_information"]["robust_fim_min"],post=info["posterior_max_m2"];
                CHECK(fim>=1e-6);CHECK(post<=.1);CHECK(info["aoi_margin_min_s"].get<double>()>=0);
                min_fim=std::min(min_fim,fim);max_posterior=std::max(max_posterior,post);
                CHECK(ext["active_mode"]==12);CHECK(ext["stage"]=="search");CHECK(ext["request_count"]==0);
                if(enabled) {
                    CHECK(ext["task31"]["search_observation_members"]["P"]==nlohmann::json::array({scene.terminal_ports.at("P")}));
                    CHECK(ext["task31"]["moving_front_members"]["P"]==nlohmann::json::array({13,14}));
                }
                REQUIRE(x.controller.committedTargets().size()==14);
                for(const auto& [id,c]:x.controller.committedTargets()){CHECK(c.x_index>=0);CHECK(c.x_index<450);CHECK(c.y_index>=0);CHECK(c.y_index<225);}
            }
            std::cout<<"TASK32_OBSERVATION_FIXTURE "<<nlohmann::json({{"enabled",enabled},{"variant",variant},{"ticks",100},
                {"minimum_formal_fim",min_fim},{"maximum_posterior",max_posterior},{"zero_gaussian",true},{"link_seed",134001}}).dump()<<'\n';
        }
    }
}
