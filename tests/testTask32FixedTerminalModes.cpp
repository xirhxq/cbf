#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("Final terminal mapping supports fixed full-member startup without estimator reset") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-terminal-port-r1.json");
    REQUIRE(f.good());
    nlohmann::json j; f >> j;
    const auto scene=gf::task31AnchorScene(j);
    REQUIRE(scene.valid);
    const std::vector<unsigned> gaussian={2027,133021,133037,133053};
    const std::vector<unsigned> link={134001,134021,134037,134053};
    for(int variant=0;variant<4;++variant) {
        Eigen::VectorXd mean; Eigen::MatrixXd covariance; nlohmann::json first_info;
        for(int mode:{0,12}) {
            auto s=gf::task10p11rFixedBaselineScenario();
            s.width_m=4500; s.height_m=2250; s.fixed_positions=scene.fixed;
            const int perturb=variant>1?variant-1:0;
            for(size_t k=0;k<s.mobile_positions.size();++k)
                s.mobile_positions[k]+=Eigen::Vector2d(750+perturb*std::sin(k),perturb*std::cos(k));
            if(mode==12) s.initial_topology=scene.goal.reference_edges;
            auto cfg=gf::task19ProductionAdapterConfig();
            cfg.target_policy_task18_cbf2026_outer=false;
            cfg.target_policy_task20_dag_lattice=true;
            cfg.distance_range_availability={true,link[variant]};
            cfg.range_noise_std_m=variant? .5:0.; cfg.range_dropout_probability=0.;
            cfg.range_random_seed=gaussian[variant];
            auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
            settings["initial"]["velocity"]["values"]=nlohmann::json::array();
            for(auto id:s.mobile_ids)
                settings["initial"]["velocity"]["values"].push_back({perturb*std::cos(id),perturb*std::sin(id)});
            gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);
            const auto init=x.adapter.initializeStageZero();
            INFO("variant="<<variant<<" mode="<<mode<<" reason="<<init.reason);
            REQUIRE(init.initialized);
            const auto initial=x.adapter.runtimeSnapshot();
            const auto info=gf::task31InformationTelemetry(x.adapter);
            if(mode==0) { mean=initial.estimate.mean; covariance=initial.estimate.covariance; first_info=info; }
            else {
                CHECK((mean-initial.estimate.mean).cwiseAbs().maxCoeff()==0.);
                CHECK((covariance-initial.estimate.covariance).cwiseAbs().maxCoeff()==0.);
                CHECK(info["range_acquisition"]==first_info["range_acquisition"]);
                CHECK(info["accepted_batch"]==first_info["accepted_batch"]);
            }
            gf::Task26ExternalReconstructor observer(x.adapter,x.controller,
                "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",
                60,1,false,true,false,scene,true,mode);
            CHECK(initial.estimator_token==x.adapter.runtimeSnapshot().estimator_token);
            for(int tick=0;tick<100;++tick) {
                observer.beforeStep();
                const auto step=x.controller.advance();
                INFO("tick="<<tick<<" reason="<<step.step.reason);
                REQUIRE(step.step.advanced);
                const auto health=gf::task31InformationTelemetry(x.adapter);
                CHECK(health["qualified_information"]["robust_fim_min"].get<double>()>=1e-6);
                CHECK(health["posterior_max_m2"].get<double>()<=.1);
                CHECK(health["aoi_margin_min_s"].get<double>()>=0.);
                const auto ext=observer.telemetry();
                CHECK(ext["active_mode"]==mode); CHECK(ext["stage"]=="search");
                CHECK(ext["request_count"]==0);
                REQUIRE(x.controller.committedTargets().size()==14);
                for(const auto& [id,c]:x.controller.committedTargets()) {
                    CHECK(c.x_index>=0); CHECK(c.x_index<450);
                    CHECK(c.y_index>=0); CHECK(c.y_index<225);
                }
                CHECK(gf::task25_detail::edgeSet(x.adapter.runtimeSnapshot().topology)==gf::task25_detail::edgeSet(s.initial_topology));
            }
            CHECK(observer.report()["planned_request_count"]==0);
            std::cout<<"TASK32_FIXED_FIXTURE "<<nlohmann::json({{"mode",mode},{"variant",variant},
                {"safe_ticks",100},{"gaussian_seed",gaussian[variant]},{"link_seed",link[variant]},
                {"sigma",cfg.range_noise_std_m},{"position_velocity_perturbation",perturb},
                {"initial_information",info["qualified_information"]},{"requests",0}}).dump()<<'\n';
        }
    }
}
