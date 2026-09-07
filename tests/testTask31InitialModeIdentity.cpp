#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>

TEST_CASE("Frozen range model full initialization mean and covariance agree across launch DAGs") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-aperture-r1.json");
    REQUIRE(f.good());nlohmann::json j;f>>j;const auto scene=gf::task31AnchorScene(j);REQUIRE(scene.valid);
    const std::vector<std::pair<double,unsigned>> cases={{0.,2027},{.5,131021},{.5,131037},{.5,131053}};
    for(const auto& [sigma,seed]:cases) {
        Eigen::VectorXd mean;Eigen::MatrixXd covariance;nlohmann::json information;
        for(int mode:{0,12}) {
            auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=scene.fixed;
            for(auto& p:s.mobile_positions)p.x()+=750;
            if(mode==12)s.initial_topology=scene.goal.reference_edges;
            auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
            cfg.distance_range_availability={true,131211};cfg.range_noise_std_m=sigma;cfg.range_dropout_probability=0;cfg.range_random_seed=seed;
            auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
            gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);REQUIRE(x.adapter.initializeStageZero().initialized);
            const auto r=x.adapter.runtimeSnapshot();const auto info=gf::task31InformationTelemetry(x.adapter);
            REQUIRE(r.estimate.mean.size()==56);REQUIRE(r.estimate.covariance.rows()==56);REQUIRE(r.estimate.covariance.cols()==56);
            if(mode==0){mean=r.estimate.mean;covariance=r.estimate.covariance;information=info;}
            else {
                CHECK((mean-r.estimate.mean).cwiseAbs().maxCoeff()==0.);
                CHECK((covariance-r.estimate.covariance).cwiseAbs().maxCoeff()==0.);
                for(const auto* key:{"range_acquisition","accepted_batch","member_position_velocity_posterior_max_eigenvalues","qualified_information","posterior_max_m2"})CHECK(info[key]==information[key]);
            }
            CHECK(r.runtime_s==0.);
        }
        std::cout<<"TASK31_INITIAL_MODE_IDENTITY "<<nlohmann::json({{"sigma",sigma},{"seed",seed},{"link_seed",131211},{"mean_entries",56},{"covariance_entries",3136},{"plant_steps",0}}).dump()<<'\n';
    }
}
