#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>
#include <iostream>
#include <limits>

// Read-only initializer seam: no controller.advance(), no new trajectory.
int main(int argc,char** argv) {
    try {
        if(argc!=5)throw std::invalid_argument("usage: asset gaussian-key link-key yaw(2|5)");
        const auto key=[](const char* text) {
            const std::string s(text);std::size_t used=0;
            if(s.empty()||s.find_first_not_of("0123456789")!=std::string::npos)
                throw std::invalid_argument("key must be an unsigned decimal integer");
            const auto v=std::stoul(s,&used);
            if(used!=s.size()||v==0||v>std::numeric_limits<unsigned>::max())
                throw std::invalid_argument("key out of range");
            return static_cast<unsigned>(v);
        };
        const auto gaussian=key(argv[2]),link=key(argv[3]),yaw=key(argv[4]);
        if(yaw!=2&&yaw!=5)throw std::invalid_argument("unregistered yaw objective");
        std::ifstream f(argv[1]);if(!f.good())throw std::invalid_argument("missing asset");
        nlohmann::json asset;f>>asset;const auto scene=gf::task31AnchorScene(asset);
        if(!scene.valid)throw std::invalid_argument("invalid anchor scene");
        auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;
        s.fixed_positions=scene.fixed;s.initial_topology=scene.goal.reference_edges;
        for(auto& p:s.mobile_positions)p.x()+=750;
        auto cfg=gf::task19ProductionAdapterConfig();
        cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
        cfg.task18_yaw_objective=static_cast<gf::Task18YawObjective>(yaw);
        cfg.distance_range_availability={true,link};cfg.range_noise_std_m=.5;
        cfg.range_dropout_probability=0.;cfg.range_random_seed=gaussian;
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);
        if(!x.adapter.initializeStageZero().initialized)throw std::runtime_error("initialization failed");
        gf::Task26ExternalReconstructor observer(x.adapter,x.controller,
            "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",
            60,1,false,true,false,scene,true,12);
        const auto state=x.adapter.runtimeSnapshot();
        if(state.runtime_s!=0.)throw std::runtime_error("initializer unexpectedly advanced plant");
        nlohmann::json owners=nlohmann::json::array(),cov=nlohmann::json::array();
        for(Eigen::Index i=0;i<state.estimate.covariance.rows();++i) {
            nlohmann::json row=nlohmann::json::array();
            for(Eigen::Index j=0;j<state.estimate.covariance.cols();++j)row.push_back(state.estimate.covariance(i,j));
            cov.push_back(row);
        }
        for(std::size_t i=0;i<s.mobile_ids.size();++i) {
            const auto id=s.mobile_ids[i];const Robot* robot=nullptr;
            for(const auto& r:x.swarm.robots)if(r->id==static_cast<int>(id))robot=r.get();
            if(!robot)throw std::runtime_error("missing mobile");
            const Eigen::Index off=4*static_cast<Eigen::Index>(i);
            owners.push_back({{"id",id},{"truth",{robot->model->getStateVariable("x"),robot->model->getStateVariable("y"),
                robot->model->getStateVariable("vx"),robot->model->getStateVariable("vy")}},
                {"yaw",robot->model->getStateVariable("yawRad")},
                {"est",{state.estimate.mean(off),state.estimate.mean(off+1),state.estimate.mean(off+2),state.estimate.mean(off+3)}}});
        }
        std::cout<<"TASK32_FROZEN_INITIAL "<<nlohmann::json({{"gaussian_seed",gaussian},{"link_seed",link},{"yaw_objective",yaw},
            {"mode",12},{"sigma",.5},{"plant_cycles",0},{"initial",{{"runtime_s",state.runtime_s},{"owners",owners},
                {"covariance",cov},{"information",gf::task31InformationTelemetry(x.adapter)}}}}).dump()<<'\n';
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
