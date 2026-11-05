#include "Swarm.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char** argv){
  try{
    if(argc!=2)throw std::runtime_error("expected materialized work2 config");
    std::ifstream input(argv[1]);json cfg;input>>cfg;
    if(!cfg.contains("work2")||cfg.at("tvt-experiment").at("row-mode")!="symmetric-l1"
       ||cfg.at("cbfs").at("route1").at("joint-resolve").get<bool>())throw std::runtime_error("unexpected work2 protocol");
    Swarm swarm(cfg);
    if(cfg.value("cano-follower-adaptation",false)) {
      const std::string request=cfg.at("cano-target-request-path").get<std::string>();
      const std::string response=cfg.at("cano-target-response-path").get<std::string>();
      swarm.canoAfterEstimatedMap=[&swarm,request,response](std::uint64_t k) {
        const auto tmp=request+".tmp";{std::ofstream out(tmp);out<<json({{"frame",k}}).dump()<<"\n";}
        if(std::rename(tmp.c_str(),request.c_str())!=0)throw std::runtime_error("Cano request rename failed");
        const auto start=std::chrono::steady_clock::now();
        while(!swarm.estimatorFileExists(response)) {
          if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>60)throw std::runtime_error("Cano response timeout");
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::ifstream in(response);json packet;in>>packet;
        if(packet.at("frame")!=k||!packet.at("followers").is_array()||packet.at("followers").size()!=12)throw std::runtime_error("Cano response schema invalid");
        std::set<int> seen;
        for(const auto& target:packet.at("followers")) {
          const int id=target.at("id").get<int>();const auto epoch=target.at("plan_frame").get<std::uint64_t>();
          if(id<1||id>14||id==7||id==14||!seen.insert(id).second||epoch>k||epoch%50!=0||k-epoch>=50)throw std::runtime_error("Cano epoch invalid");
          swarm.robots.at(id-1)->canoFollowerTarget={{"frame",k},{"plan_frame",epoch},{"raw_m",target.at("raw_m")}};
        }
        std::remove(response.c_str());
      };
    }
    swarm.run();
    // Only true map is advanced to the executed terminal pose. No terminal EKF measurement is invented.
    swarm.updateGridWorld();
    json term={{"time_s",swarm.robots.front()->runtime},{"true_coverage",swarm.gridWorldGroundTruth.getPercentage()},
      {"true_valid_count",swarm.gridWorldGroundTruth.validCount},{"true_visited_indices",json::array()},
      {"estimated_coverage_last_measured",swarm.robots.front()->gridWorld.getPercentage()},{"robots",json::array()}};
    for(size_t j=0;j<swarm.gridWorldGroundTruth.vis.size();++j)
      if(swarm.gridWorldGroundTruth.valid[j]&&swarm.gridWorldGroundTruth.vis[j])term["true_visited_indices"].push_back(j);
    for(const auto& r:swarm.robots){const auto p=r->model->xy();const auto v=r->model->getWork2PhysicalVelocity();
      term["robots"].push_back({{"id",r->id},{"truth",{p.x,p.y}},{"physical_velocity",{v[0],v[1]}},{"yaw",r->model->getStateVariable("yawRad")}});}
    std::ofstream output(cfg.at("work2").at("terminal_path").get<std::string>());output<<term.dump()<<'\n';
    return 0;
  }catch(const std::exception& e){std::cerr<<"[WORK2_NATIVE_ERROR] "<<e.what()<<'\n';return 1;}
}
