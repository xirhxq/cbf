#include "Swarm.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char** argv){
  try{
    if(argc!=2)throw std::runtime_error("expected materialized work2 config");
    std::ifstream input(argv[1]);json cfg;input>>cfg;
    if(!cfg.contains("work2")||cfg.at("tvt-experiment").at("row-mode")!="gdms-min"
       ||cfg.at("cbfs").at("route1").at("joint-resolve").get<bool>())throw std::runtime_error("unexpected work2 protocol");
    Swarm swarm(cfg);swarm.run();
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
