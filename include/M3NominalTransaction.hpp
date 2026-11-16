#pragma once
#include <stdexcept>
#include <cstdint>
namespace shared_m3 {
// Validator returns a temporary full batch; it cannot mutate these robots.
// Validation/allocation finishes before the fixed-size, noexcept commit loop.
template<class Robots,class Validator>
void commitNominals(Robots& robots,std::uint64_t frame,Validator validate){
 auto values=validate();
 if(values.size()!=14||robots.size()!=14)throw std::runtime_error("M3 full14 nominal transaction");
 for(std::size_t i=0;i<14;++i)
  if(values[i].id!=static_cast<int>(i)+1||robots[i]->id!=values[i].id||robots[i]->estimatorSnapshotFrame!=frame)
   throw std::runtime_error("M3 current full14 IDs/frame before nominal commit");
 for(std::size_t i=0;i<14;++i){
  auto& robot=robots[i];robot->canoPcgNominal=values[i].value;
  robot->canoPcgNominalFrame=frame;robot->canoPcgNominalRecord.swap(values[i].record);
 }
}
}
