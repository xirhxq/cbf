#ifndef CBF2026_SHRUNK_SECTOR_HPP
#define CBF2026_SHRUNK_SECTOR_HPP
#include <cmath>
#include <algorithm>
#include <stdexcept>
inline json updateShrunkPointSector(GridWorld& grid,const Point& estimate,double heading,double eps,const json& params) {
 const double inner=params.at("inner-radius"),outer=params.at("outer-radius"),half=params.at("half-angle-deg").get<double>()*std::acos(-1.)/180.;
 if(!std::isfinite(estimate.x)||!std::isfinite(estimate.y)||!std::isfinite(heading)||!std::isfinite(eps)||eps<0||!std::isfinite(inner)||!std::isfinite(outer)||inner<0||outer<=inner||half<=0||half>std::acos(-1.))throw std::invalid_argument("invalid shrunk-sector snapshot");
 json added=json::array();if(eps>=outer)return added;
 const int x0=grid.getNumInXLim(estimate.x-outer,"ceil"),x1=grid.getNumInXLim(estimate.x+outer,"floor");const int y0=grid.getNumInYLim(estimate.y-outer,"ceil"),y1=grid.getNumInYLim(estimate.y+outer,"floor");
 for(int x=x0;x<=x1;++x)for(int y=y0;y<=y1;++y){const Point q=grid.getPointInArea(x,y);const double dx=q.x-estimate.x,dy=q.y-estimate.y,d=std::hypot(dx,dy);
  if(d+eps>outer+1e-12 || (inner>1e-12 && d-eps<inner-1e-12))continue;
  bool angular=half>=std::acos(-1.)-1e-12;
  if(!angular && d>eps){const double off=std::abs(std::atan2(std::sin(std::atan2(dy,dx)-heading),std::cos(std::atan2(dy,dx)-heading)));angular=off+std::asin(std::min(1.,eps/d))<=half+1e-12;}
  if(!angular)continue;if(!grid.getValue(x,y))added.push_back({x,y});grid.setValue(x,y,true);
 }
 return added;
}
#endif
