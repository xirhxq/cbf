#ifndef CBF_TRIANGLE_5_HPP
#define CBF_TRIANGLE_5_HPP
#include "Triangle750.hpp"
inline Triangle750Result triangle5(const Point& outer,const Point& shared,const Point& goal) {
 const double dx=goal.x-outer.x,dy=goal.y-outer.y,len=std::hypot(dx,dy);
 if(!(len>=1e-9)||!std::isfinite(len))throw std::invalid_argument("five-node goal coincides with first base or is nonfinite");
 const double ex=dx/len,ey=dy/len,nx=-ey,ny=ex;
 const int side=(nx*(shared.x-outer.x)+ny*(shared.y-outer.y)>=0)?1:-1;
 const double original=std::min(300.,len/2.),first=(original+len/4.)/2.,step=(len-first)/2.;
 const double angle=60.-15.*std::clamp((len-2400.)/800.,0.,1.);
 const double h=step/2.*std::tan(angle*std::acos(-1.)/180.);
 auto on=[&](double d,double o){return Point(outer.x+d*ex+side*o*nx,outer.y+d*ey+side*o*ny);};
 const Point p=on(first,0.);const double R=850.,radial=R*R-h*h;
 const double lo=radial>=0?std::max(0.,step-std::sqrt(radial)):1.;
 const double hi=radial>=0?std::min(step/2.,std::sqrt(radial)):0.;
 const double qx=shared.x-p.x,qy=shared.y-p.y,along=qx*ex+qy*ey,across=side*(qx*nx+qy*ny);
 const double remain=R*R-(h-across)*(h-across);double offset;bool feasible=false;
 if(remain>=0.){const double delta=std::sqrt(std::max(0.,remain));const double low=std::max(lo,along-delta),high=std::min(hi,along+delta);if(low<=high+1e-9){offset=std::min(step/2.,std::max(0.,high));feasible=true;}}
 if(!feasible)offset=std::max(0.,std::min(step/2.,along));
 return {{{p,on(first+offset,h),on(first+step,0.),on(first+step+offset,h),goal,goal,goal}},step,first,angle,side,offset,feasible};
}
#endif
