#ifndef CBF_LOCAL_VIEWPOINT_HPP
#define CBF_LOCAL_VIEWPOINT_HPP
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace local_viewpoint {
struct P { double x,y; };
inline P add(P a,P b){return {a.x+b.x,a.y+b.y};}
inline P sub(P a,P b){return {a.x-b.x,a.y-b.y};}
inline P scale(P a,double s){return {a.x*s,a.y*s};}
inline double norm(P a){return std::hypot(a.x,a.y);}
struct Ref { P p; double margin; };
struct Box { double xmin,xmax,ymin,ymax; };
struct Pick { P point; int feasible,preferred; double angle_sin; std::string status; };
struct View { P location; bool shifted; bool in_sensor_range; bool in_domain; };
inline bool inside(P p,Box b){return p.x>=b.xmin-1e-9 && p.x<=b.xmax+1e-9 && p.y>=b.ymin-1e-9 && p.y<=b.ymax+1e-9;}
inline View view(P q,P b0,P b1,Box box,double range=850.,double radius=400.){
 P s=q;bool shifted=false;
 if(norm(sub(q,b0))>4*range || norm(sub(q,b1))>4*range){P v=sub(q,b1);double d=norm(v);if(!(d>0))throw std::invalid_argument("undefined viewpoint direction");s=sub(q,scale(v,300./d));shifted=true;}
 return {s,shifted,norm(sub(q,s))<=radius+1e-9,inside(s,box)};
}
inline Pick choose(P me,P guide,const std::vector<Ref>& refs,const std::vector<Ref>& others,Box box,double range=850.,double safe=10.,double step=25.){
 if(refs.size()!=2 || !std::isfinite(me.x) || !std::isfinite(me.y))throw std::invalid_argument("local viewpoint requires two finite assigned refs");
 const double pi=std::acos(-1.0),floor=std::sin(pi/12.0);std::vector<P> choices;choices.push_back(me);P d=sub(guide,me);double n=norm(d);if(n>1e-12)choices.push_back(add(me,scale(d,std::min(step,n)/n)));
 for(int k=0;k<16;++k){double a=2*pi*k/16.;choices.push_back({me.x+step*std::cos(a),me.y+step*std::sin(a)});}
 struct Candidate{P p;double quality,dist;};std::vector<Candidate> valid;
 for(P p:choices){
  if(!inside(p,box))continue;bool ok=true;
  for(const auto&r:refs)if(norm(sub(p,r.p))+r.margin>range+1e-9)ok=false;
  for(const auto&r:others)if(norm(sub(p,r.p))<safe+r.margin-1e-9)ok=false;
  if(!ok)continue;P u=sub(refs[0].p,p),v=sub(refs[1].p,p);double du=norm(u),dv=norm(v);if(du<1e-9||dv<1e-9)continue;double quality=std::abs(u.x*v.y-u.y*v.x)/(du*dv);valid.push_back({p,quality,norm(sub(p,guide))});
 }
 if(valid.empty())return {me,0,0,0.,"NO_FEASIBLE_LOCAL_CANDIDATE_HOLD"};
 int preferred=0;for(auto&v:valid)if(v.quality+1e-12>=floor)preferred++;
 int best=-1;for(std::size_t i=0;i<valid.size();++i){auto&c=valid[i];if(preferred && c.quality+1e-12<floor)continue;if(best<0){best=static_cast<int>(i);continue;}auto&b=valid[best];
  bool better=preferred ? c.dist<b.dist-1e-9 : (c.quality>b.quality+1e-12 || (std::abs(c.quality-b.quality)<=1e-12 && c.dist<b.dist-1e-9));
  if(better)best=static_cast<int>(i);
 }
 const auto&c=valid.at(best);return {c.p,static_cast<int>(valid.size()),preferred,c.quality,preferred?"LOCAL_CANDIDATE_SELECTED":"ANGLE_DEGRADED_LOCAL_CANDIDATE"};
}
}
#endif
