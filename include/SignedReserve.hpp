#ifndef CBF_SIGNED_RESERVE_CANDIDATE_HPP
#define CBF_SIGNED_RESERVE_CANDIDATE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "nlohmann/json.hpp"

// Experimental exact-single-integrator certificate; never uses ground truth.
namespace signed_reserve {
using J=nlohmann::json;
using V=std::array<double,2>;
inline V add(V a,V b){return {a[0]+b[0],a[1]+b[1]};}
inline V sub(V a,V b){return {a[0]-b[0],a[1]-b[1]};}
inline V mul(double s,V a){return {s*a[0],s*a[1]};}
inline double dot(V a,V b){return a[0]*b[0]+a[1]*b[1];}
inline double norm(V a){return std::hypot(a[0],a[1]);}
inline double l1(V a){return std::abs(a[0])+std::abs(a[1]);}
inline J arr(V a){return J::array({a[0],a[1]});}
struct Box{V lo,hi;};
struct Face{V a;double b,gamma;};
struct Edge{int owner,peer;bool base,collision;V relative;double E,q,B,reserve;std::vector<Face> faces;};
struct Node{V mean;double eps;J formation;};
struct State{
 static constexpr double T=.1,U=25.,ALPHA=.1,D=10.,R=850.,CAP=5.,TOL=1e-7;
 std::vector<Box> boxes;
 std::vector<V> last_commands;
 std::vector<Edge> edges;
 std::map<std::pair<int,int>,V> normals;
 std::vector<J> prefixes;
 bool initialized=false,cap_ever=false;
 J frame=J::object();
 [[noreturn]] void fail(const std::string& why,const J& detail){frame["certificate_failure"]={{"reason",why},{"detail",detail}};throw std::runtime_error("signed-reserve certificate: "+why);}
 static double support(const Box& x,V a){return (a[0]>=0?a[0]*x.hi[0]:a[0]*x.lo[0])+(a[1]>=0?a[1]*x.hi[1]:a[1]*x.lo[1]);}
 static double faceGamma(const Box& x,const Box& y,V a,double b){return b-support(x,a)-support(y,mul(-1.,a));}
 static Box point(V p){return {p,p};}
 void begin(const std::vector<Node>& nodes,const std::vector<V>& bases,std::uint64_t index){
  const int n=static_cast<int>(nodes.size());
  frame={{"frame_index",index},{"version_complete",false},{"cap_m",CAP},{"cap_exceeded_nodes",J::array()},{"certificate_failure",nullptr}};
  if(initialized && (boxes.size()!=nodes.size()||last_commands.size()!=nodes.size()))fail("node-count-mismatch",J::object());
  std::vector<Box> next;next.reserve(n);
  for(int i=0;i<n;++i){
   const auto& x=nodes[i];if(!std::isfinite(x.eps)||x.eps<0||!std::isfinite(x.mean[0])||!std::isfinite(x.mean[1]))fail("invalid-SCI-snapshot",{{"node",i+1}});
   if(x.eps>CAP){cap_ever=true;frame["cap_exceeded_nodes"].push_back({{"node",i+1},{"epsilon",x.eps}});}
   Box measurement{{x.mean[0]-x.eps,x.mean[1]-x.eps},{x.mean[0]+x.eps,x.mean[1]+x.eps}};
   if(initialized){Box predicted{add(boxes[i].lo,mul(T,last_commands[i])),add(boxes[i].hi,mul(T,last_commands[i]))};
    for(int d=0;d<2;++d){measurement.lo[d]=std::max(measurement.lo[d],predicted.lo[d]);measurement.hi[d]=std::min(measurement.hi[d],predicted.hi[d]);if(measurement.lo[d]>measurement.hi[d])fail("empty-historical-box",{{"node",i+1},{"axis",d},{"predicted_lo",arr(predicted.lo)},{"predicted_hi",arr(predicted.hi)},{"measurement_lo",arr(V{x.mean[0]-x.eps,x.mean[1]-x.eps})},{"measurement_hi",arr(V{x.mean[0]+x.eps,x.mean[1]+x.eps})}});}
   }
   next.push_back(measurement);
  }
  boxes=next;initialized=true;edges.clear();prefixes.clear();
  for(int owner=0;owner<n;++owner){
   for(int peer=0;peer<owner;++peer)addEdge(owner,peer,false,true,nodes,bases);
   for(const auto& p:nodes[owner].formation.at("baseIds"))addEdge(owner,p.get<int>(),true,false,nodes,bases);
   for(const auto& p:nodes[owner].formation.at("anchorIds")){
    int peer=p.get<int>()-1;if(peer<0||peer>=owner)fail("owner-order-invalid",{{"owner",owner+1},{"peer",peer+1}});
    addEdge(owner,peer,false,false,nodes,bases);
   }
  }
  std::vector<V> zero(n,V{0.,0.});const J backup=evaluate(zero);frame["initial_zero_witness"]=backup;
  if(backup.at("min_F").get<double>() < -TOL || backup.at("min_G").get<double>() < -TOL)fail("zero-backup-invalid",backup);
  frame["cap_exceeded_ever"]=cap_ever;
  frame["version_complete"]=true;
 }
 void addEdge(int owner,int peer,bool base,bool collision,const std::vector<Node>& nodes,const std::vector<V>& bases){
  const auto& a=nodes.at(owner);if(base && (peer<0||peer>=static_cast<int>(bases.size())))fail("invalid-base",{{"peer",peer}});
  const V other=base?bases.at(peer):nodes.at(peer).mean;const Box ob=base?point(other):boxes.at(peer);const V r=sub(a.mean,other);const double E=a.eps+(base?0.:nodes.at(peer).eps);const double s=norm(r);const double B=base?U:2*U;
  if(collision&&s<E)fail("collision-s-below-E",{{"owner",owner+1},{"peer",peer+1},{"s",s},{"E",E}});
  const double q=collision?(s-E)*(s-E)-D*D:R*R-(s+E)*(s+E);
  Edge e{owner,peer,base,collision,r,E,q,B,2*E+(collision?0.:T*B),{}};
  const double bar=base?CAP:2*CAP;
  if(collision){
   V lo=sub(boxes[owner].lo,ob.hi),hi=sub(boxes[owner].hi,ob.lo),nearest{std::clamp(0.,lo[0],hi[0]),std::clamp(0.,lo[1],hi[1])};double d=norm(nearest);V normal{};bool legal=false;
   if(d>1e-12){normal=mul(1./d,nearest);legal=faceGamma(boxes[owner],ob,mul(-1.,normal),-(D+2*bar)*norm(normal))>=-TOL;}
   auto key=std::make_pair(owner,peer);
   if(!legal && normals.count(key)){normal=normals.at(key);legal=faceGamma(boxes[owner],ob,mul(-1.,normal),-(D+2*bar)*norm(normal))>=-TOL;}
   if(!legal)fail("collision-core-not-contained",{{"owner",owner+1},{"peer",peer+1},{"box_distance",d},{"threshold",D+2*bar}});
   normals[key]=normal;V face=mul(-1.,normal);double bound=-(D+2*bar)*norm(normal);e.faces.push_back({face,bound,faceGamma(boxes[owner],ob,face,bound)});
  }else{
   const double bound=(R-2*bar)*std::cos(std::acos(-1.)/32.);
   for(int m=0;m<32;++m){double theta=2*std::acos(-1.)*m/32.;V aFace{std::cos(theta),std::sin(theta)};double g=faceGamma(boxes[owner],ob,aFace,bound);if(g< -TOL)fail("reference-core-not-contained",{{"owner",owner+1},{"peer",peer+(base?0:1)},{"base",base},{"face",m},{"gamma",g}});e.faces.push_back({aFace,bound,g});}
  }
  edges.push_back(std::move(e));
 }
 J evaluate(const std::vector<V>& commands)const{
  double minF=std::numeric_limits<double>::infinity(),minG=minF;int fe=-1,ge=-1,gface=-1;
  for(std::size_t k=0;k<edges.size();++k){const auto&e=edges[k];V w=sub(commands.at(e.owner),e.base?V{0,0}:commands.at(e.peer));double sign=e.collision?1.:-1.;double F=sign*2*dot(e.relative,w)-e.reserve*l1(w)+ALPHA*e.q;if(F<minF){minF=F;fe=static_cast<int>(k);}for(std::size_t m=0;m<e.faces.size();++m){const auto&g=e.faces[m];double value=g.gamma-T*dot(g.a,w);if(value<minG){minG=value;ge=static_cast<int>(k);gface=static_cast<int>(m);}}}
  return {{"min_F",minF},{"min_F_edge",fe},{"min_G",minG},{"min_G_edge",ge},{"min_G_face",gface}};
 }
 J inputFor(int current,const std::vector<V>& commands)const{
  J ans=J::array();
  for(std::size_t k=0;k<edges.size();++k){const auto&e=edges[k];bool own=e.owner==current;bool future=!e.base&&e.peer==current&&e.owner>current;if(!own&&!future)continue;
   V neighbor=own&&!e.base?commands.at(e.peer):V{0,0};J faces=J::array();for(const auto& f:e.faces)faces.push_back({{"a",arr(f.a)},{"gamma",f.gamma}});
   ans.push_back({{"edge_index",k},{"owner",e.owner+1},{"peer",e.peer+(e.base?0:1)},{"base",e.base},{"collision",e.collision},{"own",own},{"relative",arr(e.relative)},{"E",e.E},{"q",e.q},{"B",e.B},{"reserve",e.reserve},{"neighbor_committed",arr(neighbor)},{"faces",faces}});
  }return ans;
 }
 void prefix(int current,const std::vector<V>& commands){J p=evaluate(commands);p["committed_through"]=current+1;prefixes.push_back(p);if(p.at("min_F").get<double>() < -TOL || p.at("min_G").get<double>() < -TOL)fail("prefix-backup-residual",p);}
 void commit(const std::vector<V>& commands){last_commands=commands;}
 J record(const std::vector<V>& commands){
  frame["cap_exceeded_ever"]=cap_ever;
  frame["boxes"]=J::array();for(const auto& b:boxes)frame["boxes"].push_back({{"lo",arr(b.lo)},{"hi",arr(b.hi)}});
  frame["edges"]=J::array();for(const auto& e:edges){J a={{"owner",e.owner+1},{"peer",e.peer+(e.base?0:1)},{"base",e.base},{"collision",e.collision},{"relative",arr(e.relative)},{"E",e.E},{"q",e.q},{"B",e.B},{"reserve",e.reserve},{"faces",J::array()}};for(const auto& f:e.faces)a["faces"].push_back({{"a",arr(f.a)},{"b",f.b},{"gamma",f.gamma}});frame["edges"].push_back(a);}
  frame["prefixes"]=prefixes;frame["mixed_commands"]=J::array();for(auto u:commands)frame["mixed_commands"].push_back(arr(u));return frame;
 }
};
}
#endif
