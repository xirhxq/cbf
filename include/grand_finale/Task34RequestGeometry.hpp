#pragma once
#include "grand_finale/Task34RegisteredRoles.hpp"
#include "grand_finale/Task32FormationEndpoint.hpp"

namespace gf {
struct Task34RequestGeometry {
    bool valid=false;
    std::string reason;
    nlohmann::json frozen_goal,metadata;
    std::map<NodeId,Eigen::Vector2d> bridge,endpoint;
    std::map<std::string,Eigen::Vector2d> endpoint_fronts;
};
namespace task34_request_detail {
using Points=std::map<NodeId,Eigen::Vector2d>;
using Edges=std::set<std::pair<NodeId,NodeId>>;
inline Edges edges(const Task20DagLatticeContract& contract) {
    Edges result;for(const auto& e:contract.reference_edges)result.emplace(e.reference,e.owner);return result;
}
template<class Less> inline std::vector<NodeId> order(const Points& nodes,const Edges& edges,Less less) {
    std::vector<NodeId> result;std::set<NodeId> done;
    while(result.size()<nodes.size()) {
        std::vector<NodeId> ready;
        for(const auto& [id,p]:nodes)if(!done.count(id)) {
            bool ok=true;for(const auto& [a,b]:edges)if(b==id&&nodes.count(a)&&!done.count(a))ok=false;
            if(ok)ready.push_back(id);
        }
        if(ready.empty())return {};
        std::sort(ready.begin(),ready.end(),less);result.push_back(ready.front());done.insert(ready.front());
    }
    return result;
}
struct Beam {int makes=0;double distance=0;std::vector<std::pair<NodeId,NodeId>> assigned;};
inline bool beamLess(const Beam& a,const Beam& b) {
    return std::tie(a.makes,a.distance,a.assigned)<std::tie(b.makes,b.distance,b.assigned);
}
inline std::vector<std::map<NodeId,NodeId>> mappings(const Points& from,const Points& goal,
    const Edges& old,const Edges& next) {
    std::set<std::map<NodeId,NodeId>> unique;
    for(int variant=0;variant<4;++variant) {
        auto less=[&](NodeId a,NodeId b) {
            if(variant==0)return a<b;if(variant==1)return a>b;
            const auto& p=from.at(a);const auto& q=from.at(b);
            return std::make_tuple(p.y(),variant==2?p.x():-p.x(),a)<
                std::make_tuple(q.y(),variant==2?q.x():-q.x(),b);
        };
        const auto sorted=order(from,old,less);if(sorted.size()!=from.size())continue;
        std::vector<Beam> beam(1);
        for(const auto physical:sorted) {
            std::vector<Beam> proposals;
            for(const auto& entry:beam) {
                const std::map<NodeId,NodeId> pi(entry.assigned.begin(),entry.assigned.end());
                for(const auto& [role,p]:goal)if(!pi.count(role)) {
                    bool ready=true;int makes=0;
                    for(const auto& [a,b]:next)if(b==role) {
                        if(goal.count(a)&&!pi.count(a)){ready=false;break;}
                        if(!old.count({pi.count(a)?pi.at(a):a,physical}))++makes;
                    }
                    if(!ready)continue;
                    auto candidate=entry;candidate.makes+=makes;
                    candidate.distance+=(from.at(physical)-p).norm();candidate.assigned.emplace_back(role,physical);
                    proposals.push_back(std::move(candidate));
                }
            }
            std::sort(proposals.begin(),proposals.end(),beamLess);
            if(proposals.size()>16)proposals.resize(16);beam=std::move(proposals);
        }
        for(const auto& entry:beam)if(entry.assigned.size()==from.size())
            unique.emplace(entry.assigned.begin(),entry.assigned.end());
    }
    auto united=old;united.insert(next.begin(),next.end());
    if(order(from,united,std::less<NodeId>{}).size()==from.size()) {
        std::map<NodeId,NodeId> identity;for(const auto& [id,p]:from)identity[id]=id;unique.insert(identity);
    }
    return {unique.begin(),unique.end()};
}
// All constraints are partitioned by their quadratic roots; fixed anchors do
// not scale. A strictly checked interior point is returned, never a boundary
// assumed legal because an interval endpoint was rounded.
inline std::optional<double> bridgeScale(const Points& bridge,const Points& fixed,
    const Eigen::Vector2d& origin,const Edges& edges,const std::map<NodeId,double>& supports) {
    auto a=fixed,b=fixed;for(const auto& [id,p]:bridge){a[id]=origin;b[id]=p;}
    auto radius=[&](NodeId id){return supports.count(id)?supports.at(id):0.;};
    struct Constraint {NodeId a,b;double limit;bool upper;};std::vector<Constraint> constraints;
    for(const auto& [i,j]:edges)constraints.push_back({i,j,850-radius(i)-radius(j),true});
    for(auto i=bridge.begin();i!=bridge.end();++i) {
        for(auto j=std::next(i);j!=bridge.end();++j)constraints.push_back({i->first,j->first,10+radius(i->first)+radius(j->first),false});
        for(const auto& [id,p]:fixed)constraints.push_back({i->first,id,10+radius(i->first)+radius(id),false});
    }
    std::vector<double> cuts{0,1};
    for(const auto& c:constraints) {
        if(c.limit<=0)return std::nullopt;
        const Eigen::Vector2d x=a.at(c.a)-a.at(c.b),v=b.at(c.a)-b.at(c.b)-x;
        const long double A=(long double)v.x()*v.x()+(long double)v.y()*v.y();
        const long double B=2*((long double)x.x()*v.x()+(long double)x.y()*v.y());
        const long double C=(long double)x.x()*x.x()+(long double)x.y()*x.y()-(long double)c.limit*c.limit;
        auto add=[&](long double t){if(std::isfinite(t)&&t>0&&t<1)cuts.push_back(double(t));};
        if(A==0){if(B!=0)add(-C/B);continue;}
        const auto disc=B*B-4*A*C;if(disc<0)continue;
        const auto z=-.5L*(B+std::copysign(std::sqrt(disc),B));
        if(z==0)add(-B/(2*A));else{add(z/A);add(C/z);}
    }
    auto valid=[&](double k) {
        for(const auto& c:constraints) {
            const Eigen::Vector2d p=a.at(c.a)+k*(b.at(c.a)-a.at(c.a));
            const Eigen::Vector2d q=a.at(c.b)+k*(b.at(c.b)-a.at(c.b));
            const double d=(p-q).norm();
            if(!std::isfinite(d)||(c.upper?!(d<c.limit):!(d>c.limit)))return false;
        }return true;
    };
    if(valid(1))return 1.;
    std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
    for(std::size_t i=cuts.size()-1;i>0;--i) {
        double lo=cuts[i-1]+.5*(cuts[i]-cuts[i-1]),hi=cuts[i];if(!valid(lo))continue;
        if(valid(hi))return hi;
        for(int j=0;j<48;++j){double mid=lo+.5*(hi-lo);if(mid==lo||mid==hi)break;if(valid(mid))lo=mid;else hi=mid;}
        if(valid(lo))return lo;
    }return std::nullopt;
}
} // namespace task34_request_detail
inline Task34RequestGeometry task34RequestGeometry(const Task34RegisteredRoles& registry,
    const Task20DagLatticeContract& source,int target,const std::map<NodeId,Eigen::Vector2d>& from,
    const std::map<std::string,Eigen::Vector2d>& canonical,const std::map<NodeId,double>& supports,
    const nlohmann::json& asset) {
    using namespace task34_request_detail;Task34RequestGeometry out;
    try {
        if(!source.valid||from.size()!=source.member_roles.size())throw std::invalid_argument("source_geometry_domain");
        std::map<NodeId,NodeId> identity;
        for(const auto& [id,role]:source.member_roles) {
            if(!from.count(id)||!from.at(id).allFinite()||!supports.count(id)||
                !std::isfinite(supports.at(id))||supports.at(id)<0)throw std::invalid_argument("source_geometry_support");
            identity[id]=id;
        }
        const auto abstract=registry.instantiate(target,identity);
        for(const auto& [id,r]:supports)if((!from.count(id)&&!abstract.fixed.count(id))||!std::isfinite(r)||r<0)
            throw std::invalid_argument("support_domain");
        std::map<std::string,Eigen::Vector2d> origins;
        for(const auto& unit:abstract.contract.coverage_units) {
            if(!unit.frame_origin)throw std::invalid_argument("registered_frame_missing");
            origins[unit.id]=*unit.frame_origin;
        }
        const auto selected=task32SelectReferenceLegalEndpoint(abstract.contract,abstract.fixed,canonical,origins);
        if(!selected.endpoint.valid)throw std::invalid_argument(selected.endpoint.reason);
        const auto old=edges(source),next=edges(abstract.contract);
        const auto candidates=mappings(from,selected.endpoint.targets,old,next);
        Eigen::Vector2d origin(asset.at("frame_origin")[0].get<double>(),asset.at("frame_origin")[1].get<double>());
        Eigen::Vector2d forward(asset.at("direction")[0].get<double>(),asset.at("direction")[1].get<double>());
        const double spacing=asset.at("bridge_spacing_m"),ranking=asset.at("anchor_ranking_span_m");
        if(!origin.allFinite()||!forward.allFinite()||forward.norm()==0||!std::isfinite(spacing)||spacing<=0||!std::isfinite(ranking)||ranking<=0)
            throw std::invalid_argument("invalid_registered_bridge_frame");
        forward.normalize();const Eigen::Vector2d cross(-forward.y(),forward.x());
        std::optional<std::tuple<int,double,std::map<NodeId,NodeId>>> best;
        std::size_t static_candidates=0;
        for(const auto& pi:candidates) {
            const auto goal=registry.instantiate(target,pi);auto united=old;const auto goal_edges=edges(goal.contract);
            united.insert(goal_edges.begin(),goal_edges.end());const auto sorted=order(from,united,std::less<NodeId>{});
            if(sorted.size()!=from.size())continue;
            const auto lifted=task20LiftTargets(goal.contract,goal.fixed,selected.fronts);if(!lifted.valid)continue;
            std::map<NodeId,int> depth;std::map<NodeId,double> lateral;std::map<int,std::vector<NodeId>> rows;
            std::map<NodeId,std::vector<NodeId>> parents;
            for(const auto& [id,p]:goal.fixed){depth[id]=0;lateral[id]=cross.dot(p-origin)/ranking;}
            for(const auto& [a,b]:united)parents[b].push_back(a);
            for(const auto id:sorted){int d=0;for(const auto p:parents.at(id))d=std::max(d,depth.at(p));depth[id]=d+1;rows[d+1].push_back(id);}
            Points bridge; nlohmann::json cells=nlohmann::json::object();
            for(auto& [row,ids]:rows) {
                auto key=[&](NodeId id) {
                    double mean=0;for(const auto p:parents.at(id))mean+=lateral.at(p);mean/=parents.at(id).size();
                    return std::make_tuple(std::round(mean/1e-10),std::round(cross.dot(lifted.targets.at(id)-origin)/spacing/1e-10),id);
                };
                std::sort(ids.begin(),ids.end(),[&](NodeId a,NodeId b){return key(a)<key(b);});
                const double parity=row%2==0?.5:0.,start=std::round(-.5*(ids.size()-1)-parity)+parity;
                for(std::size_t k=0;k<ids.size();++k){const auto id=ids[k];const double slot=start+k;lateral[id]=slot;
                    bridge[id]=origin+spacing*(std::sqrt(3.)/2*row*forward+slot*cross);
                    cells[std::to_string(id)]={{"row",row},{"slot",slot}};
                }
            }
            const auto scale=bridgeScale(bridge,goal.fixed,origin,united,supports);if(!scale)continue;
            ++static_candidates;double length=0;int makes=0;
            for(auto& [id,p]:bridge){p=origin+*scale*(p-origin);length+=(from.at(id)-p).norm()+(p-lifted.targets.at(id)).norm();}
            for(const auto& edge:goal_edges)if(!old.count(edge))++makes;
            const auto score=std::make_tuple(makes,length,pi);if(best&&!(score<*best))continue;best=score;
            out.valid=true;out.reason="static_request_geometry_not_runtime_qualified";out.frozen_goal=registry.freeze(target,pi);
            out.bridge=bridge;out.endpoint=lifted.targets;out.endpoint_fronts=selected.fronts;
            out.metadata={{"actual_qualified",false},{"information_qualified",false},{"bridge_scale",*scale},
                {"bridge_cells",cells},{"make_count",makes},{"nominal_two_segment_length_m",length},
                {"endpoint_adjustment_fraction",selected.adjustment_fraction},{"endpoint_reference_margin_m",selected.reference_margin_m},
                {"endpoint_minimum_separation_m",selected.minimum_target_separation_m},
                {"numerical_method","all_quadratic_roots_partition_48_bisections_strict_recheck"}};
        }
        if(!out.valid)throw std::invalid_argument("no_static_request_geometry_in_finite_family");
        out.metadata["mapping_candidates"]=candidates.size();out.metadata["static_candidates"]=static_candidates;
    } catch(const std::exception& error){out.valid=false;out.reason=error.what();}
    return out;
}
} // namespace gf
