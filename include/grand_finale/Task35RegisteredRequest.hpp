#pragma once
#include "grand_finale/Task34RequestGeometry.hpp"
#include "grand_finale/Task35RegisteredArcPath.hpp"
namespace gf {
// Registration binding, not a new geometry search or dynamic certificate.
inline Task34RequestGeometry task35RegisteredRequest(const Task34RegisteredRoles& registry,
    const Task20DagLatticeContract& source,int target,const std::map<NodeId,Eigen::Vector2d>& applied,
    const std::map<std::string,Eigen::Vector2d>& canonical,const nlohmann::json& asset) {
    Task34RequestGeometry out;
    try {
        std::map<NodeId,NodeId> pi;
        for(const auto& item:asset.at("role_to_physical").items()) {
            const int role=std::stoi(item.key()),physical=item.value().get<int>();
            if(role<0||role>65535||physical<0||physical>65535)throw std::invalid_argument("registered_role_id_domain");
            pi.emplace(NodeId(role),NodeId(physical));
        }
        out.frozen_goal=registry.freeze(target,pi);const auto goal=registry.restore(out.frozen_goal);
        std::set<NodeId> ids;for(const auto& [id,r]:source.member_roles)ids.insert(id);
        const Task35RegisteredArcPath path(asset,ids,goal.fixed);
        if(applied!=path.start())throw std::invalid_argument("registered_path_applied_start_not_exact");
        auto edges=[](const nlohmann::json& data) {
            task34_request_detail::Edges result;
            for(const auto& e:data) {
                const int a=e.at(0).get<int>(),b=e.at(1).get<int>();
                if(a<0||a>65535||b<0||b>65535||!result.emplace(NodeId(a),NodeId(b)).second)
                    throw std::invalid_argument("registered_path_edge_domain");
            }return result;
        };
        const auto old=edges(asset.at("old_edges")),next=edges(asset.at("new_edges"));
        if(old!=task34_request_detail::edges(source)||next!=task34_request_detail::edges(goal.contract))
            throw std::invalid_argument("registered_path_graph_identity");
        auto union_edges=old;union_edges.insert(next.begin(),next.end());
        if(task34_request_detail::order(applied,union_edges,std::less<NodeId>{}).size()!=applied.size())
            throw std::invalid_argument("registered_path_union_cycle");
        const auto lifted=task20LiftTargets(goal.contract,goal.fixed,canonical);
        if(!lifted.valid||lifted.targets.size()!=applied.size())throw std::invalid_argument("registered_path_goal_domain");
        out.bridge=path.bridge();out.endpoint=path.goal();out.endpoint_fronts=canonical;
        for(const auto& [id,p]:lifted.targets)if((p-out.endpoint.at(id)).norm()>1e-8)
            throw std::invalid_argument("registered_path_goal_lifting_mismatch");
        auto static_geometry=[&](const auto& mobile,const auto& refs) {
            auto points=goal.fixed;points.insert(mobile.begin(),mobile.end());
            for(const auto& [a,b]:refs)if(!((points.at(a)-points.at(b)).norm()<850.))
                throw std::invalid_argument("registered_static_reference");
            for(auto i=mobile.begin();i!=mobile.end();++i) {
                for(auto j=std::next(i);j!=mobile.end();++j)if(!((i->second-j->second).norm()>10.))
                    throw std::invalid_argument("registered_static_separation");
                for(const auto& [id,p]:goal.fixed)if(!((i->second-p).norm()>10.))
                    throw std::invalid_argument("registered_static_fixed_separation");
            }
        };
        static_geometry(out.bridge,union_edges);static_geometry(out.endpoint,next);
        out.metadata={{"schema","task35-registered-request-v1"},{"case",asset.at("case")},
            {"start_exact",true},{"graphs_bound",true},{"goal_lifting_bound",true},
            {"static_geometry_only",true},{"runtime_information_certified",false},
            {"incoming_velocity_term_added",false},{"normal_search_changed",false}};
        out.valid=true;out.reason="registered_spatial_path_bound";
    }catch(const std::exception& e){out.valid=false;out.reason=e.what();}
    return out;
}
} // namespace gf
