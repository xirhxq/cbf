#pragma once
#include "grand_finale/Task31TriangularLattice.hpp"

namespace gf {

struct Task31CommonBridge {
    bool valid=false;
    std::string reason;
    double spacing_m=0;
    Task31Lattice lattice;
    std::vector<DirectedEdge> geometry_edges;
    std::map<NodeId,Eigen::Vector2d> targets;
};

// Request-time geometric asset only. Mobile-connected components of the
// old/new union share one temporary front; no union graph is installed.
// Slots and rows use the same generator as the final search lattice.
// The one dimensional rule d = nearest fixed-anchor separation / 3 is an
// offline design choice, not an information or actual reach certificate.
inline Task31CommonBridge task31CommonBridge(const Task20DagLatticeContract& old,
    const Task20DagLatticeContract& goal,const std::map<NodeId,Eigen::Vector2d>& fixed,
    const Eigen::Vector2d& direction,double spacing_m=0,double anchor_ranking_span_m=0,
    std::optional<Eigen::Vector2d> frame_origin=std::nullopt) {
    Task31CommonBridge out;
    auto reject=[&](const std::string& reason){out.reason=reason;return out;};
    if(!old.valid||!goal.valid||fixed.size()<2||!direction.allFinite()||direction.norm()<1e-12||
        !std::isfinite(spacing_m)||spacing_m<0||(frame_origin&&!frame_origin->allFinite()))
        return reject("invalid_bridge_inputs");
    std::set<NodeId> members,old_members;std::vector<NodeId> order;
    for(const auto& [id,r]:old.member_roles)old_members.insert(id);
    for(const auto& unit:goal.coverage_units)for(auto id:unit.members) {
        if(!members.insert(id).second)return reject("duplicate_member");
        order.push_back(id);
    }
    if(members!=old_members||members.size()!=goal.member_roles.size())return reject("different_member_identities");
    std::set<std::pair<NodeId,NodeId>> edges;
    for(const auto& contract:{old,goal})for(const auto& e:contract.reference_edges)edges.emplace(e.reference,e.owner);
    std::map<NodeId,std::vector<NodeId>> adjacent;
    for(const auto& [a,b]:edges) {
        if(!members.count(b)||(!members.count(a)&&!fixed.count(a)))return reject("unknown_edge_node");
        out.geometry_edges.push_back({a,b});
        if(members.count(a)){adjacent[a].push_back(b);adjacent[b].push_back(a);}
    }
    Task20DagLatticeContract temporary=goal;
    temporary.id="common-union-bridge";temporary.coverage_units.clear();temporary.member_roles.clear();
    temporary.reference_edges=out.geometry_edges;
    std::set<NodeId> assigned;
    for(auto root:order) {
        if(assigned.count(root))continue;
        std::set<NodeId> component{root};std::vector<NodeId> queue{root};
        for(size_t k=0;k<queue.size();++k)for(auto id:adjacent[queue[k]])if(component.insert(id).second)queue.push_back(id);
        Task20CoverageUnit unit;unit.id="bridge-"+std::to_string(temporary.coverage_units.size());
        std::set<NodeId> anchors;
        for(auto id:order)if(component.count(id)) {
            assigned.insert(id);unit.members.push_back(id);
            auto role=goal.member_roles.at(id);role.coverage_unit=unit.id;temporary.member_roles[id]=role;
        }
        for(const auto& [a,b]:edges)if(component.count(b)&&fixed.count(a))anchors.insert(a);
        unit.base_anchors.assign(anchors.begin(),anchors.end());
        unit.frame_origin=frame_origin;
        if(unit.base_anchors.size()<2)return reject("insufficient_component_anchors");
        unit.leader=unit.members.back(); // Identity metadata only; no task is selected here.
        temporary.coverage_units.push_back(unit);
    }
    out.spacing_m=spacing_m>0?spacing_m:std::numeric_limits<double>::infinity();
    for(auto a=fixed.begin();a!=fixed.end();++a) {
        if(!a->second.allFinite())return reject("nonfinite_anchor");
        if(spacing_m==0)for(auto b=std::next(a);b!=fixed.end();++b)out.spacing_m=std::min(out.spacing_m,(a->second-b->second).norm()/3.0);
    }
    if(!std::isfinite(out.spacing_m)||out.spacing_m<=1e-12)return reject("degenerate_anchor_spacing");
    out.lattice=task31TriangularLattice(temporary,fixed,direction,anchor_ranking_span_m);
    if(!out.lattice.valid)return reject(out.lattice.reason);
    constexpr double h=0.86602540378443864676;
    const Eigen::Vector2d e=direction.normalized(),n(-e.y(),e.x());
    for(const auto& unit:out.lattice.contract.coverage_units) {
        Eigen::Vector2d b=Eigen::Vector2d::Zero();for(auto id:unit.base_anchors)b+=fixed.at(id);b/=unit.base_anchors.size();
        if(unit.frame_origin)b=*unit.frame_origin;
        for(auto id:unit.members) {
            const auto cell=out.lattice.cells.at(id);
            out.targets[id]=b+h*out.spacing_m*cell.row*e+out.spacing_m*cell.slot*n;
        }
    }
    out.valid=true;out.reason="geometry_only_fresh_actual_qualification_required";return out;
}

} // namespace gf
