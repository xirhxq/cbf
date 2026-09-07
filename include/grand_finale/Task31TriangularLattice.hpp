#pragma once

#include "grand_finale/Task20DagLatticeContract.hpp"

namespace gf {

struct Task31LatticeCell { int row=0; double slot=0; };
struct Task31Lattice {
    bool valid=false;
    std::string reason;
    Task20DagLatticeContract contract;
    std::map<NodeId,Task31LatticeCell> cells;
};

// Offline asset generator, not a safety or information certificate. Semantic
// member order resolves graph automorphisms; numerical IDs never break ties.
// Separate coverage units must have no inter-unit mobile reference. Coupled
// units require a joint offline construction, not an implicit zero slot.
// Rows are longest anchored DAG depths. Adjacent row grids differ by d/2;
// horizontal nearest slots have distance d, row height sqrt(3)*d/2.
// At runtime q=b+(row/R)v+slot/(sqrt(3)*R/2) Jv, so d=|v|/(sqrt(3)*R/2).
// The contract's base-anchor barycentre defines b, terminal row centre defines
// the shared front. Fixed anchors are not outputs and are never transformed.
inline Task31Lattice task31TriangularLattice(const Task20DagLatticeContract& input,
    const std::map<NodeId,Eigen::Vector2d>& fixed,const Eigen::Vector2d& direction,
    double anchor_ranking_span_m=0,double front_similarity_gain=1.0) {
    Task31Lattice out;out.contract=input;
    auto reject=[&](const std::string& reason){out.reason=reason;return out;};
    if(!input.valid||!direction.allFinite()||direction.norm()<1e-12||fixed.empty()||
        !std::isfinite(anchor_ranking_span_m)||anchor_ranking_span_m<0||
        !std::isfinite(front_similarity_gain)||front_similarity_gain<=0||front_similarity_gain>1)
        return reject("invalid_input_frame");
    const Eigen::Vector2d forward=direction.normalized(),cross(-forward.y(),forward.x());
    std::vector<NodeId> order;std::set<NodeId> mobiles;std::set<std::string> unit_ids;
    std::map<NodeId,std::vector<NodeId>> parents;
    std::map<NodeId,int> depth;
    for(const auto& [id,p]:fixed){if(!p.allFinite())return reject("nonfinite_anchor");depth[id]=0;}
    for(const auto& u:input.coverage_units) {
        if(u.members.empty()||u.base_anchors.empty()||!unit_ids.insert(u.id).second)
            return reject("invalid_unit");
        for(auto id:u.base_anchors)if(!fixed.count(id))return reject("unknown_anchor");
        for(auto id:u.members) {
            if(fixed.count(id)||!mobiles.insert(id).second||!input.member_roles.count(id)||
                input.member_roles.at(id).member!=id||input.member_roles.at(id).coverage_unit!=u.id)
                return reject("invalid_membership");
            order.push_back(id);
        }
    }
    if(mobiles.size()!=input.member_roles.size())return reject("incomplete_membership");
    std::set<std::pair<NodeId,NodeId>> seen;
    for(const auto& e:input.reference_edges) {
        if(!mobiles.count(e.owner)||(!mobiles.count(e.reference)&&!fixed.count(e.reference))||
            !seen.emplace(e.reference,e.owner).second)return reject("invalid_edge");
        if(mobiles.count(e.reference)&&input.member_roles.at(e.reference).coverage_unit!=
            input.member_roles.at(e.owner).coverage_unit)
            return reject("unsupported_inter_unit_reference");
        parents[e.owner].push_back(e.reference);
    }
    std::vector<NodeId> sorted;
    while(sorted.size()<order.size()) {
        bool progress=false;
        for(auto id:order) {
            if(depth.count(id))continue;
            if(parents[id].size()<2)return reject("insufficient_parents");
            bool ready=true;int d=0;
            for(auto p:parents[id]){if(!depth.count(p)){ready=false;break;}d=std::max(d,depth.at(p));}
            if(ready){depth[id]=d+1;sorted.push_back(id);progress=true;}
        }
        if(!progress)return reject("cyclic_or_unanchored_graph");
    }
    constexpr double h=0.86602540378443864676;
    for(const auto& unit:input.coverage_units) {
        Eigen::Vector2d base=Eigen::Vector2d::Zero();
        for(auto id:unit.base_anchors)base+=fixed.at(id);
        base/=unit.base_anchors.size();
        if(unit.frame_origin)base=*unit.frame_origin;
        if(!base.allFinite())return reject("nonfinite_frame_origin");
        double span=anchor_ranking_span_m;
        if(span==0)for(const auto& [id,p]:fixed)span=std::max(span,std::abs(cross.dot(p-base)));
        // This scale affects parent-order ranking only, not final d or safety.
        span=std::max(span,1.0);
        std::map<NodeId,double> lateral;
        for(const auto& [id,p]:fixed)lateral[id]=cross.dot(p-base)/span;
        std::map<int,std::vector<NodeId>> rows;int maximum=0;
        for(auto id:unit.members){rows[depth.at(id)].push_back(id);maximum=std::max(maximum,depth.at(id));}
        for(auto& [row,members]:rows) {
            std::map<NodeId,double> keys;
            for(auto id:members) {
                double sum=0;
                for(auto p:parents.at(id))sum+=lateral.at(p);
                // Quantize once, then compare strictly. Pairwise epsilon
                // comparisons do not form a strict weak ordering.
                const double key=std::round((sum/parents.at(id).size())/1e-10);
                if(!std::isfinite(key))return reject("nonfinite_parent_order_key");
                keys[id]=key;
            }
            std::stable_sort(members.begin(),members.end(),[&](auto a,auto b){return keys.at(a)<keys.at(b);});
            const double parity=(row%2==0)?0.5:0.0;
            const double start=std::round(-0.5*(members.size()-1)-parity)+parity;
            for(size_t k=0;k<members.size();++k) {
                const auto id=members[k];const double slot=start+k;
                out.cells[id]={row,slot};lateral[id]=slot;
                // task20 convention adds |t|/2 to the axial component.
                const double t=slot/(h*h*maximum);
                auto& role=out.contract.member_roles.at(id);
                role.triangular_fraction=t;
                role.axial_fraction=double(row)/maximum-0.5*std::abs(t);
                if(front_similarity_gain!=1.0) {
                    // q(g)=g+gain*(q_original(g)-g). Keep the exact old
                    // arithmetic when disabled; this is not base scaling.
                    role.axial_fraction=1.0+front_similarity_gain*(role.axial_fraction-1.0);
                    role.triangular_fraction*=front_similarity_gain;
                }
            }
        }
    }
    out.contract.id=input.id+"-triangular-v1";
    out.contract.structural_signature=input.structural_signature+";dag-depth-staggered-slots-v1";
    if(front_similarity_gain!=1.0)
        out.contract.structural_signature+=";front_similarity="+std::to_string(front_similarity_gain);
    out.contract.topological_order=sorted;
    out.valid=true;out.reason="offline_geometry_not_runtime_certificate";return out;
}

} // namespace gf
