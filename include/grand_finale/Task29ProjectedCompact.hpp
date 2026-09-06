#pragma once

#include "grand_finale/Task20DagLatticeContract.hpp"

namespace gf {

struct Task29CompactProjection {
    bool valid=false;
    std::string reason,active_edge;
    double height_m=0,unconstrained_height_m=0,baseline_zero_height_m=0;
    double minimum_height_m=0,maximum_height_m=0,squared_displacement_m2=0;
    double maximum_supported_edge_m=0;
    double minimum_separation_m=std::numeric_limits<double>::infinity();
    std::size_t target_count=0;
    std::map<std::string,Eigen::Vector2d> fronts;
};

// One request-time scalar projection, not a flight certificate or executed union
// graph. The coordinator must still obtain every original fresh qualification.
inline Task29CompactProjection task29ProjectedCompact(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed,
    const std::map<std::string,Eigen::Vector2d>& zero_fronts,
    const Eigen::Vector2d& progress_axis,
    const std::map<NodeId,Eigen::Vector2d>& estimated_positions,
    const std::vector<DirectedEdge>& planning_edges,double reference_limit_m,
    double mobile_support_m,double separation_limit_m) {
    Task29CompactProjection out;
    out.reason="invalid_projection_input";
    if(!contract.valid||!progress_axis.allFinite()||std::abs(progress_axis.norm()-1)>1e-10||
        !std::isfinite(reference_limit_m)||reference_limit_m<=0||!std::isfinite(mobile_support_m)||mobile_support_m<0||
        !std::isfinite(separation_limit_m)||separation_limit_m<0||planning_edges.empty()||
        zero_fronts.size()!=contract.coverage_units.size())return out;
    for(const auto& [u,p]:zero_fronts)if(!p.allFinite())return out;
    for(const auto& u:contract.coverage_units) {
        if(u.base_anchors.empty())return out;
        for(auto id:u.members)if(!contract.member_roles.count(id))return out;
    }
    auto one=zero_fronts;for(auto& [u,p]:one)p+=progress_axis;
    const auto q0=task20LiftTargets(contract,fixed,zero_fronts),q1=task20LiftTargets(contract,fixed,one);
    if(!q0.valid||!q1.valid||estimated_positions.size()!=q0.targets.size())return out;
    std::map<NodeId,Eigen::Vector2d> offset=q0.targets,slope;
    long double numerator=0,denominator=0;
    for(const auto& [id,p]:q0.targets) {
        auto current=estimated_positions.find(id);
        if(current==estimated_positions.end()||!current->second.allFinite()||!p.allFinite()||!q1.targets.at(id).allFinite())return out;
        slope[id]=q1.targets.at(id)-p;
        numerator+=slope.at(id).dot(current->second-p);denominator+=slope.at(id).squaredNorm();
    }
    if(!(denominator>1e-20))return out;
    for(const auto& [id,p]:fixed){if(!p.allFinite()||offset.count(id))return out;offset[id]=p;slope[id]=Eigen::Vector2d::Zero();}
    long double lower=0,upper=std::numeric_limits<long double>::infinity();
    out.unconstrained_height_m=static_cast<double>(numerator/denominator);
    if(!std::isfinite(out.unconstrained_height_m))return out;
    out.reason="empty_reference_height_interval";
    for(const auto& e:planning_edges) {
        if(!offset.count(e.owner)||!offset.count(e.reference)||e.owner==e.reference)return out;
        const long double radius=reference_limit_m-mobile_support_m*((fixed.count(e.owner)?0:1)+(fixed.count(e.reference)?0:1));
        if(!(radius>0))return out;
        const Eigen::Vector2d a=offset.at(e.owner)-offset.at(e.reference),b=slope.at(e.owner)-slope.at(e.reference);
        const long double aa=b.squaredNorm(),ab=a.dot(b),cc=static_cast<long double>(a.squaredNorm())-radius*radius;
        if(aa<1e-24){if(cc>0)return out;continue;}
        const long double disc=ab*ab-aa*cc;if(disc<0)return out;
        const long double l=(-ab-std::sqrt(disc))/aa,h=(-ab+std::sqrt(disc))/aa;
        lower=std::max(lower,l);
        if(h<upper){upper=h;out.active_edge=e.id();}
        if(!(lower<upper))return out;
    }
    if(!std::isfinite(static_cast<double>(upper)))return out;
    out.minimum_height_m=std::nextafter(static_cast<double>(lower),static_cast<double>(upper));
    out.maximum_height_m=std::nextafter(static_cast<double>(upper),static_cast<double>(lower));
    if(out.minimum_height_m>out.maximum_height_m)return out;
    out.height_m=std::clamp(out.unconstrained_height_m,out.minimum_height_m,out.maximum_height_m);
    out.fronts=zero_fronts;for(auto& [u,p]:out.fronts)p+=out.height_m*progress_axis;
    const auto targets=task20LiftTargets(contract,fixed,out.fronts);
    if(!targets.valid)return out;
    auto all=targets.targets;all.insert(fixed.begin(),fixed.end());out.target_count=targets.targets.size();
    for(const auto& [id,p]:targets.targets) {
        out.squared_displacement_m2+=(p-estimated_positions.at(id)).squaredNorm();
        for(const auto& [other,q]:all)if(other!=id)
            out.minimum_separation_m=std::min(out.minimum_separation_m,(p-q).norm());
    }
    for(const auto& e:planning_edges)
        out.maximum_supported_edge_m=std::max(out.maximum_supported_edge_m,
            (all.at(e.owner)-all.at(e.reference)).norm()+mobile_support_m*((fixed.count(e.owner)?0:1)+(fixed.count(e.reference)?0:1)));
    if(!std::isfinite(out.squared_displacement_m2)||out.maximum_supported_edge_m>reference_limit_m+1e-8) {
        out.reason="projection_numerical_audit_failed";return out;
    }
    if(!(out.minimum_separation_m>separation_limit_m)) {out.reason="nominal_separation_failed";return out;}
    out.valid=true;out.reason="static_height_projected_actual_qualification_still_required";return out;
}

} // namespace gf
