#pragma once
#include "grand_finale/Task20DagLatticeContract.hpp"

namespace gf {

struct Task32FormationEndpoint {
    bool valid=false;
    std::string reason;
    std::map<NodeId,Eigen::Vector2d> targets;
    double maximum_target_reference_m=0;
    double minimum_necessary_reference_slack_m=std::numeric_limits<double>::infinity();
};

// A registered formation reference, not a search target and not a runtime
// safety certificate. A passed triangle-inequality test is only necessary.
inline Task32FormationEndpoint task32FormationEndpoint(
    const Task20DagLatticeContract& goal,const std::map<NodeId,Eigen::Vector2d>& fixed,
    const std::map<std::string,Eigen::Vector2d>& registered_fronts,
    double reference_limit=850.0,double position_error_limit=180.0) {
    Task32FormationEndpoint out;
    if(!std::isfinite(reference_limit)||reference_limit<=0||
        !std::isfinite(position_error_limit)||position_error_limit<0) {
        out.reason="invalid_completion_limits";return out;
    }
    const auto lifted=task20LiftTargets(goal,fixed,registered_fronts);
    if(!lifted.valid){out.reason=lifted.reason;return out;}
    out.targets=lifted.targets;
    auto positions=fixed;
    for(const auto& [id,p]:out.targets) {
        if(positions.count(id)||!p.allFinite()) {out.reason="invalid_member_identity_or_position";return out;}
        positions[id]=p;
    }
    bool conflict=false;
    for(const auto& e:goal.reference_edges) {
        if(!out.targets.count(e.owner)||!positions.count(e.reference)||
            !positions.at(e.reference).allFinite()) {out.reason="missing_reference_endpoint";return out;}
        const double length=(positions.at(e.owner)-positions.at(e.reference)).norm();
        const int mobile_endpoints=1+int(out.targets.count(e.reference));
        const double slack=reference_limit+mobile_endpoints*position_error_limit-length;
        out.maximum_target_reference_m=std::max(out.maximum_target_reference_m,length);
        out.minimum_necessary_reference_slack_m=std::min(out.minimum_necessary_reference_slack_m,slack);
        conflict=conflict||slack<=0;
    }
    out.valid=!conflict;
    out.reason=conflict?"known_shape_reference_conflict":"necessary_conditions_only_not_runtime_qualified";
    return out;
}

struct Task32FormationEndpointSelection {
    Task32FormationEndpoint endpoint;
    std::string selected_source;
    std::string proposed_reason;
};

inline Task32FormationEndpointSelection task32SelectFormationEndpoint(
    const Task20DagLatticeContract& goal,const std::map<NodeId,Eigen::Vector2d>& fixed,
    const std::map<std::string,Eigen::Vector2d>& proposed_fronts,
    const std::map<std::string,Eigen::Vector2d>& registered_fronts,
    double reference_limit=850.0,double position_error_limit=180.0) {
    auto proposed=task32FormationEndpoint(goal,fixed,proposed_fronts,reference_limit,position_error_limit);
    if(!proposed.valid&&proposed.reason=="known_shape_reference_conflict") {
        auto alternative=task32FormationEndpoint(goal,fixed,registered_fronts,reference_limit,position_error_limit);
        return {alternative,alternative.valid?"registered_alternative":"rejected",proposed.reason};
    }
    return {proposed,proposed.valid?"proposed_endpoint":"rejected",proposed.reason};
}

struct Task32ReferenceLegalSelection {
    Task32FormationEndpoint endpoint;
    std::map<std::string,Eigen::Vector2d> fronts;
    std::string selected_source="rejected";
    std::string classification="unproved";
    std::string numerical_method="quadratic_partition_then_48_bisections_and_strict_relift";
    double adjustment_fraction=0;
    double minimum_target_separation_m=std::numeric_limits<double>::infinity();
    double reference_margin_m=0;
    double interval_lower=0,interval_upper=0;
};

// This selects a formation ENDPOINT, never a search cell or flight phase.
// Final reference geometry and every mobile/mobile or mobile/fixed pair are
// checked. The old necessary-only public seam above remains unchanged for v2.
// A static witness is not an information, transfer, or moving-v1 certificate.
inline Task32ReferenceLegalSelection task32SelectReferenceLegalEndpoint(
    const Task20DagLatticeContract& goal,const std::map<NodeId,Eigen::Vector2d>& fixed,
    const std::map<std::string,Eigen::Vector2d>& proposed_fronts,
    const std::map<std::string,Eigen::Vector2d>& registered_fronts,
    double reference_limit=850.0,double separation_limit=10.0) {
    Task32ReferenceLegalSelection out;
    auto reject=[&](const std::string& reason) {
        out.endpoint.valid=false;out.endpoint.reason=reason;return out;
    };
    if(!goal.valid||goal.reference_edges.empty()||!std::isfinite(reference_limit)||reference_limit<=0||
        !std::isfinite(separation_limit)||separation_limit<=0)
        return reject("invalid_static_endpoint_contract");
    for(const auto& [id,p]:fixed)if(!p.allFinite())return reject("nonfinite_fixed_anchor");
    const auto start=task32FormationEndpoint(goal,fixed,proposed_fronts,reference_limit,180.0);
    if(!start.valid&&start.reason!="known_shape_reference_conflict")return reject(start.reason);
    auto evaluate=[&](double t) {
        Task32ReferenceLegalSelection result;
        result.adjustment_fraction=t;
        for(const auto& [unit,p]:proposed_fronts)
            result.fronts[unit]=t==0?p:(1-t)*p+t*registered_fronts.at(unit);
        result.endpoint=task32FormationEndpoint(goal,fixed,result.fronts,reference_limit,180.0);
        auto positions=fixed;
        for(const auto& [id,p]:result.endpoint.targets)positions[id]=p;
        for(auto first=result.endpoint.targets.begin();first!=result.endpoint.targets.end();++first) {
            for(auto second=std::next(first);second!=result.endpoint.targets.end();++second)
                result.minimum_target_separation_m=std::min(result.minimum_target_separation_m,(first->second-second->second).norm());
            for(const auto& [id,p]:fixed)
                result.minimum_target_separation_m=std::min(result.minimum_target_separation_m,(first->second-p).norm());
        }
        result.reference_margin_m=reference_limit-result.endpoint.maximum_target_reference_m;
        result.endpoint.valid=result.endpoint.valid&&result.reference_margin_m>0&&
            std::isfinite(result.minimum_target_separation_m)&&result.minimum_target_separation_m>separation_limit;
        result.classification=result.endpoint.valid?"static_geometry_witness":"unproved";
        result.endpoint.reason=result.endpoint.valid?"static_geometry_only_not_runtime_qualified":"target_geometry_not_legal";
        result.selected_source=result.endpoint.valid?(t==0?"proposed_endpoint":"registered_segment_interior"):"rejected";
        return result;
    };
    auto unchanged=evaluate(0);
    if(unchanged.endpoint.valid)return unchanged;
    if(proposed_fronts.size()!=registered_fronts.size())return reject("front_domain_mismatch");
    for(const auto& [unit,p]:proposed_fronts)
        if(!registered_fronts.count(unit)||!registered_fronts.at(unit).allFinite())return reject("front_domain_mismatch");
    const auto end=task32FormationEndpoint(goal,fixed,registered_fronts,reference_limit,180.0);
    if(!end.valid&&end.reason!="known_shape_reference_conflict")return reject(end.reason);
    // Partition at ALL squared-distance roots. Separation may become invalid
    // and valid again even when both endpoint configurations are valid.
    auto p0=fixed,p1=fixed;
    for(const auto& [id,p]:start.targets)p0[id]=p;
    for(const auto& [id,p]:end.targets)p1[id]=p;
    std::vector<double> cuts{0,1};
    auto roots=[&](NodeId a,NodeId b,double limit) {
        const long double x=(long double)p0.at(a).x()-p0.at(b).x();
        const long double y=(long double)p0.at(a).y()-p0.at(b).y();
        const long double vx=((long double)p1.at(a).x()-p1.at(b).x())-x;
        const long double vy=((long double)p1.at(a).y()-p1.at(b).y())-y;
        const long double A=vx*vx+vy*vy,B=2*(x*vx+y*vy),C=x*x+y*y-(long double)limit*limit;
        auto add=[&](long double t){if(std::isfinite(t)&&t>0&&t<1)cuts.push_back(double(t));};
        if(A==0){if(B!=0)add(-C/B);return;}
        const long double discriminant=B*B-4*A*C;
        if(discriminant<0)return;
        const long double root=std::sqrt(discriminant);
        const long double z=-.5L*(B+std::copysign(root,B));
        if(z==0)add(-B/(2*A));else{add(z/A);add(C/z);}
    };
    for(const auto& e:goal.reference_edges)roots(e.reference,e.owner,reference_limit);
    for(auto first=start.targets.begin();first!=start.targets.end();++first) {
        for(auto second=std::next(first);second!=start.targets.end();++second)roots(first->first,second->first,separation_limit);
        for(const auto& [id,p]:fixed)roots(first->first,id,separation_limit);
    }
    std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
    for(std::size_t k=1;k<cuts.size();++k) {
        const double middle=cuts[k-1]+.5*(cuts[k]-cuts[k-1]);
        auto candidate=evaluate(middle);
        if(!candidate.endpoint.valid)continue;
        double low=cuts[k-1],high=middle;
        auto boundary=evaluate(low);
        if(boundary.endpoint.valid){candidate=boundary;high=low;}
        else for(int iteration=0;iteration<48;++iteration) {
            const double mid=low+.5*(high-low);
            if(mid==low||mid==high)break;
            auto trial=evaluate(mid);
            if(trial.endpoint.valid){high=mid;candidate=std::move(trial);}else low=mid;
        }
        // Re-lift and validate the returned floating-point fronts themselves.
        candidate=evaluate(high);
        if(!candidate.endpoint.valid)continue;
        candidate.interval_lower=cuts[k-1];candidate.interval_upper=cuts[k];
        return candidate;
    }
    return reject("no_strict_static_endpoint_in_registered_segment");
}

} // namespace gf
