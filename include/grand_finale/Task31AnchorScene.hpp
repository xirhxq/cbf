#pragma once
#include "grand_finale/Task31CommonBridge.hpp"
#include "grand_finale/Task25P0MultiDag.hpp"
#include "grand_finale/Task29MovingCompletion.hpp"
#include "grand_finale/Task31TerminalPort.hpp"

namespace gf {

// Frozen offline asset. The physical map is installed once, before the EKF.
// This is a parser/contract check, not an actual transition certificate.
struct Task31AnchorScene {
    bool valid=false;
    std::string reason;
    int mode_code=12;
    double width_m=0,height_m=0,bridge_spacing_m=0,ranking_span_m=0;
    double front_similarity_gain=1.0;
    std::string front_port_binding="none";
    std::map<std::string,NodeId> terminal_ports;
    bool reparameterizedFinal() const {return front_similarity_gain!=1.0||front_port_binding!="none";}
    Eigen::Vector2d frame_origin=Eigen::Vector2d::Zero(),direction{0,1},final_front_offset=Eigen::Vector2d::Zero();
    std::map<NodeId,Eigen::Vector2d> fixed;
    Task20DagLatticeContract goal;
    Task20DagLatticeContract unscaled_goal;
    std::map<NodeId,Task31LatticeCell> cells;
    nlohmann::json identity;
};

inline Task31AnchorScene task31AnchorScene(const nlohmann::json& j) {
    Task31AnchorScene out;out.identity=j;
    auto reject=[&](const std::string& r){out.reason=r;out.valid=false;return out;};
    try {
        out.width_m=j.at("width_m");out.height_m=j.at("height_m");out.mode_code=j.at("mode_code");
        if(!std::isfinite(out.width_m)||!std::isfinite(out.height_m)||out.width_m<=0||out.height_m<=0||
            std::fmod(out.width_m,10.)!=0||std::fmod(out.height_m,10.)!=0)return reject("invalid_map");
        auto point=[](const auto& v){if(!v.is_array()||v.size()!=2)throw std::invalid_argument("point dimension");
            Eigen::Vector2d p(v[0].template get<double>(),v[1].template get<double>());
            if(!p.allFinite())throw std::invalid_argument("nonfinite point");return p;};
        for(auto it=j.at("physical_anchors").begin();it!=j.at("physical_anchors").end();++it) {
            std::size_t n=0;const auto id=std::stoi(it.key(),&n);
            if(n!=it.key().size()||(id>=1&&id<=14)||!out.fixed.emplace(id,point(it.value())).second)return reject("invalid_anchor_identity");
        }
        for(int k=0;k<3;++k)if(!out.fixed.count(100+k)||
            (out.fixed.at(100+k)-Eigen::Vector2d((.4+.1*k)*out.width_m,-50)).norm()>1e-9)return reject("legacy_anchors_changed");
        out.frame_origin=point(j.at("frame_origin"));out.final_front_offset=point(j.at("final_front_offset_m"));
        out.ranking_span_m=j.at("anchor_ranking_span_m");out.bridge_spacing_m=j.at("bridge_spacing_m");
        if(j.contains("direction"))out.direction=point(j.at("direction"));
        if(!std::isfinite(out.ranking_span_m)||!std::isfinite(out.bridge_spacing_m)||out.ranking_span_m<=0||
            out.bridge_spacing_m<=0||out.direction.norm()<1e-12)return reject("invalid_explicit_frame_scale");
        out.direction.normalize();
        out.goal=task25DagContractFromCode(out.mode_code);if(!out.goal.valid)return reject("unknown_mode");
        out.goal.fixed_anchor_ids.clear();for(const auto& [id,p]:out.fixed)out.goal.fixed_anchor_ids.push_back(id);
        out.goal.reference_edges.clear();
        for(const auto& e:j.at("reference_edges")) {
            if(!e.is_array()||e.size()!=2)return reject("invalid_edge_shape");
            out.goal.reference_edges.emplace_back(e[0].get<NodeId>(),e[1].get<NodeId>());
        }
        for(auto& u:out.goal.coverage_units)u.frame_origin=out.frame_origin;
        task20_lattice_detail::finish(out.goal);if(!out.goal.valid)return reject(out.goal.reason);
        out.front_similarity_gain=j.value("front_similarity_gain",1.0);
        out.front_port_binding=j.value("front_port_binding",std::string("none"));
        const auto observation=j.value("search_observation",std::string("legacy_front"));
        if(observation!="legacy_front"&&observation!="terminal_port")
            return reject("unknown_search_observation");
        if(observation=="terminal_port"&&out.front_port_binding!="positive_cross_terminal")
            return reject("search_observation_requires_registered_terminal_port");
        if(out.front_port_binding!="none"&&out.front_port_binding!="positive_cross_terminal")
            return reject("unknown_front_port_binding");
        if(out.front_port_binding!="none"&&out.front_similarity_gain!=1.0)
            return reject("terminal_port_and_aperture_are_separate_arms");
        const auto baseline=task31TriangularLattice(out.goal,out.fixed,out.direction,out.ranking_span_m);
        if(!baseline.valid)return reject(baseline.reason);
        out.unscaled_goal=baseline.contract;
        const auto generated=task31TriangularLattice(out.goal,out.fixed,out.direction,out.ranking_span_m,out.front_similarity_gain);
        if(!generated.valid)return reject(generated.reason);
        out.goal=generated.contract;out.cells=generated.cells;
        if(out.front_port_binding!="none") {
            const auto bound=task31PositiveTerminalPort(generated);
            if(!bound.valid)return reject(bound.reason);
            out.goal=bound.contract;out.terminal_ports=bound.ports;
        }
        if(observation=="terminal_port") {
            for(auto& unit:out.goal.coverage_units)
                unit.search_observation_members={out.terminal_ports.at(unit.id)};
            out.goal.id+="-port-observation";
            out.goal.structural_signature+=";search-observation=registered-port";
        }
        // Adopted 2026-09-18: the final finish() is unconditional so the
        // anchor-inclusive topological order is identical on every parse route
        // (the applied fixed-mode contract and the role-program instance).
        task20_lattice_detail::finish(out.goal);
        if(!out.goal.valid)return reject(out.goal.reason);
        out.goal.id+="-anchor-asset";out.valid=true;out.reason="parsed_geometry_not_actual_qualification";
        return out;
    } catch(const std::exception& e){return reject(std::string("invalid_anchor_asset:")+e.what());}
}

// One contract-derived two-dimensional displacement per unit, evaluated once
// at actual admission. No actual-state fit or independent member references.
inline std::map<std::string,Eigen::Vector2d> task31UnscaledContinuationRates(
    const Task31AnchorScene& scene,const std::map<NodeId,Eigen::Vector2d>& canonical_from) {
    if(!scene.valid||canonical_from.size()!=scene.goal.member_roles.size())
        throw std::invalid_argument("invalid canonical continuation scene");
    std::map<std::string,Eigen::Vector2d> fronts,result;
    for(const auto& unit:scene.unscaled_goal.coverage_units)
        fronts[unit.id]=scene.frame_origin+scene.final_front_offset;
    const auto to=task20LiftTargets(scene.unscaled_goal,scene.fixed,fronts);
    if(!to.valid)throw std::invalid_argument(to.reason);
    for(const auto& unit:scene.unscaled_goal.coverage_units) {
        Eigen::Matrix2d mean=Eigen::Matrix2d::Zero();
        Eigen::Vector2d displacement=Eigen::Vector2d::Zero();
        for(auto id:unit.members) {
            if(!canonical_from.count(id)||!canonical_from.at(id).allFinite())
                throw std::invalid_argument("invalid canonical member target");
            mean+=task29RoleMatrix(scene.unscaled_goal.member_roles.at(id));
            displacement+=to.targets.at(id)-canonical_from.at(id);
        }
        // The identical member-count factors cancel exactly.
        if(!mean.allFinite()||std::abs(mean.determinant())<1e-12)
            throw std::invalid_argument("singular canonical front mapping");
        result[unit.id]=mean.inverse()*displacement;
        if(!result.at(unit.id).allFinite())throw std::invalid_argument("nonfinite canonical rate");
    }
    return result;
}
} // namespace gf
