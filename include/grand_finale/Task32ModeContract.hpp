#pragma once
#include "grand_finale/Task32FixedCoverageAsset.hpp"

namespace gf {

// Pure versioned identity. No EKF, service oracle, dispatch or admission occurs.
struct Task32ModeContract {
    bool valid=false;
    std::string reason;
    int mode_code=-1;
    double width_m=0.,height_m=0.;
    std::map<NodeId,Eigen::Vector2d> fixed;
    Task20DagLatticeContract contract;
    std::string initialization_version,mapping_kind;
    nlohmann::json identity;
    bool runtime_qualified=false;
    bool strip_role_reassignment_pending=false;
};

namespace task32_mode_contract_detail {
inline bool fields(const nlohmann::json& j,std::initializer_list<const char*> names) {
    if(!j.is_object()||j.size()!=names.size())return false;
    for(const auto name:names)if(!j.contains(name))return false;
    return true;
}
inline nlohmann::json point(const Eigen::Vector2d& p) {
    return nlohmann::json::array({p.x(),p.y()});
}
inline void explicitFrames(Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed,const nlohmann::json& frames) {
    if(!frames.is_object()||frames.size()!=contract.coverage_units.size())
        throw std::invalid_argument("unit_frame_set_mismatch");
    for(auto& unit:contract.coverage_units) {
        Eigen::Vector2d origin=Eigen::Vector2d::Zero();
        for(const auto id:unit.base_anchors)origin+=fixed.at(id);
        origin/=static_cast<double>(unit.base_anchors.size());
        if(unit.frame_origin)origin=*unit.frame_origin;
        const auto declared=task32_fixed_asset_detail::point(frames.at(unit.id));
        if((declared-origin).squaredNorm()!=0.)throw std::invalid_argument("native_frame_changed");
        unit.frame_origin=declared;
    }
}

inline nlohmann::json identity(const Task32ModeContract& a,const nlohmann::json& scene) {
    using json=nlohmann::json;
    json edges=json::array(),units=json::array(),roles=json::object(),frames=json::object(),observation=json::object();
    for(const auto& e:a.contract.reference_edges)edges.push_back({e.reference,e.owner});
    for(const auto& u:a.contract.coverage_units) {
        units.push_back({{"id",u.id},{"members",u.members},{"reference_anchors",u.base_anchors},
            {"leader",u.leader},{"moving_front_members",u.front_members}});
        frames[u.id]={{"origin",point(*u.frame_origin)},{"scale_source","member_role_coefficients"},{"additional_gain",1.0}};
        const auto effective=!u.search_observation_members.empty()?u.search_observation_members:
            (!u.front_members.empty()?u.front_members:std::vector<NodeId>{u.leader});
        observation[u.id]={{"declared_members",u.search_observation_members},{"effective_members",effective},
            {"empty_declaration_fallback","moving_front_members_then_leader"}};
    }
    for(const auto& [id,r]:a.contract.member_roles)
        roles[std::to_string(id)]={{"member",r.member},{"unit",r.coverage_unit},
            {"axial_fraction",r.axial_fraction},{"triangular_fraction",r.triangular_fraction}};
    return {{"schema","task32-mode-contract-identity-v1"},{"physical_scene",scene},
        {"coverage_contract",{{"version","task32-coverage-contract-v1"},{"mode_code",a.mode_code},
            {"mapping_kind",a.mapping_kind},{"source_contract_id",a.contract.id},
            {"reference_edges",edges},{"topological_order",a.contract.topological_order},
            {"units",units},{"member_roles",roles},{"front_frames",frames},{"observation",observation},
            {"lifting",{{"version","task20-affine-triangular-v1"},{"extra_member_clipping",false}}},
            {"strip_role_reassignment_pending",a.strip_role_reassignment_pending}}},
        {"initialization",{{"version",a.initialization_version},{"executed",false},
            {"boundary","version identity only; no state, noise or EKF initialization"}}},
        {"qualification",{{"runtime_qualified",false},{"service_qualified",false},
            {"boundary","pure contract; map scope, actual initialization, safety and information require separate gates"}}}};
}
} // namespace task32_mode_contract_detail

// The registry is an independently frozen caller input, not candidate authority.
// A registered payload match does not establish scientific approval of a registry.
inline Task32ModeContract task32ModeContract(const nlohmann::json& j,
    const nlohmann::json* registered_task31_asset=nullptr) {
    using json=nlohmann::json;
    using namespace task32_mode_contract_detail;
    Task32ModeContract out;
    const auto reject=[&](const std::string& reason){out.valid=false;out.reason=reason;return out;};
    try {
        if(!fields(j,{"schema","physical_scene","coverage","initialization"})||
            j.at("schema")!="task32-mode-contract-v1")return reject("mode_contract_schema_mismatch");
        const auto& scene=j.at("physical_scene");const auto& coverage=j.at("coverage");
        const auto& init=j.at("initialization");
        if(!fields(scene,{"width_m","height_m","physical_anchors"})||
            !fields(coverage,{"mode_code","mapping"})||!fields(init,{"version"}))
            return reject("mode_contract_field_set_mismatch");
        if(init.at("version")!="task31-original-physical-launch-v1")return reject("unregistered_initialization_version");
        out.initialization_version=init.at("version").get<std::string>();
        const auto& mode=coverage.at("mode_code");
        if(!mode.is_number_integer()||(mode!=0&&mode!=1&&mode!=2&&mode!=11&&mode!=12&&mode!=13))
            return reject("unregistered_mode");
        out.mode_code=mode.get<int>();
        const auto& mapping=coverage.at("mapping");
        out.mapping_kind=mapping.at("kind").get<std::string>();
        if(out.mode_code!=1) {
            const json old={{"schema","task32-fixed-coverage-asset-v1"},
                {"width_m",scene.at("width_m")},{"height_m",scene.at("height_m")},
                {"physical_anchors",scene.at("physical_anchors")},{"mode_code",mode},{"mapping",mapping}};
            const auto parsed=task32FixedCoverageAsset(old,registered_task31_asset);
            if(!parsed.valid)return reject(parsed.reason);
            out.width_m=parsed.width_m;out.height_m=parsed.height_m;out.fixed=parsed.fixed;out.contract=parsed.contract;
        } else {
            // Mode 1 already denotes the original merged Strip; never reassign IDs.
            if(!fields(mapping,{"kind","unit_frames"})||out.mapping_kind!="native-task20-strip")
                return reject("strip_requires_original_native_mapping");
            out.width_m=scene.at("width_m");out.height_m=scene.at("height_m");
            if(!std::isfinite(out.width_m)||!std::isfinite(out.height_m)||out.width_m<=0||out.height_m<=0||
                std::fmod(out.width_m,10.)!=0||std::fmod(out.height_m,10.)!=0)return reject("invalid_map");
            const auto& anchors=scene.at("physical_anchors");if(!anchors.is_object())return reject("anchor_object_required");
            for(auto it=anchors.begin();it!=anchors.end();++it) {
                std::size_t used=0;const int id=std::stoi(it.key(),&used);
                if(used!=it.key().size()||std::to_string(id)!=it.key()||id<=14||
                    !out.fixed.emplace(id,task32_fixed_asset_detail::point(it.value())).second)
                    return reject("invalid_physical_anchor_id");
            }
            for(int k=0;k<3;++k)if(!out.fixed.count(100+k)||
                (out.fixed.at(100+k)-Eigen::Vector2d((.4+.1*k)*out.width_m,-50)).norm()>1e-9)
                return reject("legacy_anchors_changed");
            out.contract=task20DagLatticeContract(Task20LatticeMode::MergedStrip);
            explicitFrames(out.contract,out.fixed,mapping.at("unit_frames"));
            out.contract.fixed_anchor_ids.clear();for(const auto& [id,p]:out.fixed)out.contract.fixed_anchor_ids.push_back(id);
            task20_lattice_detail::finish(out.contract);
            if(!out.contract.valid)return reject(out.contract.reason);
            out.strip_role_reassignment_pending=true;
        }
        out.identity=identity(out,scene);out.valid=true;out.reason="parsed_contract_not_runtime_qualification";
        return out;
    }catch(const std::exception& e){return reject(std::string("invalid_mode_contract:")+e.what());}
}
} // namespace gf
