#pragma once
#include "grand_finale/Task25P0MultiDag.hpp"
#include "grand_finale/Task31AnchorScene.hpp"
#include "json.hpp"

namespace gf {

// Pure contract parser, not runtime admission or a service/safety certificate.
// No existing runner/controller calls this new research interface by default.
struct Task32FixedCoverageAsset {
    bool valid=false;
    std::string reason;
    int mode_code=-1;
    double width_m=0,height_m=0;
    std::map<NodeId,Eigen::Vector2d> fixed;
    Task20DagLatticeContract contract;
    nlohmann::json identity;
};

namespace task32_fixed_asset_detail {
inline Eigen::Vector2d point(const nlohmann::json& value) {
    if(!value.is_array()||value.size()!=2)throw std::invalid_argument("point dimension");
    Eigen::Vector2d p(value.at(0).get<double>(),value.at(1).get<double>());
    if(!p.allFinite())throw std::invalid_argument("nonfinite point");
    return p;
}
} // namespace task32_fixed_asset_detail

// registered_task31_asset is supplied by the caller's separately frozen
// registry, never inferred from the candidate's own embedded payload.
inline Task32FixedCoverageAsset task32FixedCoverageAsset(
    const nlohmann::json& j,const nlohmann::json* registered_task31_asset=nullptr) {
    Task32FixedCoverageAsset out;out.identity=j;
    const auto reject=[&](const std::string& reason){out.valid=false;out.reason=reason;return out;};
    try {
        const std::set<std::string> keys{"schema","width_m","height_m","mode_code","physical_anchors","mapping"};
        if(!j.is_object()||j.size()!=keys.size())return reject("fixed_asset_field_set_mismatch");
        for(auto it=j.begin();it!=j.end();++it)if(!keys.count(it.key()))return reject("unknown_fixed_asset_field");
        if(j.at("schema")!="task32-fixed-coverage-asset-v1")return reject("unknown_schema");
        const auto& mode=j.at("mode_code");
        if(!mode.is_number_integer()||(mode!=0&&mode!=2&&mode!=11&&mode!=12&&mode!=13))
            return reject("unregistered_integer_mode");
        out.width_m=j.at("width_m");out.height_m=j.at("height_m");out.mode_code=j.at("mode_code");
        if(!std::isfinite(out.width_m)||!std::isfinite(out.height_m)||out.width_m<=0||out.height_m<=0||
            std::fmod(out.width_m,10.)!=0||std::fmod(out.height_m,10.)!=0)return reject("invalid_map");
        const auto& anchors=j.at("physical_anchors");if(!anchors.is_object())return reject("anchor_object_required");
        for(auto it=anchors.begin();it!=anchors.end();++it) {
            std::size_t used=0;const int id=std::stoi(it.key(),&used);
            if(used!=it.key().size()||std::to_string(id)!=it.key()||id<=14||
                !out.fixed.emplace(id,task32_fixed_asset_detail::point(it.value())).second)
                return reject("invalid_physical_anchor_id");
        }
        for(int k=0;k<3;++k)if(!out.fixed.count(100+k)||
            (out.fixed.at(100+k)-Eigen::Vector2d((.4+.1*k)*out.width_m,-50)).norm()>1e-9)
            return reject("legacy_anchors_changed");
        const auto& mapping=j.at("mapping");
        if(!mapping.is_object()||mapping.size()!=2||!mapping.contains("kind"))return reject("mapping_field_set_mismatch");
        if(mapping.at("kind")=="frozen-task31") {
            if(out.mode_code!=12||!mapping.contains("asset"))return reject("frozen_task31_mode_mismatch");
            if(!registered_task31_asset||mapping.at("asset")!=*registered_task31_asset)
                return reject("unregistered_task31_payload");
            const auto old=task31AnchorScene(mapping.at("asset"));
            if(!old.valid||old.mode_code!=out.mode_code||old.width_m!=out.width_m||old.height_m!=out.height_m||old.fixed!=out.fixed)
                return reject("frozen_task31_scene_mismatch");
            out.contract=old.goal;out.valid=true;out.reason="parsed_contract_not_runtime_qualification";return out;
        }
        if(mapping.at("kind")!="native-task25")return reject("unregistered_mapping_kind");
        if(!mapping.contains("unit_frames"))return reject("native_unit_frames_required");
        if(out.mode_code!=0&&out.mode_code!=11&&out.mode_code!=2&&out.mode_code!=13)
            return reject("unregistered_native_mode");
        out.contract=task25DagContractFromCode(out.mode_code);
        const auto& frames=mapping.at("unit_frames");
        if(!frames.is_object()||frames.size()!=out.contract.coverage_units.size())return reject("unit_frame_set_mismatch");
        for(auto& unit:out.contract.coverage_units) {
            Eigen::Vector2d original=Eigen::Vector2d::Zero();
            for(auto id:unit.base_anchors)original+=out.fixed.at(id);
            original/=static_cast<double>(unit.base_anchors.size());
            if(unit.frame_origin)original=*unit.frame_origin;
            const auto declared=task32_fixed_asset_detail::point(frames.at(unit.id));
            if((declared-original).norm()!=0)return reject("native_frame_changed");
            unit.frame_origin=declared;
        }
        out.contract.fixed_anchor_ids.clear();
        for(const auto& [id,p]:out.fixed)out.contract.fixed_anchor_ids.push_back(id);
        task20_lattice_detail::finish(out.contract);
        if(!out.contract.valid)return reject(out.contract.reason);
        out.valid=true;out.reason="parsed_contract_not_runtime_qualification";return out;
    }catch(const std::exception& e){return reject(std::string("invalid_fixed_asset:")+e.what());}
}
} // namespace gf
