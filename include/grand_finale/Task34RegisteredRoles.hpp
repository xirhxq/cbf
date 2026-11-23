#pragma once
#include "grand_finale/Task34RoleContract.hpp"
#include "grand_finale/CheckpointCodec.hpp"

namespace gf {
class Task34RegisteredRoles {
public:
    Task34RegisteredRoles(const nlohmann::json& modes,const nlohmann::json& asset):asset_(asset) {
        if(!modes.is_array()||modes.size()!=6)throw std::invalid_argument("six_registered_modes_required");
        std::optional<nlohmann::json> scene;
        for(const auto& payload:modes) {
            const auto mode=task32ModeContract(payload,&asset_);
            if(!mode.valid||!payloads_.emplace(mode.mode_code,payload).second)
                throw std::invalid_argument("invalid_or_duplicate_registered_mode:"+mode.reason);
            if(scene&&*scene!=payload.at("physical_scene"))
                throw std::invalid_argument("role_library_physical_scene_mismatch");
            scene=payload.at("physical_scene");
        }
        if(payloads_.size()!=6)throw std::invalid_argument("incomplete_six_mode_library");
    }
    Task32ModeContract instantiate(int code,const std::map<NodeId,NodeId>& mapping) const {
        const auto found=payloads_.find(code);
        if(found==payloads_.end())throw std::invalid_argument("unregistered_role_mode");
        auto result=task34InstantiateRoleContract(found->second,&asset_,mapping);
        if(!result.valid)throw std::invalid_argument("invalid_role_instance:"+result.reason);
        return result;
    }
    nlohmann::json freeze(int code,const std::map<NodeId,NodeId>& mapping) const {
        const auto instance=instantiate(code,mapping);
        return {{"schema","task34-frozen-role-instance-v1"},{"mode",code},
            {"role_to_physical",checkpoint::encode(mapping)},
            {"library",checkpoint::encode(payloads_)},{"registered_asset",asset_},
            {"identity",instance.identity}};
    }
    Task32ModeContract restore(const nlohmann::json& saved) const {
        if(!task32_mode_contract_detail::fields(saved,{"schema","mode","role_to_physical",
                "library","registered_asset","identity"})||
            saved.at("schema")!="task34-frozen-role-instance-v1"||
            saved.at("library")!=checkpoint::encode(payloads_)||saved.at("registered_asset")!=asset_)
            throw std::invalid_argument("role_instance_library_mismatch");
        int code=-1;std::map<NodeId,NodeId> mapping;
        checkpoint::decode(saved.at("mode"),code);checkpoint::decode(saved.at("role_to_physical"),mapping);
        const auto instance=instantiate(code,mapping);
        if(freeze(code,mapping)!=saved)throw std::invalid_argument("role_instance_identity_mismatch");
        return instance;
    }
private:
    std::map<int,nlohmann::json> payloads_;
    nlohmann::json asset_;
};
} // namespace gf
