#pragma once
#include "grand_finale/Task32ModeContract.hpp"

namespace gf {
// Explicit opt-in seam; no existing runner or production dispatch uses this.
inline Task32ModeContract task34InstantiateRoleContract(const nlohmann::json& payload,
    const nlohmann::json* registered_asset,const std::map<NodeId,NodeId>& role_to_physical) {
    auto result=task32ModeContract(payload,registered_asset);
    if(!result.valid)return result;
    try {
        const auto original=result.contract;
        if(role_to_physical.size()!=original.member_roles.size())
            throw std::invalid_argument("role_mapping_domain");
        std::set<NodeId> image;
        for(const auto& [role,id]:role_to_physical) {
            if(!original.member_roles.count(role)||!original.member_roles.count(id)||
                result.fixed.count(id)||!image.insert(id).second)
                throw std::invalid_argument("role_mapping_not_mobile_bijection");
        }
        const auto physical=[&](NodeId id) {
            return result.fixed.count(id)?id:role_to_physical.at(id);
        };
        for(auto& edge:result.contract.reference_edges) {
            edge.reference=physical(edge.reference);edge.owner=physical(edge.owner);
        }
        result.contract.member_roles.clear();
        for(auto [id,role]:original.member_roles) {
            role.member=physical(id);
            result.contract.member_roles.emplace(role.member,role);
        }
        for(auto& unit:result.contract.coverage_units) {
            for(auto& id:unit.members)id=physical(id);
            for(auto& id:unit.front_members)id=physical(id);
            for(auto& id:unit.search_observation_members)id=physical(id);
            unit.leader=physical(unit.leader);
        }
        task20_lattice_detail::finish(result.contract);
        if(!result.contract.valid)throw std::invalid_argument(result.contract.reason);
        result.strip_role_reassignment_pending=false;
        result.runtime_qualified=false;
        result.identity=task32_mode_contract_detail::identity(result,payload.at("physical_scene"));
        nlohmann::json mapping=nlohmann::json::object();
        for(const auto& [role,id]:role_to_physical)mapping[std::to_string(role)]=id;
        result.identity["role_instantiation"]={{"version","task34-role-instance-v1"},
            {"role_to_physical",mapping},{"registered_template",payload},
            {"anchors_permuted",false},{"runtime_qualified",false}};
        result.reason="instantiated_contract_not_runtime_qualification";
    } catch(const std::exception& e) {
        result.valid=false;result.reason=e.what();
    }
    return result;
}
} // namespace gf
