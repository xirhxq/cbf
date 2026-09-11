#pragma once
#include "grand_finale/Task32ModeContract.hpp"
#include <optional>

namespace gf {
// Preparation only. No controller, estimator, plant or registry is mutated.
struct Task32FixedModeDispatchRequest {
    bool valid=true;
    bool enabled=false;
    std::string reason="legacy_arguments_unchanged";
    std::vector<std::string> legacy_arguments;
    std::string asset_path;
};

struct Task32FixedModeDispatch {
    bool valid=true;
    bool enabled=false;
    std::string reason="legacy_dispatch_unchanged";
    std::optional<Task32ModeContract> asset;
    nlohmann::json telemetry_identity;
};

inline Task32FixedModeDispatchRequest task32FixedModeDispatchRequest(
    const std::vector<std::string>& arguments) {
    Task32FixedModeDispatchRequest out;
    out.legacy_arguments=arguments;
    const std::string prefix="fixed-mode-asset=";
    for(std::size_t i=0;i<arguments.size();++i) {
        if(arguments[i].rfind(prefix,0)!=0)continue;
        if(i==0||i+1!=arguments.size()||arguments[i].size()==prefix.size()) {
            out.valid=false;out.reason="fixed_mode_requires_one_nonempty_final_asset_option";
            return out;
        }
        out.enabled=true;out.asset_path=arguments[i].substr(prefix.size());
        out.legacy_arguments.pop_back();out.reason="explicit_fixed_mode_asset";
    }
    return out;
}

inline Task32FixedModeDispatch task32FixedModeDispatch(
    const Task32FixedModeDispatchRequest& request,const nlohmann::json& payload,
    const nlohmann::json* registered_task31_asset=nullptr) {
    Task32FixedModeDispatch out;
    out.enabled=request.enabled;
    if(!request.valid) {
        out.valid=false;out.reason=request.reason;return out;
    }
    if(!request.enabled)return out;
    if(request.asset_path.empty()) {
        out.valid=false;out.reason="missing_fixed_mode_asset_path";return out;
    }
    auto asset=task32ModeContract(payload,registered_task31_asset);
    if(!asset.valid) {
        out.valid=false;out.reason=asset.reason;return out;
    }
    out.telemetry_identity=asset.identity;
    out.telemetry_identity["dispatch"]={{"version","task32-fixed-mode-dispatch-v1"},
        {"requested_asset_path",request.asset_path},{"enabled",true},
        {"control_configuration_changed",false},
        {"boundary","prepared identity only; runner admission and initialization remain separate"}};
    out.asset=std::move(asset);out.reason="prepared_fixed_mode_not_runtime_admitted";
    return out;
}

// Invoke on the unshifted original launch before creating Swarm/settings.
// Only geometry is prepared; velocity, yaw, estimator and scientific settings
// remain the caller's frozen inputs. Initial admission still has to succeed.
inline Task10p10Scenario task32FixedModeScenario(
    const Task32FixedModeDispatch& plan,const Task10p10Scenario& legacy) {
    if(!plan.valid)throw std::invalid_argument("invalid_fixed_mode_dispatch");
    if(!plan.enabled)return legacy;
    if(!plan.asset||!plan.asset->valid||
        plan.asset->initialization_version!="task31-original-physical-launch-v1")
        throw std::invalid_argument("unregistered_fixed_mode_initialization");
    const auto baseline=task10p11pStandardCoastalScenario();
    if(legacy.width_m!=baseline.width_m||legacy.height_m!=baseline.height_m||
        legacy.mobile_ids!=baseline.mobile_ids||legacy.mobile_positions!=baseline.mobile_positions||
        legacy.fixed_positions!=baseline.fixed_positions)
        throw std::invalid_argument("fixed_mode_requires_unshifted_original_launch");
    const auto& asset=*plan.asset;
    auto result=legacy;
    result.width_m=asset.width_m;result.height_m=asset.height_m;
    result.fixed_positions=asset.fixed;
    result.initial_topology=asset.contract.reference_edges;
    for(auto& position:result.mobile_positions)position.x()+=(asset.width_m-baseline.width_m)*.5;
    return result;
}

// Config/telemetry adapter for the prepared geometry, not a claim that an EKF
// or physical initial-state gate has been executed. Reject scene/contract drift.
inline nlohmann::json task32FixedModeGeometryIdentity(
    const Task32FixedModeDispatch& plan,const Task10p10Scenario& prepared) {
    if(!plan.valid)throw std::invalid_argument("invalid_fixed_mode_dispatch");
    if(!plan.enabled)return nullptr;
    const auto expected=task32FixedModeScenario(plan,task10p11pStandardCoastalScenario());
    if(prepared.width_m!=expected.width_m||prepared.height_m!=expected.height_m||
        prepared.fixed_positions!=expected.fixed_positions||prepared.initial_topology!=expected.initial_topology||
        prepared.mobile_ids!=expected.mobile_ids||prepared.mobile_positions!=expected.mobile_positions)
        throw std::invalid_argument("fixed_mode_prepared_geometry_mismatch");
    auto identity=plan.telemetry_identity;
    identity["initialization"]["prepared_mobile_ids"]=prepared.mobile_ids;
    auto positions=nlohmann::json::array();
    for(const auto& p:prepared.mobile_positions)positions.push_back({p.x(),p.y()});
    identity["initialization"]["prepared_mobile_positions"]=std::move(positions);
    identity["initialization"]["geometry_only"]=true;
    return identity;
}
} // namespace gf
