#pragma once
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

namespace gf {

// Read-only current observations, not historical minimum margins and not a
// new information gate. An accepted ranging link need not be a control edge.
inline nlohmann::json task31InformationTelemetry(GrandFinaleSwarmAdapter& adapter) {
    const auto r=adapter.runtimeSnapshot();const auto a=adapter.currentReferenceAudit();
    auto number=[](double x)->nlohmann::json {return std::isfinite(x)?nlohmann::json(x):nlohmann::json(nullptr);};
    nlohmann::json batch=nlohmann::json::array(),links=nlohmann::json::object(),control=nlohmann::json::array(),cov=nlohmann::json::object();
    for(const auto& e:adapter.lastAcceptedRangeBatchAudit())batch.push_back({e.measurement.edge.first,e.measurement.edge.second,
        e.measurement.range_m,e.measurement.variance_m2,e.master,e.innovation,e.innovation_variance,e.measurement.timestamp_ns});
    for(const auto& [id,s]:r.range_links)links[id]={s.age_s,s.quality,s.variance_m2};
    for(const auto& e:r.topology)control.push_back({e.reference,e.owner});
    for(auto id:r.estimate.mobile_ids)cov[std::to_string(id)]={detail::maximumPositionEigenvalue(r.estimate,id),detail::maximumVelocityEigenvalue(r.estimate,id)};
    nlohmann::json result={{"current_time_s",r.runtime_s},{"estimator_token",r.estimator_token},
        {"accepted_columns",{"first","second","range_m","variance_m2","master","innovation_m","innovation_variance_m2","timestamp_ns"}},
        {"accepted_batch",batch},{"range_link_columns",{"age_s","quality","variance_m2"}},{"range_links",links},
        {"control_edges",control},{"member_position_velocity_posterior_max_eigenvalues",cov},
        {"qualified_information",{{"minimum_count",a.minimum_information_edge_count},{"minimum_count_owner",a.minimum_information_edge_owner},
            {"fim_min",number(a.minimum_fim_eigenvalue)},{"fim_owner",a.minimum_fim_owner},
            {"robust_fim_min",number(a.minimum_robust_fim_cone_lower_bound)},{"robust_fim_owner",a.minimum_robust_fim_owner}}},
        {"reference_only",{{"minimum_effective_count",a.minimum_effective_reference_count},{"fim_min",number(a.minimum_reference_only_fim_eigenvalue)},
            {"robust_fim_min",number(a.minimum_reference_only_robust_fim_cone_lower_bound)},{"robust_fim_owner",a.minimum_reference_only_robust_fim_owner}}},
        {"posterior_max_m2",a.maximum_posterior_eigenvalue},{"aoi_margin_min_s",number(a.minimum_range_aoi_margin_s)},
        {"model_boundary","frozen all-pair ranging model; no general UWB distance cutoff; 850m is control-reference maintenance, not ranging cutoff"}};
    if(adapter.config().distance_range_availability.enabled) {
        nlohmann::json generated=nlohmann::json::array();
        for(const auto& x:adapter.lastRangeGenerationAudit())
            generated.push_back({x.batch,x.edge.first,x.edge.second,x.true_distance_m,x.probability,x.uniform,x.acquired});
        result["range_acquisition"]={{"model","distance-exponential-850-1000p05-v1"},
            {"link_seed",adapter.config().distance_range_availability.link_seed},
            {"columns",{"batch","first","second","physical_distance_m","probability","uniform","acquired"}},
            {"generated_batch",generated},
            {"boundary","physical acquisition only; accepted_batch additionally obeys unchanged innovation/quality/variance/AoI rules; reference DAG is separate"}};
        result["model_boundary"]="all-pair opportunities; acquisition1 through850m, exponential outside with probability0.05 at1000m; no fixed dropout; accepted qualified links feed unchanged EKF/FIM";
    }
    return result;
}

} // namespace gf
