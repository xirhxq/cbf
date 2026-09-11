#pragma once
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "json.hpp"

namespace gf {
// Read-only serialization before controller.advance. Not a restart checkpoint,
// and deliberately not labelled as the post-allocation final QP objective.
inline nlohmann::json task32InformationSnapshot(
    const GrandFinaleSwarmAdapter& adapter,std::size_t tick) {
    using json=nlohmann::json;
    const auto s=adapter.runtimeSnapshot();
    const auto rows=adapter.currentSnapshotHardRows(s.topology);
    const auto xy=[](const Eigen::Vector2d& p){return json::array({p.x(),p.y()});};
    json mean=json::array(),cov=json::array(),anchors=json::object(),links=json::object(),edges=json::array(),encoded_rows=json::array();
    for(Eigen::Index i=0;i<s.estimate.mean.size();++i) {
        mean.push_back(s.estimate.mean(i));json row=json::array();
        for(Eigen::Index j=0;j<s.estimate.covariance.cols();++j)row.push_back(s.estimate.covariance(i,j));
        cov.push_back(row);
    }
    for(const auto& [id,p]:s.estimate.fixed_positions)anchors[std::to_string(id)]=xy(p);
    for(const auto& [id,l]:s.range_links)links[id]={{"aoi_s",l.age_s},{"quality",l.quality},{"variance_m2",l.variance_m2}};
    for(const auto& e:s.topology)edges.push_back({e.reference,e.owner});
    for(const auto& r:rows)encoded_rows.push_back({{"id",r.id},{"kind",static_cast<int>(r.kind)},{"owner",r.owner},
        {"peer",r.peer?json(*r.peer):json(nullptr)},{"coefficient",xy(r.control_coefficient)},{"constant",r.constant},
        {"normal",xy(r.normal)},{"responsibility",r.responsibility},{"participates_in_gamma",r.participates_in_gamma},
        {"h",r.barrier_h},{"psi1",r.barrier_psi1},{"hdot",r.barrier_hdot},
        {"coefficient_reserve",r.coefficient_uncertainty_reserve},{"position_reserve_m",r.position_uncertainty_reserve_m},
        {"velocity_reserve_mps",r.velocity_uncertainty_reserve_mps}});
    return {{"schema","task32-precontrol-information-snapshot-v1"},{"phase","before_controller_advance"},{"tick",tick},
        {"runtime_s",s.runtime_s},{"mobile_ids",s.estimate.mobile_ids},{"mean",mean},{"covariance",cov},
        {"anchors",anchors},{"range_links",links},{"reference_edges",edges},{"rows",encoded_rows},
        {"estimator_token",s.estimator_token},{"topology_token",s.topology_token},{"mode",static_cast<int>(s.mode)},
        {"half_box",adapter.config().acceleration_half_box},{"minimum_range_quality",adapter.config().minimum_range_quality},
        {"maximum_range_aoi_s",adapter.config().maximum_range_aoi_s},
        {"boundary","No plant/noise/task-ledger checkpoint. Match post-step telemetry tick for actual selected/applied control."}};
}
} // namespace gf
