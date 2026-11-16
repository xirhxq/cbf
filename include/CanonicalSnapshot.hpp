#ifndef SHARED_CANONICAL_SNAPSHOT_HPP
#define SHARED_CANONICAL_SNAPSHOT_HPP
#include "Swarm.hpp"
#include "diagnostics/CanonicalCaptureAudit.hpp"
#include <algorithm>
namespace canonical {
inline std::string bitmap(const std::vector<bool>& values) {
    std::vector<unsigned char> b((values.size()+7)/8,0);for(std::size_t i=0;i<values.size();++i)if(values[i])b[i/8]|=1u<<(i%8);
    const char* d="0123456789abcdef";std::string s;s.reserve(b.size()*2);for(auto v:b){s+=d[v>>4];s+=d[v&15];}return s;
}
inline json certificate(const cbf2026::NodeRateCertificate& c) {
    json refs=json::array();for(const auto& r:c.frozenReferences)refs.push_back({{"reference_id",r.referenceId},{"direction",vector(r.direction)},{"effective_variance",r.effectiveVariance}});
    return {{"robot_id",c.robotId},{"snapshot_version",c.snapshotVersion},{"information",matrix(c.information)},{"covariance",matrix(c.covariance)},
        {"epsilon",c.epsilon},{"information_rate_bound",c.informationRateBound},{"covariance_rate_bound",c.covarianceRateBound},{"epsilon_rate_bound",c.epsilonRateBound},{"frozen_references",refs}};
}
inline json comm(const Robot& r) {
    json out={{"positions",json::array()},{"velocities",json::array()},{"yaw",json::array()},{"battery",json::array()},{"covariance",json::array()},{"uncertainty_rate",json::array()}};
    for(int id=1;id<=14;++id) {
        if(r.comm->_othersPos.count(id))out["positions"].push_back({{"id",id},{"value",{r.comm->_othersPos.at(id).x,r.comm->_othersPos.at(id).y}}});
        if(r.comm->_othersVel.count(id))out["velocities"].push_back({{"id",id},{"value",vector(r.comm->_othersVel.at(id))}});
        if(r.comm->_othersYawRad.count(id))out["yaw"].push_back({{"id",id},{"value",r.comm->_othersYawRad.at(id)}});
        if(r.comm->_othersBatteryLevel.count(id))out["battery"].push_back({{"id",id},{"value",r.comm->_othersBatteryLevel.at(id)}});
        if(r.comm->_othersPositionCovariance.count(id))out["covariance"].push_back({{"id",id},{"value",matrix(r.comm->_othersPositionCovariance.at(id))}});
        if(r.comm->_othersUncertaintyRate.count(id))out["uncertainty_rate"].push_back({{"id",id},{"value",r.comm->_othersUncertaintyRate.at(id)}});
    }
    // These sources are excluded by the declared main common contract. Do not
    // silently omit consumed endpoint/theorem state if a future method needs it.
    if(!r.comm->_othersEndpointCertificateSnapshots.empty() || !r.comm->_othersEpsilon.empty() || !r.comm->_othersBarNu.empty()
       || !r.comm->_othersCovarianceRateBound.empty() || !r.comm->_othersCertificateSnapshotVersion.empty() || !r.comm->_othersAllocationVersion.empty())
        throw std::runtime_error("capture contract lacks consumed theorem/endpoint state; independent extension required");
    out["excluded_endpoint_dictionary_sizes"]={{"snapshots",0},{"epsilon",0},{"bar_nu",0},{"rate",0},{"version",0},{"allocation",0}};
    return out;
}
inline json excludedState(const Robot& r) {
    const auto& c=r.committedCertificateState;
    if(r.certificateAvailable||r.localCertificateSnapshot.has_value()||r.lastConsumedHardConstraintProblem.has_value()||!r.lastConsumedHardProblemHash.empty()
       ||c.version!=0||c.valid||!c.nodeVersions.empty()||!c.endpoints.empty()||!c.hardProblems.empty()||!c.certificates.empty())throw std::runtime_error("capture lacks consumed endpoint/hard/committed state; explicit extension required");
    return {{"certificate_available",false},{"local_snapshot_present",false},{"last_hard_problem_present",false},{"last_hard_problem_hash",""},
        {"committed_version",c.version},{"committed_valid",false},{"committed_node_versions",0},{"committed_endpoints",0},{"committed_hard_problems",0},{"committed_certificates",0}};
}
inline json node(Robot& r) {
    if(r.runtime!=0. || r.model->getControlInput().size()!=3 || r.model->getStateSize()!=4 || r.estimatorInLoop
       || r.certificateAvailable || r.localCertificateSnapshot.has_value())throw std::runtime_error("capture initial phase/model contract mismatch");
    return {{"id",r.id},{"state",r.model->state2Json()},{"X_x_y_battery_yaw",vector(r.model->getX())},{"u0",r.model->control2Json()},
        {"velocity",vector(r.model->getVelocity())},{"runtime_s",r.runtime},{"drift_f",vector(r.model->f())},{"control_g",matrix(r.model->g())},
        {"settings_copy",r.settings},{"formation",r.myFormation},{"covariance_formation",r.myCovarianceFormation},{"part_id",r.partId},{"id_in_part",r.idInMyPart},
        {"neighbours",r.myNeighboursId},{"base_ids",r.myBasesId},{"position_covariance",matrix(r.positionCovariance)},
        {"uncertainty_history",{{"previous",r.previousUncertainty},{"current",r.currentUncertainty},{"rate",r.uncertaintyRate},{"present",r.hasUncertaintyHistory}}},
        {"rate_certificate",certificate(r.rateCertificate)},{"certificate_version",r.certificateSnapshotVersion},{"allocation_version",r.certificateAllocationVersion},
        {"estimator_active",false},{"estimator_frame",nullptr},{"map_phase","pre-estimate/pre-sensor"},{"valid_hex",bitmap(r.gridWorld.valid)},{"visited_hex",bitmap(r.gridWorld.vis)},
        {"comm",comm(r)},{"excluded_consumed_state",excludedState(r)}};
}
inline json snapshot(Swarm& s,const std::string& phase,const json& binding) {
    json nodes=json::array();for(auto& r:s.robots)nodes.push_back(node(*r));
    if(!s.certificateResetHistory.empty() || s.robots.front()->usesAnalyticTopologicalRateCertificate())throw std::runtime_error("capture excludes theorem/reset consumedstate");
    return {{"schema","shared-native-capture-receipt-v1"},{"phase",phase},{"binding",binding},{"nodes",nodes},{"swarm_config_copy",s.config},
        {"world",s.config.at("world")},{"bases",s.config.at("bases")},{"grid",{{"xNum",300},{"yNum",300},{"xLim",{-1500.,1500.}},{"yLim",{-1500.,1500.}}}},
        {"true_evaluation_map_phase","pre-sensor isolated true map"},{"true_valid_hex",bitmap(s.gridWorldGroundTruth.valid)},{"true_visited_hex",bitmap(s.gridWorldGroundTruth.vis)},
        {"reset_history",json::array()},{"charge_policy","original native discharge; charge setter only in optimise, not invoked by capture; checkCharge original before any future hold"},
        {"counts",counters()},{"method_solver_and_PCG_warm_state","fresh per method, not inherited/shared warmstart"}};
}
inline void declaredBootstrap(Swarm& s) {
    s.exchangeData();
    std::vector<Robot*> order;for(auto& r:s.robots)order.push_back(r.get());
    std::sort(order.begin(),order.end(),[](const Robot* a,const Robot* b){return a->idInMyPart==b->idInMyPart?a->partId<b->partId:a->idInMyPart<b->idInMyPart;});
    for(auto* r:order){r->updateCovarianceAndRate(.5);for(auto& q:s.robots){q->comm->receivePositionCovariance(r->id,r->positionCovariance);q->comm->receiveUncertaintyRate(r->id,r->uncertaintyRate);}}
    for(auto& r:s.robots)r->setupFormation();
    // No preset/postset CBF, estimator exchange, sensor, target, solver or hold.
}
}
#endif
