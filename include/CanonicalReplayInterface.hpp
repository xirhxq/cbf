#ifndef SHARED_CANONICAL_REPLAY_INTERFACE_HPP
#define SHARED_CANONICAL_REPLAY_INTERFACE_HPP
#include "CanonicalReplayValidation.hpp"
namespace canonical {
inline Eigen::Matrix2d matrix2(const json& x) {
    if(!x.is_array() || x.size()!=2 || x.at(0).size()!=2 || x.at(1).size()!=2)throw std::runtime_error("sharedmatrix2 schema");
    Eigen::Matrix2d m;for(int i=0;i<2;++i)for(int j=0;j<2;++j)m(i,j)=x.at(i).at(j).get<double>();
    if(!m.allFinite())throw std::runtime_error("sharedmatrix nonfinite");return m;
}
inline void prepareReplayConfig(json& method,const json& receipt) {
    validateReplayReceipt(receipt);
    if(receipt.at("schema")!="shared-native-capture-receipt-v1" || receipt.at("nodes").size()!=14
       || receipt.at("phase")!="post-declared-comm-covariance-fixedgraph-bootstrap/pre-estimator-sensor-target-control")throw std::runtime_error("complete sharednative receipt missing");
    const auto& common=receipt.at("swarm_config_copy");
    method["canonical-replay-state"]=receipt.at("nodes");
    method["initial"]["position"]={{"method","specified"},{"positions",json::array()}};
    for(const auto& n:receipt.at("nodes"))method["initial"]["position"]["positions"].push_back({n.at("state").at("x"),n.at("state").at("y")});
    if(!nativeBitsEqual(commonSettingsProjection(method),commonSettingsProjection(common)))throw std::runtime_error("complete source common config/initial native type/bits mismatch");
    // Called before each method's constructor; supplied fullX/u installed bysetup,
    // original rand branch bypassed. Fresh method solver/env/PCG state staysfresh.
}
inline void replayDeclaredBootstrap(Swarm& s,const json& receipt) {
    validateReplayReceipt(receipt);
    if(s.robots.size()!=14)throw std::runtime_error("sharedreplay14 required");
    for(std::size_t i=0;i<14;++i) {
        auto& r=*s.robots[i];const auto& n=receipt.at("nodes").at(i);
        if(n.at("id")!=r.id || r.runtime!=0.)throw std::runtime_error("sharedreplay nodephase");
        r.myFormation=n.at("formation");r.myCovarianceFormation=n.at("covariance_formation");r.positionCovariance=matrix2(n.at("position_covariance"));
        const auto& u=n.at("uncertainty_history");r.previousUncertainty=u.at("previous");r.currentUncertainty=u.at("current");r.uncertaintyRate=u.at("rate");r.hasUncertaintyHistory=u.at("present");
        const auto& c=n.at("rate_certificate");r.rateCertificate.robotId=c.at("robot_id");r.rateCertificate.snapshotVersion=c.at("snapshot_version");r.rateCertificate.information=matrix2(c.at("information"));r.rateCertificate.covariance=matrix2(c.at("covariance"));
        r.rateCertificate.epsilon=c.at("epsilon");r.rateCertificate.informationRateBound=c.at("information_rate_bound");r.rateCertificate.covarianceRateBound=c.at("covariance_rate_bound");r.rateCertificate.epsilonRateBound=c.at("epsilon_rate_bound");r.rateCertificate.frozenReferences.clear();
        for(const auto& ref:c.at("frozen_references")){Eigen::Vector2d d(ref.at("direction").at(0),ref.at("direction").at(1));r.rateCertificate.frozenReferences.push_back({ref.at("reference_id"),d,ref.at("effective_variance")});}
        r.certificateSnapshotVersion=n.at("certificate_version");r.certificateAllocationVersion=n.at("allocation_version");r.certificateAvailable=false;r.localCertificateSnapshot.reset();
        r.comm->_othersPos.clear();r.comm->_othersVel.clear();r.comm->_othersYawRad.clear();r.comm->_othersBatteryLevel.clear();r.comm->_othersPositionCovariance.clear();r.comm->_othersUncertaintyRate.clear();
        const auto& comm=n.at("comm");
        for(const auto& x:comm.at("positions"))r.comm->receivePosition2D(x.at("id"),Point(x.at("value").at(0),x.at("value").at(1)));
        for(const auto& x:comm.at("velocities")){Eigen::VectorXd v(2);v<<x.at("value").at(0).get<double>(),x.at("value").at(1).get<double>();r.comm->receiveVelocity2D(x.at("id"),v);}
        for(const auto& x:comm.at("yaw"))r.comm->receiveYawRad(x.at("id"),x.at("value"));
        for(const auto& x:comm.at("battery"))r.comm->receiveBatteryLevel(x.at("id"),x.at("value"));
        for(const auto& x:comm.at("covariance"))r.comm->receivePositionCovariance(x.at("id"),matrix2(x.at("value")));
        for(const auto& x:comm.at("uncertainty_rate"))r.comm->receiveUncertaintyRate(x.at("id"),x.at("value"));
        auto actual=node(r);auto expected=n;
        actual["settings_copy"]=commonSettingsProjection(actual.at("settings_copy"));expected["settings_copy"]=commonSettingsProjection(expected.at("settings_copy"));
        if(!nativeBitsEqual(actual,expected))throw std::runtime_error("shared complete native replay state/F/g/comm/graph/map/history/exclusion bits mismatched");
    }
    // No fresh bootstrap covariance recomputation, no RNG, estimate/sensor orcontrol.
    // Caller must obtain actual-bit receipt and later postestimated-map barrier.
}
}
#endif
