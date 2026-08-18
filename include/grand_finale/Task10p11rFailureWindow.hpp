#pragma once

#include "grand_finale/CanonicalGammaStarFeedback.hpp"
#include "grand_finale/ExactHardPolytopeDiagnostic.hpp"

namespace gf {

struct Task10p11rFailureSnapshotAudit {
    bool valid=false;
    std::string reason;
    NodeId limiting_owner=0;
    double current_gamma=-std::numeric_limits<double>::infinity();
    Eigen::Vector2d gamma_witness=Eigen::Vector2d::Zero();
    std::string dominant_row;
    HardPolytopeCertificate hard_polytope;
};

inline Task10p11rFailureSnapshotAudit auditTask10p11rFailureSnapshot(
    const std::vector<NodeId>& owners,double time_s,SupervisorMode mode,
    const std::vector<DirectedEdge>& topology,
    const std::vector<CanonicalHardRow>& rows,double input_half_box) {
    Task10p11rFailureSnapshotAudit result;
    if (owners.empty() || !std::isfinite(time_s) || time_s<0.0 ||
        !std::isfinite(input_half_box) || input_half_box<=0.0) {
        result.reason="invalid_failure_snapshot_request";
        return result;
    }
    double minimum=std::numeric_limits<double>::infinity();
    for (NodeId owner:owners) {
        const auto gamma=solveCanonicalGammaStar(rows,owner,input_half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma)) {
            result.reason="gamma_oracle_invalid";
            return result;
        }
        if (gamma.gamma<minimum) {
            minimum=gamma.gamma;
            result.limiting_owner=owner;
            result.current_gamma=gamma.gamma;
            result.gamma_witness={gamma.accelX,gamma.accelY};
            result.dominant_row=dominantCanonicalOwnerRow(
                rows,owner,result.gamma_witness);
        }
    }
    result.hard_polytope=diagnoseHardPolytope(
        result.limiting_owner,time_s,mode,topology,rows,input_half_box);
    result.valid=true;
    result.reason="failure_snapshot_audited";
    return result;
}

}  // namespace gf
