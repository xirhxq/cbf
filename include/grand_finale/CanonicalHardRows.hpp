#pragma once

#include "bridge/ExactGammaStar2D.hpp"
#include "cbf/PairwiseSecondOrderCBF.hpp"
#include "grand_finale/SnapshotRobustPairRow.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"
#include "grand_finale/Types.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gf {

enum class CanonicalHardRowKind {
    ReferenceDistance,
    Collision,
    Workspace,
    SpeedLimit,
    PlantSpeedAppliedControl,
    InputBox,
    Auxiliary
};

struct WorkspaceFacet2D {
    std::string id;
    Eigen::Vector2d outward_normal = Eigen::Vector2d::Zero();
    double offset_m = 0.0;
};

enum class WorkspaceClassK {
    Linear,
    RegularizedBraking
};

inline double workspaceBrakingAlpha1(
    double h,double braking_acceleration_mps2,double regularization_m) {
    if (!std::isfinite(h) || !std::isfinite(braking_acceleration_mps2) ||
        !std::isfinite(regularization_m) ||
        braking_acceleration_mps2<=0.0 || regularization_m<=0.0)
        throw std::invalid_argument("invalid workspace braking class-K");
    const double nonnegative_h=std::max(0.0,h);
    return std::sqrt(2.0*braking_acceleration_mps2*
               (nonnegative_h+regularization_m))-
           std::sqrt(2.0*braking_acceleration_mps2*regularization_m);
}

struct SingleSnapshotTube2D {
    double position_radius_m = 0.0;
    double velocity_radius_mps = 0.0;
    SnapshotTubeProvenance provenance =
        SnapshotTubeProvenance::CovarianceSigmaDevelopment;
};

struct CanonicalHardRow {
    std::string id;
    CanonicalHardRowKind kind = CanonicalHardRowKind::Auxiliary;
    NodeId owner = 0;
    std::optional<NodeId> peer;
    Eigen::Vector2d normal = Eigen::Vector2d::Zero();
    Eigen::Vector2d control_coefficient = Eigen::Vector2d::Zero();
    double constant = 0.0;
    double responsibility = 1.0;
    bool participates_in_gamma = true;
    double barrier_h = std::numeric_limits<double>::infinity();
    double barrier_psi1 = std::numeric_limits<double>::infinity();
    double barrier_hdot = std::numeric_limits<double>::infinity();
    double coefficient_uncertainty_reserve = 0.0;
    std::optional<SnapshotTubeProvenance> tube_provenance;
    double position_uncertainty_reserve_m = 0.0;
    double velocity_uncertainty_reserve_mps = 0.0;

    double margin(const Eigen::Vector2d& control) const {
        return control_coefficient.dot(control) + constant;
    }
};

struct CanonicalHardRowRequest {
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::map<NodeId, PairwiseSecondOrderState2D> states;
    std::vector<DirectedEdge> reference_edges;
    std::vector<UndirectedEdge> collision_pairs;
    PairwiseSecondOrderRowSpec reference_spec;
    PairwiseSecondOrderRowSpec collision_spec;
    double acceleration_half_box = 0.0;
    // A non-positive value disables speed-domain construction. Formal V1
    // pairs this limit with plant_speed_facet_count=64; the scalar gain is
    // consumed only by explicit historical SpeedLimit fixtures.
    double speed_limit_mps = 0.0;
    double speed_cbf_gain = 1.0;
    // A positive facet count selects the formal plant-level speed applied-
    // control domain.  Zero retains the historical speed-CBF fixture path.
    std::size_t plant_speed_facet_count = 0;
    double plant_speed_dt_s = 0.0;
    bool require_snapshot_robust_rows = false;
    std::map<std::string, PairwiseSnapshotTube> reference_snapshot_tubes;
    std::map<std::string, PairwiseSnapshotTube> collision_snapshot_tubes;
    std::vector<WorkspaceFacet2D> workspace_facets;
    std::map<NodeId, SingleSnapshotTube2D> workspace_snapshot_tubes;
    std::map<NodeId, SingleSnapshotTube2D> speed_snapshot_tubes;
    std::map<NodeId, SingleSnapshotTube2D> plant_speed_snapshot_tubes;
    WorkspaceClassK workspace_class_k=WorkspaceClassK::Linear;
    double workspace_alpha1_gain=1.0;
    double workspace_alpha2_gain=1.0;
    double workspace_braking_acceleration_mps2=4.0;
    double workspace_braking_regularization_m=1.0;
    // Task 13 B0-b: replace the second-order lambda-form reference/
    // collision rows with velocity-augmented braking-slack rows
    // sqrt(2*share*a*(max(h,0)+eps)) - v_closing >= 0 (researcher-
    // approved; rows stay affine in u, gamma* machinery untouched).
    bool velocity_augmented_rows = false;
    // Task 18 causal split.  These gates only refine the master switch above;
    // both default true so every pre-Task18 caller retains identical rows.
    bool velocity_augmented_reference_rows = true;
    bool velocity_augmented_collision_rows = true;
    double row_slack_epsilon_m = 0.5;
};

inline CanonicalHardRow makeCanonicalGammaRow(
    std::string id,
    NodeId owner,
    Eigen::Vector2d control_coefficient,
    double constant) {
    if (id.empty() || !control_coefficient.allFinite() ||
        !std::isfinite(constant)) {
        throw std::invalid_argument("canonical gamma row must be finite and named");
    }
    return CanonicalHardRow{
        std::move(id), CanonicalHardRowKind::Auxiliary, owner, std::nullopt,
        control_coefficient, control_coefficient, constant, 1.0, true};
}

namespace canonical_hard_row_detail {

inline void canonicalizeNodes(std::vector<NodeId>& ids, const char* kind) {
    std::sort(ids.begin(), ids.end());
    if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
        throw std::invalid_argument(std::string("duplicate ") + kind + " node");
    }
}

inline CanonicalHardRow physicalRow(
    std::string id,
    CanonicalHardRowKind kind,
    NodeId owner,
    NodeId peer,
    const Eigen::Vector2d& normal,
    const Eigen::Vector2d& coefficient,
    double central_constant,
    double responsibility,
    double barrier_h,
    double barrier_psi1) {
    return CanonicalHardRow{
        std::move(id), kind, owner, peer, normal, coefficient,
        responsibility * central_constant, responsibility, true,
        barrier_h, barrier_psi1};
}

inline void appendSharedPairRows(
    std::vector<CanonicalHardRow>& rows,
    const std::string& prefix,
    CanonicalHardRowKind kind,
    NodeId first,
    NodeId second,
    const PairwiseSecondOrderState2D& first_state,
    const PairwiseSecondOrderState2D& second_state,
    const PairwiseSecondOrderRowSpec& spec) {
    PairwiseSecondOrderState2D zero_first = first_state;
    PairwiseSecondOrderState2D zero_second = second_state;
    zero_first.acceleration.setZero();
    zero_second.acceleration.setZero();
    const auto central = buildPairwiseSecondOrderRow(
        zero_first, zero_second, spec);
    const auto kinematics = computePairwiseDistanceKinematics(
        zero_first.position, zero_second.position,
        zero_first.velocity, zero_second.velocity);
    rows.push_back(physicalRow(
        prefix + ":owner:" + std::to_string(first), kind,
        first, second, kinematics.normal, central.uCoe,
        central.constTerm, 0.5, central.h, central.psi1));
    rows.push_back(physicalRow(
        prefix + ":owner:" + std::to_string(second), kind,
        second, first, -kinematics.normal, -central.uCoe,
        central.constTerm, 0.5, central.h, central.psi1));
}

inline void appendFixedPairRow(
    std::vector<CanonicalHardRow>& rows,
    const std::string& prefix,
    CanonicalHardRowKind kind,
    NodeId mobile,
    NodeId fixed,
    const PairwiseSecondOrderState2D& mobile_state,
    const PairwiseSecondOrderState2D& fixed_state,
    const PairwiseSecondOrderRowSpec& spec) {
    PairwiseSecondOrderState2D zero_mobile = mobile_state;
    PairwiseSecondOrderState2D zero_fixed = fixed_state;
    zero_mobile.acceleration.setZero();
    zero_fixed.acceleration.setZero();
    const auto central = buildPairwiseSecondOrderRow(
        zero_mobile, zero_fixed, spec);
    const auto kinematics = computePairwiseDistanceKinematics(
        zero_mobile.position, zero_fixed.position,
        zero_mobile.velocity, zero_fixed.velocity);
    rows.push_back(physicalRow(
        prefix + ":owner:" + std::to_string(mobile), kind,
        mobile, fixed, kinematics.normal, central.uCoe,
        central.constTerm, 1.0, central.h, central.psi1));
}

inline CanonicalHardRow robustPhysicalRow(
    std::string id,
    CanonicalHardRowKind kind,
    NodeId owner,
    NodeId peer,
    const Eigen::Vector2d& normal,
    const Eigen::Vector2d& coefficient,
    double constant,
    double responsibility,
    const SnapshotRobustPairRow& robust) {
    CanonicalHardRow row{
        std::move(id), kind, owner, peer, normal, coefficient,
        constant, responsibility, true,
        robust.barrier_h_lower, robust.barrier_psi1_lower};
    row.barrier_hdot = robust.barrier_hdot_lower;
    row.coefficient_uncertainty_reserve = robust.coefficient_reserve;
    row.tube_provenance = robust.provenance;
    row.position_uncertainty_reserve_m =
        robust.position_uncertainty_reserve_m;
    row.velocity_uncertainty_reserve_mps =
        robust.velocity_uncertainty_reserve_mps;
    return row;
}

// Task 13 B0-b v2 (relative-closing form, researcher-approved
// 2026-09-01): the row bounds the owner's velocity RELATIVE to its peer
// along the pair axis,
//   n_out*(v_own - v_other + dt*u_own) <= bound   (reference upper)
//   n_out*(v_own - v_other + dt*u_own) >= -bound  (collision lower)
// with bound = sqrt(2*share*a*(max(h,0)+eps)) - velocity tube and the
// peer velocity as a tick constant.  Cooperation-semantics boundary
// (preregistered): the pairwise guarantee holds when BOTH owners honor
// the mirror rows; a non-honoring (adversarial) peer is out of scope,
// as is the classic half-row form.  With both mirror rows honored the
// relative closing speed stays below sqrt(2*share*a*(h+eps)) while the
// pair can shed relative speed at 2a (share=0.5, both braking), giving
// stop distance (2 share a h)/(4a) = share*h/2 <= h.  The 1/2-1/2
// correspondence: each owner carries the mirror responsibility for its
// own velocity; neither row conditions on the peer's control.
inline CanonicalHardRow slackPhysicalRow(
    const std::string& id,CanonicalHardRowKind kind,NodeId owner,
    NodeId reference,const Eigen::Vector2d& n_out,
    const PairwiseSecondOrderState2D& own_state,
    const Eigen::Vector2d& other_velocity,double h_robust_m,
    double tube_velocity_mps,double acceleration_half_box,double share,
    double slack_epsilon_m,double dt_s) {
    const double h=std::max(h_robust_m,0.0)+slack_epsilon_m;
    const double bound=std::max(0.0,std::sqrt(2.0*share*
        acceleration_half_box*h)-tube_velocity_mps);
    const double closing=n_out.dot(own_state.velocity-other_velocity);
    const bool collision=kind==CanonicalHardRowKind::Collision;
    const Eigen::Vector2d coefficient=collision
        ?(dt_s*n_out):(-dt_s*n_out);
    const double constant=collision
        ?(bound+closing):(bound-closing);
    CanonicalHardRow row{id,kind,owner,std::nullopt,n_out,coefficient,
        constant,1.0,true,h_robust_m,h_robust_m,
        std::numeric_limits<double>::infinity(),0.0};
    return row;
}

inline void appendRobustSharedPairRows(
    std::vector<CanonicalHardRow>& rows,
    const std::string& prefix,
    CanonicalHardRowKind kind,
    NodeId first,
    NodeId second,
    const PairwiseSecondOrderState2D& first_state,
    const PairwiseSecondOrderState2D& second_state,
    const PairwiseSecondOrderRowSpec& spec,
    const PairwiseSnapshotTube& tube,
    double acceleration_half_box,
    bool velocity_augmented=false,double slack_epsilon_m=0.5,
    double dt_s=0.1) {
    const auto robust = buildSnapshotRobustPairRow(
        first_state, second_state, spec, tube, acceleration_half_box);
    if (velocity_augmented) {
        // Task 13 B0-b v2: relative-closing braking-slack rows (peer
        // velocity as a tick constant).  Derivation preregistered: with
        // both mirror rows honored the pair closing speed stays below
        // sqrt(2*share*a*(h+eps)) while the pair sheds relative speed at
        // 2*share*a, giving stop distance share*h/2 <= h; the 1/2-1/2
        // correspondence assigns each owner the mirror responsibility for
        // its own velocity.  Rows stay affine in the owner's own control.
        const double share=0.5;
        rows.push_back(slackPhysicalRow(
            prefix + ":owner:" + std::to_string(first), kind,
            first, second, robust.nominal_normal,
            first_state, second_state.velocity,
            robust.barrier_h_lower,
            robust.velocity_uncertainty_reserve_mps,
            acceleration_half_box, share, slack_epsilon_m, dt_s));
        rows.push_back(slackPhysicalRow(
            prefix + ":owner:" + std::to_string(second), kind,
            second, first, -robust.nominal_normal,
            second_state, first_state.velocity,
            robust.barrier_h_lower,
            robust.velocity_uncertainty_reserve_mps,
            acceleration_half_box, share, slack_epsilon_m, dt_s));
        return;
    }
    rows.push_back(robustPhysicalRow(
        prefix + ":owner:" + std::to_string(first), kind,
        first, second, robust.nominal_normal,
        robust.nominal_control_coefficient,
        0.5 * robust.central_constant_lower - robust.coefficient_reserve,
        0.5, robust));
    rows.push_back(robustPhysicalRow(
        prefix + ":owner:" + std::to_string(second), kind,
        second, first, -robust.nominal_normal,
        -robust.nominal_control_coefficient,
        0.5 * robust.central_constant_lower - robust.coefficient_reserve,
        0.5, robust));
}

inline void appendRobustFixedPairRow(
    std::vector<CanonicalHardRow>& rows,
    const std::string& prefix,
    CanonicalHardRowKind kind,
    NodeId mobile,
    NodeId fixed,
    const PairwiseSecondOrderState2D& mobile_state,
    const PairwiseSecondOrderState2D& fixed_state,
    const PairwiseSecondOrderRowSpec& spec,
    const PairwiseSnapshotTube& tube,
    double acceleration_half_box,
    bool velocity_augmented=false,double slack_epsilon_m=0.5,
    double dt_s=0.1) {
    const auto robust = buildSnapshotRobustPairRow(
        mobile_state, fixed_state, spec, tube, acceleration_half_box);
    if (velocity_augmented) {
        // Fixed pairs: the reference cannot move (peer velocity zero), so
        // the relative form degenerates to the full-responsibility
        // absolute bound sqrt(2*a*(h+eps)) (share=1.0).
        rows.push_back(slackPhysicalRow(
            prefix + ":owner:" + std::to_string(mobile), kind,
            mobile, fixed, robust.nominal_normal,
            mobile_state, Eigen::Vector2d::Zero(),
            robust.barrier_h_lower,
            robust.velocity_uncertainty_reserve_mps,
            acceleration_half_box, 1.0, slack_epsilon_m, dt_s));
        return;
    }
    rows.push_back(robustPhysicalRow(
        prefix + ":owner:" + std::to_string(mobile), kind,
        mobile, fixed, robust.nominal_normal,
        robust.nominal_control_coefficient,
        robust.central_constant_lower - robust.coefficient_reserve,
        1.0, robust));
}

}  // namespace canonical_hard_row_detail

inline std::vector<CanonicalHardRow> buildCanonicalHardRows(
    CanonicalHardRowRequest request,
    std::optional<NodeId> owner_filter = std::nullopt) {
    using namespace canonical_hard_row_detail;
    canonicalizeNodes(request.mobile_ids, "mobile");
    canonicalizeNodes(request.fixed_ids, "fixed");
    if (!std::isfinite(request.acceleration_half_box) ||
        request.acceleration_half_box <= 0.0) {
        throw std::invalid_argument("acceleration half box must be positive");
    }
    if (!std::isfinite(request.speed_limit_mps) ||
        !std::isfinite(request.speed_cbf_gain) ||
        request.speed_cbf_gain <= 0.0) {
        throw std::invalid_argument("speed CBF parameters must be finite");
    }
    if (!std::isfinite(request.workspace_alpha1_gain) ||
        request.workspace_alpha1_gain<=0.0 ||
        !std::isfinite(request.workspace_alpha2_gain) ||
        request.workspace_alpha2_gain<=0.0 ||
        !std::isfinite(request.workspace_braking_acceleration_mps2) ||
        request.workspace_braking_acceleration_mps2<=0.0 ||
        !std::isfinite(request.workspace_braking_regularization_m) ||
        request.workspace_braking_regularization_m<=0.0)
        throw std::invalid_argument("invalid workspace class-K parameters");
    if (request.plant_speed_facet_count>0 &&
        (request.plant_speed_facet_count!=64 ||
         !std::isfinite(request.plant_speed_dt_s) ||
         request.plant_speed_dt_s<=0.0 || request.speed_limit_mps<=0.0)) {
        throw std::invalid_argument("invalid plant-speed applied-control parameters");
    }
    const std::set<NodeId> mobiles(
        request.mobile_ids.begin(), request.mobile_ids.end());
    if (owner_filter.has_value() && mobiles.count(*owner_filter)==0)
        throw std::invalid_argument("canonical owner filter is not mobile");
    const std::set<NodeId> fixed(request.fixed_ids.begin(), request.fixed_ids.end());
    std::set<NodeId> known = mobiles;
    for (NodeId id : fixed) {
        if (!known.insert(id).second)
            throw std::invalid_argument("mobile and fixed nodes must be disjoint");
    }
    for (NodeId id : known) {
        if (request.states.count(id) == 0)
            throw std::invalid_argument("missing pairwise state");
    }
    for (NodeId id : fixed) {
        const auto& state = request.states.at(id);
        if (state.velocity.norm() != 0.0 || state.acceleration.norm() != 0.0)
            throw std::invalid_argument("fixed node kinematics must be zero");
    }
    if (request.reference_spec.kind !=
            PairwiseSecondOrderBarrierKind::CommunicationUpper ||
        request.collision_spec.kind !=
            PairwiseSecondOrderBarrierKind::CollisionLower) {
        throw std::invalid_argument("canonical hard-row barrier kinds are fixed");
    }

    std::vector<CanonicalHardRow> rows;
    std::set<std::string> workspace_ids;
    for (const WorkspaceFacet2D& raw_facet : request.workspace_facets) {
        if (raw_facet.id.empty() || !raw_facet.outward_normal.allFinite() ||
            !std::isfinite(raw_facet.offset_m) ||
            raw_facet.outward_normal.norm() <= 1e-12 ||
            !workspace_ids.insert(raw_facet.id).second) {
            throw std::invalid_argument("invalid or duplicate workspace facet");
        }
        const double norm = raw_facet.outward_normal.norm();
        const Eigen::Vector2d normal = raw_facet.outward_normal / norm;
        const double offset = raw_facet.offset_m / norm;
        for (NodeId owner : request.mobile_ids) {
            if (owner_filter.has_value() && owner!=*owner_filter) continue;
            const auto tube_it = request.workspace_snapshot_tubes.find(owner);
            if (request.require_snapshot_robust_rows &&
                tube_it == request.workspace_snapshot_tubes.end()) {
                throw std::invalid_argument("missing workspace snapshot tube");
            }
            const SingleSnapshotTube2D tube =
                tube_it == request.workspace_snapshot_tubes.end()
                    ? SingleSnapshotTube2D{} : tube_it->second;
            if (!std::isfinite(tube.position_radius_m) ||
                tube.position_radius_m < 0.0 ||
                !std::isfinite(tube.velocity_radius_mps) ||
                tube.velocity_radius_mps < 0.0) {
                throw std::invalid_argument("invalid workspace snapshot tube");
            }
            const auto& state = request.states.at(owner);
            const Eigen::Vector2d position(state.position.x,state.position.y);
            const double h = offset-normal.dot(position)-
                tube.position_radius_m;
            const double hdot = -normal.dot(state.velocity)-
                tube.velocity_radius_mps;
            double alpha1=0.0,alpha1_derivative=0.0;
            if (request.workspace_class_k==WorkspaceClassK::Linear) {
                alpha1=request.workspace_alpha1_gain*h;
                alpha1_derivative=request.workspace_alpha1_gain;
            } else {
                alpha1=workspaceBrakingAlpha1(h,
                    request.workspace_braking_acceleration_mps2,
                    request.workspace_braking_regularization_m);
                alpha1_derivative=
                    request.workspace_braking_acceleration_mps2/
                    std::sqrt(2.0*
                        request.workspace_braking_acceleration_mps2*
                        (std::max(0.0,h)+
                         request.workspace_braking_regularization_m));
            }
            const double psi1=hdot+alpha1;
            const double class_k_constant=alpha1_derivative*hdot+
                request.workspace_alpha2_gain*psi1;
            CanonicalHardRow row{
                "workspace:"+std::to_string(owner)+":"+raw_facet.id,
                CanonicalHardRowKind::Workspace,owner,std::nullopt,
                -normal,-normal,class_k_constant,1.0,true,h,psi1,hdot};
            row.tube_provenance = tube.provenance;
            row.position_uncertainty_reserve_m = tube.position_radius_m;
            row.velocity_uncertainty_reserve_mps = tube.velocity_radius_mps;
            rows.push_back(std::move(row));
        }
    }
    std::set<std::string> reference_ids;
    for (const DirectedEdge& edge : request.reference_edges) {
        if (mobiles.count(edge.owner) == 0 || known.count(edge.reference) == 0 ||
            !reference_ids.insert(edge.id()).second) {
            throw std::invalid_argument("invalid or duplicate reference edge");
        }
        const std::string prefix = "reference:" + edge.id();
        if (owner_filter.has_value() && edge.owner!=*owner_filter &&
            (mobiles.count(edge.reference)==0 ||
             edge.reference!=*owner_filter)) {
            continue;
        }
        const auto tube = request.reference_snapshot_tubes.find(edge.id());
        if (request.require_snapshot_robust_rows &&
            tube == request.reference_snapshot_tubes.end()) {
            throw std::invalid_argument("missing reference snapshot tube");
        }
        if (mobiles.count(edge.reference) != 0) {
            if (request.require_snapshot_robust_rows) {
                appendRobustSharedPairRows(
                    rows, prefix, CanonicalHardRowKind::ReferenceDistance,
                    edge.owner, edge.reference, request.states.at(edge.owner),
                    request.states.at(edge.reference), request.reference_spec,
                    tube->second, request.acceleration_half_box,
                    request.velocity_augmented_rows&&
                        request.velocity_augmented_reference_rows,
                    request.row_slack_epsilon_m, request.plant_speed_dt_s);
            } else {
                appendSharedPairRows(
                    rows, prefix, CanonicalHardRowKind::ReferenceDistance,
                    edge.owner, edge.reference, request.states.at(edge.owner),
                    request.states.at(edge.reference), request.reference_spec);
            }
        } else {
            if (request.require_snapshot_robust_rows) {
                appendRobustFixedPairRow(
                    rows, prefix, CanonicalHardRowKind::ReferenceDistance,
                    edge.owner, edge.reference, request.states.at(edge.owner),
                    request.states.at(edge.reference), request.reference_spec,
                    tube->second, request.acceleration_half_box,
                    request.velocity_augmented_rows&&
                        request.velocity_augmented_reference_rows,
                    request.row_slack_epsilon_m, request.plant_speed_dt_s);
            } else {
                appendFixedPairRow(
                    rows, prefix, CanonicalHardRowKind::ReferenceDistance,
                    edge.owner, edge.reference, request.states.at(edge.owner),
                    request.states.at(edge.reference), request.reference_spec);
            }
        }
    }

    std::set<std::string> collision_ids;
    for (const UndirectedEdge& raw_edge : request.collision_pairs) {
        const UndirectedEdge edge = UndirectedEdge::canonical(
            raw_edge.first, raw_edge.second);
        if (known.count(edge.first) == 0 || known.count(edge.second) == 0 ||
            (mobiles.count(edge.first) == 0 && mobiles.count(edge.second) == 0) ||
            !collision_ids.insert(edge.id()).second) {
            throw std::invalid_argument("invalid or duplicate collision pair");
        }
        const std::string prefix = "collision:" + edge.id();
        if (owner_filter.has_value() && edge.first!=*owner_filter &&
            edge.second!=*owner_filter) {
            continue;
        }
        const auto tube = request.collision_snapshot_tubes.find(edge.id());
        if (request.require_snapshot_robust_rows &&
            tube == request.collision_snapshot_tubes.end()) {
            throw std::invalid_argument("missing collision snapshot tube");
        }
        if (mobiles.count(edge.first) != 0 && mobiles.count(edge.second) != 0) {
            if (request.require_snapshot_robust_rows) {
                appendRobustSharedPairRows(
                    rows, prefix, CanonicalHardRowKind::Collision,
                    edge.first, edge.second, request.states.at(edge.first),
                    request.states.at(edge.second), request.collision_spec,
                    tube->second, request.acceleration_half_box,
                    request.velocity_augmented_rows&&
                        request.velocity_augmented_collision_rows,
                    request.row_slack_epsilon_m, request.plant_speed_dt_s);
            } else {
                appendSharedPairRows(
                    rows, prefix, CanonicalHardRowKind::Collision,
                    edge.first, edge.second, request.states.at(edge.first),
                    request.states.at(edge.second), request.collision_spec);
            }
        } else {
            const NodeId mobile = mobiles.count(edge.first) ? edge.first : edge.second;
            const NodeId anchor = mobile == edge.first ? edge.second : edge.first;
            if (request.require_snapshot_robust_rows) {
                appendRobustFixedPairRow(
                    rows, prefix, CanonicalHardRowKind::Collision,
                    mobile, anchor, request.states.at(mobile),
                    request.states.at(anchor), request.collision_spec,
                    tube->second, request.acceleration_half_box,
                    request.velocity_augmented_rows&&
                        request.velocity_augmented_collision_rows,
                    request.row_slack_epsilon_m, request.plant_speed_dt_s);
            } else {
                appendFixedPairRow(
                    rows, prefix, CanonicalHardRowKind::Collision,
                    mobile, anchor, request.states.at(mobile),
                    request.states.at(anchor), request.collision_spec);
            }
        }
    }

    for (NodeId owner : request.mobile_ids) {
        if (owner_filter.has_value() && owner!=*owner_filter) continue;
        if (request.speed_limit_mps>0.0 &&
            request.plant_speed_facet_count>0) {
            const auto tube_it=request.plant_speed_snapshot_tubes.find(owner);
            if (tube_it==request.plant_speed_snapshot_tubes.end())
                throw std::invalid_argument("missing plant-speed snapshot tube");
            const SingleSnapshotTube2D& tube=tube_it->second;
            const auto plant_rows=buildPlantSpeedAppliedControlRows({
                owner,request.states.at(owner).velocity,
                tube.velocity_radius_mps,request.speed_limit_mps,
                request.plant_speed_dt_s,request.plant_speed_facet_count,
                1.0e-9});
            for (const auto& plant:plant_rows) {
                CanonicalHardRow row{
                    plant.id,CanonicalHardRowKind::PlantSpeedAppliedControl,
                    owner,std::nullopt,plant.facet_normal,
                    plant.control_coefficient,plant.constant,1.0,true};
                row.tube_provenance=tube.provenance;
                row.velocity_uncertainty_reserve_mps=
                    plant.direction_velocity_support_mps;
                rows.push_back(std::move(row));
            }
        } else if (request.speed_limit_mps > 0.0) {
            const auto tube_it = request.speed_snapshot_tubes.find(owner);
            if (tube_it == request.speed_snapshot_tubes.end()) {
                throw std::invalid_argument("missing speed snapshot tube");
            }
            const SingleSnapshotTube2D& tube = tube_it->second;
            if (!std::isfinite(tube.velocity_radius_mps) ||
                tube.velocity_radius_mps < 0.0) {
                throw std::invalid_argument("invalid speed snapshot tube");
            }
            const Eigen::Vector2d velocity = request.states.at(owner).velocity;
            const double robust_speed = velocity.norm()+tube.velocity_radius_mps;
            const double robust_h =
                request.speed_limit_mps*request.speed_limit_mps-
                robust_speed*robust_speed;
            const Eigen::Vector2d coefficient = -2.0*velocity;
            const double coefficient_reserve =
                2.0*tube.velocity_radius_mps*
                request.acceleration_half_box*std::sqrt(2.0);
            CanonicalHardRow speed_row{
                "speed:"+std::to_string(owner),
                CanonicalHardRowKind::SpeedLimit, owner, std::nullopt,
                coefficient, coefficient,
                request.speed_cbf_gain*robust_h-coefficient_reserve,
                1.0, true, robust_h, robust_h,
                std::numeric_limits<double>::infinity(),
                coefficient_reserve};
            speed_row.tube_provenance = tube.provenance;
            speed_row.velocity_uncertainty_reserve_mps =
                tube.velocity_radius_mps;
            rows.push_back(std::move(speed_row));
        }
        const double bound = request.acceleration_half_box;
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ax:lower",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            Eigen::Vector2d::UnitX(), Eigen::Vector2d::UnitX(),
            bound, 1.0, false});
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ax:upper",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            -Eigen::Vector2d::UnitX(), -Eigen::Vector2d::UnitX(),
            bound, 1.0, false});
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ay:lower",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            Eigen::Vector2d::UnitY(), Eigen::Vector2d::UnitY(),
            bound, 1.0, false});
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ay:upper",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            -Eigen::Vector2d::UnitY(), -Eigen::Vector2d::UnitY(),
            bound, 1.0, false});
    }
    std::sort(rows.begin(), rows.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.id < rhs.id;
    });
    if (owner_filter.has_value()) {
        rows.erase(std::remove_if(rows.begin(),rows.end(),[&](const auto& row) {
            return row.owner!=*owner_filter;
        }),rows.end());
    }
    if (std::adjacent_find(rows.begin(), rows.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.id == rhs.id;
        }) != rows.end()) {
        throw std::invalid_argument("duplicate canonical hard-row id");
    }
    return rows;
}

inline std::vector<CanonicalHardRow> buildCanonicalOwnerHardRows(
    CanonicalHardRowRequest request,NodeId owner) {
    return buildCanonicalHardRows(std::move(request),owner);
}

inline BridgeGammaStarSolution2D solveCanonicalGammaStar(
    const std::vector<CanonicalHardRow>& rows,
    NodeId owner,
    double acceleration_half_box) {
    std::vector<BridgeGammaStarResidual2D> residuals;
    for (const CanonicalHardRow& row : rows) {
        if (row.owner != owner || !row.participates_in_gamma) continue;
        residuals.push_back(bridgeGammaStarResidualFromAffineMargin(
            row.control_coefficient.x(), row.control_coefficient.y(),
            row.constant));
    }
    return solveExactBridgeGammaStar2D(residuals, acceleration_half_box);
}

}  // namespace gf
