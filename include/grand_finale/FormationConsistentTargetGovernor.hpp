#pragma once

#include "grand_finale/Types.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <stdexcept>
#include <vector>

namespace gf {

struct FormationTargetSlotIdentity {
    std::size_t branch=0;
    std::size_t local_rank=0;
    bool operator==(const FormationTargetSlotIdentity& other) const {
        return branch==other.branch && local_rank==other.local_rank;
    }
    bool operator!=(const FormationTargetSlotIdentity& other) const {
        return !(*this==other);
    }
};

using FormationTargetLedger=std::map<NodeId,Eigen::Vector2d>;

struct TargetSegmentMinimum {
    double distance_m=std::numeric_limits<double>::infinity();
    double s=0.0;
};

inline TargetSegmentMinimum minimumTargetTransitionSeparation(
    const Eigen::Vector2d& first_start,const Eigen::Vector2d& first_end,
    const Eigen::Vector2d& second_start,const Eigen::Vector2d& second_end) {
    TargetSegmentMinimum result;
    const Eigen::Vector2d relative_start=first_start-second_start;
    const Eigen::Vector2d relative_delta=
        (first_end-first_start)-(second_end-second_start);
    const double denominator=relative_delta.squaredNorm();
    result.s=denominator<=1e-24?0.0:std::clamp(
        -relative_start.dot(relative_delta)/denominator,0.0,1.0);
    result.distance_m=(relative_start+result.s*relative_delta).norm();
    return result;
}

inline FormationTargetLedger interpolateFormationTargetLedger(
    const FormationTargetLedger& committed,
    const FormationTargetLedger& candidate,double alpha) {
    if (!std::isfinite(alpha) || alpha<0.0 || alpha>1.0)
        throw std::invalid_argument("invalid target interpolation alpha");
    FormationTargetLedger result;
    for (const auto& [owner,current] : committed) {
        const auto found=candidate.find(owner);
        if (found==candidate.end())
            throw std::invalid_argument("incomplete target candidate");
        result.emplace(owner,(1.0-alpha)*current+alpha*found->second);
    }
    if (result.size()!=candidate.size())
        throw std::invalid_argument("target candidate owner mismatch");
    return result;
}

inline bool targetReferenceDistancesValid(
    const FormationTargetLedger& ledger,
    const std::map<NodeId,Eigen::Vector2d>& fixed_targets,
    const std::vector<DirectedEdge>& edges,double maximum_distance_m,
    double tolerance_m) {
    if (!std::isfinite(maximum_distance_m) || maximum_distance_m<=0.0 ||
        !std::isfinite(tolerance_m) || tolerance_m<0.0) return false;
    for (const auto& edge : edges) {
        const auto owner=ledger.find(edge.owner);
        const auto reference=ledger.find(edge.reference);
        const auto fixed=fixed_targets.find(edge.reference);
        if (owner==ledger.end() ||
            (reference==ledger.end() && fixed==fixed_targets.end()) ||
            !owner->second.allFinite()) return false;
        const Eigen::Vector2d reference_position=reference!=ledger.end()
            ?reference->second:fixed->second;
        if (!reference_position.allFinite() ||
            (owner->second-reference_position).norm()>
                maximum_distance_m+tolerance_m) return false;
    }
    return true;
}

inline bool targetReferenceDistancesValid(
    const FormationTargetLedger& ledger,
    const std::vector<DirectedEdge>& edges,double maximum_distance_m,
    double tolerance_m) {
    return targetReferenceDistancesValid(
        ledger,{},edges,maximum_distance_m,tolerance_m);
}

struct FormationTargetTransitionRequest {
    std::vector<NodeId> mobile_ids;
    FormationTargetLedger committed;
    FormationTargetLedger candidate;
    std::map<NodeId,FormationTargetSlotIdentity> committed_slots;
    std::map<NodeId,FormationTargetSlotIdentity> candidate_slots;
    std::map<NodeId,Eigen::Vector2d> fixed_targets;
    std::vector<DirectedEdge> reference_edges;
    double reference_plan_distance_m=849.0;
    double minimum_segment_separation_m=0.1;
    double minimum_endpoint_separation_m=0.1;
    double feasibility_tolerance_m=1e-9;
};

inline const std::vector<double>& frozenTargetTransitionAlphaLadder() {
    static const std::vector<double> ladder={1.0,0.5,0.25,0.125,0.0625};
    return ladder;
}

inline bool fixedGraphReferenceEndpointsValid(
    const FormationTargetTransitionRequest& request) {
    return targetReferenceDistancesValid(
               request.committed,request.fixed_targets,request.reference_edges,
               request.reference_plan_distance_m,
               request.feasibility_tolerance_m) &&
           targetReferenceDistancesValid(
               request.candidate,request.fixed_targets,request.reference_edges,
               request.reference_plan_distance_m,
               request.feasibility_tolerance_m);
}

struct FormationTargetTransitionAudit {
    bool valid=false;
    std::string reason;
    double selected_alpha=0.0;
    FormationTargetLedger selected;
    double minimum_segment_separation_m=
        std::numeric_limits<double>::infinity();
    double minimum_endpoint_separation_m=
        std::numeric_limits<double>::infinity();
    std::size_t audited_same_branch_pairs=0;
    std::size_t audited_cross_branch_pairs=0;
    std::vector<double> evaluated_alphas;
};

inline FormationTargetTransitionAudit selectFormationConsistentTargetTransition(
    const FormationTargetTransitionRequest& request) {
    FormationTargetTransitionAudit result;
    result.selected=request.committed;
    if (request.mobile_ids.empty() ||
        request.committed.size()!=request.mobile_ids.size() ||
        request.candidate.size()!=request.mobile_ids.size() ||
        request.committed_slots.size()!=request.mobile_ids.size() ||
        request.candidate_slots.size()!=request.mobile_ids.size() ||
        !std::isfinite(request.minimum_segment_separation_m) ||
        request.minimum_segment_separation_m<0.0 ||
        !std::isfinite(request.minimum_endpoint_separation_m) ||
        request.minimum_endpoint_separation_m<0.0 ||
        !std::isfinite(request.feasibility_tolerance_m) ||
        request.feasibility_tolerance_m<0.0) {
        result.reason="target_transition_blocked:invalid_ledger";
        return result;
    }
    std::set<NodeId> ids;
    std::set<std::pair<std::size_t,std::size_t>> slots;
    for (NodeId owner : request.mobile_ids) {
        const auto current=request.committed.find(owner);
        const auto candidate=request.candidate.find(owner);
        const auto current_slot=request.committed_slots.find(owner);
        const auto candidate_slot=request.candidate_slots.find(owner);
        if (!ids.insert(owner).second || current==request.committed.end() ||
            candidate==request.candidate.end() ||
            current_slot==request.committed_slots.end() ||
            candidate_slot==request.candidate_slots.end() ||
            !current->second.allFinite() || !candidate->second.allFinite()) {
            result.reason="target_transition_blocked:invalid_ledger";
            return result;
        }
        if (current_slot->second!=candidate_slot->second) {
            result.reason="target_transition_blocked:slot_identity";
            return result;
        }
        if (!slots.insert({current_slot->second.branch,
                           current_slot->second.local_rank}).second) {
            result.reason="target_transition_blocked:duplicate_slot";
            return result;
        }
    }
    for (std::size_t first=0;first<request.mobile_ids.size();++first)
        for (std::size_t second=first+1;second<request.mobile_ids.size();++second) {
            const NodeId first_id=request.mobile_ids[first];
            const NodeId second_id=request.mobile_ids[second];
            const auto first_slot=request.committed_slots.at(first_id);
            const auto second_slot=request.committed_slots.at(second_id);
            if (first_slot.branch==second_slot.branch) {
                ++result.audited_same_branch_pairs;
                if (first_slot.local_rank!=second_slot.local_rank) {
                    const bool first_precedes=
                        first_slot.local_rank<second_slot.local_rank;
                    const Eigen::Vector2d committed_order=first_precedes
                        ?request.committed.at(second_id)-request.committed.at(first_id)
                        :request.committed.at(first_id)-request.committed.at(second_id);
                    const Eigen::Vector2d candidate_order=first_precedes
                        ?request.candidate.at(second_id)-request.candidate.at(first_id)
                        :request.candidate.at(first_id)-request.candidate.at(second_id);
                    if (committed_order.norm()<=request.feasibility_tolerance_m ||
                        candidate_order.dot(committed_order.normalized())<=
                            request.feasibility_tolerance_m) {
                        result.reason="target_transition_blocked:slot_order";
                        return result;
                    }
                }
            } else ++result.audited_cross_branch_pairs;
        }
    for (double alpha : frozenTargetTransitionAlphaLadder()) {
        result.evaluated_alphas.push_back(alpha);
        const auto provisional=interpolateFormationTargetLedger(
            request.committed,request.candidate,alpha);
        if (!targetReferenceDistancesValid(
                provisional,request.fixed_targets,request.reference_edges,
                request.reference_plan_distance_m,
                request.feasibility_tolerance_m)) {
            result.reason="target_transition_blocked:reference_distance";
            continue;
        }
        double segment_min=std::numeric_limits<double>::infinity();
        double endpoint_min=std::numeric_limits<double>::infinity();
        for (std::size_t first=0;first<request.mobile_ids.size();++first)
            for (std::size_t second=first+1;
                 second<request.mobile_ids.size();++second) {
                const NodeId first_id=request.mobile_ids[first];
                const NodeId second_id=request.mobile_ids[second];
                segment_min=std::min(segment_min,
                    minimumTargetTransitionSeparation(
                        request.committed.at(first_id),
                        provisional.at(first_id),
                        request.committed.at(second_id),
                        provisional.at(second_id)).distance_m);
                endpoint_min=std::min(endpoint_min,
                    (provisional.at(first_id)-provisional.at(second_id)).norm());
            }
        if (segment_min+request.feasibility_tolerance_m<
            request.minimum_segment_separation_m) {
            result.reason="target_transition_blocked:segment_separation";
            continue;
        }
        if (endpoint_min+request.feasibility_tolerance_m<
            request.minimum_endpoint_separation_m) {
            result.reason="target_transition_blocked:endpoint_collision";
            continue;
        }
        result.valid=true;
        result.reason="target_transition_geometry_passed";
        result.selected_alpha=alpha;
        result.selected=provisional;
        result.minimum_segment_separation_m=segment_min;
        result.minimum_endpoint_separation_m=endpoint_min;
        return result;
    }
    return result;
}

struct TargetTransitionBlockState {
    std::size_t blocked_count=0;
    bool terminate=false;
};

inline TargetTransitionBlockState nextTargetTransitionBlockState(
    std::size_t previous_count,std::size_t maximum_count) {
    if (maximum_count==0)
        throw std::invalid_argument("target block limit must be positive");
    TargetTransitionBlockState result{previous_count+1,false};
    result.terminate=result.blocked_count>=maximum_count;
    return result;
}

}  // namespace gf
