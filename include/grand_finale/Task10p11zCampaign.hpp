#pragma once

#include "grand_finale/Task10p11yEarlyPrevention.hpp"
#include "grand_finale/Task10p11zComponentOracle.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <set>
#include <string>
#include <vector>

namespace gf {

struct Task10p11zPreregisteredCampaign {
    std::vector<std::string> profile_order{"R0","R1","R2","R3","R4"};
    double r0_tau_mps2=14.0;
    double r1_tau_mps2=14.0;
    double r2_tau_mps2=18.0;
    double r3_tau_mps2=14.0;
    std::size_t candidate_count=9;
    std::size_t component_size_limit=3;
    std::set<NodeId> gate1_component{2,4,6};
    std::string r4_status="offline_rejected";
    bool gamma_star_is_diagnostic_not_parameter=true;
};

inline Task10p11zPreregisteredCampaign task10p11zPreregisteredCampaign() {
    return {};
}

struct Task10p11zAdaptiveComponentDecision {
    bool valid=false;
    bool use_component=false;
    bool local_current_feasible=false;
    bool local_successor_feasible=false;
    std::string reason;
    std::set<NodeId> component;
    std::map<NodeId,Eigen::Vector2d> controls;
    double current_full_residual_mps2=0.0;
    double successor_full_residual_mps2=0.0;
    std::vector<std::set<NodeId>> attempted_components;
};

inline Task10p11zAdaptiveComponentDecision
task10p11zDecideAdaptiveComponent(const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& baseline_controls,
    std::size_t component_size_limit) {
    Task10p11zAdaptiveComponentDecision result;
    try {
        const auto request=task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto rows=buildCanonicalHardRows(request);
        if (baseline_controls.size()!=request.mobile_ids.size()) {
            result.reason="baseline_control_map_incomplete";
            return result;
        }
        const auto baseline_pair=solveTask10p11tDynamicPair(
            rows,request.mobile_ids,baseline_controls,
            request.acceleration_half_box,task10p11y_detail::kPair);
        result.local_current_feasible=baseline_pair.valid &&
            baseline_pair.shared_interval.feasible;
        const auto baseline_successor=task10p11y_detail::successorAudit(
            snapshot,baseline_controls);
        result.local_successor_feasible=baseline_successor.valid &&
            baseline_successor.signed_transfer_interval &&
            baseline_successor.all_local_qps &&
            baseline_successor.signed_transfer_full_rows &&
            baseline_successor.full_pair_28d;
        if (result.local_current_feasible &&
            result.local_successor_feasible) {
            result.valid=true;
            result.use_component=false;
            result.reason="distributed_current_and_successor_feasible";
            result.controls=baseline_controls;
            result.current_full_residual_mps2=
                task10p11y_detail::fullRowResidual(
                    rows,request,baseline_controls);
            result.successor_full_residual_mps2=
                baseline_successor.full_pair_28d_residual;
            return result;
        }
        if (component_size_limit<2 || component_size_limit>=14) {
            result.reason="invalid_component_size_limit";
            return result;
        }
        const auto problem=buildTask10p11sRows28d(
            rows,request.mobile_ids,true);
        const auto nominal=task10p11sOrderedControls(
            request.mobile_ids,
            task10p11s_capture_detail::nominalFromJson(
                snapshot.at("nominal_controls")));
        const auto frozen=task10p11sOrderedControls(
            request.mobile_ids,baseline_controls);
        const auto graph=task10p11z_component_detail::coupledGraph(problem);
        for (std::size_t size=2;size<=component_size_limit;++size) {
            for (const auto& component:
                 task10p11z_component_detail::connectedComponentsAtSize(
                     request.mobile_ids,size,graph)) {
                result.attempted_components.push_back(component);
                const auto current=task10p11w_detail::solveRestricted(
                    problem,nominal,frozen,component,false);
                if (!current.feasible ||
                    current.minimum_residual< -1.0e-8) continue;
                const auto successor=task10p11w_detail::successorAudit(
                    snapshot,current.controls,component,request.mobile_ids);
                if (!successor.component.feasible ||
                    successor.component.minimum_residual< -1.0e-8 ||
                    !successor.full.feasible ||
                    successor.full.minimum_residual< -1.0e-8) continue;
                result.valid=true;
                result.use_component=true;
                result.reason="certificate_ordered_component_selected";
                result.component=component;
                result.controls=task10p11sControlMap(
                    request.mobile_ids,current.controls);
                result.current_full_residual_mps2=
                    current.minimum_residual;
                result.successor_full_residual_mps2=
                    successor.component.minimum_residual;
                return result;
            }
        }
        result.reason="no_current_and_successor_component_within_frozen_limit";
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    return result;
}

inline nlohmann::json task10p11zAdaptiveComponentDecisionJson(
    const Task10p11zAdaptiveComponentDecision& value) {
    nlohmann::json attempts=nlohmann::json::array();
    for (const auto& component:value.attempted_components)
        attempts.push_back(task10p11w_detail::idsJson(component));
    return {{"valid",value.valid},{"use_component",value.use_component},
        {"local_current_feasible",value.local_current_feasible},
        {"local_successor_feasible",value.local_successor_feasible},
        {"reason",value.reason},
        {"component",task10p11w_detail::idsJson(value.component)},
        {"current_full_residual_mps2",value.current_full_residual_mps2},
        {"successor_full_residual_mps2",
            value.successor_full_residual_mps2},
        {"attempted_components",std::move(attempts)}};
}

}  // namespace gf
