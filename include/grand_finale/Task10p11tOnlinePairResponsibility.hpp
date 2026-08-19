#pragma once

#include "grand_finale/DynamicPairCertifiedControl.hpp"
#include "grand_finale/Task10p11rFailureWindow.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace gf {

struct Task10p11tOnlineConflictDiagnostic {
    bool valid = false;
    bool infeasible = false;
    std::string reason;
    NodeId limiting_owner = 0;
    double minimum_legacy_gamma_mps2 =
        std::numeric_limits<double>::infinity();
    std::vector<NodeId> infeasible_owners;
    std::vector<std::string> mobile_pair_base_ids;
};

inline bool task10p11tCoherentOnlinePair(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids,
    const std::string& base_id) {
    std::vector<const CanonicalHardRow*> selected;
    for (const CanonicalHardRow& row : rows) {
        const bool pair_kind =
            row.kind == CanonicalHardRowKind::ReferenceDistance ||
            row.kind == CanonicalHardRowKind::Collision;
        const bool mobile_peer = row.peer.has_value() &&
            std::find(mobile_ids.begin(), mobile_ids.end(), *row.peer) !=
                mobile_ids.end();
        if (pair_kind && mobile_peer && dynamicPairBaseId(row.id) == base_id)
            selected.push_back(&row);
    }
    if (selected.size() != 2) return false;
    const auto& first = *selected[0];
    const auto& second = *selected[1];
    return first.peer.has_value() && second.peer.has_value() &&
        first.owner == *second.peer && second.owner == *first.peer &&
        first.kind == second.kind &&
        std::abs(first.responsibility - 0.5) <= 1e-12 &&
        std::abs(second.responsibility - 0.5) <= 1e-12 &&
        (first.control_coefficient + second.control_coefficient).norm() <=
            1e-9 &&
        std::abs(first.constant - second.constant) <= 1e-8 &&
        std::abs(first.coefficient_uncertainty_reserve -
                 second.coefficient_uncertainty_reserve) <= 1e-10;
}

inline Task10p11tOnlineConflictDiagnostic diagnoseTask10p11tOnlineConflicts(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids,
    double time_s,
    SupervisorMode mode,
    const std::vector<DirectedEdge>& topology,
    double acceleration_half_box) {
    Task10p11tOnlineConflictDiagnostic result;
    if (mobile_ids.size() != 14 || !std::isfinite(time_s) || time_s < 0.0 ||
        !std::isfinite(acceleration_half_box) || acceleration_half_box <= 0.0) {
        result.reason = "invalid_online_conflict_diagnostic_request";
        return result;
    }
    std::set<std::string> pair_bases;
    for (NodeId owner : mobile_ids) {
        const auto gamma = solveCanonicalGammaStar(
            rows, owner, acceleration_half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma)) {
            result.reason = "online_conflict_gamma_invalid";
            return result;
        }
        if (gamma.gamma < result.minimum_legacy_gamma_mps2) {
            result.minimum_legacy_gamma_mps2 = gamma.gamma;
            result.limiting_owner = owner;
        }
        if (gamma.gamma >= 0.0) continue;
        result.infeasible_owners.push_back(owner);
        const auto certificate = diagnoseHardPolytope(
            owner, time_s, mode, topology, rows, acceleration_half_box);
        for (const std::string& row_id : certificate.minimal_conflict_row_ids) {
            const auto row = std::find_if(rows.begin(), rows.end(),
                [&](const CanonicalHardRow& value) {
                    return value.id == row_id;
                });
            if (row == rows.end() || !row->peer.has_value() ||
                std::find(mobile_ids.begin(), mobile_ids.end(), *row->peer) ==
                    mobile_ids.end()) continue;
            const std::string base_id = dynamicPairBaseId(row->id);
            if (!base_id.empty() && task10p11tCoherentOnlinePair(
                    rows, mobile_ids, base_id)) {
                pair_bases.insert(base_id);
            }
        }
    }
    result.infeasible = !result.infeasible_owners.empty();
    result.mobile_pair_base_ids.assign(pair_bases.begin(), pair_bases.end());
    result.valid = true;
    result.reason = result.infeasible
        ? "online_legacy_half_infeasible" : "online_legacy_half_feasible";
    return result;
}

inline std::optional<std::string> task10p11tUniqueOnlinePair(
    const Task10p11tOnlineConflictDiagnostic& diagnostic) {
    if (!diagnostic.valid || !diagnostic.infeasible ||
        diagnostic.mobile_pair_base_ids.size() != 1) return std::nullopt;
    return diagnostic.mobile_pair_base_ids.front();
}

}  // namespace gf
