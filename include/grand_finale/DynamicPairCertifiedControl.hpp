#pragma once

#include "grand_finale/CanonicalHardRows.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace gf {

struct DynamicPairCertifiedControlRequest {
    std::string pair_base_id;
    NodeId first_owner = 0;
    NodeId second_owner = 0;
    double transfer_interval_lower_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double transfer_interval_upper_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double transfer_mps2 = std::numeric_limits<double>::quiet_NaN();
    std::map<NodeId, Eigen::Vector2d> controls;
};

struct DynamicPairCertifiedControlAudit {
    bool valid = false;
    std::string reason;
    NodeId first_owner = 0;
    NodeId second_owner = 0;
    double minimum_local_residual_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    NodeId limiting_owner = 0;
    std::string limiting_row_id;
    double dynamic_pair_local_sum_residual_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double once_reserve_full_pair_residual_mps2 =
        std::numeric_limits<double>::quiet_NaN();
};

inline std::string dynamicPairBaseId(const std::string& row_id) {
    const std::string marker = ":owner:";
    const std::size_t position = row_id.rfind(marker);
    return position == std::string::npos
        ? std::string{} : row_id.substr(0, position);
}

inline DynamicPairCertifiedControlAudit auditDynamicPairCertifiedControls(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids,
    double acceleration_half_box,
    double tolerance,
    const DynamicPairCertifiedControlRequest& request) {
    DynamicPairCertifiedControlAudit result;
    if (mobile_ids.size() != 14 || request.controls.size() != 14 ||
        request.pair_base_id.empty() ||
        !std::isfinite(acceleration_half_box) ||
        acceleration_half_box <= 0.0 || !std::isfinite(tolerance) ||
        tolerance < 0.0 ||
        !std::isfinite(request.transfer_interval_lower_mps2) ||
        !std::isfinite(request.transfer_interval_upper_mps2) ||
        !std::isfinite(request.transfer_mps2) ||
        request.transfer_interval_lower_mps2 >
            request.transfer_interval_upper_mps2 + tolerance ||
        request.transfer_mps2 <
            request.transfer_interval_lower_mps2 - tolerance ||
        request.transfer_mps2 >
            request.transfer_interval_upper_mps2 + tolerance) {
        result.reason = "dynamic_pair_control_request_invalid";
        return result;
    }
    for (NodeId owner : mobile_ids) {
        const auto control = request.controls.find(owner);
        if (control == request.controls.end() ||
            !control->second.allFinite() ||
            control->second.cwiseAbs().maxCoeff() >
                acceleration_half_box + tolerance) {
            result.reason = "dynamic_pair_control_batch_incomplete_or_unbounded";
            return result;
        }
    }

    std::vector<const CanonicalHardRow*> pair_rows;
    for (const CanonicalHardRow& row : rows) {
        const bool pair_kind =
            row.kind == CanonicalHardRowKind::ReferenceDistance ||
            row.kind == CanonicalHardRowKind::Collision;
        const bool mobile_peer = row.peer.has_value() &&
            std::find(mobile_ids.begin(), mobile_ids.end(), *row.peer) !=
                mobile_ids.end();
        if (pair_kind && mobile_peer &&
            dynamicPairBaseId(row.id) == request.pair_base_id) {
            pair_rows.push_back(&row);
        }
    }
    if (pair_rows.size() != 2) {
        result.reason = "dynamic_pair_control_pair_not_unique";
        return result;
    }
    std::sort(pair_rows.begin(), pair_rows.end(),
        [](const auto* left, const auto* right) {
            return left->owner < right->owner;
        });
    const CanonicalHardRow& first = *pair_rows[0];
    const CanonicalHardRow& second = *pair_rows[1];
    const bool coherent =
        first.peer.has_value() && second.peer.has_value() &&
        first.owner == *second.peer && second.owner == *first.peer &&
        first.owner == request.first_owner &&
        second.owner == request.second_owner &&
        first.kind == second.kind &&
        std::abs(first.responsibility - 0.5) <= 1e-12 &&
        std::abs(second.responsibility - 0.5) <= 1e-12 &&
        (first.control_coefficient + second.control_coefficient).norm() <=
            1e-9 &&
        std::abs(first.constant - second.constant) <= 1e-8 &&
        std::abs(first.coefficient_uncertainty_reserve -
                 second.coefficient_uncertainty_reserve) <= 1e-10 &&
        first.coefficient_uncertainty_reserve >= 0.0;
    if (!coherent) {
        result.reason = "dynamic_pair_control_pair_incoherent";
        return result;
    }

    result.first_owner = first.owner;
    result.second_owner = second.owner;
    result.minimum_local_residual_mps2 =
        std::numeric_limits<double>::infinity();
    double first_residual = std::numeric_limits<double>::quiet_NaN();
    double second_residual = std::numeric_limits<double>::quiet_NaN();
    for (const CanonicalHardRow& row : rows) {
        const auto control = request.controls.find(row.owner);
        if (control == request.controls.end()) {
            result.reason = "dynamic_pair_control_row_owner_missing";
            return result;
        }
        double constant = row.constant;
        if (row.id == first.id) constant += request.transfer_mps2;
        if (row.id == second.id) constant -= request.transfer_mps2;
        const double residual =
            row.control_coefficient.dot(control->second) + constant;
        if (!std::isfinite(residual)) {
            result.reason = "dynamic_pair_control_residual_invalid";
            return result;
        }
        if (row.id == first.id) first_residual = residual;
        if (row.id == second.id) second_residual = residual;
        if (residual < result.minimum_local_residual_mps2) {
            result.minimum_local_residual_mps2 = residual;
            result.limiting_owner = row.owner;
            result.limiting_row_id = row.id;
        }
    }
    if (!std::isfinite(first_residual) || !std::isfinite(second_residual)) {
        result.reason = "dynamic_pair_control_pair_residual_missing";
        return result;
    }
    result.dynamic_pair_local_sum_residual_mps2 =
        first_residual + second_residual;
    result.once_reserve_full_pair_residual_mps2 =
        result.dynamic_pair_local_sum_residual_mps2 +
        first.coefficient_uncertainty_reserve;
    if (result.minimum_local_residual_mps2 < -tolerance ||
        result.once_reserve_full_pair_residual_mps2 < -tolerance) {
        result.reason = "dynamic_pair_control_residual_negative";
        return result;
    }
    result.valid = true;
    result.reason = "dynamic_pair_control_batch_certified";
    return result;
}

}  // namespace gf
