#pragma once

#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/DynamicPairCertifiedControl.hpp"

#include <map>

namespace gf {

struct DevelopmentFullPairCertifiedControlAudit {
    bool valid = false;
    std::string reason;
    std::size_t full_row_count = 0;
    double minimum_residual_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    std::string limiting_row_id;
};

inline DevelopmentFullPairCertifiedControlAudit
auditDevelopmentFullPairCertifiedControls(
    const std::vector<CanonicalHardRow>& rows,
    const std::vector<NodeId>& mobile_ids,
    double acceleration_half_box, double tolerance,
    const std::map<NodeId, Eigen::Vector2d>& controls) {
    DevelopmentFullPairCertifiedControlAudit result;
    if (mobile_ids.size() != 14 || controls.size() != mobile_ids.size() ||
        !std::isfinite(acceleration_half_box) || acceleration_half_box <= 0.0 ||
        !std::isfinite(tolerance) || tolerance < 0.0) {
        result.reason = "development_full_pair_request_invalid";
        return result;
    }
    for (NodeId owner : mobile_ids) {
        const auto found = controls.find(owner);
        if (found == controls.end() || !found->second.allFinite() ||
            found->second.cwiseAbs().maxCoeff() >
                acceleration_half_box + tolerance) {
            result.reason = "development_full_pair_control_invalid";
            return result;
        }
    }
    std::map<std::string, std::vector<const CanonicalHardRow*>> pairs;
    std::vector<const CanonicalHardRow*> uncoupled;
    for (const auto& row : rows) {
        const bool physical_pair =
            (row.kind == CanonicalHardRowKind::ReferenceDistance ||
             row.kind == CanonicalHardRowKind::Collision) &&
            row.peer.has_value() &&
            std::find(mobile_ids.begin(), mobile_ids.end(), *row.peer) !=
                mobile_ids.end();
        if (physical_pair) pairs[dynamicPairBaseId(row.id)].push_back(&row);
        else uncoupled.push_back(&row);
    }
    result.minimum_residual_mps2 = std::numeric_limits<double>::infinity();
    const auto consider = [&](double residual, const std::string& id) {
        if (!std::isfinite(residual))
            throw std::runtime_error("nonfinite full-pair residual");
        if (residual < result.minimum_residual_mps2) {
            result.minimum_residual_mps2 = residual;
            result.limiting_row_id = id;
        }
    };
    try {
        for (const auto* row : uncoupled) {
            consider(row->margin(controls.at(row->owner)), row->id);
            ++result.full_row_count;
        }
        for (auto& [base, halves] : pairs) {
            if (base.empty() || halves.size() != 2) {
                result.reason = "development_full_pair_incoherent_pair";
                return result;
            }
            std::sort(halves.begin(), halves.end(),
                [](const auto* lhs, const auto* rhs) {
                    return lhs->owner < rhs->owner;
                });
            const auto& first = *halves[0];
            const auto& second = *halves[1];
            if (!first.peer.has_value() || !second.peer.has_value() ||
                first.owner != *second.peer || second.owner != *first.peer ||
                first.kind != second.kind ||
                std::abs(first.responsibility - 0.5) > 1e-12 ||
                std::abs(second.responsibility - 0.5) > 1e-12 ||
                std::abs(first.coefficient_uncertainty_reserve -
                         second.coefficient_uncertainty_reserve) > 1e-10) {
                result.reason = "development_full_pair_incoherent_pair";
                return result;
            }
            const double residual =
                first.margin(controls.at(first.owner)) +
                second.margin(controls.at(second.owner)) +
                first.coefficient_uncertainty_reserve;
            consider(residual, base);
            ++result.full_row_count;
        }
    } catch (const std::exception& error) {
        result.reason = std::string("development_full_pair_audit_error:") +
            error.what();
        return result;
    }
    if (result.minimum_residual_mps2 < -tolerance) {
        result.reason = "development_full_pair_residual_negative";
        return result;
    }
    result.valid = true;
    result.reason = "development_full_pair_controls_certified";
    return result;
}

}  // namespace gf
