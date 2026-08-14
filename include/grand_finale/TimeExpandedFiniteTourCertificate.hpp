#pragma once

#include "grand_finale/FiniteHorizonWitnessVerifier.hpp"
#include "grand_finale/FiniteTourShadowEnvelope.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace gf {

struct FiniteTourNode {
    std::size_t stage = 0;
    ShadowStateBox shadow;
};

struct FiniteTourHardAudit {
    bool robust_hocbf = false;
    bool effective_references = false;
    bool fim_posterior = false;
    bool aoi = false;
    bool sensing = false;
    bool full_runtime_terminal = false;
};

struct FiniteEdgeBlackoutContract {
    std::string link_id;
    double measurement_period_s = 0.0;
    std::size_t maximum_consecutive_dropouts = 0;
    double initial_aoi_upper_s = 0.0;
    double maximum_aoi_s = 0.0;
    double measurement_variance_lower_m2 = 0.0;
    double measurement_variance_upper_m2 = 0.0;
    double minimum_quality = 0.0;
};

struct TaggedScalarUpdateBound {
    std::string link_id;
    ScalarUpdateBound bound;
};

struct FiniteTourShadowCycle {
    Eigen::MatrixXd transition;
    Eigen::MatrixXd disturbance_input;
    Eigen::VectorXd absolute_disturbance_bound;
    std::vector<TaggedScalarUpdateBound> scalar_updates;
};

struct FiniteTourEdgeCertificate {
    std::size_t source_stage = 0;
    std::size_t target_stage = 0;
    FiniteHorizonWitnessResult witness;
    ShadowStateBox reachable_terminal_shadow;
    std::size_t expected_scalar_update_count = 0;
    std::size_t bounded_scalar_update_count = 0;
    FiniteTourHardAudit hard_audit;
    std::set<std::string> certified_cells;
    std::vector<FiniteEdgeBlackoutContract> blackout_contracts;
    std::vector<FiniteTourShadowCycle> shadow_cycles;
};

struct FiniteTourRequest {
    std::vector<FiniteTourNode> nodes;
    std::vector<FiniteTourEdgeCertificate> edges;
    std::set<std::string> required_cells;
    double duration_upper_s = 0.0;
};

struct FiniteTourResult {
    bool valid = false;
    std::string reason;
    double duration_upper_s = 0.0;
    std::set<std::string> certified_cells;
    double final_shadow_radius = 0.0;
};

namespace time_expanded_tour_detail {

inline bool contains(
    const ShadowStateBox& outer,
    const ShadowStateBox& inner,
    double tolerance = 1.0e-12) {
    finite_tour_shadow_detail::requireBox(outer);
    finite_tour_shadow_detail::requireBox(inner);
    return outer.lower.size() == inner.lower.size() &&
        (outer.lower.array() <= inner.lower.array() + tolerance).all() &&
        (outer.upper.array() + tolerance >= inner.upper.array()).all();
}

inline double maximumRadius(const ShadowStateBox& box) {
    finite_tour_shadow_detail::requireBox(box);
    return std::max(
        box.lower.cwiseAbs().maxCoeff(), box.upper.cwiseAbs().maxCoeff());
}

}  // namespace time_expanded_tour_detail

inline FiniteTourResult verifyTimeExpandedFiniteTour(
    const FiniteTourRequest& request) {
    FiniteTourResult result;
    if (request.nodes.size() < 2 ||
        request.edges.size() + 1 != request.nodes.size()) {
        result.reason = "tour_shape";
        return result;
    }
    if (!std::isfinite(request.duration_upper_s) ||
        request.duration_upper_s <= 0.0) {
        result.reason = "tour_duration";
        return result;
    }
    for (std::size_t stage = 0; stage < request.nodes.size(); ++stage) {
        if (request.nodes[stage].stage != stage) {
            result.reason = "stage_order";
            return result;
        }
        try {
            finite_tour_shadow_detail::requireBox(request.nodes[stage].shadow);
        } catch (const std::invalid_argument&) {
            result.reason = "shadow_node";
            return result;
        }
    }

    double duration = 0.0;
    for (std::size_t stage = 0; stage < request.edges.size(); ++stage) {
        const FiniteTourEdgeCertificate& edge = request.edges[stage];
        if (edge.source_stage != stage || edge.target_stage != stage + 1) {
            result.reason = "stage_order";
            return result;
        }
        if (!edge.witness.valid) {
            result.reason = "witness:" + edge.witness.reason;
            return result;
        }
        if (!std::isfinite(edge.witness.duration_upper_s) ||
            edge.witness.duration_upper_s <= 0.0) {
            result.reason = "witness_duration";
            return result;
        }
        if (edge.shadow_cycles.empty()) {
            result.reason = "shadow_cycle_missing";
            return result;
        }
        ShadowStateBox composed_shadow = request.nodes[stage].shadow;
        std::size_t composed_update_count = 0;
        std::map<std::string, std::size_t> updates_by_link;
        try {
            for (const FiniteTourShadowCycle& cycle : edge.shadow_cycles) {
                composed_shadow = propagateShadowPrediction(
                    composed_shadow, cycle.transition,
                    cycle.disturbance_input,
                    cycle.absolute_disturbance_bound);
                std::vector<ScalarUpdateBound> updates;
                updates.reserve(cycle.scalar_updates.size());
                for (const TaggedScalarUpdateBound& tagged :
                     cycle.scalar_updates) {
                    if (tagged.link_id.empty()) {
                        result.reason = "scalar_update_link";
                        return result;
                    }
                    updates.push_back(tagged.bound);
                    ++updates_by_link[tagged.link_id];
                    ++composed_update_count;
                }
                composed_shadow = accumulateScalarUpdateBatch(
                    composed_shadow, updates);
            }
        } catch (const std::invalid_argument&) {
            result.reason = "shadow_cycle_invalid";
            return result;
        }
        if (edge.expected_scalar_update_count == 0 ||
            edge.bounded_scalar_update_count !=
                edge.expected_scalar_update_count ||
            composed_update_count != edge.expected_scalar_update_count) {
            result.reason = "scalar_update_bound_missing";
            return result;
        }
        if (edge.blackout_contracts.empty()) {
            result.reason = "blackout_contract_missing";
            return result;
        }
        std::set<std::string> blackout_links;
        for (const FiniteEdgeBlackoutContract& blackout :
             edge.blackout_contracts) {
            if (blackout.link_id.empty() ||
                !blackout_links.insert(blackout.link_id).second ||
                !std::isfinite(blackout.measurement_period_s) ||
                blackout.measurement_period_s <= 0.0 ||
                !std::isfinite(blackout.initial_aoi_upper_s) ||
                blackout.initial_aoi_upper_s < 0.0 ||
                !std::isfinite(blackout.maximum_aoi_s) ||
                blackout.maximum_aoi_s <= 0.0) {
                result.reason = "blackout_contract_invalid";
                return result;
            }
            if (!std::isfinite(
                    blackout.measurement_variance_lower_m2) ||
                blackout.measurement_variance_lower_m2 <= 0.0 ||
                !std::isfinite(
                    blackout.measurement_variance_upper_m2) ||
                blackout.measurement_variance_upper_m2 <
                    blackout.measurement_variance_lower_m2 ||
                !std::isfinite(blackout.minimum_quality) ||
                blackout.minimum_quality <= 0.0 ||
                blackout.minimum_quality > 1.0) {
                result.reason = "blackout_measurement_contract";
                return result;
            }
            const double worst_gap = blackout.initial_aoi_upper_s +
                static_cast<double>(
                    blackout.maximum_consecutive_dropouts + 1) *
                    blackout.measurement_period_s;
            if (worst_gap > blackout.maximum_aoi_s + 1.0e-12) {
                result.reason = "blackout_aoi_gap";
                return result;
            }
            const std::size_t required_slots = static_cast<std::size_t>(
                std::ceil(edge.witness.duration_upper_s /
                          blackout.measurement_period_s - 1.0e-12));
            if (updates_by_link[blackout.link_id] < required_slots) {
                result.reason = "blackout_slot_missing";
                return result;
            }
        }
        if (!edge.hard_audit.robust_hocbf) {
            result.reason = "robust_hocbf";
            return result;
        }
        if (!edge.hard_audit.effective_references) {
            result.reason = "effective_references";
            return result;
        }
        if (!edge.hard_audit.fim_posterior) {
            result.reason = "fim_posterior";
            return result;
        }
        if (!edge.hard_audit.aoi) {
            result.reason = "aoi";
            return result;
        }
        if (!edge.hard_audit.sensing) {
            result.reason = "sensing";
            return result;
        }
        if (!edge.hard_audit.full_runtime_terminal) {
            result.reason = "full_runtime_terminal";
            return result;
        }
        try {
            if (!time_expanded_tour_detail::contains(
                    edge.reachable_terminal_shadow, composed_shadow) ||
                !time_expanded_tour_detail::contains(
                    request.nodes[stage + 1].shadow,
                    edge.reachable_terminal_shadow)) {
                result.reason = "shadow_terminal_inclusion";
                return result;
            }
        } catch (const std::invalid_argument&) {
            result.reason = "shadow_terminal_inclusion";
            return result;
        }
        duration += edge.witness.duration_upper_s;
        result.certified_cells.insert(
            edge.certified_cells.begin(), edge.certified_cells.end());
    }
    if (duration > request.duration_upper_s + 1.0e-12) {
        result.reason = "tour_duration";
        return result;
    }
    if (!std::includes(
            result.certified_cells.begin(), result.certified_cells.end(),
            request.required_cells.begin(), request.required_cells.end())) {
        result.reason = "required_cell_uncovered";
        return result;
    }

    result.valid = true;
    result.reason = "accepted";
    result.duration_upper_s = duration;
    result.final_shadow_radius = time_expanded_tour_detail::maximumRadius(
        request.nodes.back().shadow);
    return result;
}

}  // namespace gf
