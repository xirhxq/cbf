#pragma once

#include "cbf/PairwiseSecondOrderCBF.hpp"
#include "grand_finale/CertifiedCoverageRoadmap.hpp"
#include "grand_finale/CertifiedTranslationPrimitive.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace gf {

struct PairIntervalAuditRequest {
    SecondOrderBox2D first;
    SecondOrderBox2D second;
    PairwiseSecondOrderRowSpec spec;
    double tolerance = 0.0;
};

struct PairIntervalAuditResult {
    bool valid = false;
    std::string reason;
    double minimum_h = -std::numeric_limits<double>::infinity();
    double minimum_psi1 = -std::numeric_limits<double>::infinity();
};

struct FiniteWitnessInterval {
    double duration_s = 0.0;
    std::vector<PairIntervalAuditRequest> pairs;
    double hard_control_margin = 0.0;
    double fim_margin = 0.0;
    double posterior_margin = 0.0;
    std::size_t minimum_effective_reference_count = 0;
};

struct FiniteHorizonWitnessRequest {
    double duration_upper_s = 0.0;
    std::vector<FiniteWitnessInterval> intervals;
    FiniteInformationContract information;
    double nominal_hard_margin = 0.0;
    double solver_control_error = 0.0;
    bool terminal_inclusion = false;
};

struct FiniteHorizonWitnessResult {
    bool valid = false;
    std::string reason;
    double duration_upper_s = 0.0;
    double minimum_pair_margin = std::numeric_limits<double>::infinity();
    std::size_t minimum_effective_reference_count =
        std::numeric_limits<std::size_t>::max();
};

namespace finite_horizon_witness_detail {

inline Interval difference(const Interval& lhs, const Interval& rhs) {
    return {lhs.lower - rhs.upper, lhs.upper - rhs.lower};
}

inline double minimumAbsolute(const Interval& value) {
    if (value.lower <= 0.0 && value.upper >= 0.0) return 0.0;
    return std::min(std::abs(value.lower), std::abs(value.upper));
}

inline double maximumAbsolute(const Interval& value) {
    return std::max(std::abs(value.lower), std::abs(value.upper));
}

inline bool informationWordValid(
    const FiniteInformationContract& contract,
    std::string& reason) {
    if (!std::isfinite(contract.horizon_s) || contract.horizon_s <= 0.0 ||
        !std::isfinite(contract.maximum_aoi_s) ||
        contract.maximum_aoi_s <= 0.0 ||
        !std::isfinite(contract.initial_aoi_upper_s) ||
        contract.initial_aoi_upper_s < 0.0 ||
        contract.initial_aoi_upper_s > contract.maximum_aoi_s) {
        reason = "initial_aoi";
        return false;
    }
    for (const std::string& link : contract.required_links) {
        std::vector<double> times;
        for (const auto& event : contract.refreshes) {
            if (event.link_id != link) continue;
            if (!std::isfinite(event.time_s) || event.time_s < 0.0 ||
                event.time_s > contract.horizon_s ||
                !std::isfinite(event.variance_m2) || event.variance_m2 <= 0.0 ||
                !std::isfinite(event.quality) || event.quality <= 0.0 ||
                event.quality > 1.0) {
                reason = "range_refresh_invalid";
                return false;
            }
            times.push_back(event.time_s);
        }
        if (times.empty()) {
            reason = "range_refresh_missing";
            return false;
        }
        std::sort(times.begin(), times.end());
        if (contract.initial_aoi_upper_s + times.front() >
            contract.maximum_aoi_s + 1.0e-12) {
            reason = "range_aoi_gap";
            return false;
        }
        for (std::size_t index = 1; index < times.size(); ++index) {
            if (times[index] - times[index - 1] >
                contract.maximum_aoi_s + 1.0e-12) {
                reason = "range_aoi_gap";
                return false;
            }
        }
        if (contract.horizon_s - times.back() >
            contract.maximum_aoi_s + 1.0e-12) {
            reason = "range_aoi_gap";
            return false;
        }
    }
    return true;
}

}  // namespace finite_horizon_witness_detail

inline PairIntervalAuditResult auditPairInterval(
    const PairIntervalAuditRequest& request) {
    using namespace finite_horizon_witness_detail;
    PairIntervalAuditResult result;
    const auto& spec = request.spec;
    if (!std::isfinite(spec.distanceLimit) || spec.distanceLimit <= 0.0 ||
        !std::isfinite(spec.uncertainty) || spec.uncertainty < 0.0 ||
        !std::isfinite(spec.k) || spec.k <= 0.0 ||
        !std::isfinite(spec.lambda1) || spec.lambda1 <= 0.0 ||
        !std::isfinite(request.tolerance) || request.tolerance < 0.0) {
        result.reason = "invalid_pair_interval";
        return result;
    }
    const Interval dx = difference(
        request.first.position.x, request.second.position.x);
    const Interval dy = difference(
        request.first.position.y, request.second.position.y);
    const double distance_lower = std::hypot(
        minimumAbsolute(dx), minimumAbsolute(dy));
    const double distance_upper = std::hypot(
        maximumAbsolute(dx), maximumAbsolute(dy));
    const Interval dvx = difference(
        request.first.velocity.x, request.second.velocity.x);
    const Interval dvy = difference(
        request.first.velocity.y, request.second.velocity.y);
    const double relative_speed_upper = std::hypot(
        maximumAbsolute(dvx), maximumAbsolute(dvy));
    if (!std::isfinite(distance_lower) || !std::isfinite(distance_upper) ||
        distance_lower < 1.0e-9) {
        result.reason = "pair_distance_singularity";
        return result;
    }
    if (spec.kind == PairwiseSecondOrderBarrierKind::CommunicationUpper) {
        result.minimum_h = spec.k *
            (spec.distanceLimit - distance_upper - spec.uncertainty);
    } else {
        result.minimum_h = spec.k *
            (distance_lower - spec.distanceLimit - spec.uncertainty);
    }
    result.minimum_psi1 =
        spec.lambda1 * result.minimum_h - spec.k * relative_speed_upper;
    if (result.minimum_h < -request.tolerance ||
        result.minimum_psi1 < -request.tolerance) {
        result.reason = spec.kind ==
                PairwiseSecondOrderBarrierKind::CommunicationUpper
            ? "reference_initial_set"
            : "collision_initial_set";
        return result;
    }
    result.valid = true;
    result.reason = "accepted";
    return result;
}

inline FiniteHorizonWitnessResult verifyFiniteHorizonWitness(
    const FiniteHorizonWitnessRequest& request) {
    FiniteHorizonWitnessResult result;
    if (!std::isfinite(request.duration_upper_s) ||
        request.duration_upper_s <= 0.0 || request.intervals.empty()) {
        result.reason = "invalid_duration";
        return result;
    }
    if (!request.terminal_inclusion) {
        result.reason = "terminal_inclusion";
        return result;
    }
    if (!std::isfinite(request.nominal_hard_margin) ||
        !std::isfinite(request.solver_control_error) ||
        request.solver_control_error < 0.0 ||
        request.nominal_hard_margin <= request.solver_control_error) {
        result.reason = "qp_solution_map_unverified";
        return result;
    }
    if (!finite_horizon_witness_detail::informationWordValid(
            request.information, result.reason))
        return result;
    double duration = 0.0;
    for (const auto& interval : request.intervals) {
        if (!std::isfinite(interval.duration_s) || interval.duration_s <= 0.0 ||
            !std::isfinite(interval.hard_control_margin) ||
            interval.hard_control_margin <= 0.0 ||
            !std::isfinite(interval.fim_margin) || interval.fim_margin <= 0.0 ||
            !std::isfinite(interval.posterior_margin) ||
            interval.posterior_margin <= 0.0) {
            result.reason = "interval_margin";
            return result;
        }
        if (interval.minimum_effective_reference_count < 2) {
            result.reason = "effective_reference_count";
            return result;
        }
        duration += interval.duration_s;
        result.minimum_effective_reference_count = std::min(
            result.minimum_effective_reference_count,
            interval.minimum_effective_reference_count);
        for (const auto& pair : interval.pairs) {
            const auto audit = auditPairInterval(pair);
            if (!audit.valid) {
                result.reason = audit.reason;
                return result;
            }
            result.minimum_pair_margin = std::min(
                result.minimum_pair_margin,
                std::min(audit.minimum_h, audit.minimum_psi1));
        }
    }
    if (duration + 1.0e-12 < request.duration_upper_s ||
        request.information.horizon_s + 1.0e-12 <
            request.duration_upper_s) {
        result.reason = "interval_horizon";
        return result;
    }
    result.valid = true;
    result.reason = "accepted";
    result.duration_upper_s = request.duration_upper_s;
    return result;
}

}  // namespace gf
