#ifndef CBF_BRIDGE_RESERVE_TASK_HOMOTOPY_HPP
#define CBF_BRIDGE_RESERVE_TASK_HOMOTOPY_HPP

#include <cmath>
#include <limits>
#include <vector>

struct BridgeReserveTaskCandidate {
    double alpha = 0.0;
    double accelX = 0.0;
    double accelY = 0.0;
    double predictedBudget = -std::numeric_limits<double>::infinity();
};

struct BridgeReserveTaskSelection {
    bool selected = false;
    bool gateSatisfied = false;
    bool nominalRetained = false;
    double alpha = 0.0;
    double accelX = 0.0;
    double accelY = 0.0;
    double predictedBudget = -std::numeric_limits<double>::infinity();
    double taskDeviation = std::numeric_limits<double>::infinity();
    size_t candidateIndex = std::numeric_limits<size_t>::max();
    bool selectedMaximumPredictedBudget = false;
    double maximumPredictedBudget =
            -std::numeric_limits<double>::infinity();
    size_t maximumCandidateIndex = std::numeric_limits<size_t>::max();
};

inline std::vector<BridgeReserveTaskCandidate> buildReserveTaskHomotopy(
        double legacyAccelX,
        double legacyAccelY,
        double reserveAccelX,
        double reserveAccelY,
        size_t intervals) {
    if (intervals == 0
        || !std::isfinite(legacyAccelX) || !std::isfinite(legacyAccelY)
        || !std::isfinite(reserveAccelX) || !std::isfinite(reserveAccelY)) {
        return {};
    }

    std::vector<BridgeReserveTaskCandidate> candidates;
    candidates.reserve(intervals + 1);
    for (size_t index = 0; index <= intervals; ++index) {
        const double alpha =
                static_cast<double>(index) / static_cast<double>(intervals);
        candidates.push_back({
                alpha,
                (1.0 - alpha) * legacyAccelX + alpha * reserveAccelX,
                (1.0 - alpha) * legacyAccelY + alpha * reserveAccelY,
                -std::numeric_limits<double>::infinity(),
        });
    }
    return candidates;
}

namespace bridge_reserve_task_detail {

inline bool finite(const BridgeReserveTaskCandidate &candidate) {
    return std::isfinite(candidate.alpha)
           && candidate.alpha >= 0.0
           && candidate.alpha <= 1.0
           && std::isfinite(candidate.accelX)
           && std::isfinite(candidate.accelY)
           && std::isfinite(candidate.predictedBudget);
}

inline bool lexicographicallyEarlier(
        const BridgeReserveTaskCandidate &candidate,
        const BridgeReserveTaskCandidate &current) {
    if (candidate.accelX < current.accelX) {
        return true;
    }
    return candidate.accelX == current.accelX
           && candidate.accelY < current.accelY;
}

inline bool betterSafe(
        const BridgeReserveTaskCandidate &candidate,
        const BridgeReserveTaskCandidate &current) {
    if (candidate.alpha < current.alpha) {
        return true;
    }
    if (candidate.alpha > current.alpha) {
        return false;
    }
    if (candidate.predictedBudget > current.predictedBudget) {
        return true;
    }
    if (candidate.predictedBudget < current.predictedBudget) {
        return false;
    }
    return lexicographicallyEarlier(candidate, current);
}

inline bool betterFallback(
        const BridgeReserveTaskCandidate &candidate,
        const BridgeReserveTaskCandidate &current) {
    if (candidate.predictedBudget > current.predictedBudget) {
        return true;
    }
    if (candidate.predictedBudget < current.predictedBudget) {
        return false;
    }
    if (candidate.alpha < current.alpha) {
        return true;
    }
    if (candidate.alpha > current.alpha) {
        return false;
    }
    return lexicographicallyEarlier(candidate, current);
}

}  // namespace bridge_reserve_task_detail

inline BridgeReserveTaskSelection selectReserveTaskHomotopy(
        const std::vector<BridgeReserveTaskCandidate> &candidates,
        double legacyAccelX,
        double legacyAccelY,
        double gateTau,
        double tolerance = 1.0e-9) {
    BridgeReserveTaskSelection selection;
    if (!std::isfinite(legacyAccelX) || !std::isfinite(legacyAccelY)
        || !std::isfinite(gateTau)
        || !std::isfinite(tolerance) || tolerance < 0.0) {
        return selection;
    }

    const BridgeReserveTaskCandidate *bestSafe = nullptr;
    const BridgeReserveTaskCandidate *bestFallback = nullptr;
    size_t bestSafeIndex = std::numeric_limits<size_t>::max();
    size_t bestFallbackIndex = std::numeric_limits<size_t>::max();
    for (size_t index = 0; index < candidates.size(); ++index) {
        const auto &candidate = candidates[index];
        if (!bridge_reserve_task_detail::finite(candidate)) {
            continue;
        }
        if (candidate.predictedBudget >= gateTau
            && (bestSafe == nullptr
                || bridge_reserve_task_detail::betterSafe(
                        candidate, *bestSafe))) {
            bestSafe = &candidate;
            bestSafeIndex = index;
        }
        if (bestFallback == nullptr
            || bridge_reserve_task_detail::betterFallback(
                    candidate, *bestFallback)) {
            bestFallback = &candidate;
            bestFallbackIndex = index;
        }
    }

    const BridgeReserveTaskCandidate *winner =
            bestSafe != nullptr ? bestSafe : bestFallback;
    if (winner == nullptr) {
        return selection;
    }

    selection.selected = true;
    selection.gateSatisfied = bestSafe != nullptr;
    selection.alpha = winner->alpha;
    selection.accelX = winner->accelX;
    selection.accelY = winner->accelY;
    selection.predictedBudget = winner->predictedBudget;
    selection.taskDeviation = std::hypot(
            winner->accelX - legacyAccelX,
            winner->accelY - legacyAccelY);
    selection.nominalRetained = selection.taskDeviation <= tolerance;
    selection.candidateIndex = bestSafe != nullptr
            ? bestSafeIndex : bestFallbackIndex;
    selection.selectedMaximumPredictedBudget =
            selection.candidateIndex == bestFallbackIndex;
    selection.maximumPredictedBudget = bestFallback->predictedBudget;
    selection.maximumCandidateIndex = bestFallbackIndex;
    return selection;
}

inline BridgeReserveTaskSelection selectMaximumReserveHomotopy(
        const std::vector<BridgeReserveTaskCandidate> &candidates,
        double legacyAccelX,
        double legacyAccelY,
        double gateTau,
        double tolerance = 1.0e-9) {
    BridgeReserveTaskSelection selection;
    if (!std::isfinite(legacyAccelX) || !std::isfinite(legacyAccelY)
        || !std::isfinite(gateTau)
        || !std::isfinite(tolerance) || tolerance < 0.0) {
        return selection;
    }

    const BridgeReserveTaskCandidate *endpoint = nullptr;
    const BridgeReserveTaskCandidate *bestFallback = nullptr;
    size_t endpointIndex = std::numeric_limits<size_t>::max();
    size_t bestFallbackIndex = std::numeric_limits<size_t>::max();
    for (size_t index = 0; index < candidates.size(); ++index) {
        const auto &candidate = candidates[index];
        if (!bridge_reserve_task_detail::finite(candidate)) {
            continue;
        }
        if (bestFallback == nullptr
            || bridge_reserve_task_detail::betterFallback(
                    candidate, *bestFallback)) {
            bestFallback = &candidate;
            bestFallbackIndex = index;
        }
        if (candidate.alpha == 1.0
            && (endpoint == nullptr
            || bridge_reserve_task_detail::lexicographicallyEarlier(
                    candidate, *endpoint))) {
            endpoint = &candidate;
            endpointIndex = index;
        }
    }
    if (endpoint == nullptr) {
        return selection;
    }

    selection.selected = true;
    selection.gateSatisfied = endpoint->predictedBudget >= gateTau;
    selection.alpha = endpoint->alpha;
    selection.accelX = endpoint->accelX;
    selection.accelY = endpoint->accelY;
    selection.predictedBudget = endpoint->predictedBudget;
    selection.taskDeviation = std::hypot(
            endpoint->accelX - legacyAccelX,
            endpoint->accelY - legacyAccelY);
    selection.nominalRetained = selection.taskDeviation <= tolerance;
    selection.candidateIndex = endpointIndex;
    selection.selectedMaximumPredictedBudget =
            endpointIndex == bestFallbackIndex;
    selection.maximumPredictedBudget = bestFallback->predictedBudget;
    selection.maximumCandidateIndex = bestFallbackIndex;
    return selection;
}

#endif  // CBF_BRIDGE_RESERVE_TASK_HOMOTOPY_HPP
