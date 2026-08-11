#ifndef CBF_BRIDGE_TASK_AWARE_GAMMA_SELECTOR_HPP
#define CBF_BRIDGE_TASK_AWARE_GAMMA_SELECTOR_HPP

#include <cmath>
#include <limits>
#include <vector>

struct BridgeTaskAwareGammaCandidate {
    double accelX = 0.0;
    double accelY = 0.0;
    double gamma = -std::numeric_limits<double>::infinity();
};

struct BridgeTaskAwareGammaSelection {
    bool selected = false;
    bool reserveSatisfied = false;
    bool nominalRetained = false;
    double accelX = 0.0;
    double accelY = 0.0;
    double gamma = -std::numeric_limits<double>::infinity();
    double taskDeviation = std::numeric_limits<double>::infinity();
};

inline double bridgeTaskAccelerationDeviation(
        const BridgeTaskAwareGammaCandidate &candidate,
        double nominalAccelX,
        double nominalAccelY) {
    return std::hypot(candidate.accelX - nominalAccelX,
                      candidate.accelY - nominalAccelY);
}

inline BridgeTaskAwareGammaSelection bridgeSelectTaskAwareGammaCandidate(
        const std::vector<BridgeTaskAwareGammaCandidate> &candidates,
        double nominalAccelX,
        double nominalAccelY,
        double reserve,
        double tolerance = 1.0e-9) {
    BridgeTaskAwareGammaSelection selection;
    if (!std::isfinite(nominalAccelX) || !std::isfinite(nominalAccelY)
        || !std::isfinite(reserve) || !std::isfinite(tolerance) || tolerance < 0.0) {
        return selection;
    }

    const BridgeTaskAwareGammaCandidate *bestSafe = nullptr;
    double bestSafeDeviation = std::numeric_limits<double>::infinity();
    const BridgeTaskAwareGammaCandidate *bestFallback = nullptr;
    double bestFallbackDeviation = std::numeric_limits<double>::infinity();

    for (const auto &candidate : candidates) {
        if (!std::isfinite(candidate.accelX) || !std::isfinite(candidate.accelY)
            || !std::isfinite(candidate.gamma)) {
            continue;
        }
        const double deviation = bridgeTaskAccelerationDeviation(
                candidate, nominalAccelX, nominalAccelY);
        if (candidate.gamma >= reserve - tolerance) {
            if (bestSafe == nullptr
                || deviation < bestSafeDeviation - tolerance
                || (std::abs(deviation - bestSafeDeviation) <= tolerance
                    && candidate.gamma > bestSafe->gamma + tolerance)) {
                bestSafe = &candidate;
                bestSafeDeviation = deviation;
            }
        }
        if (bestFallback == nullptr
            || candidate.gamma > bestFallback->gamma + tolerance
            || (std::abs(candidate.gamma - bestFallback->gamma) <= tolerance
                && deviation < bestFallbackDeviation - tolerance)) {
            bestFallback = &candidate;
            bestFallbackDeviation = deviation;
        }
    }

    const BridgeTaskAwareGammaCandidate *winner = bestSafe != nullptr
                                                   ? bestSafe : bestFallback;
    if (winner == nullptr) {
        return selection;
    }
    selection.selected = true;
    selection.reserveSatisfied = bestSafe != nullptr;
    selection.accelX = winner->accelX;
    selection.accelY = winner->accelY;
    selection.gamma = winner->gamma;
    selection.taskDeviation = bridgeTaskAccelerationDeviation(
            *winner, nominalAccelX, nominalAccelY);
    selection.nominalRetained = selection.taskDeviation <= tolerance;
    return selection;
}

#endif  // CBF_BRIDGE_TASK_AWARE_GAMMA_SELECTOR_HPP
