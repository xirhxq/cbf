#ifndef CBF_BRIDGE_SEARCH_TRACKER_HPP
#define CBF_BRIDGE_SEARCH_TRACKER_HPP

#include "bridge/BridgeExperiment.hpp"
#include "bridge/BridgeTopology.hpp"
#include "ComputingGeometry/Point.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <vector>

struct BridgeSearchPenaltyDiagnostics {
    double penalty = 0.0;
    double minRobustMargin = std::numeric_limits<double>::quiet_NaN();
    double boundaryDeficit = 0.0;
};

struct ScheduleSample {
    double runtime = 0.0;
    double coverage = 0.0;
    double entropy = 0.0;
};

struct BridgeSearchGoalDecision {
    Point goal = Point(0.0, 0.0);
    Point baselineGoal = Point(0.0, 0.0);
    bool predictiveEnabled = false;
    bool predictiveChangedGoal = false;
    bool exposureEnabled = false;
    bool exposureChangedGoal = false;
    bool beliefConcentrationChangedGoal = false;
    double selectedPenalty = 0.0;
    double maxPenalty = 0.0;
    double selectedMinRobustMargin = std::numeric_limits<double>::quiet_NaN();
    double selectedBoundaryDeficit = 0.0;
    double selectedExposureUtility = 0.0;
    double maxExposureUtility = 0.0;
    bool exposureServiceGateEnabled = false;
    bool exposureServiceScheduleEnabled = false;
    bool serviceScheduleDue = false;
    bool selectedExposureServiceEligible = true;
    double selectedServiceUtility = 0.0;
    double maxServiceUtility = 0.0;
    double requiredServiceUtility = 0.0;
    double searchedCells = 0.0;
    double requiredSearchedCells = 0.0;
    double serviceScheduleDeficit = 0.0;
    double requiredRobustMargin = 0.0;
    int evaluatedCandidates = 0;
    int penalizedCandidates = 0;
    int serviceRejectedCandidates = 0;
    bool beliefConcentrationEnabled = false;
    double selectedBeliefConcentrationUtility = 0.0;
    double maxBeliefConcentrationUtility = 0.0;
    double beliefCentroidX = 0.0;
    double beliefCentroidY = 0.0;
    double topBeliefMass = 0.0;
    double peakSearchedFraction = 0.0;
};

class BridgeSearchTracker {
public:
    BridgeSearchTracker() = default;

    BridgeSearchTracker(const json &config, const BridgeExperimentConfig &bridge)
        : bridge_(bridge) {
        spacing_ = config.at("world").value("spacing", 100.0);
        width_ = static_cast<int>(std::round(bridgeWorldExtent(config, 0) / spacing_));
        height_ = static_cast<int>(std::round(bridgeWorldExtent(config, 1) / spacing_));
        width_ = std::max(width_, 1);
        height_ = std::max(height_, 1);
        belief_.assign(static_cast<size_t>(width_ * height_), 1.0);
        searched_.assign(static_cast<size_t>(width_ * height_), false);
        if (config.contains("bridge") && config.at("bridge").contains("search")) {
            const json &search = config.at("bridge").at("search");
            negativeDetectionFactor_ = search.value("negative-detection-factor", negativeDetectionFactor_);
            positiveDetectionFactor_ = search.value("positive-detection-factor", positiveDetectionFactor_);
            clarityWeight_ = search.value("clarity-weight", clarityWeight_);
            beliefWeight_ = search.value("belief-weight", beliefWeight_);
            travelWeight_ = search.value("travel-weight", travelWeight_);
            targetPriorStrength_ = search.value("target-prior-strength", targetPriorStrength_);
            targetPriorSigma_ = search.value("target-prior-sigma", targetPriorSigma_);
            goalBoundaryMargin_ = search.value("goal-boundary-margin", goalBoundaryMargin_);
            predictiveFeasibilityWeight_ = search.value("predictive-feasibility-weight", predictiveFeasibilityWeight_);
            predictiveRobustMargin_ = search.value("predictive-robust-margin", predictiveRobustMargin_);
            predictiveHorizon_ = search.value("predictive-horizon", predictiveHorizon_);
            predictiveStep_ = search.value("predictive-step-m", predictiveStep_);
            exposureWeight_ = search.value("exposure-weight", exposureWeight_);
            exposureRadius_ = search.value("exposure-radius-m", exposureRadius_);
            exposureHalfAngleRad_ = search.value("exposure-half-angle-deg", exposureHalfAngleRad_ * 180.0 / M_PI) * M_PI / 180.0;
            exposureUnsearchedOnly_ = search.value("exposure-unsearched-only", exposureUnsearchedOnly_);
            exposureLookaheadSteps_ = std::max(1, search.value("exposure-lookahead-steps", exposureLookaheadSteps_));
            exposureLookaheadStep_ = search.value("exposure-lookahead-step-m", exposureLookaheadStep_);
            exposureLookaheadDiscount_ = search.value("exposure-lookahead-discount", exposureLookaheadDiscount_);
            if (exposureLookaheadDiscount_ <= 0.0 || !std::isfinite(exposureLookaheadDiscount_)) {
                exposureLookaheadDiscount_ = 1.0;
            }
            exposureServiceGateMinCells_ = search.value("exposure-service-gate-min-cells", exposureServiceGateMinCells_);
            if (exposureServiceGateMinCells_ < 0.0 || !std::isfinite(exposureServiceGateMinCells_)) {
                exposureServiceGateMinCells_ = 0.0;
            }
            exposureServiceGateRatio_ = search.value("exposure-service-gate-ratio", exposureServiceGateRatio_);
            if (exposureServiceGateRatio_ < 0.0 || !std::isfinite(exposureServiceGateRatio_)) {
                exposureServiceGateRatio_ = 0.0;
            }
            exposureServiceScheduleRateCellsPerS_ = search.value(
                "exposure-service-schedule-rate-cells-per-s",
                exposureServiceScheduleRateCellsPerS_);
            if (exposureServiceScheduleRateCellsPerS_ < 0.0 || !std::isfinite(exposureServiceScheduleRateCellsPerS_)) {
                exposureServiceScheduleRateCellsPerS_ = 0.0;
            }
            exposureServiceScheduleSlackCells_ = search.value(
                "exposure-service-schedule-slack-cells",
                exposureServiceScheduleSlackCells_);
            if (exposureServiceScheduleSlackCells_ < 0.0 || !std::isfinite(exposureServiceScheduleSlackCells_)) {
                exposureServiceScheduleSlackCells_ = 0.0;
            }
            exposureServiceScheduleAdaptive_ = search.value(
                "exposure-service-schedule-adaptive",
                exposureServiceScheduleAdaptive_);
            exposureServiceScheduleRateMinCellsPerS_ = search.value(
                "exposure-service-schedule-rate-min-cells-per-s",
                exposureServiceScheduleRateMinCellsPerS_);
            if (exposureServiceScheduleRateMinCellsPerS_ < 0.0 || !std::isfinite(exposureServiceScheduleRateMinCellsPerS_)) {
                exposureServiceScheduleRateMinCellsPerS_ = 0.0;
            }
            exposureServiceScheduleSaturationWindow_ = std::max(
                1,
                search.value("exposure-service-schedule-saturation-window",
                             exposureServiceScheduleSaturationWindow_));
            exposureServiceScheduleCutFactor_ = search.value(
                "exposure-service-schedule-cut-factor",
                exposureServiceScheduleCutFactor_);
            if (!std::isfinite(exposureServiceScheduleCutFactor_)
                || exposureServiceScheduleCutFactor_ < 0.0
                || exposureServiceScheduleCutFactor_ > 1.0) {
                exposureServiceScheduleCutFactor_ = 0.5;
            }
            exposureServiceScheduleStallFactor_ = search.value(
                "exposure-service-schedule-stall-factor",
                exposureServiceScheduleStallFactor_);
            if (!std::isfinite(exposureServiceScheduleStallFactor_)
                || exposureServiceScheduleStallFactor_ < 0.0
                || exposureServiceScheduleStallFactor_ > 1.0) {
                exposureServiceScheduleStallFactor_ = 0.3;
            }
            beliefConcentrationWeight_ = search.value(
                "belief-concentration-weight",
                beliefConcentrationWeight_);
            if (!std::isfinite(beliefConcentrationWeight_) || beliefConcentrationWeight_ < 0.0) {
                beliefConcentrationWeight_ = 0.0;
            }
            beliefConcentrationRadiusM_ = search.value(
                "belief-concentration-radius-m",
                beliefConcentrationRadiusM_);
            if (!std::isfinite(beliefConcentrationRadiusM_) || beliefConcentrationRadiusM_ <= 0.0) {
                beliefConcentrationRadiusM_ = std::max(spacing_ * 3.0, 300.0);
            }
            beliefConcentrationSigmaM_ = search.value(
                "belief-concentration-sigma-m",
                beliefConcentrationSigmaM_);
            if (!std::isfinite(beliefConcentrationSigmaM_) || beliefConcentrationSigmaM_ <= 0.0) {
                beliefConcentrationSigmaM_ = beliefConcentrationRadiusM_ * 0.5;
            }
            beliefConcentrationMode_ = search.value(
                "belief-concentration-mode",
                beliefConcentrationMode_);
            if (beliefConcentrationMode_ != "mass"
                && beliefConcentrationMode_ != "ridge"
                && beliefConcentrationMode_ != "gradient"
                && beliefConcentrationMode_ != "information_gain"
                && beliefConcentrationMode_ != "explore_mass"
                && beliefConcentrationMode_ != "verify"
                && beliefConcentrationMode_ != "hybrid") {
                beliefConcentrationMode_ = "mass";
            }
            if (search.contains("target-prior-center")) {
                const json &center = search.at("target-prior-center");
                targetPriorCenter_ = Point(
                    center.value("x", bridge_.target.x),
                    center.value("y", bridge_.target.y)
                );
                hasTargetPriorCenter_ = true;
            }
        }
        applyTargetPrior();
        normalizeBelief();
    }

    void observeCells(const json &updates, double runtime) {
        for (const auto &update : updates) {
            if (!update.is_array() || update.size() != 2) {
                continue;
            }
            int x = update.at(0).get<int>();
            int y = update.at(1).get<int>();
            if (!inside(x, y)) {
                continue;
            }
            size_t idx = index(x, y);
            searched_[idx] = true;
            if (cellContainsTargetCenter(x, y)) {
                markDetected(runtime);
            } else {
                belief_[idx] *= negativeDetectionFactor_;
            }
        }
        normalizeBelief();
        lastRuntime_ = runtime;
        recordScheduleHistory(runtime);
    }

    void observeRobots(const std::vector<Point> &positions, double runtime) {
        if (detected_) {
            return;
        }
        Point target(bridge_.target.x, bridge_.target.y);
        for (const Point &position : positions) {
            if (position.distance_to(target) <= bridge_.target.radius) {
                markDetected(runtime);
                normalizeBelief();
                return;
            }
        }
        lastRuntime_ = runtime;
        recordScheduleHistory(runtime);
    }

    json snapshot() const {
        return {
            {"coverage_ratio", coverageRatio()},
            {"belief_entropy", entropy()},
            {"belief_at_target", beliefAtTarget()},
            {"detected", detected_},
            {"detection_time_s", detected_ ? detectionTime_ : -1.0},
            {"last_runtime_s", lastRuntime_}
        };
    }

    Point chooseGoal(const Point &position, const std::string &policy) const {
        return chooseGoalDecision(position, policy, nullptr, nullptr).goal;
    }

    BridgeSearchGoalDecision chooseGoalWithDiagnostics(const Point &position, const std::string &policy) const {
        return chooseGoalDecision(position, policy, nullptr, nullptr);
    }

    Point chooseGoal(
        const Point &position,
        const std::string &policy,
        const Point &anchor,
        const BridgeTopologyConfig &topologyConfig
    ) const {
        return chooseGoalDecision(position, policy, &anchor, &topologyConfig).goal;
    }

    BridgeSearchGoalDecision chooseGoalWithDiagnostics(
        const Point &position,
        const std::string &policy,
        const Point &anchor,
        const BridgeTopologyConfig &topologyConfig
    ) const {
        return chooseGoalDecision(position, policy, &anchor, &topologyConfig);
    }

    Point chooseGoal(
        const Point &position,
        const std::string &policy,
        const Point *anchor,
        const BridgeTopologyConfig *topologyConfig
    ) const {
        return chooseGoalDecision(position, policy, anchor, topologyConfig).goal;
    }

    BridgeSearchGoalDecision chooseGoalDecision(
        const Point &position,
        const std::string &policy,
        const Point *anchor,
        const BridgeTopologyConfig *topologyConfig
    ) const {
        BridgeSearchGoalDecision decision;
        bool useExposureServiceSchedule = policy == "active-predictive-exposure"
                                          && exposureServiceScheduleRateCellsPerS_ > 0.0
                                          && std::isfinite(exposureServiceScheduleRateCellsPerS_);
        decision.exposureServiceScheduleEnabled = useExposureServiceSchedule;
        decision.searchedCells = static_cast<double>(searchedCellCount());
        if (useExposureServiceSchedule) {
            double effectiveRate = exposureServiceScheduleRateCellsPerS_;
            if (exposureServiceScheduleAdaptive_) {
                effectiveRate = adaptiveScheduleRate();
            }
            decision.requiredSearchedCells = std::max(
                0.0,
                effectiveRate * lastRuntime_ - exposureServiceScheduleSlackCells_);
            decision.serviceScheduleDeficit = std::max(
                0.0,
                decision.requiredSearchedCells - decision.searchedCells);
            decision.serviceScheduleDue = decision.serviceScheduleDeficit > 1.0e-9;
        }
        bool useActiveBelief = (policy == "active"
                                || policy == "active-predictive"
                                || policy == "active-predictive-exposure") && !detected_;
        if (decision.serviceScheduleDue) {
            useActiveBelief = false;
        }
        bool usePredictivePenalty = (policy == "active-predictive"
                                     || policy == "active-predictive-exposure")
                                    && anchor != nullptr
                                    && topologyConfig != nullptr
                                    && predictiveFeasibilityWeight_ > 0.0
                                    && std::isfinite(predictiveFeasibilityWeight_);
        bool useExposureUtility = policy == "active-predictive-exposure"
                                  && exposureWeight_ > 0.0
                                  && std::isfinite(exposureWeight_)
                                  && exposureRadius_ > 0.0
                                  && std::isfinite(exposureRadius_)
                                  && exposureHalfAngleRad_ > 0.0
                                  && std::isfinite(exposureHalfAngleRad_)
                                  && !decision.serviceScheduleDue;
        bool useExposureServiceGate = useExposureUtility
                                      && (exposureServiceGateMinCells_ > 0.0
                                          || exposureServiceGateRatio_ > 0.0);
        decision.predictiveEnabled = usePredictivePenalty;
        decision.exposureEnabled = useExposureUtility;
        decision.exposureServiceGateEnabled = useExposureServiceGate;
        decision.requiredRobustMargin = predictiveRobustMargin_;
        bool useBeliefConcentration = useActiveBelief
                                      && beliefConcentrationWeight_ > 0.0
                                      && std::isfinite(beliefConcentrationWeight_)
                                      && !decision.serviceScheduleDue;
        decision.beliefConcentrationEnabled = useBeliefConcentration;
        Point beliefCentroid(0.0, 0.0);
        double topBeliefMass = 0.0;
        double ridgeSigma = beliefConcentrationSigmaM_;
        if (ridgeSigma <= 0.0 || !std::isfinite(ridgeSigma)) {
            ridgeSigma = std::max(spacing_ * 1.5, 150.0);
        }
        if (useBeliefConcentration) {
            beliefCentroidAndTopMass(beliefCentroid, topBeliefMass, ridgeSigma);
            decision.beliefCentroidX = beliefCentroid.x;
            decision.beliefCentroidY = beliefCentroid.y;
            decision.topBeliefMass = topBeliefMass;
            decision.peakSearchedFraction = peakSearchedFraction(beliefCentroid);
        }
        if (useExposureServiceGate) {
            for (int y = 0; y < height_; ++y) {
                for (int x = 0; x < width_; ++x) {
                    Point center((static_cast<double>(x) + 0.5) * spacing_,
                                 (static_cast<double>(y) + 0.5) * spacing_);
                    if (!insideGoalBoundaryMargin(center)) {
                        continue;
                    }
                    decision.maxServiceUtility = std::max(
                        decision.maxServiceUtility,
                        frontSectorServiceUtility(position, center));
                }
            }
            decision.requiredServiceUtility = std::max(
                exposureServiceGateMinCells_,
                exposureServiceGateRatio_ * decision.maxServiceUtility);
        }
        double bestScore = -std::numeric_limits<double>::infinity();
        double bestBaselineScore = -std::numeric_limits<double>::infinity();
        int bestX = 0;
        int bestY = 0;
        int bestBaselineX = 0;
        int bestBaselineY = 0;
        bool found = false;
        bool foundBaseline = false;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                size_t idx = index(x, y);
                double clarityNeed = searched_[idx] ? 0.0 : 1.0;
                Point center((static_cast<double>(x) + 0.5) * spacing_,
                             (static_cast<double>(y) + 0.5) * spacing_);
                if (!insideGoalBoundaryMargin(center)) {
                    continue;
                }
                ++decision.evaluatedCandidates;
                double travel = position.distance_to(center);
                double baselineScore = clarityWeight_ * clarityNeed - travelWeight_ * travel;
                if (useActiveBelief) {
                    baselineScore += beliefWeight_ * belief_[idx] * static_cast<double>(belief_.size());
                }
                if (baselineScore > bestBaselineScore) {
                    bestBaselineScore = baselineScore;
                    bestBaselineX = x;
                    bestBaselineY = y;
                    foundBaseline = true;
                }
                double score = baselineScore;
                BridgeSearchPenaltyDiagnostics penalty;
                double exposureUtility = 0.0;
                double serviceUtility = 0.0;
                bool serviceEligible = true;
                if (useExposureUtility) {
                    exposureUtility = frontSectorExposureUtility(position, center);
                    decision.maxExposureUtility = std::max(decision.maxExposureUtility, exposureUtility);
                    if (useExposureServiceGate) {
                        serviceUtility = frontSectorServiceUtility(position, center);
                        serviceEligible = serviceUtility + 1.0e-9 >= decision.requiredServiceUtility;
                        if (!serviceEligible && exposureUtility > 0.0) {
                            ++decision.serviceRejectedCandidates;
                        }
                    }
                    if (serviceEligible) {
                        score += exposureWeight_ * exposureUtility;
                    } else {
                        exposureUtility = 0.0;
                    }
                }
                if (usePredictivePenalty) {
                    penalty = predictiveFeasibilityDiagnostics(position, center, *anchor, *topologyConfig);
                    score -= penalty.penalty;
                    if (penalty.penalty > 0.0) {
                        ++decision.penalizedCandidates;
                    }
                    decision.maxPenalty = std::max(decision.maxPenalty, penalty.penalty);
                }
                double concentrationUtility = 0.0;
                if (useBeliefConcentration) {
                    concentrationUtility = beliefConcentrationUtility(
                        center, beliefCentroid, topBeliefMass, ridgeSigma,
                        belief_[idx], searched_[idx],
                        decision.peakSearchedFraction);
                    decision.maxBeliefConcentrationUtility = std::max(
                        decision.maxBeliefConcentrationUtility,
                        concentrationUtility);
                    score += beliefConcentrationWeight_ * concentrationUtility;
                }
                if (score > bestScore) {
                    bestScore = score;
                    bestX = x;
                    bestY = y;
                    found = true;
                    decision.selectedPenalty = penalty.penalty;
                    decision.selectedMinRobustMargin = penalty.minRobustMargin;
                    decision.selectedBoundaryDeficit = penalty.boundaryDeficit;
                    decision.selectedExposureUtility = exposureUtility;
                    decision.selectedServiceUtility = serviceUtility;
                    decision.selectedExposureServiceEligible = serviceEligible;
                    decision.selectedBeliefConcentrationUtility = concentrationUtility;
                }
            }
        }
        if (!found || !foundBaseline) {
            Point fallback = fallbackGoal(position);
            decision.goal = fallback;
            decision.baselineGoal = fallback;
            return decision;
        }
        decision.goal = Point((static_cast<double>(bestX) + 0.5) * spacing_,
                              (static_cast<double>(bestY) + 0.5) * spacing_);
        decision.baselineGoal = Point((static_cast<double>(bestBaselineX) + 0.5) * spacing_,
                                      (static_cast<double>(bestBaselineY) + 0.5) * spacing_);
        decision.predictiveChangedGoal = usePredictivePenalty && decision.goal.distance_to(decision.baselineGoal) > 1.0e-6;
        decision.exposureChangedGoal = useExposureUtility
                                       && decision.selectedExposureUtility > 0.0
                                       && decision.goal.distance_to(decision.baselineGoal) > 1.0e-6;
        decision.beliefConcentrationChangedGoal = useBeliefConcentration
                                                   && decision.selectedBeliefConcentrationUtility > 0.0
                                                   && decision.goal.distance_to(decision.baselineGoal) > 1.0e-6;
        return decision;
    }

    json finalMapJson() const {
        json rows = json::array();
        for (int y = 0; y < height_; ++y) {
            json row = json::array();
            for (int x = 0; x < width_; ++x) {
                size_t idx = index(x, y);
                row.push_back({
                    {"searched", searched_[idx]},
                    {"belief", belief_[idx]}
                });
            }
            rows.push_back(row);
        }
        return rows;
    }

    double coverageRatio() const {
        int searchedCount = searchedCellCount();
        return searched_.empty() ? 0.0 : static_cast<double>(searchedCount) / static_cast<double>(searched_.size());
    }

    double entropy() const {
        return bridgeEntropy(belief_);
    }

    void recordScheduleHistory(double runtime) {
        if (!exposureServiceScheduleAdaptive_) {
            return;
        }
        double sampleCoverage = coverageRatio();
        double sampleEntropy = entropy();
        if (!scheduleHistory_.empty()
            && scheduleHistory_.back().runtime >= runtime) {
            return;
        }
        scheduleHistory_.push_back({runtime, sampleCoverage, sampleEntropy});
        int window = std::max(2, exposureServiceScheduleSaturationWindow_ * 4 + 1);
        while (static_cast<int>(scheduleHistory_.size()) > window) {
            scheduleHistory_.pop_front();
        }
    }

    double adaptiveScheduleRate() const {
        double baseRate = exposureServiceScheduleRateCellsPerS_;
        double floorRate = std::min(exposureServiceScheduleRateMinCellsPerS_, baseRate);
        if (baseRate <= floorRate) {
            return baseRate;
        }
        int unit = std::max(1, exposureServiceScheduleSaturationWindow_);
        int entropyWindow = unit * 4;
        int historySize = static_cast<int>(scheduleHistory_.size());
        if (historySize < entropyWindow + 1) {
            return baseRate;
        }
        const ScheduleSample &latest = scheduleHistory_.back();
        const ScheduleSample &entropyPast = scheduleHistory_[historySize - 1 - entropyWindow];
        if (latest.runtime <= entropyPast.runtime) {
            return baseRate;
        }
        double dt = latest.runtime - entropyPast.runtime;
        double entropyDelta = latest.entropy - entropyPast.entropy;
        double entropySlope = entropyDelta / dt;
        bool entropySaturated = entropySlope >= -1.0e-4;
        const ScheduleSample &coveragePast = scheduleHistory_[historySize - 1 - unit];
        double coverageGrowth = latest.coverage - coveragePast.coverage;
        double medianCoverageGrowth = rollingMedianCoverageGrowth(unit);
        bool coverageStall = medianCoverageGrowth > 0.0
                             && coverageGrowth < 0.5 * medianCoverageGrowth;
        if (!entropySaturated) {
            return baseRate;
        }
        double retained = coverageStall ? exposureServiceScheduleStallFactor_
                                        : exposureServiceScheduleCutFactor_;
        return floorRate + retained * (baseRate - floorRate);
    }

    double rollingMedianCoverageGrowth(int window) const {
        int historySize = static_cast<int>(scheduleHistory_.size());
        if (historySize < 2) {
            return 0.0;
        }
        std::vector<double> growths;
        growths.reserve(static_cast<size_t>(historySize - 1));
        for (int i = 1; i < historySize; ++i) {
            growths.push_back(scheduleHistory_[i].coverage - scheduleHistory_[i - 1].coverage);
        }
        std::sort(growths.begin(), growths.end());
        return growths[growths.size() / 2];
    }

    double beliefAtTarget() const {
        int tx = clampCell(static_cast<int>(std::floor(bridge_.target.x / spacing_)), width_);
        int ty = clampCell(static_cast<int>(std::floor(bridge_.target.y / spacing_)), height_);
        return belief_[index(tx, ty)];
    }

    bool detected() const {
        return detected_;
    }

    double detectionTime() const {
        return detectionTime_;
    }

private:
    BridgeExperimentConfig bridge_;
    int width_ = 1;
    int height_ = 1;
    double spacing_ = 100.0;
    double negativeDetectionFactor_ = 0.85;
    double positiveDetectionFactor_ = 3.0;
    double clarityWeight_ = 1.0;
    double beliefWeight_ = 4.0;
    double travelWeight_ = 0.001;
    double targetPriorStrength_ = 0.0;
    double targetPriorSigma_ = 0.0;
    Point targetPriorCenter_;
    bool hasTargetPriorCenter_ = false;
    double goalBoundaryMargin_ = 0.0;
    double predictiveFeasibilityWeight_ = 0.0;
    double predictiveRobustMargin_ = 0.0;
    int predictiveHorizon_ = 1;
    double predictiveStep_ = 0.0;
    double exposureWeight_ = 0.0;
    double exposureRadius_ = 0.0;
    double exposureHalfAngleRad_ = M_PI / 4.0;
    bool exposureUnsearchedOnly_ = true;
    int exposureLookaheadSteps_ = 1;
    double exposureLookaheadStep_ = 0.0;
    double exposureLookaheadDiscount_ = 1.0;
    double exposureServiceGateMinCells_ = 0.0;
    double exposureServiceGateRatio_ = 0.0;
    double exposureServiceScheduleRateCellsPerS_ = 0.0;
    double exposureServiceScheduleSlackCells_ = 0.0;
    bool exposureServiceScheduleAdaptive_ = false;
    double exposureServiceScheduleRateMinCellsPerS_ = 0.0;
    int exposureServiceScheduleSaturationWindow_ = 4;
    double exposureServiceScheduleCutFactor_ = 0.5;
    double exposureServiceScheduleStallFactor_ = 0.3;
    double beliefConcentrationWeight_ = 0.0;
    double beliefConcentrationRadiusM_ = 300.0;
    double beliefConcentrationSigmaM_ = 150.0;
    std::string beliefConcentrationMode_ = "mass";
    std::deque<ScheduleSample> scheduleHistory_;
    bool detected_ = false;
    double detectionTime_ = -1.0;
    double lastRuntime_ = 0.0;
    std::vector<double> belief_;
    std::vector<bool> searched_;

    bool inside(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    size_t index(int x, int y) const {
        return static_cast<size_t>(y * width_ + x);
    }

    int searchedCellCount() const {
        int searchedCount = 0;
        for (bool value : searched_) {
            if (value) {
                ++searchedCount;
            }
        }
        return searchedCount;
    }

    bool cellContainsTargetCenter(int x, int y) const {
        int tx = clampCell(static_cast<int>(std::floor(bridge_.target.x / spacing_)), width_);
        int ty = clampCell(static_cast<int>(std::floor(bridge_.target.y / spacing_)), height_);
        return x == tx && y == ty;
    }

    void markDetected(double runtime) {
        if (detected_) {
            return;
        }
        detected_ = true;
        detectionTime_ = runtime;
        int tx = clampCell(static_cast<int>(std::floor(bridge_.target.x / spacing_)), width_);
        int ty = clampCell(static_cast<int>(std::floor(bridge_.target.y / spacing_)), height_);
        belief_[index(tx, ty)] *= positiveDetectionFactor_;
    }

    static int clampCell(int value, int upper) {
        return std::max(0, std::min(value, upper - 1));
    }

    void normalizeBelief() {
        double total = std::accumulate(belief_.begin(), belief_.end(), 0.0);
        if (total <= 0.0) {
            double uniform = 1.0 / static_cast<double>(belief_.size());
            std::fill(belief_.begin(), belief_.end(), uniform);
            return;
        }
        for (double &value : belief_) {
            value /= total;
        }
    }

    void applyTargetPrior() {
        if (targetPriorStrength_ <= 0.0 || !std::isfinite(targetPriorStrength_)) {
            return;
        }
        double sigma = targetPriorSigma_;
        if (sigma <= 0.0 || !std::isfinite(sigma)) {
            sigma = std::max(bridge_.target.radius, spacing_);
        }
        sigma = std::max(sigma, 1e-6);
        Point target = hasTargetPriorCenter_ ? targetPriorCenter_ : Point(bridge_.target.x, bridge_.target.y);
        double denominator = 2.0 * sigma * sigma;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                Point center((static_cast<double>(x) + 0.5) * spacing_,
                             (static_cast<double>(y) + 0.5) * spacing_);
                double distance = center.distance_to(target);
                belief_[index(x, y)] += targetPriorStrength_ * std::exp(-(distance * distance) / denominator);
            }
        }
    }

    bool insideGoalBoundaryMargin(const Point &center) const {
        if (goalBoundaryMargin_ <= 0.0 || !std::isfinite(goalBoundaryMargin_)) {
            return true;
        }
        double maxX = static_cast<double>(width_) * spacing_;
        double maxY = static_cast<double>(height_) * spacing_;
        return center.x >= goalBoundaryMargin_
               && center.y >= goalBoundaryMargin_
               && center.x <= maxX - goalBoundaryMargin_
               && center.y <= maxY - goalBoundaryMargin_;
    }

    Point fallbackGoal(const Point &position) const {
        return Point(std::max(spacing_ * 0.5, std::min(position.x, (static_cast<double>(width_) - 0.5) * spacing_)),
                     std::max(spacing_ * 0.5, std::min(position.y, (static_cast<double>(height_) - 0.5) * spacing_)));
    }

    double frontSectorExposureUtilityAtSensor(
        const Point &sensor,
        double headingAngle,
        std::vector<bool> *countedCells
    ) const {
        double utility = 0.0;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                size_t idx = index(x, y);
                if (countedCells != nullptr && (*countedCells)[idx]) {
                    continue;
                }
                if (exposureUnsearchedOnly_ && searched_[idx]) {
                    continue;
                }
                Point center((static_cast<double>(x) + 0.5) * spacing_,
                             (static_cast<double>(y) + 0.5) * spacing_);
                Point relative = center - sensor;
                double distance = relative.len();
                if (distance <= 1.0e-9 || distance > exposureRadius_) {
                    continue;
                }
                double bearing = std::atan2(relative.y, relative.x);
                double angleError = std::atan2(std::sin(bearing - headingAngle), std::cos(bearing - headingAngle));
                if (std::abs(angleError) <= exposureHalfAngleRad_) {
                    utility += belief_[idx];
                    if (countedCells != nullptr) {
                        (*countedCells)[idx] = true;
                    }
                }
            }
        }
        return utility;
    }

    double frontSectorExposureUtility(const Point &position, const Point &candidate) const {
        Point heading = candidate - position;
        double headingNorm = heading.len();
        if (headingNorm <= 1.0e-9) {
            return 0.0;
        }
        double headingAngle = std::atan2(heading.y, heading.x);
        if (exposureLookaheadSteps_ <= 1) {
            return frontSectorExposureUtilityAtSensor(candidate, headingAngle, nullptr);
        }

        double step = exposureLookaheadStep_;
        if (step <= 0.0 || !std::isfinite(step)) {
            step = std::max(spacing_ * 0.5, exposureRadius_ * 0.5);
        }
        std::vector<bool> countedCells(belief_.size(), false);
        Point simulated = position;
        double discount = 1.0;
        double utility = 0.0;
        bool reachedCandidate = false;
        for (int i = 0; i < exposureLookaheadSteps_; ++i) {
            Point direction = candidate - simulated;
            double distance = direction.len();
            if (distance <= 1.0e-9) {
                reachedCandidate = true;
                break;
            }
            Point unit = direction / distance;
            double travel = std::min(step, distance);
            simulated = simulated + unit * travel;
            if (travel + 1.0e-9 >= distance) {
                reachedCandidate = true;
            }
            utility += discount * frontSectorExposureUtilityAtSensor(simulated, headingAngle, &countedCells);
            discount *= exposureLookaheadDiscount_;
            if (reachedCandidate) {
                break;
            }
        }
        if (!reachedCandidate) {
            utility += discount * frontSectorExposureUtilityAtSensor(candidate, headingAngle, &countedCells);
        }
        return utility;
    }

    double frontSectorServiceUtilityAtSensor(
        const Point &sensor,
        double headingAngle,
        std::vector<bool> *countedCells
    ) const {
        double utility = 0.0;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                size_t idx = index(x, y);
                if (searched_[idx]) {
                    continue;
                }
                if (countedCells != nullptr && (*countedCells)[idx]) {
                    continue;
                }
                Point center((static_cast<double>(x) + 0.5) * spacing_,
                             (static_cast<double>(y) + 0.5) * spacing_);
                Point relative = center - sensor;
                double distance = relative.len();
                if (distance <= 1.0e-9 || distance > exposureRadius_) {
                    continue;
                }
                double bearing = std::atan2(relative.y, relative.x);
                double angleError = std::atan2(std::sin(bearing - headingAngle), std::cos(bearing - headingAngle));
                if (std::abs(angleError) <= exposureHalfAngleRad_) {
                    utility += 1.0;
                    if (countedCells != nullptr) {
                        (*countedCells)[idx] = true;
                    }
                }
            }
        }
        return utility;
    }

    double frontSectorServiceUtility(const Point &position, const Point &candidate) const {
        Point heading = candidate - position;
        double headingNorm = heading.len();
        if (headingNorm <= 1.0e-9) {
            return 0.0;
        }
        double headingAngle = std::atan2(heading.y, heading.x);
        if (exposureLookaheadSteps_ <= 1) {
            return frontSectorServiceUtilityAtSensor(candidate, headingAngle, nullptr);
        }

        double step = exposureLookaheadStep_;
        if (step <= 0.0 || !std::isfinite(step)) {
            step = std::max(spacing_ * 0.5, exposureRadius_ * 0.5);
        }
        std::vector<bool> countedCells(searched_.size(), false);
        Point simulated = position;
        double utility = 0.0;
        bool reachedCandidate = false;
        for (int i = 0; i < exposureLookaheadSteps_; ++i) {
            Point direction = candidate - simulated;
            double distance = direction.len();
            if (distance <= 1.0e-9) {
                reachedCandidate = true;
                break;
            }
            Point unit = direction / distance;
            double travel = std::min(step, distance);
            simulated = simulated + unit * travel;
            if (travel + 1.0e-9 >= distance) {
                reachedCandidate = true;
            }
            utility += frontSectorServiceUtilityAtSensor(simulated, headingAngle, &countedCells);
            if (reachedCandidate) {
                break;
            }
        }
        if (!reachedCandidate) {
            utility += frontSectorServiceUtilityAtSensor(candidate, headingAngle, &countedCells);
        }
        return utility;
    }

    BridgeSearchPenaltyDiagnostics predictiveFeasibilityDiagnostics(
        const Point &position,
        const Point &candidate,
        const Point &anchor,
        const BridgeTopologyConfig &topologyConfig
    ) const {
        BridgeSearchPenaltyDiagnostics diagnostics;
        if (predictiveHorizon_ <= 0) {
            return diagnostics;
        }
        double step = predictiveStep_;
        if (step <= 0.0 || !std::isfinite(step)) {
            step = spacing_ * 0.5;
        }
        double minRobustMargin = std::numeric_limits<double>::infinity();
        double boundaryDeficit = 0.0;
        Point simulated = position;
        for (int i = 0; i < predictiveHorizon_; ++i) {
            Point direction = candidate - simulated;
            double distance = direction.len();
            if (distance <= 1.0e-9) {
                break;
            }
            Point unit = direction / distance;
            simulated = simulated + unit * std::min(step, distance);
            minRobustMargin = std::min(minRobustMargin, bridgeRobustMargin(anchor, simulated, topologyConfig));
            if (goalBoundaryMargin_ > 0.0 && std::isfinite(goalBoundaryMargin_)) {
                double maxX = static_cast<double>(width_) * spacing_;
                double maxY = static_cast<double>(height_) * spacing_;
                double boundaryMargin = std::min({
                    simulated.x,
                    simulated.y,
                    maxX - simulated.x,
                    maxY - simulated.y
                });
                boundaryDeficit += std::max(0.0, goalBoundaryMargin_ - boundaryMargin);
            }
        }
        if (!std::isfinite(minRobustMargin)) {
            minRobustMargin = bridgeRobustMargin(anchor, candidate, topologyConfig);
        }
        double robustDeficit = std::max(0.0, predictiveRobustMargin_ - minRobustMargin);
        diagnostics.penalty = predictiveFeasibilityWeight_ * (robustDeficit + boundaryDeficit);
        diagnostics.minRobustMargin = minRobustMargin;
        diagnostics.boundaryDeficit = boundaryDeficit;
        return diagnostics;
    }

    double predictiveFeasibilityPenalty(
        const Point &position,
        const Point &candidate,
        const Point &anchor,
        const BridgeTopologyConfig &topologyConfig
    ) const {
        return predictiveFeasibilityDiagnostics(position, candidate, anchor, topologyConfig).penalty;
    }

    void beliefCentroidAndTopMass(Point &centroid, double &topMass, double sigma) const {
        double totalMass = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                double mass = belief_[index(x, y)];
                totalMass += mass;
                sumX += mass * (static_cast<double>(x) + 0.5) * spacing_;
                sumY += mass * (static_cast<double>(y) + 0.5) * spacing_;
            }
        }
        if (totalMass <= 0.0 || !std::isfinite(totalMass)) {
            centroid = Point(static_cast<double>(width_) * spacing_ * 0.5,
                             static_cast<double>(height_) * spacing_ * 0.5);
            topMass = 0.0;
            return;
        }
        centroid = Point(sumX / totalMass, sumY / totalMass);
        double cutoff = beliefConcentrationRadiusM_;
        if (cutoff <= 0.0 || !std::isfinite(cutoff)) {
            cutoff = std::max(spacing_ * 3.0, 300.0);
        }
        double cutoffSq = cutoff * cutoff;
        double mass = 0.0;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                Point center((static_cast<double>(x) + 0.5) * spacing_,
                             (static_cast<double>(y) + 0.5) * spacing_);
                double dx = center.x - centroid.x;
                double dy = center.y - centroid.y;
                if (dx * dx + dy * dy <= cutoffSq) {
                    mass += belief_[index(x, y)];
                }
            }
        }
        topMass = mass;
    }

    double peakSearchedFraction(const Point &centroid) const {
        double cutoff = beliefConcentrationRadiusM_;
        if (cutoff <= 0.0 || !std::isfinite(cutoff)) {
            cutoff = std::max(spacing_ * 3.0, 300.0);
        }
        double cutoffSq = cutoff * cutoff;
        double searchedMass = 0.0;
        double totalMass = 0.0;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                Point center((static_cast<double>(x) + 0.5) * spacing_,
                             (static_cast<double>(y) + 0.5) * spacing_);
                double dx = center.x - centroid.x;
                double dy = center.y - centroid.y;
                if (dx * dx + dy * dy > cutoffSq) {
                    continue;
                }
                double m = belief_[index(x, y)];
                if (m <= 0.0) {
                    continue;
                }
                totalMass += m;
                if (searched_[index(x, y)]) {
                    searchedMass += m;
                }
            }
        }
        if (totalMass <= 0.0 || !std::isfinite(totalMass)) {
            return 0.0;
        }
        double frac = searchedMass / totalMass;
        if (!std::isfinite(frac)) {
            return 0.0;
        }
        if (frac < 0.0) {
            return 0.0;
        }
        if (frac > 1.0) {
            return 1.0;
        }
        return frac;
    }

    double maxCellBelief() const {
        double m = 0.0;
        for (double v : belief_) {
            if (v > m) {
                m = v;
            }
        }
        return m;
    }

    double beliefConcentrationUtility(
        const Point &candidate,
        const Point &centroid,
        double topMass,
        double sigma,
        double candidateBelief,
        bool candidateSearched,
        double peakSearchedFrac
    ) const {
        double dx = candidate.x - centroid.x;
        double dy = candidate.y - centroid.y;
        double distanceSq = dx * dx + dy * dy;
        if (beliefConcentrationMode_ == "information_gain") {
            if (candidateSearched) {
                return 0.0;
            }
            double n = static_cast<double>(belief_.size());
            double b = candidateBelief * n;
            double variance = b * (1.0 - b);
            if (variance <= 0.0 || !std::isfinite(variance)) {
                return 0.0;
            }
            double sigmaSq = sigma * sigma;
            if (sigmaSq <= 0.0 || !std::isfinite(sigmaSq)) {
                sigmaSq = (std::max(spacing_ * 1.5, 150.0)) * (std::max(spacing_ * 1.5, 150.0));
            }
            double ridgeWeight = std::exp(-distanceSq / (2.0 * sigmaSq));
            return variance * ridgeWeight;
        }
        if (beliefConcentrationMode_ == "verify") {
            double n = static_cast<double>(belief_.size());
            double b = candidateBelief * n;
            double sigmaSq = sigma * sigma;
            if (sigmaSq <= 0.0 || !std::isfinite(sigmaSq)) {
                sigmaSq = (std::max(spacing_ * 1.5, 150.0)) * (std::max(spacing_ * 1.5, 150.0));
            }
            double ridgeWeight = std::exp(-distanceSq / (2.0 * sigmaSq));
            return b * ridgeWeight;
        }
        if (beliefConcentrationMode_ == "hybrid") {
            double sigmaSq = sigma * sigma;
            if (sigmaSq <= 0.0 || !std::isfinite(sigmaSq)) {
                sigmaSq = (std::max(spacing_ * 1.5, 150.0)) * (std::max(spacing_ * 1.5, 150.0));
            }
            double ridgeWeight = std::exp(-distanceSq / (2.0 * sigmaSq));
            double gate = peakSearchedFrac;
            if (!std::isfinite(gate)) {
                gate = 0.0;
            }
            if (gate < 0.0) {
                gate = 0.0;
            }
            if (gate > 1.0) {
                gate = 1.0;
            }
            double bRel = candidateBelief;
            double maxB = maxCellBelief();
            if (maxB > 0.0 && std::isfinite(maxB)) {
                bRel = candidateBelief / maxB;
            }
            if (bRel < 0.0) {
                bRel = 0.0;
            }
            if (bRel > 1.0) {
                bRel = 1.0;
            }
            double verifyGate = 0.0;
            if (gate >= 0.75) {
                verifyGate = (gate - 0.75) / 0.25;
                if (verifyGate > 1.0) {
                    verifyGate = 1.0;
                }
            }
            double exploreGate = 1.0 - verifyGate;
            double verifyTerm = 0.0;
            if (candidateSearched && verifyGate > 0.0) {
                verifyTerm = verifyGate * bRel;
            }
            double exploreTerm = 0.0;
            if (!candidateSearched && exploreGate > 0.0) {
                exploreTerm = exploreGate * bRel * 0.25;
            }
            return (verifyTerm + exploreTerm) * ridgeWeight;
        }
        if (beliefConcentrationMode_ == "explore_mass") {
            if (candidateSearched) {
                return 0.0;
            }
            double sigmaSq = sigma * sigma;
            if (sigmaSq <= 0.0 || !std::isfinite(sigmaSq)) {
                sigmaSq = (std::max(spacing_ * 1.5, 150.0)) * (std::max(spacing_ * 1.5, 150.0));
            }
            double ridgeWeight = std::exp(-distanceSq / (2.0 * sigmaSq));
            return topMass * ridgeWeight;
        }
        if (beliefConcentrationMode_ == "gradient") {
            double distance = std::sqrt(distanceSq);
            if (distance <= 1.0e-9) {
                return topMass;
            }
            double radius = beliefConcentrationRadiusM_;
            if (radius <= 0.0 || !std::isfinite(radius)) {
                radius = std::max(spacing_ * 3.0, 300.0);
            }
            return topMass * std::max(0.0, 1.0 - distance / radius);
        }
        if (beliefConcentrationMode_ == "ridge") {
            double sigmaSq = sigma * sigma;
            if (sigmaSq <= 0.0 || !std::isfinite(sigmaSq)) {
                sigmaSq = (std::max(spacing_ * 1.5, 150.0)) * (std::max(spacing_ * 1.5, 150.0));
            }
            return topMass * std::exp(-distanceSq / (2.0 * sigmaSq));
        }
        double radius = beliefConcentrationRadiusM_;
        if (radius <= 0.0 || !std::isfinite(radius)) {
            radius = std::max(spacing_ * 3.0, 300.0);
        }
        double radiusSq = radius * radius;
        if (distanceSq > radiusSq) {
            return 0.0;
        }
        return topMass;
    }
};

#endif
