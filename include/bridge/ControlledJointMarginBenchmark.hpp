#ifndef CBF_CONTROLLED_JOINT_MARGIN_BENCHMARK_HPP
#define CBF_CONTROLLED_JOINT_MARGIN_BENCHMARK_HPP

#include "bridge/ExactGammaStar2D.hpp"
#include "cbf/PairwiseSecondOrderCBF.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

constexpr double CONTROLLED_JOINT_MARGIN_RANGE_M = 500.0;
constexpr double CONTROLLED_JOINT_MARGIN_ACCEL_BOUND_MPS2 = 2.0;
constexpr double CONTROLLED_JOINT_MARGIN_SAMPLE_TIME_S = 0.5;
constexpr double CONTROLLED_JOINT_MARGIN_TAU_MPS2 = 1.0;
constexpr int CONTROLLED_JOINT_MARGIN_HOMOTOPY_INTERVALS = 8;

enum class ControlledJointMarginPressure {
    Low,
    Critical,
};

enum class ControlledJointMarginController {
    HardQp,
    MarginFeedback,
};

struct ControlledJointMarginSpec {
    ControlledJointMarginPressure pressure = ControlledJointMarginPressure::Low;
    ControlledJointMarginController controller = ControlledJointMarginController::HardQp;
    double lateralOffsetM = 0.0;
};

struct ControlledJointMarginCandidate {
    int index = 0;
    double alpha = 0.0;
    Eigen::Vector2d command = Eigen::Vector2d::Zero();
    double predictedNextGamma = std::numeric_limits<double>::quiet_NaN();
    double taskDeviation = std::numeric_limits<double>::quiet_NaN();
};

struct ControlledJointMarginResult {
    bool valid = false;
    bool currentTaskFeasible = false;
    ControlledJointMarginSpec spec;
    Point initialPosition;
    Eigen::Vector2d initialVelocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d taskCommand = Eigen::Vector2d::Zero();
    Eigen::Vector2d command = Eigen::Vector2d::Zero();
    Point nextPosition;
    Eigen::Vector2d nextVelocity = Eigen::Vector2d::Zero();
    double currentGamma = std::numeric_limits<double>::quiet_NaN();
    double nominalNextGamma = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> nominalNextSingletonGammas;
    int selectedCandidateIndex = 0;
    double selectedAlpha = 0.0;
    double predictedNextGamma = std::numeric_limits<double>::quiet_NaN();
    double actualNextGamma = std::numeric_limits<double>::quiet_NaN();
    std::vector<ControlledJointMarginCandidate> candidates;
};

inline const std::array<double, 5> &controlledJointMarginFrozenOffsets() {
    static const std::array<double, 5> offsets = {
            -0.5, -0.25, 0.0, 0.25, 0.5,
    };
    return offsets;
}

namespace controlled_joint_margin_detail {

inline std::array<Point, 2> bases() {
    return {Point(0.0, -100.0), Point(0.0, 100.0)};
}

inline bool isFrozenOffset(double offset) {
    const auto &offsets = controlledJointMarginFrozenOffsets();
    return std::find(offsets.begin(), offsets.end(), offset) != offsets.end();
}

inline std::vector<PairwiseSecondOrderRow> communicationRows(
        const Point &position,
        const Eigen::Vector2d &velocity) {
    const PairwiseSecondOrderState2D self{
            position,
            velocity,
            Eigen::Vector2d::Zero(),
    };
    const PairwiseSecondOrderRowSpec rowSpec{
            PairwiseSecondOrderBarrierKind::CommunicationUpper,
            CONTROLLED_JOINT_MARGIN_RANGE_M,
            0.0,
            1.0,
            1.0,
            1.0,
            0.0,
    };

    std::vector<PairwiseSecondOrderRow> rows;
    rows.reserve(2);
    for (const auto &base : bases()) {
        rows.push_back(buildPairwiseSecondOrderRow(
                self,
                {base, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                rowSpec));
    }
    return rows;
}

inline std::vector<BridgeGammaStarResidual2D> residuals(
        const std::vector<PairwiseSecondOrderRow> &rows) {
    std::vector<BridgeGammaStarResidual2D> result;
    result.reserve(rows.size());
    for (const auto &row : rows) {
        result.push_back(bridgeGammaStarResidualFromAffineMargin(
                row.uCoe(0), row.uCoe(1), row.constTerm));
    }
    return result;
}

inline BridgeGammaStarSolution2D margin(
        const Point &position,
        const Eigen::Vector2d &velocity) {
    return solveExactBridgeGammaStar2D(
            residuals(communicationRows(position, velocity)),
            CONTROLLED_JOINT_MARGIN_ACCEL_BOUND_MPS2);
}

inline std::pair<Point, Eigen::Vector2d> propagate(
        const Point &position,
        const Eigen::Vector2d &velocity,
        const Eigen::Vector2d &command) {
    const double time = CONTROLLED_JOINT_MARGIN_SAMPLE_TIME_S;
    return {
            Point(position.x + time * velocity(0)
                          + 0.5 * time * time * command(0),
                  position.y + time * velocity(1)
                          + 0.5 * time * time * command(1)),
            velocity + time * command,
    };
}

inline double commandResidual(
        const PairwiseSecondOrderRow &row,
        const Eigen::Vector2d &command) {
    return row.constTerm + row.uCoe.dot(command);
}

inline bool betterAdmittedCandidate(
        const ControlledJointMarginCandidate &candidate,
        const ControlledJointMarginCandidate &best) {
    if (candidate.taskDeviation != best.taskDeviation) {
        return candidate.taskDeviation < best.taskDeviation;
    }
    if (candidate.predictedNextGamma != best.predictedNextGamma) {
        return candidate.predictedNextGamma > best.predictedNextGamma;
    }
    if (candidate.command(0) != best.command(0)) {
        return candidate.command(0) < best.command(0);
    }
    if (candidate.command(1) != best.command(1)) {
        return candidate.command(1) < best.command(1);
    }
    return candidate.index < best.index;
}

inline bool betterFallbackCandidate(
        const ControlledJointMarginCandidate &candidate,
        const ControlledJointMarginCandidate &best) {
    if (candidate.predictedNextGamma != best.predictedNextGamma) {
        return candidate.predictedNextGamma > best.predictedNextGamma;
    }
    return betterAdmittedCandidate(candidate, best);
}

}  // namespace controlled_joint_margin_detail

inline ControlledJointMarginResult runControlledJointMarginCell(
        const ControlledJointMarginSpec &spec) {
    if (!controlled_joint_margin_detail::isFrozenOffset(spec.lateralOffsetM)) {
        throw std::invalid_argument(
                "controlled joint-margin lateral offset is outside the frozen matrix");
    }

    const double initialX = std::sqrt(450.0 * 450.0 - 100.0 * 100.0);
    const Point initialPosition(initialX, spec.lateralOffsetM);
    const Eigen::Vector2d initialVelocity(
            spec.pressure == ControlledJointMarginPressure::Low ? 18.0 : 20.45,
            0.0);
    const Eigen::Vector2d taskCommand(2.0, 0.0);
    const auto currentRows = controlled_joint_margin_detail::communicationRows(
            initialPosition, initialVelocity);
    const auto current = solveExactBridgeGammaStar2D(
            controlled_joint_margin_detail::residuals(currentRows),
            CONTROLLED_JOINT_MARGIN_ACCEL_BOUND_MPS2);
    if (!current.valid) {
        return {};
    }

    bool taskFeasible = taskCommand.cwiseAbs().maxCoeff()
            <= CONTROLLED_JOINT_MARGIN_ACCEL_BOUND_MPS2;
    for (const auto &row : currentRows) {
        taskFeasible = taskFeasible
                && controlled_joint_margin_detail::commandResidual(
                           row, taskCommand) >= 0.0;
    }
    if (!taskFeasible) {
        throw std::runtime_error(
                "frozen task command is unexpectedly outside the current hard set");
    }

    const auto nominalNext = controlled_joint_margin_detail::propagate(
            initialPosition, initialVelocity, taskCommand);
    const auto nominalNextRows = controlled_joint_margin_detail::communicationRows(
            nominalNext.first, nominalNext.second);
    const auto nominalNextMargin = solveExactBridgeGammaStar2D(
            controlled_joint_margin_detail::residuals(nominalNextRows),
            CONTROLLED_JOINT_MARGIN_ACCEL_BOUND_MPS2);
    if (!nominalNextMargin.valid) {
        return {};
    }

    std::vector<double> singletonGammas;
    singletonGammas.reserve(nominalNextRows.size());
    for (const auto &row : nominalNextRows) {
        const auto singleton = solveExactBridgeGammaStar2D(
                controlled_joint_margin_detail::residuals({row}),
                CONTROLLED_JOINT_MARGIN_ACCEL_BOUND_MPS2);
        if (!singleton.valid) {
            return {};
        }
        singletonGammas.push_back(singleton.gamma);
    }

    const Eigen::Vector2d maximumMarginCommand(current.accelX, current.accelY);
    std::vector<ControlledJointMarginCandidate> candidates;
    candidates.reserve(CONTROLLED_JOINT_MARGIN_HOMOTOPY_INTERVALS + 1);
    for (int index = 0;
         index <= CONTROLLED_JOINT_MARGIN_HOMOTOPY_INTERVALS;
         ++index) {
        const double alpha = static_cast<double>(index)
                / CONTROLLED_JOINT_MARGIN_HOMOTOPY_INTERVALS;
        const Eigen::Vector2d candidateCommand =
                (1.0 - alpha) * taskCommand + alpha * maximumMarginCommand;
        const auto candidateNext = controlled_joint_margin_detail::propagate(
                initialPosition, initialVelocity, candidateCommand);
        const auto candidateMargin = controlled_joint_margin_detail::margin(
                candidateNext.first, candidateNext.second);
        if (!candidateMargin.valid) {
            return {};
        }
        candidates.push_back({
                index,
                alpha,
                candidateCommand,
                candidateMargin.gamma,
                (candidateCommand - taskCommand).norm(),
        });
    }

    const ControlledJointMarginCandidate *selected = nullptr;
    for (const auto &candidate : candidates) {
        if (candidate.predictedNextGamma < CONTROLLED_JOINT_MARGIN_TAU_MPS2) {
            continue;
        }
        if (selected == nullptr
            || controlled_joint_margin_detail::betterAdmittedCandidate(
                       candidate, *selected)) {
            selected = &candidate;
        }
    }
    if (selected == nullptr) {
        for (const auto &candidate : candidates) {
            if (selected == nullptr
                || controlled_joint_margin_detail::betterFallbackCandidate(
                           candidate, *selected)) {
                selected = &candidate;
            }
        }
    }
    if (selected == nullptr) {
        return {};
    }

    const Eigen::Vector2d executedCommand =
            spec.controller == ControlledJointMarginController::HardQp
                    ? taskCommand
                    : selected->command;
    const auto actualNext = controlled_joint_margin_detail::propagate(
            initialPosition, initialVelocity, executedCommand);
    const auto actualNextMargin = controlled_joint_margin_detail::margin(
            actualNext.first, actualNext.second);
    if (!actualNextMargin.valid) {
        return {};
    }

    ControlledJointMarginResult result;
    result.valid = true;
    result.currentTaskFeasible = taskFeasible;
    result.spec = spec;
    result.initialPosition = initialPosition;
    result.initialVelocity = initialVelocity;
    result.taskCommand = taskCommand;
    result.command = executedCommand;
    result.nextPosition = actualNext.first;
    result.nextVelocity = actualNext.second;
    result.currentGamma = current.gamma;
    result.nominalNextGamma = nominalNextMargin.gamma;
    result.nominalNextSingletonGammas = std::move(singletonGammas);
    result.selectedCandidateIndex =
            spec.controller == ControlledJointMarginController::HardQp
                    ? 0
                    : selected->index;
    result.selectedAlpha =
            spec.controller == ControlledJointMarginController::HardQp
                    ? 0.0
                    : selected->alpha;
    result.predictedNextGamma =
            spec.controller == ControlledJointMarginController::HardQp
                    ? nominalNextMargin.gamma
                    : selected->predictedNextGamma;
    result.actualNextGamma = actualNextMargin.gamma;
    result.candidates = std::move(candidates);
    return result;
}

inline const char *controlledJointMarginPressureName(
        ControlledJointMarginPressure pressure) {
    return pressure == ControlledJointMarginPressure::Low ? "low" : "critical";
}

inline const char *controlledJointMarginControllerName(
        ControlledJointMarginController controller) {
    return controller == ControlledJointMarginController::HardQp
            ? "hard-qp"
            : "margin-feedback";
}

inline bool controlledJointMarginSha256(const std::string &value) {
    return value.size() == 64
            && std::all_of(value.begin(), value.end(), [](char character) {
                return (character >= '0' && character <= '9')
                        || (character >= 'a' && character <= 'f');
            });
}

inline json controlledJointMarginResultJson(
        const ControlledJointMarginResult &result,
        const std::string &sourceFingerprint) {
    if (!result.valid) {
        throw std::invalid_argument(
                "cannot serialize an invalid controlled joint-margin result");
    }
    if (!controlledJointMarginSha256(sourceFingerprint)) {
        throw std::invalid_argument(
                "controlled joint-margin source fingerprint must be lowercase SHA-256");
    }

    json rows = json::array();
    const auto currentRows = controlled_joint_margin_detail::communicationRows(
            result.initialPosition, result.initialVelocity);
    for (size_t index = 0; index < currentRows.size(); ++index) {
        const auto &row = currentRows.at(index);
        rows.push_back({
                {"identity", "secondOrderFixedCommCBF(base-"
                        + std::to_string(index) + ")"},
                {"reference", "b_" + std::to_string(index)},
                {"control_coefficient", {row.uCoe(0), row.uCoe(1)}},
                {"constant_term_mps2", row.constTerm},
                {"h_m", row.h},
                {"h_dot_mps", row.hdot},
                {"psi_1_mps", row.psi1},
        });
    }

    json candidates = json::array();
    for (const auto &candidate : result.candidates) {
        candidates.push_back({
                {"index", candidate.index},
                {"alpha", candidate.alpha},
                {"command_mps2", {
                        candidate.command(0), candidate.command(1)}},
                {"predicted_next_gamma_star_mps2",
                        candidate.predictedNextGamma},
                {"task_deviation_mps2", candidate.taskDeviation},
        });
    }

    return {
            {"schema", "controlled-joint-margin-cell-v1"},
            {"source_fingerprint_sha256", sourceFingerprint},
            {"specification", {
                    {"pressure", controlledJointMarginPressureName(
                            result.spec.pressure)},
                    {"controller", controlledJointMarginControllerName(
                            result.spec.controller)},
                    {"lateral_offset_m", result.spec.lateralOffsetM},
                    {"fixed_references_m", {
                            {0.0, -100.0}, {0.0, 100.0}}},
                    {"communication_range_m", CONTROLLED_JOINT_MARGIN_RANGE_M},
                    {"acceleration_half_box_mps2",
                            CONTROLLED_JOINT_MARGIN_ACCEL_BOUND_MPS2},
                    {"sample_time_s", CONTROLLED_JOINT_MARGIN_SAMPLE_TIME_S},
                    {"lambda_1_per_s", 1.0},
                    {"lambda_2_per_s", 1.0},
                    {"lookahead_steps", 1},
                    {"homotopy_intervals",
                            CONTROLLED_JOINT_MARGIN_HOMOTOPY_INTERVALS},
                    {"predictive_threshold_mps2",
                            CONTROLLED_JOINT_MARGIN_TAU_MPS2},
            }},
            {"initial_state", {
                    {"position_m", {
                            result.initialPosition.x, result.initialPosition.y}},
                    {"velocity_mps", {
                            result.initialVelocity(0), result.initialVelocity(1)}},
            }},
            {"task_command_mps2", {
                    result.taskCommand(0), result.taskCommand(1)}},
            {"rows", rows},
            {"budget", {
                    {"current_gamma_star_mps2", result.currentGamma},
                    {"current_task_feasible", result.currentTaskFeasible},
                    {"nominal_next_gamma_star_mps2", result.nominalNextGamma},
                    {"nominal_next_singleton_gamma_star_mps2",
                            result.nominalNextSingletonGammas},
            }},
            {"candidates", candidates},
            {"selection", {
                    {"candidate_index", result.selectedCandidateIndex},
                    {"alpha", result.selectedAlpha},
                    {"command_mps2", {result.command(0), result.command(1)}},
                    {"predicted_next_gamma_star_mps2",
                            result.predictedNextGamma},
            }},
            {"execution", {
                    {"next_position_m", {
                            result.nextPosition.x, result.nextPosition.y}},
                    {"next_velocity_mps", {
                            result.nextVelocity(0), result.nextVelocity(1)}},
                    {"actual_next_gamma_star_mps2", result.actualNextGamma},
            }},
    };
}

#endif
