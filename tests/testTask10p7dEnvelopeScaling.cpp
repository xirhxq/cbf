#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedTranslationPrimitive.hpp"
#include "grand_finale/FiniteHorizonWitnessVerifier.hpp"
#include "grand_finale/FiniteTourShadowEnvelope.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kMobileCount = 14;
constexpr std::size_t kFixedCount = 3;
constexpr std::size_t kSlots = 133;
constexpr double kDt = 0.1;
constexpr double kInnovation = 0.05;
constexpr double kRangeVariance = 1.0;
constexpr double kProcessAcceleration = 1.0e-4;
constexpr double kControlError = 1.0e-5;
constexpr double kSensorRadius = 2.0;

std::vector<gf::NodeId> mobileIds() {
    std::vector<gf::NodeId> ids;
    for (gf::NodeId id = 1; id <= kMobileCount; ++id) ids.push_back(id);
    return ids;
}

std::vector<Eigen::Vector2d> initialPositions() {
    std::vector<Eigen::Vector2d> result;
    for (double y : {8.0, 16.0, 24.0, 32.0}) {
        for (double x : {8.0, 16.0, 24.0, 32.0}) {
            if (result.size() == kMobileCount) return result;
            result.push_back({x, y});
        }
    }
    return result;
}

std::map<gf::NodeId, Eigen::Vector2d> fixedPositions() {
    return {{100, {2.0, 2.0}}, {101, {2.0, 38.0}}, {102, {38.0, 2.0}}};
}

std::vector<gf::DirectedEdge> topology() {
    std::vector<gf::DirectedEdge> result;
    for (gf::NodeId owner : mobileIds()) {
        result.push_back({100, owner});
        result.push_back({owner == 2 ? gf::NodeId{1} : gf::NodeId{101}, owner});
    }
    return result;
}

json settings14p3() {
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
    settings["num"] = kMobileCount;
    settings["optimiser"] = "OSQP";
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0, 1, 2}};
    settings["bases"] = {{2.0, 2.0}, {2.0, 38.0}, {38.0, 2.0}};
    settings["initial"]["position"]["positions"] = json::array();
    settings["initial"]["velocity"]["values"] = json::array();
    for (const Eigen::Vector2d& position : initialPositions()) {
        settings["initial"]["position"]["positions"].push_back(
            {position.x(), position.y()});
        settings["initial"]["velocity"]["values"].push_back({0.0, 0.0});
    }
    settings["world"]["boundary"] = {
        {0.0, 0.0}, {40.0, 0.0}, {40.0, 40.0}, {0.0, 40.0}};
    settings["world"]["spacing"] = 1.0;
    settings["searching"]["downward"]["radius"] = kSensorRadius;
    return settings;
}

Eigen::MatrixXd transition() {
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(56, 56);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        result(4 * mobile, 4 * mobile + 2) = kDt;
        result(4 * mobile + 1, 4 * mobile + 3) = kDt;
    }
    return result;
}

Eigen::MatrixXd disturbanceInput() {
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(56, 28);
    Eigen::Matrix<double, 4, 2> block;
    block << 0.5 * kDt * kDt, 0.0, 0.0, 0.5 * kDt * kDt,
             kDt, 0.0, 0.0, kDt;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        result.block<4, 2>(4 * mobile, 2 * mobile) = block;
    return result;
}

struct EnvelopeState {
    gf::ShadowStateBox shadow;
    Eigen::MatrixXd covariance;
};

EnvelopeState initialEnvelope() {
    Eigen::VectorXd lower(56), upper(56);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        lower.segment<4>(4 * mobile) << -0.01, -0.01, -0.002, -0.002;
        upper.segment<4>(4 * mobile) << 0.01, 0.01, 0.002, 0.002;
    }
    return {{lower, upper}, 1.0e-4 * Eigen::MatrixXd::Identity(56, 56)};
}

std::vector<std::string> rangeSlotIds() {
    std::vector<std::string> result;
    for (gf::NodeId first = 1; first <= kMobileCount; ++first) {
        for (gf::NodeId second = first + 1;
             second <= kMobileCount; ++second)
            result.push_back(gf::UndirectedEdge::canonical(first, second).id());
        for (gf::NodeId fixed : {gf::NodeId{100}, gf::NodeId{101}, gf::NodeId{102}})
            result.push_back(gf::UndirectedEdge::canonical(first, fixed).id());
    }
    std::sort(result.begin(), result.end());
    return result;
}

void advanceEnvelope(EnvelopeState& state) {
    const auto F = transition();
    state.shadow = gf::propagateShadowPrediction(
        state.shadow, F, disturbanceInput(),
        Eigen::VectorXd::Constant(28, kProcessAcceleration));
    state.covariance = F * state.covariance * F.transpose();
    const Eigen::VectorXd gain = gf::branchIndependentComponentGainBounds(
        state.covariance, std::sqrt(2.0), kRangeVariance);
    std::vector<gf::ScalarUpdateBound> updates;
    updates.reserve(rangeSlotIds().size());
    for (const std::string& link : rangeSlotIds()) {
        (void)link;
        updates.push_back({gain, kInnovation});
    }
    state.shadow = gf::accumulateScalarUpdateBatch(state.shadow, updates);
}

double axisRadius(const gf::ShadowStateBox& box, std::size_t mobile, int axis) {
    const Eigen::Index index = static_cast<Eigen::Index>(4 * mobile + axis);
    return std::max(std::abs(box.lower(index)), std::abs(box.upper(index)));
}

double positionSupport(const gf::ShadowStateBox& box) {
    double result = 0.0;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        result = std::max(result, std::hypot(
            axisRadius(box, mobile, 0), axisRadius(box, mobile, 1)));
    return result;
}

double velocitySupport(const gf::ShadowStateBox& box) {
    double result = 0.0;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        result = std::max(result, std::hypot(
            axisRadius(box, mobile, 2), axisRadius(box, mobile, 3)));
    return result;
}

gf::SecondOrderBox2D pointState(const Eigen::Vector2d& position) {
    return {{{position.x(), position.x()}, {position.y(), position.y()}},
            {{0.0, 0.0}, {0.0, 0.0}}};
}

gf::SecondOrderBox2D estimateBox(
    const gf::SecondOrderBox2D& truth,
    const gf::ShadowStateBox& shadow,
    std::size_t mobile) {
    const Eigen::Index offset = static_cast<Eigen::Index>(4 * mobile);
    return {{{truth.position.x.lower - shadow.upper(offset),
              truth.position.x.upper - shadow.lower(offset)},
             {truth.position.y.lower - shadow.upper(offset + 1),
              truth.position.y.upper - shadow.lower(offset + 1)}},
            {{truth.velocity.x.lower - shadow.upper(offset + 2),
              truth.velocity.x.upper - shadow.lower(offset + 2)},
             {truth.velocity.y.lower - shadow.upper(offset + 3),
              truth.velocity.y.upper - shadow.lower(offset + 3)}}};
}

double maxAbs(const gf::Interval& value) {
    return std::max(std::abs(value.lower), std::abs(value.upper));
}

double minAbs(const gf::Interval& value) {
    if (value.lower <= 0.0 && value.upper >= 0.0) return 0.0;
    return std::min(std::abs(value.lower), std::abs(value.upper));
}

double canonicalHardMarginLower(
    const gf::PairIntervalAuditRequest& request,
    bool both_mobile,
    double nominal_acceleration_norm) {
    const auto audited = gf::auditPairInterval(request);
    if (!audited.valid) return -std::numeric_limits<double>::infinity();
    const gf::Interval dx{
        request.first.position.x.lower - request.second.position.x.upper,
        request.first.position.x.upper - request.second.position.x.lower};
    const gf::Interval dy{
        request.first.position.y.lower - request.second.position.y.upper,
        request.first.position.y.upper - request.second.position.y.lower};
    const gf::Interval dvx{
        request.first.velocity.x.lower - request.second.velocity.x.upper,
        request.first.velocity.x.upper - request.second.velocity.x.lower};
    const gf::Interval dvy{
        request.first.velocity.y.lower - request.second.velocity.y.upper,
        request.first.velocity.y.upper - request.second.velocity.y.lower};
    const double distance_lower = std::hypot(minAbs(dx), minAbs(dy));
    const double speed_upper = std::hypot(maxAbs(dvx), maxAbs(dvy));
    if (!(distance_lower > 0.0))
        return -std::numeric_limits<double>::infinity();
    double constant_lower = audited.minimum_h - 2.0 * speed_upper;
    if (request.spec.kind ==
        PairwiseSecondOrderBarrierKind::CommunicationUpper)
        constant_lower -= speed_upper * speed_upper / distance_lower;
    return (both_mobile ? 0.5 : 1.0) * constant_lower -
           nominal_acceleration_norm;
}

struct CanonicalAudit {
    double h = std::numeric_limits<double>::infinity();
    double psi1 = std::numeric_limits<double>::infinity();
    double hard = std::numeric_limits<double>::infinity();
};

CanonicalAudit auditCanonicalRows(
    const std::vector<gf::SecondOrderBox2D>& physical,
    const gf::ShadowStateBox& shadow,
    double applied_relative_support,
    double nominal_acceleration_norm) {
    CanonicalAudit result;
    std::vector<gf::SecondOrderBox2D> estimates;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        estimates.push_back(estimateBox(physical[mobile], shadow, mobile));
    const auto audit_pair = [&](const gf::SecondOrderBox2D& first,
                                const gf::SecondOrderBox2D& second,
                                PairwiseSecondOrderBarrierKind kind,
                                bool both_mobile) {
        const gf::PairIntervalAuditRequest request{
            first, second,
            {kind,
             kind == PairwiseSecondOrderBarrierKind::CommunicationUpper
                 ? 850.0 : 0.1,
             (kind == PairwiseSecondOrderBarrierKind::CommunicationUpper
                  ? 0.02 : 0.0) + applied_relative_support,
             1.0, 1.0, 1.0, 0.0},
            1.0e-12};
        const auto audit = gf::auditPairInterval(request);
        result.h = std::min(result.h, audit.minimum_h);
        result.psi1 = std::min(result.psi1, audit.minimum_psi1);
        result.hard = std::min(result.hard, canonicalHardMarginLower(
            request, both_mobile, nominal_acceleration_norm));
    };
    for (const auto& edge : topology()) {
        const auto& owner = estimates.at(edge.owner - 1);
        if (edge.reference <= kMobileCount)
            audit_pair(owner, estimates.at(edge.reference - 1),
                       PairwiseSecondOrderBarrierKind::CommunicationUpper, true);
        else
            audit_pair(owner, pointState(fixedPositions().at(edge.reference)),
                       PairwiseSecondOrderBarrierKind::CommunicationUpper, false);
    }
    for (std::size_t first = 0; first < kMobileCount; ++first) {
        for (std::size_t second = first + 1; second < kMobileCount; ++second)
            audit_pair(estimates[first], estimates[second],
                       PairwiseSecondOrderBarrierKind::CollisionLower, true);
        for (const auto& [id, fixed] : fixedPositions()) {
            (void)id;
            audit_pair(estimates[first], pointState(fixed),
                       PairwiseSecondOrderBarrierKind::CollisionLower, false);
        }
    }
    result.hard = std::min(result.hard, 0.4 - nominal_acceleration_norm);
    return result;
}

struct StageResult {
    std::string stage;
    bool valid = false;
    std::string first_failure = "none";
    std::size_t completed_cycles = 0;
    double duration_s = 0.0;
    double displacement_m = 0.0;
    double position_support = 0.0;
    double velocity_support = 0.0;
    double covariance_upper = 0.0;
    double minimum_h = std::numeric_limits<double>::infinity();
    double minimum_psi1 = std::numeric_limits<double>::infinity();
    double minimum_hard_margin = std::numeric_limits<double>::infinity();
    double minimum_qp_residual = std::numeric_limits<double>::infinity();
    double minimum_fim = std::numeric_limits<double>::infinity();
    double posterior_margin = std::numeric_limits<double>::infinity();
    double aoi_margin = std::numeric_limits<double>::infinity();
    std::size_t minimum_references = std::numeric_limits<std::size_t>::max();
    std::set<std::string> robust_cells;
};

double minimumFormationDistance() {
    const auto points = initialPositions();
    double result = std::numeric_limits<double>::infinity();
    for (std::size_t first = 0; first < points.size(); ++first)
        for (std::size_t second = first + 1; second < points.size(); ++second)
            result = std::min(result, (points[first] - points[second]).norm());
    for (const auto& point : points)
        for (const auto& [id, fixed] : fixedPositions()) {
            (void)id;
            result = std::min(result, (point - fixed).norm());
        }
    return result;
}

double fimLower(
    std::size_t owner,
    const std::vector<Eigen::Vector2d>& centers,
    double estimate_radius,
    const Eigen::MatrixXd& covariance) {
    std::vector<Eigen::Vector2d> directions;
    std::vector<double> angular;
    std::vector<double> weights;
    for (const auto& edge : topology()) {
        if (edge.owner != owner + 1) continue;
        Eigen::Vector2d reference;
        double reference_radius = 0.0;
        double reference_covariance = 0.0;
        if (edge.reference <= kMobileCount) {
            const std::size_t index = edge.reference - 1;
            reference = centers[index];
            reference_radius = estimate_radius;
            reference_covariance =
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    covariance.block<2, 2>(4 * index, 4 * index))
                    .eigenvalues().maxCoeff();
        } else {
            reference = fixedPositions().at(edge.reference);
        }
        const Eigen::Vector2d relative = reference - centers[owner];
        const double radius = estimate_radius + reference_radius;
        if (!(radius < relative.norm())) return 0.0;
        directions.push_back(relative.normalized());
        angular.push_back(std::asin(radius / relative.norm()));
        weights.push_back(1.0 / (kRangeVariance + reference_covariance));
    }
    if (directions.size() != 2) return 0.0;
    const double angle = std::acos(std::clamp(
        directions[0].dot(directions[1]), -1.0, 1.0));
    const double low = std::max(0.0, angle - angular[0] - angular[1]);
    const double high = std::min(
        std::acos(-1.0), angle + angular[0] + angular[1]);
    if (low <= 0.0 || high >= std::acos(-1.0)) return 0.0;
    const double sin2 = std::min(
        std::pow(std::sin(low), 2), std::pow(std::sin(high), 2));
    const double w1 = weights[0], w2 = weights[1];
    return 0.5 * (w1 + w2 - std::sqrt(
        (w1 - w2) * (w1 - w2) + 4.0 * w1 * w2 * (1.0 - sin2)));
}

std::set<std::string> sensingCells(
    Swarm& swarm,
    const std::vector<gf::SecondOrderBox2D>& physical,
    const gf::ShadowStateBox& shadow,
    double applied_shadow_support) {
    std::set<std::string> result;
    const double half_diagonal = std::sqrt(0.5);
    for (int x = 0; x < swarm.gridWorldGroundTruth.xNum; ++x) {
        for (int y = 0; y < swarm.gridWorldGroundTruth.yNum; ++y) {
            const Eigen::Vector2d cell{
                swarm.gridWorldGroundTruth.getCellCenterX(x),
                swarm.gridWorldGroundTruth.getCellCenterY(y)};
            for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
                const auto estimate = estimateBox(
                    physical[mobile], shadow, mobile);
                const double dx = std::max(
                    std::abs(cell.x() - estimate.position.x.lower),
                    std::abs(cell.x() - estimate.position.x.upper));
                const double dy = std::max(
                    std::abs(cell.y() - estimate.position.y.lower),
                    std::abs(cell.y() - estimate.position.y.upper));
                if (std::hypot(dx, dy) + applied_shadow_support + half_diagonal <=
                    kSensorRadius + 1.0e-12) {
                    result.insert(std::to_string(x) + ":" + std::to_string(y));
                    break;
                }
            }
        }
    }
    return result;
}

StageResult runStage(
    const std::string& name,
    const std::vector<Eigen::Vector2d>& commands) {
    StageResult result;
    result.stage = name;
    EnvelopeState worst = initialEnvelope();
    advanceEnvelope(worst);  // successful initialization service batch
    for (std::size_t cycle = 0; cycle < commands.size(); ++cycle)
        advanceEnvelope(worst);
    const double applied_position_support = positionSupport(worst.shadow);
    const double applied_relative_support = 2.0 * applied_position_support;

    json settings = settings14p3();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapterConfig config;
    config.solver_profile = gf::SolverProfile::OpenSource;
    config.dt_s = kDt;
    config.minimum_dwell_s = kDt;
    config.acceleration_half_box = 0.4;
    config.estimator_acceleration_variance = 0.0;
    config.uncertainty_sigma = 0.0;
    config.certified_error_bound_m = 0.0;
    config.certified_shadow_single_position_support_m =
        applied_position_support;
    config.certified_shadow_relative_position_support_m =
        applied_relative_support;
    config.maximum_accepted_range_innovation_m = kInnovation;
    config.sensor_radius_m = kSensorRadius;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, mobileIds(), fixedPositions(), topology(), config);

    EnvelopeState initialized_envelope = initialEnvelope();
    advanceEnvelope(initialized_envelope);
    std::vector<gf::SecondOrderBox2D> initialized_physical;
    for (const auto& position : initialPositions())
        initialized_physical.push_back(pointState(position));
    for (auto& state : initialized_physical)
        state = gf::propagateExactZoh(
            state, {{0.0, 0.0}, {0.0, 0.0}}, kDt,
            kProcessAcceleration + kControlError);
    const double initialized_position = positionSupport(initialized_envelope.shadow);
    const double initialized_estimate_radius =
        std::sqrt(2.0) * (0.5 * kDt * kDt *
            (kProcessAcceleration + kControlError)) + initialized_position;
    const CanonicalAudit initialized_audit = auditCanonicalRows(
        initialized_physical, initialized_envelope.shadow,
        applied_relative_support, 0.0);

    std::map<gf::NodeId, Eigen::Vector2d> zero;
    for (gf::NodeId id : mobileIds()) zero[id] = Eigen::Vector2d::Zero();
    const auto initialization = adapter.stepWithNominal(zero);
    if (!initialization.advanced) {
        result.first_failure = initialized_audit.h < 0.0
            ? "canonical_initial_set_at_stage_start"
            : "initialization_hard_control";
        result.position_support = applied_position_support;
        result.velocity_support = velocitySupport(worst.shadow);
        result.covariance_upper = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(
            worst.covariance).eigenvalues().maxCoeff();
        result.minimum_h = initialized_audit.h;
        result.minimum_psi1 = initialized_audit.psi1;
        result.minimum_hard_margin = initialized_audit.hard;
        result.posterior_margin = 0.1 -
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(
                initialized_envelope.covariance).eigenvalues().maxCoeff();
        for (std::size_t owner = 0; owner < kMobileCount; ++owner)
            result.minimum_fim = std::min(result.minimum_fim,
                fimLower(owner, initialPositions(), initialized_estimate_radius,
                         initialized_envelope.covariance));
        result.robust_cells = sensingCells(
            swarm, initialized_physical, initialized_envelope.shadow,
            applied_position_support);
        return result;
    }

    EnvelopeState envelope = initialEnvelope();
    advanceEnvelope(envelope);
    std::vector<gf::SecondOrderBox2D> physical = initialized_physical;
    std::vector<Eigen::Vector2d> centers = initialPositions();
    Eigen::Vector2d common_velocity = Eigen::Vector2d::Zero();

    for (const Eigen::Vector2d& command : commands) {
        const CanonicalAudit preapply = auditCanonicalRows(
            physical, envelope.shadow, applied_relative_support,
            command.norm());
        result.minimum_h = std::min(result.minimum_h, preapply.h);
        result.minimum_psi1 = std::min(result.minimum_psi1, preapply.psi1);
        result.minimum_hard_margin = std::min(
            result.minimum_hard_margin, preapply.hard);
        if (preapply.h < 0.0) {
            result.first_failure = "hocbf_h_preapply";
            break;
        }
        if (preapply.psi1 < 0.0) {
            result.first_failure = "hocbf_psi1_preapply";
            break;
        }
        if (preapply.hard <= kControlError) {
            result.first_failure = "hard_control_preapply";
            break;
        }
        std::map<gf::NodeId, Eigen::Vector2d> nominal;
        for (gf::NodeId id : mobileIds()) nominal[id] = command;
        const auto step = adapter.stepWithNominal(nominal);
        if (!step.advanced) {
            result.first_failure = "actual_qp";
            break;
        }
        result.minimum_qp_residual = std::min(
            result.minimum_qp_residual, step.minimum_hard_residual);
        for (const auto& [id, control] : step.applied_controls) {
            if ((control - nominal.at(id)).cwiseAbs().maxCoeff() >
                kControlError + 1.0e-12) {
                result.first_failure = "nominal_solution_map";
                break;
            }
        }
        if (result.first_failure != "none") break;
        for (auto& center : centers)
            center += common_velocity * kDt +
                      0.5 * command * kDt * kDt;
        common_velocity += command * kDt;
        for (auto& state : physical)
            state = gf::propagateExactZoh(
                state,
                {{command.x(), command.x()}, {command.y(), command.y()}},
                kDt, kProcessAcceleration + kControlError);
        advanceEnvelope(envelope);

        const CanonicalAudit canonical = auditCanonicalRows(
            physical, envelope.shadow, applied_relative_support,
            command.norm());
        result.minimum_h = std::min(result.minimum_h, canonical.h);
        result.minimum_psi1 = std::min(result.minimum_psi1, canonical.psi1);
        result.minimum_hard_margin = std::min(
            result.minimum_hard_margin, canonical.hard);
        double estimate_position_radius = 0.0;
        for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
            const auto estimate = estimateBox(physical[mobile], envelope.shadow, mobile);
            estimate_position_radius = std::max(estimate_position_radius,
                std::hypot(
                    0.5 * (estimate.position.x.upper - estimate.position.x.lower),
                    0.5 * (estimate.position.y.upper - estimate.position.y.lower)));
        }
        for (std::size_t owner = 0; owner < kMobileCount; ++owner)
            result.minimum_fim = std::min(result.minimum_fim,
                fimLower(owner, centers, estimate_position_radius,
                         envelope.covariance));
        const double posterior = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(
            envelope.covariance).eigenvalues().maxCoeff();
        result.posterior_margin = std::min(
            result.posterior_margin, 0.1 - posterior);
        const auto references = adapter.currentReferenceAudit();
        result.minimum_references = std::min(
            result.minimum_references,
            references.minimum_effective_reference_count);
        result.aoi_margin = std::min(result.aoi_margin,
            0.6 - (4.0 + 1.0) * kDt);
        const auto cells = sensingCells(
            swarm, physical, envelope.shadow, applied_position_support);
        result.robust_cells.insert(cells.begin(), cells.end());
        ++result.completed_cycles;

        if (result.minimum_h < 0.0) result.first_failure = "hocbf_h";
        else if (result.minimum_psi1 < 0.0) result.first_failure = "hocbf_psi1";
        else if (result.minimum_hard_margin <= kControlError)
            result.first_failure = "hard_control";
        else if (result.minimum_qp_residual < -1.0e-7)
            result.first_failure = "actual_qp_residual";
        else if (result.minimum_references < 2)
            result.first_failure = "effective_references";
        else if (result.minimum_fim < 1.0e-6)
            result.first_failure = "fim";
        else if (result.posterior_margin <= 0.0)
            result.first_failure = "posterior";
        else if (result.aoi_margin < -1.0e-12)
            result.first_failure = "aoi";
        if (result.first_failure != "none") break;
    }
    if (result.first_failure == "none" && result.robust_cells.empty())
        result.first_failure = "sensing";
    result.valid = result.first_failure == "none" &&
                   result.completed_cycles == commands.size();
    result.duration_s = result.completed_cycles * kDt;
    result.displacement_m = (centers.front() - initialPositions().front()).norm();
    result.position_support = positionSupport(envelope.shadow);
    result.velocity_support = velocitySupport(envelope.shadow);
    result.covariance_upper = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(
        envelope.covariance).eigenvalues().maxCoeff();
    return result;
}

void emit(const StageResult& result) {
    json metric{{"stage", result.stage}, {"valid", result.valid},
        {"first_exhausted_margin", result.first_failure},
        {"completed_cycles", result.completed_cycles},
        {"maximum_certified_horizon_s", result.duration_s},
        {"displacement_m", result.displacement_m},
        {"position_support_m", result.position_support},
        {"velocity_support_mps", result.velocity_support},
        {"covariance_upper_eigenvalue", result.covariance_upper},
        {"minimum_h", result.minimum_h},
        {"minimum_psi1", result.minimum_psi1},
        {"minimum_hard_margin", result.minimum_hard_margin},
        {"minimum_actual_qp_residual", result.minimum_qp_residual},
        {"minimum_fim", result.minimum_fim},
        {"posterior_margin", result.posterior_margin},
        {"aoi_margin_s", result.aoi_margin},
        {"minimum_effective_references", result.minimum_references},
        {"robust_sensing_cells", result.robust_cells.size()}};
    std::cout << "TASK10P7D_METRIC " << metric.dump() << '\n';
}

}  // namespace

TEST_CASE("Task 10.7d counts every 14+3 sequential range slot") {
    REQUIRE(gf::completeRangeSlotCount(14, 3) == 133);
    const auto links = rangeSlotIds();
    REQUIRE(links.size() == kSlots);
    REQUIRE(std::set<std::string>(links.begin(), links.end()).size() == kSlots);
    REQUIRE_THROWS_AS(gf::completeRangeSlotCount(0, 3), std::invalid_argument);
}

TEST_CASE("Task 10.7d physical audit uses exact ZOH centers") {
    const StageResult one_accelerating_cycle = runStage(
        "exact_zoh_probe", {{0.3, 0.0}});
    REQUIRE(one_accelerating_cycle.displacement_m == doctest::Approx(
        0.5 * 0.3 * kDt * kDt));
}

TEST_CASE("Task 10.7d audits the first nominal interval before apply") {
    const StageResult boundary_command = runStage(
        "preapply_probe", {{0.4, 0.0}});
    REQUIRE_FALSE(boundary_command.valid);
    REQUIRE(boundary_command.first_failure == "hard_control_preapply");
    REQUIRE(boundary_command.completed_cycles == 0);
}

TEST_CASE("Task 10.7d stops at the first failed deterministic envelope tier") {
    std::vector<Eigen::Vector2d> static_commands{Eigen::Vector2d::Zero()};
    const StageResult static_stage = runStage("static_one_cycle", static_commands);
    emit(static_stage);
    REQUIRE(static_stage.valid);

    const auto commands = [](const gf::TranslationPrimitive& primitive) {
        std::vector<Eigen::Vector2d> result;
        for (const auto& phase : primitive.phases)
            result.push_back({
                0.5 * (phase.acceleration.x.lower + phase.acceleration.x.upper),
                0.5 * (phase.acceleration.y.lower + phase.acceleration.y.upper)});
        return result;
    };
    const auto certified_translation = gf::makeRestToRestTranslation(
        {1.0, 0.0}, 0.3, kDt, 20);
    const auto first_failed_translation = gf::makeRestToRestTranslation(
        {1.0, 0.0}, 0.3, kDt, 21);
    REQUIRE(certified_translation.has_value());
    REQUIRE(first_failed_translation.has_value());
    const StageResult forward_stage = runStage(
        "forward_max_certified", commands(*certified_translation));
    emit(forward_stage);
    REQUIRE(forward_stage.valid);

    const auto reverse = gf::reverseTranslation(*certified_translation);
    auto roundtrip_commands = commands(*certified_translation);
    const auto reverse_commands = commands(reverse);
    roundtrip_commands.insert(
        roundtrip_commands.end(), reverse_commands.begin(), reverse_commands.end());
    const StageResult roundtrip = runStage(
        "forward_return", roundtrip_commands);
    emit(roundtrip);
    REQUIRE_FALSE(roundtrip.valid);
    REQUIRE(roundtrip.first_failure != "none");

    // Post-failure diagnostic only: this prefix is shorter than the failed
    // round trip and brackets the maximum completed rest-to-rest witness.
    const StageResult boundary_failure = runStage(
        "forward_first_failed_neighbor", commands(*first_failed_translation));
    emit(boundary_failure);
    REQUIRE_FALSE(boundary_failure.valid);
    REQUIRE(boundary_failure.first_failure == "sensing");
}
