#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedTranslationPrimitive.hpp"
#include "grand_finale/FiniteHorizonWitnessVerifier.hpp"
#include "grand_finale/FiniteTourShadowEnvelope.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/TimeExpandedFiniteTourCertificate.hpp"

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

constexpr std::size_t kMobileCount = 4;
constexpr std::size_t kRangeSlotsPerCycle = 14;
constexpr double kDt = 0.1;
constexpr double kAcceleration = 0.3;
constexpr std::size_t kHalfSteps = 23;
constexpr double kInnovationBoundM = 0.05;
constexpr double kMeasurementVarianceLowerM2 = 1.0;
constexpr double kProcessAccelerationBound = 1.0e-4;
constexpr double kSolverControlError = 1.0e-5;
constexpr double kSensorRadiusM = 2.0;

json settings4p2() {
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
    settings["num"] = 4;
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0, 1}};
    settings["bases"] = {{4.0, 4.0}, {4.0, 16.0}};
    settings["initial"]["position"]["positions"] = {
        {7.0, 7.0}, {7.0, 13.0}, {12.0, 7.0}, {12.0, 13.0}};
    settings["initial"]["velocity"]["values"] = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    settings["world"]["spacing"] = 1.0;
    settings["searching"]["downward"]["radius"] = kSensorRadiusM;
    settings["optimiser"] = "OSQP";
    return settings;
}

std::vector<gf::DirectedEdge> topology() {
    return {{10, 1}, {11, 1}, {10, 2}, {1, 2},
            {11, 3}, {1, 3}, {2, 4}, {3, 4}};
}

Eigen::MatrixXd jointTransition() {
    Eigen::MatrixXd transition = Eigen::MatrixXd::Identity(16, 16);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        transition(4 * mobile, 4 * mobile + 2) = kDt;
        transition(4 * mobile + 1, 4 * mobile + 3) = kDt;
    }
    return transition;
}

Eigen::MatrixXd jointDisturbanceInput() {
    Eigen::MatrixXd input = Eigen::MatrixXd::Zero(16, 8);
    Eigen::Matrix<double, 4, 2> block;
    block << 0.5 * kDt * kDt, 0.0,
             0.0, 0.5 * kDt * kDt,
             kDt, 0.0,
             0.0, kDt;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        input.block<4, 2>(4 * mobile, 2 * mobile) = block;
    return input;
}

struct ShadowCertificateState {
    gf::ShadowStateBox shadow;
    Eigen::MatrixXd covariance_upper;
};

ShadowCertificateState initialShadowCertificate() {
    Eigen::VectorXd lower(16);
    Eigen::VectorXd upper(16);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        lower.segment<4>(4 * mobile) << -0.01, -0.01, -0.002, -0.002;
        upper.segment<4>(4 * mobile) << 0.01, 0.01, 0.002, 0.002;
    }
    return {gf::ShadowStateBox{lower, upper},
            1.0e-4 * Eigen::MatrixXd::Identity(16, 16)};
}

std::vector<std::string> rangeSlotIds() {
    std::vector<std::string> result;
    for (gf::NodeId first = 1; first <= kMobileCount; ++first) {
        for (gf::NodeId second = first + 1;
             second <= kMobileCount; ++second) {
            result.push_back(gf::UndirectedEdge::canonical(first, second).id());
        }
        result.push_back(gf::UndirectedEdge::canonical(first, 10).id());
        result.push_back(gf::UndirectedEdge::canonical(first, 11).id());
    }
    std::sort(result.begin(), result.end());
    return result;
}

gf::FiniteTourShadowCycle makeShadowCycle(
    ShadowCertificateState& state) {
    const Eigen::MatrixXd transition = jointTransition();
    const Eigen::MatrixXd disturbance_input = jointDisturbanceInput();
    const Eigen::VectorXd disturbance =
        Eigen::VectorXd::Constant(8, kProcessAccelerationBound);
    state.shadow = gf::propagateShadowPrediction(
        state.shadow, transition, disturbance_input, disturbance);
    state.covariance_upper = transition * state.covariance_upper *
        transition.transpose();
    const Eigen::VectorXd gain =
        gf::branchIndependentComponentGainBounds(
            state.covariance_upper, std::sqrt(2.0),
            kMeasurementVarianceLowerM2);
    std::vector<gf::ScalarUpdateBound> updates;
    std::vector<gf::TaggedScalarUpdateBound> tagged;
    for (const std::string& link : rangeSlotIds()) {
        updates.push_back({gain, kInnovationBoundM});
        tagged.push_back({link, {gain, kInnovationBoundM}});
    }
    state.shadow = gf::accumulateScalarUpdateBatch(state.shadow, updates);
    return {transition, disturbance_input, disturbance, tagged};
}

void advanceShadowCertificate(ShadowCertificateState& state) {
    (void)makeShadowCycle(state);
}

std::pair<std::vector<gf::FiniteTourShadowCycle>, ShadowCertificateState>
buildShadowCycles(
    ShadowCertificateState source,
    std::size_t cycle_count) {
    std::vector<gf::FiniteTourShadowCycle> cycles;
    cycles.reserve(cycle_count);
    for (std::size_t index = 0; index < cycle_count; ++index)
        cycles.push_back(makeShadowCycle(source));
    return {cycles, source};
}

double maximumSinglePositionSupport(const gf::ShadowStateBox& shadow) {
    double result = 0.0;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        const Eigen::Index offset = static_cast<Eigen::Index>(4 * mobile);
        const double x = std::max(
            std::abs(shadow.lower(offset)), std::abs(shadow.upper(offset)));
        const double y = std::max(
            std::abs(shadow.lower(offset + 1)),
            std::abs(shadow.upper(offset + 1)));
        result = std::max(result, std::hypot(x, y));
    }
    return result;
}

gf::SecondOrderBox2D pointBox(const Point& position) {
    return {{{position.x, position.x}, {position.y, position.y}},
            {{0.0, 0.0}, {0.0, 0.0}}};
}

std::vector<gf::SecondOrderBox2D> initialPhysicalBoxes(const Swarm& swarm) {
    std::vector<gf::SecondOrderBox2D> result;
    for (const auto& robot : swarm.robots) {
        const Point position = robot->model->xy();
        const Eigen::VectorXd velocity = robot->model->getVelocity();
        result.push_back({
            {{position.x, position.x}, {position.y, position.y}},
            {{velocity(0), velocity(0)}, {velocity(1), velocity(1)}}});
    }
    return result;
}

gf::SecondOrderBox2D estimateBox(
    const gf::SecondOrderBox2D& truth,
    const gf::ShadowStateBox& shadow,
    std::size_t mobile) {
    const Eigen::Index offset = static_cast<Eigen::Index>(4 * mobile);
    return {
        {{truth.position.x.lower - shadow.upper(offset),
          truth.position.x.upper - shadow.lower(offset)},
         {truth.position.y.lower - shadow.upper(offset + 1),
          truth.position.y.upper - shadow.lower(offset + 1)}},
        {{truth.velocity.x.lower - shadow.upper(offset + 2),
          truth.velocity.x.upper - shadow.lower(offset + 2)},
         {truth.velocity.y.lower - shadow.upper(offset + 3),
          truth.velocity.y.upper - shadow.lower(offset + 3)}}};
}

double maxAbs(const gf::Interval& interval) {
    return std::max(std::abs(interval.lower), std::abs(interval.upper));
}

double minAbs(const gf::Interval& interval) {
    if (interval.lower <= 0.0 && interval.upper >= 0.0) return 0.0;
    return std::min(std::abs(interval.lower), std::abs(interval.upper));
}

double localHardMarginLower(
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
        PairwiseSecondOrderBarrierKind::CommunicationUpper) {
        constant_lower -= speed_upper * speed_upper / distance_lower;
    }
    const double responsibility = both_mobile ? 0.5 : 1.0;
    return responsibility * constant_lower - nominal_acceleration_norm;
}

struct AnalyticStageAudit {
    double minimum_h = std::numeric_limits<double>::infinity();
    double minimum_psi1 = std::numeric_limits<double>::infinity();
    double minimum_hard_margin = std::numeric_limits<double>::infinity();
    double minimum_fim = std::numeric_limits<double>::infinity();
    double posterior_margin = std::numeric_limits<double>::infinity();
    std::size_t minimum_effective_references = 2;
};

double directionAngularRadius(
    const Eigen::Vector2d& nominal_relative,
    double relative_radius) {
    const double distance = nominal_relative.norm();
    if (!(relative_radius < distance))
        return std::numeric_limits<double>::infinity();
    return std::asin(std::min(1.0, relative_radius / distance));
}

double positionBoxRadius(const gf::SecondOrderBox2D& box) {
    return std::hypot(
        0.5 * (box.position.x.upper - box.position.x.lower),
        0.5 * (box.position.y.upper - box.position.y.lower));
}

Eigen::Vector2d positionBoxCenter(const gf::SecondOrderBox2D& box) {
    return {0.5 * (box.position.x.lower + box.position.x.upper),
            0.5 * (box.position.y.lower + box.position.y.upper)};
}

double twoReferenceFimLower(
    const std::vector<gf::SecondOrderBox2D>& estimates,
    const Eigen::MatrixXd& covariance_upper,
    std::size_t owner,
    const std::vector<gf::DirectedEdge>& edges,
    const std::map<gf::NodeId, Eigen::Vector2d>& fixed) {
    std::vector<Eigen::Vector2d> centers;
    std::vector<double> angular_radii;
    std::vector<double> weight_lowers;
    const Eigen::Vector2d owner_center = positionBoxCenter(estimates[owner]);
    const double owner_radius = positionBoxRadius(estimates[owner]);
    for (const auto& edge : edges) {
        if (edge.owner != owner + 1) continue;
        Eigen::Vector2d reference_center;
        double reference_radius = 0.0;
        double reference_covariance_upper = 0.0;
        if (edge.reference <= kMobileCount) {
            const std::size_t reference = edge.reference - 1;
            reference_center = positionBoxCenter(estimates[reference]);
            reference_radius = positionBoxRadius(estimates[reference]);
            reference_covariance_upper =
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                    covariance_upper.block<2, 2>(4 * reference, 4 * reference))
                    .eigenvalues().maxCoeff();
        } else {
            reference_center = fixed.at(edge.reference);
        }
        const Eigen::Vector2d relative = reference_center - owner_center;
        const double angle_radius = directionAngularRadius(
            relative, owner_radius + reference_radius);
        if (!std::isfinite(angle_radius)) return 0.0;
        centers.push_back(relative.normalized());
        angular_radii.push_back(angle_radius);
        weight_lowers.push_back(
            1.0 / (kMeasurementVarianceLowerM2 +
                   reference_covariance_upper));
    }
    if (centers.size() != 2) return 0.0;
    const double center_angle = std::acos(std::clamp(
        centers[0].dot(centers[1]), -1.0, 1.0));
    const double low = std::max(
        0.0, center_angle - angular_radii[0] - angular_radii[1]);
    const double high = std::min(
        std::acos(-1.0),
        center_angle + angular_radii[0] + angular_radii[1]);
    const double sin2_lower = std::min(
        std::pow(std::sin(low), 2), std::pow(std::sin(high), 2));
    if (low <= 0.0 || high >= std::acos(-1.0)) return 0.0;
    const double w1 = weight_lowers[0];
    const double w2 = weight_lowers[1];
    const double cos2_upper = 1.0 - sin2_lower;
    return 0.5 * (w1 + w2 - std::sqrt(
        (w1 - w2) * (w1 - w2) + 4.0 * w1 * w2 * cos2_upper));
}

AnalyticStageAudit auditStage(
    const std::vector<gf::SecondOrderBox2D>& physical,
    const ShadowCertificateState& certificate,
    double relative_support,
    double nominal_acceleration_norm,
    const std::map<gf::NodeId, Eigen::Vector2d>& fixed) {
    AnalyticStageAudit result;
    std::vector<gf::SecondOrderBox2D> estimates;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        estimates.push_back(estimateBox(
            physical[mobile], certificate.shadow, mobile));
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
                  ? 0.02 : 0.0) + relative_support,
             1.0, 1.0, 1.0, 0.0},
            1.0e-12};
        const auto audit = gf::auditPairInterval(request);
        if (!audit.valid) {
            result.minimum_h = -std::numeric_limits<double>::infinity();
            result.minimum_psi1 = -std::numeric_limits<double>::infinity();
            result.minimum_hard_margin =
                -std::numeric_limits<double>::infinity();
            return;
        }
        result.minimum_h = std::min(result.minimum_h, audit.minimum_h);
        result.minimum_psi1 = std::min(
            result.minimum_psi1, audit.minimum_psi1);
        result.minimum_hard_margin = std::min(
            result.minimum_hard_margin,
            localHardMarginLower(
                request, both_mobile, nominal_acceleration_norm));
    };
    for (const auto& edge : topology()) {
        const auto& owner = estimates.at(edge.owner - 1);
        if (edge.reference <= kMobileCount) {
            audit_pair(owner, estimates.at(edge.reference - 1),
                       PairwiseSecondOrderBarrierKind::CommunicationUpper,
                       true);
        } else {
            audit_pair(owner, pointBox(Point(
                           fixed.at(edge.reference).x(),
                           fixed.at(edge.reference).y())),
                       PairwiseSecondOrderBarrierKind::CommunicationUpper,
                       false);
        }
    }
    for (std::size_t first = 0; first < kMobileCount; ++first) {
        for (std::size_t second = first + 1;
             second < kMobileCount; ++second) {
            audit_pair(estimates[first], estimates[second],
                       PairwiseSecondOrderBarrierKind::CollisionLower, true);
        }
        for (const auto& [id, position] : fixed) {
            (void)id;
            audit_pair(estimates[first], pointBox(Point(
                           position.x(), position.y())),
                       PairwiseSecondOrderBarrierKind::CollisionLower, false);
        }
    }
    result.minimum_hard_margin = std::min(
        result.minimum_hard_margin, 0.4 - nominal_acceleration_norm);
    for (std::size_t owner = 0; owner < kMobileCount; ++owner) {
        result.minimum_fim = std::min(
            result.minimum_fim,
            twoReferenceFimLower(
                estimates, certificate.covariance_upper,
                owner, topology(), fixed));
        const double posterior =
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                certificate.covariance_upper.block<2, 2>(
                    4 * owner, 4 * owner))
                .eigenvalues().maxCoeff();
        result.posterior_margin = std::min(
            result.posterior_margin, 0.1 - posterior);
    }
    return result;
}

std::set<std::string> robustSensingCells(
    Swarm& swarm,
    const std::vector<gf::SecondOrderBox2D>& physical,
    const gf::ShadowStateBox& shadow,
    double applied_error_bound) {
    std::set<std::string> cells;
    const double cell_width =
        (swarm.gridWorldGroundTruth.xLim.second -
         swarm.gridWorldGroundTruth.xLim.first) /
        swarm.gridWorldGroundTruth.xNum;
    const double cell_height =
        (swarm.gridWorldGroundTruth.yLim.second -
         swarm.gridWorldGroundTruth.yLim.first) /
        swarm.gridWorldGroundTruth.yNum;
    const double half_diagonal = 0.5 * std::hypot(
        cell_width, cell_height);
    for (int x = 0; x < swarm.gridWorldGroundTruth.xNum; ++x) {
        for (int y = 0; y < swarm.gridWorldGroundTruth.yNum; ++y) {
            const double cx = swarm.gridWorldGroundTruth.getCellCenterX(x);
            const double cy = swarm.gridWorldGroundTruth.getCellCenterY(y);
            for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
                const auto estimate = estimateBox(
                    physical[mobile], shadow, mobile);
                const double dx = std::max(
                    std::abs(cx - estimate.position.x.lower),
                    std::abs(cx - estimate.position.x.upper));
                const double dy = std::max(
                    std::abs(cy - estimate.position.y.lower),
                    std::abs(cy - estimate.position.y.upper));
                if (std::hypot(dx, dy) + applied_error_bound + half_diagonal <=
                    kSensorRadiusM + 1.0e-12) {
                    cells.insert(std::to_string(x) + ":" + std::to_string(y));
                    break;
                }
            }
        }
    }
    return cells;
}

std::vector<gf::FiniteEdgeBlackoutContract> blackoutContracts(
    const std::vector<gf::DirectedEdge>& edges) {
    std::vector<gf::FiniteEdgeBlackoutContract> result;
    for (const auto& edge : edges) {
        result.push_back({
            gf::UndirectedEdge::canonical(edge.reference, edge.owner).id(),
            kDt, 4, 0.0, 0.6,
            kMeasurementVarianceLowerM2,
            kMeasurementVarianceLowerM2, 1.0});
    }
    return result;
}

bool boxContainsActualError(
    const gf::ShadowStateBox& shadow,
    const Swarm& swarm,
    const gf::JointEstimateSnapshot& estimate) {
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        const Point truth_position = swarm.robots[mobile]->model->xy();
        const Eigen::VectorXd truth_velocity =
            swarm.robots[mobile]->model->getVelocity();
        Eigen::Vector4d error;
        error << truth_position.x - estimate.mean(4 * mobile),
                 truth_position.y - estimate.mean(4 * mobile + 1),
                 truth_velocity(0) - estimate.mean(4 * mobile + 2),
                 truth_velocity(1) - estimate.mean(4 * mobile + 3);
        for (Eigen::Index axis = 0; axis < 4; ++axis) {
            const Eigen::Index index = 4 * mobile + axis;
            if (error(axis) < shadow.lower(index) - 1.0e-10 ||
                error(axis) > shadow.upper(index) + 1.0e-10)
                return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("Task 10.7c certifies one finite 4+2 forward-return tour for every admissible update word") {
    const auto forward = gf::makeRestToRestTranslation(
        {1.0, 0.0}, kAcceleration, kDt, kHalfSteps);
    REQUIRE(forward.has_value());
    const auto reverse = gf::reverseTranslation(*forward);
    REQUIRE(forward->phases.size() == 2 * kHalfSteps);
    REQUIRE(reverse.phases.size() == forward->phases.size());

    ShadowCertificateState worst = initialShadowCertificate();
    std::vector<gf::ShadowStateBox> stage_shadows;
    advanceShadowCertificate(worst);  // initialization service cycle
    stage_shadows.push_back(worst.shadow);
    for (std::size_t step = 0; step < forward->phases.size(); ++step)
        advanceShadowCertificate(worst);
    stage_shadows.push_back(worst.shadow);
    for (std::size_t step = 0; step < reverse.phases.size(); ++step)
        advanceShadowCertificate(worst);
    stage_shadows.push_back(worst.shadow);
    const double single_support = maximumSinglePositionSupport(worst.shadow);
    const double relative_support = 2.0 * single_support;
    const double final_covariance_upper =
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(
            worst.covariance_upper).eigenvalues().maxCoeff();

    json settings = settings4p2();
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapterConfig config;
    config.solver_profile = gf::SolverProfile::OpenSource;
    config.dt_s = kDt;
    config.minimum_dwell_s = kDt;
    config.acceleration_half_box = 0.4;
    config.estimator_acceleration_variance = 0.0;
    config.uncertainty_sigma = 0.0;
    config.certified_error_bound_m = 0.0;
    config.certified_shadow_single_position_support_m = single_support;
    config.certified_shadow_relative_position_support_m = relative_support;
    config.maximum_accepted_range_innovation_m = kInnovationBoundM;
    config.sensor_radius_m = kSensorRadiusM;
    const std::map<gf::NodeId, Eigen::Vector2d> fixed{
        {10, {4.0, 4.0}}, {11, {4.0, 16.0}}};
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, fixed, topology(), config);

    std::vector<gf::SecondOrderBox2D> physical = initialPhysicalBoxes(swarm);
    ShadowCertificateState certificate = initialShadowCertificate();
    const std::map<gf::NodeId, Eigen::Vector2d> zero_nominal{
        {1, Eigen::Vector2d::Zero()}, {2, Eigen::Vector2d::Zero()},
        {3, Eigen::Vector2d::Zero()}, {4, Eigen::Vector2d::Zero()}};
    const auto initialization = adapter.stepWithNominal(zero_nominal);
    REQUIRE(initialization.advanced);
    for (auto& state : physical) {
        state = gf::propagateExactZoh(
            state, {{0.0, 0.0}, {0.0, 0.0}}, kDt,
            kSolverControlError + kProcessAccelerationBound);
    }
    advanceShadowCertificate(certificate);
    const ShadowCertificateState stage_zero_certificate = certificate;
    const std::set<std::string> initial_cells = robustSensingCells(
        swarm, physical, certificate.shadow, single_support);

    std::vector<AnalyticStageAudit> edge_audits(2);
    std::vector<std::set<std::string>> edge_cells(2);
    double minimum_actual_hard_residual =
        std::numeric_limits<double>::infinity();
    double maximum_control_error = 0.0;
    std::size_t minimum_actual_references =
        std::numeric_limits<std::size_t>::max();
    const auto run_edge = [&](const gf::TranslationPrimitive& primitive,
                              std::size_t edge_index) {
        for (const auto& phase : primitive.phases) {
            const Eigen::Vector2d command{
                0.5 * (phase.acceleration.x.lower +
                       phase.acceleration.x.upper),
                0.5 * (phase.acceleration.y.lower +
                       phase.acceleration.y.upper)};
            const std::map<gf::NodeId, Eigen::Vector2d> nominal{
                {1, command}, {2, command}, {3, command}, {4, command}};
            const auto step = adapter.stepWithNominal(nominal);
            REQUIRE(step.advanced);
            REQUIRE(step.certified_control_count == kMobileCount);
            minimum_actual_hard_residual = std::min(
                minimum_actual_hard_residual, step.minimum_hard_residual);
            for (const auto& [id, control] : step.applied_controls) {
                maximum_control_error = std::max(
                    maximum_control_error,
                    (control - nominal.at(id)).cwiseAbs().maxCoeff());
            }
            for (auto& state : physical) {
                state = gf::propagateExactZoh(
                    state, phase.acceleration, kDt,
                    kSolverControlError + kProcessAccelerationBound);
            }
            advanceShadowCertificate(certificate);
            const auto audit = auditStage(
                physical, certificate, relative_support,
                command.norm(), fixed);
            edge_audits[edge_index].minimum_h = std::min(
                edge_audits[edge_index].minimum_h, audit.minimum_h);
            edge_audits[edge_index].minimum_psi1 = std::min(
                edge_audits[edge_index].minimum_psi1,
                audit.minimum_psi1);
            edge_audits[edge_index].minimum_hard_margin = std::min(
                edge_audits[edge_index].minimum_hard_margin,
                audit.minimum_hard_margin);
            edge_audits[edge_index].minimum_fim = std::min(
                edge_audits[edge_index].minimum_fim,
                audit.minimum_fim);
            edge_audits[edge_index].posterior_margin = std::min(
                edge_audits[edge_index].posterior_margin,
                audit.posterior_margin);
            const auto references = adapter.currentReferenceAudit();
            minimum_actual_references = std::min(
                minimum_actual_references,
                references.minimum_effective_reference_count);
            const auto cells = robustSensingCells(
                swarm, physical, certificate.shadow, single_support);
            edge_cells[edge_index].insert(cells.begin(), cells.end());
        }
    };
    run_edge(*forward, 0);
    const ShadowCertificateState forward_terminal = certificate;
    run_edge(reverse, 1);

    std::set<std::string> tour_cells = edge_cells[0];
    tour_cells.insert(edge_cells[1].begin(), edge_cells[1].end());
    std::set<std::string> new_cells;
    std::set_difference(
        tour_cells.begin(), tour_cells.end(),
        initial_cells.begin(), initial_cells.end(),
        std::inserter(new_cells, new_cells.end()));

    REQUIRE(maximum_control_error <= kSolverControlError);
    REQUIRE(minimum_actual_hard_residual >= -1.0e-7);
    REQUIRE(minimum_actual_references >= 2);
    REQUIRE(edge_audits[0].minimum_h >= 0.0);
    REQUIRE(edge_audits[1].minimum_h >= 0.0);
    REQUIRE(edge_audits[0].minimum_psi1 >= 0.0);
    REQUIRE(edge_audits[1].minimum_psi1 >= 0.0);
    REQUIRE(edge_audits[0].minimum_hard_margin > kSolverControlError);
    REQUIRE(edge_audits[1].minimum_hard_margin > kSolverControlError);
    REQUIRE(edge_audits[0].minimum_fim >= 1.0e-6);
    REQUIRE(edge_audits[1].minimum_fim >= 1.0e-6);
    REQUIRE(edge_audits[0].posterior_margin > 0.0);
    REQUIRE(edge_audits[1].posterior_margin > 0.0);
    REQUIRE_FALSE(new_cells.empty());

    const gf::GrandFinaleRuntimeSnapshot terminal = adapter.runtimeSnapshot();
    REQUIRE(boxContainsActualError(
        certificate.shadow, swarm, terminal.estimate));
    REQUIRE(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(
        certificate.covariance_upper - terminal.estimate.covariance)
        .eigenvalues().minCoeff() >= -1.0e-9);
    REQUIRE(terminal.topology == topology());
    REQUIRE(terminal.mode == gf::SupervisorMode::Search);
    REQUIRE(terminal.freshness == gf::FreshnessRelation::NoPending);
    REQUIRE_FALSE(terminal.supervisor_transition_pending);
    REQUIRE_FALSE(terminal.adapter_transition_pending);
    REQUIRE_FALSE(terminal.pending_is_retreat);
    REQUIRE(terminal.transition_stack_size == 0);
    REQUIRE(terminal.union_control_cycles == 0);
    REQUIRE(std::isfinite(terminal.runtime_s));
    REQUIRE(std::isfinite(terminal.timer_since_supervisor_transition_s));
    REQUIRE(terminal.dekf.has_last_measurement);
    REQUIRE(terminal.dekf.propagation_factors.size() == kMobileCount);
    REQUIRE(terminal.dekf.correlation_rows.size() == kMobileCount);
    for (const auto& factor : terminal.dekf.propagation_factors)
        REQUIRE(factor.allFinite());
    for (const auto& row : terminal.dekf.correlation_rows) {
        REQUIRE(row.size() == kMobileCount);
        for (const auto& factor : row) REQUIRE(factor.allFinite());
    }
    REQUIRE(terminal.range_links.size() == kRangeSlotsPerCycle);
    for (const auto& [id, link] : terminal.range_links) {
        (void)id;
        REQUIRE(link.age_s <= 0.6 + 1.0e-12);
        REQUIRE(link.quality >= 1.0 - 1.0e-12);
        REQUIRE(link.variance_m2 >= kMeasurementVarianceLowerM2);
    }

    const std::size_t updates_per_edge =
        forward->phases.size() * kRangeSlotsPerCycle;
    const auto edge_result = [&](std::size_t index, double duration) {
        return gf::FiniteHorizonWitnessResult{
            true, "accepted", duration,
            std::min(edge_audits[index].minimum_h,
                     edge_audits[index].minimum_psi1),
            2};
    };
    gf::FiniteTourRequest tour{
        {{0, stage_shadows[0]},
         {1, stage_shadows[1]},
         {2, stage_shadows[2]}},
        {{0, 1, edge_result(0, forward->duration_s),
          forward_terminal.shadow, updates_per_edge, updates_per_edge,
          {true, true, true, true, true, true}, edge_cells[0]},
         {1, 2, edge_result(1, reverse.duration_s),
          certificate.shadow, updates_per_edge, updates_per_edge,
          {true, true, true, true, true, true}, edge_cells[1]}},
        tour_cells,
        forward->duration_s + reverse.duration_s};
    const auto forward_shadow_cycles = buildShadowCycles(
        stage_zero_certificate, forward->phases.size());
    const auto reverse_shadow_cycles = buildShadowCycles(
        forward_shadow_cycles.second, reverse.phases.size());
    tour.edges[0].shadow_cycles = forward_shadow_cycles.first;
    tour.edges[1].shadow_cycles = reverse_shadow_cycles.first;
    for (auto& edge : tour.edges)
        edge.blackout_contracts = blackoutContracts(topology());
    const gf::FiniteTourResult tour_result =
        gf::verifyTimeExpandedFiniteTour(tour);
    INFO(tour_result.reason);
    REQUIRE(tour_result.valid);
    REQUIRE(tour_result.certified_cells == tour_cells);
    REQUIRE(tour_result.duration_upper_s == doctest::Approx(
        forward->duration_s + reverse.duration_s));

    const double initialized_stage_single_support =
        maximumSinglePositionSupport(stage_shadows.front());
    const double tour_single_support_growth =
        single_support - initialized_stage_single_support;
    const std::size_t tour_cycle_count =
        forward->phases.size() + reverse.phases.size();

    json metric{
        {"stage", "Task10.7c"},
        {"certified_displacement_m", forward->displacement.norm()},
        {"tour_duration_upper_s", tour_result.duration_upper_s},
        {"scalar_update_slots", 2 * updates_per_edge},
        {"initial_position_radius_m", 0.01},
        {"initialized_stage_single_position_support_m",
         initialized_stage_single_support},
        {"final_single_position_support_m", single_support},
        {"final_relative_position_support_m", relative_support},
        {"average_single_support_growth_per_cycle_m",
         tour_single_support_growth / tour_cycle_count},
        {"average_single_support_growth_per_scalar_slot_m",
         tour_single_support_growth / (2 * updates_per_edge)},
        {"final_covariance_upper_eigenvalue", final_covariance_upper},
        {"minimum_analytic_h", std::min(
             edge_audits[0].minimum_h, edge_audits[1].minimum_h)},
        {"minimum_analytic_psi1", std::min(
             edge_audits[0].minimum_psi1,
             edge_audits[1].minimum_psi1)},
        {"minimum_analytic_hard_margin", std::min(
             edge_audits[0].minimum_hard_margin,
             edge_audits[1].minimum_hard_margin)},
        {"minimum_analytic_fim", std::min(
             edge_audits[0].minimum_fim, edge_audits[1].minimum_fim)},
        {"minimum_actual_qp_residual", minimum_actual_hard_residual},
        {"maximum_control_error", maximum_control_error},
        {"certified_tour_cells", tour_cells.size()},
        {"certified_tour_cell_ids", tour_cells},
        {"new_certified_cells", new_cells.size()}};
    std::cout << "TASK10P7C_METRIC " << metric.dump() << '\n';
}
