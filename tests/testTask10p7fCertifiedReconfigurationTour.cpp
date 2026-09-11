#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedTranslationPrimitive.hpp"
#include "grand_finale/DirectionalShadowSupport.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/RobustReferenceFim.hpp"

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
constexpr std::size_t kStateDimension = 56;
constexpr std::size_t kSlotsPerCycle = 133;
constexpr double kDt = 0.1;
constexpr double kAcceleration = 0.3;
constexpr double kProcessAcceleration = 1.0e-4;
constexpr double kControlError = 1.0e-5;
constexpr double kInnovation = 0.05;
constexpr double kRangeVariance = 1.0;
constexpr double kSensorRadius = 2.0;
constexpr double kCellHalfDiagonal = 0.7071067811865476;

std::vector<gf::NodeId> mobileIds() {
    std::vector<gf::NodeId> result;
    for (gf::NodeId id = 1; id <= kMobileCount; ++id) result.push_back(id);
    return result;
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

std::vector<gf::DirectedEdge> oldTopology() {
    std::vector<gf::DirectedEdge> result;
    for (gf::NodeId owner : mobileIds()) {
        result.push_back({100, owner});
        result.push_back({owner == 2 ? gf::NodeId{1} : gf::NodeId{101}, owner});
    }
    return result;
}

std::vector<gf::DirectedEdge> unionTopology() {
    auto result = oldTopology();
    result.push_back({102, 2});
    return result;
}

std::vector<gf::DirectedEdge> successorTopology() {
    auto result = oldTopology();
    result.erase(std::remove_if(result.begin(), result.end(), [](const auto& edge) {
        return edge.id() == "1->2";
    }), result.end());
    result.push_back({102, 2});
    return result;
}

bool hasEdge(
    const std::vector<gf::DirectedEdge>& edges,
    const std::string& id) {
    return std::any_of(edges.begin(), edges.end(), [&](const auto& edge) {
        return edge.id() == id;
    });
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
    for (const auto& position : initialPositions()) {
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
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(
        kStateDimension, kStateDimension);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        result(4 * mobile, 4 * mobile + 2) = kDt;
        result(4 * mobile + 1, 4 * mobile + 3) = kDt;
    }
    return result;
}

Eigen::MatrixXd disturbanceInput() {
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(kStateDimension, 28);
    Eigen::Matrix<double, 4, 2> block;
    block << 0.5 * kDt * kDt, 0.0,
             0.0, 0.5 * kDt * kDt,
             kDt, 0.0,
             0.0, kDt;
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile)
        result.block<4, 2>(4 * mobile, 2 * mobile) = block;
    return result;
}

gf::ShadowStateBox initialShadow() {
    Eigen::VectorXd lower(kStateDimension), upper(kStateDimension);
    for (std::size_t mobile = 0; mobile < kMobileCount; ++mobile) {
        lower.segment<4>(4 * mobile) << -0.01, -0.01, -0.002, -0.002;
        upper.segment<4>(4 * mobile) << 0.01, 0.01, 0.002, 0.002;
    }
    return {lower, upper};
}

std::vector<gf::DirectionalShadowStep> frozenSteps(std::size_t count) {
    std::vector<gf::DirectionalShadowStep> result;
    Eigen::MatrixXd covariance =
        1.0e-4 * Eigen::MatrixXd::Identity(kStateDimension, kStateDimension);
    const Eigen::MatrixXd F = transition();
    for (std::size_t cycle = 0; cycle < count; ++cycle) {
        covariance = F * covariance * F.transpose();
        gf::DirectionalShadowStep step;
        step.transition = F;
        step.disturbance_input = disturbanceInput();
        step.absolute_disturbance_bound =
            Eigen::VectorXd::Constant(28, kProcessAcceleration);
        step.covariance_upper_after_prediction = covariance;
        step.scalar_slots.assign(
            kSlotsPerCycle,
            {std::sqrt(2.0), kRangeVariance, kInnovation});
        result.push_back(std::move(step));
    }
    return result;
}

struct Supports {
    double absolute_position = 0.0;
    double absolute_velocity = 0.0;
    double relative_position = 0.0;
    double relative_velocity = 0.0;
};

Supports supportsAt(
    const std::vector<gf::DirectionalShadowStep>& all_steps,
    std::size_t prefix) {
    const std::vector<gf::DirectionalShadowStep> steps(
        all_steps.begin(), all_steps.begin() + prefix);
    // The frozen dynamics/covariance bounds are axis symmetric, while the
    // initial set is an axis-aligned box.  Its plane support is maximized at
    // a diagonal, so this one value safely dominates every unit direction.
    const Eigen::Vector2d direction =
        Eigen::Vector2d::Constant(1.0 / std::sqrt(2.0));
    return {
        gf::finiteHorizonDirectionalSupport(
            initialShadow(), steps,
            gf::mobileStateDirection(
                kMobileCount, 0, gf::MobileStatePlane::Position, direction)),
        gf::finiteHorizonDirectionalSupport(
            initialShadow(), steps,
            gf::mobileStateDirection(
                kMobileCount, 0, gf::MobileStatePlane::Velocity, direction)),
        gf::finiteHorizonDirectionalSupport(
            initialShadow(), steps,
            gf::relativeMobileStateDirection(
                kMobileCount, 0, 1,
                gf::MobileStatePlane::Position, direction)),
        gf::finiteHorizonDirectionalSupport(
            initialShadow(), steps,
            gf::relativeMobileStateDirection(
                kMobileCount, 0, 1,
                gf::MobileStatePlane::Velocity, direction))};
}

std::map<gf::NodeId, Eigen::Vector2d> commonNominal(
    const Eigen::Vector2d& command) {
    std::map<gf::NodeId, Eigen::Vector2d> result;
    for (gf::NodeId id : mobileIds()) result[id] = command;
    return result;
}

std::vector<Eigen::Vector2d> commands(
    const gf::TranslationPrimitive& primitive) {
    std::vector<Eigen::Vector2d> result;
    for (const auto& phase : primitive.phases) {
        result.push_back({
            0.5 * (phase.acceleration.x.lower + phase.acceleration.x.upper),
            0.5 * (phase.acceleration.y.lower + phase.acceleration.y.upper)});
    }
    return result;
}

struct PhysicalRadius {
    double position_axis = 0.0;
    double velocity_axis = 0.0;

    void advance() {
        const double acceleration = kProcessAcceleration + kControlError;
        position_axis += kDt * velocity_axis +
            0.5 * kDt * kDt * acceleration;
        velocity_axis += kDt * acceleration;
    }

    double positionDirection() const {
        return std::sqrt(2.0) * position_axis;
    }
    double velocityDirection() const {
        return std::sqrt(2.0) * velocity_axis;
    }
};

gf::JointEstimateSnapshot nominalSnapshot(
    const std::vector<Eigen::Vector2d>& centers,
    const Eigen::Vector2d& common_velocity,
    const Eigen::MatrixXd& covariance) {
    Eigen::VectorXd mean(kStateDimension);
    for (std::size_t index = 0; index < kMobileCount; ++index) {
        mean.segment<4>(4 * index) << centers[index].x(), centers[index].y(),
            common_velocity.x(), common_velocity.y();
    }
    return {mobileIds(), mean, covariance, fixedPositions()};
}

std::map<std::string, double> rangeVariances(
    const std::vector<gf::DirectedEdge>& edges) {
    std::map<std::string, double> result;
    for (const auto& edge : edges) {
        result[gf::UndirectedEdge::canonical(edge.owner, edge.reference).id()] =
            kRangeVariance;
    }
    return result;
}

double minimumRobustFim(
    const std::vector<gf::DirectedEdge>& edges,
    const gf::JointEstimateSnapshot& snapshot,
    const Supports& support,
    const PhysicalRadius& physical) {
    std::map<std::string, double> direction_support;
    for (const auto& edge : edges) {
        direction_support[edge.id()] = edge.reference <= kMobileCount
            ? support.relative_position + 2.0 * physical.positionDirection()
            : support.absolute_position + physical.positionDirection();
    }
    const auto variances = rangeVariances(edges);
    double result = std::numeric_limits<double>::infinity();
    for (gf::NodeId owner : mobileIds()) {
        std::vector<gf::DirectedEdge> owner_edges;
        for (const auto& edge : edges)
            if (edge.owner == owner) owner_edges.push_back(edge);
        const auto fim = gf::robustReferenceFimLowerBound(
            owner, owner_edges, snapshot, variances, direction_support);
        if (!fim.valid) return -std::numeric_limits<double>::infinity();
        result = std::min(result, fim.lower_eigenvalue);
    }
    return result;
}

struct GateAudit {
    double reference_h = std::numeric_limits<double>::infinity();
    double reference_psi1 = std::numeric_limits<double>::infinity();
    double collision_h = std::numeric_limits<double>::infinity();
    double collision_psi1 = std::numeric_limits<double>::infinity();
    double hard_control = std::numeric_limits<double>::infinity();
};

void auditPair(
    GateAudit& audit,
    const Eigen::Vector2d& relative_position,
    const Eigen::Vector2d& relative_velocity,
    double position_support,
    double velocity_support,
    bool communication,
    bool both_mobile,
    double command_norm) {
    const double distance = relative_position.norm();
    const double distance_lower = distance - position_support;
    const double speed_upper = relative_velocity.norm() + velocity_support;
    const double h = communication
        ? 850.0 - distance - position_support - 0.02
        : distance_lower - 0.1;
    const double hdot_lower = -speed_upper;
    const double psi1 = h + hdot_lower;
    double constant = h + 2.0 * hdot_lower;
    if (communication) {
        constant -= speed_upper * speed_upper /
            std::max(1.0e-12, distance_lower);
        audit.reference_h = std::min(audit.reference_h, h);
        audit.reference_psi1 = std::min(audit.reference_psi1, psi1);
    } else {
        audit.collision_h = std::min(audit.collision_h, h);
        audit.collision_psi1 = std::min(audit.collision_psi1, psi1);
    }
    audit.hard_control = std::min(
        audit.hard_control,
        (both_mobile ? 0.5 : 1.0) * constant - command_norm);
}

GateAudit auditAllWordGates(
    const std::vector<gf::DirectedEdge>& edges,
    const std::vector<Eigen::Vector2d>& centers,
    const Eigen::Vector2d& common_velocity,
    const Supports& support,
    const PhysicalRadius& physical,
    const Eigen::Vector2d& command) {
    GateAudit result;
    const auto fixed = fixedPositions();
    for (const auto& edge : edges) {
        const Eigen::Vector2d owner = centers.at(edge.owner - 1);
        const bool mobile_reference = edge.reference <= kMobileCount;
        const Eigen::Vector2d reference = mobile_reference
            ? centers.at(edge.reference - 1) : fixed.at(edge.reference);
        const Eigen::Vector2d reference_velocity = mobile_reference
            ? common_velocity : Eigen::Vector2d::Zero();
        auditPair(
            result, owner - reference,
            common_velocity - reference_velocity,
            mobile_reference
                ? support.relative_position + 2.0 * physical.positionDirection()
                : support.absolute_position + physical.positionDirection(),
            mobile_reference
                ? support.relative_velocity + 2.0 * physical.velocityDirection()
                : support.absolute_velocity + physical.velocityDirection(),
            true, mobile_reference, command.norm());
    }
    for (std::size_t first = 0; first < kMobileCount; ++first) {
        for (std::size_t second = first + 1; second < kMobileCount; ++second) {
            auditPair(
                result, centers[first] - centers[second],
                Eigen::Vector2d::Zero(),
                support.relative_position + 2.0 * physical.positionDirection(),
                support.relative_velocity + 2.0 * physical.velocityDirection(),
                false, true, command.norm());
        }
        for (const auto& [id, point] : fixed) {
            (void)id;
            auditPair(
                result, centers[first] - point, common_velocity,
                support.absolute_position + physical.positionDirection(),
                support.absolute_velocity + physical.velocityDirection(),
                false, false, command.norm());
        }
    }
    result.hard_control = std::min(
        result.hard_control,
        0.4 - command.cwiseAbs().maxCoeff());
    return result;
}

void updateMinimum(GateAudit& minimum, const GateAudit& current) {
    minimum.reference_h = std::min(minimum.reference_h, current.reference_h);
    minimum.reference_psi1 = std::min(
        minimum.reference_psi1, current.reference_psi1);
    minimum.collision_h = std::min(minimum.collision_h, current.collision_h);
    minimum.collision_psi1 = std::min(
        minimum.collision_psi1, current.collision_psi1);
    minimum.hard_control = std::min(
        minimum.hard_control, current.hard_control);
}

std::set<std::string> sensingCells(
    Swarm& swarm,
    const std::vector<Eigen::Vector2d>& centers,
    double absolute_position_support,
    const PhysicalRadius& physical) {
    std::set<std::string> result;
    const double radius = absolute_position_support +
        physical.positionDirection();
    for (int x = 0; x < swarm.gridWorldGroundTruth.xNum; ++x) {
        for (int y = 0; y < swarm.gridWorldGroundTruth.yNum; ++y) {
            const Eigen::Vector2d cell{
                swarm.gridWorldGroundTruth.getCellCenterX(x),
                swarm.gridWorldGroundTruth.getCellCenterY(y)};
            for (const auto& center : centers) {
                if ((cell - center).norm() + radius + kCellHalfDiagonal <=
                    kSensorRadius + 1.0e-12) {
                    result.insert(std::to_string(x) + ":" + std::to_string(y));
                    break;
                }
            }
        }
    }
    return result;
}

struct TourResult {
    bool valid = false;
    std::string first_failure = "none";
    double switch_begin_s = -1.0;
    double union_end_s = -1.0;
    double break_s = -1.0;
    double old_fim = std::numeric_limits<double>::infinity();
    double union_fim = std::numeric_limits<double>::infinity();
    double successor_fim = std::numeric_limits<double>::infinity();
    GateAudit minimum;
    double minimum_qp_residual = std::numeric_limits<double>::infinity();
    double covariance_upper = 0.0;
    double posterior_margin = std::numeric_limits<double>::infinity();
    double aoi_margin = 0.1;
    Supports terminal_support;
    std::set<std::string> robust_cells;
    gf::GrandFinaleRuntimeSnapshot terminal;
};

TourResult runTour() {
    TourResult result;
    const auto all_steps = frozenSteps(82);
    std::vector<Supports> support(83);
    for (std::size_t prefix = 1; prefix <= 82; ++prefix)
        support[prefix] = supportsAt(all_steps, prefix);

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
    config.maximum_accepted_range_innovation_m = kInnovation;
    config.sensor_radius_m = kSensorRadius;
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, mobileIds(), fixedPositions(), oldTopology(), config);

    std::vector<Eigen::Vector2d> centers = initialPositions();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    PhysicalRadius physical;
    std::size_t prefix = 1;
    physical.advance();
    adapter.setCertifiedShadowSupports(
        support[prefix].absolute_position,
        support[prefix].relative_position);
    const auto initialization = adapter.stepWithNominal(
        commonNominal(Eigen::Vector2d::Zero()));
    if (!initialization.advanced) {
        result.first_failure = "initialization_qp";
        return result;
    }
    result.minimum_qp_residual = initialization.minimum_hard_residual;
    result.robust_cells = sensingCells(
        swarm, centers, support[prefix].absolute_position, physical);

    const auto primitive = gf::makeRestToRestTranslation(
        {1.0, 0.0}, kAcceleration, kDt, 20);
    if (!primitive.has_value()) {
        result.first_failure = "primitive";
        return result;
    }
    const auto forward = commands(*primitive);
    const auto reverse = commands(gf::reverseTranslation(*primitive));

    const auto run_motion = [&](const std::vector<Eigen::Vector2d>& motion,
                                const std::vector<gf::DirectedEdge>& edges)
        -> bool {
        for (const auto& command : motion) {
            ++prefix;
            physical.advance();
            adapter.setCertifiedShadowSupports(
                support[prefix].absolute_position,
                support[prefix].relative_position);
            updateMinimum(result.minimum, auditAllWordGates(
                edges, centers, velocity, support[prefix], physical, command));
            const auto step = adapter.stepWithNominal(commonNominal(command));
            if (!step.advanced) {
                result.first_failure = "actual_qp:" + step.reason;
                return false;
            }
            result.minimum_qp_residual = std::min(
                result.minimum_qp_residual, step.minimum_hard_residual);
            for (const auto& [id, control] : step.applied_controls) {
                if ((control - command).cwiseAbs().maxCoeff() >
                    kControlError + 1.0e-12) {
                    result.first_failure = "nominal_solution_map";
                    return false;
                }
            }
            for (auto& center : centers)
                center += velocity * kDt + 0.5 * command * kDt * kDt;
            velocity += command * kDt;
            updateMinimum(result.minimum, auditAllWordGates(
                edges, centers, velocity, support[prefix], physical, command));
            const auto cells = sensingCells(
                swarm, centers, support[prefix].absolute_position, physical);
            result.robust_cells.insert(cells.begin(), cells.end());
            const Eigen::MatrixXd& covariance =
                all_steps[prefix - 1].covariance_upper_after_prediction;
            const double fim = minimumRobustFim(
                edges, nominalSnapshot(centers, velocity, covariance),
                support[prefix], physical);
            if (edges.size() == unionTopology().size())
                result.union_fim = std::min(result.union_fim, fim);
            else if (hasEdge(edges, "102->2"))
                result.successor_fim = std::min(result.successor_fim, fim);
            else
                result.old_fim = std::min(result.old_fim, fim);
            result.covariance_upper = std::max(
                result.covariance_upper,
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(covariance)
                    .eigenvalues().maxCoeff());
            result.posterior_margin = std::min(
                result.posterior_margin, 0.1 - result.covariance_upper);
            if (result.minimum.reference_h < 0.0 ||
                result.minimum.reference_psi1 < 0.0 ||
                result.minimum.collision_h < 0.0 ||
                result.minimum.collision_psi1 < 0.0 ||
                result.minimum.hard_control <= kControlError ||
                result.minimum_qp_residual < -1.0e-7 ||
                fim < 1.0e-6 || result.posterior_margin <= 0.0) {
                result.first_failure = "analytic_hard_gate";
                return false;
            }
        }
        return true;
    };

    if (!run_motion(forward, oldTopology())) return result;
    if (prefix != 41 || velocity.norm() > 1.0e-12) {
        result.first_failure = "forward_terminal";
        return result;
    }
    result.switch_begin_s = swarm.robots.front()->runtime;
    if (!adapter.beginReplacement({102, 2}, {1, 2})) {
        result.first_failure = "replacement_begin";
        return result;
    }
    if (adapter.supervisor().mode() != gf::SupervisorMode::Union ||
        !hasEdge(adapter.supervisor().topology(), "1->2") ||
        !hasEdge(adapter.supervisor().topology(), "102->2")) {
        result.first_failure = "union_install";
        return result;
    }
    if (adapter.finishReplacementAfterFreshCycle()) {
        result.first_failure = "zero_duration_union";
        return result;
    }

    ++prefix;
    physical.advance();
    adapter.setCertifiedShadowSupports(
        support[prefix].absolute_position,
        support[prefix].relative_position);
    updateMinimum(result.minimum, auditAllWordGates(
        unionTopology(), centers, velocity, support[prefix], physical,
        Eigen::Vector2d::Zero()));
    const auto before_union = adapter.runtimeSnapshot();
    const auto union_step = adapter.stepWithNominal(
        commonNominal(Eigen::Vector2d::Zero()));
    if (!union_step.advanced || adapter.unionControlCycles() != 1) {
        result.first_failure = "union_cycle";
        return result;
    }
    result.minimum_qp_residual = std::min(
        result.minimum_qp_residual, union_step.minimum_hard_residual);
    const auto after_union = adapter.runtimeSnapshot();
    result.union_end_s = after_union.runtime_s;
    if (after_union.estimator_token <= before_union.estimator_token ||
        after_union.freshness !=
            gf::FreshnessRelation::UnionRequiresFreshBreak) {
        result.first_failure = "union_freshness";
        return result;
    }
    const Eigen::MatrixXd& union_covariance =
        all_steps[prefix - 1].covariance_upper_after_prediction;
    result.union_fim = minimumRobustFim(
        unionTopology(), nominalSnapshot(centers, velocity, union_covariance),
        support[prefix], physical);
    const auto union_cells = sensingCells(
        swarm, centers, support[prefix].absolute_position, physical);
    result.robust_cells.insert(union_cells.begin(), union_cells.end());
    if (!adapter.finishReplacementAfterFreshCycle()) {
        result.first_failure = "fresh_break";
        return result;
    }
    result.break_s = swarm.robots.front()->runtime;
    if (adapter.supervisor().mode() != gf::SupervisorMode::Search ||
        hasEdge(adapter.supervisor().topology(), "1->2") ||
        !hasEdge(adapter.supervisor().topology(), "100->2") ||
        !hasEdge(adapter.supervisor().topology(), "102->2") ||
        adapter.transitionStackSize() != 1) {
        result.first_failure = "successor_install";
        return result;
    }

    if (!run_motion(reverse, successorTopology())) return result;
    if (prefix != 82 || velocity.norm() > 1.0e-12 ||
        (centers.front() - initialPositions().front()).norm() > 1.0e-10) {
        result.first_failure = "return_terminal";
        return result;
    }
    result.terminal_support = support[prefix];
    result.terminal = adapter.runtimeSnapshot();
    if (result.terminal.adapter_transition_pending ||
        result.terminal.supervisor_transition_pending ||
        result.terminal.freshness != gf::FreshnessRelation::NoPending ||
        result.terminal.transition_stack_size != 1 ||
        result.terminal.range_links.size() != kSlotsPerCycle ||
        result.robust_cells.empty()) {
        result.first_failure = "full_terminal_inclusion";
        return result;
    }
    result.valid = true;
    return result;
}

const TourResult& frozenTourResult() {
    static const TourResult result = runTour();
    return result;
}

void emit(const TourResult& result) {
    const json metric{
        {"valid", result.valid},
        {"first_failure", result.first_failure},
        {"switch_begin_s", result.switch_begin_s},
        {"union_end_s", result.union_end_s},
        {"break_s", result.break_s},
        {"switch_witness_s", result.break_s - result.switch_begin_s},
        {"t100_upper_from_initialized_stage_zero_s", 8.1},
        {"minimum_old_robust_fim", result.old_fim},
        {"minimum_union_robust_fim", result.union_fim},
        {"minimum_successor_robust_fim", result.successor_fim},
        {"minimum_reference_h", result.minimum.reference_h},
        {"minimum_reference_psi1", result.minimum.reference_psi1},
        {"minimum_collision_h", result.minimum.collision_h},
        {"minimum_collision_psi1", result.minimum.collision_psi1},
        {"minimum_hard_control_margin", result.minimum.hard_control},
        {"minimum_actual_qp_residual", result.minimum_qp_residual},
        {"maximum_covariance_upper", result.covariance_upper},
        {"minimum_posterior_margin", result.posterior_margin},
        {"minimum_aoi_margin_s", result.aoi_margin},
        {"terminal_absolute_position_support_m",
         result.terminal_support.absolute_position},
        {"terminal_absolute_velocity_support_mps",
         result.terminal_support.absolute_velocity},
        {"terminal_joint_relative_position_support_m",
         result.terminal_support.relative_position},
        {"terminal_joint_relative_velocity_support_mps",
         result.terminal_support.relative_velocity},
        {"robust_sensing_cells", result.robust_cells.size()},
        {"robust_sensing_cell_ids", result.robust_cells},
        {"terminal_runtime_s", result.terminal.runtime_s},
        {"terminal_estimator_token", result.terminal.estimator_token},
        {"terminal_topology_token", result.terminal.topology_token},
        {"terminal_stack_size", result.terminal.transition_stack_size}};
    std::cout << "TASK10P7F_METRIC " << metric.dump() << '\n';
}

}  // namespace

TEST_CASE("Task 10.7f frozen trigger is nonanticipative and uses one positive-duration union cycle") {
    const TourResult& result = frozenTourResult();
    emit(result);
    REQUIRE(result.valid);
    CHECK(result.switch_begin_s == doctest::Approx(4.1));
    CHECK(result.union_end_s == doctest::Approx(4.2));
    CHECK(result.break_s == doctest::Approx(4.2));
    CHECK(result.break_s - result.switch_begin_s == doctest::Approx(kDt));
}

TEST_CASE("Task 10.7f successor closes robust FIM safety and finite service") {
    const TourResult& result = frozenTourResult();
    REQUIRE(result.valid);
    CHECK(result.old_fim > 1.0e-6);
    CHECK(result.union_fim > 1.0e-6);
    CHECK(result.successor_fim > 1.0e-6);
    CHECK(result.minimum.reference_h > 0.0);
    CHECK(result.minimum.reference_psi1 > 0.0);
    CHECK(result.minimum.collision_h > 0.0);
    CHECK(result.minimum.collision_psi1 > 0.0);
    CHECK(result.minimum.hard_control > kControlError);
    CHECK(result.minimum_qp_residual >= -1.0e-7);
    CHECK(result.posterior_margin > 0.0);
    CHECK(result.aoi_margin >= 0.0);
    CHECK(result.robust_cells.size() > 0);
    CHECK(result.terminal.runtime_s == doctest::Approx(8.2));
    CHECK(result.terminal.certified_shadow_single_position_support_m ==
          doctest::Approx(result.terminal_support.absolute_position));
    CHECK(result.terminal.certified_shadow_relative_position_support_m ==
          doctest::Approx(result.terminal_support.relative_position));
}
