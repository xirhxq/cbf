#include "Swarm.hpp"
#include "grand_finale/D1DevelopmentExperiment.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/ReferenceGeometry.hpp"

#include <Eigen/Eigenvalues>

#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;

std::vector<gf::NodeId> mobileIds() {
    std::vector<gf::NodeId> ids;
    for (int id = 1; id <= 14; ++id) ids.push_back(id);
    return ids;
}

std::vector<Eigen::Vector2d> mobilePositions(const gf::D1Map& map) {
    const std::vector<double> xs = map.width_m > 40.0
        ? std::vector<double>{10.0, 30.0, 50.0, 70.0}
        : std::vector<double>{8.0, 16.0, 24.0, 32.0};
    const std::vector<double> ys{8.0, 16.0, 24.0, 32.0};
    std::vector<Eigen::Vector2d> positions;
    for (double y : ys) for (double x : xs) {
        if (positions.size() == 14) return positions;
        positions.push_back({x, y});
    }
    return positions;
}

std::map<gf::NodeId, Eigen::Vector2d> fixedPositions(const gf::D1Map& map) {
    return {{100, {2.0, 2.0}},
            {101, {2.0, map.height_m - 2.0}},
            {102, {map.width_m - 2.0, 2.0}}};
}

std::vector<gf::DirectedEdge> initialTopology() {
    std::vector<gf::DirectedEdge> edges;
    for (gf::NodeId owner : mobileIds()) {
        edges.push_back({100, owner});
        edges.push_back({static_cast<gf::NodeId>(owner == 2 ? 1 : 101), owner});
    }
    return edges;
}

json swarmSettings(
    const gf::D1Protocol& protocol,
    const gf::D1Case& run,
    const std::map<gf::NodeId, Eigen::Vector2d>& fixed) {
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
    settings["num"] = protocol.mobile_uavs;
    settings["optimiser"] = run.solver == "gurobi" ? "Gurobi" : "OSQP";
    settings["execute"]["random-seed"] = run.seed;
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0, 1, 2}};
    settings["bases"] = json::array();
    for (const auto& [id, position] : fixed) {
        (void)id;
        settings["bases"].push_back({position.x(), position.y()});
    }
    settings["initial"]["position"]["positions"] = json::array();
    settings["initial"]["velocity"]["values"] = json::array();
    for (const auto& position : mobilePositions(run.map)) {
        settings["initial"]["position"]["positions"].push_back(
            {position.x(), position.y()});
        settings["initial"]["velocity"]["values"].push_back({0.0, 0.0});
    }
    settings["world"]["boundary"] = {
        {0.0, 0.0}, {run.map.width_m, 0.0},
        {run.map.width_m, run.map.height_m}, {0.0, run.map.height_m}};
    settings["world"]["spacing"] = protocol.grid_spacing_m;
    settings["searching"]["downward"]["radius"] = protocol.sensor_radius_m;
    return settings;
}

gf::GrandFinaleSwarmAdapterConfig adapterConfig(
    const gf::D1Protocol& protocol,
    const gf::D1Case& run,
    const json& raw) {
    gf::GrandFinaleSwarmAdapterConfig config;
    config.solver_profile = run.solver == "gurobi"
        ? gf::SolverProfile::Gurobi : gf::SolverProfile::OpenSource;
    config.dt_s = protocol.control_period_s;
    config.minimum_dwell_s = protocol.control_period_s;
    config.acceleration_half_box = 0.4;
    config.sensor_radius_m = protocol.sensor_radius_m;
    config.certified_error_bound_m = 0.0;
    config.range_random_seed = run.seed;
    config.range_noise_std_m = raw.at("range_noise_std_m").get<double>();
    config.range_dropout_probability =
        raw.at("range_dropout_probability").get<double>();
    return config;
}

struct RunMetrics : gf::D1Metrics {
    std::string failure_reason;
    std::string last_topology_failure_reason;
    std::string last_fresh_break_reason;
    std::size_t steps = 0;
    double minimum_reference_h = std::numeric_limits<double>::infinity();
    double minimum_reference_psi1 = std::numeric_limits<double>::infinity();
    double minimum_collision_h = std::numeric_limits<double>::infinity();
    double minimum_collision_psi1 = std::numeric_limits<double>::infinity();
    double minimum_hard_residual = std::numeric_limits<double>::infinity();
    double maximum_position_error_m = 0.0;
    double maximum_velocity_error_mps = 0.0;
    double minimum_fim_margin = std::numeric_limits<double>::infinity();
    double minimum_posterior_margin = std::numeric_limits<double>::infinity();
    double minimum_aoi_margin_s = std::numeric_limits<double>::infinity();
    std::size_t qp_infeasible = 0, solver_failures = 0, rejected_controls = 0;
    std::size_t proposals = 0, certifier_rejections = 0, no_goods = 0;
    std::size_t fresh_break_rejections = 0;
    std::size_t union_cycles = 0, retreats = 0, holds = 0, switches = 0;
    double minimum_switch_duration_s = std::numeric_limits<double>::infinity();
    double maximum_switch_duration_s = 0.0;
    double minimum_switch_dwell_s = std::numeric_limits<double>::infinity();
    double qp_wall_s = 0.0, miqp_wall_s = 0.0;
    double topology_strategy_wall_s = 0.0, total_wall_s = 0.0;
};

std::map<gf::NodeId, Eigen::Vector2d> estimatePositions(
    const gf::JointEstimateSnapshot& snapshot) {
    std::map<gf::NodeId, Eigen::Vector2d> result;
    for (std::size_t i = 0; i < snapshot.mobile_ids.size(); ++i)
        result[snapshot.mobile_ids[i]] = snapshot.mean.segment<2>(4 * i);
    return result;
}

double compatibilityScore(
    const gf::DirectedEdge& edge,
    const gf::JointEstimateSnapshot& snapshot,
    const std::map<gf::NodeId, Eigen::Vector2d>& fixed,
    const std::map<gf::NodeId, Eigen::Vector2d>& nominal) {
    const auto positions = estimatePositions(snapshot);
    const Eigen::Vector2d owner = positions.at(edge.owner);
    const Eigen::Vector2d reference = positions.count(edge.reference)
        ? positions.at(edge.reference) : fixed.at(edge.reference);
    const Eigen::Vector2d delta = reference - owner;
    if (delta.norm() < 1.0e-12) return 0.0;
    return delta.normalized().dot(nominal.at(edge.owner));
}

std::vector<gf::D1ReplacementCandidate> candidateReplacements(
    gf::NodeId owner,
    gf::GrandFinaleSwarmAdapter& adapter,
    const std::map<gf::NodeId, Eigen::Vector2d>& fixed) {
    const auto snapshot = adapter.runtimeSnapshot().estimate;
    const auto nominal = adapter.currentNominalControls();
    const auto topology = adapter.supervisor().topology();
    std::vector<gf::DirectedEdge> owned;
    std::set<gf::NodeId> current_refs;
    for (const auto& edge : topology) if (edge.owner == owner) {
        owned.push_back(edge);
        current_refs.insert(edge.reference);
    }
    if (owned.size() != 2) return {};
    const gf::DirectedEdge removal = owned.back();
    const double old_score = compatibilityScore(
        removal, snapshot, fixed, nominal);
    std::vector<gf::D1ReplacementCandidate> result;
    std::vector<gf::NodeId> references = mobileIds();
    for (const auto& [id, p] : fixed) { (void)p; references.push_back(id); }
    for (gf::NodeId reference : references) {
        if (reference == owner || current_refs.count(reference)) continue;
        const gf::DirectedEdge addition{reference, owner};
        const double score = compatibilityScore(
            addition, snapshot, fixed, nominal);
        if (score > old_score + 1.0e-9)
            result.push_back({addition, removal, score});
    }
    return result;
}

gf::TopologyRequest proposedRequest(
    gf::GrandFinaleSwarmAdapter& adapter,
    const std::vector<gf::D1ReplacementCandidate>& candidates,
    const std::map<gf::NodeId, Eigen::Vector2d>& fixed) {
    if (candidates.empty()) throw std::invalid_argument("empty D1 proposal");
    gf::TopologyRequest request;
    request.mobile_ids = mobileIds();
    for (const auto& [id, p] : fixed) { (void)p; request.fixed_ids.push_back(id); }
    request.old_edges = adapter.supervisor().topology();
    request.eligible_edges = gf::d1ProposedEligibleEdges(
        request.old_edges, candidates);
    const auto snapshot = adapter.runtimeSnapshot().estimate;
    const auto nominal = adapter.currentNominalControls();
    request.min_indegree = 2;
    request.max_indegree = 2;
    for (const auto& edge : request.eligible_edges)
        request.progress_coefficients[edge.id()] =
            compatibilityScore(edge, snapshot, fixed, nominal);
    const auto positions = estimatePositions(snapshot);
    for (std::size_t i = 0; i < request.eligible_edges.size(); ++i) {
        for (std::size_t j = i + 1; j < request.eligible_edges.size(); ++j) {
            const auto& a = request.eligible_edges[i];
            const auto& b = request.eligible_edges[j];
            if (a.owner != b.owner) continue;
            const Eigen::Vector2d pa = positions.count(a.reference)
                ? positions.at(a.reference) : fixed.at(a.reference);
            const Eigen::Vector2d pb = positions.count(b.reference)
                ? positions.at(b.reference) : fixed.at(b.reference);
            const Eigen::Vector2d po = positions.at(a.owner);
            if ((pa-po).norm() < 1e-12 || (pb-po).norm() < 1e-12) continue;
            const double cross = std::abs(
                (pa-po).normalized().x() * (pb-po).normalized().y() -
                (pa-po).normalized().y() * (pb-po).normalized().x());
            request.fim_pair_coefficients[a.id()+"|"+b.id()] = cross*cross;
        }
    }
    return request;
}

void auditAfterStep(
    Swarm& swarm,
    gf::GrandFinaleSwarmAdapter& adapter,
    const gf::GrandFinaleSwarmStep& step,
    RunMetrics& metrics) {
    metrics.minimum_hard_residual = std::min(
        metrics.minimum_hard_residual, step.minimum_hard_residual);
    metrics.qp_wall_s += step.qp_wall_s;
    const auto rows = adapter.currentSnapshotHardRows(
        adapter.supervisor().topology());
    for (const auto& row : rows) {
        if (row.kind == gf::CanonicalHardRowKind::ReferenceDistance) {
            metrics.minimum_reference_h = std::min(
                metrics.minimum_reference_h, row.barrier_h);
            metrics.minimum_reference_psi1 = std::min(
                metrics.minimum_reference_psi1, row.barrier_psi1);
        } else if (row.kind == gf::CanonicalHardRowKind::Collision) {
            metrics.minimum_collision_h = std::min(
                metrics.minimum_collision_h, row.barrier_h);
            metrics.minimum_collision_psi1 = std::min(
                metrics.minimum_collision_psi1, row.barrier_psi1);
        }
    }
    const auto runtime = adapter.runtimeSnapshot();
    for (std::size_t i = 0; i < runtime.estimate.mobile_ids.size(); ++i) {
        const Eigen::Vector2d p_est = runtime.estimate.mean.segment<2>(4*i);
        const Eigen::Vector2d v_est = runtime.estimate.mean.segment<2>(4*i+2);
        const Point p = swarm.robots[i]->model->xy();
        const Eigen::Vector2d v = swarm.robots[i]->model->getVelocity().head<2>();
        const double pe = (p_est-Eigen::Vector2d(p.x,p.y)).norm();
        const double ve = (v_est-v).norm();
        metrics.maximum_position_error_m = std::max(
            metrics.maximum_position_error_m, pe);
        metrics.maximum_velocity_error_mps = std::max(
            metrics.maximum_velocity_error_mps, ve);
        const double pt = adapter.config().uncertainty_sigma * std::sqrt(
            std::max(0.0, gf::detail::maximumPositionEigenvalue(
                runtime.estimate, runtime.estimate.mobile_ids[i])));
        const double vt = adapter.config().uncertainty_sigma * std::sqrt(
            std::max(0.0, gf::detail::maximumVelocityEigenvalue(
                runtime.estimate, runtime.estimate.mobile_ids[i])));
        metrics.observeContainment(pe <= pt + 1e-12, ve <= vt + 1e-12);
    }
    const auto reference = adapter.currentReferenceAudit();
    metrics.minimum_fim_margin = std::min(
        metrics.minimum_fim_margin,
        reference.minimum_fim_eigenvalue - 1.0e-6);
    metrics.minimum_posterior_margin = std::min(
        metrics.minimum_posterior_margin,
        adapter.config().maximum_posterior_eigenvalue_m2 -
            reference.maximum_posterior_eigenvalue);
    if (reference.minimum_effective_reference_count < 2 ||
        !std::isfinite(reference.minimum_fim_eigenvalue) ||
        !std::isfinite(reference.maximum_posterior_eigenvalue)) {
        metrics.minimum_fim_margin = -std::numeric_limits<double>::infinity();
    }
    for (const auto& edge : adapter.supervisor().topology()) {
        const std::string id = gf::UndirectedEdge::canonical(
            edge.reference, edge.owner).id();
        const auto link = runtime.range_links.find(id);
        const double margin = link == runtime.range_links.end()
            ? -std::numeric_limits<double>::infinity()
            : adapter.config().maximum_range_aoi_s - link->second.age_s;
        metrics.minimum_aoi_margin_s = std::min(
            metrics.minimum_aoi_margin_s, margin);
    }
}

RunMetrics runCase(
    const gf::D1Protocol& protocol,
    const gf::D1Case& run,
    const json& raw,
    int smoke_steps = -1) {
    const auto wall_start = Clock::now();
    RunMetrics metrics;
    const auto fixed = fixedPositions(run.map);
    json settings = swarmSettings(protocol, run, fixed);
    seedRandomFromConfig(settings, 0U);
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, mobileIds(), fixed, initialTopology(),
        adapterConfig(protocol, run, raw));
    std::optional<double> switch_start;
    const std::size_t maximum_steps = smoke_steps >= 0
        ? static_cast<std::size_t>(smoke_steps)
        : static_cast<std::size_t>(std::ceil(
            protocol.timeout_s / protocol.control_period_s));
    for (std::size_t cycle = 0; cycle < maximum_steps; ++cycle) {
        const auto pending_action = gf::d1PendingAction(
            adapter.supervisor().mode(), adapter.unionControlCycles() >= 1,
            adapter.transitionStackSize());
        if (pending_action == gf::D1PendingAction::FinishFreshTransition) {
            if (!adapter.finishReplacementAfterFreshCycle()) {
                ++metrics.fresh_break_rejections;
                ++metrics.certifier_rejections;
                metrics.last_fresh_break_reason =
                    adapter.lastCertificationReason();
                switch_start.reset();
            } else {
                ++metrics.switches;
                const double duration = swarm.robots.front()->runtime -
                    switch_start.value_or(swarm.robots.front()->runtime);
                metrics.minimum_switch_duration_s = std::min(
                    metrics.minimum_switch_duration_s, duration);
                metrics.maximum_switch_duration_s = std::max(
                    metrics.maximum_switch_duration_s, duration);
                metrics.minimum_switch_dwell_s = std::min(
                    metrics.minimum_switch_dwell_s,
                    duration);
                switch_start.reset();
            }
        } else if (pending_action == gf::D1PendingAction::BeginRetreat) {
            if (!adapter.beginRetreatReplacement()) {
                metrics.outcome = gf::D1Outcome::SafetyFailure;
                metrics.failure_reason = "retreat_certificate_rejected";
                break;
            }
            switch_start = swarm.robots.front()->runtime;
        }
        if (run.strategy != gf::D1TopologyStrategy::FixedDag && cycle > 0 &&
            cycle % static_cast<std::size_t>(
                protocol.topology_heartbeat_cycles) == 0 &&
            adapter.supervisor().mode() == gf::SupervisorMode::Search) {
            const gf::NodeId owner = 1 + static_cast<gf::NodeId>(
                (cycle / protocol.topology_heartbeat_cycles - 1) % 14);
            const auto candidates = candidateReplacements(owner, adapter, fixed);
            if (!candidates.empty()) {
                ++metrics.proposals;
                bool started = false;
                const auto miqp_start = Clock::now();
                if (run.strategy == gf::D1TopologyStrategy::ProposedCertified) {
                    const auto result = adapter.proposeAndBegin(
                        proposedRequest(adapter, candidates, fixed));
                    started = result.transition_started;
                    metrics.no_goods += result.no_good_rejections;
                    metrics.certifier_rejections += result.no_good_rejections;
                    if (!started && result.reason.find("solver") != std::string::npos)
                        ++metrics.solver_failures;
                    if (!started)
                        metrics.last_topology_failure_reason = result.reason;
                } else {
                    std::vector<gf::D1ReplacementCandidate> remaining = candidates;
                    while (!remaining.empty()) {
                        const auto choice = gf::chooseD1Replacement(
                            run.strategy, remaining);
                        if (!choice.has_value()) break;
                        started = adapter.beginReplacement(
                            choice->addition, choice->removal, false);
                        if (started) break;
                        ++metrics.certifier_rejections;
                        remaining.erase(std::remove_if(
                            remaining.begin(), remaining.end(),
                            [&](const auto& candidate) {
                                return candidate.addition.id() ==
                                    choice->addition.id();
                            }), remaining.end());
                    }
                    if (!started) adapter.supervisor().requestReformation(
                        swarm.robots.front()->runtime, false,
                        adapter.transitionStackSize() > 0);
                }
                const double topology_wall = std::chrono::duration<double>(
                    Clock::now() - miqp_start).count();
                metrics.topology_strategy_wall_s += topology_wall;
                if (run.strategy ==
                    gf::D1TopologyStrategy::ProposedCertified) {
                    metrics.miqp_wall_s += topology_wall;
                }
                if (started) switch_start = swarm.robots.front()->runtime;
            }
        }
        const auto step = adapter.step();
        if (!step.advanced) {
            ++metrics.rejected_controls;
            metrics.failure_reason = step.reason;
            metrics.outcome = gf::classifyD1StepFailure(step.reason);
            if (metrics.outcome == gf::D1Outcome::ControlFailure)
                ++metrics.qp_infeasible;
            if (metrics.outcome == gf::D1Outcome::SolverFailure)
                ++metrics.solver_failures;
            break;
        }
        ++metrics.steps;
        if (step.mode == gf::SupervisorMode::Union) ++metrics.union_cycles;
        if (step.mode == gf::SupervisorMode::Retreat) ++metrics.retreats;
        if (step.mode == gf::SupervisorMode::Hold) ++metrics.holds;
        metrics.observeCoverage(
            swarm.robots.front()->runtime,
            step.truth_coverage, step.certified_coverage);
        auditAfterStep(swarm, adapter, step, metrics);
        if (!std::isfinite(metrics.minimum_aoi_margin_s) ||
            metrics.minimum_aoi_margin_s < -1.0e-12 ||
            metrics.minimum_fim_margin < -1.0e-12 ||
            metrics.minimum_posterior_margin < -1.0e-12) {
            metrics.outcome = gf::D1Outcome::InformationFailure;
            metrics.failure_reason = "reference_information_aoi_gate";
            break;
        }
        if (step.minimum_hard_residual < -adapter.config().residual_tolerance ||
            metrics.minimum_reference_h < -1e-7 ||
            metrics.minimum_reference_psi1 < -1e-7 ||
            metrics.minimum_collision_h < -1e-7 ||
            metrics.minimum_collision_psi1 < -1e-7) {
            metrics.outcome = gf::D1Outcome::SafetyFailure;
            metrics.failure_reason = "post_step_robust_barrier_audit";
            break;
        }
        if (metrics.t95_true_s.has_value()) {
            metrics.outcome = gf::D1Outcome::Completed;
            metrics.failure_reason.clear();
            break;
        }
    }
    if (metrics.outcome == gf::D1Outcome::Exception) {
        if (smoke_steps >= 0) {
            metrics.outcome = gf::D1Outcome::Completed;
            metrics.failure_reason = "smoke_horizon";
        } else {
            metrics.markTimeout();
            metrics.failure_reason = "timeout";
        }
    }
    metrics.total_wall_s = std::chrono::duration<double>(
        Clock::now() - wall_start).count();
    return metrics;
}

std::string outcomeName(gf::D1Outcome outcome) {
    switch (outcome) {
        case gf::D1Outcome::Completed: return "completed";
        case gf::D1Outcome::Timeout: return "timeout";
        case gf::D1Outcome::SafetyFailure: return "safety_failure";
        case gf::D1Outcome::ControlFailure: return "control_failure";
        case gf::D1Outcome::InformationFailure: return "information_failure";
        case gf::D1Outcome::SolverFailure: return "solver_failure";
        case gf::D1Outcome::Exception: return "exception";
    }
    return "unknown";
}

json metricJson(const gf::D1Case& run, const RunMetrics& m) {
    const auto optional = [](const std::optional<double>& value) -> json {
        return value.has_value() ? json(*value) : json(nullptr);
    };
    return {
        {"stage", "D1"}, {"case_key", run.key()},
        {"pair_key", run.pairKey()}, {"shared_digest", run.shared_digest},
        {"strategy", gf::d1StrategyName(run.strategy)},
        {"solver", run.solver}, {"map", run.map.id}, {"seed", run.seed},
        {"outcome", outcomeName(m.outcome)},
        {"denominator_included", m.denominator_included},
        {"failure_reason", m.failure_reason}, {"steps", m.steps},
        {"last_topology_failure_reason", m.last_topology_failure_reason},
        {"last_fresh_break_reason", m.last_fresh_break_reason},
        {"t95_true_s", optional(m.t95_true_s)},
        {"t95_proxy_s", optional(m.t95_proxy_s)},
        {"final_true_coverage", m.final_true_coverage},
        {"final_proxy_coverage", m.final_proxy_coverage},
        {"true_proxy_gap", m.final_true_coverage-m.final_proxy_coverage},
        {"minimum_reference_h", m.minimum_reference_h},
        {"minimum_reference_psi1", m.minimum_reference_psi1},
        {"minimum_collision_h", m.minimum_collision_h},
        {"minimum_collision_psi1", m.minimum_collision_psi1},
        {"minimum_robust_hard_residual", m.minimum_hard_residual},
        {"qp_infeasible", m.qp_infeasible},
        {"solver_failures", m.solver_failures},
        {"rejected_controls", m.rejected_controls},
        {"maximum_position_error_m", m.maximum_position_error_m},
        {"maximum_velocity_error_mps", m.maximum_velocity_error_mps},
        {"position_tube_exceedance_steps", m.position_tube_exceedance_steps},
        {"velocity_tube_exceedance_steps", m.velocity_tube_exceedance_steps},
        {"minimum_reference_fim_proxy_margin", m.minimum_fim_margin},
        {"minimum_posterior_margin", m.minimum_posterior_margin},
        {"minimum_aoi_margin_s", m.minimum_aoi_margin_s},
        {"reformation_proposals", m.proposals},
        {"certifier_rejections", m.certifier_rejections},
        {"fresh_break_rejections", m.fresh_break_rejections},
        {"no_good_count", m.no_goods}, {"union_cycles", m.union_cycles},
        {"retreat_cycles", m.retreats}, {"hold_cycles", m.holds},
        {"completed_switches", m.switches},
        {"minimum_switch_duration_s", std::isfinite(m.minimum_switch_duration_s)
            ? json(m.minimum_switch_duration_s) : json(nullptr)},
        {"maximum_switch_duration_s", m.maximum_switch_duration_s},
        {"minimum_switch_dwell_s", std::isfinite(m.minimum_switch_dwell_s)
            ? json(m.minimum_switch_dwell_s) : json(nullptr)},
        {"qp_wall_s", m.qp_wall_s}, {"miqp_wall_s", m.miqp_wall_s},
        {"topology_strategy_wall_s", m.topology_strategy_wall_s},
        {"total_wall_s", m.total_wall_s},
        {"tube_semantics", "covariance_3sigma_development_proxy"}};
}

}  // namespace

int main(int argc, char** argv) {
    const bool smoke = argc > 1 && std::string(argv[1]) == "--smoke";
    const bool probe = argc > 1 && std::string(argv[1]) == "--probe";
    const json raw = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) +
        "/config/grand_finale/d1_development_efficiency.json"));
    const gf::D1Protocol protocol = gf::parseD1Protocol(raw);
    const auto cases = gf::expandD1Cases(protocol);
    std::size_t emitted = 0;
    for (const auto& run : cases) {
        if (smoke && (run.solver != "open_source" ||
            run.map.id != "compact_square" || run.seed != 2027)) continue;
        if (probe && (run.solver != "open_source" ||
            run.map.id != "compact_square" || run.seed != 2027 ||
            run.strategy != gf::D1TopologyStrategy::ProposedCertified))
            continue;
        RunMetrics metrics;
        try {
            metrics = runCase(protocol, run, raw, smoke ? 1 : (probe ? 200 : -1));
        } catch (const std::exception& error) {
            metrics.outcome = gf::D1Outcome::Exception;
            metrics.failure_reason = error.what();
        } catch (...) {
            metrics.outcome = gf::D1Outcome::Exception;
            metrics.failure_reason = "unknown exception";
        }
        std::cout << "D1_METRIC " << metricJson(run, metrics).dump() << '\n';
        ++emitted;
    }
    if ((smoke && emitted != 3) || (probe && emitted != 1) ||
        (!smoke && !probe && emitted != 36)) return 2;
    return 0;
}
