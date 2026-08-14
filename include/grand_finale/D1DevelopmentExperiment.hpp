#pragma once

#include "utils.h"
#include "grand_finale/Types.hpp"
#include "grand_finale/HybridSupervisor.hpp"

#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

enum class D1TopologyStrategy {
    FixedDag,
    ProposedCertified,
    GreedyCertified
};

enum class D1PendingAction {
    StepOnly,
    BeginRetreat,
    FinishFreshTransition
};

inline D1PendingAction d1PendingAction(
    SupervisorMode mode,
    bool has_fresh_union_cycle,
    std::size_t transition_stack_size) {
    if (mode == SupervisorMode::Union && has_fresh_union_cycle)
        return D1PendingAction::FinishFreshTransition;
    if (mode == SupervisorMode::Retreat && transition_stack_size > 0)
        return D1PendingAction::BeginRetreat;
    return D1PendingAction::StepOnly;
}

struct D1ReplacementCandidate {
    DirectedEdge addition;
    DirectedEdge removal;
    double compatibility_score = 0.0;
};

inline std::vector<DirectedEdge> d1ProposedEligibleEdges(
    const std::vector<DirectedEdge>& old_edges,
    const std::vector<D1ReplacementCandidate>& candidates) {
    if (candidates.empty()) throw std::invalid_argument("empty D1 proposal");
    const DirectedEdge removal = candidates.front().removal;
    std::vector<DirectedEdge> result;
    for (const auto& edge : old_edges)
        if (edge.id() != removal.id()) result.push_back(edge);
    for (const auto& candidate : candidates) {
        if (candidate.removal.id() != removal.id())
            throw std::invalid_argument("D1 candidates disagree on removal");
        result.push_back(candidate.addition);
    }
    std::sort(result.begin(), result.end(),
        [](const DirectedEdge& a, const DirectedEdge& b) {
            return a.id() < b.id();
        });
    result.erase(std::unique(result.begin(), result.end(),
        [](const DirectedEdge& a, const DirectedEdge& b) {
            return a.id() == b.id();
        }), result.end());
    return result;
}

inline std::optional<D1ReplacementCandidate> chooseD1Replacement(
    D1TopologyStrategy strategy,
    std::vector<D1ReplacementCandidate> candidates) {
    if (strategy == D1TopologyStrategy::FixedDag || candidates.empty())
        return std::nullopt;
    if (strategy == D1TopologyStrategy::ProposedCertified)
        return std::nullopt;
    std::sort(candidates.begin(), candidates.end(),
        [](const D1ReplacementCandidate& lhs,
           const D1ReplacementCandidate& rhs) {
            if (lhs.compatibility_score != rhs.compatibility_score)
                return lhs.compatibility_score > rhs.compatibility_score;
            return lhs.addition.id() < rhs.addition.id();
        });
    return candidates.front();
}

inline std::string d1StrategyName(D1TopologyStrategy strategy) {
    switch (strategy) {
        case D1TopologyStrategy::FixedDag: return "fixed_dag";
        case D1TopologyStrategy::ProposedCertified:
            return "proposed_certified";
        case D1TopologyStrategy::GreedyCertified:
            return "greedy_certified";
    }
    throw std::invalid_argument("unknown D1 strategy");
}

inline D1TopologyStrategy parseD1Strategy(const std::string& value) {
    if (value == "fixed_dag") return D1TopologyStrategy::FixedDag;
    if (value == "proposed_certified")
        return D1TopologyStrategy::ProposedCertified;
    if (value == "greedy_certified")
        return D1TopologyStrategy::GreedyCertified;
    throw std::invalid_argument("unknown D1 strategy: " + value);
}

struct D1Map {
    std::string id;
    double width_m = 0.0;
    double height_m = 0.0;
};

struct D1Protocol {
    std::string protocol;
    int mobile_uavs = 0;
    int fixed_buavs = 0;
    double control_period_s = 0.0;
    double timeout_s = 0.0;
    int topology_heartbeat_cycles = 0;
    double grid_spacing_m = 0.0;
    double sensor_radius_m = 0.0;
    double range_noise_std_m = 0.0;
    double range_dropout_probability = 0.0;
    double truth_t95_fraction = 0.95;
    std::vector<D1TopologyStrategy> strategies;
    std::vector<std::string> solver_profiles;
    std::vector<unsigned int> development_seeds;
    std::vector<D1Map> maps;
};

inline D1Protocol parseD1Protocol(const json& raw) {
    if (raw.contains("strategy_overrides")) {
        for (const auto& [strategy, overrides] :
             raw.at("strategy_overrides").items()) {
            (void)strategy;
            if (!overrides.empty()) {
                throw std::invalid_argument(
                    "D1 strategy overrides may only select topology strategy");
            }
        }
    }
    D1Protocol result;
    result.protocol = raw.at("protocol").get<std::string>();
    result.mobile_uavs = raw.at("mobile_uavs").get<int>();
    result.fixed_buavs = raw.at("fixed_buavs").get<int>();
    result.control_period_s = raw.at("control_period_s").get<double>();
    result.timeout_s = raw.at("timeout_s").get<double>();
    result.topology_heartbeat_cycles =
        raw.at("topology_heartbeat_cycles").get<int>();
    result.grid_spacing_m = raw.at("grid_spacing_m").get<double>();
    result.sensor_radius_m = raw.at("sensor_radius_m").get<double>();
    result.range_noise_std_m = raw.at("range_noise_std_m").get<double>();
    result.range_dropout_probability =
        raw.at("range_dropout_probability").get<double>();
    result.truth_t95_fraction = raw.at("truth_t95_fraction").get<double>();
    for (const auto& item : raw.at("strategies"))
        result.strategies.push_back(
            parseD1Strategy(item.get<std::string>()));
    result.solver_profiles =
        raw.at("solver_profiles").get<std::vector<std::string>>();
    result.development_seeds =
        raw.at("development_seeds").get<std::vector<unsigned int>>();
    for (const auto& item : raw.at("maps")) {
        result.maps.push_back({
            item.at("id").get<std::string>(),
            item.at("width_m").get<double>(),
            item.at("height_m").get<double>()});
    }
    if (result.strategies.size() != 3 || result.solver_profiles.size() != 2 ||
        result.development_seeds.size() != 3 || result.maps.size() != 2 ||
        result.mobile_uavs != 14 || result.fixed_buavs != 3 ||
        !std::isfinite(result.control_period_s) ||
        result.control_period_s <= 0.0 || !std::isfinite(result.timeout_s) ||
        result.timeout_s <= 0.0 || result.topology_heartbeat_cycles <= 0 ||
        !std::isfinite(result.grid_spacing_m) ||
        result.grid_spacing_m <= 0.0 ||
        !std::isfinite(result.sensor_radius_m) ||
        result.sensor_radius_m <= 0.0 ||
        !std::isfinite(result.range_noise_std_m) ||
        result.range_noise_std_m < 0.0 ||
        !std::isfinite(result.range_dropout_probability) ||
        result.range_dropout_probability < 0.0 ||
        result.range_dropout_probability > 1.0 ||
        !std::isfinite(result.truth_t95_fraction) ||
        result.truth_t95_fraction <= 0.0 || result.truth_t95_fraction > 1.0) {
        throw std::invalid_argument("invalid frozen D1 protocol");
    }
    return result;
}

inline std::string d1SharedDigest(
    const D1Protocol& protocol,
    const D1Map& map,
    const std::string& solver,
    unsigned int seed) {
    std::ostringstream stream;
    stream << std::setprecision(17)
           << protocol.protocol << '|' << protocol.mobile_uavs << '|'
           << protocol.fixed_buavs << '|' << protocol.control_period_s << '|'
           << protocol.timeout_s << '|' << protocol.topology_heartbeat_cycles
           << '|' << protocol.grid_spacing_m << '|' << protocol.sensor_radius_m
           << '|' << protocol.range_noise_std_m << '|'
           << protocol.range_dropout_probability
           << '|' << protocol.truth_t95_fraction << '|' << map.id << '|'
           << map.width_m << '|' << map.height_m << '|' << solver << '|'
           << seed;
    return stream.str();
}

struct D1Case {
    D1TopologyStrategy strategy = D1TopologyStrategy::FixedDag;
    std::string solver;
    D1Map map;
    unsigned int seed = 0;
    std::string shared_digest;

    std::string pairKey() const {
        return solver + "/" + map.id + "/" + std::to_string(seed);
    }
    std::string key() const {
        return pairKey() + "/" + d1StrategyName(strategy);
    }
};

inline std::vector<D1Case> expandD1Cases(const D1Protocol& protocol) {
    std::vector<D1Case> result;
    for (const std::string& solver : protocol.solver_profiles) {
        if (solver != "gurobi" && solver != "open_source")
            throw std::invalid_argument("unknown D1 solver profile");
        for (const D1Map& map : protocol.maps) {
            for (unsigned int seed : protocol.development_seeds) {
                const std::string digest =
                    d1SharedDigest(protocol, map, solver, seed);
                for (D1TopologyStrategy strategy : protocol.strategies)
                    result.push_back({strategy, solver, map, seed, digest});
            }
        }
    }
    return result;
}

enum class D1Outcome {
    Completed,
    Timeout,
    SafetyFailure,
    ControlFailure,
    InformationFailure,
    SolverFailure,
    Exception
};

inline D1Outcome classifyD1StepFailure(const std::string& reason) {
    if (reason.find("polytope_empty") != std::string::npos ||
        reason.find("infeasible") != std::string::npos)
        return D1Outcome::ControlFailure;
    if (reason.find("solver") != std::string::npos ||
        reason.find("gurobi") != std::string::npos ||
        reason.find("osqp") != std::string::npos)
        return D1Outcome::SolverFailure;
    return D1Outcome::SafetyFailure;
}

struct D1Metrics {
    D1Outcome outcome = D1Outcome::Exception;
    bool denominator_included = true;
    std::optional<double> t95_true_s;
    std::optional<double> t95_proxy_s;
    double final_true_coverage = 0.0;
    double final_proxy_coverage = 0.0;
    std::size_t position_tube_exceedance_steps = 0;
    std::size_t velocity_tube_exceedance_steps = 0;
    bool all_steps_within_development_tube = true;

    void observeCoverage(double time_s, double truth, double proxy) {
        final_true_coverage = truth;
        final_proxy_coverage = proxy;
        if (!t95_true_s.has_value() && truth >= 0.95) t95_true_s = time_s;
        if (!t95_proxy_s.has_value() && proxy >= 0.95) t95_proxy_s = time_s;
    }
    void observeContainment(bool position_contained, bool velocity_contained) {
        if (!position_contained) ++position_tube_exceedance_steps;
        if (!velocity_contained) ++velocity_tube_exceedance_steps;
        all_steps_within_development_tube =
            all_steps_within_development_tube && position_contained &&
            velocity_contained;
    }
    void markTimeout() { outcome = D1Outcome::Timeout; }
};

}  // namespace gf
