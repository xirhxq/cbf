#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/ReferenceGeometry.hpp"
#include "utils.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kMobileCount = 14;
constexpr int kSeed = 2027;

struct MapCase {
    std::string id;
    double width;
    double height;
};

std::vector<gf::NodeId> mobileIds() {
    std::vector<gf::NodeId> ids;
    for (int id = 1; id <= kMobileCount; ++id) ids.push_back(id);
    return ids;
}

std::vector<Eigen::Vector2d> mobilePositions(const MapCase& map) {
    const std::vector<double> xs = map.width > 40.0
        ? std::vector<double>{10.0, 30.0, 50.0, 70.0}
        : std::vector<double>{8.0, 16.0, 24.0, 32.0};
    const std::vector<double> ys{8.0, 16.0, 24.0, 32.0};
    std::vector<Eigen::Vector2d> result;
    for (double y : ys) {
        for (double x : xs) {
            if (result.size() == kMobileCount) return result;
            result.push_back({x, y});
        }
    }
    return result;
}

std::map<gf::NodeId, Eigen::Vector2d> fixedPositions(
    const MapCase& map,
    bool fim_rejection_geometry = false) {
    auto result = std::map<gf::NodeId, Eigen::Vector2d>{
        {100, {2.0, 2.0}},
        {101, {2.0, map.height - 2.0}},
        {102, {map.width - 2.0, 2.0}}};
    if (fim_rejection_geometry) {
        const Eigen::Vector2d owner = mobilePositions(map).front();
        result[102] = owner + 0.5 * (result.at(101) - owner);
    }
    return result;
}

json swarmSettings(
    const MapCase& map,
    gf::SolverProfile profile,
    const std::map<gf::NodeId, Eigen::Vector2d>& fixed) {
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
    settings["num"] = kMobileCount;
    settings["optimiser"] = profile == gf::SolverProfile::Gurobi
        ? "Gurobi" : "OSQP";
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0, 1, 2}};
    settings["bases"] = json::array();
    for (const auto& [id, position] : fixed) {
        (void)id;
        settings["bases"].push_back({position.x(), position.y()});
    }
    settings["initial"]["position"]["positions"] = json::array();
    settings["initial"]["velocity"]["values"] = json::array();
    for (const Eigen::Vector2d& position : mobilePositions(map)) {
        settings["initial"]["position"]["positions"].push_back(
            {position.x(), position.y()});
        settings["initial"]["velocity"]["values"].push_back({0.0, 0.0});
    }
    settings["world"]["boundary"] = {
        {0.0, 0.0}, {map.width, 0.0},
        {map.width, map.height}, {0.0, map.height}};
    settings["world"]["spacing"] = 2.0;
    settings["searching"]["downward"]["radius"] = 3.0;
    return settings;
}

std::vector<gf::DirectedEdge> initialTopology() {
    std::vector<gf::DirectedEdge> edges;
    for (gf::NodeId owner : mobileIds()) {
        edges.push_back({100, owner});
        edges.push_back({
            static_cast<gf::NodeId>(owner == 2 ? 1 : 101), owner});
    }
    return edges;
}

gf::GrandFinaleSwarmAdapterConfig adapterConfig(gf::SolverProfile profile) {
    gf::GrandFinaleSwarmAdapterConfig config;
    config.solver_profile = profile;
    config.dt_s = 0.1;
    config.minimum_dwell_s = 0.1;
    config.acceleration_half_box = 0.4;
    config.sensor_radius_m = 3.0;
    config.certified_error_bound_m = 0.0;
    return config;
}

std::string profileName(gf::SolverProfile profile) {
    return profile == gf::SolverProfile::Gurobi ? "gurobi" : "open_source";
}

struct D0Metrics {
    bool passed = true;
    std::string failure_reason;
    int advanced_steps = 0;
    int completed_switches = 0;
    int no_good_rejections = 0;
    std::size_t minimum_effective_reference_count =
        std::numeric_limits<std::size_t>::max();
    bool cycle_rejected_by_miqp_solver = false;
    bool fim_rejected_by_exact_certifier = false;
    double minimum_hard_residual = std::numeric_limits<double>::infinity();
    double minimum_fim_eigenvalue = std::numeric_limits<double>::infinity();
    double maximum_posterior_eigenvalue = 0.0;
    double truth_coverage = 0.0;
    double certified_coverage = 0.0;
    bool false_covered = false;
};

void auditSnapshot(
    gf::GrandFinaleSwarmAdapter& adapter,
    D0Metrics& metrics) {
    const gf::CurrentReferenceAudit audit = adapter.currentReferenceAudit();
    metrics.minimum_effective_reference_count = std::min(
        metrics.minimum_effective_reference_count,
        audit.minimum_effective_reference_count);
    if (audit.minimum_effective_reference_count < 2) {
        metrics.passed = false;
        metrics.failure_reason = "effective_reference_count";
    }
    metrics.maximum_posterior_eigenvalue = std::max(
        metrics.maximum_posterior_eigenvalue,
        audit.maximum_posterior_eigenvalue);
    metrics.minimum_fim_eigenvalue = std::min(
        metrics.minimum_fim_eigenvalue,
        audit.minimum_fim_eigenvalue);
    if (!(metrics.maximum_posterior_eigenvalue <= 0.1)) {
        metrics.passed = false;
        metrics.failure_reason = "posterior";
    }
    if (!(metrics.minimum_fim_eigenvalue >= 1.0e-6)) {
        metrics.passed = false;
        metrics.failure_reason = "fim";
    }
}

bool certifiedStep(
    gf::GrandFinaleSwarmAdapter& adapter,
    D0Metrics& metrics) {
    const auto step = adapter.step();
    if (!step.advanced || step.certified_control_count != kMobileCount ||
        step.minimum_hard_residual < -1.0e-7) {
        metrics.passed = false;
        metrics.failure_reason = step.reason;
        return false;
    }
    ++metrics.advanced_steps;
    metrics.minimum_hard_residual = std::min(
        metrics.minimum_hard_residual, step.minimum_hard_residual);
    metrics.truth_coverage = step.truth_coverage;
    metrics.certified_coverage = step.certified_coverage;
    metrics.false_covered =
        !adapter.coverage().certifiedSubsetOfTruth();
    if (metrics.false_covered) {
        metrics.passed = false;
        metrics.failure_reason = "false_covered";
    }
    auditSnapshot(adapter, metrics);
    return metrics.passed;
}

bool completeSwitch(
    gf::GrandFinaleSwarmAdapter& adapter,
    const gf::DirectedEdge& addition,
    const gf::DirectedEdge& removal,
    D0Metrics& metrics) {
    if (!adapter.beginReplacement(addition, removal)) {
        metrics.passed = false;
        metrics.failure_reason = "switch_certificate";
        return false;
    }
    if (!certifiedStep(adapter, metrics) || adapter.unionControlCycles() < 1) {
        if (metrics.failure_reason.empty()) metrics.failure_reason = "union_cycle";
        metrics.passed = false;
        return false;
    }
    if (!adapter.finishReplacementAfterFreshCycle()) {
        metrics.passed = false;
        metrics.failure_reason = "fresh_break_certificate";
        return false;
    }
    ++metrics.completed_switches;
    return true;
}

gf::TopologyRequest rejectionRequest(
    const gf::GrandFinaleSwarmAdapter& adapter) {
    std::vector<gf::DirectedEdge> eligible;
    for (const auto& edge : adapter.supervisor().topology()) {
        if (edge.id() != "100->1") eligible.push_back(edge);
    }
    eligible.push_back({102, 1});
    std::map<std::string, double> progress;
    for (const auto& edge : adapter.supervisor().topology())
        progress[edge.id()] = edge.id() == "101->1" ? 100.0 : 0.0;
    progress["102->1"] = 20.0;
    return {mobileIds(), {100, 101, 102}, eligible,
            adapter.supervisor().topology(), 2, 2,
            progress, {}, {}, {}};
}

D0Metrics runCase(
    const MapCase& map,
    gf::SolverProfile profile,
    const std::string& demand,
    unsigned int seed) {
    const bool rejection = demand == "cycle_and_fim_rejection";
    const auto fixed = fixedPositions(map, rejection);
    json settings = swarmSettings(map, profile, fixed);
    settings["execute"]["random-seed"] = seed;
    if (seedRandomFromConfig(settings, 0U) != seed)
        throw std::runtime_error("development seed was not applied");
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, mobileIds(), fixed, initialTopology(), adapterConfig(profile));
    D0Metrics metrics;
    if (!certifiedStep(adapter, metrics)) return metrics;

    if (demand == "single_switch") {
        completeSwitch(adapter, {102, 1}, {100, 1}, metrics);
    } else if (demand == "three_edgewise_switches") {
        for (int index = 0; index < 3 && metrics.passed; ++index) {
            const bool uses_100 = gf::transition_certifier_detail::contains(
                adapter.supervisor().topology(), {100, 1});
            completeSwitch(
                adapter,
                uses_100 ? gf::DirectedEdge{102, 1}
                         : gf::DirectedEdge{100, 1},
                uses_100 ? gf::DirectedEdge{100, 1}
                         : gf::DirectedEdge{102, 1}, metrics);
            if (index < 2 && metrics.passed) certifiedStep(adapter, metrics);
        }
    } else if (rejection) {
        auto cycle_eligible = adapter.supervisor().topology();
        cycle_eligible.push_back({2, 1});
        std::map<std::string, double> cycle_progress;
        for (const auto& edge : cycle_eligible)
            cycle_progress[edge.id()] = edge.id() == "2->1" ? 100.0 : 0.0;
        const gf::TopologyModel cycle_model({
            mobileIds(), {100, 101, 102}, cycle_eligible,
            adapter.supervisor().topology(), 2, 2,
            cycle_progress, {}, {}, {}});
        std::unique_ptr<gf::TopologySolver> cycle_solver;
        if (profile == gf::SolverProfile::Gurobi)
            cycle_solver = std::make_unique<gf::GurobiTopologySolver>();
        else
            cycle_solver = std::make_unique<gf::HighsTopologySolver>();
        const gf::TopologySolution cycle_solution =
            cycle_solver->solve(cycle_model);
        metrics.cycle_rejected_by_miqp_solver =
            cycle_solution.status == gf::TopologySolveStatus::Optimal &&
            !gf::transition_certifier_detail::contains(
                cycle_solution.edges, {2, 1}) &&
            gf::transition_certifier_detail::contains(
                cycle_solution.edges, {100, 1});
        const auto proposal = adapter.proposeAndBegin(rejectionRequest(adapter));
        metrics.no_good_rejections = static_cast<int>(
            proposal.no_good_rejections);
        std::string trace = proposal.reason;
        for (const auto& reason : proposal.rejection_reasons)
            trace += "|" + reason;
        metrics.fim_rejected_by_exact_certifier =
            trace.find("fim") != std::string::npos;
        if (!metrics.cycle_rejected_by_miqp_solver ||
            proposal.transition_started || proposal.no_good_rejections < 1 ||
            !metrics.fim_rejected_by_exact_certifier) {
            metrics.passed = false;
            metrics.failure_reason = "rejection_trace:" + trace;
        } else if (!certifiedStep(adapter, metrics) ||
                   adapter.supervisor().mode() != gf::SupervisorMode::Hold) {
            metrics.passed = false;
            metrics.failure_reason = "hold_flow";
        }
    } else {
        certifiedStep(adapter, metrics);
    }
    return metrics;
}

void printMetrics(
    const MapCase& map,
    gf::SolverProfile profile,
    const std::string& demand,
    const D0Metrics& metrics) {
    json record{
        {"stage", "D0"}, {"seed", kSeed},
        {"solver", profileName(profile)}, {"map", map.id},
        {"topology_demand", demand}, {"passed", metrics.passed},
        {"failure_reason", metrics.failure_reason},
        {"advanced_steps", metrics.advanced_steps},
        {"completed_switches", metrics.completed_switches},
        {"no_good_rejections", metrics.no_good_rejections},
        {"minimum_effective_reference_count",
         metrics.minimum_effective_reference_count},
        {"cycle_rejected_by_miqp_solver",
         metrics.cycle_rejected_by_miqp_solver},
        {"fim_rejected_by_exact_certifier",
         metrics.fim_rejected_by_exact_certifier},
        {"minimum_hard_residual", metrics.minimum_hard_residual},
        {"minimum_fim_eigenvalue", metrics.minimum_fim_eigenvalue},
        {"maximum_posterior_eigenvalue", metrics.maximum_posterior_eigenvalue},
        {"truth_coverage", metrics.truth_coverage},
        {"certified_coverage", metrics.certified_coverage},
        {"false_covered", metrics.false_covered}};
    std::cout << "D0_METRIC " << record.dump() << '\n';
}

}  // namespace

TEST_CASE("Task 10.5 D0 runs all sixteen 14+3 deterministic mechanisms") {
    const json config = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) +
        "/config/grand_finale/task10p5_d0_14p3.json"));
    const unsigned int seed = config.at("development_seed").get<unsigned int>();
    REQUIRE(seed == kSeed);
    REQUIRE(config.at("mobile_uavs") == kMobileCount);
    REQUIRE(config.at("fixed_buavs") == 3);
    REQUIRE(config.at("control_period_s") == doctest::Approx(0.1));
    REQUIRE(config.at("grid_spacing_m") == doctest::Approx(2.0));
    REQUIRE(config.at("sensor_radius_m") == doctest::Approx(3.0));
    std::vector<MapCase> maps;
    for (const auto& item : config.at("maps"))
        maps.push_back({
            item.at("id").get<std::string>(),
            item.at("width_m").get<double>(),
            item.at("height_m").get<double>()});
    std::vector<std::string> demands =
        config.at("topology_demands").get<std::vector<std::string>>();
    std::vector<gf::SolverProfile> profiles;
    for (const std::string& name :
         config.at("solver_profiles").get<std::vector<std::string>>()) {
        if (name == "gurobi") profiles.push_back(gf::SolverProfile::Gurobi);
        else if (name == "open_source")
            profiles.push_back(gf::SolverProfile::OpenSource);
        else FAIL("unknown D0 solver profile");
    }
    REQUIRE(maps.size() == 2);
    REQUIRE(demands.size() == 4);
    REQUIRE(profiles.size() == 2);
    int completed = 0;
    for (const auto profile : profiles) {
        for (const auto& map : maps) {
            for (const auto& demand : demands) {
                CAPTURE(profileName(profile));
                CAPTURE(map.id);
                CAPTURE(demand);
                D0Metrics metrics;
                try {
                    metrics = runCase(map, profile, demand, seed);
                } catch (const std::exception& error) {
                    metrics.passed = false;
                    metrics.failure_reason =
                        std::string("exception:") + error.what();
                } catch (...) {
                    metrics.passed = false;
                    metrics.failure_reason = "exception:unknown";
                }
                printMetrics(map, profile, demand, metrics);
                CHECK(metrics.passed);
                CHECK_FALSE(metrics.false_covered);
                CHECK(metrics.minimum_hard_residual >= -1.0e-7);
                ++completed;
            }
        }
    }
    CHECK(completed == 16);
}
