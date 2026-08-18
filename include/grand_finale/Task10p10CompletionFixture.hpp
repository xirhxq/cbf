#pragma once

#include "Swarm.hpp"
#include "grand_finale/CoverageScaleBudget.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

#include <fstream>
#include <memory>
#include <optional>

namespace gf {

struct Task10p10FrozenConfig {
    std::string cbf2026_source_commit;
    double grid_spacing_m = 0.0;
    double sector_inner_radius_m = 0.0;
    double sector_outer_radius_m = 0.0;
    double sector_half_angle_deg = 0.0;
    double acceleration_half_box_mps2 = 0.0;
    double control_period_s = 0.0;
    std::optional<double> speed_limit_mps;
    double completion_fraction = 0.0;
    double timeout_cap_s = 0.0;
    double minimum_timeout_factor = 0.0;
};

inline Task10p10FrozenConfig loadTask10p10Config() {
    const json raw = json::parse(std::ifstream(
        std::string(PROJECT_ROOT)+
        "/config/grand_finale/task10p10_coverage_scale_gate.json"));
    const auto& coverage = raw.at("coverage");
    Task10p10FrozenConfig result;
    result.cbf2026_source_commit = raw.at("cbf2026_source_commit");
    result.grid_spacing_m = raw.at("grid_spacing_m");
    result.sector_inner_radius_m = coverage.at("inner_radius_m");
    result.sector_outer_radius_m = coverage.at("outer_radius_m");
    result.sector_half_angle_deg = coverage.at("half_angle_deg");
    result.acceleration_half_box_mps2 = raw.at("acceleration_half_box_mps2");
    result.control_period_s = raw.at("control_period_s");
    if (!raw.at("speed_limit_mps").is_null())
        result.speed_limit_mps = raw.at("speed_limit_mps").get<double>();
    result.completion_fraction = raw.at("completion_fraction");
    result.timeout_cap_s = raw.at("timeout_cap_s");
    result.minimum_timeout_factor = raw.at("minimum_timeout_factor");
    return result;
}

struct Task10p10Scenario {
    std::string id;
    double width_m = 0.0;
    double height_m = 0.0;
    std::vector<NodeId> mobile_ids;
    std::vector<Eigen::Vector2d> mobile_positions;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    std::vector<DirectedEdge> initial_topology;
};

inline std::vector<NodeId> task10p10MobileIds(std::size_t count) {
    std::vector<NodeId> ids;
    for (std::size_t index=0; index<count; ++index)
        ids.push_back(static_cast<NodeId>(index+1));
    return ids;
}

inline Task10p10Scenario task10p10EasyScenario() {
    return {"easy",600.0,400.0,{1,2,3,4},
        {{50.0,120.0},{60.0,280.0},{80.0,160.0},{90.0,240.0}},
        {{100,{0.0,50.0}},{101,{0.0,350.0}},{102,{550.0,200.0}}},
        {{100,1},{101,1},{100,2},{1,2},
         {101,3},{1,3},{2,4},{3,4}}};
}

inline std::vector<Eigen::Vector2d> task10p10ScaleMobilePositions() {
    std::vector<Eigen::Vector2d> positions;
    const std::vector<double> xs{60.0,80.0,100.0,120.0};
    const std::vector<double> ys{130.0,210.0,290.0,370.0};
    for (double y : ys) for (double x : xs) {
        if (positions.size()==14) return positions;
        positions.push_back({x,y});
    }
    return positions;
}

inline std::vector<DirectedEdge> task10p10TwoFixedTopology() {
    std::vector<DirectedEdge> edges;
    for (NodeId owner : task10p10MobileIds(14)) {
        edges.push_back({100,owner});
        edges.push_back({101,owner});
    }
    return edges;
}

inline Task10p10Scenario task10p10NonbindingScenario() {
    return {"nonbinding",800.0,500.0,task10p10MobileIds(14),
        task10p10ScaleMobilePositions(),
        {{100,{0.0,100.0}},{101,{0.0,400.0}},{102,{700.0,250.0}}},
        task10p10TwoFixedTopology()};
}

inline Task10p10Scenario task10p10BindingScenario() {
    return {"binding",1800.0,500.0,task10p10MobileIds(14),
        task10p10ScaleMobilePositions(),
        {{100,{0.0,250.0}},{101,{800.0,450.0}},{102,{1600.0,250.0}}},
        task10p10TwoFixedTopology()};
}

struct Task10p10ScenarioBudget {
    CoverageScaleBudget coverage;
    double farthest_required_center_m = 0.0;
    RestToRestTime motion;
    double timeout_factor = 0.0;
};

inline Task10p10ScenarioBudget evaluateTask10p10ScenarioBudget(
    const Task10p10Scenario& scenario) {
    const auto frozen = loadTask10p10Config();
    const CoverageGridSpec grid{
        scenario.width_m,scenario.height_m,frozen.grid_spacing_m};
    const SectorFootprintSpec footprint{
        frozen.sector_inner_radius_m,frozen.sector_outer_radius_m,
        frozen.sector_half_angle_deg*M_PI/180.0};
    std::vector<SectorFootprintStage> initialized;
    for (const auto& position : scenario.mobile_positions)
        initialized.push_back({position,0.0});

    // This is an optimistic reachability union, not a service route: every
    // finite cell center is admitted only after its minimum distance from an
    // initialized vehicle is included in the common motion budget.  The later
    // formal smoke, not this upper bound, tests whether the frontier policy
    // actually services the denominator.
    std::vector<SectorFootprintStage> reachable;
    double farthest = 0.0;
    const int nx = static_cast<int>(std::llround(
        scenario.width_m/frozen.grid_spacing_m));
    const int ny = static_cast<int>(std::llround(
        scenario.height_m/frozen.grid_spacing_m));
    for (int ix=0; ix<nx; ++ix) {
        for (int iy=0; iy<ny; ++iy) {
            const Eigen::Vector2d center(
                (ix+0.5)*frozen.grid_spacing_m,
                (iy+0.5)*frozen.grid_spacing_m);
            double nearest = std::numeric_limits<double>::infinity();
            for (const auto& initial : scenario.mobile_positions)
                nearest = std::min(nearest,(center-initial).norm());
            farthest = std::max(farthest,nearest);
            reachable.push_back({center,0.0});
        }
    }
    Task10p10ScenarioBudget result;
    result.coverage = evaluateCoverageScaleBudget(
        grid,footprint,initialized,reachable,frozen.completion_fraction);
    result.farthest_required_center_m = farthest;
    result.motion = minimumRestToRestTime(
        farthest,frozen.acceleration_half_box_mps2,frozen.speed_limit_mps);
    result.timeout_factor = frozen.timeout_cap_s/result.motion.seconds;
    return result;
}

inline json task10p10SwarmSettings(
    const Task10p10Scenario& scenario,
    SolverProfile profile) {
    const auto frozen = loadTask10p10Config();
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT)+"/config/config_second_order.json"));
    settings["num"] = scenario.mobile_ids.size();
    settings["optimiser"] =
        profile==SolverProfile::Gurobi ? "Gurobi" : "OSQP";
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0,1,2}};
    settings["bases"] = json::array();
    for (const auto& [id,position] : scenario.fixed_positions) {
        (void)id;
        settings["bases"].push_back({position.x(),position.y()});
    }
    settings["initial"]["position"]["method"] = "specified";
    settings["initial"]["position"]["positions"] = json::array();
    settings["initial"]["velocity"]["values"] = json::array();
    for (const auto& position : scenario.mobile_positions) {
        settings["initial"]["position"]["positions"].push_back(
            {position.x(),position.y()});
        settings["initial"]["velocity"]["values"].push_back({0.0,0.0});
    }
    settings["initial"]["yawDeg"] = 0.0;
    settings["world"]["boundary"] = {
        {0.0,0.0},{scenario.width_m,0.0},
        {scenario.width_m,scenario.height_m},{0.0,scenario.height_m}};
    settings["world"]["spacing"] = frozen.grid_spacing_m;
    settings["searching"]["method"] = "front-sector";
    settings["searching"]["front-sector"] = {
        {"inner-radius",frozen.sector_inner_radius_m},
        {"outer-radius",frozen.sector_outer_radius_m},
        {"half-angle-deg",frozen.sector_half_angle_deg}};
    return settings;
}

inline GrandFinaleSwarmAdapterConfig task10p10AdapterConfig(
    SolverProfile profile) {
    const auto frozen = loadTask10p10Config();
    GrandFinaleSwarmAdapterConfig config;
    config.solver_profile = profile;
    config.dt_s = frozen.control_period_s;
    config.minimum_dwell_s = frozen.control_period_s;
    config.acceleration_half_box = frozen.acceleration_half_box_mps2;
    // Historical Task 10.10 scale fixture, retained as development evidence.
    // Formal Task 10.11g/l scenarios override this with 10 m.
    config.collision_distance_m = 0.1;
    config.sensor_radius_m = frozen.sector_outer_radius_m;
    config.coverage_footprint_kind = CoverageFootprintKind::ForwardSector;
    config.coverage_inner_radius_m = frozen.sector_inner_radius_m;
    config.coverage_half_angle_rad =
        frozen.sector_half_angle_deg*M_PI/180.0;
    config.range_dropout_probability = 0.0;
    return config;
}

struct Task10p10Fixture {
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;

    Task10p10Fixture(const Task10p10Scenario& scenario,SolverProfile profile)
        : settings(task10p10SwarmSettings(scenario,profile)),swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
                  scenario.initial_topology,task10p10AdapterConfig(profile)) {}
};

inline std::unique_ptr<Task10p10Fixture> makeTask10p10Fixture(
    const Task10p10Scenario& scenario,
    SolverProfile profile) {
    return std::make_unique<Task10p10Fixture>(scenario,profile);
}

}  // namespace gf
