#pragma once

#include "grand_finale/Task18ExperimentFixture.hpp"

namespace gf {

struct Task19ProductionDefaults {
    std::string policy="production";
    double predictive_tau_mps2=14.0;
    double coverage_checkpoint_s=500.0;
    double resource_watchdog_s=900.0;
    bool terminate_at_certified_t100=true;
};

inline Task19ProductionDefaults task19ProductionDefaults() {
    return {};
}

class Task19EfficiencyAccumulator {
public:
    Task19EfficiencyAccumulator(std::size_t valid_cells,double dt_s)
        : valid_cells_(valid_cells),dt_s_(dt_s) {
        if (valid_cells_==0||!std::isfinite(dt_s_)||dt_s_<=0.0)
            throw std::invalid_argument("invalid Task 19 efficiency domain");
    }

    void observe(std::size_t certified_cells) {
        if (certified_cells>valid_cells_)
            throw std::invalid_argument("certified count exceeds Task 19 domain");
        j_uncovered_cell_seconds_+=
            static_cast<double>(valid_cells_-certified_cells)*dt_s_;
        latch(certified_cells,0.50,t50_tick_);
        latch(certified_cells,0.95,t95_tick_);
        latch(certified_cells,0.99,t99_tick_);
        latch(certified_cells,1.00,t100_tick_);
        ++observations_;
    }

    double j_uncovered_cell_seconds() const {
        return j_uncovered_cell_seconds_;
    }
    std::optional<std::size_t> t50_tick() const { return t50_tick_; }
    std::optional<std::size_t> t95_tick() const { return t95_tick_; }
    std::optional<std::size_t> t99_tick() const { return t99_tick_; }
    std::optional<std::size_t> t100_tick() const { return t100_tick_; }

private:
    void latch(std::size_t certified_cells,double fraction,
        std::optional<std::size_t>& output) {
        if (!output.has_value()&&
            static_cast<double>(certified_cells)>=
                fraction*static_cast<double>(valid_cells_)-1.0e-12)
            output=observations_;
    }

    std::size_t valid_cells_;
    double dt_s_;
    std::size_t observations_=0;
    double j_uncovered_cell_seconds_=0.0;
    std::optional<std::size_t> t50_tick_,t95_tick_,t99_tick_,t100_tick_;
};

inline double task19SquaredControlEnergyIncrement(
    const std::map<NodeId,Eigen::Vector2d>& controls,double dt_s) {
    double result=0.0;
    for (const auto& [owner,control]:controls) {
        static_cast<void>(owner);
        result+=control.squaredNorm()*dt_s;
    }
    return result;
}

enum class Task19RunOutcome {
    CertifiedT100,
    BudgetCensored,
    SafetyFailure
};

inline Task19RunOutcome classifyTask19Run(
    std::optional<std::size_t> t100_tick,bool hard_stop,
    bool runtime_safety_violation) {
    if (hard_stop||runtime_safety_violation)
        return Task19RunOutcome::SafetyFailure;
    if (t100_tick.has_value()) return Task19RunOutcome::CertifiedT100;
    return Task19RunOutcome::BudgetCensored;
}

inline std::string task19RunOutcomeName(Task19RunOutcome outcome) {
    switch (outcome) {
        case Task19RunOutcome::CertifiedT100: return "certified_t100";
        case Task19RunOutcome::BudgetCensored: return "budget_censored";
        case Task19RunOutcome::SafetyFailure: return "safety_failure";
    }
    throw std::runtime_error("unknown Task 19 run outcome");
}

inline GrandFinaleSwarmAdapterConfig task19ProductionAdapterConfig(
    double range_noise_std_m=0.0,
    double range_dropout_probability=0.0,
    unsigned int range_random_seed=2027) {
    auto config=task18ExperimentAdapterConfig(false,
        Task18YawObjective::ActualVelocity,range_noise_std_m,
        range_dropout_probability,range_random_seed,true,
        GammaFeedbackSelectionMode::LeastIntervention,false);
    config.predictive_gamma_tau_mps2=14.0;
    return config;
}

struct Task19ProductionFixture {
    Task10p10Scenario scenario;
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11hSimpleCoverageController controller;

    Task19ProductionFixture(double noise_std_m,double dropout_probability,
        unsigned int seed)
        : scenario(task10p11rFixedBaselineScenario()),
          settings(task10p11pSwarmSettings(scenario,SolverProfile::Gurobi)),
          swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task19ProductionAdapterConfig(
                  noise_std_m,dropout_probability,seed)),
          controller(swarm,adapter,{}, {},
              task10p11rAuthorityContract().branches) {}
};

inline std::unique_ptr<Task19ProductionFixture> makeTask19ProductionFixture(
    double noise_std_m=0.0,double dropout_probability=0.0,
    unsigned int seed=2027) {
    return std::make_unique<Task19ProductionFixture>(
        noise_std_m,dropout_probability,seed);
}

}  // namespace gf
