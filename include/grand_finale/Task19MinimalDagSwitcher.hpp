#pragma once

#include "grand_finale/Task19ProductionBaseline.hpp"

#include <deque>
#include <set>

namespace gf {

inline std::vector<DirectedEdge> task19OriginDag() {
    return task10p11rFixedReferenceTopology();
}

inline std::vector<DirectedEdge> task19MicrofixDag() {
    auto result=task19OriginDag();
    for (auto& edge:result) {
        if (edge==DirectedEdge{101,2}) edge=DirectedEdge{100,2};
        else if (edge==DirectedEdge{101,9}) edge=DirectedEdge{102,9};
    }
    return result;
}

inline std::set<std::string> task19CanonicalDag(
    const std::vector<DirectedEdge>& topology) {
    std::set<std::string> result;
    for (const auto& edge:topology) result.insert(edge.id());
    return result;
}

inline bool task19SameDag(const std::vector<DirectedEdge>& first,
    const std::vector<DirectedEdge>& second) {
    return task19CanonicalDag(first)==task19CanonicalDag(second);
}

inline std::size_t task19DagReplacementCount(
    const std::vector<DirectedEdge>& first,
    const std::vector<DirectedEdge>& second) {
    const auto a=task19CanonicalDag(first);
    const auto b=task19CanonicalDag(second);
    return static_cast<std::size_t>(std::count_if(a.begin(),a.end(),
        [&](const auto& edge) { return b.count(edge)==0; }));
}

// Coverage-role equivalence is intentionally narrower than graph validity:
// only the two researcher-approved anchor rewires may differ from origin.
// Leaders, member order and target lifting remain the frozen Task 18 contract.
inline bool task19CoverageRoleEquivalentDag(
    const std::vector<DirectedEdge>& topology) {
    if (topology.size()!=task19OriginDag().size()) return false;
    const auto actual=task19CanonicalDag(topology);
    auto required=task19CanonicalDag(task19OriginDag());
    required.erase(DirectedEdge{101,2}.id());
    required.erase(DirectedEdge{101,9}.id());
    for (const auto& edge:required)
        if (actual.count(edge)==0) return false;
    const bool owner2=(actual.count(DirectedEdge{101,2}.id())+
                       actual.count(DirectedEdge{100,2}.id()))==1;
    const bool owner9=(actual.count(DirectedEdge{101,9}.id())+
                       actual.count(DirectedEdge{102,9}.id()))==1;
    return owner2&&owner9&&actual.size()==required.size()+2;
}

struct Task19InterventionSignalConfig {
    std::size_t window_ticks=250;
    std::size_t minimum_dwell_ticks=2250;
    std::size_t evaluation_period_ticks=50;
    double trigger_fraction=0.40;
};

class Task19InterventionSignal {
public:
    explicit Task19InterventionSignal(Task19InterventionSignalConfig config)
        : config_(config) {
        if (config_.window_ticks==0||config_.minimum_dwell_ticks==0||
            config_.evaluation_period_ticks==0||
            !std::isfinite(config_.trigger_fraction)||
            config_.trigger_fraction<0.0||config_.trigger_fraction>1.0)
            throw std::invalid_argument("invalid Task 19 switch signal");
    }

    void observe(std::size_t tick,std::size_t intervened,
        std::size_t owner_ticks) {
        if (owner_ticks==0||intervened>owner_ticks)
            throw std::invalid_argument("invalid Task 19 intervention count");
        samples_.push_back({tick,intervened,owner_ticks});
        intervention_sum_+=intervened;
        owner_sum_+=owner_ticks;
        while (samples_.size()>config_.window_ticks) {
            intervention_sum_-=samples_.front().intervened;
            owner_sum_-=samples_.front().owners;
            samples_.pop_front();
        }
    }

    bool evaluationDue(std::size_t tick) const {
        return tick+1>=config_.minimum_dwell_ticks&&
            (tick+1)%config_.evaluation_period_ticks==0&&
            samples_.size()==config_.window_ticks;
    }

    double rollingFraction() const {
        return owner_sum_==0?0.0:
            static_cast<double>(intervention_sum_)/
            static_cast<double>(owner_sum_);
    }

    bool shouldSwitch(std::size_t tick) const {
        return evaluationDue(tick)&&
            rollingFraction()>config_.trigger_fraction;
    }

private:
    struct Sample {
        std::size_t tick;
        std::size_t intervened;
        std::size_t owners;
    };
    Task19InterventionSignalConfig config_;
    std::deque<Sample> samples_;
    std::size_t intervention_sum_=0;
    std::size_t owner_sum_=0;
};

struct Task19DagSwitchConfig {
    Task19InterventionSignalConfig signal;
};

struct Task19DagSwitchAudit {
    std::size_t signal_evaluations=0;
    std::size_t switch_requests=0;
    std::size_t preflight_rejections=0;
    std::size_t transition_start_failures=0;
    std::size_t certified_edge_replacements=0;
    std::optional<std::size_t> trigger_tick;
    double trigger_fraction=0.0;
    std::string failure_reason;
};

struct Task19DagSwitchEvent {
    bool signal_evaluated=false;
    bool switch_requested=false;
    bool transition_started=false;
    bool transition_finished=false;
    double rolling_intervention_fraction=0.0;
    std::string reason;
};

class Task19OriginMicrofixSwitcher {
public:
    Task19OriginMicrofixSwitcher(GrandFinaleSwarmAdapter& adapter,
        Task19DagSwitchConfig config={})
        : adapter_(adapter),signal_(config.signal) {}

    Task19DagSwitchEvent afterStep(std::size_t tick,
        const GrandFinaleSwarmStep& step) {
        Task19DagSwitchEvent event;
        std::size_t intervened=0;
        for (const auto& [owner,diagnostic]:step.gamma_feedback) {
            static_cast<void>(owner);
            if (diagnostic.intervened) ++intervened;
        }
        if (!step.gamma_feedback.empty())
            signal_.observe(tick,intervened,step.gamma_feedback.size());
        event.rolling_intervention_fraction=signal_.rollingFraction();

        if (phase_==Phase::FirstPending||phase_==Phase::SecondPending) {
            const auto runtime=adapter_.runtimeSnapshot();
            if (runtime.adapter_transition_pending&&
                runtime.mode==SupervisorMode::Union&&
                adapter_.finishReplacementAfterFreshCycle()) {
                event.transition_finished=true;
                ++audit_.certified_edge_replacements;
                if (phase_==Phase::FirstPending) {
                    // Require one fresh SEARCH cycle before certifying the
                    // second replacement.  The intermediate topology is a
                    // transient stage of the same coverage-role-equivalent
                    // DAG action; the target map is unchanged throughout.
                    phase_=Phase::SecondReady;
                    event.reason="first_microfix_edge_complete";
                } else {
                    phase_=Phase::Complete;
                    event.reason="microfix_dag_action_complete";
                }
            }
            return event;
        }
        if (phase_==Phase::SecondReady) {
            if (adapter_.beginReplacement(
                    DirectedEdge{102,9},DirectedEdge{101,9})) {
                phase_=Phase::SecondPending;
                event.transition_started=true;
                event.reason="second_microfix_edge_started";
            } else {
                ++audit_.transition_start_failures;
                event.reason="second_microfix_edge_start_rejected";
            }
            return event;
        }
        if (phase_!=Phase::Monitoring) return event;
        if (!signal_.evaluationDue(tick)) return event;
        event.signal_evaluated=true;
        ++audit_.signal_evaluations;
        if (!signal_.shouldSwitch(tick)) {
            event.reason="rolling_intervention_below_threshold";
            return event;
        }
        event.switch_requested=true;
        ++audit_.switch_requests;
        audit_.trigger_tick=tick;
        audit_.trigger_fraction=signal_.rollingFraction();
        if (!task19SameDag(adapter_.runtimeSnapshot().topology,
                           task19OriginDag())) {
            phase_=Phase::Failed;
            audit_.failure_reason="switch_requires_origin_dag";
            event.reason=audit_.failure_reason;
            return event;
        }
        const auto first=adapter_.auditReplacement(
            DirectedEdge{100,2},DirectedEdge{101,2});
        const auto second=adapter_.auditReplacement(
            DirectedEdge{102,9},DirectedEdge{101,9});
        if (!first.valid||!second.valid) {
            ++audit_.preflight_rejections;
            event.reason="microfix_pair_preflight_rejected";
            return event;
        }
        if (!adapter_.beginReplacement(
                DirectedEdge{100,2},DirectedEdge{101,2})) {
            ++audit_.transition_start_failures;
            event.reason="first_microfix_edge_start_rejected";
            return event;
        }
        phase_=Phase::FirstPending;
        event.transition_started=true;
        event.reason="first_microfix_edge_started";
        return event;
    }

    bool complete() const { return phase_==Phase::Complete; }
    bool failed() const { return phase_==Phase::Failed; }
    const Task19DagSwitchAudit& audit() const { return audit_; }

private:
    enum class Phase {
        Monitoring,FirstPending,SecondReady,SecondPending,Complete,Failed
    };
    GrandFinaleSwarmAdapter& adapter_;
    Task19InterventionSignal signal_;
    Task19DagSwitchAudit audit_;
    Phase phase_=Phase::Monitoring;
};

}  // namespace gf
