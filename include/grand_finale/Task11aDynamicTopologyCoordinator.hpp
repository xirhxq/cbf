#pragma once

#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p9MechanismFixture.hpp"
#include "grand_finale/Task10p11rStopAttribution.hpp"
#include "grand_finale/Task10p11ahTerminalRecoveryOptimizer.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace gf {

struct Task11aFrozenConstants {
    std::size_t evaluation_period_ticks=50;
    std::size_t minimum_dwell_ticks=10;
    bool proposals_enabled=true;
    std::size_t minimum_indegree=2;
    std::size_t maximum_indegree=2;
};

inline Task11aFrozenConstants task11aFrozenConstants() { return {}; }

// Attribution state required by prereg section 11 (three-element failure
// attribution): which frozen constant was in effect at any moment, and
// whether a certified switch was ever blocked by dwell.
struct Task11aAttributionState {
    std::size_t tick=0;
    std::size_t ticks_since_last_evaluation=
        std::numeric_limits<std::size_t>::max();
    std::size_t ticks_since_last_commit=
        std::numeric_limits<std::size_t>::max();
    std::size_t evaluations=0;
    std::size_t solver_optimal_no_change=0;
    std::size_t solver_no_feasible_assignment=0;
    std::size_t solver_failures=0;
    std::size_t certified_switches=0;
    std::size_t dwell_blocked_evaluations=0;
    std::size_t immediate_triggers=0;
    std::size_t immediate_triggers_unhandled_at_failure=0;
    bool last_immediate_trigger_condition_met=false;
};

struct Task11aEvaluationRecord {
    std::size_t tick=0;
    std::string kind;  // periodic | immediate | disabled
    std::string outcome;  // no_change_optimal | switch_certified |
                          // no_feasible_assignment | solver_failure |
                          // dwell_blocked | disabled
    std::size_t no_good_rejections=0;
    std::vector<std::string> rejection_reasons;
    std::vector<DirectedEdge> selected_topology;
    std::vector<std::string> switch_additions;
    std::vector<std::string> switch_removals;
    bool immediate_trigger_condition_met=false;
};

// Full-course certified reference reconfiguration as a first-class mechanism
// (prereg section 2): periodic re-solve of the assignment MIQP over fresh
// eligibility; hold when the current topology remains optimal; certified
// single-replacement make-before-break when it does not.  With
// proposals_enabled=false the coordinator never touches the adapter and the
// run is the fixed baseline bit-for-bit (zero-drift identity obligation).
class Task11aDynamicTopologyCoordinator {
public:
    Task11aDynamicTopologyCoordinator(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter,
        Task11aFrozenConstants constants)
        : swarm_(swarm),adapter_(adapter),constants_(constants) {}

    bool proposalsEnabled() const { return constants_.proposals_enabled; }

    const Task11aAttributionState& attribution() const { return state_; }

    // Immediate trigger: an owner's current reference assignment is losing a
    // qualification gate (fewer than two effective references).  Pure
    // observation via the adapter audit; never mutates control state.
    bool immediateTriggerConditionMet() {
        if (!constants_.proposals_enabled) return false;
        const auto audit=adapter_.currentReferenceAudit();
        const bool met=audit.minimum_effective_reference_count<2;
        state_.last_immediate_trigger_condition_met=met;
        return met;
    }

    Task11aEvaluationRecord maybeEvaluateAndPropose(std::size_t tick) {
        Task11aEvaluationRecord record;
        record.tick=tick;
        state_.tick=tick;
        if (!constants_.proposals_enabled) {
            record.kind="disabled";
            record.outcome="disabled";
            return record;
        }
        const bool periodic=tick%constants_.evaluation_period_ticks==0;
        const bool immediate=immediateTriggerConditionMet();
        record.immediate_trigger_condition_met=immediate;
        const bool dwell_block=ticks_since_commit_.has_value() &&
            tick-*ticks_since_commit_<constants_.minimum_dwell_ticks;
        if (immediate) ++state_.immediate_triggers;
        if (!periodic&&!immediate) return record;  // silent tick
        if (dwell_block) {
            ++state_.dwell_blocked_evaluations;
            record.kind=immediate?"immediate":"periodic";
            record.outcome="dwell_blocked";
            state_.ticks_since_last_evaluation=0;
            ++state_.evaluations;
            return record;
        }
        record.kind=immediate?"immediate":"periodic";
        ++state_.evaluations;
        state_.ticks_since_last_evaluation=0;
        auto outcome=evaluateOnce(tick,record,immediate);
        record.outcome=outcome;
        return record;
    }

    // Test hook: pretend a commit happened `ticks_ago` ticks ago so the
    // dwell accounting can be exercised without a real transition.
    void noteCommitForTest(std::size_t ticks_ago) {
        ticks_since_commit_=ticks_ago;
    }

    // Probe-only forced evaluation (prereg section 11 failure attribution):
    // ignores period/trigger/dwell so the stopped state answers directly
    // whether a certified alternative exists.  Never used in the live loop.
    Task11aEvaluationRecord forceEvaluate(std::size_t tick) {
        Task11aEvaluationRecord record;
        record.tick=tick;
        record.kind="probe";
        ++state_.evaluations;
        record.outcome=evaluateOnce(tick,record,false);
        return record;
    }

    // After every control cycle: complete a pending make-before-break with a
    // fresh certificate once one union cycle has been served.
    bool recordCycle(const SimpleCoverageControlStep& step) {
        if (!constants_.proposals_enabled) return false;
        if (state_.ticks_since_last_evaluation!=
            std::numeric_limits<std::size_t>::max())
            ++state_.ticks_since_last_evaluation;
        if (ticks_since_commit_.has_value()) ++*ticks_since_commit_;
        if (!step.step.advanced) return false;
        const auto runtime=adapter_.runtimeSnapshot();
        if (runtime.adapter_transition_pending &&
            runtime.mode==SupervisorMode::Union &&
            adapter_.finishReplacementAfterFreshCycle()) {
            ++state_.certified_switches;
            ticks_since_commit_=0;
            return true;
        }
        return false;
    }

private:
    std::vector<DirectedEdge> eligibleUniverse() const {
        const auto runtime=adapter_.runtimeSnapshot();
        std::map<std::string,RangeLinkState> links;
        for (const auto& [id,link] : runtime.range_links)
            links[id]={link.age_s,link.quality};
        const auto gates=buildEligibility(runtime.estimate,links,
            {adapter_.config().add_reference_distance_m,
             adapter_.config().reference_distance_m,
             adapter_.config().maximum_range_aoi_s,
             adapter_.config().maximum_reference_position_eigenvalue_m2,
             adapter_.config().minimum_range_quality,
             adapter_.config().uncertainty_sigma},{});
        std::set<std::string> current;
        for (const auto& edge:runtime.topology) current.insert(edge.id());
        std::vector<DirectedEdge> candidates;
        for (const auto& gate:gates.candidates)
            if (gate.eligible&&!current.count(gate.edge.id()))
                candidates.push_back(gate.edge);
        std::sort(candidates.begin(),candidates.end(),
            [](const auto& a,const auto& b){return a.id()<b.id();});
        return candidates;
    }

    std::string evaluateOnce(std::size_t tick,Task11aEvaluationRecord& record,
                             bool immediate) {
        (void)tick;(void)immediate;
        const auto runtime=adapter_.runtimeSnapshot();
        TopologyRequest request;
        request.mobile_ids=runtime.estimate.mobile_ids;
        for (const auto& [id,position]:runtime.estimate.fixed_positions) {
            (void)position;
            request.fixed_ids.push_back(id);
        }
        request.old_edges=runtime.topology;
        request.eligible_edges=runtime.topology;
        const auto candidates=eligibleUniverse();
        request.eligible_edges.insert(request.eligible_edges.end(),
            candidates.begin(),candidates.end());
        request.min_indegree=constants_.minimum_indegree;
        request.max_indegree=constants_.maximum_indegree;
        // Coverage-policy neutrality (prereg section 2, premise 4): the
        // topology objective carries no coverage-progress term, so the
        // optimum keeps the current assignment unless eligibility forces a
        // change.
        for (const auto& edge:request.eligible_edges)
            request.progress_coefficients[edge.id()]=0.0;
        std::unique_ptr<TopologySolver> solver;
        if (adapter_.config().solver_profile==SolverProfile::Gurobi) {
#ifdef ENABLE_GUROBI
            solver=std::make_unique<GurobiTopologySolver>();
#endif
        } else solver=std::make_unique<HighsTopologySolver>();
        if (!solver) {
            ++state_.solver_failures;
            return "solver_failure";
        }
        const TopologySolution optimal=solver->solve(TopologyModel(request));
        if (optimal.status!=TopologySolveStatus::Optimal) {
            ++state_.solver_no_feasible_assignment;
            record.rejection_reasons.push_back(optimal.detail);
            return "no_feasible_assignment";
        }
        const auto additions=edgeDifferenceNew(optimal.edges,
            runtime.topology);
        const auto removals=edgeDifferenceNew(runtime.topology,optimal.edges);
        if (additions.empty()&&removals.empty()) {
            ++state_.solver_optimal_no_change;
            record.selected_topology=optimal.edges;
            return "no_change_optimal";
        }
        const auto proposal=adapter_.proposeAndBegin(request);
        record.no_good_rejections=proposal.no_good_rejections;
        record.rejection_reasons=proposal.rejection_reasons;
        record.selected_topology=proposal.selected_topology;
        if (proposal.transition_started) {
            for (const auto& edge:proposal.selected_topology) {
                const bool had=std::any_of(runtime.topology.begin(),
                    runtime.topology.end(),[&](const DirectedEdge& old) {
                        return old.id()==edge.id();
                    });
                if (!had) record.switch_additions.push_back(edge.id());
            }
            for (const auto& edge:runtime.topology) {
                const bool kept=std::any_of(
                    proposal.selected_topology.begin(),
                    proposal.selected_topology.end(),
                    [&](const DirectedEdge& fresh) {
                        return fresh.id()==edge.id();
                    });
                if (!kept) record.switch_removals.push_back(edge.id());
            }
            return "switch_certified";
        }
        if (proposal.no_good_rejections>0) return "certifier_exhausted";
        ++state_.solver_failures;
        return "solver_failure";
    }

    static std::vector<DirectedEdge> edgeDifferenceNew(
        const std::vector<DirectedEdge>& a,
        const std::vector<DirectedEdge>& b) {
        std::vector<DirectedEdge> difference;
        for (const auto& edge:a) {
            const bool present=std::any_of(b.begin(),b.end(),
                [&](const DirectedEdge& other){return other.id()==edge.id();});
            if (!present) difference.push_back(edge);
        }
        return difference;
    }

    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    Task11aFrozenConstants constants_;
    Task11aAttributionState state_;
    std::optional<std::size_t> ticks_since_commit_;
};

// Supplementary requirement (researcher, 2026-08-31): annotate every archived
// stop with the 10.11ai limiting-row localization semantics — the failure
// class is reference-row / collision-row / information-eligibility-gate.
struct Task11aFailureClassification {
    bool valid=false;
    std::string fail_reason;
    std::string limiting_row_id;
    std::string failure_class;
    NodeId limiting_owner=0;
    double minimum_owner_local_gamma_mps2=
        std::numeric_limits<double>::quiet_NaN();
};

inline Task11aFailureClassification task11aClassifyStopFrame(
    Task10p11rFixedBaselineFixture& fixture,double tau) {
    Task11aFailureClassification classification;
    try {
        auto frame=task10p11ah_optimizer_detail::nativeFrame(fixture,
            std::nullopt);
        const Eigen::VectorXd native=task10p11sOrderedControls(
            frame.boundary.request.mobile_ids,
            frame.prepared.control.step.applied_controls);
        std::string limiting;
        const double minimum=task10p11af_detail::independentMinimumResidual(
            frame.problem,native,&limiting);
        classification.limiting_row_id=limiting;
        const auto rows=buildCanonicalHardRows(frame.boundary.request);
        classification.limiting_owner=0;
        double worst=std::numeric_limits<double>::infinity();
        for (NodeId owner:frame.boundary.request.mobile_ids) {
            const auto gamma=solveCanonicalGammaStar(rows,owner,
                frame.boundary.request.acceleration_half_box);
            if (gamma.valid&&std::isfinite(gamma.gamma)&&gamma.gamma<worst) {
                worst=gamma.gamma;
                classification.limiting_owner=owner;
            }
        }
        classification.minimum_owner_local_gamma_mps2=worst;
        if (limiting.rfind("collision:",0)==0)
            classification.failure_class="collision_row";
        else if (limiting.rfind("reference:",0)==0)
            classification.failure_class="reference_row";
        else if (worst<0.0)
            classification.failure_class="hard_row";
        else
            classification.failure_class="information_eligibility_gate";
        (void)tau;(void)minimum;
        classification.valid=true;
    } catch (const std::exception& error) {
        classification.fail_reason=error.what();
    }
    return classification;
}

}  // namespace gf
