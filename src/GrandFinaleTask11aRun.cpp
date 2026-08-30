#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>

namespace {

using json=nlohmann::json;

json topologyIds(const std::vector<gf::DirectedEdge>& edges) {
    json ids=json::array();
    for (const auto& edge:edges) ids.push_back(edge.id());
    return ids;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=5&&argc!=6) {
        std::cerr<<"usage: GrandFinaleTask11aRun TAU PROPOSALS OUTPUT_JSON "
            "PROGRESS_DIR [WINDOW_S]\n";
        return 2;
    }
    const auto started=std::chrono::steady_clock::now();
    const auto elapsed=[&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
    };
    try {
        const double tau=std::stod(argv[1]);
        const bool proposals=std::string(argv[2])=="on";
        const double window_s=argc==6?std::stod(argv[5]):500.0;
        const std::filesystem::path progress_directory=argv[4];
        std::filesystem::create_directories(progress_directory);
        auto constants=gf::task11aFrozenConstants();
        constants.proposals_enabled=proposals;
        auto fixture=gf::makeTask10p11rFixedBaselineFixture(
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau);
        if (!fixture->adapter.initializeStageZero().initialized)
            throw std::runtime_error("stage-zero initialization failed");
        gf::Task11aDynamicTopologyCoordinator coordinator(
            fixture->swarm,fixture->adapter,constants);
        json record={{"protocol","task11a-full-course-run-v1"},
            {"preregistration","task-11a-approved-2026-08-31"},
            {"tau_mps2",tau},{"proposals_enabled",proposals},
            {"window_s",window_s},
            {"frozen_constants",{{"evaluation_period_ticks",
                constants.evaluation_period_ticks},
                {"minimum_dwell_ticks",constants.minimum_dwell_ticks},
                {"minimum_indegree",constants.minimum_indegree},
                {"maximum_indegree",constants.maximum_indegree}}},
            {"identity_t0",{{"topology_matches_fixed_baseline",
                fixture->topologyFrozen()},
                {"topology",topologyIds(fixture->adapter.runtimeSnapshot().
                    topology)}}},
            {"complete",false}};
        auto emit=[&](const std::string& name,const json& payload) {
            gf::writeTask10p11vJson(progress_directory/name,payload);
        };
        emit("00-config.json",record);
        auto coverage_fraction=[&]() {
            return fixture->adapter.coverage().truthFraction();
        };
        std::optional<std::size_t> t95_tick,t100_tick;
        json events=json::array();
        std::size_t tick=0;
        bool hard_stop=false;
        gf::SimpleCoverageControlStep last_step;
        while (fixture->adapter.runtimeSnapshot().runtime_s<window_s) {
            const auto evaluation=coordinator.maybeEvaluateAndPropose(tick);
            if (!evaluation.kind.empty()&&evaluation.kind!="disabled"&&
                !evaluation.outcome.empty())
                events.push_back({{"tick",tick},{"kind",evaluation.kind},
                    {"outcome",evaluation.outcome},
                    {"no_good_rejections",evaluation.no_good_rejections},
                    {"rejection_reasons",evaluation.rejection_reasons},
                    {"switch_additions",evaluation.switch_additions},
                    {"switch_removals",evaluation.switch_removals}});
            last_step=fixture->controller.advance();
            const bool switch_completed=
                coordinator.recordCycle(last_step);
            if (switch_completed)
                events.push_back({{"tick",tick},
                    {"outcome","fresh_successor_committed"}});
            const double fraction=coverage_fraction();
            if (!t95_tick.has_value()&&fraction>=0.95)
                t95_tick=tick;
            if (!t100_tick.has_value()&&fraction>=1.0-1e-12)
                t100_tick=tick;
            if (tick%100==0||!last_step.step.advanced) {
                const auto runtime=fixture->adapter.runtimeSnapshot();
                events.push_back({{"tick",tick},{"kind","cycle"},
                    {"advanced",last_step.step.advanced},
                    {"reason",last_step.step.reason},
                    {"mode",static_cast<int>(last_step.step.mode)},
                    {"coverage_fraction",fraction},
                    {"minimum_hard_residual",
                        gf::task10p11w_detail::number(
                            last_step.step.minimum_hard_residual)},
                    {"topology_version",runtime.topology_token}});
            }
            if (!last_step.step.advanced) {
                hard_stop=true;
                break;
            }
            ++tick;
        }
        record["t95_tick"]=(t95_tick.has_value()?
            json(*t95_tick):json(nullptr));
        record["t100_tick"]=(t100_tick.has_value()?
            json(*t100_tick):json(nullptr));
        record["final_coverage_fraction"]=coverage_fraction();
        record["cycles"]=tick;
        record["hard_stop"]=hard_stop;
        record["failure"]={{"advanced",last_step.step.advanced},
            {"reason",last_step.step.reason},
            {"same_reason_as_fixed_132p4",last_step.step.reason==
                "current_gamma_negative"},
            {"minimum_hard_residual",gf::task10p11w_detail::number(
                last_step.step.minimum_hard_residual)}};
        // Prereg section 11 attribution: post-hoc probe on the stopped
        // boundary state distinguishes "mechanism ineffective" (no certified
        // alternative even when authorised) from "frozen constants locked the
        // mechanism" (a certified alternative existed but the trigger/dwell
        // semantics had not authorised a switch in time).
        if (hard_stop) {
          try {
            gf::Task11aFrozenConstants probe_constants=constants;
            probe_constants.proposals_enabled=true;
            gf::Task11aDynamicTopologyCoordinator probe(
                fixture->swarm,fixture->adapter,probe_constants);
            const std::size_t probe_tick=
                coordinator.attribution().tick+1;
            const auto probe_record=probe.forceEvaluate(probe_tick);
            const auto checkpoint=gf::makeTask10p11vSparseRestartCheckpoint(
                *fixture,std::nullopt,
                fixture->controller.successfulControlCycles());
            gf::writeTask10p11vJson(
                progress_directory/"failure-sparse-checkpoint.json",
                checkpoint);
            const std::size_t unhandled=
                coordinator.attribution().last_immediate_trigger_condition_met
                ?1u:0u;
            json failure_attribution={
                {"failure_reason",last_step.step.reason},
                {"same_mode_as_fixed_132p4",last_step.step.reason==
                    "current_gamma_negative"},
                {"historical_132p4_reference",{{"owner",2},
                    {"rule","fixed reference 101->2 + mobile reference 2->4"
                        " + input box; current_gamma_negative"}}},
                {"attribution_state",{{"ticks_since_last_evaluation",
                    coordinator.attribution().ticks_since_last_evaluation},
                    {"ticks_since_last_commit",
                    coordinator.attribution().ticks_since_last_commit},
                    {"dwell_blocked_evaluations",
                    coordinator.attribution().dwell_blocked_evaluations},
                    {"immediate_triggers",
                    coordinator.attribution().immediate_triggers},
                    {"immediate_triggers_unhandled_at_failure",unhandled},
                    {"evaluations",coordinator.attribution().evaluations},
                    {"certified_switches",
                    coordinator.attribution().certified_switches}}},
                {"probe",{{"outcome",probe_record.outcome},
                    {"kind",probe_record.kind},
                    {"no_good_rejections",probe_record.no_good_rejections},
                    {"rejection_reasons",probe_record.rejection_reasons},
                    {"mechanism_verdict",
                        probe_record.outcome=="switch_certified"?
                            "constants_locked_mechanism":
                            (probe_record.outcome=="no_feasible_assignment"?
                                "no_certified_alternative":"mechanism_"
                                "ineffective_on_this_state")}}}};
            record["failure_attribution"]=failure_attribution;
            emit("99-failure-attribution.json",failure_attribution);
          } catch (const std::exception& packaging_error) {
              record["failure_attribution_packaging_error"]=
                  packaging_error.what();
              gf::writeTask10p11vJson(argv[3],record);
          }
        }
        // Supplementary requirement (2026-08-31): annotate every archived
        // stop — any hard stop, or any stop after crossing 132.4 s — with the
        // 10.11ai limiting-row failure class.
        const bool crossed_132p4=fixture->adapter.runtimeSnapshot().
            runtime_s>=132.4;
        if (hard_stop||crossed_132p4) {
          try {
            const auto classification=gf::task11aClassifyStopFrame(*fixture,
                tau);
            record["stop_frame_classification"]={{"annotated",true},
                {"hard_stop",hard_stop},{"crossed_132p4",crossed_132p4},
                {"valid",classification.valid},
                {"fail_reason",classification.fail_reason},
                {"failure_class",classification.failure_class},
                {"limiting_row_id",classification.limiting_row_id},
                {"limiting_owner",classification.limiting_owner},
                {"minimum_owner_local_gamma_mps2",
                    gf::task10p11w_detail::number(
                        classification.minimum_owner_local_gamma_mps2)}};
            emit("98-stop-frame-classification.json",
                record["stop_frame_classification"]);
          } catch (const std::exception& classification_error) {
              record["stop_classification_packaging_error"]=
                  classification_error.what();
          }
        }
        record["evaluations"]=events;
        // Primary result is written before the archival extras so an
        // infrastructure failure there cannot destroy the science record.
        gf::writeTask10p11vJson(argv[3],record);
        std::cout<<record.dump(2)<<'\n';
        record["attribution_final"]={{"evaluations",
            coordinator.attribution().evaluations},
            {"solver_optimal_no_change",
            coordinator.attribution().solver_optimal_no_change},
            {"solver_no_feasible_assignment",
            coordinator.attribution().solver_no_feasible_assignment},
            {"certified_switches",coordinator.attribution().certified_switches},
            {"dwell_blocked_evaluations",
            coordinator.attribution().dwell_blocked_evaluations},
            {"immediate_triggers",coordinator.attribution().immediate_triggers}};
        record["complete"]=true;
        record["wall_time_s"]=elapsed();
        record["claim_boundary"]={{"single_deterministic_development_scenario",
            true},{"safety_completion_claim_requires_c1_t100",true},
            {"c2_t95_efficiency_exploratory_only",true},
            {"recursive_feasibility_claimed",false},
            {"production_controller_claimed",false}};
        gf::writeTask10p11vJson(argv[3],record);
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 11a run failed: "<<error.what()<<'\n';
        return 4;
    }
}
