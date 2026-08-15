#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11SharedFrontierFixture.hpp"

#include <iostream>

TEST_CASE("Task 10.11c easy and nonbinding replays compare frozen diagnostic modes") {
    std::vector<std::pair<std::string,gf::Task10p11ComputeProfile>> profiles_out;
    const auto check_work=[](
        const gf::Task10p11SmokeResult& result,std::size_t mobile_count) {
        const std::size_t candidates_per_owner=9;
        CHECK(result.rollout_gamma_work.policy_evaluations==
            result.policy_rollout_cycles);
        CHECK(result.rollout_gamma_work.exact_gamma_solves==
            mobile_count*(candidates_per_owner+1)*
                result.policy_rollout_cycles);
        CHECK(result.rollout_gamma_work.canonical_row_rebuilds==
            (1+mobile_count*candidates_per_owner)*
                result.policy_rollout_cycles);
        CHECK(result.rollout_qp_solves==
            mobile_count*result.policy_rollout_cycles);
        CHECK(result.policy_wall_median_s<=result.policy_wall_p95_s);
        CHECK(result.policy_wall_p95_s<=result.policy_wall_maximum_s);
        CHECK(result.epoch_wall_median_s<=result.epoch_wall_p95_s);
        CHECK(result.epoch_wall_p95_s<=result.epoch_wall_maximum_s);
    };
    const std::vector<gf::SolverProfile> profiles{
        gf::SolverProfile::OpenSource,
#ifdef ENABLE_GUROBI
        gf::SolverProfile::Gurobi
#endif
    };
    for (const auto profile:profiles) {
        for (const auto mode:{
                gf::GammaFeedbackSelectionMode::DiagnosticsOnly,
                gf::GammaFeedbackSelectionMode::MaximumPredictedMargin}) {
            const auto easy=gf::runTask10p11SharedSmoke(
                gf::task10p10EasyScenario(),profile,60.0,mode);
            INFO("easy profile=",static_cast<int>(profile),
                 " mode=",static_cast<int>(mode)," reason=",easy.reason,
                 " t=",easy.runtime_s," gamma=[",easy.minimum_current_gamma,
                 ",",easy.maximum_current_gamma,"] uplift=",
                 easy.maximum_predicted_gamma_uplift," intervention=",
                 easy.first_feedback_intervention_s);
            std::cout<<"TASK10P11C_REPLAY scenario=easy profile="
                <<static_cast<int>(profile)<<" mode="<<static_cast<int>(mode)
                <<" reason="<<easy.reason<<" t="<<easy.runtime_s
                <<" coverage="<<easy.final_coverage
                <<" gamma_min="<<easy.minimum_current_gamma
                <<" gamma_max="<<easy.maximum_current_gamma
                <<" uplift="<<easy.maximum_predicted_gamma_uplift
                <<" first_intervention="<<easy.first_feedback_intervention_s
                <<" braking="<<easy.minimum_braking_slack_m
                <<" rollout_reason="<<easy.last_rollout_reason
                <<" first_negative="<<easy.first_negative_source
                <<" bundles="<<easy.candidate_bundles
                <<" rollout_cycles="<<easy.policy_rollout_cycles
                <<" gamma_solves="<<easy.rollout_gamma_work.exact_gamma_solves
                <<" row_rebuilds="
                <<easy.rollout_gamma_work.canonical_row_rebuilds
                <<" qp_solves="<<easy.rollout_qp_solves
                <<" wall_median="<<easy.policy_wall_median_s
                <<" wall_p95="<<easy.policy_wall_p95_s
                <<" wall_max="<<easy.policy_wall_maximum_s
                <<" epoch_median="<<easy.epoch_wall_median_s
                <<" epoch_p95="<<easy.epoch_wall_p95_s
                <<" epoch_max="<<easy.epoch_wall_maximum_s<<'\n';
            CHECK(easy.minimum_robust_residual>=-1e-7);
            CHECK(std::isfinite(easy.minimum_current_gamma));
            CHECK(easy.maximum_predicted_gamma_uplift>=-1e-12);
            check_work(easy,gf::task10p10EasyScenario().mobile_ids.size());
            profiles_out.push_back({
                "easy:"+std::to_string(static_cast<int>(profile))+":"+
                    std::to_string(static_cast<int>(mode)),easy.compute_profile});
            CHECK(easy.epoch_wall_maximum_s<=0.5);
            if (mode==gf::GammaFeedbackSelectionMode::DiagnosticsOnly) {
                CHECK(easy.completed_horizon);
                CHECK(easy.feedback_interventions==0);
            } else {
                CHECK(easy.feedback_interventions>0);
                const bool classified = easy.reason=="completed_horizon" ||
                    easy.reason=="allocator_search_exhausted" ||
                    easy.reason=="hard_polytope_empty";
                CHECK(classified);
            }

            const auto nonbinding=gf::runTask10p11SharedSmoke(
                gf::task10p10NonbindingScenario(),profile,30.0,mode);
            INFO("nonbinding profile=",static_cast<int>(profile),
                 " mode=",static_cast<int>(mode)," reason=",nonbinding.reason,
                 " t=",nonbinding.runtime_s," gamma=[",
                 nonbinding.minimum_current_gamma,",",
                 nonbinding.maximum_current_gamma,"] uplift=",
                 nonbinding.maximum_predicted_gamma_uplift," intervention=",
                 nonbinding.first_feedback_intervention_s,
                 " braking=",nonbinding.minimum_braking_slack_m);
            std::cout<<"TASK10P11C_REPLAY scenario=nonbinding profile="
                <<static_cast<int>(profile)<<" mode="<<static_cast<int>(mode)
                <<" reason="<<nonbinding.reason<<" t="<<nonbinding.runtime_s
                <<" coverage="<<nonbinding.final_coverage
                <<" gamma_min="<<nonbinding.minimum_current_gamma
                <<" gamma_max="<<nonbinding.maximum_current_gamma
                <<" uplift="<<nonbinding.maximum_predicted_gamma_uplift
                <<" first_intervention="
                <<nonbinding.first_feedback_intervention_s
                <<" braking="<<nonbinding.minimum_braking_slack_m
                <<" dominant="<<nonbinding.first_feedback_dominant_row
                <<" rollout_reason="<<nonbinding.last_rollout_reason
                <<" first_negative="<<nonbinding.first_negative_source
                <<" first_negative_t="
                <<nonbinding.first_negative_braking_prediction_s
                <<" bundles="<<nonbinding.candidate_bundles
                <<" rollout_cycles="<<nonbinding.policy_rollout_cycles
                <<" gamma_solves="
                <<nonbinding.rollout_gamma_work.exact_gamma_solves
                <<" row_rebuilds="
                <<nonbinding.rollout_gamma_work.canonical_row_rebuilds
                <<" qp_solves="<<nonbinding.rollout_qp_solves
                <<" wall_median="<<nonbinding.policy_wall_median_s
                <<" wall_p95="<<nonbinding.policy_wall_p95_s
                <<" wall_max="<<nonbinding.policy_wall_maximum_s
                <<" epoch_median="<<nonbinding.epoch_wall_median_s
                <<" epoch_p95="<<nonbinding.epoch_wall_p95_s
                <<" epoch_max="<<nonbinding.epoch_wall_maximum_s<<'\n';
            CHECK(nonbinding.minimum_robust_residual>=-1e-7);
            CHECK(std::isfinite(nonbinding.minimum_current_gamma));
            CHECK(nonbinding.final_coverage>nonbinding.initial_coverage);
            CHECK(nonbinding.maximum_predicted_gamma_uplift>=-1e-12);
            check_work(nonbinding,
                gf::task10p10NonbindingScenario().mobile_ids.size());
            profiles_out.push_back({
                "nonbinding:"+std::to_string(static_cast<int>(profile))+":"+
                    std::to_string(static_cast<int>(mode)),
                nonbinding.compute_profile});
            CHECK(nonbinding.epoch_wall_maximum_s<=0.5);
            REQUIRE_FALSE(nonbinding.gamma_trace.empty());
            for (const auto& trace:nonbinding.gamma_trace) {
                CHECK(std::isfinite(trace.minimum_current_gamma));
                CHECK(std::isfinite(trace.minimum_nominal_predicted_gamma));
                CHECK(std::isfinite(
                    trace.minimum_maximum_candidate_predicted_gamma));
                CHECK(std::isfinite(trace.minimum_selected_predicted_gamma));
            }
            if (mode==gf::GammaFeedbackSelectionMode::DiagnosticsOnly)
                CHECK(nonbinding.feedback_interventions==0);
            else {
                CHECK(nonbinding.feedback_interventions>0);
                if (std::isfinite(
                        nonbinding.first_negative_braking_prediction_s)) {
                    CHECK(nonbinding.first_feedback_intervention_s<
                          nonbinding.first_negative_braking_prediction_s);
                }
            }
        }
    }
    for (const auto& [label,compute] : profiles_out) {
        for (const auto phase:{
                gf::Task10p11ComputePhase::CandidateBundleConstruction,
                gf::Task10p11ComputePhase::EstimatorPropagation,
                gf::Task10p11ComputePhase::CanonicalRowRebuild,
                gf::Task10p11ComputePhase::ExactHardProjection,
                gf::Task10p11ComputePhase::CurrentGamma,
                gf::Task10p11ComputePhase::PredictedGamma,
                gf::Task10p11ComputePhase::SolverInitialization,
                gf::Task10p11ComputePhase::SolverModelUpdate,
                gf::Task10p11ComputePhase::RobustQpSolve,
                gf::Task10p11ComputePhase::ResidualTokenAudit,
                gf::Task10p11ComputePhase::DiagnosticSerialization}) {
            const auto summary=compute.summary(phase);
            std::cout<<"TASK10P11E_PROFILE label="<<label
                <<" phase="<<gf::task10p11ComputePhaseName(phase)
                <<" calls="<<summary.calls<<" total="<<summary.total_s
                <<" median="<<summary.median_s<<" p95="<<summary.p95_s
                <<" max="<<summary.maximum_s<<'\n';
        }
    }
}
