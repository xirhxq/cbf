#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11SharedFrontierFixture.hpp"

#include <iostream>

TEST_CASE("Task 10.11c easy and nonbinding replays compare frozen diagnostic modes") {
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
                <<" first_negative="<<easy.first_negative_source<<'\n';
            CHECK(easy.minimum_robust_residual>=-1e-7);
            CHECK(std::isfinite(easy.minimum_current_gamma));
            CHECK(easy.maximum_predicted_gamma_uplift>=-1e-12);
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
                <<nonbinding.first_negative_braking_prediction_s<<'\n';
            CHECK(nonbinding.minimum_robust_residual>=-1e-7);
            CHECK(std::isfinite(nonbinding.minimum_current_gamma));
            CHECK(nonbinding.final_coverage>nonbinding.initial_coverage);
            CHECK(nonbinding.maximum_predicted_gamma_uplift>=-1e-12);
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
}
