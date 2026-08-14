#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11SharedFrontierFixture.hpp"

TEST_CASE("Task 10.11 shared allocator request has no topology-strategy label") {
    auto fixture=gf::makeTask10p11Fixture(
        gf::task10p10EasyScenario(),gf::SolverProfile::OpenSource);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto request=fixture->controller.currentPipelineRequest();
    CHECK(request.allocation.agents.size() == 4);
    CHECK_FALSE(request.allocation.cells.empty());
    CHECK(request.allocation.snapshot_token > 0);
}

TEST_CASE("Task 10.11 original collision counterexamples remain viable beyond failure epochs") {
    for (const auto profile : {
             gf::SolverProfile::OpenSource,gf::SolverProfile::Gurobi}) {
        const auto easy=gf::runTask10p11SharedSmoke(
            gf::task10p10EasyScenario(),profile,60.0);
        INFO(easy.reason," t=",easy.runtime_s," coverage=",easy.final_coverage);
        CHECK(easy.completed_horizon);
        CHECK(easy.runtime_s >= 60.0-1e-9);
        CHECK(easy.final_coverage > easy.initial_coverage);
        CHECK(easy.minimum_robust_residual >= -1e-7);

        const auto nonbinding=gf::runTask10p11SharedSmoke(
            gf::task10p10NonbindingScenario(),profile,30.0);
        INFO(nonbinding.reason," t=",nonbinding.runtime_s,
             " coverage=",nonbinding.final_coverage,
             " exhaustion=",static_cast<int>(nonbinding.final_exhaustion),
             " truncated=",nonbinding.search_budget_truncated,
             " bundles=",nonbinding.final_candidate_bundles,
             " fast=",nonbinding.fast_rejections,
             " exact=",nonbinding.exact_rejections,
             " rollout=",nonbinding.rollout_rejections);
        INFO("last rollout reason=",nonbinding.last_rollout_reason);
        INFO("failed rollout cycle=",nonbinding.failed_rollout_cycle,
             " conflict owners=",nonbinding.rollout_conflicts.size());
        std::string conflict_summary;
        for (const auto& certificate : nonbinding.rollout_conflicts) {
            conflict_summary += "owner="+std::to_string(certificate.owner)+":";
            INFO("owner=",certificate.owner," conflict rows=",
                 certificate.minimal_conflict_row_ids.size());
            for (const auto& id : certificate.minimal_conflict_row_ids) {
                conflict_summary += id+",";
                const auto row=std::find_if(
                    certificate.rows.begin(),certificate.rows.end(),
                    [&](const auto& candidate) { return candidate.id==id; });
                if (row!=certificate.rows.end()) {
                    conflict_summary += "n=("+
                        std::to_string(row->normal.x())+","+
                        std::to_string(row->normal.y())+") c="+
                        std::to_string(row->constant)+" h="+
                        std::to_string(row->barrier_h)+" psi1="+
                        std::to_string(row->barrier_psi1)+" rp="+
                        std::to_string(row->position_reserve_m)+" rv="+
                        std::to_string(row->velocity_reserve_mps)+";";
                }
                INFO("conflict row=",id);
            }
        }
        INFO("minimal conflicts=",conflict_summary);
        INFO("rejected rollout traces=",nonbinding.rejected_rollouts.size(),
             " first negative=",nonbinding.first_negative_source,
             " minimum braking slack=",nonbinding.minimum_braking_slack_m);
        for (std::size_t attempt=0;
             attempt<nonbinding.rejected_rollouts.size();++attempt) {
            const auto& rejected=nonbinding.rejected_rollouts[attempt];
            REQUIRE_FALSE(rejected.braking_trace.empty());
            for (const auto& trace : rejected.braking_trace) {
                INFO("attempt=",attempt," cycle=",trace.cycle,
                     " reason=",trace.snapshot.reason,
                     " slack=",trace.snapshot.minimum_braking_slack_m,
                     " source=",trace.snapshot.first_negative_source);
            }
        }
        if (!nonbinding.rollout_conflicts.empty()) {
            CAPTURE(nonbinding.rollout_conflicts.front().owner);
            CAPTURE(nonbinding.rollout_conflicts.front()
                        .minimal_conflict_row_ids);
        }
        CHECK_FALSE(nonbinding.completed_horizon);
        CHECK(nonbinding.reason == "allocator_search_exhausted");
        CHECK(nonbinding.runtime_s == doctest::Approx(20.0));
        CHECK(nonbinding.final_coverage > nonbinding.initial_coverage);
        CHECK(nonbinding.minimum_robust_residual >= -1e-7);
        CHECK(nonbinding.last_rollout_reason ==
            "snapshot_braking_inadmissible");
        CHECK(nonbinding.first_negative_source ==
            "workspace:4:facet:0");
        CHECK(nonbinding.minimum_braking_slack_m < 0.0);
        CHECK(nonbinding.failed_rollout_cycle == 1);
        CHECK(nonbinding.search_budget_truncated);
        CHECK(nonbinding.rollout_conflicts.empty());
        REQUIRE(nonbinding.rejected_rollouts.size() == 4);
        for (const auto& rejected : nonbinding.rejected_rollouts) {
            REQUIRE(rejected.braking_trace.size() == 2);
            CHECK(rejected.braking_trace[0].snapshot.reason ==
                "braking_horizon_insufficient");
            CHECK(rejected.braking_trace[0].snapshot.minimum_braking_slack_m >
                0.0);
            CHECK(rejected.braking_trace[1].snapshot.reason ==
                "snapshot_braking_inadmissible");
            CHECK(rejected.braking_trace[1].snapshot.first_negative_source ==
                "workspace:4:facet:0");
        }
    }
}
