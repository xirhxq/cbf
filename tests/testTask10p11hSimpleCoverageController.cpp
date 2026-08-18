#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/D1DevelopmentExperiment.hpp"
#include "grand_finale/ExactHardPolytopeDiagnostic.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

namespace {

struct Trace {
    std::vector<std::vector<std::string>> targets;
    std::vector<std::map<gf::NodeId,Eigen::Vector2d>> controls;
    double initial_coverage=0.0;
    double final_coverage=0.0;
    double minimum_residual=std::numeric_limits<double>::infinity();
    std::string failure;
    double failure_time_s=0.0;
    std::vector<gf::HardPolytopeCertificate> conflicts;
    std::map<gf::NodeId,double> failure_gamma;
    std::map<gf::NodeId,Eigen::Vector4d> failure_estimate;
    std::map<gf::NodeId,gf::FrontierCell> failure_targets;
    gf::SimpleCoveragePhase final_phase=gf::SimpleCoveragePhase::CoverageSearch;
    std::optional<double> t100_coverage_s;
    std::size_t settling_dwell_cycles=0;
    gf::NaturalSettlingAudit last_post_audit;
    double final_runtime_s=0.0;
    std::size_t t100_event_count=0;
    gf::BoundaryExcursionAudit boundary_excursion;
};

gf::BoundaryPolicyConfig noneBoundary() {
    gf::BoundaryPolicyConfig config;
    config.policy=gf::BoundaryPolicy::None;
    return config;
}

Trace run(
    const gf::Task10p10Scenario& scenario,gf::SolverProfile profile,
    gf::D1TopologyStrategy strategy,int cycles) {
    (void)strategy;
    auto fixture=gf::makeTask10p11gFixture(scenario,profile,noneBoundary());
    Trace result;
    const auto stage=fixture->adapter.initializeStageZero();
    if (!stage.initialized) { result.failure=stage.reason; return result; }
    result.initial_coverage=stage.truth_coverage;
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    for (int cycle=0;cycle<cycles;++cycle) {
        if (controller.settled()) break;
        const auto advanced=controller.advance();
        if (advanced.t100_event_latched) ++result.t100_event_count;
        result.boundary_excursion=advanced.boundary_excursion;
        if (!advanced.step.advanced) {
            result.failure=advanced.reason;
            result.failure_time_s=fixture->swarm.robots.front()->runtime;
            result.final_coverage=fixture->adapter.coverage().truthFraction();
            const auto rows=fixture->adapter.currentSnapshotHardRows(
                fixture->adapter.supervisor().topology());
            const auto runtime=fixture->adapter.runtimeSnapshot();
            for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index)
                result.failure_estimate[runtime.estimate.mobile_ids[index]]=
                    runtime.estimate.mean.segment<4>(4*index);
            result.failure_targets=controller.committedTargets();
            result.final_runtime_s=fixture->swarm.robots.front()->runtime;
            result.final_phase=controller.phase();
            result.t100_coverage_s=controller.t100CoverageS();
            result.settling_dwell_cycles=controller.settlingDwellCycles();
            for (const auto owner : scenario.mobile_ids) {
                const auto gamma=gf::solveCanonicalGammaStar(rows,owner,
                    fixture->adapter.config().acceleration_half_box);
                result.failure_gamma[owner]=gamma.gamma;
                auto conflict=gf::diagnoseHardPolytope(owner,
                    result.failure_time_s,fixture->adapter.supervisor().mode(),
                    fixture->adapter.supervisor().topology(),rows,
                    fixture->adapter.config().acceleration_half_box);
                if (!conflict.exact_feasible)
                    result.conflicts.push_back(std::move(conflict));
            }
            return result;
        }
        std::vector<std::string> ids;
        for (const auto& [owner,target] : advanced.committed_targets) {
            (void)owner; ids.push_back(target.id());
        }
        result.targets.push_back(ids);
        result.controls.push_back(advanced.step.applied_controls);
        result.minimum_residual=std::min(
            result.minimum_residual,advanced.step.minimum_hard_residual);
        if (advanced.t100_coverage_s.has_value())
            result.last_post_audit=advanced.settling_audit;
    }
    result.final_coverage=fixture->adapter.coverage().truthFraction();
    result.final_runtime_s=fixture->swarm.robots.front()->runtime;
    result.final_phase=controller.phase();
    result.t100_coverage_s=controller.t100CoverageS();
    result.settling_dwell_cycles=controller.settlingDwellCycles();
    return result;
}

TEST_CASE("T100 latch is statistics-only and centroid fallback is natural") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalLeaderEasyScenario(),
        gf::SolverProfile::OpenSource,noneBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    bool observed=false;
    std::size_t event_epoch=0;
    for (int cycle=0;cycle<300 && !observed;++cycle) {
        const auto step=controller.advance();
        REQUIRE(step.step.advanced);
        if (!step.t100_event_latched) continue;
        observed=true;
        event_epoch=step.target_epoch;
        REQUIRE(step.t100_coverage_s.has_value());
        CHECK(step.phase==gf::SimpleCoveragePhase::CoverageSearch);
        CHECK(controller.phase()==gf::SimpleCoveragePhase::CoverageSearch);
        CHECK(fixture->adapter.runtimeSnapshot().mode==
              gf::SupervisorMode::Search);
        CHECK_FALSE(controller.committedTargets().empty());
        CHECK(controller.committedTargets()==step.committed_targets);
        CHECK(*step.t100_coverage_s==doctest::Approx(
            fixture->swarm.robots.front()->runtime));
        CHECK(fixture->adapter.coverage().truthFraction()==
              doctest::Approx(1.0));
    }
    REQUIRE(observed);
    bool centroid_epoch=false;
    for (int cycle=0;cycle<20 && !centroid_epoch;++cycle) {
        const auto step=controller.advance();
        REQUIRE(step.step.advanced);
        CHECK_FALSE(step.t100_event_latched);
        CHECK(step.phase==gf::SimpleCoveragePhase::CoverageSearch);
        CHECK(fixture->adapter.runtimeSnapshot().mode==
              gf::SupervisorMode::Search);
        if (!step.allocation_evaluated || step.target_epoch==event_epoch)
            continue;
        centroid_epoch=true;
        CHECK(step.allocation.centroid_fallback_leaders==
              std::set<gf::NodeId>{7,14});
        CHECK(step.allocation.reason=="leader_centroid_fallback");
        CHECK(step.committed_targets.size()==14);
    }
    REQUIRE(centroid_epoch);
}

TEST_CASE("Legacy target projection is opt-in and main target digest stays unchanged") {
    auto main_fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalNonbindingScenario(),
        gf::SolverProfile::OpenSource,noneBoundary());
    auto legacy_fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalNonbindingScenario(),
        gf::SolverProfile::OpenSource,noneBoundary());
    REQUIRE(main_fixture->adapter.initializeStageZero().initialized);
    REQUIRE(legacy_fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController main(
        main_fixture->swarm,main_fixture->adapter);
    gf::SimpleCoveragePolicyConfig legacy_config;
    legacy_config.target_projection=
        gf::TargetProjection::LegacySearchPolygonClippingAblation;
    gf::Task10p11hSimpleCoverageController legacy(
        legacy_fixture->swarm,legacy_fixture->adapter,legacy_config);
    const auto main_step=main.advance();
    const auto legacy_step=legacy.advance();
    REQUIRE(main_step.step.advanced);
    REQUIRE(legacy_step.step.advanced);
    CHECK(main_step.allocation.request_digest==
          legacy_step.allocation.request_digest);
    CHECK(main_step.committed_targets.at(1).center.y()<0.0);
    CHECK(legacy_step.committed_targets.at(1).center.y()==
          doctest::Approx(0.0));
}

TEST_CASE("Boundary none leader easy dual-solver finite-horizon stop gate") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        const auto result=run(gf::task10p11hCoastalLeaderEasyScenario(),profile,
            gf::D1TopologyStrategy::FixedDag,1200);
        INFO("profile=",static_cast<int>(profile)," failure=",result.failure,
             " failure_t=",result.failure_time_s," coverage=",
             result.final_coverage," t100=",
             result.t100_coverage_s.value_or(-1.0)," phase=",
             static_cast<int>(result.final_phase)," max_outside=",
             result.boundary_excursion.maximum_outside_distance_m,
             " outside_duration=",
             result.boundary_excursion.any_outside_duration_s,
             " simultaneous=",
             result.boundary_excursion.maximum_simultaneous_outside,
             " max_position=",
             result.boundary_excursion.maximum_position.transpose(),
             " outside_cells=",
             result.boundary_excursion.outside_observer_new_truth_cells);
        for (const auto& conflict : result.conflicts) {
            const std::string profile_name=
                profile==gf::SolverProfile::Gurobi?"gurobi":"open_source";
            MESSAGE("TASK10P11H_NONE_EASY_CONFLICT profile=",profile_name,
                " owner=",conflict.owner," exact=",conflict.exact_feasible,
                " rows=",conflict.minimal_conflict_row_ids.size());
            for (const auto& id : conflict.minimal_conflict_row_ids) {
                const auto row=std::find_if(conflict.rows.begin(),
                    conflict.rows.end(),[&](const auto& value) {
                        return value.id==id;
                    });
                if (row==conflict.rows.end()) {
                    MESSAGE("TASK10P11H_NONE_EASY_ROW id=",id,
                        " kind=input_box");
                    continue;
                }
                MESSAGE("TASK10P11H_NONE_EASY_ROW id=",row->id,
                    " kind=",static_cast<int>(row->kind)," n=",
                    row->normal.transpose()," c=",row->constant,
                    " h=",row->barrier_h," psi1=",row->barrier_psi1,
                    " rp=",row->position_reserve_m,
                    " rv=",row->velocity_reserve_mps,
                    " rc=",row->coefficient_reserve);
            }
            const auto& state=result.failure_estimate.at(conflict.owner);
            MESSAGE("TASK10P11H_NONE_EASY_STATE owner=",conflict.owner,
                " p=",state.head<2>().transpose()," v=",
                state.tail<2>().transpose()," gamma=",
                result.failure_gamma.at(conflict.owner));
        }
        REQUIRE(result.failure.empty());
        REQUIRE(result.t100_coverage_s.has_value());
        CHECK(result.t100_event_count==1);
        CHECK(result.minimum_residual>=-1e-7);
        CHECK(result.final_phase==gf::SimpleCoveragePhase::CoverageSearch);
    }
}

}

TEST_CASE("Controller commits the authoritative leader-only branch ledger") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalNonbindingScenario(),
        gf::SolverProfile::OpenSource,noneBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    const auto advanced=controller.advance();
    REQUIRE(advanced.step.advanced);
    REQUIRE(advanced.committed_targets.size()==14);
    const auto& q7=advanced.committed_targets.at(7).center;
    const Eigen::Vector2d d=(q7-Eigen::Vector2d(250.0,-50.0))/4.0;
    CHECK(advanced.committed_targets.at(1).center.isApprox(
        q7-3.0*d));
    CHECK(advanced.committed_targets.at(2).center.isApprox(
        q7-3.0*d+Eigen::Rotation2Dd(-M_PI/3.0)*d));
    CHECK(advanced.committed_targets.at(7).x_index>=0);
    CHECK(advanced.committed_targets.at(14).x_index>=0);
}

TEST_CASE("Post coverage CVT exposes the unfrozen settling margin without hiding hard failure") {
    const auto result=run(gf::task10p11hCoastalLeaderEasyScenario(),
        gf::SolverProfile::OpenSource,gf::D1TopologyStrategy::FixedDag,1200);
    MESSAGE("TASK10P11H_POST_CVT scenario=coastal_easy profile=open_source",
        " failure=",result.failure," failure_t=",result.failure_time_s,
        " coverage=",result.final_coverage," t100=",
        result.t100_coverage_s.value_or(-1.0)," final_phase=",
        static_cast<int>(result.final_phase)," pos=",
        result.last_post_audit.maximum_position_error_m," speed=",
        result.last_post_audit.maximum_speed_mps," yaw=",
        result.last_post_audit.maximum_yaw_error_rad," residual=",
        result.last_post_audit.minimum_hard_residual," refs=",
        result.last_post_audit.minimum_effective_reference_count," fim=",
        result.last_post_audit.minimum_robust_fim," posterior=",
        result.last_post_audit.maximum_posterior," aoi=",
        result.last_post_audit.minimum_aoi_margin_s," pending=",
        result.last_post_audit.no_pending_transition," dwell=",
        result.settling_dwell_cycles," settled_t=",result.final_runtime_s);
    for (const auto& conflict : result.conflicts) {
        MESSAGE("TASK10P11H_LEADER_EASY_CONFLICT owner=",conflict.owner,
            " rows=",conflict.minimal_conflict_row_ids.size());
        for (const auto& id : conflict.minimal_conflict_row_ids) {
            MESSAGE("TASK10P11H_LEADER_EASY_CONFLICT_ID owner=",
                conflict.owner," id=",id);
            const auto row=std::find_if(conflict.rows.begin(),
                conflict.rows.end(),[&](const auto& value) {
                    return value.id==id;
                });
            if (row!=conflict.rows.end())
                MESSAGE("TASK10P11H_LEADER_EASY_ROW owner=",conflict.owner,
                    " id=",row->id," kind=",static_cast<int>(row->kind),
                    " n=",row->normal.transpose()," c=",row->constant,
                    " h=",row->barrier_h," psi1=",row->barrier_psi1,
                    " rp=",row->position_reserve_m,
                    " rv=",row->velocity_reserve_mps,
                    " rc=",row->coefficient_reserve);
        }
        const auto& state=result.failure_estimate.at(conflict.owner);
        const auto target=result.failure_targets.find(conflict.owner);
        MESSAGE("TASK10P11H_LEADER_EASY_STATE owner=",conflict.owner,
            " p=",state.head<2>().transpose()," v=",
            state.tail<2>().transpose()," gamma=",
            result.failure_gamma.at(conflict.owner)," target=",
            target==result.failure_targets.end()
                ?std::string("none"):target->second.id());
    }
    REQUIRE(result.failure.empty());
    REQUIRE(result.t100_coverage_s.has_value());
    CHECK(result.final_phase==gf::SimpleCoveragePhase::CoverageSearch);
}

TEST_CASE("Post coverage CVT reports nonbinding settling margin") {
    const auto result=run(gf::task10p11hCoastalNonbindingScenario(),
        gf::SolverProfile::OpenSource,gf::D1TopologyStrategy::FixedDag,1200);
    MESSAGE("TASK10P11H_POST_CVT scenario=coastal_nonbinding profile=open_source",
        " failure=",result.failure," t100=",
        result.t100_coverage_s.value_or(-1.0)," final_phase=",
        static_cast<int>(result.final_phase)," pos=",
        result.last_post_audit.maximum_position_error_m," speed=",
        result.last_post_audit.maximum_speed_mps," yaw=",
        result.last_post_audit.maximum_yaw_error_rad," residual=",
        result.last_post_audit.minimum_hard_residual," refs=",
        result.last_post_audit.minimum_effective_reference_count," fim=",
        result.last_post_audit.minimum_robust_fim," posterior=",
        result.last_post_audit.maximum_posterior," aoi=",
        result.last_post_audit.minimum_aoi_margin_s," pending=",
        result.last_post_audit.no_pending_transition," dwell=",
        result.settling_dwell_cycles," settled_t=",result.final_runtime_s);
    REQUIRE(result.failure.empty());
    REQUIRE(result.t100_coverage_s.has_value());
}

TEST_CASE("Simple coastal easy policy is identical across topology strategy labels") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        const auto fixed=run(gf::task10p11hCoastalLeaderEasyScenario(),profile,
            gf::D1TopologyStrategy::FixedDag,1200);
        REQUIRE(fixed.failure.empty());
        REQUIRE(fixed.t100_coverage_s.has_value());
        CHECK(fixed.final_phase==gf::SimpleCoveragePhase::CoverageSearch);
        MESSAGE("TASK10P11H_STAGE_A scenario=coastal_easy profile=",
            static_cast<int>(profile)," t100=",*fixed.t100_coverage_s,
            " settled_t=",fixed.final_runtime_s," residual=",
            fixed.minimum_residual);
        CHECK(fixed.final_coverage>fixed.initial_coverage);
        CHECK(fixed.minimum_residual>=-1e-7);
        for (const auto strategy : {gf::D1TopologyStrategy::ProposedCertified,
                                    gf::D1TopologyStrategy::GreedyCertified}) {
            const auto other=run(gf::task10p11hCoastalLeaderEasyScenario(),profile,
                strategy,1200);
            REQUIRE(other.failure.empty());
            CHECK(other.t100_coverage_s==fixed.t100_coverage_s);
            CHECK(other.final_phase==gf::SimpleCoveragePhase::CoverageSearch);
            CHECK(other.targets==fixed.targets);
            REQUIRE(other.controls.size()==fixed.controls.size());
            for (std::size_t cycle=0;cycle<fixed.controls.size();++cycle)
                for (const auto& [owner,control] : fixed.controls[cycle])
                    CHECK((other.controls[cycle].at(owner)-control).norm()<=1e-10);
        }
    }
}

TEST_CASE("Simple coastal nonbinding advances beyond multiple logical planner ticks") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        const auto result=run(gf::task10p11hCoastalNonbindingScenario(),profile,
            gf::D1TopologyStrategy::FixedDag,1200);
        INFO("profile=",static_cast<int>(profile)," failure=",result.failure,
             " t=",result.failure_time_s," initial=",result.initial_coverage,
             " final=",result.final_coverage," residual=",result.minimum_residual,
             " t100=",result.t100_coverage_s.value_or(-1.0)," runtime=",
             result.final_runtime_s," pos=",
             result.last_post_audit.maximum_position_error_m," speed=",
             result.last_post_audit.maximum_speed_mps," yaw=",
             result.last_post_audit.maximum_yaw_error_rad," post_residual=",
             result.last_post_audit.minimum_hard_residual," refs=",
             result.last_post_audit.minimum_effective_reference_count," fim=",
             result.last_post_audit.minimum_robust_fim," posterior=",
             result.last_post_audit.maximum_posterior," aoi=",
             result.last_post_audit.minimum_aoi_margin_s," pending=",
             result.last_post_audit.no_pending_transition," dwell=",
             result.settling_dwell_cycles);
        for (const auto& conflict : result.conflicts) {
            INFO("owner=",conflict.owner," conflict_count=",
                 conflict.minimal_conflict_row_ids.size());
            for (const auto& id : conflict.minimal_conflict_row_ids) {
                const auto row=std::find_if(conflict.rows.begin(),
                    conflict.rows.end(),[&](const auto& value) {
                        return value.id==id;
                    });
                if (row!=conflict.rows.end())
                    MESSAGE("TASK10P11H_CONFLICT owner=",conflict.owner,
                         " row=",row->id," kind=",static_cast<int>(row->kind),
                         " n=",row->normal.transpose()," c=",row->constant,
                         " h=",row->barrier_h," psi1=",row->barrier_psi1,
                         " rp=",row->position_reserve_m,
                         " rv=",row->velocity_reserve_mps,
                         " rc=",row->coefficient_reserve);
            }
        }
        for (const auto& entry : result.failure_gamma)
            MESSAGE("TASK10P11H_FAILURE_GAMMA owner=",entry.first,
                    " gamma=",entry.second);
        for (const auto& conflict : result.conflicts) {
            const auto& state=result.failure_estimate.at(conflict.owner);
            const auto target=result.failure_targets.find(conflict.owner);
            MESSAGE("TASK10P11H_FAILURE_STATE owner=",conflict.owner,
                    " p=",state.head<2>().transpose()," v=",
                    state.tail<2>().transpose()," target=",
                    target==result.failure_targets.end()
                        ? std::string("none") : target->second.id());
        }
        REQUIRE(result.failure.empty());
        REQUIRE(result.t100_coverage_s.has_value());
        CHECK(result.final_phase==gf::SimpleCoveragePhase::CoverageSearch);
        MESSAGE("TASK10P11H_STAGE_A scenario=coastal_nonbinding profile=",
            static_cast<int>(profile)," t100=",*result.t100_coverage_s,
            " settled_t=",result.final_runtime_s," residual=",
            result.minimum_residual);
        CHECK(result.final_coverage>result.initial_coverage);
        CHECK(result.minimum_residual>=-1e-7);
    }
}

TEST_CASE("HOLD mode ignores committed coverage target and retains yaw") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalLeaderEasyScenario(),
        gf::SolverProfile::OpenSource,noneBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    REQUIRE(controller.advance().step.advanced);
    const auto epoch=controller.targetEpoch();
    std::vector<double> yaw;
    for (const auto& robot : fixture->swarm.robots)
        yaw.push_back(robot->model->getStateVariable("yawRad"));
    REQUIRE(fixture->adapter.supervisor().requestReformation(
        fixture->swarm.robots.front()->runtime,false,false)==
        gf::SupervisorMode::Hold);
    const auto hold=controller.advance();
    REQUIRE(hold.step.advanced);
    CHECK(hold.step.mode==gf::SupervisorMode::Hold);
    CHECK(hold.step.applied_yaw_rates_radps.empty());
    CHECK(controller.targetEpoch()==epoch);
    for (std::size_t index=0;index<yaw.size();++index)
        CHECK(fixture->swarm.robots[index]->model->getStateVariable("yawRad")==
              doctest::Approx(yaw[index]));
}

TEST_CASE("Coastal binding uses formal mobile relay proposal and a fresh UNION cycle") {
    const auto scenario=gf::task10p11hCoastalBindingActiveScenario();
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        for (const auto strategy : {gf::D1TopologyStrategy::FixedDag,
                 gf::D1TopologyStrategy::ProposedCertified,
                 gf::D1TopologyStrategy::GreedyCertified}) {
            auto fixture=gf::makeTask10p11gFixture(
                scenario,profile,noneBoundary());
            REQUIRE(fixture->adapter.initializeStageZero().initialized);
            gf::Task10p11hSimpleCoverageController controller(
                fixture->swarm,fixture->adapter);
            REQUIRE(controller.advance().step.advanced);
            REQUIRE(controller.advance().step.advanced);
            const auto old_rows=fixture->adapter.currentSnapshotHardRows(
                fixture->adapter.supervisor().topology());
            double old_h=std::numeric_limits<double>::infinity();
            for (const auto& row : old_rows)
                if (row.kind==gf::CanonicalHardRowKind::ReferenceDistance)
                    old_h=std::min(old_h,row.barrier_h);
            CHECK(old_h>0.0);
            CHECK(old_h<100.0);
            const auto candidates=gf::task10p11hCoastalRelayCandidates(
                fixture->adapter);
            REQUIRE(candidates.size()>1);
            CHECK(std::any_of(candidates.begin(),candidates.end(),
                [](const auto& edge) { return edge.id()=="13->14"; }));

            if (strategy==gf::D1TopologyStrategy::FixedDag) {
                const auto step=controller.advance();
                REQUIRE(step.step.advanced);
                CHECK(fixture->adapter.supervisor().mode()==
                      gf::SupervisorMode::Search);
            } else if (strategy==gf::D1TopologyStrategy::ProposedCertified) {
                const auto proposal=fixture->adapter.proposeAndBegin(
                    gf::task10p11hCoastalRelayProposal(fixture->adapter));
                INFO(proposal.reason);
                REQUIRE(proposal.transition_started);
                CHECK(std::any_of(proposal.selected_topology.begin(),
                    proposal.selected_topology.end(),[](const auto& edge) {
                        return edge.id()=="13->14";
                    }));
            } else {
                std::vector<gf::D1ReplacementCandidate> choices;
                for (const auto& edge : candidates)
                    choices.push_back({edge,{11,14},
                        edge.id()=="13->14"?1.0:0.0});
                const auto selected=gf::chooseD1Replacement(strategy,choices);
                REQUIRE(selected.has_value());
                REQUIRE(selected->addition.id()=="13->14");
                REQUIRE(fixture->adapter.beginReplacement(
                    selected->addition,selected->removal));
            }
            if (strategy!=gf::D1TopologyStrategy::FixedDag) {
                const auto union_step=controller.advance();
                REQUIRE(union_step.step.advanced);
                CHECK(union_step.step.mode==gf::SupervisorMode::Union);
                CHECK(union_step.step.minimum_hard_residual>=
                      -fixture->adapter.config().residual_tolerance);
                REQUIRE(fixture->adapter.finishReplacementAfterFreshCycle());
                const auto successor=fixture->adapter.supervisor().topology();
                CHECK(std::any_of(successor.begin(),successor.end(),
                    [](const auto& edge) { return edge.id()=="13->14"; }));
                CHECK_FALSE(std::any_of(successor.begin(),successor.end(),
                    [](const auto& edge) { return edge.id()=="11->14"; }));
            }
            double minimum_residual=std::numeric_limits<double>::infinity();
            gf::NaturalSettlingAudit last_post_audit;
            for (int cycle=0;cycle<2400 && !controller.settled();++cycle) {
                const auto advanced=controller.advance();
                INFO("profile=",static_cast<int>(profile)," strategy=",
                    static_cast<int>(strategy)," cycle=",cycle,
                    " phase=",static_cast<int>(controller.phase()),
                    " coverage=",fixture->adapter.coverage().truthFraction(),
                    " failure=",advanced.reason);
                REQUIRE(advanced.step.advanced);
                minimum_residual=std::min(minimum_residual,
                    advanced.step.minimum_hard_residual);
                if (advanced.t100_coverage_s.has_value())
                    last_post_audit=advanced.settling_audit;
            }
            REQUIRE(controller.t100CoverageS().has_value());
            INFO("t100=",*controller.t100CoverageS()," runtime=",
                fixture->swarm.robots.front()->runtime," pos=",
                last_post_audit.maximum_position_error_m," speed=",
                last_post_audit.maximum_speed_mps," yaw=",
                last_post_audit.maximum_yaw_error_rad," residual=",
                last_post_audit.minimum_hard_residual," refs=",
                last_post_audit.minimum_effective_reference_count," fim=",
                last_post_audit.minimum_robust_fim," posterior=",
                last_post_audit.maximum_posterior," aoi=",
                last_post_audit.minimum_aoi_margin_s," pending=",
                last_post_audit.no_pending_transition," dwell=",
                controller.settlingDwellCycles());
            REQUIRE(controller.settled());
            CHECK(minimum_residual>=
                  -fixture->adapter.config().residual_tolerance);
            MESSAGE("TASK10P11H_STAGE_A scenario=coastal_binding_active profile=",
                static_cast<int>(profile)," strategy=",static_cast<int>(strategy),
                " t100=",*controller.t100CoverageS()," settled_t=",
                fixture->swarm.robots.front()->runtime," residual=",
                minimum_residual);
        }
    }
}
