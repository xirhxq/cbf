#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task15ForwardCoveragePolicy.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/Task10p11rFixedBaseline.hpp"

namespace {

gf::FrontierCell task15Cell(int x,int y) {
    return {x,y,{10.0*x+5.0,10.0*y+5.0}};
}

std::vector<gf::FrontierCell> task15Domain() {
    std::vector<gf::FrontierCell> cells;
    cells.reserve(90000);
    for (int x=0;x<300;++x) for (int y=0;y<300;++y)
        cells.push_back(task15Cell(x,y));
    return cells;
}

std::map<gf::NodeId,Eigen::Vector2d> task15Fixed() {
    return {{100,{1200.0,-50.0}},{101,{1500.0,-50.0}},
            {102,{1800.0,-50.0}}};
}

gf::Task15ForwardCoverageRequest task15Request(
    std::vector<gf::FrontierCell> uncovered) {
    gf::Task15ForwardCoverageRequest request;
    request.fixed_positions=task15Fixed();
    request.domain_cells=task15Domain();
    request.uncovered_cells=std::move(uncovered);
    for (int id=1;id<=14;++id)
        request.agents.push_back({static_cast<gf::NodeId>(id),
            {id<=7?1400.0:1600.0,20.0},{0.0,0.0},M_PI/2.0,0.05});
    return request;
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> task15Fixture() {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0,
        true,false,false,false,false,false,false,false,false,false,false,
        false,true,false,false,false,false,false,false,350.0,
        0.0,0.0,2027,4.0,gf::WorkspaceClassK::Linear,
        1.0,1.0,4.0,1.0,false,4.0,1.0,1.5,false,true);
}

}  // namespace

TEST_CASE("Task 15 forward lift matches the CBF2026 worked example") {
    const Eigen::Vector2d base(1500.0,-50.0);
    const Eigen::Vector2d endpoint(1905.0,355.0);
    const auto squad=gf::task13UnifiedCoverageSquads()[0];
    const auto targets=gf::task15ForwardTargets(squad,base,endpoint);

    REQUIRE(targets.size()==7);
    const Eigen::Vector2d section(101.25,101.25);
    CHECK(targets.at(1).isApprox(base+section,1e-12));
    CHECK(targets.at(2).isApprox(
        base+section+Eigen::Rotation2Dd(-M_PI/3.0)*section,1e-12));
    CHECK(targets.at(3).isApprox(base+2.0*section,1e-12));
    CHECK(targets.at(7).isApprox(endpoint,1e-12));
}

TEST_CASE("Task 15 lift is mirror-equivariant and independent of responsible member") {
    const Eigen::Vector2d base(1500.0,-50.0);
    const Eigen::Vector2d endpoint(5.0,2995.0);
    const auto squads=gf::task13UnifiedCoverageSquads();
    const auto a=gf::task15ForwardTargets(squads[0],base,endpoint);
    const auto b=gf::task15ForwardTargets(squads[1],base,
        Eigen::Vector2d(2995.0,2995.0));
    for (std::size_t local=0;local<7;++local) {
        const Eigen::Vector2d reflected(3000.0-a.at(
            squads[0].members[local]).x(),a.at(squads[0].members[local]).y());
        CHECK(b.at(squads[1].members[local]).isApprox(reflected,1e-9));
    }
    const auto geometry=gf::task15EvaluateForwardGeometry(
        squads[0],base,endpoint,
        {{100,{1200.0,-50.0}},{101,base},{102,{1800.0,-50.0}}});
    CHECK(geometry.targets.at(7).isApprox(endpoint,1e-12));
    CHECK(geometry.maximum_reference_edge_m<2000.0);
}

TEST_CASE("Task 15 union footprint preserves a real task ID away from target centers") {
    auto request=task15Request({task15Cell(0,299),task15Cell(1,299),
        task15Cell(2,299)});

    const auto candidate=gf::task15ForwardCandidateForTask(
        gf::task13UnifiedCoverageSquads()[0],request.uncovered_cells[0],
        request);
    REQUIRE(candidate.has_value());
    CHECK(candidate->task.id()=="0:299");
    CHECK(candidate->endpoint.x_index>=0);
    CHECK(candidate->responsible_member>=1);
    CHECK(candidate->responsible_member<=7);
    CHECK(candidate->predicted_new_certified_cells>=1);
    CHECK((candidate->targets.at(candidate->responsible_member)-
           candidate->task.center).norm()>1.0);
    for (const auto& [member,id]:candidate->target_ids) {
        (void)member;
        CHECK(id=="0:299");
    }
}

TEST_CASE("Task 15 allocator handles 0 1 2 and historical three residual tasks") {
    auto three=gf::allocateTask15ForwardCoverage(task15Request({
        task15Cell(0,299),task15Cell(149,299),task15Cell(299,299)}));
    REQUIRE(three.valid);
    CHECK(three.active_squads==2);
    CHECK(three.assignments.at("A").candidate.task.id().find('-')==
        std::string::npos);
    CHECK(three.assignments.at("B").candidate.task.id().find('-')==
        std::string::npos);
    CHECK(three.assignments.at("A").candidate.task.id()!=
        three.assignments.at("B").candidate.task.id());

    auto two_request=task15Request(
        {task15Cell(0,299),task15Cell(299,299)});
    two_request.retained={{"A",three.assignments.at("A").candidate},
                          {"B",three.assignments.at("B").candidate}};
    const auto two=gf::allocateTask15ForwardCoverage(two_request);
    REQUIRE(two.valid);
    CHECK(two.active_squads==2);
    CHECK(two.targets.size()==14);

    auto one_request=task15Request({task15Cell(0,299)});
    one_request.retained=two_request.retained;
    const auto one=gf::allocateTask15ForwardCoverage(one_request);
    REQUIRE(one.valid);
    CHECK(one.active_squads==1);
    for (const auto& [owner,target]:one.targets) {
        (void)owner;
        CHECK(target.id().find('-')==std::string::npos);
    }

    auto zero_request=task15Request({});
    zero_request.retained=one_request.retained;
    const auto zero=gf::allocateTask15ForwardCoverage(zero_request);
    REQUIRE(zero.valid);
    CHECK(zero.active_squads==0);
    CHECK(zero.targets.size()==14);
}

TEST_CASE("Task 15 Level A dominates utility and Level B minimizes overrun") {
    gf::Task15ForwardCoverageCandidate a,b;
    a.level_a=true;
    a.utility=1.0;
    a.task=task15Cell(2,3);
    b.level_a=false;
    b.utility=1000.0;
    b.reference_overrun_m=1.0;
    b.task=task15Cell(1,3);
    CHECK(gf::task15CandidateLess(a,b));
    CHECK_FALSE(gf::task15CandidateLess(b,a));
    a.level_a=false;
    a.reference_overrun_m=5.0;
    b.reference_overrun_m=10.0;
    CHECK(gf::task15CandidateLess(a,b));
}

TEST_CASE("Task 15 persistence uses the registered five percent utility band") {
    gf::Task15ForwardCoverageCandidate retained,proposed;
    retained.level_a=true;
    retained.predicted_new_certified_cells=1;
    retained.utility=95.0;
    proposed.level_a=true;
    proposed.predicted_new_certified_cells=2;
    proposed.utility=100.0;
    CHECK(gf::task15RetainWithinUtilityBand(retained,proposed,0.05));
    retained.utility=94.999;
    CHECK_FALSE(gf::task15RetainWithinUtilityBand(
        retained,proposed,0.05));
    retained.utility=100.0;
    retained.level_a=false;
    CHECK_FALSE(gf::task15RetainWithinUtilityBand(
        retained,proposed,0.05));
    retained.level_a=true;
    retained.predicted_new_certified_cells=0;
    CHECK_FALSE(gf::task15RetainWithinUtilityBand(
        retained,proposed,0.05));
}

TEST_CASE("Task 15 same-tick reselection exhausts the registered shortlist") {
    gf::Task15ForwardCoverageConfig config;
    config.shortlist_capacity=32;
    CHECK(gf::task15ReselectionAttemptLimit(config)==32);
}

TEST_CASE("Task 15 governed target ledger exposes the true all-target separation") {
    std::map<gf::NodeId,Eigen::Vector2d> targets{
        {1,{0.0,0.0}},{2,{6.0,8.0}},{3,{30.0,0.0}}};
    CHECK(gf::task15TargetLedgerMinimumSeparation(targets,{} )==
        doctest::Approx(10.0));
    CHECK(gf::task15TargetLedgerMinimumSeparation(
        targets,{{100,{3.0,4.0}}})==doctest::Approx(5.0));
}

TEST_CASE("Task 15 production integration switch is default disabled") {
    const gf::GrandFinaleSwarmAdapterConfig config;
    CHECK_FALSE(config.target_policy_task15_forward);
    CHECK(config.task15_forward_shortlist_capacity==32);
    CHECK(config.task15_forward_update_period_cycles==10);
}

TEST_CASE("Task 15 controller produces a governed real-ID ledger on first tick") {
    auto fixture=task15Fixture();
    CHECK(fixture->adapter.config().target_policy_task15_forward);
    CHECK_FALSE(fixture->adapter.config().target_policy_unified_h2);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto step=fixture->controller.advance();
    REQUIRE(step.task15_allocation_evaluated);
    REQUIRE(step.task15_allocation.valid);
    CHECK(step.task15_allocation.active_squads==2);
    CHECK(step.target_governor_evaluated);
    CHECK(step.committed_targets.size()==14);
    for (const auto& [owner,target]:step.committed_targets) {
        (void)owner;
        CHECK(target.x_index>=0);
        CHECK(target.y_index>=0);
        CHECK(target.id().find('-')==std::string::npos);
    }
    CHECK(step.step.advanced);
}

TEST_CASE("Task 15 receding governor stays viable through the speed-boundary approach") {
    auto fixture=task15Fixture();
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    for (int tick=0;tick<100;++tick) {
        const auto step=fixture->controller.advance();
        CAPTURE(tick);
        CAPTURE(step.reason);
        CAPTURE(step.step.reason);
        CAPTURE(step.target_governor_common_fraction);
        REQUIRE(step.step.advanced);
        REQUIRE_FALSE(step.target_governor_reselect_required);
    }
}
