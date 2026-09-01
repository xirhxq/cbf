#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task13UnifiedCoveragePolicy.hpp"
#include "grand_finale/Task10p11rFixedBaseline.hpp"

namespace {

gf::FrontierCell cell(int x,int y) {
    return {x,y,{10.0*x+5.0,10.0*y+5.0}};
}

std::map<gf::NodeId,Eigen::Vector2d> fixed() {
    return {{100,{1200.0,-50.0}},{101,{1500.0,-50.0}},
            {102,{1800.0,-50.0}}};
}

gf::Task13UnifiedCoverageRequest request(
    const std::vector<gf::FrontierCell>& uncovered) {
    gf::Task13UnifiedCoverageRequest value;
    value.uncovered_cells=uncovered;
    value.fixed_positions=fixed();
    for (int id=1;id<=14;++id)
        value.agents.push_back({static_cast<gf::NodeId>(id),
            {id<=7?300.0:2700.0,1500.0},M_PI/2.0});
    return value;
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> unifiedFixture() {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0,
        true,false,false,false,false,false,false,false,false,false,false,
        false,true,false,true,false,false,false,true);
}

void retainOnly(
    gf::Task10p11rFixedBaselineFixture& fixture,
    const std::vector<gf::FrontierCell>& uncovered) {
    auto state=fixture.adapter.fixedRestartState();
    std::fill(state.coverage.truth.begin(),state.coverage.truth.end(),true);
    std::fill(state.coverage.certified.begin(),
              state.coverage.certified.end(),true);
    for (const auto& value:uncovered) {
        const std::size_t index=static_cast<std::size_t>(
            value.x_index*300+value.y_index);
        state.coverage.truth[index]=false;
        state.coverage.certified[index]=false;
    }
    fixture.adapter.restoreFixedRestartState(state);
}

std::pair<gf::Task13UnifiedCoverageWitness,
          gf::Task13UnifiedCoverageWitness> compatiblePair(
    const gf::FrontierCell& a_cell,const gf::FrontierCell& b_cell) {
    const auto squads=gf::task13UnifiedCoverageSquads();
    for (gf::NodeId a:squads[0].members) {
        const auto wa=gf::task13TaperedWitness(
            squads[0],a,a_cell,fixed(),{});
        if (!wa.has_value()) continue;
        for (gf::NodeId b:squads[1].members) {
            const auto wb=gf::task13TaperedWitness(
                squads[1],b,b_cell,fixed(),{});
            if (wb.has_value()&&gf::task13CrossMinimum(*wa,*wb)>10.0)
                return {*wa,*wb};
        }
    }
    throw std::runtime_error("fixture pair not compatible");
}

}  // namespace

TEST_CASE("H2 tapered inverse preserves real cell identity and Level-A geometry") {
    const auto squads=gf::task13UnifiedCoverageSquads();
    const auto witness=gf::task13TaperedWitness(
        squads[0],7,cell(0,299),fixed(),{});
    REQUIRE(witness.has_value());
    CHECK(witness->cell.id()=="0:299");
    CHECK(witness->responsible_member==7);
    CHECK((witness->targets.at(7)-cell(0,299).center).norm()<1e-8);
    CHECK(witness->maximum_reference_edge_m<850.0);
    CHECK(witness->minimum_target_separation_m>10.0);
    CHECK(witness->targets.size()==7);
    for (gf::NodeId member:squads[0].members)
        CHECK(witness->target_ids.at(member)=="0:299");
}

TEST_CASE("H2 tapered fan closes both archived 0:299 blocker variants") {
    const auto squads=gf::task13UnifiedCoverageSquads();
    for (const auto retained_cell:{cell(36,222),cell(36,223)}) {
        bool compatible=false;
        for (gf::NodeId a:squads[0].members) {
            const auto wa=gf::task13TaperedWitness(
                squads[0],a,cell(0,299),fixed(),{});
            if (!wa.has_value()) continue;
            for (gf::NodeId b:squads[1].members) {
                const auto wb=gf::task13TaperedWitness(
                    squads[1],b,retained_cell,fixed(),{});
                compatible=compatible||(wb.has_value()&&
                    gf::task13CrossMinimum(*wa,*wb)>10.0);
            }
        }
        CHECK(compatible);
    }
}

TEST_CASE("Uniform domain-center service pose keeps real ID and certified standoff") {
    const auto squads=gf::task13UnifiedCoverageSquads();
    gf::Task13UnifiedCoverageConfig config;
    config.certified_service_standoff_m=350.0;
    const auto witness=gf::task13TaperedWitness(
        squads[0],7,cell(0,299),fixed(),config);
    REQUIRE(witness.has_value());
    CHECK(witness->cell.id()=="0:299");
    CHECK((witness->targets.at(7)-cell(0,299).center).norm()==
        doctest::Approx(350.0));
    CHECK(witness->maximum_reference_edge_m<767.0);
    CHECK(witness->minimum_target_separation_m>20.0);
    for (const auto& [member,id]:witness->target_ids) {
        (void)member;
        CHECK(id=="0:299");
    }
}

TEST_CASE("Unified allocator retains real IDs and re-embeds on compatibility event") {
    auto retained=compatiblePair(cell(299,0),cell(36,222));
    auto value=request({cell(0,299)});
    value.retained={{"A",retained.first},{"B",retained.second}};
    const auto result=gf::allocateTask13UnifiedCoverage(value);
    REQUIRE(result.valid);
    CHECK(result.active_squads==1);
    CHECK(result.assignments.at("A").witness.cell.id()=="0:299");
    CHECK(result.assignments.at("B").witness.cell.id()=="36:222");
    CHECK(result.minimum_cross_target_separation_m>10.0);
    for (const auto& [squad,assignment]:result.assignments) {
        (void)squad;
        for (const auto& [member,id]:assignment.witness.target_ids) {
            (void)member;
            CHECK(id.find('-')==std::string::npos);
        }
    }
}

TEST_CASE("Zero tasks retains exact configurations and adjacent tasks do not violate strict separation") {
    auto retained=compatiblePair(cell(7,299),cell(274,299));
    auto zero=request({});
    zero.retained={{"A",retained.first},{"B",retained.second}};
    const auto held=gf::allocateTask13UnifiedCoverage(zero);
    REQUIRE(held.valid);
    CHECK(held.active_squads==0);
    CHECK(held.assignments.at("A").witness.digest==retained.first.digest);
    CHECK(held.assignments.at("B").witness.digest==retained.second.digest);

    auto adjacent=request({cell(150,150),cell(150,151)});
    adjacent.retained=zero.retained;
    const auto selected=gf::allocateTask13UnifiedCoverage(adjacent);
    REQUIRE(selected.valid);
    for (const auto& [squad,assignment]:selected.assignments) {
        (void)squad;
        CHECK((assignment.witness.targets.at(
            assignment.witness.responsible_member)-
            assignment.witness.cell.center).norm()<1e-8);
    }
    CHECK(selected.active_squads==1);
    CHECK(selected.minimum_cross_target_separation_m>10.0);
}

TEST_CASE("Unified H2 fixture flag is an independent production policy path") {
    const auto config=gf::task10p11rFixtureAdapterConfig(
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0,
        true,false,false,false,false,false,false,false,false,false,false,
        false,true,false,true,false,false,false,true);
    CHECK(config.target_policy_unified_h2);
    CHECK_FALSE(config.target_policy_v2);
    CHECK_FALSE(config.target_policy_v3);
    CHECK_FALSE(config.target_policy_v6);
    CHECK(config.unified_h2_minimum_half_width_m==doctest::Approx(7.0));
    CHECK(config.unified_h2_fan_ratio==doctest::Approx(0.0075));
    CHECK(config.unified_h2_service_standoff_m==doctest::Approx(350.0));
}

TEST_CASE("Production H2 path is certified-event driven across 2, 1, and 0 tasks") {
    auto fixture=unifiedFixture();
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    retainOnly(*fixture,{cell(0,299),cell(36,222)});

    const auto two=fixture->controller.advance();
    REQUIRE(two.step.advanced);
    REQUIRE(two.unified_allocation_evaluated);
    REQUIRE(two.unified_allocation.valid);
    CHECK(two.unified_allocation.active_squads==2);
    CHECK(two.committed_targets.size()==14);
    for (const auto& [owner,target]:two.committed_targets) {
        (void)owner;
        CHECK((target.id()=="0:299"||target.id()=="36:222"));
    }

    retainOnly(*fixture,{cell(36,222)});
    const auto one=fixture->controller.advance();
    REQUIRE(one.step.advanced);
    REQUIRE(one.unified_allocation_evaluated);
    REQUIRE(one.unified_allocation.valid);
    CHECK(one.unified_allocation.active_squads==1);
    CHECK(one.unified_allocation.assignments.at("A").witness.cell.id()==
          "0:299");
    CHECK(one.unified_allocation.assignments.at("B").witness.cell.id()==
          "36:222");

    const auto held_one=fixture->controller.advance();
    REQUIRE(held_one.step.advanced);
    CHECK_FALSE(held_one.unified_allocation_evaluated);
    CHECK(held_one.target_epoch==one.target_epoch);

    retainOnly(*fixture,{});
    const auto zero=fixture->controller.advance();
    REQUIRE(zero.step.advanced);
    REQUIRE(zero.unified_allocation_evaluated);
    REQUIRE(zero.unified_allocation.valid);
    CHECK(zero.unified_allocation.active_squads==0);
    CHECK(zero.t100_event_latched);
    CHECK(zero.t100_coverage_s.has_value());
    CHECK(zero.committed_targets==one.committed_targets);
}
