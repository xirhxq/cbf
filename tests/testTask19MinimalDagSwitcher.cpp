#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task19MinimalDagSwitcher.hpp"

TEST_CASE("Task 19 origin and microfix preserve the frozen coverage roles") {
    const auto origin=gf::task19OriginDag();
    const auto microfix=gf::task19MicrofixDag();
    CHECK(gf::task19CoverageRoleEquivalentDag(origin));
    CHECK(gf::task19CoverageRoleEquivalentDag(microfix));
    CHECK(gf::task19DagReplacementCount(origin,microfix)==2);
    const auto& squads=gf::task13UnifiedCoverageSquads();
    CHECK(squads[0].leader==7);
    CHECK(squads[1].leader==14);
    CHECK(squads[0].members.front()==1);
    CHECK(squads[0].members.back()==7);
    CHECK(squads[1].members.front()==8);
    CHECK(squads[1].members.back()==14);
}

TEST_CASE("Task 19 switching signal honors window dwell and threshold") {
    gf::Task19InterventionSignal signal({4,10,5,0.40});
    for (std::size_t tick=0;tick<9;++tick) {
        signal.observe(tick,14,14);
        CHECK_FALSE(signal.shouldSwitch(tick));
    }
    signal.observe(9,5,14);
    CHECK(signal.evaluationDue(9));
    CHECK(signal.rollingFraction()==doctest::Approx(47.0/56.0));
    CHECK(signal.shouldSwitch(9));

    gf::Task19InterventionSignal low({4,10,5,0.40});
    for (std::size_t tick=0;tick<10;++tick)
        low.observe(tick,tick>=6?5:6,14);
    CHECK(low.evaluationDue(9));
    CHECK(low.rollingFraction()<0.40);
    CHECK_FALSE(low.shouldSwitch(9));
}

TEST_CASE("Task 19 switcher completes the two-edge DAG action make-before-break") {
    auto fixture=gf::makeTask19ProductionFixture();
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task19DagSwitchConfig config;
    config.signal={1,1,1,0.0};
    gf::Task19OriginMicrofixSwitcher switcher(fixture->adapter,config);

    auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    auto event=switcher.afterStep(0,step.step);
    CHECK(event.switch_requested);
    CHECK(event.transition_started);
    CHECK(fixture->adapter.runtimeSnapshot().adapter_transition_pending);

    step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    event=switcher.afterStep(1,step.step);
    CHECK(event.transition_finished);
    CHECK_FALSE(event.transition_started);

    step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    event=switcher.afterStep(2,step.step);
    CHECK(event.transition_started);

    step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    event=switcher.afterStep(3,step.step);
    CHECK(event.transition_finished);
    CHECK(switcher.complete());
    CHECK(gf::task19SameDag(fixture->adapter.runtimeSnapshot().topology,
                            gf::task19MicrofixDag()));
    CHECK(switcher.audit().certified_edge_replacements==2);
    for (const auto& [owner,target]:step.committed_targets) {
        CAPTURE(owner);
        CHECK(target.x_index>=0);
        CHECK(target.y_index>=0);
    }
}
