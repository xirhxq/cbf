#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11lCollisionModelAudit.hpp"

TEST_CASE("Task 10.11l formal coastal initial states satisfy ten metre separation") {
    const std::vector<gf::Task10p10Scenario> scenarios{
        gf::task10p11hCoastalEasyScenario(),
        gf::task10p11hCoastalLeaderEasyScenario(),
        gf::task10p11hCoastalNonbindingScenario(),
        gf::task10p11hCoastalBindingScenario(),
        gf::task10p11hCoastalBindingActiveScenario()};
    for (const auto& scenario : scenarios) {
        CAPTURE(scenario.id);
        const auto audit=gf::auditTask10p11lInitialSeparation(scenario,10.0);
        CHECK(audit.valid);
        CHECK(audit.minimum_distance_m>=10.0);
        MESSAGE("TASK10P11L_INITIAL scenario=",scenario.id,
            " pair=",audit.first,"--",audit.second,
            " kind=",audit.minimum_kind==gf::Task10p11lPairKind::MobileMobile
                ?"mobile_mobile":"mobile_fixed",
            " distance_m=",audit.minimum_distance_m,
            " margin_m=",audit.margin_m);
    }
}

TEST_CASE("Task 10.11l rejects the historical eight metre interior-anchor fixture") {
    const auto audit=gf::auditTask10p11lInitialSeparation(
        gf::task10p11gBindingMechanismScenario(),10.0);
    CHECK_FALSE(audit.valid);
    CHECK(audit.minimum_kind==gf::Task10p11lPairKind::MobileMobile);
    CHECK(audit.minimum_distance_m==doctest::Approx(8.0));
    CHECK(audit.margin_m==doctest::Approx(-2.0));
}
