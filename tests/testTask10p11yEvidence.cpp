#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11yEvidence.hpp"

#include <chrono>
#include <limits>

namespace {

std::filesystem::path temporaryDirectory(const std::string& label) {
    return std::filesystem::temp_directory_path()/(
        "task10p11y-"+label+"-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
}

}  // namespace

TEST_CASE("non-finite evidence is rejected before JSON publication") {
    const auto directory=temporaryDirectory("finite");
    const auto path=directory/"result.json";
    const nlohmann::json invalid={{"minimum",
        -std::numeric_limits<double>::infinity()}};
    CHECK_THROWS_WITH_AS(gf::writeTask10p11vJson(path,invalid),
        "non-finite Task 10.11v JSON value at $.minimum",
        std::invalid_argument);
    CHECK_FALSE(std::filesystem::exists(path));
    std::filesystem::remove_all(directory);
}

TEST_CASE("not-applicable metrics round-trip with explicit validity") {
    const auto directory=temporaryDirectory("metric");
    const auto path=directory/"result.json";
    const nlohmann::json evidence={
        {"observed",gf::task10p11yMetric(0.125,"observed")},
        {"missing",gf::task10p11yMetric(std::nullopt,
            "not_applicable_no_advanced_cycle")}};
    gf::writeTask10p11vJson(path,evidence);
    const auto restored=gf::readTask10p11vJson(path);
    CHECK(restored==evidence);
    CHECK(restored.at("missing").at("status")=="not_applicable");
    CHECK(restored.at("missing").at("value").is_null());
    std::filesystem::remove_all(directory);
}

TEST_CASE("cycle statistics count only advanced decisions") {
    gf::Task10p11yCycleStatistics statistics;
    statistics.observe(true,true,2.0);
    statistics.observe(false,true,100.0);
    statistics.observe(true,false,3.0);
    CHECK(statistics.advanced_cycles==2);
    CHECK(statistics.intervention_cycles==1);
    CHECK(statistics.cumulative_control_deviation==doctest::Approx(2.0));
}

TEST_CASE("post-advance hard-gate failure publishes current packed state") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto step=fixture->controller.advance();
    INFO(step.reason);
    REQUIRE(step.step.advanced);
    const double post_time=fixture->swarm.robots.front()->runtime;

    const auto directory=temporaryDirectory("post-gate");
    const auto path=directory/"failure.json";
    gf::writeTask10p11yPostAdvanceFailureCheckpoint(path,*fixture,
        "fixture_injected_post_advance_gate",{{"fixture",true}});
    const auto restored=gf::readTask10p11vJson(path);
    const auto audit=gf::auditTask10p11vRestartCheckpoint(restored);
    INFO(audit.reason);
    CHECK(audit.offline_oracle_complete);
    CHECK(audit.deterministic_restart_complete);
    CHECK(restored.at("restart_checkpoint").at("capture_event")==
        "post_advance_hard_gate_failure");
    CHECK(restored.at("runtime").at("runtime_s").get<double>()==
        doctest::Approx(post_time));
    CHECK(restored.at("restart_checkpoint").at("runtime_s").get<double>()==
        doctest::Approx(post_time));
    CHECK(restored.at("task10p11y").at("advanced")==true);
    std::filesystem::remove_all(directory);
}
