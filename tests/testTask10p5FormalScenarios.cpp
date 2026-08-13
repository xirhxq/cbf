#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"

#include <fstream>
#include <memory>

namespace {

json settings4p2(bool collinear = false) {
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
    settings["num"] = 4;
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0, 1}};
    settings["bases"] = collinear
        ? json{{4.0, 10.0}, {6.0, 10.0}}
        : json{{4.0, 4.0}, {4.0, 16.0}};
    settings["initial"]["position"]["positions"] = collinear
        ? json{{8.0, 10.0}, {10.0, 10.0}, {12.0, 10.0}, {14.0, 10.0}}
        : json{{7.0, 7.0}, {7.0, 13.0}, {12.0, 7.0}, {12.0, 13.0}};
    settings["initial"]["velocity"]["values"] = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    settings["world"]["spacing"] = 1.0;
    settings["searching"]["downward"]["radius"] = 2.0;
    return settings;
}

std::vector<gf::DirectedEdge> topology() {
    return {{10, 1}, {11, 1}, {10, 2}, {1, 2},
            {11, 3}, {1, 3}, {2, 4}, {3, 4}};
}

struct FormalFixture {
    json settings;
    Swarm swarm;
    gf::GrandFinaleSwarmAdapter adapter;

    explicit FormalFixture(
        bool collinear = false,
        double error_bound = 0.05,
        gf::SolverProfile profile = gf::SolverProfile::OpenSource,
        double initial_variance = 1.0e-4)
        : settings(settings4p2(collinear)),
          swarm(settings),
          adapter(
              swarm, {1, 2, 3, 4},
              collinear
                ? std::map<gf::NodeId, Eigen::Vector2d>{
                    {10, {4.0, 10.0}}, {11, {6.0, 10.0}}}
                : std::map<gf::NodeId, Eigen::Vector2d>{
                    {10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
              topology(),
              [&] {
                  gf::GrandFinaleSwarmAdapterConfig config;
                  config.solver_profile = profile;
                  config.dt_s = 0.1;
                  config.acceleration_half_box = 0.4;
                  config.sensor_radius_m = 2.0;
                  config.certified_error_bound_m = error_bound;
                  config.initial_position_variance_m2 = initial_variance;
                  return config;
              }()) {}
};

void requireCertifiedStep(const gf::GrandFinaleSwarmStep& step) {
    INFO(step.reason);
    REQUIRE(step.advanced);
    CHECK(step.certified_control_count == 4);
    CHECK(step.minimum_hard_residual >= -1e-7);
}

gf::TopologyRequest proposalRequest(
    const gf::GrandFinaleSwarmAdapter& adapter,
    gf::DirectedEdge candidate) {
    std::vector<gf::DirectedEdge> eligible =
        adapter.supervisor().topology();
    if (!gf::transition_certifier_detail::contains(eligible, candidate))
        eligible.push_back(candidate);
    return gf::TopologyRequest{
        {1, 2, 3, 4}, {10, 11}, eligible,
        adapter.supervisor().topology(), 2, 2,
        {{candidate.id(), 10.0}}, {}, {}, {}};
}

}  // namespace

TEST_CASE("4+2 formal scenario 1: normal certified coverage") {
    FormalFixture fixture;
    double first = 0.0;
    double last = 0.0;
    for (int step = 0; step < 8; ++step) {
        const auto result = fixture.adapter.step();
        requireCertifiedStep(result);
        if (step == 0) first = result.certified_coverage;
        last = result.certified_coverage;
    }
    CHECK(last >= first);
    CHECK(fixture.adapter.coverage().certifiedSubsetOfTruth());
}

TEST_CASE("4+2 formal scenario 2: required reconfiguration spans a full union cycle") {
    FormalFixture fixture;
    requireCertifiedStep(fixture.adapter.step());
    const auto proposal = fixture.adapter.proposeAndBegin(
        proposalRequest(fixture.adapter, {11, 2}));
    REQUIRE(proposal.transition_started);
    const auto union_step = fixture.adapter.step();
    requireCertifiedStep(union_step);
    CHECK(union_step.mode == gf::SupervisorMode::Union);
    CHECK(fixture.adapter.unionControlCycles() == 1);
    REQUIRE(fixture.adapter.finishReplacementAfterFreshCycle());
    CHECK(fixture.adapter.transitionStackSize() == 1);
}

TEST_CASE("4+2 formal scenario 3: multiple replacements remain edge-by-edge") {
    FormalFixture fixture;
    requireCertifiedStep(fixture.adapter.step());
    for (int transition = 0; transition < 3; ++transition) {
        const bool currently_uses_10 = gf::transition_certifier_detail::contains(
            fixture.adapter.supervisor().topology(), {10, 2});
        REQUIRE(fixture.adapter.beginReplacement(
            currently_uses_10 ? gf::DirectedEdge{11, 2}
                              : gf::DirectedEdge{10, 2},
            currently_uses_10 ? gf::DirectedEdge{10, 2}
                              : gf::DirectedEdge{11, 2}));
        const auto union_step = fixture.adapter.step();
        requireCertifiedStep(union_step);
        CHECK(fixture.adapter.unionControlCycles() == 1);
        REQUIRE(fixture.adapter.finishReplacementAfterFreshCycle());
        if (transition < 2) requireCertifiedStep(fixture.adapter.step());
    }
    CHECK(fixture.adapter.transitionStackSize() == 3);
}

TEST_CASE("4+2 formal scenario 4: cyclic candidate is rejected then HOLD is QP-controlled") {
    FormalFixture fixture;
    requireCertifiedStep(fixture.adapter.step());
    CHECK_FALSE(fixture.adapter.beginReplacement({4, 2}, {10, 2}));
    CHECK(fixture.adapter.supervisor().mode() == gf::SupervisorMode::Hold);
    const auto hold = fixture.adapter.step();
    requireCertifiedStep(hold);
    CHECK(hold.mode == gf::SupervisorMode::Hold);
}

TEST_CASE("4+2 formal scenario 5: degenerate reference FIM rejects switching without bypassing QP") {
    FormalFixture fixture(true);
    requireCertifiedStep(fixture.adapter.step());
    const auto proposal = fixture.adapter.proposeAndBegin(
        proposalRequest(fixture.adapter, {11, 2}));
    CHECK_FALSE(proposal.transition_started);
    CHECK(proposal.no_good_rejections > 0);
    CHECK(fixture.adapter.supervisor().mode() == gf::SupervisorMode::Hold);
    const auto hold = fixture.adapter.step();
    requireCertifiedStep(hold);
}

TEST_CASE("4+2 formal scenario 6: gamma warning enters REFORM with certified QP") {
    FormalFixture fixture;
    requireCertifiedStep(fixture.adapter.step());
    CHECK(fixture.adapter.supervisor().observeGamma(
              fixture.swarm.robots.front()->runtime, 0.01, true, true) ==
          gf::SupervisorMode::Reform);
    const auto reform = fixture.adapter.step();
    requireCertifiedStep(reform);
    CHECK(reform.mode == gf::SupervisorMode::Reform);
}

TEST_CASE("4+2 formal scenario 7: no candidate uses certified HOLD or RETREAT flow") {
    FormalFixture hold_fixture;
    requireCertifiedStep(hold_fixture.adapter.step());
    CHECK_FALSE(hold_fixture.adapter.requestCertifiedRetreat());
    const auto hold = hold_fixture.adapter.step();
    requireCertifiedStep(hold);
    CHECK(hold.mode == gf::SupervisorMode::Hold);

    FormalFixture retreat_fixture;
    requireCertifiedStep(retreat_fixture.adapter.step());
    REQUIRE(retreat_fixture.adapter.beginReplacement({11, 2}, {10, 2}));
    requireCertifiedStep(retreat_fixture.adapter.step());
    REQUIRE(retreat_fixture.adapter.finishReplacementAfterFreshCycle());
    requireCertifiedStep(retreat_fixture.adapter.step());
    REQUIRE(retreat_fixture.adapter.requestCertifiedRetreat());
    const auto retreat = retreat_fixture.adapter.step();
    requireCertifiedStep(retreat);
    CHECK(retreat.mode == gf::SupervisorMode::Retreat);
    REQUIRE(retreat_fixture.adapter.beginRetreatReplacement());
    const auto reverse_union = retreat_fixture.adapter.step();
    requireCertifiedStep(reverse_union);
    CHECK(reverse_union.mode == gf::SupervisorMode::Union);
    REQUIRE(retreat_fixture.adapter.finishReplacementAfterFreshCycle());
    CHECK(retreat_fixture.adapter.transitionStackSize() == 0);
}

TEST_CASE("4+2 formal scenario 8: uncertainty makes certified coverage conservative") {
    FormalFixture low(false, 0.0, gf::SolverProfile::OpenSource, 1.0e-4);
    FormalFixture high(false, 0.0, gf::SolverProfile::OpenSource, 0.04);
    const auto low_result = low.adapter.step();
    const auto high_result = high.adapter.step();
    requireCertifiedStep(low_result);
    requireCertifiedStep(high_result);
    CHECK(high_result.truth_coverage > 0.0);
    CHECK(high_result.certified_coverage <= low_result.certified_coverage);
    CHECK(high.adapter.coverage().certifiedSubsetOfTruth());
}

TEST_CASE("Exact certifier rejection adds no-good cuts until a different candidate certifies") {
    json settings = settings4p2();
    settings["initial"]["position"]["positions"][2] = {7.0, 1.0};
    Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(
        swarm, {1, 2, 3, 4}, {{10, {4.0, 4.0}}, {11, {4.0, 16.0}}},
        topology(), [&] {
            gf::GrandFinaleSwarmAdapterConfig config;
            config.solver_profile = gf::SolverProfile::OpenSource;
            return config;
        }());
    requireCertifiedStep(adapter.step());
    std::vector<gf::DirectedEdge> eligible = topology();
    eligible.push_back({3, 2});
    eligible.push_back({11, 2});
    const gf::TopologyRequest request{
        {1, 2, 3, 4}, {10, 11}, eligible, topology(), 2, 2,
        {{"3->2", 20.0}, {"11->2", 10.0}}, {},
        {{"1->2|3->2", 100.0}}, {}};
    const auto proposal = adapter.proposeAndBegin(request);
    std::string rejection_trace = proposal.reason;
    for (const auto& reason : proposal.rejection_reasons)
        rejection_trace += "|" + reason;
    INFO(rejection_trace);
    REQUIRE(proposal.transition_started);
    CHECK(proposal.no_good_rejections >= 1);
    CHECK_FALSE(proposal.rejection_reasons.empty());
    CHECK(proposal.no_good_rejections >= 1);
    CHECK(rejection_trace.find("fim") != std::string::npos);
}

#ifdef ENABLE_GUROBI
TEST_CASE("Gurobi profile executes all eight 4+2 formal mechanisms") {
    SUBCASE("normal coverage") {
        FormalFixture fixture(false, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
    }
    SUBCASE("required reconfiguration") {
        FormalFixture fixture(false, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
        REQUIRE(fixture.adapter.beginReplacement({11, 2}, {10, 2}));
        requireCertifiedStep(fixture.adapter.step());
        REQUIRE(fixture.adapter.finishReplacementAfterFreshCycle());
    }
    SUBCASE("multiple edge replacements") {
        FormalFixture fixture(false, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
        for (int index = 0; index < 2; ++index) {
            const bool uses_10 = gf::transition_certifier_detail::contains(
                fixture.adapter.supervisor().topology(), {10, 2});
            REQUIRE(fixture.adapter.beginReplacement(
                uses_10 ? gf::DirectedEdge{11, 2} : gf::DirectedEdge{10, 2},
                uses_10 ? gf::DirectedEdge{10, 2} : gf::DirectedEdge{11, 2}));
            requireCertifiedStep(fixture.adapter.step());
            REQUIRE(fixture.adapter.finishReplacementAfterFreshCycle());
            if (index == 0) requireCertifiedStep(fixture.adapter.step());
        }
    }
    SUBCASE("cycle rejection") {
        FormalFixture fixture(false, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
        CHECK_FALSE(fixture.adapter.beginReplacement({4, 2}, {10, 2}));
        requireCertifiedStep(fixture.adapter.step());
    }
    SUBCASE("FIM degeneration") {
        FormalFixture fixture(true, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
        CHECK_FALSE(fixture.adapter.beginReplacement({11, 2}, {10, 2}));
        requireCertifiedStep(fixture.adapter.step());
    }
    SUBCASE("gamma warning") {
        FormalFixture fixture(false, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
        fixture.adapter.supervisor().observeGamma(
            fixture.swarm.robots.front()->runtime, 0.01, true, true);
        const auto step = fixture.adapter.step();
        requireCertifiedStep(step);
        CHECK(step.mode == gf::SupervisorMode::Reform);
    }
    SUBCASE("no-candidate hold") {
        FormalFixture fixture(false, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
        CHECK_FALSE(fixture.adapter.requestCertifiedRetreat());
        const auto step = fixture.adapter.step();
        requireCertifiedStep(step);
        CHECK(step.mode == gf::SupervisorMode::Hold);
    }
    SUBCASE("certified retreat stack") {
        FormalFixture fixture(false, 0.05, gf::SolverProfile::Gurobi);
        requireCertifiedStep(fixture.adapter.step());
        REQUIRE(fixture.adapter.beginReplacement({11, 2}, {10, 2}));
        requireCertifiedStep(fixture.adapter.step());
        REQUIRE(fixture.adapter.finishReplacementAfterFreshCycle());
        requireCertifiedStep(fixture.adapter.step());
        REQUIRE(fixture.adapter.requestCertifiedRetreat());
        requireCertifiedStep(fixture.adapter.step());
        REQUIRE(fixture.adapter.beginRetreatReplacement());
        requireCertifiedStep(fixture.adapter.step());
        REQUIRE(fixture.adapter.finishReplacementAfterFreshCycle());
        CHECK(fixture.adapter.transitionStackSize() == 0);
    }
    SUBCASE("uncertainty conservative coverage") {
        FormalFixture low(
            false, 0.0, gf::SolverProfile::Gurobi, 1.0e-4);
        FormalFixture high(
            false, 0.0, gf::SolverProfile::Gurobi, 0.04);
        const auto low_step = low.adapter.step();
        const auto high_step = high.adapter.step();
        requireCertifiedStep(low_step);
        requireCertifiedStep(high_step);
        CHECK(high_step.certified_coverage <= low_step.certified_coverage);
        CHECK(high.adapter.coverage().certifiedSubsetOfTruth());
    }
}
#endif
