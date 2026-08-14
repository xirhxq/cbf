#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "Swarm.hpp"
#include "grand_finale/D1DevelopmentExperiment.hpp"
#include "grand_finale/ExactHardPolytopeDiagnostic.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/InformationGateDiagnostic.hpp"
#include "grand_finale/Task10p9MechanismFixture.hpp"

#include <fstream>
#include <memory>

namespace {

json baseSettings(
    int mobile_count,
    const std::vector<Eigen::Vector2d>& mobile,
    const std::map<gf::NodeId,Eigen::Vector2d>& fixed,
    double width, double height) {
    json settings = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
    settings["num"] = mobile_count;
    settings["optimiser"] = "OSQP";
    settings["formation"]["parts"] = 1;
    settings["formation"]["bases-id"] = {{0,1,2}};
    settings["bases"] = json::array();
    for (const auto& [id,p] : fixed) {
        (void)id; settings["bases"].push_back({p.x(),p.y()});
    }
    settings["initial"]["position"]["positions"] = json::array();
    settings["initial"]["velocity"]["values"] = json::array();
    for (const auto& p : mobile) {
        settings["initial"]["position"]["positions"].push_back({p.x(),p.y()});
        settings["initial"]["velocity"]["values"].push_back({0.0,0.0});
    }
    settings["world"]["boundary"] =
        {{0.0,0.0},{width,0.0},{width,height},{0.0,height}};
    settings["world"]["spacing"] = 2.0;
    settings["searching"]["downward"]["radius"] = 3.0;
    return settings;
}

gf::GrandFinaleSwarmAdapterConfig adapterConfig(gf::SolverProfile profile) {
    gf::GrandFinaleSwarmAdapterConfig config;
    config.solver_profile = profile;
    config.dt_s = 0.1;
    config.minimum_dwell_s = 0.1;
    config.acceleration_half_box = 0.4;
    config.sensor_radius_m = 3.0;
    config.range_dropout_probability = 0.0;
    return config;
}

struct Fixture {
    json settings;
    Swarm swarm;
    gf::GrandFinaleSwarmAdapter adapter;
    Fixture(json raw, std::vector<gf::NodeId> ids,
            std::map<gf::NodeId,Eigen::Vector2d> fixed,
            std::vector<gf::DirectedEdge> topology,
            gf::SolverProfile profile,
            unsigned int seed = 2027,
            double dropout = 0.0)
        : settings(std::move(raw)), swarm(settings),
          adapter(swarm, std::move(ids), std::move(fixed),
                  std::move(topology), [&] {
                      auto config = adapterConfig(profile);
                      config.range_random_seed = seed;
                      config.range_dropout_probability = dropout;
                      return config;
                  }()) {}
};

double ownerRobustFim(
    gf::GrandFinaleSwarmAdapter& adapter, gf::NodeId owner) {
    const auto runtime = adapter.runtimeSnapshot();
    std::vector<gf::DirectedEdge> edges;
    for (const auto& edge : runtime.topology)
        if (edge.owner == owner) edges.push_back(edge);
    std::map<std::string,double> variances;
    for (const auto& [id,link] : runtime.range_links)
        variances[id] = link.variance_m2;
    return gf::robustReferenceFimConeLowerBound(owner, edges,
        runtime.estimate, variances, adapter.config().uncertainty_sigma);
}

}

TEST_CASE("Task 10.9 traces frozen SI values through formal runtime components") {
    const auto frozen = json::parse(std::ifstream(
        std::string(PROJECT_ROOT) +
        "/config/grand_finale/task10p9_feasibility_gate.json"));
    const auto specification = gf::task10p9EasyScenario();
    auto settings = baseSettings(4, specification.mobile_positions,
        specification.fixed_positions,
        frozen.at("easy_map_m").at(0).get<double>(),
        frozen.at("easy_map_m").at(1).get<double>());
    settings["world"]["spacing"] = frozen.at("grid_spacing_m");
    settings["searching"]["downward"]["radius"] =
        frozen.at("sensor_radius_m");
    Fixture fixture(settings, specification.mobile_ids,
        specification.fixed_positions, specification.initial_topology,
        gf::SolverProfile::OpenSource);
    CHECK(fixture.swarm.gridWorldGroundTruth.xNum == 10);
    CHECK(fixture.swarm.gridWorldGroundTruth.yNum == 10);
    const Point p = fixture.swarm.robots.front()->model->xy();
    CHECK(p.x == doctest::Approx(specification.mobile_positions.front().x()));
    CHECK(p.y == doctest::Approx(specification.mobile_positions.front().y()));
    CHECK(fixture.adapter.config().reference_distance_m ==
          doctest::Approx(frozen.at("reference_keep_m").get<double>()));
    CHECK(fixture.adapter.config().add_reference_distance_m ==
          doctest::Approx(frozen.at("reference_add_m").get<double>()));
    CHECK(fixture.adapter.config().collision_distance_m ==
          doctest::Approx(frozen.at("collision_distance_m").get<double>()));
    CHECK(fixture.adapter.config().dt_s ==
          doctest::Approx(frozen.at("control_period_s").get<double>()));
    CHECK(fixture.adapter.config().acceleration_half_box ==
          doctest::Approx(
              frozen.at("acceleration_half_box_mps2").get<double>()));
    const auto zoh = propagateDoubleIntegratorPlanarZoh(
        Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
        Eigen::Vector2d(0.4,0.0), 0.1);
    CHECK(zoh.position.x() == doctest::Approx(0.002));
    CHECK(zoh.velocity.x() == doctest::Approx(0.04));
    REQUIRE(fixture.adapter.step().advanced);
    const auto rows = fixture.adapter.currentSnapshotHardRows(
        fixture.adapter.supervisor().topology());
    const auto reference = std::find_if(rows.begin(), rows.end(),
        [](const auto& row) {
            return row.kind == gf::CanonicalHardRowKind::ReferenceDistance;
        });
    REQUIRE(reference != rows.end());
    CHECK(reference->control_coefficient.allFinite());
    CHECK(std::isfinite(reference->constant));
    CHECK(std::isfinite(reference->position_uncertainty_reserve_m));
    CHECK(std::isfinite(reference->velocity_uncertainty_reserve_mps));
}

TEST_CASE("Task 10.9 easy scenario advances beyond initialization footprint") {
    const auto specification = gf::task10p9EasyScenario();
    for (const auto strategy : {gf::D1TopologyStrategy::FixedDag,
             gf::D1TopologyStrategy::ProposedCertified,
             gf::D1TopologyStrategy::GreedyCertified}) {
        (void)strategy;
        Fixture fixture(baseSettings(4, specification.mobile_positions,
            specification.fixed_positions, 20.0, 20.0),
            {1,2,3,4}, specification.fixed_positions,
            specification.initial_topology, gf::SolverProfile::OpenSource);
        const auto first = fixture.adapter.step();
        REQUIRE(first.advanced);
        const double initialized = first.truth_coverage;
        bool progressed = false;
        for (int cycle=0; cycle<80; ++cycle) {
            const auto step = fixture.adapter.step();
            REQUIRE(step.advanced);
            progressed = progressed || step.truth_coverage > initialized + 1e-12;
        }
        CHECK(progressed);
        MESSAGE("TASK10P9_EASY strategy=", gf::d1StrategyName(strategy),
                " initial_truth=", initialized,
                " final_truth=", fixture.adapter.coverage().truthFraction());
    }
}

TEST_CASE("Task 10.9 inactive scenario has ample reference margin and no switch") {
    const auto specification = gf::task10p9EasyScenario();
    for (const auto strategy : {gf::D1TopologyStrategy::FixedDag,
             gf::D1TopologyStrategy::ProposedCertified,
             gf::D1TopologyStrategy::GreedyCertified}) {
        (void)strategy;
        Fixture fixture(baseSettings(4, specification.mobile_positions,
            specification.fixed_positions, 80.0, 40.0),
            {1,2,3,4}, specification.fixed_positions,
            specification.initial_topology, gf::SolverProfile::OpenSource);
        REQUIRE(fixture.adapter.step().advanced);
        const auto rows = fixture.adapter.currentSnapshotHardRows(
            fixture.adapter.supervisor().topology());
        CHECK(gf::minimumReferenceBarrierH(rows) > 800.0);
        CHECK(fixture.adapter.transitionStackSize() == 0);
        CHECK(fixture.adapter.supervisor().mode() == gf::SupervisorMode::Search);
        MESSAGE("TASK10P9_INACTIVE strategy=", gf::d1StrategyName(strategy),
                " min_reference_h=", gf::minimumReferenceBarrierH(rows),
                " switches=0");
    }
}

TEST_CASE("Task 10.9 active geometry has strict old-add overlap") {
    const auto active = gf::task10p9ActiveScenario();
    const auto overlap = gf::referenceBallOverlap(
        active.fixed_positions.at(100), active.fixed_positions.at(102),
        850.0, 849.0);
    CHECK(overlap.strict);
    CHECK(overlap.slack_m == doctest::Approx(99.0));
    CHECK(active.map_width_m / 850.0 == doctest::Approx(2.0));
}

TEST_CASE("Task 10.9 active formal MIQP certifies a full fresh union replacement") {
    const auto specification = gf::task10p9ActiveScenario();
    for (const auto profile : {gf::SolverProfile::Gurobi,
                               gf::SolverProfile::OpenSource}) {
        for (const auto strategy : {gf::D1TopologyStrategy::FixedDag,
                 gf::D1TopologyStrategy::ProposedCertified,
                 gf::D1TopologyStrategy::GreedyCertified}) {
            Fixture fixture(baseSettings(14, specification.mobile_positions,
                specification.fixed_positions, specification.map_width_m,
                specification.map_height_m), specification.mobile_ids,
                specification.fixed_positions, specification.initial_topology,
                profile);
            const auto initialized = fixture.adapter.step();
            REQUIRE(initialized.advanced);
            // One identical SEARCH/frontier control-estimator-ZOH cycle is the
            // frozen nonanticipative trigger prefix for all three strategies.
            const auto pre_trigger = fixture.adapter.step();
            REQUIRE(pre_trigger.advanced);
            const double old_robust_fim = ownerRobustFim(fixture.adapter,2);
            CHECK(old_robust_fim > 1e-6);
            const auto rows = fixture.adapter.currentSnapshotHardRows(
                fixture.adapter.supervisor().topology());
            const double old_reference_h = gf::minimumReferenceBarrierH(rows);
            CHECK(old_reference_h < 100.0);
            CHECK(old_reference_h > 0.0);
            const auto candidates =
                gf::task10p9ActiveCandidateUniverse(fixture.adapter);
            REQUIRE(candidates.size() == 14);
            CHECK(gf::containsEdge(candidates, {102,2}));
            const auto reference_audit = fixture.adapter.currentReferenceAudit();
            CHECK(reference_audit.minimum_effective_reference_count >= 2);
            CHECK(reference_audit.maximum_posterior_eigenvalue <
                  fixture.adapter.config().maximum_posterior_eigenvalue_m2);

            if (strategy == gf::D1TopologyStrategy::FixedDag) {
                const auto fixed_step = fixture.adapter.step();
                REQUIRE(fixed_step.advanced);
                CHECK(fixture.adapter.supervisor().mode() ==
                      gf::SupervisorMode::Search);
                CHECK(gf::containsEdge(
                    fixture.adapter.supervisor().topology(), {100,2}));
                CHECK_FALSE(gf::containsEdge(
                    fixture.adapter.supervisor().topology(), {102,2}));
                continue;
            }

            if (strategy == gf::D1TopologyStrategy::ProposedCertified) {
                const auto proposal = fixture.adapter.proposeAndBegin(
                    gf::task10p9ActiveProposal(fixture.adapter));
                INFO(proposal.reason);
                REQUIRE(proposal.transition_started);
                CHECK(gf::containsEdge(proposal.selected_topology, {102,2}));
                CHECK_FALSE(gf::containsEdge(proposal.selected_topology, {100,2}));
            } else {
                std::vector<gf::D1ReplacementCandidate> greedy_candidates;
                for (const auto& edge : candidates)
                    greedy_candidates.push_back({edge,{100,2},
                        edge.id() == "102->2" ? 1.0 : 0.0});
                const auto choice = gf::chooseD1Replacement(
                    gf::D1TopologyStrategy::GreedyCertified,
                    greedy_candidates);
                REQUIRE(choice.has_value());
                CHECK(choice->addition.id() == "102->2");
                REQUIRE(fixture.adapter.beginReplacement(
                    choice->addition,choice->removal));
            }
            const auto union_step = fixture.adapter.step();
            REQUIRE(union_step.advanced);
            const double union_robust_fim = ownerRobustFim(fixture.adapter,2);
            CHECK(union_robust_fim > 1e-6);
            CHECK(union_step.mode == gf::SupervisorMode::Union);
            CHECK(fixture.adapter.unionControlCycles() == 1);
            REQUIRE(fixture.adapter.finishReplacementAfterFreshCycle());
            CHECK(gf::containsEdge(
                fixture.adapter.supervisor().topology(), {102,2}));
            const double successor_robust_fim = ownerRobustFim(fixture.adapter,2);
            CHECK(successor_robust_fim > 1e-6);
            MESSAGE("TASK10P9_ACTIVE profile=",
                    profile == gf::SolverProfile::Gurobi
                        ? std::string("gurobi") : std::string("open_source"),
                    " strategy=", gf::d1StrategyName(strategy),
                    " candidate_count=", candidates.size(),
                    " old_h=", old_reference_h,
                    " fim_old_union_successor=", old_robust_fim, ",",
                    union_robust_fim, ",", successor_robust_fim,
                    " union_cycles=1 residual=", union_step.minimum_hard_residual,
                    " switch_s=0.1");
        }
    }
}

TEST_CASE("Task 10.9 reproduces all fixed paired failures with exact row certificates") {
    for (const bool elongated : {false,true}) {
        const auto specification = elongated
            ? gf::task10p9D1ElongatedScenario()
            : gf::task10p9D1SquareScenario();
        for (const auto profile : {gf::SolverProfile::Gurobi,
                                   gf::SolverProfile::OpenSource}) {
            for (const unsigned int seed : {2027U,2029U,2039U}) {
                Fixture fixture(baseSettings(14, specification.mobile_positions,
                    specification.fixed_positions, specification.map_width_m,
                    specification.map_height_m), specification.mobile_ids,
                    specification.fixed_positions,
                    specification.initial_topology, profile, seed, 0.02);
                bool failed = false;
                gf::HardPolytopeCertificate first_certificate;
                for (int cycle=0; cycle<300; ++cycle) {
                    const auto step = fixture.adapter.step();
                    if (step.advanced) continue;
                    REQUIRE(step.reason == "hard_polytope_empty");
                    const auto rows = fixture.adapter.currentSnapshotHardRows(
                        fixture.adapter.supervisor().topology());
                    for (gf::NodeId owner : specification.mobile_ids) {
                        auto certificate = gf::diagnoseHardPolytope(owner,
                            fixture.swarm.robots.front()->runtime,
                            fixture.adapter.supervisor().mode(),
                            fixture.adapter.supervisor().topology(), rows, 0.4);
                        if (!certificate.exact_feasible) {
                            first_certificate = std::move(certificate);
                            break;
                        }
                    }
                    failed = true;
                    break;
                }
                REQUIRE(failed);
                REQUIRE(first_certificate.owner > 0);
                REQUIRE_FALSE(first_certificate.minimal_conflict_row_ids.empty());
                for (const auto& record : first_certificate.rows) {
                    CHECK(record.normal.allFinite());
                    CHECK(std::isfinite(record.constant));
                    CHECK(std::isfinite(record.position_reserve_m));
                    CHECK(std::isfinite(record.velocity_reserve_mps));
                    CHECK(std::isfinite(record.coefficient_reserve));
                }
                std::string ids;
                for (const auto& id : first_certificate.minimal_conflict_row_ids)
                    ids += (ids.empty() ? "" : ",") + id;
                const std::string map_name = elongated ? "elongated" : "square";
                const std::string profile_name =
                    profile == gf::SolverProfile::Gurobi ? "gurobi" : "open_source";
                MESSAGE("TASK10P9_FIXED_CONFLICT map=", map_name,
                        " profile=", profile_name,
                        " seed=", seed, " owner=", first_certificate.owner,
                        " time=", first_certificate.time_s, " rows=", ids);
                if (!elongated) {
                    CHECK(first_certificate.time_s >= 15.0);
                    CHECK(first_certificate.time_s <= 16.0);
                } else {
                    CHECK(first_certificate.time_s >= 23.0);
                    CHECK(first_certificate.time_s <= 25.0);
                }
            }
        }
    }
}
