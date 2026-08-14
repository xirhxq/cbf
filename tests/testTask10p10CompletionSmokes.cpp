#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/D1DevelopmentExperiment.hpp"
#include "grand_finale/ExactHardPolytopeDiagnostic.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"
#include "grand_finale/Task10p9MechanismFixture.hpp"

namespace {

std::string certificateSummary(const gf::HardPolytopeCertificate& certificate) {
    std::ostringstream out;
    for (const auto& id : certificate.minimal_conflict_row_ids) {
        out << " [" << id;
        const auto row = std::find_if(certificate.rows.begin(),certificate.rows.end(),
            [&](const auto& candidate) { return candidate.id == id; });
        if (row != certificate.rows.end()) {
            out << " kind=" << static_cast<int>(row->kind)
                << " n=(" << row->normal.x() << ',' << row->normal.y() << ')'
                << " c=" << row->constant << " h=" << row->barrier_h
                << " psi1=" << row->barrier_psi1
                << " rp=" << row->position_reserve_m
                << " rv=" << row->velocity_reserve_mps
                << " rc=" << row->coefficient_reserve;
        }
        out << ']';
    }
    return out.str();
}

struct CompletionResult {
    double initialized = 0.0;
    double t95_s = -1.0;
    std::string failure;
    double failure_time_s = -1.0;
    double failure_coverage = -1.0;
    double stage_zero_gridworld_coverage = -1.0;
    std::vector<gf::HardPolytopeCertificate> infeasible_certificates;
    std::map<gf::NodeId,Eigen::Vector2d> estimated_positions;
    std::map<gf::NodeId,Eigen::Vector2d> truth_positions;
};

CompletionResult runCompletion(
    const gf::Task10p10Scenario& scenario,
    gf::D1TopologyStrategy strategy) {
    (void)strategy;
    auto fixture = gf::makeTask10p10Fixture(
        scenario,gf::SolverProfile::OpenSource);
    CompletionResult result;
    const auto initialized = fixture->adapter.initializeStageZero();
    result.initialized = initialized.truth_coverage;
    result.stage_zero_gridworld_coverage =
        fixture->swarm.gridWorldGroundTruth.getPercentage();
    if (!initialized.initialized) {
        result.failure = initialized.reason;
        return result;
    }
    const int maximum_cycles = static_cast<int>(
        gf::loadTask10p10Config().timeout_cap_s /
        gf::loadTask10p10Config().control_period_s);
    for (int cycle=0; cycle<maximum_cycles; ++cycle) {
        const auto step = fixture->adapter.step();
        if (!step.advanced) {
            result.failure = step.reason;
            result.failure_time_s = fixture->swarm.robots.front()->runtime;
            result.failure_coverage = fixture->adapter.coverage().truthFraction();
            const auto runtime = fixture->adapter.runtimeSnapshot();
            for (std::size_t index=0;
                 index<runtime.estimate.mobile_ids.size();++index) {
                result.estimated_positions[
                    runtime.estimate.mobile_ids[index]] =
                    runtime.estimate.mean.segment<2>(4*index);
            }
            for (const auto& robot : fixture->swarm.robots) {
                result.truth_positions[robot->id] = robot->model->xy().vec();
            }
            if (step.reason == "hard_polytope_empty") {
                const auto rows = fixture->adapter.currentSnapshotHardRows(
                    fixture->adapter.supervisor().topology());
                for (const auto owner : scenario.mobile_ids) {
                    const auto certificate = gf::diagnoseHardPolytope(
                        owner,result.failure_time_s,
                        fixture->adapter.supervisor().mode(),
                        fixture->adapter.supervisor().topology(),rows,
                        fixture->adapter.config().acceleration_half_box);
                    if (!certificate.exact_feasible)
                        result.infeasible_certificates.push_back(certificate);
                }
            }
            return result;
        }
        if (step.truth_coverage >= 0.95) {
            result.t95_s = fixture->swarm.robots.front()->runtime;
            return result;
        }
    }
    result.failure = "timeout";
    return result;
}

}

TEST_CASE("Task 10.10 stage zero initializes estimator and sector coverage without physics") {
    auto fixture = gf::makeTask10p10Fixture(
        gf::task10p10NonbindingScenario(),gf::SolverProfile::OpenSource);
    CHECK(fixture->swarm.robots.front()->runtime == doctest::Approx(0.0));
    const auto initialized = fixture->adapter.initializeStageZero();
    REQUIRE(initialized.initialized);
    CHECK(initialized.estimator_version > 0);
    CHECK(fixture->swarm.robots.front()->runtime == doctest::Approx(0.0));
    CHECK(initialized.truth_coverage > 0.0);
    CHECK(initialized.truth_coverage < 0.95);
    CHECK(initialized.truth_coverage == doctest::Approx(
        gf::evaluateTask10p10ScenarioBudget(
            gf::task10p10NonbindingScenario()).coverage.initial_fraction));
    CHECK(fixture->swarm.gridWorldGroundTruth.getPercentage() ==
        doctest::Approx(initialized.truth_coverage));
    MESSAGE("stage-zero adapter=",initialized.truth_coverage,
        " swarm-grid=",fixture->swarm.gridWorldGroundTruth.getPercentage());
}

TEST_CASE("Task 10.10 preserves the shared completion-smoke failure certificates") {
    for (const auto strategy : {gf::D1TopologyStrategy::FixedDag,
             gf::D1TopologyStrategy::ProposedCertified,
             gf::D1TopologyStrategy::GreedyCertified}) {
        const auto easy = runCompletion(gf::task10p10EasyScenario(),strategy);
        INFO(easy.failure," t=",easy.failure_time_s,
            " coverage=",easy.failure_coverage,
            " stage-zero-grid=",easy.stage_zero_gridworld_coverage);
        for (const auto& certificate : easy.infeasible_certificates) {
            MESSAGE("easy owner=",certificate.owner,
                " estimate=",easy.estimated_positions.at(certificate.owner).transpose(),
                " truth=",easy.truth_positions.at(certificate.owner).transpose(),
                " conflict=",certificateSummary(certificate));
        }
        CHECK(easy.initialized < 0.95);
        CHECK(easy.t95_s < 0.0);
        CHECK(easy.failure == "hard_polytope_empty");
        CHECK(easy.failure_time_s == doctest::Approx(52.8));
        CHECK_FALSE(easy.infeasible_certificates.empty());

        const auto nonbinding = runCompletion(
            gf::task10p10NonbindingScenario(),strategy);
        INFO(nonbinding.failure," t=",nonbinding.failure_time_s,
            " coverage=",nonbinding.failure_coverage,
            " stage-zero-grid=",nonbinding.stage_zero_gridworld_coverage);
        for (const auto& certificate : nonbinding.infeasible_certificates) {
            MESSAGE("nonbinding owner=",certificate.owner,
                " estimate=",nonbinding.estimated_positions.at(certificate.owner).transpose(),
                " truth=",nonbinding.truth_positions.at(certificate.owner).transpose(),
                " conflict=",certificateSummary(certificate));
        }
        CHECK(nonbinding.initialized < 0.95);
        CHECK(nonbinding.t95_s < 0.0);
        CHECK(nonbinding.failure == "hard_polytope_empty");
        CHECK(nonbinding.failure_time_s == doctest::Approx(20.1));
        CHECK_FALSE(nonbinding.infeasible_certificates.empty());
    }
}
