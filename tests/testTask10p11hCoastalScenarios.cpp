#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/TopologyModel.hpp"

namespace {

void verifyCoastalScenario(
    const gf::Task10p10Scenario& scenario,gf::SolverProfile profile) {
    for (const auto& [id,position] : scenario.fixed_positions) {
        (void)id;
        CHECK(position.y()<0.0);
    }
    for (const auto& position : scenario.mobile_positions)
        CHECK(position.y()>=0.0);
    gf::TopologyRequest topology_request;
    topology_request.mobile_ids=scenario.mobile_ids;
    topology_request.fixed_ids={100,101,102};
    topology_request.eligible_edges=scenario.initial_topology;
    topology_request.old_edges=scenario.initial_topology;
    topology_request.min_indegree=2;
    topology_request.max_indegree=2;
    const auto topology=gf::TopologyModel(topology_request).evaluate(
        scenario.initial_topology);
    INFO(scenario.id," topology=",topology.reason);
    REQUIRE(topology.valid);

    auto fixture=gf::makeTask10p11gFixture(scenario,profile);
    for (const auto& robot : fixture->swarm.robots)
        CHECK(robot->model->getStateVariable("yawRad")==doctest::Approx(M_PI/2.0));
    const auto initialized=fixture->adapter.initializeStageZero();
    REQUIRE(initialized.initialized);
    CHECK(initialized.truth_coverage>0.0);
    CHECK(initialized.truth_coverage<0.95);
    const auto reference=fixture->adapter.currentReferenceAudit();
    CHECK(reference.minimum_effective_reference_count>=2);
    CHECK(reference.minimum_robust_fim_cone_lower_bound>1.0e-6);
    CHECK(reference.maximum_posterior_eigenvalue<
          fixture->adapter.config().maximum_posterior_eigenvalue_m2);
    CHECK(reference.minimum_range_aoi_margin_s>=0.0);
    const auto rows=fixture->adapter.currentSnapshotHardRows(
        fixture->adapter.supervisor().topology());
    for (const auto owner : scenario.mobile_ids) {
        const auto exact=gf::evaluateProgressCompatibility(rows,owner,
            Eigen::Vector2d::Zero(),4.0,
            {std::numeric_limits<double>::max(),0.0,1e-10,true});
        INFO(scenario.id," owner=",owner," reason=",exact.reason);
        REQUIRE(exact.polytope_nonempty);
    }
}

}

TEST_CASE("Coastal scenarios keep all fixed anchors shore-side and outside denominator") {
    for (const auto scenario : {gf::task10p11hCoastalEasyScenario(),
                                gf::task10p11hCoastalNonbindingScenario(),
                                gf::task10p11hCoastalBindingScenario()}) {
        CHECK(scenario.fixed_positions==gf::task10p11hCoastalAnchors());
        CHECK(scenario.height_m>0.0);
        CHECK(scenario.initial_topology.size()==2*scenario.mobile_ids.size());
    }
}

TEST_CASE("Leader-only easy readiness contains both immutable seven-UAV branches") {
    const auto scenario=gf::task10p11hCoastalLeaderEasyScenario();
    CHECK(scenario.width_m==doctest::Approx(400.0));
    CHECK(scenario.height_m==doctest::Approx(600.0));
    CHECK(scenario.mobile_ids==gf::task10p10MobileIds(14));
    REQUIRE(scenario.mobile_positions.size()==14);
    CHECK(scenario.initial_topology.size()==28);
    CHECK(std::find(scenario.mobile_ids.begin(),scenario.mobile_ids.end(),7)!=
          scenario.mobile_ids.end());
    CHECK(std::find(scenario.mobile_ids.begin(),scenario.mobile_ids.end(),14)!=
          scenario.mobile_ids.end());
}

TEST_CASE("Coastal initial DAG and all hard qualification gates pass both profiles") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        verifyCoastalScenario(
            gf::task10p11hCoastalLeaderEasyScenario(),profile);
        verifyCoastalScenario(gf::task10p11hCoastalEasyScenario(),profile);
        verifyCoastalScenario(gf::task10p11hCoastalNonbindingScenario(),profile);
        verifyCoastalScenario(gf::task10p11hCoastalBindingScenario(),profile);
    }
}

TEST_CASE("Coastal nonbinding geometry keeps two shore references within 850 everywhere") {
    const std::vector<Eigen::Vector2d> corners={
        {0.0,0.0},{500.0,0.0},{0.0,700.0},{500.0,700.0}};
    const auto anchors=gf::task10p11hCoastalAnchors();
    for (const auto& point : corners) {
        std::vector<double> distance;
        for (const auto& [id,anchor] : anchors) {
            (void)id; distance.push_back((point-anchor).norm());
        }
        std::sort(distance.begin(),distance.end());
        CHECK(distance[1]<849.0);
    }
}

TEST_CASE("Coastal binding necessarily activates shore-reference distance offshore") {
    const Eigen::Vector2d far_cell(250.0,1795.0);
    for (const auto& [id,anchor] : gf::task10p11hCoastalAnchors()) {
        (void)id;
        CHECK((far_cell-anchor).norm()>850.0);
    }
}

TEST_CASE("Coastal binding active snapshot has no sea anchor and a strict mobile relay replacement") {
    const auto scenario=gf::task10p11hCoastalBindingActiveScenario();
    CHECK(scenario.fixed_positions==gf::task10p11hCoastalAnchors());
    for (const auto& [id,anchor] : scenario.fixed_positions) {
        (void)id;
        CHECK(anchor.y()<0.0);
    }
    const auto owner_position=scenario.mobile_positions.at(13);
    const auto old_reference=scenario.mobile_positions.at(10);
    const auto new_reference=scenario.mobile_positions.at(12);
    CHECK((owner_position-old_reference).norm()<849.0);
    CHECK((owner_position-old_reference).norm()>750.0);
    CHECK((owner_position-new_reference).norm()<749.0);
    const auto has_edge=[&](gf::DirectedEdge query) {
        return std::any_of(scenario.initial_topology.begin(),
            scenario.initial_topology.end(),[&](const auto& edge) {
                return edge.id()==query.id();
            });
    };
    CHECK(has_edge({11,14}));
    CHECK_FALSE(has_edge({13,14}));
}
