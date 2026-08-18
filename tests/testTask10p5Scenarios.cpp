#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GrandFinaleExperiment.hpp"

#include <algorithm>
#include <map>
#include <vector>

namespace {

struct Fixture4p2 {
    std::vector<gf::NodeId> mobiles{1, 2, 3, 4};
    std::vector<gf::NodeId> fixed_ids{10, 11};
    std::map<gf::NodeId, Eigen::Vector2d> fixed{
        {10, {-1.0, 0.0}}, {11, {-1.0, 4.0}}};
    gf::grand_finale_experiment_detail::TruthStateMap truth{
        {1, Eigen::Vector4d(1.0, 1.0, 0.0, 0.0)},
        {2, Eigen::Vector4d(1.0, 3.0, 0.0, 0.0)},
        {3, Eigen::Vector4d(3.0, 1.0, 0.0, 0.0)},
        {4, Eigen::Vector4d(3.0, 3.0, 0.0, 0.0)}};
    std::vector<gf::DirectedEdge> topology{
        {10, 1}, {11, 1}, {10, 2}, {1, 2},
        {11, 3}, {1, 3}, {2, 4}, {3, 4}};
    std::vector<gf::DirectedEdge> candidates{
        {10, 1}, {11, 1}, {10, 2}, {1, 2}, {11, 2}, {4, 2},
        {11, 3}, {1, 3}, {2, 4}, {3, 4}};

    gf::JointEstimateSnapshot estimate() const {
        return gf::InterimMasterDekf(
            gf::grand_finale_experiment_detail::estimates(truth), fixed)
            .reconstructForAudit();
    }

    gf::TransitionCertificationContext context(
        const gf::JointEstimateSnapshot& snapshot) const {
        gf::TransitionCertificationContext result;
        result.topology_version = 1;
        result.estimator_version = 0;
        result.mobile_ids = mobiles;
        result.fixed_ids = fixed_ids;
        result.r_max = 2;
        result.max_reference_distance_m = 850.0;
        result.min_fim_eigenvalue = 1e-6;
        result.max_posterior_eigenvalue_m2 = 0.1;
        result.gamma_accept = 0.05;
        result.estimate = snapshot;
        for (const auto& edge : candidates) {
            const std::string range_id = gf::UndirectedEdge::canonical(
                edge.reference, edge.owner).id();
            result.range_variances_m2[range_id] = 1.0;
            const double distance =
                (gf::detail::nodePosition(snapshot, edge.owner) -
                 gf::detail::nodePosition(snapshot, edge.reference)).norm();
            result.edge_gates[edge.id()] =
                gf::CertifiedEdgeGate{true, true, distance + 0.02};
            result.information_edges.push_back(edge);
        }
        result.information_range_variances_m2=result.range_variances_m2;
        result.hard_row_request =
            gf::grand_finale_experiment_detail::hardRowRequest(
                mobiles, fixed_ids, snapshot, topology);
        return result;
    }
};

double minimumGamma(const Fixture4p2& fixture) {
    const auto snapshot = fixture.estimate();
    const auto rows = gf::buildCanonicalHardRows(
        gf::grand_finale_experiment_detail::hardRowRequest(
            fixture.mobiles, fixture.fixed_ids, snapshot,
            fixture.topology));
    double minimum = std::numeric_limits<double>::infinity();
    for (gf::NodeId owner : fixture.mobiles) {
        const auto gamma = gf::solveCanonicalGammaStar(rows, owner, 0.4);
        REQUIRE(gamma.valid);
        minimum = std::min(minimum, gamma.gamma);
    }
    return minimum;
}

}  // namespace

TEST_CASE("Task 10.5 4+2 cycle candidate is rejected before installation") {
    const Fixture4p2 fixture;
    const auto result = gf::TransitionCertifier{}.certify(
        gf::TransitionProposal{
            fixture.topology, {4, 2}, {10, 2}, 1, 0},
        fixture.context(fixture.estimate()), false);

    CHECK_FALSE(result.valid);
    CHECK(result.union_state.reason == "dag");
}

TEST_CASE("Task 10.5 4+2 collinear information graph fails the local FIM hard gate") {
    const Fixture4p2 fixture;
    auto snapshot = fixture.estimate();
    snapshot.fixed_positions.at(10) = Eigen::Vector2d(-1.0, -0.5);
    snapshot.fixed_positions.at(11) = Eigen::Vector2d(-2.0, -0.5);
    snapshot.mean.segment<2>(0) << 1.0, -0.5;
    auto context = fixture.context(snapshot);
    context.hard_row_request.states.at(1).position = Point(1.0, -0.5);
    context.hard_row_request.states.at(10).position = Point(-1.0, -0.5);
    context.hard_row_request.states.at(11).position = Point(-2.0, -0.5);

    const auto result = gf::TransitionCertifier{}.certify(
        gf::TransitionProposal{
            fixture.topology, {11, 2}, {10, 2}, 1, 0},
        context, false);

    CHECK_FALSE(result.valid);
    CHECK(result.old_state.reason == "fim");
}

TEST_CASE("A transverse accepted information edge repairs collinear reference parents") {
    const Fixture4p2 fixture;
    auto snapshot=fixture.estimate();
    snapshot.fixed_positions.at(10)=Eigen::Vector2d(-1.0,-0.5);
    snapshot.fixed_positions.at(11)=Eigen::Vector2d(-2.0,-0.5);
    snapshot.mean.segment<2>(0)<<1.0,-0.5;
    auto context=fixture.context(snapshot);
    context.hard_row_request.states.at(1).position=Point(1.0,-0.5);
    context.hard_row_request.states.at(10).position=Point(-1.0,-0.5);
    context.hard_row_request.states.at(11).position=Point(-2.0,-0.5);
    context.information_edges.emplace_back(2,1);
    context.information_range_variances_m2["1--2"]=1.0;

    const auto result=gf::TransitionCertifier{}.certify(
        gf::TransitionProposal{
            fixture.topology,{11,2},{10,2},1,0},context,false);
    CHECK(result.old_state.valid);
}

TEST_CASE("Task 10.5 exact positive gamma warning triggers REFORM") {
    const Fixture4p2 fixture;
    const double gamma = minimumGamma(fixture);
    REQUIRE(gamma > 0.0);
    gf::HybridSupervisor supervisor({0.0, gamma + 0.01, gamma + 0.02});

    CHECK(supervisor.observeGamma(0.0, gamma, true, true) ==
          gf::SupervisorMode::Reform);
}

TEST_CASE("Task 10.5 no-candidate warning selects RETREAT or HOLD without softening") {
    const Fixture4p2 fixture;
    const double gamma = minimumGamma(fixture);
    REQUIRE(gamma > 0.0);

    gf::HybridSupervisor retreat({0.0, gamma + 0.01, gamma + 0.02});
    CHECK(retreat.observeGamma(0.0, gamma, false, true) ==
          gf::SupervisorMode::Retreat);

    gf::HybridSupervisor hold({0.0, gamma + 0.01, gamma + 0.02});
    CHECK(hold.observeGamma(0.0, gamma, false, false) ==
          gf::SupervisorMode::Hold);
}

TEST_CASE("Task 10.5 uncertainty containment makes 4+2 coverage conservative") {
    gf::CertifiedCoverageTracker coverage({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    for (int agent = 0; agent < 4; ++agent) {
        coverage.observe(
            Point(1.0, 1.0), Point(1.6, 1.0), 0.7, 1.6);
    }

    CHECK(coverage.truthCoveredCount() == 1);
    CHECK(coverage.certifiedCoveredCount() == 0);
    CHECK(coverage.certifiedSubsetOfTruth());
}
