#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/TransitionCertifier.hpp"

#include <map>
#include <string>
#include <vector>

namespace {

std::vector<gf::DirectedEdge> oldGraph() {
    return {{10, 1}, {11, 1}, {10, 2}, {1, 2}, {11, 3}, {2, 3}};
}

gf::TransitionCertificationContext context() {
    gf::TransitionCertificationContext context;
    context.topology_version = 7;
    context.estimator_version = 13;
    context.mobile_ids = {1, 2, 3};
    context.fixed_ids = {10, 11};
    context.r_max = 2;
    context.max_reference_distance_m = 850.0;
    context.min_fim_eigenvalue = 1e-6;
    context.max_posterior_eigenvalue_m2 = 0.1;
    context.gamma_accept = 0.1;

    Eigen::VectorXd mean(12);
    mean << 100.0, 50.0, 0.0, 0.0,
            849.0, 100.0, 0.0, 0.0,
            700.0, 300.0, 0.0, 0.0;
    context.estimate = gf::JointEstimateSnapshot{
        {1, 2, 3}, mean, 0.01 * Eigen::MatrixXd::Identity(12, 12),
        {{10, Eigen::Vector2d(800.0, 0.0)},
         {11, Eigen::Vector2d(0.0, 100.0)}}};
    context.range_variances_m2 = {
        {"1--10", 1.0}, {"1--11", 1.0}, {"1--2", 1.0},
        {"2--10", 1.0}, {"2--11", 1.0}, {"2--3", 1.0},
        {"3--11", 1.0}};

    for (const gf::DirectedEdge& edge : {
             gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1},
             gf::DirectedEdge{10, 2}, gf::DirectedEdge{11, 2},
             gf::DirectedEdge{1, 2}, gf::DirectedEdge{3, 2},
             gf::DirectedEdge{11, 3}, gf::DirectedEdge{2, 3}}) {
        const Eigen::Vector2d owner = gf::detail::nodePosition(
            context.estimate, edge.owner);
        const Eigen::Vector2d reference = gf::detail::nodePosition(
            context.estimate, edge.reference);
        context.edge_gates[edge.id()] = {
            true, true, (owner - reference).norm()};
    }

    gf::CanonicalHardRowRequest hard_rows;
    hard_rows.mobile_ids = context.mobile_ids;
    hard_rows.fixed_ids = context.fixed_ids;
    hard_rows.states = {
        {1, {Point(100.0, 50.0), Eigen::Vector2d::Zero(),
             Eigen::Vector2d::Zero()}},
        {2, {Point(849.0, 100.0), Eigen::Vector2d::Zero(),
             Eigen::Vector2d::Zero()}},
        {3, {Point(700.0, 300.0), Eigen::Vector2d::Zero(),
             Eigen::Vector2d::Zero()}},
        {10, {Point(800.0, 0.0), Eigen::Vector2d::Zero(),
              Eigen::Vector2d::Zero()}},
        {11, {Point(0.0, 100.0), Eigen::Vector2d::Zero(),
              Eigen::Vector2d::Zero()}},
    };
    hard_rows.collision_pairs = {
        gf::UndirectedEdge::canonical(1, 2),
        gf::UndirectedEdge::canonical(1, 3),
        gf::UndirectedEdge::canonical(2, 3)};
    hard_rows.reference_spec = {
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0, 0.0, 1.0, 1.0, 1.0, 0.0};
    hard_rows.collision_spec = {
        PairwiseSecondOrderBarrierKind::CollisionLower,
        20.0, 0.0, 1.0, 1.0, 1.0, 0.0};
    hard_rows.acceleration_half_box = 2.0;
    context.hard_row_request = hard_rows;
    return context;
}

gf::TransitionProposal successfulProposal() {
    return gf::TransitionProposal{
        oldGraph(), {11, 2}, {10, 2}, 7, 13};
}

}  // namespace

TEST_CASE("A certified replacement validates old union successor and reverse states") {
    const auto certificate = gf::TransitionCertifier{}.certify(
        successfulProposal(), context(), true);

    REQUIRE(certificate.valid);
    CHECK(certificate.reverse_valid);
    CHECK(certificate.old_state.valid);
    CHECK(certificate.union_state.valid);
    CHECK(certificate.successor_state.valid);
    CHECK(certificate.union_edges.size() == oldGraph().size() + 1);
    CHECK(certificate.successor_edges.size() == oldGraph().size());
    CHECK(certificate.minimum_gamma >= doctest::Approx(0.1));
}

TEST_CASE("Forward certification rejects cycle union budget and one-reference successor") {
    SUBCASE("add edge closes a mobile cycle") {
        auto proposal = successfulProposal();
        proposal.new_edge = {3, 2};
        const auto result = gf::TransitionCertifier{}.certify(
            proposal, context(), false);
        CHECK_FALSE(result.valid);
        CHECK(result.union_state.reason == "dag");
    }
    SUBCASE("new near-limit row violates strict gamma accept") {
        auto strict = context();
        strict.gamma_accept = 4.0;
        const auto result = gf::TransitionCertifier{}.certify(
            successfulProposal(), strict, false);
        CHECK_FALSE(result.valid);
        CHECK(result.old_state.valid);
        CHECK(result.union_state.reason == "gamma_accept");
    }
    SUBCASE("deleting after a duplicate add leaves one reference") {
        auto proposal = successfulProposal();
        proposal.new_edge = {1, 2};
        const auto result = gf::TransitionCertifier{}.certify(
            proposal, context(), false);
        CHECK_FALSE(result.valid);
        CHECK(result.reason == "invalid_replacement");
    }
}

TEST_CASE("Distance FIM posterior and snapshot versions are fail-closed") {
    SUBCASE("850 metre hard gate") {
        auto invalid = context();
        invalid.edge_gates.at("11->2").robust_distance_m = 850.01;
        const auto result = gf::TransitionCertifier{}.certify(
            successfulProposal(), invalid, false);
        CHECK_FALSE(result.valid);
        CHECK(result.union_state.reason == "reference_distance");
    }
    SUBCASE("FIM gate") {
        auto invalid = context();
        invalid.min_fim_eigenvalue = 1e3;
        const auto result = gf::TransitionCertifier{}.certify(
            successfulProposal(), invalid, false);
        CHECK_FALSE(result.valid);
        CHECK(result.old_state.reason == "fim");
    }
    SUBCASE("posterior gate") {
        auto invalid = context();
        invalid.max_posterior_eigenvalue_m2 = 1e-4;
        const auto result = gf::TransitionCertifier{}.certify(
            successfulProposal(), invalid, false);
        CHECK_FALSE(result.valid);
        CHECK(result.old_state.reason == "posterior");
    }
    SUBCASE("stale estimator snapshot") {
        auto proposal = successfulProposal();
        proposal.expected_estimator_version = 12;
        const auto result = gf::TransitionCertifier{}.certify(
            proposal, context(), false);
        CHECK_FALSE(result.valid);
        CHECK(result.reason == "stale_snapshot");
    }
}

TEST_CASE("Guaranteed certification requires the old edge to remain add-eligible") {
    auto reverse_blocked = context();
    reverse_blocked.edge_gates.at("10->2").add_valid = false;
    const auto result = gf::TransitionCertifier{}.certify(
        successfulProposal(), reverse_blocked, true);

    CHECK_FALSE(result.valid);
    CHECK(result.forward_valid);
    CHECK_FALSE(result.reverse_valid);
    CHECK(result.reason == "reverse_certificate");
}

TEST_CASE("A transition is exactly one same-owner replacement") {
    SUBCASE("the removed edge must belong to the old graph") {
        auto proposal = successfulProposal();
        proposal.old_edge = {3, 2};
        auto permissive = context();
        permissive.r_max = 3;
        const auto result = gf::TransitionCertifier{}.certify(
            proposal, permissive, false);
        CHECK_FALSE(result.valid);
        CHECK(result.reason == "invalid_replacement");
    }
    SUBCASE("the added edge must be new") {
        auto proposal = successfulProposal();
        proposal.new_edge = {1, 2};
        const auto result = gf::TransitionCertifier{}.certify(
            proposal, context(), false);
        CHECK_FALSE(result.valid);
        CHECK(result.reason == "invalid_replacement");
    }
    SUBCASE("add and remove must have the same owner") {
        auto proposal = successfulProposal();
        proposal.new_edge = {3, 2};
        proposal.old_edge = {11, 3};
        const auto result = gf::TransitionCertifier{}.certify(
            proposal, context(), false);
        CHECK_FALSE(result.valid);
        CHECK(result.reason == "invalid_replacement");
    }
}

TEST_CASE("A feasible affine HOCBF row cannot certify outside the extended safe set") {
    auto unsafe = context();
    unsafe.hard_row_request.states.at(2).position = Point(105.0, 50.0);
    unsafe.hard_row_request.states.at(2).velocity << 20.0, 0.0;
    unsafe.estimate.mean.segment<2>(4) << 105.0, 50.0;
    unsafe.estimate.mean.segment<2>(6) << 20.0, 0.0;
    for (auto& [id, gate] : unsafe.edge_gates) {
        (void)id;
        gate.robust_distance_m = 1.0;
    }
    const auto result = gf::TransitionCertifier{}.certify(
        successfulProposal(), unsafe, false);
    CHECK_FALSE(result.valid);
    CHECK(result.old_state.reason == "collision_initial_set");
}
