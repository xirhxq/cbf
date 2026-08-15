#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalHardRows.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace {

gf::CanonicalHardRowRequest requestWith(
    std::vector<gf::DirectedEdge> references) {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids = {1, 2};
    request.fixed_ids = {10, 11};
    request.states = {
        {1, {Point(100.0, 0.0), Eigen::Vector2d(0.1, 0.0),
             Eigen::Vector2d::Zero()}},
        {2, {Point(200.0, 40.0), Eigen::Vector2d(-0.1, 0.0),
             Eigen::Vector2d::Zero()}},
        {10, {Point(0.0, 0.0), Eigen::Vector2d::Zero(),
              Eigen::Vector2d::Zero()}},
        {11, {Point(0.0, 100.0), Eigen::Vector2d::Zero(),
              Eigen::Vector2d::Zero()}},
    };
    request.reference_edges = std::move(references);
    request.collision_pairs = {gf::UndirectedEdge::canonical(1, 2)};
    request.reference_spec = {
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0, 2.0, 1.0, 1.0, 1.0, 0.0};
    request.collision_spec = {
        PairwiseSecondOrderBarrierKind::CollisionLower,
        20.0, 1.0, 1.0, 1.0, 1.0, 0.0};
    request.acceleration_half_box = 2.0;
    return request;
}

std::set<std::string> ids(const std::vector<gf::CanonicalHardRow>& rows) {
    std::set<std::string> result;
    for (const auto& row : rows) result.insert(row.id);
    return result;
}

double independentVertexGamma(
    const std::vector<gf::CanonicalHardRow>& rows,
    gf::NodeId owner,
    double half_box) {
    std::vector<const gf::CanonicalHardRow*> active;
    for (const auto& row : rows) {
        if (row.owner == owner && row.participates_in_gamma) active.push_back(&row);
    }
    std::vector<Eigen::Vector2d> candidates = {
        {-half_box, -half_box}, {-half_box, half_box},
        {half_box, -half_box}, {half_box, half_box}, {0.0, 0.0}};
    for (std::size_t first = 0; first < active.size(); ++first) {
        for (std::size_t second = first + 1; second < active.size(); ++second) {
            const Eigen::Vector2d first_difference =
                active[first]->control_coefficient -
                active[second]->control_coefficient;
            const double first_rhs =
                active[second]->constant - active[first]->constant;
            for (int axis = 0; axis < 2; ++axis) {
                for (double face : {-half_box, half_box}) {
                    Eigen::Matrix2d boundary;
                    boundary.row(0) = first_difference.transpose();
                    boundary.row(1).setZero();
                    boundary(1, axis) = 1.0;
                    if (std::abs(boundary.determinant()) < 1e-12) continue;
                    const Eigen::Vector2d point =
                        boundary.fullPivLu().solve(
                            Eigen::Vector2d(first_rhs, face));
                    if (point.cwiseAbs().maxCoeff() <= half_box + 1e-12)
                        candidates.push_back(point);
                }
            }
            for (std::size_t third = second + 1; third < active.size(); ++third) {
                Eigen::Matrix2d intersection;
                intersection.row(0) = first_difference.transpose();
                intersection.row(1) =
                    (active[first]->control_coefficient -
                     active[third]->control_coefficient).transpose();
                if (std::abs(intersection.determinant()) < 1e-12) continue;
                const Eigen::Vector2d point = intersection.fullPivLu().solve(
                    Eigen::Vector2d(
                        first_rhs,
                        active[third]->constant - active[first]->constant));
                if (point.cwiseAbs().maxCoeff() <= half_box + 1e-12)
                    candidates.push_back(point);
            }
        }
    }
    double optimum = -std::numeric_limits<double>::infinity();
    for (const Eigen::Vector2d& point : candidates) {
        double margin = std::numeric_limits<double>::infinity();
        for (const auto* row : active) margin = std::min(margin, row->margin(point));
        optimum = std::max(optimum, margin);
    }
    return optimum;
}

}  // namespace

TEST_CASE("Canonical rows preserve collision and input gates across graph states") {
    const std::vector<gf::DirectedEdge> old_edges = {
        {10, 1}, {11, 1}, {10, 2}, {1, 2}};
    auto union_edges = old_edges;
    union_edges.push_back({11, 2});
    const std::vector<gf::DirectedEdge> successor_edges = {
        {10, 1}, {11, 1}, {1, 2}, {11, 2}};

    const auto old_rows = gf::buildCanonicalHardRows(requestWith(old_edges));
    const auto union_rows = gf::buildCanonicalHardRows(requestWith(union_edges));
    const auto successor_rows =
        gf::buildCanonicalHardRows(requestWith(successor_edges));

    for (const auto& rows : {old_rows, union_rows, successor_rows}) {
        const auto row_ids = ids(rows);
        CHECK(std::is_sorted(rows.begin(), rows.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.id < rhs.id;
        }));
        CHECK(row_ids.count("collision:1--2:owner:1") == 1);
        CHECK(row_ids.count("collision:1--2:owner:2") == 1);
        CHECK(row_ids.count("input:1:ax:lower") == 1);
        CHECK(row_ids.count("input:1:ax:upper") == 1);
        CHECK(row_ids.count("input:1:ay:lower") == 1);
        CHECK(row_ids.count("input:1:ay:upper") == 1);
        for (gf::NodeId owner : {gf::NodeId{1}, gf::NodeId{2}}) {
            const auto exact = gf::solveCanonicalGammaStar(rows, owner, 2.0);
            REQUIRE(exact.valid);
            CHECK(exact.gamma == doctest::Approx(
                independentVertexGamma(rows, owner, 2.0)).epsilon(1e-10));
        }
    }
    CHECK(ids(old_rows).count("reference:10->2:owner:2") == 1);
    CHECK(ids(union_rows).count("reference:11->2:owner:2") == 1);
    CHECK(ids(successor_rows).count("reference:10->2:owner:2") == 0);
}

TEST_CASE("Canonical physical rows require and expose complete snapshot tubes") {
    auto request = requestWith({{10, 1}, {1, 2}});
    request.require_snapshot_robust_rows = true;
    request.reference_snapshot_tubes = {
        {"10->1", {0.30, 0.20,
                   gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment}},
        {"1->2", {0.50, 0.35,
                  gf::SnapshotTubeProvenance::ExternallyCertified}}};
    request.collision_snapshot_tubes = {
        {"1--2", {0.50, 0.35,
                  gf::SnapshotTubeProvenance::ExternallyCertified}}};
    const auto rows = gf::buildCanonicalHardRows(request);
    const auto fixed = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:10->1:owner:1";
    });
    const auto mobile_first = std::find_if(
        rows.begin(), rows.end(), [](const auto& row) {
            return row.id == "reference:1->2:owner:1";
        });
    const auto mobile_second = std::find_if(
        rows.begin(), rows.end(), [](const auto& row) {
            return row.id == "reference:1->2:owner:2";
        });
    REQUIRE(fixed != rows.end());
    REQUIRE(mobile_first != rows.end());
    REQUIRE(mobile_second != rows.end());
    REQUIRE(fixed->tube_provenance.has_value());
    CHECK(*fixed->tube_provenance ==
          gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment);
    CHECK(std::isfinite(fixed->barrier_hdot));
    CHECK(fixed->coefficient_uncertainty_reserve > 0.0);
    CHECK(mobile_first->responsibility == doctest::Approx(0.5));
    CHECK(mobile_second->responsibility == doctest::Approx(0.5));
    CHECK(mobile_first->coefficient_uncertainty_reserve > 0.0);
    CHECK(mobile_second->coefficient_uncertainty_reserve > 0.0);

    auto missing = request;
    missing.reference_snapshot_tubes.erase("1->2");
    CHECK_THROWS_WITH_AS(
        gf::buildCanonicalHardRows(missing),
        "missing reference snapshot tube", std::invalid_argument);
}

TEST_CASE("Robust canonical half rows retain full local coefficient reserves") {
    auto request = requestWith({{1, 2}});
    request.require_snapshot_robust_rows = true;
    request.reference_snapshot_tubes = {
        {"1->2", {0.7, 0.4,
                  gf::SnapshotTubeProvenance::ExternallyCertified}}};
    request.collision_snapshot_tubes = {
        {"1--2", {0.7, 0.4,
                  gf::SnapshotTubeProvenance::ExternallyCertified}}};
    const auto rows = gf::buildCanonicalHardRows(request);
    const auto first = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:1->2:owner:1";
    });
    const auto second = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:1->2:owner:2";
    });
    REQUIRE(first != rows.end());
    REQUIRE(second != rows.end());
    CHECK(first->coefficient_uncertainty_reserve == doctest::Approx(
        second->coefficient_uncertainty_reserve));
    CHECK(first->coefficient_uncertainty_reserve > 0.0);

    const auto robust = gf::buildSnapshotRobustPairRow(
        request.states.at(1), request.states.at(2), request.reference_spec,
        request.reference_snapshot_tubes.at("1->2"),
        request.acceleration_half_box);
    CHECK(first->constant + second->constant == doctest::Approx(
        robust.central_constant_lower - 2.0 * robust.coefficient_reserve));
}

TEST_CASE("Mobile pair responsibility reconstructs the centralized coupled row") {
    const auto rows = gf::buildCanonicalHardRows(
        requestWith({{10, 1}, {11, 1}, {1, 2}, {10, 2}}));
    const auto owner = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:1->2:owner:2";
    });
    const auto reference = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:1->2:owner:1";
    });
    REQUIRE(owner != rows.end());
    REQUIRE(reference != rows.end());
    CHECK(owner->responsibility == doctest::Approx(0.5));
    CHECK(reference->responsibility == doctest::Approx(0.5));

    auto physical_request = requestWith({{1, 2}});
    auto first = physical_request.states.at(2);
    auto second = physical_request.states.at(1);
    first.acceleration.setZero();
    second.acceleration.setZero();
    const auto centralized = buildPairwiseSecondOrderRow(
        first, second, physical_request.reference_spec);
    CHECK((owner->control_coefficient - centralized.uCoe).norm()
          == doctest::Approx(0.0).epsilon(1e-12));
    CHECK((reference->control_coefficient + centralized.uCoe).norm()
          == doctest::Approx(0.0).epsilon(1e-12));
    CHECK((owner->control_coefficient + reference->control_coefficient).norm()
          == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(owner->constant + reference->constant
          == doctest::Approx(centralized.constTerm).epsilon(1e-12));

    const auto collision_first = std::find_if(
        rows.begin(), rows.end(), [](const auto& row) {
            return row.id == "collision:1--2:owner:1";
        });
    const auto collision_second = std::find_if(
        rows.begin(), rows.end(), [](const auto& row) {
            return row.id == "collision:1--2:owner:2";
        });
    REQUIRE(collision_first != rows.end());
    REQUIRE(collision_second != rows.end());
    CHECK(collision_first->responsibility == doctest::Approx(0.5));
    CHECK(collision_second->responsibility == doctest::Approx(0.5));
    auto collision_request = requestWith({});
    auto collision_owner = collision_request.states.at(1);
    auto collision_peer = collision_request.states.at(2);
    collision_owner.acceleration.setZero();
    collision_peer.acceleration.setZero();
    const auto centralized_collision = buildPairwiseSecondOrderRow(
        collision_owner, collision_peer, collision_request.collision_spec);
    CHECK(collision_first->constant + collision_second->constant
          == doctest::Approx(centralized_collision.constTerm).epsilon(1e-12));

    const auto fixed = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:10->1:owner:1";
    });
    REQUIRE(fixed != rows.end());
    CHECK(fixed->responsibility == doctest::Approx(1.0));
}

TEST_CASE("Two local half-row controls imply each centralized physical pair row") {
    const auto request = requestWith({{1, 2}, {10, 1}});
    const auto rows = gf::buildCanonicalHardRows(request);
    const auto verify_pair = [&](const std::string& first_id,
                                 const std::string& second_id,
                                 const Eigen::Vector2d& first_control,
                                 const Eigen::Vector2d& second_control,
                                 const auto& centralized) {
        const auto first = std::find_if(rows.begin(), rows.end(), [&](const auto& row) {
            return row.id == first_id;
        });
        const auto second = std::find_if(rows.begin(), rows.end(), [&](const auto& row) {
            return row.id == second_id;
        });
        REQUIRE(first != rows.end());
        REQUIRE(second != rows.end());
        const double first_margin = first->margin(first_control);
        const double second_margin = second->margin(second_control);
        REQUIRE(first_margin >= 0.0);
        REQUIRE(second_margin >= 0.0);
        const double coupled = centralized.uCoe.dot(first_control)
                             - centralized.uCoe.dot(second_control)
                             + centralized.constTerm;
        CHECK(first_margin + second_margin
              == doctest::Approx(coupled).epsilon(1e-12));
        CHECK(coupled >= 0.0);
    };

    auto communication_owner = request.states.at(2);
    auto communication_reference = request.states.at(1);
    communication_owner.acceleration.setZero();
    communication_reference.acceleration.setZero();
    const auto communication = buildPairwiseSecondOrderRow(
        communication_owner, communication_reference, request.reference_spec);
    verify_pair(
        "reference:1->2:owner:2", "reference:1->2:owner:1",
        {0.2, 0.1}, {-0.2, -0.1}, communication);

    auto collision_first_state = request.states.at(1);
    auto collision_second_state = request.states.at(2);
    collision_first_state.acceleration.setZero();
    collision_second_state.acceleration.setZero();
    const auto collision = buildPairwiseSecondOrderRow(
        collision_first_state, collision_second_state, request.collision_spec);
    verify_pair(
        "collision:1--2:owner:1", "collision:1--2:owner:2",
        {-0.2, -0.1}, {0.2, 0.1}, collision);

    const auto fixed = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:10->1:owner:1";
    });
    REQUIRE(fixed != rows.end());
    CHECK(fixed->responsibility == doctest::Approx(1.0));
    CHECK(fixed->constant != doctest::Approx(0.5 * fixed->constant));
}

TEST_CASE("Half-row composition covers equality and detects a violated local premise") {
    auto request = requestWith({{1, 2}, {10, 1}});
    const auto rows = gf::buildCanonicalHardRows(request);
    const auto owner = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:1->2:owner:2";
    });
    const auto peer = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:1->2:owner:1";
    });
    REQUIRE(owner != rows.end());
    REQUIRE(peer != rows.end());
    const Eigen::Vector2d owner_boundary =
        -owner->constant * owner->control_coefficient /
        owner->control_coefficient.squaredNorm();
    const Eigen::Vector2d peer_boundary =
        -peer->constant * peer->control_coefficient /
        peer->control_coefficient.squaredNorm();
    CHECK(owner->margin(owner_boundary) == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(peer->margin(peer_boundary) == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(owner->margin(owner_boundary) + peer->margin(peer_boundary)
          == doctest::Approx(0.0).epsilon(1e-10));

    const Eigen::Vector2d violated = owner_boundary -
        1e-3 * owner->control_coefficient.normalized();
    CHECK(owner->margin(violated) < 0.0);
    const bool both_local_premises_hold =
        owner->margin(violated) >= 0.0 &&
        peer->margin(peer_boundary) >= 0.0;
    CHECK_FALSE(both_local_premises_hold);

    const auto fixed = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "reference:10->1:owner:1";
    });
    REQUIRE(fixed != rows.end());
    auto mobile = request.states.at(1);
    auto anchor = request.states.at(10);
    mobile.acceleration.setZero();
    anchor.acceleration.setZero();
    const auto centralized = buildPairwiseSecondOrderRow(
        mobile, anchor, request.reference_spec);
    CHECK((fixed->control_coefficient - centralized.uCoe).norm()
          == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(fixed->constant
          == doctest::Approx(centralized.constTerm).epsilon(1e-12));

    const auto collision_first = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "collision:1--2:owner:1";
    });
    const auto collision_second = std::find_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.id == "collision:1--2:owner:2";
    });
    REQUIRE(collision_first != rows.end());
    REQUIRE(collision_second != rows.end());
    const Eigen::Vector2d collision_boundary_first =
        -collision_first->constant * collision_first->control_coefficient /
        collision_first->control_coefficient.squaredNorm();
    const Eigen::Vector2d collision_boundary_second =
        -collision_second->constant * collision_second->control_coefficient /
        collision_second->control_coefficient.squaredNorm();
    CHECK(collision_first->margin(collision_boundary_first)
          == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(collision_second->margin(collision_boundary_second)
          == doctest::Approx(0.0).epsilon(1e-10));
    const Eigen::Vector2d collision_violated = collision_boundary_first -
        1e-3 * collision_first->control_coefficient.normalized();
    CHECK(collision_first->margin(collision_violated) < 0.0);

    auto fixed_collision_request = requestWith({});
    fixed_collision_request.collision_pairs.push_back(
        gf::UndirectedEdge::canonical(1, 10));
    const auto fixed_collision_rows = gf::buildCanonicalHardRows(
        fixed_collision_request);
    const auto fixed_collision = std::find_if(
        fixed_collision_rows.begin(), fixed_collision_rows.end(), [](const auto& row) {
            return row.id == "collision:1--10:owner:1";
        });
    REQUIRE(fixed_collision != fixed_collision_rows.end());
    auto mobile_collision = fixed_collision_request.states.at(1);
    auto anchor_collision = fixed_collision_request.states.at(10);
    mobile_collision.acceleration.setZero();
    anchor_collision.acceleration.setZero();
    const auto centralized_fixed_collision = buildPairwiseSecondOrderRow(
        mobile_collision, anchor_collision,
        fixed_collision_request.collision_spec);
    CHECK(fixed_collision->responsibility == doctest::Approx(1.0));
    CHECK((fixed_collision->control_coefficient -
           centralized_fixed_collision.uCoe).norm()
          == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(fixed_collision->constant
          == doctest::Approx(
              centralized_fixed_collision.constTerm).epsilon(1e-12));
}

TEST_CASE("Canonical gamma-star matches an independent vertex oracle for all signs") {
    const auto check_sign = [](double constant) {
        std::vector<gf::CanonicalHardRow> rows = {
            gf::makeCanonicalGammaRow("x+", 1, {1.0, 0.0}, constant),
            gf::makeCanonicalGammaRow("x-", 1, {-1.0, 0.0}, constant),
            gf::makeCanonicalGammaRow("y+", 1, {0.0, 1.0}, constant),
            gf::makeCanonicalGammaRow("y-", 1, {0.0, -1.0}, constant)};
        const auto exact = gf::solveCanonicalGammaStar(rows, 1, 1.0);
        REQUIRE(exact.valid);
        CHECK(exact.gamma == doctest::Approx(
            independentVertexGamma(rows, 1, 1.0)).epsilon(1e-12));
        CHECK(exact.gamma == doctest::Approx(constant).epsilon(1e-12));
        return exact.gamma;
    };
    CHECK(check_sign(0.5) > 0.0);
    CHECK(check_sign(0.0) == 0.0);
    CHECK(check_sign(-0.5) < 0.0);
}

TEST_CASE("Degenerate rows remain exact and coincident physical geometry fails closed") {
    const std::vector<gf::CanonicalHardRow> flat = {
        gf::makeCanonicalGammaRow("flat", 1, {0.0, 0.0}, 0.25)};
    const auto exact = gf::solveCanonicalGammaStar(flat, 1, 2.0);
    REQUIRE(exact.valid);
    CHECK(exact.gamma == doctest::Approx(0.25));
    CHECK(exact.gamma == doctest::Approx(independentVertexGamma(flat, 1, 2.0)));

    auto request = requestWith({{10, 1}, {11, 1}, {1, 2}, {10, 2}});
    request.states.at(2).position = request.states.at(1).position;
    CHECK_THROWS_AS(gf::buildCanonicalHardRows(request), std::invalid_argument);
}
TEST_CASE("Owner-focused canonical rows exactly equal filtering the full ledger") {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={1,2};
    request.fixed_ids={100};
    request.states[1]={{0.0,0.0},{0.1,0.0},Eigen::Vector2d::Zero()};
    request.states[2]={{10.0,0.0},{-0.1,0.0},Eigen::Vector2d::Zero()};
    request.states[100]={{5.0,5.0},Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    request.workspace_facets={{"x-upper",{1.0,0.0},20.0}};
    request.workspace_snapshot_tubes[1]={0.1,0.2};
    request.workspace_snapshot_tubes[2]={0.2,0.1};
    request.reference_edges={{1,2},{100,2}};
    request.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.reference_snapshot_tubes["1->2"]={0.1,0.1};
    request.reference_snapshot_tubes["100->2"]={0.2,0.2};
    request.collision_pairs={gf::UndirectedEdge::canonical(1,2),
        gf::UndirectedEdge::canonical(1,100)};
    request.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        0.5,0.0,1.0,1.0,1.0,0.0};
    request.collision_snapshot_tubes["1--2"]={0.1,0.1};
    request.collision_snapshot_tubes["1--100"]={0.2,0.2};
    request.acceleration_half_box=0.4;
    request.require_snapshot_robust_rows=true;

    const auto full=gf::buildCanonicalHardRows(request);
    for (gf::NodeId owner : request.mobile_ids) {
        std::vector<gf::CanonicalHardRow> filtered;
        std::copy_if(full.begin(),full.end(),std::back_inserter(filtered),
            [owner](const auto& row) { return row.owner==owner; });
        const auto focused=gf::buildCanonicalOwnerHardRows(request,owner);
        REQUIRE(focused.size()==filtered.size());
        for (std::size_t index=0;index<focused.size();++index) {
            CHECK(focused[index].id==filtered[index].id);
            CHECK(focused[index].kind==filtered[index].kind);
            CHECK(focused[index].owner==filtered[index].owner);
            CHECK(focused[index].peer==filtered[index].peer);
            CHECK((focused[index].normal-filtered[index].normal).norm()<=1e-12);
            CHECK((focused[index].control_coefficient-
                   filtered[index].control_coefficient).norm()<=1e-12);
            CHECK(focused[index].constant==doctest::Approx(
                filtered[index].constant).epsilon(1e-12));
            CHECK(focused[index].position_uncertainty_reserve_m==
                doctest::Approx(filtered[index].position_uncertainty_reserve_m));
            CHECK(focused[index].velocity_uncertainty_reserve_mps==
                doctest::Approx(filtered[index].velocity_uncertainty_reserve_mps));
            CHECK(focused[index].coefficient_uncertainty_reserve==
                doctest::Approx(filtered[index].coefficient_uncertainty_reserve));
        }
    }
}
