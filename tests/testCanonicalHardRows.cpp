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
