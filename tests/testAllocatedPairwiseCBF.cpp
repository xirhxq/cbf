#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "cbf/AllocatedPairwiseCBF.hpp"

#include <limits>

namespace {

cbf2026::EdgeSnapshot localizationSnapshot() {
    return {
        {cbf2026::EdgeKind::Localization, 1, 2, -1},
        Eigen::Vector2d(0.6, 0.8),
        5.0,
        0.2,
        0.3,
        1.4,
        7,
        11
    };
}

cbf2026::EdgeSnapshot collisionSnapshot() {
    auto snapshot = localizationSnapshot();
    snapshot.edge.kind = cbf2026::EdgeKind::Collision;
    snapshot.alpha = 2.0;
    return snapshot;
}

}

TEST_CASE("collision endpoint rows sum to the sign-reversed coupled row") {
    const auto snapshot = collisionSnapshot();
    const auto rows = cbf2026::allocatedCollisionRows(
        snapshot, 0.5, 0.5
    );
    REQUIRE(rows.size() == 2);

    const auto full = cbf2026::reconstructFullRow(rows);
    CHECK(full.coefficientI.isApprox(snapshot.normal));
    CHECK(full.coefficientJ.isApprox(-snapshot.normal));
    CHECK(full.constant == doctest::Approx(
        -snapshot.nuI - snapshot.nuJ + snapshot.alpha
    ));
}

TEST_CASE("hovering base localization assigns the complete row to the UAV") {
    auto snapshot = localizationSnapshot();
    snapshot.edge = {
        cbf2026::EdgeKind::Localization,
        2,
        2,
        4
    };
    snapshot.nuI = 0.25;
    snapshot.nuJ = 0.0;
    snapshot.alpha = 0.9;

    const auto rows = cbf2026::allocatedLocalizationRows(
        snapshot, 1.0, 0.0
    );
    REQUIRE(rows.size() == 1);
    CHECK(rows.front().owner == 2);
    CHECK(rows.front().coefficient.isApprox(-snapshot.normal));
    CHECK(rows.front().constant == doctest::Approx(0.65));
    CHECK(rows.front().allocation == doctest::Approx(1.0));
}

TEST_CASE("edge snapshots derive the canonical normal from ordered positions") {
    const auto pair = cbf2026::makeEdgeSnapshot(
        cbf2026::canonicalUavEdge(
            cbf2026::EdgeKind::Localization, 2, 1
        ),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(4.0, 4.0),
        0.2,
        0.3,
        1.0,
        7,
        11
    );
    CHECK(pair.normal.isApprox(Eigen::Vector2d(-0.6, -0.8)));
    CHECK(pair.separation == doctest::Approx(5.0));

    const auto base = cbf2026::makeEdgeSnapshot(
        cbf2026::canonicalBaseLocalizationEdge(2, 4),
        Eigen::Vector2d(4.0, 4.0),
        Eigen::Vector2d(1.0, 0.0),
        0.2,
        0.0,
        1.0,
        7,
        11
    );
    CHECK(base.normal.isApprox(Eigen::Vector2d(0.6, 0.8)));
    CHECK(base.separation == doctest::Approx(5.0));
}

TEST_CASE("allocated rows reject invalid responsibility shares") {
    const auto pair = localizationSnapshot();
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(pair, -0.1, 1.1),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(
            pair,
            std::numeric_limits<double>::quiet_NaN(),
            0.5
        ),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(pair, 0.4, 0.5),
        std::invalid_argument
    );

    auto base = pair;
    base.edge = cbf2026::canonicalBaseLocalizationEdge(1, 0);
    base.nuJ = 0.0;
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(base, 0.5, 0.5),
        std::invalid_argument
    );
}

TEST_CASE("allocated rows reject malformed shared edge snapshots") {
    auto snapshot = localizationSnapshot();
    snapshot.separation = 0.0;
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(snapshot, 0.5, 0.5),
        std::invalid_argument
    );

    snapshot = localizationSnapshot();
    snapshot.normal = Eigen::Vector2d(2.0, 0.0);
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(snapshot, 0.5, 0.5),
        std::invalid_argument
    );

    snapshot = localizationSnapshot();
    snapshot.nuI = -0.1;
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(snapshot, 0.5, 0.5),
        std::invalid_argument
    );

    snapshot = localizationSnapshot();
    snapshot.alpha = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(
        cbf2026::allocatedLocalizationRows(snapshot, 0.5, 0.5),
        std::invalid_argument
    );

    CHECK_THROWS_AS(
        cbf2026::allocatedCollisionRows(
            localizationSnapshot(), 0.5, 0.5
        ),
        std::invalid_argument
    );
}

TEST_CASE("single hovering-base endpoint independently reconstructs its full row") {
    const auto snapshot = cbf2026::makeEdgeSnapshot(
        cbf2026::canonicalBaseLocalizationEdge(2, 4),
        Eigen::Vector2d(3.0, 4.0),
        Eigen::Vector2d::Zero(),
        0.25,
        0.0,
        0.9,
        7,
        11
    );
    const auto rows = cbf2026::allocatedLocalizationRows(
        snapshot, 1.0, 0.0
    );
    REQUIRE(rows.size() == 1);

    const auto full = cbf2026::reconstructFullRow(rows);
    CHECK(full.coefficientI.isApprox(-snapshot.normal));
    CHECK(full.coefficientJ.isZero());
    CHECK(full.constant == doctest::Approx(0.65));
}

TEST_CASE("full-row reconstruction rejects mixed or duplicated endpoint rows") {
    const auto valid = cbf2026::allocatedLocalizationRows(
        localizationSnapshot(), 0.5, 0.5
    );

    auto rows = valid;
    rows[1].owner = rows[0].owner;
    CHECK_THROWS_AS(
        cbf2026::reconstructFullRow(rows),
        std::invalid_argument
    );

    rows = valid;
    rows[1].snapshotVersion += 1;
    CHECK_THROWS_AS(
        cbf2026::reconstructFullRow(rows),
        std::invalid_argument
    );

    rows = valid;
    rows[1].allocationVersion += 1;
    CHECK_THROWS_AS(
        cbf2026::reconstructFullRow(rows),
        std::invalid_argument
    );

    rows = valid;
    rows[1].edge.high += 1;
    CHECK_THROWS_AS(
        cbf2026::reconstructFullRow(rows),
        std::invalid_argument
    );

    rows = valid;
    rows[1].allocation = 0.4;
    CHECK_THROWS_AS(
        cbf2026::reconstructFullRow(rows),
        std::invalid_argument
    );

    rows = valid;
    rows[0].constant = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(
        cbf2026::reconstructFullRow(rows),
        std::invalid_argument
    );

    rows = valid;
    rows[1].coefficient[0] =
        std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(
        cbf2026::reconstructFullRow(rows),
        std::invalid_argument
    );
}

TEST_CASE("endpoint planar coefficients adapt exactly to three-input control") {
    const auto localization = cbf2026::allocatedLocalizationRows(
        localizationSnapshot(), 0.5, 0.5
    );
    const Eigen::VectorXd localizationControl =
        cbf2026::endpointRowToModelControl(localization[0], 3);
    REQUIRE(localizationControl.size() == 3);
    CHECK(localizationControl[0] == localization[0].coefficient[0]);
    CHECK(localizationControl[1] == localization[0].coefficient[1]);
    CHECK(localizationControl[2] == 0.0);

    const auto collision = cbf2026::allocatedCollisionRows(
        collisionSnapshot(), 0.5, 0.5
    );
    const Eigen::VectorXd collisionControl =
        cbf2026::endpointRowToModelControl(collision[0], 4);
    CHECK(collisionControl.head<2>() == collision[0].coefficient);
    CHECK(collisionControl.tail(2).isZero());
    CHECK_THROWS_AS(
        cbf2026::endpointRowToModelControl(localization[0], 1),
        std::invalid_argument
    );
}

TEST_CASE("generic allocated-row construction dispatches by canonical edge kind") {
    const auto localization = cbf2026::allocatedRows(
        localizationSnapshot(), 0.5, 0.5
    );
    const auto collision = cbf2026::allocatedRows(
        collisionSnapshot(), 0.5, 0.5
    );
    REQUIRE(localization.size() == 2);
    REQUIRE(collision.size() == 2);
    CHECK(localization[0].coefficient.isApprox(
        -localizationSnapshot().normal
    ));
    CHECK(collision[0].coefficient.isApprox(
        collisionSnapshot().normal
    ));
}

TEST_CASE("fixed endpoint allocation infeasibility is labeled conservatism") {
    auto snapshot = collisionSnapshot();
    snapshot.normal = Eigen::Vector2d::UnitX();
    snapshot.nuI = 25.0;
    snapshot.nuJ = 0.0;
    snapshot.alpha = -2.0;

    const auto allToJ = cbf2026::allocatedCollisionRows(
        snapshot, 0.0, 1.0
    );
    const auto half = cbf2026::allocatedCollisionRows(
        snapshot, 0.5, 0.5
    );
    const auto full = cbf2026::reconstructFullRow(half);

    const double bound = 25.0;
    const bool allToIFeasible =
        allToJ[0].constant
        + bound * allToJ[0].coefficient.lpNorm<1>() >= 0.0;
    const bool allToJFeasible =
        allToJ[1].constant
        + bound * allToJ[1].coefficient.lpNorm<1>() >= 0.0;
    const bool halfIFeasible =
        half[0].constant
        + bound * half[0].coefficient.lpNorm<1>() >= 0.0;
    const bool halfJFeasible =
        half[1].constant
        + bound * half[1].coefficient.lpNorm<1>() >= 0.0;
    const bool coupledFeasible =
        full.constant
        + bound * full.coefficientI.lpNorm<1>()
        + bound * full.coefficientJ.lpNorm<1>() >= 0.0;

    CHECK(allToIFeasible);
    CHECK(allToJFeasible);
    CHECK_FALSE(halfIFeasible);
    CHECK(halfJFeasible);
    CHECK(coupledFeasible);
    CHECK(cbf2026::allocatedFeasibilityLabel(
        coupledFeasible, {halfIFeasible, halfJFeasible}
    ) == "allocation_conservatism");
}

TEST_CASE("localization endpoint rows sum to coupled row") {
    const auto snapshot = localizationSnapshot();
    const auto rows = cbf2026::allocatedLocalizationRows(
        snapshot, 0.5, 0.5
    );
    REQUIRE(rows.size() == 2);

    const auto full = cbf2026::reconstructFullRow(rows);
    CHECK(full.coefficientI.isApprox(-snapshot.normal));
    CHECK(full.coefficientJ.isApprox(snapshot.normal));
    CHECK(full.constant == doctest::Approx(
        -snapshot.nuI - snapshot.nuJ + snapshot.alpha
    ));
}
