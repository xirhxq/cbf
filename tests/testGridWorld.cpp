#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "world/world"

TEST_CASE("GridWorldCellGeometryMatchesPointDistanceAndAngle") {
    GridWorld grid({0.0, 10.0}, 10, {0.0, 10.0}, 10);
    Point center(2.0, 3.0);

    CHECK(grid.distanceToCellCenter(5, 7, center)
          == doctest::Approx(std::sqrt(3.5 * 3.5 + 4.5 * 4.5)));
    CHECK(grid.angleFromCellCenter(5, 7, center)
          == doctest::Approx(std::atan2(4.5, 3.5)));
}

TEST_CASE("GridWorldSectorAngleHelpersHandleWrappedIntervals") {
    CHECK(GridWorld::normalizeAngle(-M_PI / 2.0) == doctest::Approx(3.0 * M_PI / 2.0));
    CHECK(GridWorld::normalizeAngle(5.0 * M_PI) == doctest::Approx(M_PI));

    const double start = GridWorld::normalizeAngle(11.0 * M_PI / 6.0);
    const double end = GridWorld::normalizeAngle(M_PI / 6.0);

    CHECK(GridWorld::isAngleBetweenWrapped(0.0, start, end));
    CHECK(GridWorld::isAngleBetweenWrapped(GridWorld::normalizeAngle(23.0 * M_PI / 12.0), start, end));
    CHECK(GridWorld::isAngleBetweenWrapped(M_PI / 12.0, start, end));
    CHECK_FALSE(GridWorld::isAngleBetweenWrapped(M_PI / 2.0, start, end));
}

TEST_CASE("GridWorldSearchAreaFunctionsAcceptParamsByConstReference") {
    using SearchAreaFunction = json (GridWorld::*)(Point, const json&, bool, bool);

    auto circle = static_cast<SearchAreaFunction>(&GridWorld::setValueInCircle);
    auto tiltedCone = static_cast<SearchAreaFunction>(&GridWorld::setValueInTiltedCone);
    auto sectorRing = static_cast<SearchAreaFunction>(&GridWorld::setValueInSectorRing);

    CHECK(circle != nullptr);
    CHECK(tiltedCone != nullptr);
    CHECK(sectorRing != nullptr);
}

TEST_CASE("GridWorld exposes stable centers for valid unexplored cells only") {
    GridWorld grid({0.0, 20.0}, 2, {0.0, 20.0}, 2);
    grid.valid[grid.getIndex(1, 0)] = false;
    grid.validCount = 3;
    grid.setValue(0, 1, true);

    const auto cells = grid.getUnexploredCellCenters();

    REQUIRE(cells.size() == 2);
    CHECK(cells[0].x_index == 0);
    CHECK(cells[0].y_index == 0);
    CHECK(cells[0].center.x == doctest::Approx(5.0));
    CHECK(cells[0].center.y == doctest::Approx(5.0));
    CHECK(cells[1].x_index == 1);
    CHECK(cells[1].y_index == 1);
    CHECK(cells[1].center.x == doctest::Approx(15.0));
    CHECK(cells[1].center.y == doctest::Approx(15.0));
}
