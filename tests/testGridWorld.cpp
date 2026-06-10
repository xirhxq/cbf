#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "world/world"

TEST_CASE("GridWorldCellGeometryMatchesPointDistanceAndAngle") {
    GridWorld grid({0.0, 10.0}, 10, {0.0, 10.0}, 10);
    Point center(2.0, 3.0);

    CHECK(grid.distanceToCellCenter(5, 7, center) == doctest::Approx(5.0));
    CHECK(grid.angleFromCellCenter(5, 7, center) == doctest::Approx(std::atan2(4.0, 3.0)));
}
