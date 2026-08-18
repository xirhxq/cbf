#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/BoundaryPolicy.hpp"

namespace {

std::vector<Eigen::Vector2d> rectangle(double xmax=10.0,double ymax=20.0) {
    return {{0.0,0.0},{xmax,0.0},{xmax,ymax},{0.0,ymax}};
}

}

TEST_CASE("None boundary policy emits no position boundary facets") {
    gf::BoundaryPolicyConfig config;
    config.policy=gf::BoundaryPolicy::None;
    const auto result=gf::buildBoundaryBlueprint(config,rectangle(),{});
    CHECK(result.hard_facets.empty());
    CHECK(result.soft_facets.empty());
}

TEST_CASE("Soft search retention emits only separately typed soft facets") {
    gf::BoundaryPolicyConfig config;
    config.policy=gf::BoundaryPolicy::SoftSearchRetention;
    const auto result=gf::buildBoundaryBlueprint(config,rectangle(),{});
    CHECK(result.hard_facets.empty());
    REQUIRE(result.soft_facets.size()==4);
    CHECK(result.soft_facets[0].id=="facet:0");
    CHECK(result.soft_facets[0].outward_normal.isApprox(
        Eigen::Vector2d(0.0,-1.0)));
    CHECK(result.soft_facets[0].offset_m==doctest::Approx(0.0));
}

TEST_CASE("Hard boundary selects search or explicit polygon without target input") {
    gf::BoundaryPolicyConfig config;
    config.policy=gf::BoundaryPolicy::HardFlightBoundary;
    config.flight_polygon_source=gf::FlightPolygonSource::SearchPolygon;
    auto result=gf::buildBoundaryBlueprint(config,rectangle(),{});
    REQUIRE(result.hard_facets.size()==4);
    CHECK(result.soft_facets.empty());
    CHECK(result.hard_facets[1].outward_normal.isApprox(
        Eigen::Vector2d(1.0,0.0)));
    CHECK(result.hard_facets[1].offset_m==doctest::Approx(10.0));

    config.flight_polygon_source=gf::FlightPolygonSource::ExplicitPolygon;
    result=gf::buildBoundaryBlueprint(config,rectangle(),rectangle(30.0,40.0));
    REQUIRE(result.hard_facets.size()==4);
    CHECK(result.hard_facets[1].offset_m==doctest::Approx(30.0));
}

TEST_CASE("Explicit polygon is required only by its hard source") {
    gf::BoundaryPolicyConfig config;
    config.policy=gf::BoundaryPolicy::HardFlightBoundary;
    config.flight_polygon_source=gf::FlightPolygonSource::ExplicitPolygon;
    CHECK_THROWS_AS(gf::buildBoundaryBlueprint(config,rectangle(),{}),
                    std::invalid_argument);
    config.policy=gf::BoundaryPolicy::None;
    CHECK_NOTHROW(gf::buildBoundaryBlueprint(config,rectangle(),{}));
}

TEST_CASE("Finite-horizon excursion uses Euclidean distance outside polygon") {
    const auto polygon=rectangle();
    CHECK(gf::distanceOutsidePolygon(Eigen::Vector2d(5.0,10.0),polygon)==
          doctest::Approx(0.0));
    CHECK(gf::distanceOutsidePolygon(Eigen::Vector2d(5.0,-3.0),polygon)==
          doctest::Approx(3.0));
    CHECK(gf::distanceOutsidePolygon(Eigen::Vector2d(-3.0,-4.0),polygon)==
          doctest::Approx(5.0));
}
