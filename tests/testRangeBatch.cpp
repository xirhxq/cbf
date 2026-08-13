#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/RangeBatch.hpp"

#include <limits>
#include <stdexcept>
#include <vector>

namespace {

gf::RangeMeasurement measurement(
    std::int64_t timestamp_ns,
    gf::NodeId first,
    gf::NodeId second,
    double range_m = 10.0,
    double variance_m2 = 0.25) {
    return gf::RangeMeasurement{
        timestamp_ns,
        gf::UndirectedEdge::canonical(first, second),
        range_m,
        variance_m2};
}

}  // namespace

TEST_CASE("Range batches sort by timestamp then canonical edge") {
    const std::vector<gf::RangeMeasurement> batch =
        gf::canonicalizeRangeBatch({
            measurement(20, 3, 1),
            measurement(10, 4, 2),
            measurement(10, 3, 1)});

    REQUIRE(batch.size() == 3);
    CHECK(batch[0].timestamp_ns == 10);
    CHECK(batch[0].edge.id() == "1--3");
    CHECK(batch[1].timestamp_ns == 10);
    CHECK(batch[1].edge.id() == "2--4");
    CHECK(batch[2].timestamp_ns == 20);
    CHECK(batch[2].edge.id() == "1--3");
}

TEST_CASE("A physical range sample is processed only once") {
    const std::vector<gf::RangeMeasurement> duplicate = {
        measurement(10, 1, 3),
        measurement(10, 3, 1)};

    CHECK_THROWS_WITH_AS(
        gf::canonicalizeRangeBatch(duplicate),
        "duplicate range sample 10:1--3",
        std::invalid_argument);
}

TEST_CASE("Cyclic range graphs are valid estimator batches") {
    const std::vector<gf::RangeMeasurement> ring =
        gf::canonicalizeRangeBatch({
            measurement(10, 3, 1),
            measurement(10, 2, 3),
            measurement(10, 1, 2)});

    REQUIRE(ring.size() == 3);
    CHECK(ring[0].edge.id() == "1--2");
    CHECK(ring[1].edge.id() == "1--3");
    CHECK(ring[2].edge.id() == "2--3");
}

TEST_CASE("Invalid scalar range samples fail closed") {
    CHECK_THROWS_WITH_AS(
        gf::canonicalizeRangeBatch({measurement(10, 1, 2, -1.0)}),
        "range_m must be finite and non-negative",
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(
        gf::canonicalizeRangeBatch({
            measurement(10, 1, 2, 1.0,
                        std::numeric_limits<double>::infinity())}),
        "variance_m2 must be positive and finite",
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(
        gf::canonicalizeRangeBatch({measurement(10, 1, 2, 1.0, 0.0)}),
        "variance_m2 must be positive and finite",
        std::invalid_argument);
}
