#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Types.hpp"

#include <stdexcept>

TEST_CASE("Undirected edges canonicalize endpoint order") {
    const gf::UndirectedEdge edge = gf::UndirectedEdge::canonical(7, 2);

    CHECK(edge.first == 2);
    CHECK(edge.second == 7);
    CHECK(edge.id() == "2--7");
}

TEST_CASE("Directed edges preserve reference-to-owner orientation") {
    const gf::DirectedEdge edge{2, 7};

    CHECK(edge.reference == 2);
    CHECK(edge.owner == 7);
    CHECK(edge.id() == "2->7");
}

TEST_CASE("Self edges are rejected") {
    CHECK_THROWS_AS((gf::DirectedEdge{3, 3}), std::invalid_argument);
    CHECK_THROWS_AS(gf::UndirectedEdge::canonical(3, 3), std::invalid_argument);
}
