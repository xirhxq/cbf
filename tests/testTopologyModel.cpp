#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/TopologyModel.hpp"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

gf::TopologyRequest baseRequest() {
    return gf::TopologyRequest{
        {1, 2},
        {10, 11},
        {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1},
         gf::DirectedEdge{1, 2}, gf::DirectedEdge{10, 2},
         gf::DirectedEdge{2, 1}, gf::DirectedEdge{11, 2}},
        {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1},
         gf::DirectedEdge{1, 2}, gf::DirectedEdge{10, 2}},
        2,
        3,
        {{"10->1", 3.0}, {"11->1", 2.0}, {"1->2", 4.0},
         {"10->2", 1.0}, {"2->1", -1.0}, {"11->2", 0.5}},
        {},
        {{"10->1|11->1", 5.0}, {"1->2|10->2", 4.0}}};
}

std::vector<gf::DirectedEdge> ladder() {
    return {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1},
            gf::DirectedEdge{1, 2}, gf::DirectedEdge{10, 2}};
}

}  // namespace

TEST_CASE("Canonical variable numbering is invariant to eligible-edge order") {
    gf::TopologyRequest first_request = baseRequest();
    gf::TopologyRequest second_request = baseRequest();
    std::reverse(
        second_request.eligible_edges.begin(),
        second_request.eligible_edges.end());

    const gf::TopologyModel first(first_request);
    const gf::TopologyModel second(second_request);

    REQUIRE(first.edges().size() == second.edges().size());
    for (std::size_t index = 0; index < first.edges().size(); ++index) {
        CHECK(first.edges()[index].edge.id() == second.edges()[index].edge.id());
        CHECK(first.edges()[index].variable_index ==
              second.edges()[index].variable_index);
    }
}

TEST_CASE("A fixed-anchor ladder satisfies indegree rmax and DAG levels") {
    const gf::TopologyModel model(baseRequest());
    const gf::TopologyEvaluation result = model.evaluate(ladder());

    REQUIRE(result.valid);
    CHECK(result.levels.at(10) == 0);
    CHECK(result.levels.at(11) == 0);
    CHECK(result.levels.at(1) == 1);
    CHECK(result.levels.at(2) == 2);
    CHECK(result.objective.progress == doctest::Approx(10.0));
    CHECK(result.objective.fim_geometry == doctest::Approx(9.0));
    CHECK(result.objective.negative_switch_count == 0);
    CHECK(result.objective.negative_edge_count == -4);
}

TEST_CASE("Cycles insufficient indegree and rmax overflow are rejected") {
    const gf::TopologyModel model(baseRequest());

    const gf::TopologyEvaluation cycle = model.evaluate({
        gf::DirectedEdge{10, 1}, gf::DirectedEdge{2, 1},
        gf::DirectedEdge{11, 2}, gf::DirectedEdge{1, 2}});
    CHECK_FALSE(cycle.valid);
    CHECK(cycle.reason == "cycle");

    const gf::TopologyEvaluation insufficient = model.evaluate({
        gf::DirectedEdge{10, 1}, gf::DirectedEdge{1, 2},
        gf::DirectedEdge{10, 2}});
    CHECK_FALSE(insufficient.valid);
    CHECK(insufficient.reason == "indegree");

    gf::TopologyRequest tight = baseRequest();
    tight.max_indegree = 2;
    const gf::TopologyEvaluation overflow = gf::TopologyModel(tight).evaluate({
        gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1},
        gf::DirectedEdge{10, 2}, gf::DirectedEdge{11, 2},
        gf::DirectedEdge{1, 2}});
    CHECK_FALSE(overflow.valid);
    CHECK(overflow.reason == "indegree");
}

TEST_CASE("Invalid candidate universes fail before optimization") {
    gf::TopologyRequest invalid_owner = baseRequest();
    invalid_owner.eligible_edges.push_back(gf::DirectedEdge{1, 10});
    CHECK_THROWS_WITH_AS(
        (gf::TopologyModel{invalid_owner}),
        "eligible edge owner must be a mobile node",
        std::invalid_argument);

    gf::TopologyRequest unknown_reference = baseRequest();
    unknown_reference.eligible_edges.push_back(gf::DirectedEdge{99, 1});
    CHECK_THROWS_WITH_AS(
        (gf::TopologyModel{unknown_reference}),
        "eligible edge reference must be a known mobile or fixed node",
        std::invalid_argument);
}

TEST_CASE("Required eligible edges are a hard topology constraint") {
    gf::TopologyRequest request=baseRequest();
    request.required_edges={gf::DirectedEdge{10,1}};
    const gf::TopologyModel model(request);
    const auto missing=model.evaluate({
        gf::DirectedEdge{2,1},gf::DirectedEdge{11,1},
        gf::DirectedEdge{1,2},gf::DirectedEdge{10,2}});
    CHECK_FALSE(missing.valid);
    CHECK(missing.reason=="required_edge");
    CHECK(model.evaluate(ladder()).valid);

    request.required_edges={gf::DirectedEdge{99,1}};
    CHECK_THROWS_WITH_AS((gf::TopologyModel{request}),
        "required edge must be eligible",std::invalid_argument);
}

TEST_CASE("Binary product rows implement y equals x1 times x2 exactly") {
    const std::array<gf::BinaryProductRow, 3> rows =
        gf::binaryProductRows(2, 5, 8);
    for (int first = 0; first <= 1; ++first) {
        for (int second = 0; second <= 1; ++second) {
            for (int product = 0; product <= 1; ++product) {
                const bool rows_hold = gf::binaryProductRowsHold(
                    rows, {{2, first}, {5, second}, {8, product}});
                CHECK(rows_hold == (product == first * second));
            }
        }
    }
}
