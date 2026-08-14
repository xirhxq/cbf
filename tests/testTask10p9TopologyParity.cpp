#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GurobiTopologySolver.hpp"
#include "grand_finale/HighsTopologySolver.hpp"
#include "grand_finale/TopologyParityDiagnostic.hpp"

namespace {
gf::TopologyRequest request4p2() {
    return {{1,2,3,4}, {10,11},
        {{10,1},{11,1},{10,2},{1,2},{3,2},
         {11,3},{1,3},{2,3},{10,4},{2,4},{3,4}},
        {{10,1},{11,1},{10,2},{1,2},{11,3},{1,3},{10,4},{2,4}},
        2, 2,
        {{"10->1",1},{"11->1",1},{"10->2",1},{"1->2",2},
         {"3->2",8},{"11->3",1},{"1->3",2},{"2->3",8},
         {"10->4",1},{"2->4",3},{"3->4",4}}, {},
        {{"10->1|11->1",2},{"10->2|1->2",2},
         {"11->3|1->3",2},{"10->4|3->4",3}}, {}};
}
}

TEST_CASE("Task 10.9 frozen request has exhaustive Gurobi HiGHS parity") {
    std::vector<gf::TopologyRequest> requests{request4p2()};
    auto rejected_optimum = request4p2();
    rejected_optimum.forbidden_topologies.push_back(
        gf::exhaustiveTopologyOracle(
            gf::TopologyModel(rejected_optimum), 1e-8).edges);
    requests.push_back(std::move(rejected_optimum));
    auto shifted_progress = request4p2();
    shifted_progress.progress_coefficients["11->3"] = 20.0;
    requests.push_back(std::move(shifted_progress));

    for (std::size_t index = 0; index < requests.size(); ++index) {
        CAPTURE(index);
        const gf::TopologyModel model(requests[index]);
        gf::GurobiTopologySolver gurobi;
        gf::HighsTopologySolver highs;
        const auto report = gf::compareTopologyParity(
            model, gurobi.solve(model), highs.solve(model), 1e-8);
        INFO(report.reason);
        INFO(report.highs_detail);
        CHECK(report.exhaustive_feasible);
        CHECK(report.gurobi_matches);
        CHECK(report.highs_matches);
        CHECK(report.selected_edge_tie_matches);
        CHECK(report.highs_detail.find("pass0") != std::string::npos);
    }
}

TEST_CASE("Task 10.9 equal objective but noncanonical selected edges fails tie parity") {
    gf::TopologyRequest request{{1}, {10,11,12},
        {{10,1},{11,1},{12,1}}, {}, 2,2,
        {},{}, {},{}};
    const gf::TopologyModel model(request);
    const auto oracle = gf::exhaustiveTopologyOracle(model, 1e-10);
    REQUIRE(oracle.feasible);
    gf::TopologySolution alternative{gf::TopologySolveStatus::Optimal,
        {{10,1},{12,1}}, {}, model.evaluate({{10,1},{12,1}}).objective,
        "synthetic-equal-objective"};
    CHECK(gf::topologyObjectiveEquivalent(
        oracle.objective, alternative.objective, 1e-10));
    CHECK_FALSE(gf::topologySelectionMatchesOracle(oracle, alternative));
}

TEST_CASE("Task 10.9 HiGHS warning remains executable but error is fatal") {
    CHECK(gf::highsStatusAllowsExecution(HighsStatus::kOk));
    CHECK(gf::highsStatusAllowsExecution(HighsStatus::kWarning));
    CHECK_FALSE(gf::highsStatusAllowsExecution(HighsStatus::kError));
}
