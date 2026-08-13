#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/SolverProfile.hpp"

#include <stdexcept>

TEST_CASE("Gurobi profile expands to the approved solver tuple") {
    const nlohmann::json config = {
        {"solver", {
            {"profile", "gurobi"},
            {"topology", "gurobi_miqp"},
            {"control", "gurobi_qp"},
            {"reserve", "gurobi_lp"},
            {"seed", 2027}
        }}
    };

    const gf::SolverSelection selection = gf::loadSolverSelection(config);

    CHECK(selection.profile == gf::SolverProfile::Gurobi);
    CHECK(selection.topology == "gurobi_miqp");
    CHECK(selection.control == "gurobi_qp");
    CHECK(selection.reserve == "gurobi_lp");
    CHECK(selection.seed == 2027);
}

TEST_CASE("Open-source profile expands to the approved solver tuple") {
    const nlohmann::json config = {
        {"solver", {
            {"profile", "open_source"},
            {"topology", "highs_milp"},
            {"control", "osqp_qp"},
            {"reserve", "highs_lp"},
            {"seed", 2027}
        }}
    };

    const gf::SolverSelection selection = gf::loadSolverSelection(config);

    CHECK(selection.profile == gf::SolverProfile::OpenSource);
    CHECK(selection.topology == "highs_milp");
    CHECK(selection.control == "osqp_qp");
    CHECK(selection.reserve == "highs_lp");
    CHECK(selection.seed == 2027);
}

TEST_CASE("Solver profiles reject backend mixtures") {
    const nlohmann::json mixed = {
        {"solver", {
            {"profile", "open_source"},
            {"topology", "highs_milp"},
            {"control", "gurobi_qp"},
            {"reserve", "highs_lp"},
            {"seed", 2027}
        }}
    };

    CHECK_THROWS_WITH_AS(
        gf::loadSolverSelection(mixed),
        "open_source profile requires highs_milp/osqp_qp/highs_lp",
        std::invalid_argument);
}

TEST_CASE("Solver selection requires an approved profile and deterministic seed") {
    const nlohmann::json unknown_profile = {
        {"solver", {
            {"profile", "automatic"},
            {"topology", "highs_milp"},
            {"control", "osqp_qp"},
            {"reserve", "highs_lp"},
            {"seed", 2027}
        }}
    };
    const nlohmann::json missing_seed = {
        {"solver", {
            {"profile", "gurobi"},
            {"topology", "gurobi_miqp"},
            {"control", "gurobi_qp"},
            {"reserve", "gurobi_lp"}
        }}
    };
    const nlohmann::json negative_seed = {
        {"solver", {
            {"profile", "gurobi"},
            {"topology", "gurobi_miqp"},
            {"control", "gurobi_qp"},
            {"reserve", "gurobi_lp"},
            {"seed", -1}
        }}
    };

    CHECK_THROWS_WITH_AS(
        gf::loadSolverSelection(unknown_profile),
        "solver.profile must be gurobi or open_source",
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(
        gf::loadSolverSelection(missing_seed),
        "solver.seed is required",
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(
        gf::loadSolverSelection(negative_seed),
        "solver.seed must be a uint32 integer",
        std::invalid_argument);
}
