#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "cbf/HardInteriorSelection.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace {

cbf2026::HardConstraintRow row(
    std::initializer_list<double> coefficients,
    double constant
) {
    cbf2026::HardConstraintRow hardRow;
    hardRow.coefficients = Eigen::VectorXd(
        static_cast<Eigen::Index>(coefficients.size())
    );
    Eigen::Index index = 0;
    for (const double coefficient : coefficients) {
        hardRow.coefficients[index++] = coefficient;
    }
    hardRow.constant = constant;
    return hardRow;
}

cbf2026::HardConstraintProblem boxWithRows(
    std::initializer_list<cbf2026::HardConstraintRow> rows,
    double planarComponentMax = 25.0,
    double yawRateMax = 0.35
) {
    cbf2026::HardConstraintProblem problem;
    problem.controlSize = 3;
    problem.planarComponentMax = planarComponentMax;
    problem.yawRateMax = yawRateMax;
    problem.rows.assign(rows.begin(), rows.end());
    problem.bounds = cbf2026::theoremInputBounds(
        planarComponentMax, yawRateMax
    );
    return problem;
}

} // namespace

TEST_CASE("planar Chebyshev radius uses local hard rows and component box") {
    const auto problem = boxWithRows({
        row({1.0, 0.0, 0.0}, 2.0),
        row({-1.0, 0.0, 0.0}, 2.0),
        row({0.0, 1.0, 0.0}, 3.0),
        row({0.0, -1.0, 0.0}, 3.0),
    });

    const auto result = cbf2026::solvePlanarHardRowChebyshev(problem);

    CHECK(result.radius == doctest::Approx(2.0));
    CHECK(result.witness.isApprox(Eigen::Vector2d(0.0, -1.0), 1e-12));
    CHECK(result.tightHardRows == std::vector<std::size_t>({0, 1, 2}));
}

TEST_CASE("yaw never enters planar Chebyshev radius") {
    const auto problem = boxWithRows(
        {row({1.0, 0.0, 0.0}, 1.0)}, 25.0, 1e-12
    );

    CHECK(cbf2026::solvePlanarHardRowChebyshev(problem).radius
          == doctest::Approx(26.0));
}

TEST_CASE("frozen floor is fractional capped and tolerance aware") {
    CHECK(cbf2026::frozenInteriorFloor(0.8) == doctest::Approx(0.08));
    CHECK(cbf2026::frozenInteriorFloor(2.0) == doctest::Approx(0.10));
    CHECK(cbf2026::frozenInteriorFloor(5e-10) == doctest::Approx(0.0));
    CHECK(cbf2026::frozenInteriorFloor(0.8, 0.10, 0.10, 0.20)
          == doctest::Approx(0.06));
}

TEST_CASE("Chebyshev primitive rejects missing planar component bounds") {
    auto problem = boxWithRows({row({1.0, 0.0, 0.0}, 1.0)});
    problem.bounds.erase(problem.bounds.begin());

    CHECK_THROWS_AS(
        cbf2026::solvePlanarHardRowChebyshev(problem), std::invalid_argument
    );
}

TEST_CASE("Chebyshev primitive rejects duplicate component bounds") {
    auto problem = boxWithRows({row({1.0, 0.0, 0.0}, 1.0)});
    problem.bounds[1] = problem.bounds[0];

    CHECK_THROWS_AS(
        cbf2026::solvePlanarHardRowChebyshev(problem), std::invalid_argument
    );
}

TEST_CASE("Chebyshev primitive rejects hard rows with yaw coefficients") {
    const auto problem = boxWithRows({row({1.0, 0.0, 1e-12}, 1.0)});

    CHECK_THROWS_AS(
        cbf2026::solvePlanarHardRowChebyshev(problem), std::invalid_argument
    );
}

TEST_CASE("Chebyshev primitive rejects malformed and nonfinite hard data") {
    const auto malformed = boxWithRows({row({1.0, 0.0}, 1.0)});
    CHECK_THROWS_AS(
        cbf2026::solvePlanarHardRowChebyshev(malformed), std::invalid_argument
    );

    const auto nonfinite = boxWithRows({row({
        std::numeric_limits<double>::infinity(), 0.0, 0.0
    }, 1.0)});
    CHECK_THROWS_AS(
        cbf2026::solvePlanarHardRowChebyshev(nonfinite), std::invalid_argument
    );
}

TEST_CASE("Chebyshev primitive rejects an unbounded radius with no finite vertex") {
    const auto problem = boxWithRows({});

    CHECK_THROWS_AS(
        cbf2026::solvePlanarHardRowChebyshev(problem), std::runtime_error
    );
}

TEST_CASE("Chebyshev primitive selects lexicographically first tied witness") {
    const auto problem = boxWithRows({row({1.0, 0.0, 0.0}, 1.0)});

    const auto result = cbf2026::solvePlanarHardRowChebyshev(problem);

    CHECK(result.radius == doctest::Approx(26.0));
    CHECK(result.witness.isApprox(Eigen::Vector2d(25.0, -25.0), 1e-12));
    CHECK(result.tightHardRows == std::vector<std::size_t>({0}));
}
