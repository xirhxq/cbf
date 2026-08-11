#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "bridge/ExactGammaStar2D.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace {

double fourCornerGamma(
        const std::vector<BridgeGammaStarResidual2D> &residuals,
        double halfBox) {
    double best = -std::numeric_limits<double>::infinity();
    for (double ax : {-halfBox, halfBox}) {
        for (double ay : {-halfBox, halfBox}) {
            double worst = std::numeric_limits<double>::infinity();
            for (const auto &residual : residuals) {
                worst = std::min(
                        worst,
                        residual.constant - residual.ax * ax - residual.ay * ay);
            }
            best = std::max(best, worst);
        }
    }
    return best;
}

}  // namespace

TEST_CASE("Affine-margin adapter preserves the physical acceleration sign") {
    const auto residual = bridgeGammaStarResidualFromAffineMargin(1.0, -2.0, 0.5);
    const auto valueAt = [&](double ax, double ay) {
        return residual.constant - residual.ax * ax - residual.ay * ay;
    };

    CHECK(valueAt(1.0, 0.5) == doctest::Approx(0.5));
    CHECK(valueAt(-1.0, 0.5) == doctest::Approx(-1.5));
}

TEST_CASE("Exact gamma-star keeps an interior optimum that corner scoring misses") {
    const std::vector<BridgeGammaStarResidual2D> residuals = {
            {1.0, 0.0, 0.5},
            {-1.0, 0.0, 0.5},
    };

    CHECK(exactBridgeGammaStar2D(residuals, 2.0) == doctest::Approx(0.5));
    CHECK(fourCornerGamma(residuals, 2.0) == doctest::Approx(-1.5));
}

TEST_CASE("Exact gamma-star reverses the corner-only candidate ranking") {
    const std::vector<BridgeGammaStarResidual2D> interiorCandidate = {
            {1.0, 0.0, 0.5},
            {-1.0, 0.0, 0.5},
    };
    const std::vector<BridgeGammaStarResidual2D> constantCandidate = {
            {0.0, 0.0, -0.2},
    };

    CHECK(exactBridgeGammaStar2D(interiorCandidate, 2.0)
          > exactBridgeGammaStar2D(constantCandidate, 2.0));
    CHECK(fourCornerGamma(interiorCandidate, 2.0)
          < fourCornerGamma(constantCandidate, 2.0));
}
