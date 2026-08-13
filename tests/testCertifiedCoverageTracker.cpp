#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedCoverageTracker.hpp"

TEST_CASE("Certified coverage is tightened by error bound and cell half diagonal") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    const Point estimate(1.0, 1.0);

    tracker.observe(estimate, estimate, 0.1, 1.2);
    CHECK(tracker.truthCoveredCount() == 1);
    CHECK(tracker.certifiedCoveredCount() == 0);

    tracker.observe(estimate, estimate, 0.1, 1.6);
    CHECK(tracker.certifiedCoveredCount() == 1);
    CHECK(tracker.certifiedCoveredCount() <= tracker.truthCoveredCount());
}

TEST_CASE("A finite rectangular GridWorld reaches certified T100") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    tracker.observe(Point(1.0, 1.0), Point(1.0, 1.0), 0.1, 1.6);
    CHECK_FALSE(tracker.reachedCertifiedT100());
    tracker.observe(Point(3.0, 1.0), Point(3.0, 1.0), 0.1, 1.6);
    CHECK(tracker.reachedCertifiedT100());
    CHECK(tracker.certifiedFraction() == doctest::Approx(1.0));
}

TEST_CASE("Truth and certified maps remain distinct under estimator displacement") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    tracker.observe(Point(1.0, 1.0), Point(3.0, 1.0), 0.1, 1.6);

    CHECK(tracker.truthCovered(0, 0));
    CHECK_FALSE(tracker.certifiedCovered(0, 0));
    CHECK(tracker.certifiedCovered(1, 0));
}
