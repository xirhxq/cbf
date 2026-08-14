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
    CHECK(tracker.falseCertifiedCount() == 1);
    CHECK_FALSE(tracker.certifiedSubsetOfTruth());
}

TEST_CASE("A containing uncertainty radius is conservative and prevents false coverage") {
    gf::CertifiedCoverageTracker tracker({0.0, 4.0}, 2, {0.0, 2.0}, 1);
    tracker.observe(Point(1.0, 1.0), Point(1.6, 1.0), 0.7, 1.6);

    CHECK(tracker.truthCoveredCount() == 1);
    CHECK(tracker.certifiedCoveredCount() == 0);
    CHECK(tracker.falseCertifiedCount() == 0);
    CHECK(tracker.certifiedSubsetOfTruth());
}

TEST_CASE("Forward-sector coverage uses frozen heading and angular tightening") {
    gf::CertifiedCoverageTracker east({-4.0,4.0},4,{-1.0,1.0},1);
    east.observeSector(
        Point(0.0,0.0),Point(0.0,0.0),0.0,
        0.0,5.0,M_PI/3.0,0.0);
    CHECK_FALSE(east.truthCovered(0,0));
    CHECK_FALSE(east.truthCovered(1,0));
    CHECK(east.truthCovered(2,0));
    CHECK(east.truthCovered(3,0));
    CHECK_FALSE(east.certifiedCovered(2,0));
    CHECK(east.certifiedCovered(3,0));

    gf::CertifiedCoverageTracker west({-4.0,4.0},4,{-1.0,1.0},1);
    west.observeSector(
        Point(0.0,0.0),Point(0.0,0.0),0.0,
        0.0,5.0,M_PI/3.0,M_PI);
    CHECK(west.truthCovered(0,0));
    CHECK(west.truthCovered(1,0));
    CHECK_FALSE(west.truthCovered(2,0));
    CHECK_FALSE(west.truthCovered(3,0));
}
