#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "bridge/PostDetectionResponse.hpp"
#include "bridge/BridgeExperiment.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>

TEST_CASE("R13 frozen response distances have the preregistered static geometry") {
    const auto low = bridgeR13StaticGeometry(BRIDGE_R13_DISTANCE_LOW_M);
    const auto critical = bridgeR13StaticGeometry(BRIDGE_R13_DISTANCE_CRITICAL_M);
    const auto negative = bridgeR13StaticGeometry(BRIDGE_R13_DISTANCE_NEGATIVE_M);

    CHECK(low.audit.valid);
    const std::array<double, 8> lowEdges = {
            779.8179077130271, 351.5843929861811,
            548.4877184501877, 604.8667434160126,
            548.4877184501878, 548.4877184501878,
            548.4877184501877, 548.4877184501877};
    for (std::size_t index = 0; index < lowEdges.size(); ++index) {
        CHECK(low.exactEightEdgeCertificate.edges.at(index).length
              == doctest::Approx(lowEdges.at(index)).epsilon(1e-12));
    }
    CHECK(critical.audit.valid);
    CHECK(critical.audit.maxAssignedEdgeM == doctest::Approx(840.0).epsilon(1e-12));
    CHECK(critical.audit.maxAssignedEdgeOwnerId == 1);
    CHECK(critical.audit.maxAssignedEdgeReferenceKind == "base");
    CHECK(critical.audit.maxAssignedEdgeReferenceId == 0);
    const std::array<double, 8> criticalEdges = {
            840.0, 408.97652228396555, 609.4307982779862,
            660.6291682099297, 609.4307982779861,
            609.4307982779861, 609.4307982779862,
            609.4307982779862};
    for (std::size_t index = 0; index < criticalEdges.size(); ++index) {
        CHECK(critical.exactEightEdgeCertificate.edges.at(index).length
              == doctest::Approx(criticalEdges.at(index)).epsilon(1e-12));
    }

    CHECK_FALSE(negative.audit.valid);
    CHECK(negative.audit.worldValid);
    CHECK(negative.audit.separationValid);
    CHECK_FALSE(negative.audit.physicalCommunicationValid);
    REQUIRE(negative.audit.rejectionReasons.size() == 1);
    CHECK(negative.audit.rejectionReasons.front() == "u1-base0-over-physical-range");
    REQUIRE(negative.audit.overPhysicalRangeEdges.size() == 1);
    CHECK(negative.audit.overPhysicalRangeEdges.front().ownerId == 1);
    CHECK(negative.audit.overPhysicalRangeEdges.front().referenceKind == "base");
    CHECK(negative.audit.overPhysicalRangeEdges.front().referenceId == 0);
    CHECK(negative.audit.overPhysicalRangeEdges.front().length > 850.0);
}

TEST_CASE("R13 distance constants retain formula precision") {
    CHECK(BRIDGE_R13_DISTANCE_CRITICAL_M
          == bridgeR13DistanceForAssignedEdgeLimit(840.0));
    CHECK(BRIDGE_R13_DISTANCE_LOW_M
          == 0.9 * bridgeR13DistanceForAssignedEdgeLimit(840.0));
    CHECK(BRIDGE_R13_DISTANCE_NEGATIVE_M
          == bridgeR13DistanceForAssignedEdgeLimit(850.0) + 25.0);
}

TEST_CASE("R13 dwell is continuous and resets on any within-step exit or hard-gate failure") {
    BridgeR13DwellTracker tracker(50.0, 30.0);
    const Point target(100.0, 100.0);

    for (int step = 0; step < 59; ++step) {
        const auto state = tracker.observeZohInterval(
                Point(100.0, 100.0), Point(0.0, 0.0), Point(0.0, 0.0),
                target, 0.5, true);
        CHECK_FALSE(state.complete);
    }
    CHECK(tracker.snapshot().continuousDwellS == doctest::Approx(29.5));

    auto complete = tracker.observeZohInterval(
            Point(100.0, 100.0), Point(0.0, 0.0), Point(0.0, 0.0),
            target, 0.5, true);
    CHECK(complete.complete);
    CHECK(complete.continuousDwellS == doctest::Approx(30.0));

    tracker.reset();
    auto exited = tracker.observeZohInterval(
            Point(149.0, 100.0), Point(8.0, 0.0), Point(-16.0, 0.0),
            target, 1.0, true);
    CHECK_FALSE(exited.intervalValid);
    CHECK(exited.maxHorizontalDistanceM > 50.0);
    CHECK(exited.continuousDwellS == 0.0);

    auto invalidGate = tracker.observeZohInterval(
            target, Point(0.0, 0.0), Point(0.0, 0.0), target, 1.0, false);
    CHECK_FALSE(invalidGate.intervalValid);
    CHECK(invalidGate.continuousDwellS == 0.0);

    auto nonfinite = tracker.observeZohInterval(
            Point(std::numeric_limits<double>::quiet_NaN(), 100.0),
            Point(0.0, 0.0), Point(0.0, 0.0), target, 1.0, true);
    CHECK_FALSE(nonfinite.intervalValid);
    CHECK(nonfinite.continuousDwellS == 0.0);
}

TEST_CASE("R13 candidate mechanism labels do not confuse fallback with least intervention") {
    CHECK(bridgeR13CandidateMechanism(10.112488, 14.0, true)
          == "maximum-reserve-fallback");
    CHECK(bridgeR13CandidateMechanism(14.0, 14.0, true)
          == "reserve-admissible-least-intervention");
    CHECK(bridgeR13CandidateMechanism(20.0, 14.0, false)
          == "feedback-disabled");
    CHECK(bridgeR13CandidateMechanism(
                  20.0, 14.0, true, "maximum-reserve")
          == "maximum-reserve-comparator");
}

TEST_CASE("R13 response config publishes one static tuple and forbids search state") {
    std::ifstream stream(std::filesystem::path(PROJECT_ROOT)
                         / "config" / "single_ladder_campaign_template_r10.json");
    REQUIRE(stream.good());
    json config;
    stream >> config;
    config["bridge"]["row"] = "R13";
    config["bridge"].erase("search-policy");
    config["bridge"]["task-mode"] = "post-detection-response";
    config["bridge"].erase("search");
    config["bridge"]["response"] = {
            {"mode", "known-static-target-reach-and-dwell"},
            {"distance-m", BRIDGE_R13_DISTANCE_CRITICAL_M},
            {"leader-id", 4}, {"target-radius-m", 50.0},
            {"dwell-time-s", 30.0},
            {"terminal-communication-margin-m", 10.0}};
    config["bridge"]["target"] = {
            {"x", BRIDGE_R13_BASE_X_M + BRIDGE_R13_DISTANCE_CRITICAL_M},
            {"y", 1255.0}, {"radius", 50.0}};
    config["bridge"]["nominal"]["gamma-star-feedback"]["predictive-gate"] = 14.0;

    const auto bridge = loadBridgeExperimentConfig(config);
    CHECK(bridge.postDetectionResponseEnabled);
    CHECK(bridge.jointSingleLadderGoalsEnabled);
    CHECK(bridge.responseDistanceM == BRIDGE_R13_DISTANCE_CRITICAL_M);
    CHECK(bridge.responseDwellTimeS == 30.0);

    config["bridge"]["search"] = {{"map-update-radius-m", 10.0}};
    CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);
    config["bridge"].erase("search");
    config["bridge"]["response"]["waypoint-switching"] = false;
    CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);

    config["bridge"]["response"].erase("waypoint-switching");
    config["bases"] = {{251.0, 1000.0}, {250.0, 1510.0}};
    CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);
    config["bases"] = {{250.0, 1000.0}, {250.0, 1510.0}};
    config["execute"]["time-step"] = 1.0;
    CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);
}
