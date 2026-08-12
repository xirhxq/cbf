#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "bridge/BridgeExperiment.hpp"
#include "bridge/BridgeSearchTracker.hpp"
#include "bridge/SingleLadderGoalSelector.hpp"
#include "bridge/BridgeTopology.hpp"
#include "bridge/HocbfFeasibilityGuard.hpp"

namespace {

json fixedSingleLadderTopologyConfig() {
    return {
        {"bridge", {
            {"topology", {
                {"max-range", 850.0},
                {"uncertainty-multiplier", 0.0},
                {"certified-margin", 0.0},
                {"certified-only", true},
                {"fail-safe-hold", true},
                {"fixed-references", {
                    {"1", {{"anchor-ids", json::array()}, {"base-ids", {0, 1}}}},
                    {"2", {{"anchor-ids", {1}}, {"base-ids", {0}}}},
                    {"3", {{"anchor-ids", {2, 1}}, {"base-ids", json::array()}}},
                    {"4", {{"anchor-ids", {3, 2}}, {"base-ids", json::array()}}}
                }}
            }}
        }}
    };
}

json validFullRowFeedbackConfig() {
    json config = fixedSingleLadderTopologyConfig();
    config["model"] = "DoubleIntegrate2D";
    config["optimiser"] = "Gurobi";
    config["num"] = 4;
    config["bases"] = {{0.0, 0.0}, {0.0, 100.0}};
    config["position_covariance"] = {{"enable", false}};
    config["bridge"]["enabled"] = true;
    config["bridge"]["topology-policy"] = "fixed";
    config["bridge"]["safety-filter"] = "second-order-hocbf";
    config["bridge"]["nominal"] = {
        {"guard", {
            {"enabled", true},
            {"mode", "hocbf-feasible-projection"},
            {"tolerance", 1.0e-9}
        }},
        {"gamma-star-feedback", {
            {"enabled", true},
            {"mode", "reserve-task-homotopy"},
            {"homotopy-intervals", 8},
            {"lookahead-steps", 1},
            {"predictive-gate", 0.0}
        }}
    };
    config["cbfs"] = {
        {"high-order", {
            {"enabled", true},
            {"acceleration-bound", 2.0},
            {"feasibility-slack", {{"enabled", false}}}
        }},
        {"with-slack", {
            {"cvt", {{"on", false}}},
            {"cvt-yaw", {{"on", false}}},
            {"target-yaw", {{"on", false}}}
        }},
        {"without-slack", {
            {"safety", {
                {"on", true},
                {"safe-distance", 10.0},
                {"safe-distance-tightening-margin", 0.0},
                {"consider-uncertainty", false}
            }},
            {"comm-fixed", {
                {"on", true},
                {"max-range", 850.0},
                {"range-tightening-margin", 0.0},
                {"consider-uncertainty", false}
            }},
            {"comm-auto", {{"on", false}}}
        }}
    };
    return config;
}

}  // namespace

TEST_CASE("Fixed bridge references preserve the single triangular ladder") {
    const auto topology = loadBridgeTopologyConfig(fixedSingleLadderTopologyConfig());
    CHECK_NOTHROW(validateBridgeFixedReferences(topology.fixedReferences, 4, 2));
    CHECK_NOTHROW(validateBridgeSingleTriangularLadder(topology.fixedReferences));

    const auto first = chooseBridgeTopology(
        {{1, Point(0.0, 0.0)}, {2, Point(1.0, 0.0)},
         {3, Point(2.0, 0.0)}, {4, Point(3.0, 0.0)}},
        topology,
        "fixed",
        {{0, Point(-1.0, 0.0)}, {1, Point(-1.0, 1.0)}});
    const auto moved = chooseBridgeTopology(
        {{1, Point(9.0, 4.0)}, {2, Point(-2.0, 7.0)},
         {3, Point(8.0, -3.0)}, {4, Point(0.0, 6.0)}},
        topology,
        "fixed",
        {{0, Point(0.0, 0.0)}, {1, Point(1.0, 1.0)}});

    CHECK(first.references == moved.references);
    CHECK(first.references.at(1).anchorIds.empty());
    CHECK(first.references.at(1).baseIds == std::vector<int>({0, 1}));
    CHECK(first.references.at(2).anchorIds == std::vector<int>({1}));
    CHECK(first.references.at(2).baseIds == std::vector<int>({0}));
    CHECK(first.references.at(3).anchorIds == std::vector<int>({2, 1}));
    CHECK(first.references.at(3).baseIds.empty());
    CHECK(first.references.at(4).anchorIds == std::vector<int>({3, 2}));
    CHECK(first.references.at(4).baseIds.empty());
}

TEST_CASE("Fixed ladder certification evaluates every mobile and base edge") {
    auto config = loadBridgeTopologyConfig(fixedSingleLadderTopologyConfig());
    config.maxRange = 850.0;
    config.uncertaintyMultiplier = 0.0;
    config.sigma0 = 0.0;
    config.referenceVariance = 0.0;
    config.certifiedMargin = 0.0;
    config.certifiedOnly = true;
    config.failSafeHold = true;

    const std::map<int, Point> positions = {
            {1, Point(10.0, 5.0)},
            {2, Point(20.0, 0.0)},
            {3, Point(30.0, 0.0)},
            {4, Point(40.0, 0.0)},
    };
    const std::map<int, Point> bases = {
            {0, Point(0.0, 0.0)},
            {1, Point(0.0, 10.0)},
    };

    const auto certified = chooseBridgeTopology(
            positions, config, "fixed", bases);
    REQUIRE(certified.accepted);
    CHECK(certified.certified);
    CHECK_FALSE(certified.failSafe);
    CHECK(certified.log.at("geometry").size() == 8);
    CHECK(certified.log.at("min_robust_margin").get<double>()
          == doctest::Approx(certified.minRobustMargin));
    CHECK(certified.log.at("fail_safe_hold") == true);

    auto outOfRange = positions;
    outOfRange.at(1) = Point(900.0, 0.0);
    const auto uncertified = chooseBridgeTopology(
            outOfRange, config, "fixed", bases);
    REQUIRE(uncertified.accepted);
    CHECK_FALSE(uncertified.certified);
    CHECK(uncertified.failSafe);
    CHECK(uncertified.minRobustMargin < 0.0);
    CHECK(uncertified.log.at("certified") == false);
    CHECK(uncertified.log.at("fail_safe") == true);

    const auto missingBase = chooseBridgeTopology(
            positions, config, "fixed", {{0, Point(0.0, 0.0)}});
    CHECK_FALSE(missingBase.accepted);
    CHECK_FALSE(missingBase.certified);
    CHECK(missingBase.failSafe);
}

TEST_CASE("Fixed bridge references reject malformed ladders") {
    auto requireInvalid = [](BridgeFixedReferenceMap references) {
        CHECK_THROWS_AS(validateBridgeFixedReferences(references, 4, 2), std::invalid_argument);
    };

    const auto valid = loadBridgeTopologyConfig(fixedSingleLadderTopologyConfig()).fixedReferences;

    SUBCASE("missing robot") {
        auto references = valid;
        references.erase(4);
        requireInvalid(references);
    }
    SUBCASE("self reference") {
        auto references = valid;
        references.at(3).anchorIds = {3, 1};
        requireInvalid(references);
    }
    SUBCASE("duplicate mobile reference") {
        auto references = valid;
        references.at(3).anchorIds = {2, 2};
        requireInvalid(references);
    }
    SUBCASE("forward mobile reference") {
        auto references = valid;
        references.at(3).anchorIds = {4, 1};
        requireInvalid(references);
    }
    SUBCASE("invalid base reference") {
        auto references = valid;
        references.at(1).baseIds = {0, 2};
        requireInvalid(references);
    }
    SUBCASE("wrong reference count") {
        auto references = valid;
        references.at(4).anchorIds = {3};
        requireInvalid(references);
    }
}

TEST_CASE("Structurally valid predecessor graphs cannot replace the declared ladder") {
    auto references =
            loadBridgeTopologyConfig(fixedSingleLadderTopologyConfig()).fixedReferences;
    references.at(4).anchorIds = {3, 1};

    CHECK_NOTHROW(validateBridgeFixedReferences(references, 4, 2));
    CHECK_THROWS_AS(
            validateBridgeSingleTriangularLadder(references),
            std::invalid_argument);
}

TEST_CASE("BridgeExperimentLoadsDisabledDefault") {
    json config = json::object();
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);

    CHECK_FALSE(bridge.enabled);
    CHECK(bridge.row == "debug");
    CHECK(bridge.reportCadence == doctest::Approx(1.0));
}

TEST_CASE("BridgeExperimentLoadsNominalGuardDefaultDisabled") {
    json config = {
        {"bridge", {
            {"enabled", true},
            {"nominal", json::object()}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);

    CHECK_FALSE(bridge.nominalGuardEnabled);
    CHECK(bridge.nominalGuardMode == "hocbf-feasible-projection");
    CHECK(bridge.nominalGuardTolerance == doctest::Approx(1.0e-9));
}

TEST_CASE("BridgeExperimentLoadsNominalGuardConfig") {
    json config = {
        {"bridge", {
            {"enabled", true},
            {"nominal", {
                {"guard", {
                    {"enabled", true},
                    {"mode", "hocbf-feasible-projection"},
                    {"tolerance", 1.0e-8}
                }}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);

    CHECK(bridge.nominalGuardEnabled);
    CHECK(bridge.nominalGuardMode == "hocbf-feasible-projection");
    CHECK(bridge.nominalGuardTolerance == doctest::Approx(1.0e-8));
}

TEST_CASE("Full-row feedback accepts only the declared homotopy configuration") {
    const auto bridge = loadBridgeExperimentConfig(validFullRowFeedbackConfig());

    CHECK(bridge.gammaStarFeedbackEnabled);
    CHECK(bridge.gammaStarFeedbackMode == "reserve-task-homotopy");
    CHECK(bridge.gammaStarFeedbackAnalysisRole == "main");
    CHECK(bridge.gammaStarHomotopyIntervals == 8);
    CHECK(bridge.gammaStarLookaheadSteps == 1);
    CHECK(bridge.gammaStarPredictiveGate == doctest::Approx(0.0));

    auto buffered = validFullRowFeedbackConfig();
    buffered["bridge"]["nominal"]["gamma-star-feedback"]["predictive-gate"] =
            20.0;
    const auto bufferedBridge = loadBridgeExperimentConfig(buffered);
    CHECK(bufferedBridge.gammaStarPredictiveGate == doctest::Approx(20.0));
}

TEST_CASE("Bridge search exposes explicit task-completion termination") {
    auto config = validFullRowFeedbackConfig();
    config["bridge"]["search"]["stop-on-detection"] = true;

    const auto bridge = loadBridgeExperimentConfig(config);

    CHECK(bridge.stopOnDetection);
}

TEST_CASE("Full-row feedback accepts fixed physical-row tightening") {
    auto config = validFullRowFeedbackConfig();
    config["cbfs"]["without-slack"]["comm-fixed"]
            ["range-tightening-margin"] = 1.0;
    config["cbfs"]["without-slack"]["safety"]
            ["safe-distance-tightening-margin"] = 0.5;

    CHECK_NOTHROW(loadBridgeExperimentConfig(config));
}

TEST_CASE("Full-row feedback rejects hidden method and boundary changes") {
    auto requireInvalid = [](json config) {
        CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);
    };

    SUBCASE("independent acceleration box") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["accel-half-box"] = 2.0;
        requireInvalid(config);
    }
    SUBCASE("direction lattice") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["direction-count"] = 8;
        requireInvalid(config);
    }
    SUBCASE("magnitude lattice") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["magnitude-count"] = 3;
        requireInvalid(config);
    }
    SUBCASE("zero homotopy intervals") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["homotopy-intervals"] = 0;
        requireInvalid(config);
    }
    SUBCASE("undeclared homotopy intervals") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["homotopy-intervals"] = 3;
        requireInvalid(config);
    }
    SUBCASE("main role cannot use a sensitivity library") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["homotopy-intervals"] = 4;
        requireInvalid(config);
    }
    SUBCASE("zero lookahead") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["lookahead-steps"] = 0;
        requireInvalid(config);
    }
    SUBCASE("non-main lookahead") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["lookahead-steps"] = 5;
        requireInvalid(config);
    }
    SUBCASE("nonfinite predictive gate") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["predictive-gate"] =
                std::numeric_limits<double>::infinity();
        requireInvalid(config);
    }
    SUBCASE("negative predictive gate") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]["predictive-gate"] = -1.0e-12;
        requireInvalid(config);
    }
    SUBCASE("wrong mobile count") {
        auto config = validFullRowFeedbackConfig();
        config["num"] = 3;
        requireInvalid(config);
    }
    SUBCASE("unaudited QP solver") {
        auto config = validFullRowFeedbackConfig();
        config["optimiser"] = "OSQP";
        requireInvalid(config);
    }
    SUBCASE("wrong base count") {
        auto config = validFullRowFeedbackConfig();
        config["bases"].erase(config["bases"].begin());
        requireInvalid(config);
    }
    SUBCASE("unfrozen topology range") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["topology"]["max-range"] = 900.0;
        requireInvalid(config);
    }
    SUBCASE("topology certificate without fail-safe hold") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["topology"]["fail-safe-hold"] = false;
        requireInvalid(config);
    }
    SUBCASE("adaptive topology") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["topology-policy"] = "adaptive-relay";
        requireInvalid(config);
    }
    SUBCASE("automatic communication graph") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["without-slack"]["comm-auto"]["on"] = true;
        requireInvalid(config);
    }
    SUBCASE("communication tightening consumes the entire range") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["without-slack"]["comm-fixed"]
                ["range-tightening-margin"] = 850.0;
        requireInvalid(config);
    }
    SUBCASE("negative safety tightening") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["without-slack"]["safety"]
                ["safe-distance-tightening-margin"] = -1.0;
        requireInvalid(config);
    }
    SUBCASE("position covariance") {
        auto config = validFullRowFeedbackConfig();
        config["position_covariance"]["enable"] = true;
        requireInvalid(config);
    }
    SUBCASE("communication uncertainty") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["without-slack"]["comm-fixed"]["consider-uncertainty"] = true;
        requireInvalid(config);
    }
    SUBCASE("safety uncertainty") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["without-slack"]["safety"]["consider-uncertainty"] = true;
        requireInvalid(config);
    }
    SUBCASE("raw communication range") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 849.0;
        requireInvalid(config);
    }
    SUBCASE("raw safety distance") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["without-slack"]["safety"]["safe-distance"] = 11.0;
        requireInvalid(config);
    }
    SUBCASE("unrelated soft task CBF") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["with-slack"]["cvt"]["on"] = true;
        requireInvalid(config);
    }
    SUBCASE("unfrozen feasibility-guard tolerance") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["guard"]["tolerance"] = 1.0e-8;
        requireInvalid(config);
    }
    SUBCASE("hard-row feasibility slack") {
        auto config = validFullRowFeedbackConfig();
        config["cbfs"]["high-order"]["feasibility-slack"]["enabled"] = true;
        requireInvalid(config);
    }
}

TEST_CASE("Full-row feedback admits only predeclared sensitivity libraries") {
    for (int intervals : {4, 16}) {
        auto config = validFullRowFeedbackConfig();
        auto &feedback = config["bridge"]["nominal"]["gamma-star-feedback"];
        feedback["analysis-role"] = "sensitivity";
        feedback["homotopy-intervals"] = intervals;

        const auto bridge = loadBridgeExperimentConfig(config);
        CHECK(bridge.gammaStarFeedbackAnalysisRole == "sensitivity");
        CHECK(bridge.gammaStarHomotopyIntervals == intervals);
    }

    auto invalid = validFullRowFeedbackConfig();
    auto &feedback = invalid["bridge"]["nominal"]["gamma-star-feedback"];
    feedback["analysis-role"] = "sensitivity";
    feedback["homotopy-intervals"] = 8;
    CHECK_THROWS_AS(loadBridgeExperimentConfig(invalid), std::invalid_argument);
}

TEST_CASE("Full-row feedback accepts only the frozen comparator and ablation tuples") {
    SUBCASE("predictive-soft comparator") {
        auto config = validFullRowFeedbackConfig();
        auto &feedback = config["bridge"]["nominal"]["gamma-star-feedback"];
        feedback["analysis-role"] = "comparator";
        feedback["constraint-execution"] = "soft";
        config["cbfs"]["high-order"]["feasibility-slack"]["enabled"] = true;
        config["cbfs"]["objective-function"]["k_delta"] = 1000.0;

        const auto bridge = loadBridgeExperimentConfig(config);
        CHECK(bridge.gammaStarFeedbackAnalysisRole == "comparator");
        CHECK(bridge.gammaStarFeedbackConstraintExecution == "soft");
        CHECK(bridge.gammaStarFeedbackSelectionRule == "least-intervention");
    }

    SUBCASE("maximum-reserve hard ablation") {
        auto config = validFullRowFeedbackConfig();
        auto &feedback = config["bridge"]["nominal"]["gamma-star-feedback"];
        feedback["analysis-role"] = "ablation";
        feedback["selection-rule"] = "maximum-reserve";

        const auto bridge = loadBridgeExperimentConfig(config);
        CHECK(bridge.gammaStarFeedbackAnalysisRole == "ablation");
        CHECK(bridge.gammaStarFeedbackConstraintExecution == "hard");
        CHECK(bridge.gammaStarFeedbackSelectionRule == "maximum-reserve");
    }

    auto requireInvalid = [](json config) {
        CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);
    };
    SUBCASE("main cannot execute soft rows") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]
                ["constraint-execution"] = "soft";
        config["cbfs"]["high-order"]["feasibility-slack"]["enabled"] = true;
        config["cbfs"]["objective-function"]["k_delta"] = 1000.0;
        requireInvalid(config);
    }
    SUBCASE("comparator requires soft rows") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]
                ["analysis-role"] = "comparator";
        requireInvalid(config);
    }
    SUBCASE("soft comparator requires frozen penalty") {
        auto config = validFullRowFeedbackConfig();
        auto &feedback = config["bridge"]["nominal"]["gamma-star-feedback"];
        feedback["analysis-role"] = "comparator";
        feedback["constraint-execution"] = "soft";
        config["cbfs"]["high-order"]["feasibility-slack"]["enabled"] = true;
        config["cbfs"]["objective-function"]["k_delta"] = 999.0;
        requireInvalid(config);
    }
    SUBCASE("ablation requires maximum reserve") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]
                ["analysis-role"] = "ablation";
        requireInvalid(config);
    }
    SUBCASE("maximum reserve is not a main result") {
        auto config = validFullRowFeedbackConfig();
        config["bridge"]["nominal"]["gamma-star-feedback"]
                ["selection-rule"] = "maximum-reserve";
        requireInvalid(config);
    }
}

TEST_CASE("BridgeExperimentLoadsRelaySupportGuardConfig") {
    json config = {
        {"bridge", {
            {"enabled", true},
            {"nominal", {
                {"relay-support-guard", {
                    {"enabled", true},
                    {"robust-margin", 35.0}
                }}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);

    CHECK(bridge.relaySupportGuardEnabled);
    CHECK(bridge.relaySupportRobustMargin == doctest::Approx(35.0));
}

TEST_CASE("BridgeExperimentLoadsSupportChainGuardConfig") {
    json config = {
        {"bridge", {
            {"enabled", true},
            {"nominal", {
                {"support-chain-guard", {
                    {"enabled", true},
                    {"robust-margin", 45.0}
                }}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);

    CHECK(bridge.supportChainGuardEnabled);
    CHECK(bridge.supportChainRobustMargin == doctest::Approx(45.0));
}

TEST_CASE("BridgeExperimentLoadsAllActiveSupportChainGuardScope") {
    json config = {
        {"bridge", {
            {"enabled", true},
            {"nominal", {
                {"support-chain-guard", {
                    {"enabled", true},
                    {"robust-margin", 45.0},
                    {"scope", "all-active-edges"}
                }}
            }}
        }}
    };

    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);

    CHECK(bridge.supportChainGuardEnabled);
    CHECK(bridge.supportChainRobustMargin == doctest::Approx(45.0));
    CHECK(bridge.supportChainGuardScope == "all-active-edges");
}

TEST_CASE("HocbfGuardClipsAccelerationBoxByHalfspace") {
    std::vector<BridgeHocbfHalfspace2D> constraints = {
        {1.0, 0.0, 0.0}
    };

    BridgeHocbfProjectionResult result = projectBridgeHocbfNominalAcceleration(
        -1.0,
        0.5,
        2.0,
        constraints,
        1.0e-9);

    REQUIRE(result.feasible);
    CHECK(result.active);
    CHECK(result.projected_ax == doctest::Approx(0.0));
    CHECK(result.projected_ay == doctest::Approx(0.5));
    CHECK(result.margin_before == doctest::Approx(-1.0));
    CHECK(result.margin_after == doctest::Approx(0.0));
    CHECK(result.vertex_count >= 4);
}

TEST_CASE("HocbfGuardReportsEmptyPolygonForConflictingHalfspaces") {
    std::vector<BridgeHocbfHalfspace2D> constraints = {
        {1.0, 0.0, 1.0},
        {-1.0, 0.0, 1.0}
    };

    BridgeHocbfProjectionResult result = projectBridgeHocbfNominalAcceleration(
        0.0,
        0.0,
        2.0,
        constraints,
        1.0e-9);

    CHECK_FALSE(result.feasible);
    CHECK_FALSE(result.active);
    CHECK(result.vertex_count == 0);
}

TEST_CASE("HocbfGuardLeavesInteriorNominalAccelerationUnchanged") {
    std::vector<BridgeHocbfHalfspace2D> constraints = {
        {1.0, 0.0, -1.0},
        {0.0, 1.0, -1.0}
    };

    BridgeHocbfProjectionResult result = projectBridgeHocbfNominalAcceleration(
        0.2,
        0.3,
        2.0,
        constraints,
        1.0e-9);

    REQUIRE(result.feasible);
    CHECK_FALSE(result.active);
    CHECK(result.projected_ax == doctest::Approx(0.2));
    CHECK(result.projected_ay == doctest::Approx(0.3));
}

TEST_CASE("BridgeExperimentMetadataRecordsPaperScaleAndSampleTimes") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {3000.0, 0.0}, {3000.0, 3000.0}, {0.0, 3000.0}}},
            {"spacing", 100.0}
        }},
        {"execute", {
            {"time-total", 400.0},
            {"time-step", 0.5}
        }},
        {"bridge", {
            {"enabled", true},
            {"row", "R4"},
            {"search-policy", "active"},
            {"topology-policy", "adaptive-relay"},
            {"safety-filter", "second-order-hocbf"},
            {"report-cadence", 1.0},
            {"target", {{"x", 2400.0}, {"y", 2100.0}, {"radius", 160.0}}}
        }}
    };

    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    json metadata = makeBridgeMetadata(config, bridge);

    CHECK(bridge.enabled);
    CHECK(bridge.row == "R4");
    CHECK(bridge.searchPolicy == "active");
    CHECK(bridge.topologyPolicy == "adaptive-relay");
    CHECK(bridge.safetyFilter == "second-order-hocbf");
    CHECK(bridge.target.x == doctest::Approx(2400.0));
    CHECK(bridge.target.y == doctest::Approx(2100.0));
    CHECK(metadata.at("area_width_m").get<double>() == doctest::Approx(3000.0));
    CHECK(metadata.at("area_height_m").get<double>() == doctest::Approx(3000.0));
    CHECK(metadata.at("horizon_s").get<double>() == doctest::Approx(400.0));
    CHECK(metadata.at("control_sample_time_s").get<double>() == doctest::Approx(0.5));
    CHECK(metadata.at("report_cadence_s").get<double>() == doctest::Approx(1.0));
}

TEST_CASE("nearest-feasible-single-ladder policy requires the frozen task contract") {
    json config = validFullRowFeedbackConfig();
    config["world"] = {
        {"boundary", {{0.0, 0.0}, {1800.0, 0.0},
                      {1800.0, 2000.0}, {0.0, 2000.0}}},
        {"spacing", 100.0}
    };
    config["bases"] = {{250.0, 1000.0}, {250.0, 1510.0}};
    config["bridge"]["search-policy"] =
            "nearest-feasible-single-ladder";
    config["bridge"]["search"] = {
        {"rotation-deg", 60.0},
        {"leader-id", 4},
        {"goal-max-range-m", 849.0},
        {"goal-min-separation-m", 10.0},
        {"initial-leader-grid", {4, 11}},
    };
    config["cbfs"]["without-slack"]["comm-fixed"]
          ["range-tightening-margin"] = 1.0;

    const auto bridge = loadBridgeExperimentConfig(config);
    CHECK(bridge.jointSingleLadderGoalsEnabled);

    config["bases"] = {{260.0, 1010.0}, {260.0, 1520.0}};
    CHECK(loadBridgeExperimentConfig(config).jointSingleLadderGoalsEnabled);
    config["bases"] = {{250.0, 1000.0}, {250.0, 1510.0}};

    config["bridge"]["search"]["goal-max-range-m"] = 849.1;
    CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);

    config["bridge"]["search"]["goal-max-range-m"] = 849.0;
    config["bridge"]["nominal"]["goal-diversion"] = {
        {"enabled", true},
        {"distance-threshold", 120.0},
        {"radial-threshold", 4.0},
        {"separation-scale", 1.0},
        {"max-offset", 200.0},
        {"pair-scope", "all"},
    };
    CHECK_THROWS_AS(loadBridgeExperimentConfig(config), std::invalid_argument);
}

TEST_CASE("canonical r10 template satisfies the C++ joint goal contract") {
    std::ifstream stream(
            std::filesystem::path(PROJECT_ROOT)
            / "config" / "single_ladder_campaign_template_r10.json");
    REQUIRE(stream.good());
    json config;
    stream >> config;

    const auto bridge = loadBridgeExperimentConfig(config);
    CHECK(bridge.jointSingleLadderGoalsEnabled);
    CHECK(bridge.jointSingleLadderLeaderId == 4);
    CHECK(bridge.jointSingleLadderRotationDeg == 60.0);
    CHECK(bridge.jointSingleLadderGoalMaxRange == 849.0);
    CHECK(bridge.jointSingleLadderGoalMinSeparation == 10.0);
    CHECK(bridge.jointSingleLadderInitialRow == 4);
    CHECK(bridge.jointSingleLadderInitialColumn == 11);
}

TEST_CASE("BridgeSearchTrackerUpdatesCoverageBeliefEntropyAndDetection") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {300.0, 0.0}, {300.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 150.0}, {"y", 150.0}, {"radius", 30.0}}},
            {"search", {
                {"negative-detection-factor", 0.70},
                {"positive-detection-factor", 3.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);

    CHECK(tracker.coverageRatio() == doctest::Approx(0.0));
    double entropy0 = tracker.entropy();

    tracker.observeCells(json::array({{0, 0}}), 0.0);
    CHECK(tracker.coverageRatio() == doctest::Approx(1.0 / 9.0));
    CHECK(tracker.entropy() < entropy0);
    CHECK_FALSE(tracker.detected());

    tracker.observeRobots({Point(150.0, 150.0)}, 4.0);
    CHECK(tracker.detected());
    CHECK(tracker.detectionTime() == doctest::Approx(4.0));
    CHECK(tracker.beliefAtTarget() > 0.0);

    json snapshot = tracker.snapshot();
    CHECK(snapshot.at("detected").get<bool>());
    CHECK(snapshot.at("detection_time_s").get<double>() == doctest::Approx(4.0));
    CHECK(snapshot.at("coverage_ratio").get<double>() == doctest::Approx(1.0 / 9.0));
}

TEST_CASE("BridgeSearchTrackerDetectsTargetWhenSensorCoversTargetCell") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {300.0, 0.0}, {300.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 150.0}, {"y", 150.0}, {"radius", 80.0}}},
            {"search", {
                {"negative-detection-factor", 0.70},
                {"positive-detection-factor", 3.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);

    tracker.observeCells(json::array({{1, 1}}), 7.0);

    CHECK(tracker.detected());
    CHECK(tracker.detectionTime() == doctest::Approx(7.0));
    CHECK(tracker.snapshot().at("detected").get<bool>());
}

TEST_CASE("BridgeSearchTrackerSelectsUnsearchedHighBeliefGoal") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {300.0, 0.0}, {300.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 250.0}, {"y", 250.0}, {"radius", 50.0}}},
            {"search", {
                {"clarity-weight", 1.0},
                {"belief-weight", 4.0},
                {"travel-weight", 0.001}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    tracker.observeCells(json::array({{0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {2, 1}, {0, 2}, {1, 2}}), 0.0);

    Point goal = tracker.chooseGoal(Point(0.0, 0.0), "active");

    CHECK(goal.x == doctest::Approx(250.0));
    CHECK(goal.y == doctest::Approx(250.0));
}

TEST_CASE("BridgeSearchTrackerTargetPriorBiasesActiveGoalBeforeObservations") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {300.0, 0.0}, {300.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 250.0}, {"y", 250.0}, {"radius", 50.0}}},
            {"search", {
                {"belief-weight", 4.0},
                {"travel-weight", 0.001},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 80.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);

    Point activeGoal = tracker.chooseGoal(Point(0.0, 0.0), "active");
    Point coverageGoal = tracker.chooseGoal(Point(0.0, 0.0), "coverage");

    CHECK(activeGoal.x == doctest::Approx(250.0));
    CHECK(activeGoal.y == doctest::Approx(250.0));
    CHECK(coverageGoal.x == doctest::Approx(50.0));
    CHECK(coverageGoal.y == doctest::Approx(50.0));
}

TEST_CASE("BridgeSearchTrackerTargetPriorCanUseOffsetCenter") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {400.0, 0.0}, {400.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 350.0}, {"y", 250.0}, {"radius", 50.0}}},
            {"search", {
                {"belief-weight", 4.0},
                {"travel-weight", 0.001},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 60.0},
                {"target-prior-center", {{"x", 50.0}, {"y", 50.0}}}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);

    Point activeGoal = tracker.chooseGoal(Point(350.0, 250.0), "active");

    CHECK(activeGoal.x == doctest::Approx(50.0));
    CHECK(activeGoal.y == doctest::Approx(50.0));
}

TEST_CASE("BridgeSearchTrackerFallsBackToCoverageGoalAfterDetection") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {300.0, 0.0}, {300.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 250.0}, {"y", 250.0}, {"radius", 60.0}}},
            {"search", {
                {"belief-weight", 4.0},
                {"travel-weight", 0.001},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 80.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);

    tracker.observeRobots({Point(250.0, 250.0)}, 10.0);

    Point goal = tracker.chooseGoal(Point(0.0, 0.0), "active");

    CHECK(tracker.detected());
    CHECK(goal.x == doctest::Approx(50.0));
    CHECK(goal.y == doctest::Approx(50.0));
}

TEST_CASE("BridgeSearchTrackerKeepsGoalsAwayFromBoundaryMargin") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {300.0, 0.0}, {300.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 250.0}, {"y", 250.0}, {"radius", 60.0}}},
            {"search", {
                {"travel-weight", 0.001},
                {"goal-boundary-margin", 100.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);

    Point goal = tracker.chooseGoal(Point(0.0, 0.0), "coverage");

    CHECK(goal.x == doctest::Approx(150.0));
    CHECK(goal.y == doctest::Approx(150.0));
}

TEST_CASE("BridgeSearchTrackerPredictivePolicyPenalizesRiskyLocalizationGoal") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {400.0, 0.0}, {400.0, 100.0}, {0.0, 100.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 350.0}, {"y", 50.0}, {"radius", 40.0}}},
            {"search", {
                {"belief-weight", 4.0},
                {"travel-weight", 0.001},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 40.0},
                {"predictive-feasibility-weight", 5.0},
                {"predictive-robust-margin", 40.0},
                {"predictive-horizon", 2},
                {"predictive-step-m", 300.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    BridgeTopologyConfig topology;
    topology.maxRange = 220.0;
    topology.healthyQuality = 1.0;
    topology.deniedQuality = 1.0;
    topology.sigma0 = 0.0;
    topology.referenceVariance = 0.0;
    topology.uncertaintyMultiplier = 0.0;

    Point position(50.0, 50.0);
    Point anchor(0.0, 50.0);

    Point activeGoal = tracker.chooseGoal(position, "active");
    Point predictiveGoal = tracker.chooseGoal(position, "active-predictive", anchor, topology);

    CHECK(activeGoal.x == doctest::Approx(350.0));
    CHECK(predictiveGoal.x < activeGoal.x);
    CHECK(bridgeRobustMargin(anchor, predictiveGoal, topology) >= doctest::Approx(40.0).epsilon(1.0e-6));
}

TEST_CASE("BridgeSearchTrackerPredictiveDiagnosticsReportGateIntervention") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {400.0, 0.0}, {400.0, 100.0}, {0.0, 100.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 350.0}, {"y", 50.0}, {"radius", 40.0}}},
            {"search", {
                {"belief-weight", 4.0},
                {"travel-weight", 0.001},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 40.0},
                {"predictive-feasibility-weight", 5.0},
                {"predictive-robust-margin", 40.0},
                {"predictive-horizon", 2},
                {"predictive-step-m", 300.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    BridgeTopologyConfig topology;
    topology.maxRange = 220.0;
    topology.healthyQuality = 1.0;
    topology.deniedQuality = 1.0;
    topology.sigma0 = 0.0;
    topology.referenceVariance = 0.0;
    topology.uncertaintyMultiplier = 0.0;

    Point position(50.0, 50.0);
    Point anchor(0.0, 50.0);

    auto decision = tracker.chooseGoalWithDiagnostics(position, "active-predictive", anchor, topology);

    CHECK(decision.predictiveEnabled);
    CHECK(decision.predictiveChangedGoal);
    CHECK(decision.baselineGoal.x == doctest::Approx(350.0));
    CHECK(decision.goal.x < decision.baselineGoal.x);
    CHECK(decision.evaluatedCandidates > 0);
    CHECK(decision.penalizedCandidates > 0);
    CHECK(decision.maxPenalty > 0.0);
    CHECK(decision.selectedPenalty < decision.maxPenalty);
    CHECK(decision.selectedMinRobustMargin + 1.0e-6 >= 40.0);
    CHECK(decision.requiredRobustMargin == doctest::Approx(40.0));
}

TEST_CASE("BridgeSearchTrackerExposurePolicyChoosesViewingGoal") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {500.0, 0.0}, {500.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 450.0}, {"y", 150.0}, {"radius", 40.0}}},
            {"search", {
                {"clarity-weight", 0.0},
                {"belief-weight", 4.0},
                {"travel-weight", 0.0},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 35.0},
                {"predictive-feasibility-weight", 0.01},
                {"predictive-robust-margin", 0.0},
                {"predictive-horizon", 1},
                {"predictive-step-m", 100.0},
                {"exposure-weight", 5000.0},
                {"exposure-radius-m", 180.0},
                {"exposure-half-angle-deg", 25.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    BridgeTopologyConfig topology;
    topology.maxRange = 1000.0;
    topology.healthyQuality = 1.0;
    topology.deniedQuality = 1.0;
    topology.sigma0 = 0.0;
    topology.referenceVariance = 0.0;
    topology.uncertaintyMultiplier = 0.0;

    Point position(50.0, 150.0);
    Point anchor(0.0, 150.0);

    Point activeGoal = tracker.chooseGoal(position, "active-predictive", anchor, topology);
    Point exposureGoal = tracker.chooseGoal(position, "active-predictive-exposure", anchor, topology);
    auto exposureDecision = tracker.chooseGoalWithDiagnostics(position, "active-predictive-exposure", anchor, topology);

    CHECK(activeGoal.x == doctest::Approx(450.0));
    CHECK(exposureGoal.x < activeGoal.x);
    CHECK(exposureGoal.y == doctest::Approx(150.0));
    CHECK(exposureDecision.exposureEnabled);
    CHECK(exposureDecision.selectedExposureUtility > 0.0);
}

TEST_CASE("BridgeSearchTrackerExposureLookaheadAccumulatesRouteUtility") {
    json oneStepConfig = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {600.0, 0.0}, {600.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 550.0}, {"y", 150.0}, {"radius", 40.0}}},
            {"search", {
                {"clarity-weight", 0.0},
                {"belief-weight", 0.0},
                {"travel-weight", 0.0},
                {"predictive-feasibility-weight", 0.0},
                {"exposure-weight", 1000.0},
                {"exposure-radius-m", 140.0},
                {"exposure-half-angle-deg", 20.0},
                {"exposure-lookahead-steps", 1},
                {"exposure-lookahead-discount", 1.0}
            }}
        }}
    };
    json lookaheadConfig = oneStepConfig;
    lookaheadConfig["bridge"]["search"]["exposure-lookahead-steps"] = 4;
    lookaheadConfig["bridge"]["search"]["exposure-lookahead-step-m"] = 100.0;

    BridgeExperimentConfig oneStepBridge = loadBridgeExperimentConfig(oneStepConfig);
    BridgeExperimentConfig lookaheadBridge = loadBridgeExperimentConfig(lookaheadConfig);
    BridgeSearchTracker oneStepTracker(oneStepConfig, oneStepBridge);
    BridgeSearchTracker lookaheadTracker(lookaheadConfig, lookaheadBridge);
    BridgeTopologyConfig topology;
    topology.maxRange = 1000.0;
    topology.healthyQuality = 1.0;
    topology.deniedQuality = 1.0;
    topology.sigma0 = 0.0;
    topology.referenceVariance = 0.0;
    topology.uncertaintyMultiplier = 0.0;

    Point position(50.0, 150.0);
    Point anchor(0.0, 150.0);

    auto oneStepDecision = oneStepTracker.chooseGoalWithDiagnostics(position, "active-predictive-exposure", anchor, topology);
    auto lookaheadDecision = lookaheadTracker.chooseGoalWithDiagnostics(position, "active-predictive-exposure", anchor, topology);

    CHECK(oneStepDecision.exposureEnabled);
    CHECK(lookaheadDecision.exposureEnabled);
    CHECK(oneStepDecision.maxExposureUtility > 0.0);
    CHECK(lookaheadDecision.maxExposureUtility > oneStepDecision.maxExposureUtility);
    CHECK(lookaheadDecision.selectedExposureUtility >= oneStepDecision.selectedExposureUtility);
}

TEST_CASE("BridgeSearchTrackerExposureServiceGateRejectsLowServiceCandidates") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {500.0, 0.0}, {500.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 450.0}, {"y", 150.0}, {"radius", 40.0}}},
            {"search", {
                {"clarity-weight", 0.0},
                {"belief-weight", 4.0},
                {"travel-weight", 0.0},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 35.0},
                {"predictive-feasibility-weight", 0.01},
                {"predictive-robust-margin", 0.0},
                {"predictive-horizon", 1},
                {"predictive-step-m", 100.0},
                {"exposure-weight", 5000.0},
                {"exposure-radius-m", 180.0},
                {"exposure-half-angle-deg", 25.0},
                {"exposure-service-gate-min-cells", 1000.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    BridgeTopologyConfig topology;
    topology.maxRange = 1000.0;
    topology.healthyQuality = 1.0;
    topology.deniedQuality = 1.0;
    topology.sigma0 = 0.0;
    topology.referenceVariance = 0.0;
    topology.uncertaintyMultiplier = 0.0;

    Point position(50.0, 150.0);
    Point anchor(0.0, 150.0);

    Point activeGoal = tracker.chooseGoal(position, "active-predictive", anchor, topology);
    auto gatedDecision = tracker.chooseGoalWithDiagnostics(position, "active-predictive-exposure", anchor, topology);

    CHECK(gatedDecision.exposureEnabled);
    CHECK(gatedDecision.exposureServiceGateEnabled);
    CHECK(gatedDecision.requiredServiceUtility == doctest::Approx(1000.0));
    CHECK(gatedDecision.maxExposureUtility > 0.0);
    CHECK(gatedDecision.serviceRejectedCandidates > 0);
    CHECK_FALSE(gatedDecision.selectedExposureServiceEligible);
    CHECK(gatedDecision.selectedExposureUtility == doctest::Approx(0.0));
    CHECK(gatedDecision.goal.x == doctest::Approx(activeGoal.x));
    CHECK(gatedDecision.goal.y == doctest::Approx(activeGoal.y));
}

TEST_CASE("BridgeSearchTrackerServiceScheduleFallsBackToCoverageWhenBehind") {
    json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {500.0, 0.0}, {500.0, 300.0}, {0.0, 300.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 450.0}, {"y", 150.0}, {"radius", 40.0}}},
            {"search", {
                {"clarity-weight", 1.0},
                {"belief-weight", 200.0},
                {"travel-weight", 0.001},
                {"target-prior-strength", 1000.0},
                {"target-prior-sigma", 35.0},
                {"predictive-feasibility-weight", 0.01},
                {"predictive-robust-margin", 0.0},
                {"predictive-horizon", 1},
                {"predictive-step-m", 100.0},
                {"exposure-weight", 100.0},
                {"exposure-radius-m", 180.0},
                {"exposure-half-angle-deg", 25.0},
                {"exposure-service-schedule-rate-cells-per-s", 0.5},
                {"exposure-service-schedule-slack-cells", 0.0}
            }}
        }}
    };
    BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    BridgeTopologyConfig topology;
    topology.maxRange = 1000.0;
    topology.healthyQuality = 1.0;
    topology.deniedQuality = 1.0;
    topology.sigma0 = 0.0;
    topology.referenceVariance = 0.0;
    topology.uncertaintyMultiplier = 0.0;

    Point position(50.0, 50.0);
    Point anchor(0.0, 50.0);
    Point activeGoal = tracker.chooseGoal(position, "active-predictive-exposure", anchor, topology);
    Point coverageGoal = tracker.chooseGoal(position, "coverage");
    CHECK(activeGoal.distance_to(coverageGoal) > 1.0);

    tracker.observeCells(json::array(), 10.0);
    Point scheduledGoal = tracker.chooseGoal(position, "active-predictive-exposure", anchor, topology);
    CHECK(scheduledGoal.x == doctest::Approx(coverageGoal.x));
    CHECK(scheduledGoal.y == doctest::Approx(coverageGoal.y));
}

TEST_CASE("single-ladder joint goal uses the frozen positive sixty degree rotation") {
    const auto tuple = bridgeBuildSingleLadderGoalTuple(
            Point(250.0, 1000.0),
            Point(250.0, 1510.0),
            Point(1150.0, 450.0));

    CHECK(tuple.goals.at(0).distance_to(
                  Point(823.5752250232366, 1443.4614317029973)) < 1.0e-9);
    CHECK(tuple.goals.at(1).distance_to(
                  Point(700.0, 852.5)) < 1.0e-9);
    CHECK(tuple.goals.at(2).distance_to(
                  Point(1273.5752250232367, 1040.9614317029973)) < 1.0e-9);
    CHECK(tuple.goals.at(3).distance_to(Point(1150.0, 450.0)) < 1.0e-12);
}

TEST_CASE("single-ladder joint goal certifies exactly the canonical eight edges") {
    const Point base0(250.0, 1000.0);
    const Point base1(250.0, 1510.0);
    const auto tuple = bridgeBuildSingleLadderGoalTuple(
            base0, base1, Point(1150.0, 450.0));

    const auto certificate = bridgeCertifySingleLadderGoalTuple(
            tuple,
            base0,
            base1,
            bridgeSingleTriangularLadderReferences(),
            Point(0.0, 0.0),
            Point(1800.0, 2000.0),
            849.0,
            10.0);

    REQUIRE(certificate.valid);
    CHECK(certificate.rejectionReasons.empty());
    REQUIRE(certificate.edges.size() == 8);
    CHECK(certificate.edges.at(0).ownerId == 1);
    CHECK(certificate.edges.at(0).referenceKind == "base");
    CHECK(certificate.edges.at(0).referenceId == 0);
    CHECK(certificate.edges.at(7).ownerId == 4);
    CHECK(certificate.edges.at(7).referenceKind == "mobile");
    CHECK(certificate.edges.at(7).referenceId == 2);
    for (const auto &edge : certificate.edges) {
        CHECK(edge.length <= 849.0);
    }
    REQUIRE(certificate.mobileSeparations.size() == 6);
    for (const double separation : certificate.mobileSeparations) {
        CHECK(separation >= 10.0);
    }
}

TEST_CASE("single-ladder joint goal certificate fails closed on malformed geometry") {
    const Point base0(250.0, 1000.0);
    const Point base1(250.0, 1510.0);
    const Point worldMin(0.0, 0.0);
    const Point worldMax(1800.0, 2000.0);
    const auto references = bridgeSingleTriangularLadderReferences();

    SUBCASE("outside world") {
        const auto tuple = bridgeBuildSingleLadderGoalTuple(
                base0, base1, Point(1750.0, 50.0));
        const auto certificate = bridgeCertifySingleLadderGoalTuple(
                tuple, base0, base1, references,
                worldMin, worldMax, 849.0, 10.0);
        CHECK_FALSE(certificate.valid);
        CHECK(std::find(certificate.rejectionReasons.begin(),
                        certificate.rejectionReasons.end(),
                        "goal-outside-world")
              != certificate.rejectionReasons.end());
    }

    SUBCASE("assigned edge over range") {
        const auto tuple = bridgeBuildSingleLadderGoalTuple(
                base0, base1, Point(1750.0, 1950.0));
        const auto certificate = bridgeCertifySingleLadderGoalTuple(
                tuple, base0, base1, references,
                Point(-2000.0, -2000.0), Point(3000.0, 3000.0),
                849.0, 10.0);
        CHECK_FALSE(certificate.valid);
        CHECK(std::find(certificate.rejectionReasons.begin(),
                        certificate.rejectionReasons.end(),
                        "assigned-edge-over-range")
              != certificate.rejectionReasons.end());
    }

    SUBCASE("mobile separation below minimum") {
        const auto tuple = bridgeBuildSingleLadderGoalTuple(
                base0, base1, Point(250.0, 1255.0));
        const auto certificate = bridgeCertifySingleLadderGoalTuple(
                tuple, base0, base1, references,
                worldMin, worldMax, 849.0, 10.0);
        CHECK_FALSE(certificate.valid);
        CHECK(std::find(certificate.rejectionReasons.begin(),
                        certificate.rejectionReasons.end(),
                        "mobile-separation-below-minimum")
              != certificate.rejectionReasons.end());
    }

    SUBCASE("reference map mismatch") {
        auto malformedReferences = references;
        malformedReferences.at(4).anchorIds = {2, 3};
        const auto tuple = bridgeBuildSingleLadderGoalTuple(
                base0, base1, Point(1150.0, 450.0));
        const auto certificate = bridgeCertifySingleLadderGoalTuple(
                tuple, base0, base1, malformedReferences,
                worldMin, worldMax, 849.0, 10.0);
        CHECK_FALSE(certificate.valid);
        CHECK(std::find(certificate.rejectionReasons.begin(),
                        certificate.rejectionReasons.end(),
                        "reference-map-mismatch")
              != certificate.rejectionReasons.end());
    }

    SUBCASE("non-finite goal") {
        auto tuple = bridgeBuildSingleLadderGoalTuple(
                base0, base1, Point(1150.0, 450.0));
        tuple.goals.at(2).x = std::numeric_limits<double>::quiet_NaN();
        const auto certificate = bridgeCertifySingleLadderGoalTuple(
                tuple, base0, base1, references,
                worldMin, worldMax, 849.0, 10.0);
        CHECK_FALSE(certificate.valid);
        CHECK(std::find(certificate.rejectionReasons.begin(),
                        certificate.rejectionReasons.end(),
                        "non-finite-goal")
              != certificate.rejectionReasons.end());
    }

    SUBCASE("tuple does not reproduce the frozen section geometry") {
        auto tuple = bridgeBuildSingleLadderGoalTuple(
                base0, base1, Point(1150.0, 450.0));
        tuple.goals.at(1).x += 1.0;
        const auto certificate = bridgeCertifySingleLadderGoalTuple(
                tuple, base0, base1, references,
                worldMin, worldMax, 849.0, 10.0);
        CHECK_FALSE(certificate.valid);
        CHECK(std::find(certificate.rejectionReasons.begin(),
                        certificate.rejectionReasons.end(),
                        "tuple-geometry-mismatch")
              != certificate.rejectionReasons.end());
    }
}

TEST_CASE("single-ladder joint goal certificate uses exact closed thresholds") {
    const Point commonBase(0.0, 0.0);
    const auto references = bridgeSingleTriangularLadderReferences();

    const auto rangeBoundary = bridgeBuildSingleLadderGoalTuple(
            commonBase, commonBase, Point(1698.0, 0.0));
    const auto rangeCertificate = bridgeCertifySingleLadderGoalTuple(
            rangeBoundary, commonBase, commonBase, references,
            Point(-1000.0, -1000.0), Point(3000.0, 3000.0),
            849.0, 10.0);
    CHECK(rangeCertificate.valid);
    for (const auto &edge : rangeCertificate.edges) {
        CHECK(edge.length == doctest::Approx(849.0));
    }

    const auto separationBoundary = bridgeBuildSingleLadderGoalTuple(
            commonBase, commonBase, Point(20.0, 0.0));
    const auto separationCertificate = bridgeCertifySingleLadderGoalTuple(
            separationBoundary, commonBase, commonBase, references,
            Point(-100.0, -100.0), Point(100.0, 100.0),
            849.0, 10.0);
    CHECK(separationCertificate.valid);
    CHECK(*std::min_element(
                  separationCertificate.mobileSeparations.begin(),
                  separationCertificate.mobileSeparations.end())
          == doctest::Approx(10.0));

    const auto belowSeparation = bridgeBuildSingleLadderGoalTuple(
            commonBase, commonBase, Point(19.0, 0.0));
    const auto belowCertificate = bridgeCertifySingleLadderGoalTuple(
            belowSeparation, commonBase, commonBase, references,
            Point(-100.0, -100.0), Point(100.0, 100.0),
            849.0, 10.0);
    CHECK_FALSE(belowCertificate.valid);
}

TEST_CASE("BridgeSearchTracker selects and persists the nearest feasible single-ladder joint goal") {
    const json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {1800.0, 0.0},
                          {1800.0, 2000.0}, {0.0, 2000.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {
            {"enabled", true},
            {"target", {{"x", 1450.0}, {"y", 350.0}, {"radius", 50.0}}}
        }}
    };
    const BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    const std::array<Point, 2> bases = {
            Point(250.0, 1000.0), Point(250.0, 1510.0)};

    const auto first = tracker.choosePersistentSingleLadderGoals(
            Point(1133.3459118601275, 490.0),
            bases,
            bridgeSingleTriangularLadderReferences());
    CHECK(first.selectedRow == 4);
    CHECK(first.selectedColumn == 11);
    CHECK(first.tuple.leaderGoal.distance_to(Point(1150.0, 450.0)) < 1.0e-12);
    CHECK(first.certificate.valid);
    CHECK_FALSE(first.reusedPrevious);
    CHECK(first.selectionEpoch == 1);
    CHECK(first.unsearchedCells.size() == 360);

    const auto held = tracker.choosePersistentSingleLadderGoals(
            Point(900.0, 900.0),
            bases,
            bridgeSingleTriangularLadderReferences());
    CHECK(held.reusedPrevious);
    CHECK(held.selectionEpoch == first.selectionEpoch);
    CHECK(held.tuple.leaderGoal.distance_to(first.tuple.leaderGoal) < 1.0e-12);
}

TEST_CASE("BridgeSearchTracker rejects nearer incompatible ladder goals and uses exact grid tie breaks") {
    const json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {1800.0, 0.0},
                          {1800.0, 2000.0}, {0.0, 2000.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {{"enabled", true},
                    {"target", {{"x", 1450.0}, {"y", 350.0},
                                {"radius", 50.0}}}}}
    };
    const BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    const std::array<Point, 2> bases = {
            Point(250.0, 1000.0), Point(250.0, 1510.0)};

    SUBCASE("nearest incompatible candidate is retained in the rejection ledger") {
        BridgeSearchTracker tracker(config, bridge);
        const auto decision = tracker.choosePersistentSingleLadderGoals(
                Point(1750.0, 50.0), bases,
                bridgeSingleTriangularLadderReferences());
        REQUIRE(decision.candidates.size() > 1);
        CHECK(decision.candidates.front().row == 0);
        CHECK(decision.candidates.front().column == 17);
        CHECK_FALSE(decision.candidates.front().accepted);
        CHECK(decision.certificate.valid);
        CHECK(decision.selectedRow != 0);
    }

    SUBCASE("equal squared distance uses row then column") {
        BridgeSearchTracker tracker(config, bridge);
        const auto decision = tracker.choosePersistentSingleLadderGoals(
                Point(1100.0, 500.0), bases,
                bridgeSingleTriangularLadderReferences());
        CHECK(decision.selectedRow == 4);
        CHECK(decision.selectedColumn == 10);
    }
}

TEST_CASE("BridgeSearchTracker changes epoch only after the leader cell is searched") {
    const json config = {
        {"world", {
            {"boundary", {{0.0, 0.0}, {1800.0, 0.0},
                          {1800.0, 2000.0}, {0.0, 2000.0}}},
            {"spacing", 100.0}
        }},
        {"bridge", {{"enabled", true},
                    {"target", {{"x", 1450.0}, {"y", 350.0},
                                {"radius", 50.0}}}}}
    };
    const BridgeExperimentConfig bridge = loadBridgeExperimentConfig(config);
    BridgeSearchTracker tracker(config, bridge);
    const std::array<Point, 2> bases = {
            Point(250.0, 1000.0), Point(250.0, 1510.0)};
    const auto references = bridgeSingleTriangularLadderReferences();
    const auto first = tracker.choosePersistentSingleLadderGoals(
            Point(1133.3459118601275, 490.0), bases, references);

    tracker.observeCells(json::array({{11, 4}}), 0.5);
    const auto second = tracker.choosePersistentSingleLadderGoals(
            Point(1133.3459118601275, 490.0), bases, references);
    CHECK_FALSE(second.reusedPrevious);
    CHECK(second.selectionEpoch == first.selectionEpoch + 1);
    CHECK((second.selectedRow != first.selectedRow
           || second.selectedColumn != first.selectedColumn));

    json allCells = json::array();
    for (int row = 0; row < 20; ++row) {
        for (int column = 0; column < 18; ++column) {
            allCells.push_back({column, row});
        }
    }
    tracker.observeCells(allCells, 1.0);
    const auto exhausted = tracker.choosePersistentSingleLadderGoals(
            Point(1133.3459118601275, 490.0), bases, references);
    CHECK(exhausted.reusedPrevious);
    CHECK(exhausted.noFeasibleGoal);
    CHECK(exhausted.selectionEpoch == second.selectionEpoch);
    CHECK(exhausted.tuple.leaderGoal.distance_to(second.tuple.leaderGoal)
          < 1.0e-12);
}

TEST_CASE("BridgeTopologyRejectsDeniedDirectLinkAndAcceptsRelay") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.denialCenter = Point(1500.0, 1500.0);
    config.denialHalfSize = Point(500.0, 500.0);
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    std::map<int, Point> positions = {
        {1, Point(300.0, 1500.0)},
        {2, Point(1200.0, 1500.0)},
        {3, Point(2300.0, 1500.0)}
    };

    BridgeTopologyDecision fixed = chooseBridgeTopology(positions, config, "fixed");
    BridgeTopologyDecision adaptive = chooseBridgeTopology(positions, config, "adaptive-relay");

    CHECK(fixed.accepted);
    CHECK(fixed.anchorIds.at(3).front() == 1);
    CHECK(adaptive.accepted);
    CHECK(adaptive.anchorIds.at(2).front() == 1);
    CHECK(adaptive.anchorIds.at(3).front() == 2);
    CHECK(adaptive.relayActive);
    CHECK(adaptive.rejectedCandidates >= 1);
    CHECK(adaptive.minRobustMargin >= fixed.minRobustMargin);
}

TEST_CASE("BridgeTopologyCertifiedOnlyTriggersFailSafeForUncertifiedFixedGraph") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.certifiedOnly = true;
    config.healthyQuality = 1.0;
    config.deniedQuality = 1.0;
    config.sigma0 = 0.0;
    config.referenceVariance = 0.0;
    config.uncertaintyMultiplier = 0.0;

    std::map<int, Point> positions = {
        {1, Point(0.0, 0.0)},
        {2, Point(600.0, 0.0)},
        {3, Point(1300.0, 0.0)}
    };

    BridgeTopologyDecision fixed = chooseBridgeTopology(positions, config, "fixed");
    BridgeTopologyDecision adaptive = chooseBridgeTopology(positions, config, "adaptive-relay");

    CHECK(fixed.accepted);
    CHECK_FALSE(fixed.certified);
    CHECK(fixed.failSafe);
    CHECK(fixed.log.at("certified") == false);
    CHECK(fixed.log.at("fail_safe") == true);
    CHECK(adaptive.accepted);
    CHECK(adaptive.certified);
    CHECK_FALSE(adaptive.failSafe);
    CHECK(adaptive.anchorIds.at(3).front() == 2);
}

TEST_CASE("BridgeTopologyAdaptiveChainUsesCertifiedUpstreamAnchors") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.certifiedOnly = true;
    config.healthyQuality = 1.0;
    config.deniedQuality = 1.0;
    config.sigma0 = 0.0;
    config.referenceVariance = 0.0;
    config.uncertaintyMultiplier = 0.0;

    std::map<int, Point> positions = {
        {1, Point(0.0, 0.0)},
        {2, Point(900.0, 0.0)},
        {3, Point(1800.0, 0.0)},
        {4, Point(2700.0, 0.0)}
    };

    BridgeTopologyDecision chain = chooseBridgeTopology(positions, config, "adaptive-chain");

    CHECK(chain.accepted);
    CHECK(chain.certified);
    CHECK_FALSE(chain.failSafe);
    CHECK(chain.relayActive);
    CHECK(chain.anchorIds.at(2).front() == 1);
    CHECK(chain.anchorIds.at(3).front() == 2);
    CHECK(chain.anchorIds.at(4).front() == 3);
    CHECK(chain.minRobustMargin == doctest::Approx(300.0));
}

TEST_CASE("BridgeTopologyPrefersPhysicallyFeasibleRelayOverInfeasibleDirectLink") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.physicalSwitchMargin = 10.0;
    config.denialCenter = Point(1200.0, 1500.0);
    config.denialHalfSize = Point(100.0, 100.0);
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    std::map<int, Point> positions = {
        {1, Point(0.0, 1500.0)},
        {2, Point(800.0, 1500.0)},
        {3, Point(1250.0, 1500.0)}
    };

    BridgeTopologyDecision adaptive = chooseBridgeTopology(positions, config, "adaptive-relay");

    CHECK(adaptive.anchorIds.at(3).front() == 2);
    CHECK(adaptive.relayActive);
}

TEST_CASE("BridgeTopologyKeepsRelayWhenDirectLinkIsPhysicallyImpossibleAtMarginBoundary") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.physicalSwitchMargin = 10.0;
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    std::map<int, Point> positions = {
        {1, Point(478.5, 1861.2)},
        {2, Point(1132.4, 2180.9)},
        {3, Point(1755.2, 1176.6)},
        {4, Point(2107.7, 1497.3)}
    };

    BridgeTopologyDecision adaptive = chooseBridgeTopology(positions, config, "adaptive-relay");

    CHECK(adaptive.anchorIds.at(4).front() == 2);
    CHECK(adaptive.relayActive);
}

TEST_CASE("BridgeTopologyReserveSwitchesBeforeDirectMarginFails") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.robustSwitchMargin = 80.0;
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    std::map<int, Point> positions = {
        {1, Point(0.0, 0.0)},
        {2, Point(550.0, 0.0)},
        {3, Point(1100.0, 0.0)}
    };

    BridgeTopologyDecision reactive = chooseBridgeTopology(positions, config, "adaptive-relay");
    BridgeTopologyDecision reserve = chooseBridgeTopology(positions, config, "adaptive-relay-reserve");

    CHECK(reactive.anchorIds.at(3).front() == 1);
    CHECK_FALSE(reactive.relayActive);
    CHECK(reserve.anchorIds.at(3).front() == 2);
    CHECK(reserve.relayActive);
    CHECK(reserve.minRobustMargin > reactive.minRobustMargin);
}

TEST_CASE("BridgeRelaySupportGoalPullsRelayInsideRobustReserve") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    std::map<int, Point> positions = {
        {1, Point(0.0, 0.0)},
        {2, Point(1180.0, 0.0)},
        {3, Point(1700.0, 0.0)},
        {4, Point(1700.0, 200.0)}
    };

    double currentMargin = bridgeRobustMargin(positions.at(1), positions.at(2), config);
    Point supportGoal = bridgeRelaySupportGoal(positions, config, 80.0);
    double supportMargin = bridgeRobustMargin(positions.at(1), supportGoal, config);

    CHECK(currentMargin < 80.0);
    CHECK(supportGoal.x < positions.at(2).x);
    CHECK(supportMargin >= doctest::Approx(80.0).epsilon(1.0e-6));
}

TEST_CASE("BridgeSupportChainGoalPullsDownstreamGoalInsideAnchorReserve") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    Point anchor(0.0, 0.0);
    Point current(900.0, 0.0);
    Point desiredGoal(1400.0, 0.0);

    Point gatedGoal = bridgeSupportChainGoal(anchor, current, desiredGoal, config, 80.0);
    double gatedMargin = bridgeRobustMargin(anchor, gatedGoal, config);

    CHECK(gatedGoal.x > current.x);
    CHECK(gatedGoal.x < desiredGoal.x);
    CHECK(gatedMargin >= doctest::Approx(80.0).epsilon(1.0e-6));
}

TEST_CASE("BridgeSupportChainGoalLeavesInteriorGoalUnchanged") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    Point anchor(0.0, 0.0);
    Point current(900.0, 0.0);
    Point desiredGoal(950.0, 30.0);

    Point gatedGoal = bridgeSupportChainGoal(anchor, current, desiredGoal, config, 80.0);

    CHECK(gatedGoal.x == doctest::Approx(desiredGoal.x));
    CHECK(gatedGoal.y == doctest::Approx(desiredGoal.y));
}

TEST_CASE("BridgePredictiveSupportChainGoalIgnoresFarGoalWhenOneStepMarginIsSafe") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    Point anchor(0.0, 0.0);
    Point current(900.0, 0.0);
    Point predictedNext(905.0, 0.0);
    Point desiredGoal(1400.0, 0.0);

    Point gatedGoal = bridgePredictiveSupportChainGoal(anchor, current, predictedNext, desiredGoal, config, 25.0);

    CHECK(gatedGoal.x == doctest::Approx(desiredGoal.x));
    CHECK(gatedGoal.y == doctest::Approx(desiredGoal.y));
}

TEST_CASE("BridgePredictiveSupportChainGoalProjectsWhenOneStepMarginIsUnsafe") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.healthyQuality = 0.95;
    config.deniedQuality = 0.15;

    Point anchor(0.0, 0.0);
    Point current(1080.0, 0.0);
    Point predictedNext(1150.0, 0.0);
    Point desiredGoal(1400.0, 0.0);

    Point gatedGoal = bridgePredictiveSupportChainGoal(anchor, current, predictedNext, desiredGoal, config, 25.0);
    double gatedMargin = bridgeRobustMargin(anchor, gatedGoal, config);

    CHECK(gatedGoal.x < desiredGoal.x);
    CHECK(gatedMargin >= doctest::Approx(25.0).epsilon(1.0e-6));
}

TEST_CASE("BridgePredictiveMovingAnchorSupportGoalAccountsForAnchorMotion") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.healthyQuality = 1.0;
    config.deniedQuality = 1.0;
    config.sigma0 = 0.0;
    config.referenceVariance = 0.0;
    config.uncertaintyMultiplier = 0.0;

    Point anchorCurrent(0.0, 0.0);
    Point anchorPredicted(120.0, 0.0);
    Point childCurrent(1050.0, 0.0);
    Point childPredicted(1260.0, 0.0);
    Point desiredGoal(1400.0, 0.0);

    Point gatedGoal = bridgePredictiveMovingAnchorSupportGoal(
        anchorCurrent,
        anchorPredicted,
        childCurrent,
        childPredicted,
        desiredGoal,
        config,
        80.0);

    CHECK(gatedGoal.x < desiredGoal.x);
    CHECK(bridgeRobustMargin(anchorPredicted, gatedGoal, config) >= doctest::Approx(80.0).epsilon(1.0e-6));
}

TEST_CASE("BridgeOneStepSupportGoalSelectsBetterPredictedMargin") {
    BridgeTopologyConfig config;
    config.maxRange = 1200.0;
    config.healthyQuality = 1.0;
    config.deniedQuality = 1.0;
    config.sigma0 = 0.0;
    config.referenceVariance = 0.0;
    config.uncertaintyMultiplier = 0.0;

    Point anchorPredicted(0.0, 0.0);
    Point primaryGoal(900.0, 0.0);
    Point primaryPredicted(1300.0, 0.0);
    Point fallbackGoal(0.0, 0.0);
    Point fallbackPredicted(1120.0, 0.0);

    auto selected = bridgeChooseBetterOneStepSupportGoal(
        anchorPredicted,
        primaryGoal,
        primaryPredicted,
        fallbackGoal,
        fallbackPredicted,
        config);

    CHECK(selected.goal.x == doctest::Approx(fallbackGoal.x));
    CHECK(selected.predicted.x == doctest::Approx(fallbackPredicted.x));
    CHECK(selected.margin == doctest::Approx(80.0));
    CHECK(selected.improved);
}
