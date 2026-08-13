#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/InterimMasterDekf.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace {

std::vector<gf::MobileEstimate> initialMobiles() {
    std::vector<gf::MobileEstimate> mobiles;
    const std::vector<Eigen::Vector2d> positions = {
        {0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {0.0, 3.0}};
    for (std::size_t index = 0; index < positions.size(); ++index) {
        Eigen::Vector4d mean;
        mean << positions[index].x(), positions[index].y(),
                0.1 * static_cast<double>(index + 1),
                -0.05 * static_cast<double>(index + 1);
        Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity();
        covariance.diagonal() << 1.0 + 0.1 * index,
                                 0.8 + 0.1 * index,
                                 0.4, 0.3;
        mobiles.push_back(gf::MobileEstimate{
            static_cast<gf::NodeId>(index + 1), mean, covariance});
    }
    return mobiles;
}

std::map<gf::NodeId, Eigen::Vector2d> fixedAnchors() {
    return {{10, Eigen::Vector2d{-3.0, 0.0}},
            {11, Eigen::Vector2d{7.0, 3.0}}};
}

gf::RangeMeasurement range(
    std::int64_t timestamp_ns,
    gf::NodeId first,
    gf::NodeId second,
    double measured_range) {
    return gf::RangeMeasurement{
        timestamp_ns,
        gf::UndirectedEdge::canonical(first, second),
        measured_range,
        0.25};
}

void checkEquivalent(
    const gf::InterimMasterDekf& distributed,
    const gf::CentralizedEkfOracle& centralized,
    double tolerance = 1e-10) {
    const gf::JointEstimateSnapshot actual = distributed.reconstructForAudit();
    const gf::JointEstimateSnapshot expected = centralized.snapshot();
    REQUIRE(actual.mobile_ids == expected.mobile_ids);
    CHECK(actual.mean.isApprox(expected.mean, tolerance));
    CHECK(actual.covariance.isApprox(expected.covariance, tolerance));

    for (gf::NodeId id : actual.mobile_ids) {
        const gf::MarginalEstimate marginal = distributed.marginal(id);
        const std::size_t index = static_cast<std::size_t>(id - 1);
        CHECK(marginal.mean.isApprox(
            expected.mean.segment<4>(4 * index), tolerance));
        CHECK(marginal.covariance.isApprox(
            expected.covariance.block<4, 4>(4 * index, 4 * index),
            tolerance));
    }
    CHECK(distributed.crossCovariance(1, 3).isApprox(
        expected.covariance.block<4, 4>(0, 8), tolerance));
}

void applyMeasurement(
    gf::InterimMasterDekf& distributed,
    gf::CentralizedEkfOracle& centralized,
    gf::NodeId master,
    const gf::RangeMeasurement& measurement) {
    const gf::InterimMasterMessage message =
        distributed.makeUpdate(master, measurement);
    distributed.applyUpdate(message);
    centralized.update({measurement});
}

}  // namespace

TEST_CASE("Local second-order propagation equals the centralized oracle") {
    gf::InterimMasterDekf distributed(initialMobiles(), fixedAnchors());
    gf::CentralizedEkfOracle centralized(initialMobiles(), fixedAnchors());
    const std::vector<Eigen::Vector2d> accelerations = {
        {0.2, -0.1}, {-0.1, 0.3}, {0.0, -0.2}, {0.4, 0.1}};

    for (std::size_t index = 0; index < accelerations.size(); ++index) {
        distributed.propagateLocal(
            static_cast<gf::NodeId>(index + 1),
            accelerations[index], 0.2, 0.04);
    }
    centralized.propagate(accelerations, 0.2, 0.04);

    checkEquivalent(distributed, centralized);
}

TEST_CASE("Interim-master updates match the oracle after each edge of a cyclic batch") {
    gf::InterimMasterDekf distributed(initialMobiles(), fixedAnchors());
    gf::CentralizedEkfOracle centralized(initialMobiles(), fixedAnchors());
    const std::vector<gf::RangeMeasurement> batch =
        gf::canonicalizeRangeBatch({
            range(10, 2, 3, 3.05),
            range(10, 1, 2, 4.05),
            range(10, 1, 4, 3.02),
            range(10, 3, 4, 4.04),
            range(10, 1, 10, 3.01),
            range(10, 3, 11, 3.03)});

    for (const gf::RangeMeasurement& measurement : batch) {
        const gf::NodeId master =
            measurement.edge.first <= 4
                ? measurement.edge.first
                : measurement.edge.second;
        applyMeasurement(distributed, centralized, master, measurement);
        checkEquivalent(distributed, centralized);
    }
}

TEST_CASE("Multiple propagation and measurement batches retain oracle equivalence") {
    gf::InterimMasterDekf distributed(initialMobiles(), fixedAnchors());
    gf::CentralizedEkfOracle centralized(initialMobiles(), fixedAnchors());
    const std::vector<Eigen::Vector2d> first_acceleration = {
        {0.1, 0.0}, {0.0, 0.1}, {-0.1, 0.0}, {0.0, -0.1}};
    const std::vector<Eigen::Vector2d> second_acceleration = {
        {-0.2, 0.1}, {0.1, -0.2}, {0.2, 0.0}, {-0.1, 0.2}};

    for (std::size_t index = 0; index < 4; ++index) {
        distributed.propagateLocal(
            static_cast<gf::NodeId>(index + 1),
            first_acceleration[index], 0.1, 0.02);
    }
    centralized.propagate(first_acceleration, 0.1, 0.02);
    applyMeasurement(distributed, centralized, 2, range(20, 1, 2, 4.01));
    applyMeasurement(distributed, centralized, 1, range(20, 1, 10, 3.02));

    for (std::size_t index = 0; index < 4; ++index) {
        distributed.propagateLocal(
            static_cast<gf::NodeId>(index + 1),
            second_acceleration[index], 0.15, 0.03);
    }
    centralized.propagate(second_acceleration, 0.15, 0.03);
    applyMeasurement(distributed, centralized, 4, range(30, 3, 4, 4.02));
    applyMeasurement(distributed, centralized, 3, range(30, 3, 11, 3.04));

    checkEquivalent(distributed, centralized);
}

TEST_CASE("Interim-master messages reject invalid ownership order and version") {
    gf::InterimMasterDekf distributed(initialMobiles(), fixedAnchors());

    CHECK_THROWS_WITH_AS(
        distributed.makeUpdate(4, range(10, 1, 2, 4.0)),
        "interim master must be a mobile endpoint of the measured edge",
        std::invalid_argument);

    const gf::InterimMasterMessage first =
        distributed.makeUpdate(1, range(10, 1, 2, 4.0));
    distributed.applyUpdate(first);
    CHECK_THROWS_WITH_AS(
        distributed.applyUpdate(first),
        "stale or repeated interim-master message",
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(
        distributed.makeUpdate(1, range(9, 1, 4, 3.0)),
        "range measurements must be applied in canonical order",
        std::invalid_argument);

    const gf::InterimMasterMessage stale =
        distributed.makeUpdate(2, range(20, 2, 3, 3.0));
    distributed.propagateLocal(1, Eigen::Vector2d::Zero(), 0.1, 0.0);
    CHECK_THROWS_WITH_AS(
        distributed.applyUpdate(stale),
        "stale or repeated interim-master message",
        std::invalid_argument);
}

TEST_CASE("Malformed broadcast factors and non-PSD initialization fail closed") {
    gf::InterimMasterDekf distributed(initialMobiles(), fixedAnchors());
    gf::InterimMasterMessage malformed =
        distributed.makeUpdate(1, range(10, 1, 2, 4.0));
    malformed.gamma_by_mobile.pop_back();
    CHECK_THROWS_WITH_AS(
        distributed.applyUpdate(malformed),
        "interim-master message dimension mismatch",
        std::invalid_argument);

    gf::InterimMasterMessage non_finite =
        distributed.makeUpdate(1, range(10, 1, 2, 4.2));
    non_finite.gamma_by_mobile[2](0) =
        std::numeric_limits<double>::quiet_NaN();
    const gf::JointEstimateSnapshot before = distributed.reconstructForAudit();
    CHECK_THROWS_WITH_AS(
        distributed.applyUpdate(non_finite),
        "invalid interim-master correlation factor",
        std::invalid_argument);
    const gf::JointEstimateSnapshot after = distributed.reconstructForAudit();
    CHECK(after.mean.isApprox(before.mean, 0.0));
    CHECK(after.covariance.isApprox(before.covariance, 0.0));

    std::vector<gf::MobileEstimate> invalid = initialMobiles();
    invalid.front().covariance(0, 0) = -1.0;
    CHECK_THROWS_AS(
        gf::InterimMasterDekf(invalid, fixedAnchors()),
        std::invalid_argument);
}
