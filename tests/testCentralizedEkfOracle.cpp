#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CentralizedEkfOracle.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <vector>

namespace {

gf::MobileEstimate mobile(
    gf::NodeId id,
    const Eigen::Vector4d& mean,
    const Eigen::Matrix4d& covariance = Eigen::Matrix4d::Identity()) {
    return gf::MobileEstimate{id, mean, covariance};
}

gf::RangeMeasurement range(
    std::int64_t timestamp_ns,
    gf::NodeId first,
    gf::NodeId second,
    double range_m,
    double variance_m2) {
    return gf::RangeMeasurement{
        timestamp_ns,
        gf::UndirectedEdge::canonical(first, second),
        range_m,
        variance_m2};
}

double minimumEigenvalue(const Eigen::MatrixXd& matrix) {
    return Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(matrix)
        .eigenvalues()
        .minCoeff();
}

gf::CentralizedEkfOracle ringOracle() {
    return gf::CentralizedEkfOracle(
        {
            mobile(1, (Eigen::Vector4d() << 0.0, 0.0, 0.0, 0.0).finished()),
            mobile(2, (Eigen::Vector4d() << 4.0, 0.0, 0.0, 0.0).finished()),
            mobile(3, (Eigen::Vector4d() << 0.0, 3.0, 0.0, 0.0).finished())
        },
        {{10, Eigen::Vector2d{-3.0, 0.0}}});
}

}  // namespace

TEST_CASE("Second-order prediction uses known acceleration and keeps anchors fixed") {
    gf::CentralizedEkfOracle oracle(
        {mobile(7, (Eigen::Vector4d() << 0.0, 0.0, 1.0, 2.0).finished())},
        {{20, Eigen::Vector2d{9.0, 8.0}}});

    oracle.propagate({Eigen::Vector2d{2.0, -1.0}}, 0.5, 0.0);
    const gf::JointEstimateSnapshot snapshot = oracle.snapshot();

    REQUIRE(snapshot.mobile_ids == std::vector<gf::NodeId>{7});
    CHECK(snapshot.mean(0) == doctest::Approx(0.75));
    CHECK(snapshot.mean(1) == doctest::Approx(0.875));
    CHECK(snapshot.mean(2) == doctest::Approx(2.0));
    CHECK(snapshot.mean(3) == doctest::Approx(1.5));
    CHECK(snapshot.fixed_position(20).isApprox(Eigen::Vector2d{9.0, 8.0}));
}

TEST_CASE("A scalar anchor range update matches the hand-derived Kalman result") {
    gf::CentralizedEkfOracle oracle(
        {mobile(1, (Eigen::Vector4d() << 3.0, 0.0, 0.0, 0.0).finished())},
        {{10, Eigen::Vector2d{0.0, 0.0}}});

    oracle.update({range(1, 1, 10, 2.0, 1.0)});
    const gf::JointEstimateSnapshot snapshot = oracle.snapshot();

    CHECK(snapshot.mean(0) == doctest::Approx(2.5).epsilon(1e-12));
    CHECK(snapshot.covariance(0, 0) == doctest::Approx(0.5).epsilon(1e-12));
    CHECK(snapshot.covariance.trace() < 4.0);
}

TEST_CASE("A cyclic range batch is deterministic under input permutation") {
    const std::vector<gf::RangeMeasurement> ordered = {
        range(5, 1, 2, 4.1, 0.25),
        range(5, 1, 3, 3.1, 0.25),
        range(5, 2, 3, 5.1, 0.25)};
    const std::vector<gf::RangeMeasurement> shuffled = {
        ordered[2], ordered[0], ordered[1]};

    gf::CentralizedEkfOracle first = ringOracle();
    gf::CentralizedEkfOracle second = ringOracle();
    first.update(ordered);
    second.update(shuffled);

    const gf::JointEstimateSnapshot a = first.snapshot();
    const gf::JointEstimateSnapshot b = second.snapshot();
    CHECK(a.mean.isApprox(b.mean, 1e-12));
    CHECK(a.covariance.isApprox(b.covariance, 1e-12));
}

TEST_CASE("Joseph updates preserve symmetric positive semidefinite covariance") {
    gf::CentralizedEkfOracle oracle = ringOracle();
    oracle.update({
        range(5, 1, 2, 4.1, 0.25),
        range(5, 1, 3, 3.1, 0.25),
        range(5, 2, 3, 5.1, 0.25),
        range(5, 1, 10, 3.2, 0.25)});

    const Eigen::MatrixXd covariance = oracle.snapshot().covariance;
    CHECK(covariance.isApprox(covariance.transpose(), 1e-12));
    CHECK(minimumEigenvalue(covariance) >= -1e-12);
}
