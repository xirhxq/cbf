#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/ReferenceEligibility.hpp"
#include "grand_finale/TransitionCertifier.hpp"

TEST_CASE("An edge in the keep-only shell cannot support a reverse certificate") {
    Eigen::VectorXd mean(4);
    mean << 849.5, 0.0, 0.0, 0.0;
    const gf::JointEstimateSnapshot snapshot{
        {1}, mean, 1e-4 * Eigen::MatrixXd::Identity(4, 4),
        {{10, Eigen::Vector2d::Zero()}}};
    const std::map<std::string, gf::RangeLinkState> links{
        {"1--10", {0.0, 1.0}}};
    const gf::EligibilityThresholds thresholds{
        849.0, 850.0, 1.0, 0.1, 0.5, 0.0};
    const auto keep = gf::buildEligibility(
        snapshot, links, thresholds, {{10, 1}});
    const auto add = gf::buildEligibility(snapshot, links, thresholds, {});
    REQUIRE(keep.candidates.size() == 1);
    REQUIRE(add.candidates.size() == 1);
    CHECK(keep.candidates.front().eligible);
    CHECK_FALSE(add.candidates.front().eligible);
    CHECK(add.candidates.front().reason == "robust_distance");
}

TEST_CASE("Non-finite eligibility thresholds cannot bypass a hard gate") {
    Eigen::VectorXd mean(4);
    mean.setZero();
    const gf::JointEstimateSnapshot snapshot{
        {1}, mean, Eigen::MatrixXd::Identity(4, 4),
        {{10, Eigen::Vector2d::Ones()}}};
    const std::map<std::string, gf::RangeLinkState> links{
        {"1--10", {0.0, 1.0}}};
    auto thresholds = gf::EligibilityThresholds{1.0, 2.0, 1.0, 2.0, 0.5, 1.0};
    thresholds.max_aoi_s = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(
        gf::buildEligibility(snapshot, links, thresholds, {}),
        std::invalid_argument);
}

#include <Eigen/Dense>

#include <map>
#include <string>
#include <vector>

namespace {

gf::JointEstimateSnapshot eligibilitySnapshot(double second_x = 790.0) {
    Eigen::VectorXd mean(12);
    mean << 0.0, 0.0, 0.0, 0.0,
            second_x, 0.0, 0.0, 0.0,
            900.0, 0.0, 0.0, 0.0;
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(12, 12);
    covariance.diagonal() << 1.0, 1.0, 0.1, 0.1,
                             1.0, 1.0, 0.1, 0.1,
                             1.0, 1.0, 0.1, 0.1;
    return gf::JointEstimateSnapshot{
        {1, 2, 3}, mean, covariance,
        {{10, Eigen::Vector2d{0.0, 790.0}}}};
}

std::map<std::string, gf::RangeLinkState> freshLinks() {
    return {
        {"1--2", {0.1, 1.0}}, {"1--3", {0.1, 1.0}},
        {"1--10", {0.1, 1.0}}, {"2--3", {0.1, 1.0}},
        {"2--10", {0.1, 1.0}}, {"3--10", {0.1, 1.0}}};
}

gf::EligibilityThresholds thresholds() {
    return gf::EligibilityThresholds{
        800.0, 850.0, 1.0, 4.0, 0.5, 1.0};
}

const gf::ReferenceCandidate& candidate(
    const gf::EligibilitySnapshot& snapshot,
    const std::string& edge_id) {
    for (const gf::ReferenceCandidate& value : snapshot.candidates) {
        if (value.edge.id() == edge_id) {
            return value;
        }
    }
    throw std::runtime_error("candidate not found");
}

}  // namespace

TEST_CASE("Eligibility scans every ordered reference-owner pair without KNN pruning") {
    const gf::EligibilitySnapshot result = gf::buildEligibility(
        eligibilitySnapshot(), freshLinks(), thresholds(), {});

    CHECK(result.candidates.size() == 9);
    CHECK(candidate(result, "2->1").eligible);
    CHECK(candidate(result, "10->1").eligible);
    CHECK_FALSE(candidate(result, "3->1").eligible);
    CHECK(candidate(result, "3->1").reason == "robust_distance");
}

TEST_CASE("Existing references use the keep distance while new references use add distance") {
    const gf::JointEstimateSnapshot estimate = eligibilitySnapshot(820.0);
    const gf::EligibilitySnapshot add_result = gf::buildEligibility(
        estimate, freshLinks(), thresholds(), {});
    const gf::EligibilitySnapshot keep_result = gf::buildEligibility(
        estimate, freshLinks(), thresholds(), {gf::DirectedEdge{2, 1}});

    CHECK_FALSE(candidate(add_result, "2->1").eligible);
    CHECK(candidate(keep_result, "2->1").eligible);
}

TEST_CASE("Stale low-quality and uncertain references are rejected independently") {
    std::map<std::string, gf::RangeLinkState> links = freshLinks();
    links["1--2"].age_s = 2.0;
    links["1--10"].quality = 0.1;
    gf::JointEstimateSnapshot estimate = eligibilitySnapshot();
    estimate.covariance(8, 8) = 9.0;
    const gf::EligibilitySnapshot result = gf::buildEligibility(
        estimate, links, thresholds(), {});

    CHECK(candidate(result, "2->1").reason == "range_aoi");
    CHECK(candidate(result, "10->1").reason == "range_quality");
    CHECK(candidate(result, "3->1").reason == "reference_covariance");
}
