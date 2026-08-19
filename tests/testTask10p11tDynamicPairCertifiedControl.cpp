#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/DynamicPairCertifiedControl.hpp"

namespace {

std::vector<gf::NodeId> mobileIds() {
    std::vector<gf::NodeId> ids;
    for (gf::NodeId owner = 1; owner <= 14; ++owner) ids.push_back(owner);
    return ids;
}

gf::CanonicalHardRow pairRow(
    gf::NodeId owner, gf::NodeId peer,
    const Eigen::Vector2d& coefficient) {
    gf::CanonicalHardRow row;
    row.id = "reference:2->4:owner:" + std::to_string(owner);
    row.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    row.owner = owner;
    row.peer = peer;
    row.control_coefficient = coefficient;
    row.constant = 0.5;
    row.responsibility = 0.5;
    row.coefficient_uncertainty_reserve = 0.5;
    return row;
}

gf::DynamicPairCertifiedControlRequest request() {
    gf::DynamicPairCertifiedControlRequest result;
    result.pair_base_id = "reference:2->4";
    result.first_owner = 2;
    result.second_owner = 4;
    result.transfer_interval_lower_mps2 = -0.5;
    result.transfer_interval_upper_mps2 = 0.5;
    result.transfer_mps2 = 0.25;
    for (gf::NodeId owner = 1; owner <= 14; ++owner)
        result.controls.emplace(owner, Eigen::Vector2d::Zero());
    return result;
}

}  // namespace

TEST_CASE("dynamic pair batch preserves every hard row and one-reserve full pair") {
    std::vector<gf::CanonicalHardRow> rows{
        pairRow(2, 4, {1.0, 0.0}),
        pairRow(4, 2, {-1.0, 0.0})};
    for (gf::NodeId owner = 1; owner <= 14; ++owner) {
        gf::CanonicalHardRow row;
        row.id = "local:" + std::to_string(owner);
        row.owner = owner;
        row.control_coefficient = {0.0, 1.0};
        row.constant = 1.0;
        rows.push_back(row);
    }

    const auto audit = gf::auditDynamicPairCertifiedControls(
        rows, mobileIds(), 4.0, 1e-8, request());

    REQUIRE(audit.valid);
    CHECK(audit.reason == "dynamic_pair_control_batch_certified");
    CHECK(audit.minimum_local_residual_mps2 == doctest::Approx(0.25));
    CHECK(audit.dynamic_pair_local_sum_residual_mps2 ==
          doctest::Approx(1.0));
    CHECK(audit.once_reserve_full_pair_residual_mps2 ==
          doctest::Approx(1.5));
    CHECK(audit.limiting_owner == 4);
}

TEST_CASE("stale interval and any unrelated negative row fail closed") {
    std::vector<gf::CanonicalHardRow> rows{
        pairRow(2, 4, {1.0, 0.0}),
        pairRow(4, 2, {-1.0, 0.0})};
    auto stale = request();
    stale.transfer_interval_upper_mps2 = 0.2;
    CHECK_FALSE(gf::auditDynamicPairCertifiedControls(
        rows, mobileIds(), 4.0, 1e-8, stale).valid);

    gf::CanonicalHardRow unrelated;
    unrelated.id = "unrelated:7";
    unrelated.owner = 7;
    unrelated.constant = -0.1;
    rows.push_back(unrelated);
    const auto rejected = gf::auditDynamicPairCertifiedControls(
        rows, mobileIds(), 4.0, 1e-8, request());
    CHECK_FALSE(rejected.valid);
    CHECK(rejected.reason == "dynamic_pair_control_residual_negative");
    CHECK(rejected.limiting_owner == 7);
}
