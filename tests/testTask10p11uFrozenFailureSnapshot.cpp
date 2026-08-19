#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11uFrozenFailureSnapshot.hpp"

namespace {

nlohmann::json frozenSummary() {
    return {
        {"protocol", "task10p11t-online-pair-v1"},
        {"outcome", "failed"},
        {"reason",
         "pair_coordination_failed:dynamic_pair_responsibility_interval_empty"},
        {"simulated_time_s", 132.89999999999674},
        {"active_pair_base_id", "reference:2->4"},
        {"responsibility_cycles", 5},
        {"source", {
            {"parent_commit", std::string(40, '1')},
            {"parent_tree", std::string(40, '2')},
            {"cbf_commit", std::string(40, '3')},
            {"cbf_tree", std::string(40, '4')},
            {"binary_sha256", std::string(64, '5')}}},
        {"frozen_model_config_digest", "1217127288044531733"},
        {"topology_frozen", true},
        {"responsibility_trace", nlohmann::json::array({{
            {"decision_time_s", 132.79999999999674},
            {"applied_time_s", 132.89999999999674},
            {"pair_base_id", "reference:2->4"}}})},
        {"conflict_events", nlohmann::json::array({{
            {"time_s", 132.89999999999674},
            {"mobile_pair_base_ids", {"reference:2->4"}},
            {"pair_step_reason",
             "pair_coordination_failed:dynamic_pair_responsibility_interval_empty"}}})}
    };
}

}  // namespace

TEST_CASE("132.9 summary proves local transfer empty but cannot authorize 28D") {
    const auto result = gf::runTask10p11uFrozenFailureAudit(frozenSummary());

    CHECK(result.at("input_summary_valid") == true);
    CHECK(result.at("local_signed_transfer").at("determined") == true);
    CHECK(result.at("local_signed_transfer").at("feasible") == false);
    CHECK(result.at("snapshot_preflight").at("complete") == false);
    CHECK(result.at("current_full_pair_28d").at("performed") == false);
    CHECK(result.at("current_full_pair_28d").at("status") == "undetermined");
    CHECK(result.at("successor").at("performed") == false);
    CHECK(result.at("pair_2_4_component_4d").at("performed") == false);
    CHECK(result.at("reason") ==
          "frozen_132p9_snapshot_incomplete_for_full_28d");
    CHECK(result.at("trajectory_run_performed") == false);
    CHECK(result.at("production_controller_modified") == false);
}

TEST_CASE("wrong protocol or terminal time fails before scientific classification") {
    auto wrong_protocol = frozenSummary();
    wrong_protocol["protocol"] = "unexpected";
    CHECK_THROWS_AS(gf::runTask10p11uFrozenFailureAudit(wrong_protocol),
                    std::invalid_argument);

    auto wrong_time = frozenSummary();
    wrong_time["simulated_time_s"] = 132.8;
    CHECK_THROWS_AS(gf::runTask10p11uFrozenFailureAudit(wrong_time),
                    std::invalid_argument);
}
