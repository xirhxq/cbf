#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11wConflictComponentOracle.hpp"

#include <filesystem>

TEST_CASE("Task 10.11w audits exactly the six frozen intervention checkpoints") {
    const std::filesystem::path root =
        std::filesystem::path(PROJECT_ROOT).parent_path() /
        "docs/evidence/task10p11v-unique-recapture/checkpoints";
    const std::vector<std::filesystem::path> paths = {
        root / "checkpoint-000-t132.4-first_dynamic_intervention.json",
        root / "checkpoint-001-t132.5-dynamic_intervention.json",
        root / "checkpoint-002-t132.6-dynamic_intervention.json",
        root / "checkpoint-003-t132.7-dynamic_intervention.json",
        root / "checkpoint-004-t132.8-dynamic_intervention.json",
        root / "checkpoint-005-t132.9-fail_closed.json"};

    const auto result = gf::runTask10p11wOfflineOracle(paths);

    CHECK(result.at("protocol") == "task10p11w-offline-oracle-v1");
    CHECK(result.at("checkpoint_count") == 6);
    CHECK(result.at("trajectory_run_performed") == false);
    CHECK(result.at("production_controller_modified") == false);
    CHECK(result.at("fixed_topology_modified") == false);
    REQUIRE(result.at("frames").size() == 6);
    for (const auto& frame : result.at("frames")) {
        CHECK(frame.contains("current_local_signed_transfer"));
        CHECK(frame.at("full_28d_oracle").at("feasible") == true);
        CHECK(frame.at("component_search").at("pair_2_4_tested") == true);
        CHECK(frame.at("component_search").at("minimum_component").is_array());
        CHECK(frame.at("component_search").at("minimum_component_successor")
                  .at("performed") == true);
        REQUIRE(frame.at("preventive_homotopy").at("candidates").size() == 9);
        for (const auto& candidate :
             frame.at("preventive_homotopy").at("candidates")) {
            CHECK(candidate.at("alpha").get<double>() >= 0.0);
            CHECK(candidate.at("alpha").get<double>() <= 1.0);
            CHECK(candidate.at("successor").at("performed") == true);
            CHECK(candidate.at("successor").at("full_row_residual_recomputed") ==
                  true);
            CHECK(candidate.at("successor").contains("local_signed_transfer"));
            CHECK(candidate.at("successor").contains("component_feasibility"));
            CHECK(candidate.at("successor").contains("full_pair_feasibility"));
        }
    }
    CHECK(result.at("summary").contains("earliest_preventive_intervention_s"));
    CHECK(result.at("summary").contains("last_local_recovery_s"));
    CHECK(result.at("summary").contains(
        "fallback_restores_local_feasibility"));
    CHECK(result.at("summary").at("pair_2_4_4d_sufficient") == true);
    CHECK(result.at("summary").at("earliest_preventive_intervention_s") ==
          doctest::Approx(132.8));
    CHECK(result.at("summary").at("last_local_recovery_s") ==
          doctest::Approx(132.8));
    CHECK(result.at("frames").at(4).at("preventive_homotopy")
          .at("selection") == "least_coverage_deviation_restoring_local");
    CHECK(result.at("frames").at(4).at("preventive_homotopy")
          .at("selected_index") == 8);
    CHECK(result.at("frames").at(5).at("preventive_homotopy")
          .at("selection") ==
          "second_order_inspired_maximum_predicted_margin_fallback");
    CHECK(result.at("frames").at(5).at("preventive_homotopy")
          .at("selected_successor_feasible") == true);
    CHECK(result.at("summary").at("fallback_restores_local_feasibility") ==
          false);
    CHECK(result.at("claim_boundary").at("recursive_feasibility_claimed") ==
          false);
}
