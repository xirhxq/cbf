#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11zComponentOracle.hpp"

TEST_CASE("133.0 certificate gate finds cardinality-minimal connected components") {
    const auto checkpoint=std::filesystem::path(PROJECT_ROOT).parent_path()/
        "docs/evidence/fixed-baseline-recovery-campaign/stage-b/"
        "pair-2-4-component/checkpoints/"
        "checkpoint-0001-t133.0-fail_closed.json";
    const auto gate=gf::runTask10p11zComponentGate(checkpoint,13);
    INFO(gate.reason);
    REQUIRE(gate.valid);
    CHECK(gate.snapshot_conditioning=="two_applied_4d_prefix_cycles");
    CHECK_FALSE(gate.pair_2_4_current_feasible);
    CHECK(gate.full_28d_current_feasible);
    REQUIRE(gate.current_minimum_component.size()>=2);
    CHECK(gate.current_minimum_component.count(2)==1);
    CHECK(gate.current_minimum_component.count(4)==1);
    CHECK(gate.current_minimum_component.size()<14);
    REQUIRE(gate.successor_minimum_component.size()>=
        gate.current_minimum_component.size());
    CHECK(gate.successor_minimum_component.count(2)==1);
    CHECK(gate.successor_minimum_component.count(4)==1);
    CHECK(gate.successor_minimum_component.size()<14);
    CHECK(gate.current_selected.row_count==1113);
    CHECK(gate.current_selected.minimum_residual_mps2>=-1.0e-8);
    CHECK(gate.successor_selected.current_row_count==1113);
    CHECK(gate.successor_selected.successor_row_count==1113);
    CHECK(gate.successor_selected.current_minimum_residual_mps2>=-1.0e-8);
    CHECK(gate.successor_selected.successor_minimum_residual_mps2>=-1.0e-8);
    for (const auto& attempt:gate.attempts) {
        if (attempt.component.size()<gate.current_minimum_component.size()) {
            CHECK_FALSE(attempt.current_feasible);
            CHECK(attempt.infeasibility_certificate.valid);
            CHECK(attempt.infeasibility_certificate.contradiction< -1.0e-9);
            CHECK(attempt.infeasibility_certificate.stationarity_max_abs<=1.0e-8);
            CHECK_FALSE(attempt.iis_rows.empty());
        }
    }
    const auto encoded=gf::task10p11zComponentGateJson(gate);
    CHECK_NOTHROW(gf::validateTask10p11vFiniteJson(encoded));
}
