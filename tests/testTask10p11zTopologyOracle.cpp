#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11zTopologyOracle.hpp"

TEST_CASE("earlier topology gate audits temporal old union new chain without advancing") {
    const auto sparse=std::filesystem::path(PROJECT_ROOT).parent_path()/
        "docs/evidence/task10p11v-unique-recapture/checkpoints/"
        "sparse-1300-t130.0.json";
    const auto gate=gf::runTask10p11zEarlyTopologyGate(sparse);
    INFO(gate.reason);
    INFO(gate.old_phase.performed,gate.old_phase.current_feasible,
        gate.old_phase.successor_feasible);
    INFO(gate.union_phase.performed,gate.union_phase.current_feasible,
        gate.union_phase.successor_feasible);
    INFO(gate.new_phase.performed,gate.new_phase.current_feasible,
        gate.new_phase.successor_feasible);
    REQUIRE(gate.valid);
    CHECK(gate.snapshot_time_s==doctest::Approx(130.0));
    CHECK(gate.rule_addition=="1->4");
    CHECK(gate.rule_removal=="2->4");
    CHECK(gate.old_phase.performed);
    CHECK(gate.union_phase.performed);
    CHECK(gate.new_phase.performed);
    CHECK(gate.old_phase.current_row_count==1113);
    CHECK(gate.union_phase.current_row_count==1114);
    CHECK(gate.new_phase.current_row_count==1113);
    CHECK_FALSE(gate.fresh_post_union_information_epoch_available);
    CHECK_FALSE(gate.temporal_chain_reached_new);
    CHECK_FALSE(gate.offline_gate_passed);
    CHECK(gate.r4_status=="offline_rejected");
    CHECK(gate.reason=="union_current_infeasible");
    CHECK_FALSE(gate.trajectory_advanced);
    const auto encoded=gf::task10p11zTopologyGateJson(gate);
    CHECK_NOTHROW(gf::validateTask10p11vFiniteJson(encoded));
}
