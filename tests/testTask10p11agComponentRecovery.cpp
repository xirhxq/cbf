#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task10p11agComponentRecovery.hpp"

#include <filesystem>

TEST_CASE("Task 10.11ag 157.9 component gate starts from collision 2-9") {
    const auto path=std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/
        "evidence"/"task10p11ag-full-domain-predecessor-recovery"/"gate2"/
        "derived-from-sparse-157.9.json";
    const auto snapshot=gf::readTask10p11vJson(path);
    const auto controls=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
    const auto gate=gf::runTask10p11agComponentGate(snapshot,controls,2);
    REQUIRE(gate.valid);
    CHECK(gate.selected_component==std::set<gf::NodeId>{2,9});
    CHECK(gate.frozen_maximum_component_size==2);
    REQUIRE(gate.attempts.size()==1);
    CHECK(gate.attempts.front().current_row_count==1113);
    CHECK(gate.attempts.front().successor_row_count==1113);
    CHECK(gate.attempts.front().current_feasible);
    CHECK(gate.attempts.front().successor_feasible);
}
