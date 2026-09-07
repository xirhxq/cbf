#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"

TEST_CASE("A scene may retain H0 references while a paired DAG selects six physical anchors") {
    auto h0=gf::task25DagContractFromCode(0);
    auto goal=gf::task25DagContractFromCode(12);
    const auto original=h0.reference_edges;
    const std::vector<gf::NodeId> anchors{100,101,102,103,104,105};
    h0.fixed_anchor_ids=anchors;goal.fixed_anchor_ids=anchors;
    for(auto& e:goal.reference_edges)if(e.reference==100)e.reference=103;
        else if(e.reference==101)e.reference=104;else if(e.reference==102)e.reference=105;
    gf::task20_lattice_detail::finish(h0);gf::task20_lattice_detail::finish(goal);
    REQUIRE(h0.valid);REQUIRE(goal.valid);
    CHECK(h0.reference_edges==original);CHECK(h0.topological_order.size()==20);
    const auto plan=gf::task26ReplacementPlan(h0.reference_edges,goal.reference_edges,anchors);
    REQUIRE(plan.valid);CHECK_FALSE(plan.replacements.empty());
    CHECK_FALSE(gf::task26ReplacementPlan(h0.reference_edges,goal.reference_edges).valid);
    goal.fixed_anchor_ids.pop_back();gf::task20_lattice_detail::finish(goal);CHECK_FALSE(goal.valid);
    h0.fixed_anchor_ids.push_back(1);gf::task20_lattice_detail::finish(h0);CHECK_FALSE(h0.valid);
    auto legacy=gf::task25DagContractFromCode(0);REQUIRE(legacy.valid);CHECK(legacy.topological_order.size()==17);
}
