#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11hSimpleCoveragePolicy.hpp"

namespace {

gf::FrontierCell cell(int x,double px) {
    return {x,0,{px,0.0}};
}

gf::SimpleCoveragePolicyRequest request() {
    gf::SimpleCoveragePolicyRequest value;
    value.agents={{1,{0.0,0.0},Eigen::Vector2d::Zero()},
                  {2,{10.0,0.0},Eigen::Vector2d::Zero()}};
    value.uncovered_cells={cell(1,1.0),cell(5,5.0),cell(9,9.0)};
    value.domain_cells={cell(0,0.0),cell(2,2.0),cell(8,8.0),cell(10,10.0)};
    return value;
}

}

TEST_CASE("Estimator Voronoi ownership and nearest targets use deterministic ties") {
    const auto result=gf::allocateSimpleCoverageTargets(request(),{});
    REQUIRE(result.valid);
    CHECK(result.voronoi_owner.at("1:0")==1);
    CHECK(result.voronoi_owner.at("5:0")==1);
    CHECK(result.voronoi_owner.at("9:0")==2);
    CHECK(result.targets.at(1).id()=="1:0");
    CHECK(result.targets.at(2).id()=="9:0");
    CHECK(result.targets.at(1).id()!=result.targets.at(2).id());
}

TEST_CASE("Empty local uncovered partition falls back to full-domain CVT centroid") {
    auto value=request();
    value.uncovered_cells={cell(1,1.0),cell(2,2.0),cell(3,3.0)};
    const auto result=gf::allocateSimpleCoverageTargets(value,{});
    REQUIRE(result.valid);
    CHECK(result.targets.size()==2);
    CHECK(result.targets.at(1).id()=="1:0");
    CHECK(result.targets.at(2).id()=="-2:-1");
    CHECK(result.targets.at(2).center.x()==doctest::Approx(9.0));
    CHECK(result.targets.at(2).center.y()==doctest::Approx(0.0));
    CHECK(result.cvt_fallback_owners==std::set<gf::NodeId>{2});
}

TEST_CASE("T100 request gives every owner a full-domain CVT centroid") {
    auto value=request();
    value.uncovered_cells.clear();
    const auto result=gf::allocateSimpleCoverageTargets(value,{});
    REQUIRE(result.valid);
    CHECK(result.reason=="full_domain_cvt");
    CHECK(result.targets.at(1).id()=="-1:-1");
    CHECK(result.targets.at(1).center.x()==doctest::Approx(1.0));
    CHECK(result.targets.at(2).id()=="-2:-1");
    CHECK(result.targets.at(2).center.x()==doctest::Approx(9.0));
    CHECK(result.cvt_fallback_owners==std::set<gf::NodeId>{1,2});
}

TEST_CASE("Finite age promotion prevents repeated nearest-cell preference without deleting cells") {
    auto value=request();
    value.uncovered_cells={cell(1,1.0),cell(4,4.0),cell(9,9.0)};
    value.fairness_ages={{"1:0",0},{"4:0",3},{"9:0",0}};
    const auto result=gf::allocateSimpleCoverageTargets(value,
        {10,3,3,1e-12});
    REQUIRE(result.valid);
    CHECK(result.targets.at(1).id()=="4:0");
    CHECK(result.uncovered_input_count==3);
    CHECK(result.remaining_unassigned_count==1);
}

TEST_CASE("Input order and topology strategy labels cannot change simple allocation") {
    auto first=request();
    auto second=first;
    std::reverse(second.agents.begin(),second.agents.end());
    std::reverse(second.uncovered_cells.begin(),second.uncovered_cells.end());
    const auto a=gf::allocateSimpleCoverageTargets(first,{});
    const auto b=gf::allocateSimpleCoverageTargets(second,{});
    REQUIRE(a.valid);
    REQUIRE(b.valid);
    CHECK(a.targets==b.targets);
    CHECK(a.request_digest==b.request_digest);
}

TEST_CASE("Fairness ledger resets only served cells and never removes denominator IDs") {
    gf::SimpleCoverageFairnessLedger ledger;
    const std::vector<gf::FrontierCell> denominator={cell(1,1.0),cell(2,2.0)};
    ledger.advanceUncovered(denominator);
    ledger.advanceUncovered(denominator);
    CHECK(ledger.ages().at("1:0")==2);
    CHECK(ledger.ages().at("2:0")==2);
    ledger.recordServed("1:0");
    CHECK(ledger.ages().at("1:0")==0);
    CHECK(ledger.ages().at("2:0")==2);
    CHECK(ledger.denominatorIds().size()==2);
}
