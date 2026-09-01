#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedCoverageTracker.hpp"
#include "grand_finale/Task10p11hLeaderCoveragePolicy.hpp"

namespace {

gf::FrontierCell gridCell(int x,int y) {
    return {x,y,{10.0*static_cast<double>(x)+5.0,
                 10.0*static_cast<double>(y)+5.0}};
}

gf::LeaderCoverageRequest tailRequest(
    const std::vector<gf::FrontierCell>& uncovered) {
    gf::LeaderCoverageRequest request;
    request.uncovered_cells=uncovered;
    request.leader_centroid_primary=true;
    request.leader_reachability_filter=true;
    for (int x=0;x<300;++x)
        for (int y=0;y<300;++y)
            request.domain_cells.push_back(gridCell(x,y));
    const Eigen::Vector2d leader7(1034.0045,1227.7089);
    const Eigen::Vector2d leader14(2851.3672,2614.1525);
    for (int id=1;id<=14;++id)
        request.agents.push_back({static_cast<gf::NodeId>(id),
            id<=7?leader7:leader14,Eigen::Vector2d::Zero()});
    return request;
}

void observeAllocatedRealTargets(
    gf::CertifiedCoverageTracker& tracker,
    const gf::LeaderCoverageResult& allocation) {
    for (const gf::NodeId leader:{gf::NodeId{7},gf::NodeId{14}}) {
        const auto& target=allocation.leader_targets.at(leader);
        if (target.x_index<0 || target.y_index<0) continue;
        const Point point(target.center.x(),target.center.y());
        tracker.observe(point,point,0.0,8.0);
    }
}

std::vector<gf::FrontierCell> stillUncovered(
    gf::CertifiedCoverageTracker& tracker,
    const std::vector<gf::FrontierCell>& cells) {
    std::vector<gf::FrontierCell> result;
    for (const auto& cell:cells)
        if (!tracker.truthCovered(cell.x_index,cell.y_index))
            result.push_back(cell);
    return result;
}

}  // namespace

TEST_CASE("Official v5 three-cell tail reallocates real IDs and reaches T100") {
    gf::CertifiedCoverageTracker tracker({0.0,3000.0},300,
        {0.0,3000.0},300);
    auto state=tracker.restartState();
    std::fill(state.truth.begin(),state.truth.end(),true);
    std::fill(state.certified.begin(),state.certified.end(),true);
    const std::vector<gf::FrontierCell> residual={
        gridCell(0,299),gridCell(298,299),gridCell(299,299)};
    for (const auto& cell:residual) {
        const int index=tracker.truthGrid().getIndex(
            cell.x_index,cell.y_index);
        state.truth.at(index)=false;
        state.certified.at(index)=false;
    }
    tracker.restoreRestartState(state);
    REQUIRE(tracker.truthCoveredCount()==89997);
    REQUIRE(tracker.certifiedCoveredCount()==89997);

    auto uncovered=residual;
    const auto first=gf::allocateLeaderCoverageTargets(
        tailRequest(uncovered));
    REQUIRE(first.valid);
    CHECK(first.leader_targets.at(7).id()=="0:299");
    CHECK(first.leader_targets.at(14).id()=="298:299");
    observeAllocatedRealTargets(tracker,first);
    uncovered=stillUncovered(tracker,residual);
    REQUIRE(uncovered.size()==1);
    CHECK(uncovered.front().id()=="299:299");

    const auto second=gf::allocateLeaderCoverageTargets(
        tailRequest(uncovered));
    REQUIRE(second.valid);
    CHECK(second.leader_targets.at(7).id()=="-7:-1");
    CHECK(second.leader_targets.at(14).id()=="299:299");
    observeAllocatedRealTargets(tracker,second);

    CHECK(tracker.truthCoveredCount()==90000);
    CHECK(tracker.certifiedCoveredCount()==90000);
    CHECK(tracker.truthFraction()==doctest::Approx(1.0));
    CHECK(tracker.certifiedFraction()==doctest::Approx(1.0));
    CHECK(tracker.reachedCertifiedT100());
    CHECK(tracker.certifiedSubsetOfTruth());
}
