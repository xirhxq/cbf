#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GurobiTopologySolver.hpp"
#include "grand_finale/HighsTopologySolver.hpp"
#include "grand_finale/TargetLiftProposalPrototype.hpp"
#include "grand_finale/TopologyParityDiagnostic.hpp"

namespace {

gf::TopologyRequest request4p2() {
    return {{1,2,3,4},{100,101},
        {{100,1},{101,1},{100,2},{1,2},{3,2},
         {101,3},{1,3},{2,3},{100,4},{2,4},{3,4}},
        {{100,1},{101,1},{100,2},{1,2},
         {101,3},{1,3},{100,4},{2,4}},
        2,2,
        {{"100->1",1},{"101->1",1},{"100->2",1},{"1->2",2},
         {"3->2",8},{"101->3",1},{"1->3",2},{"2->3",8},
         {"100->4",1},{"2->4",3},{"3->4",4}}, {},
        {{"100->1|101->1",2},{"100->2|1->2",2},
         {"101->3|1->3",2},{"100->4|3->4",3}}, {}};
}

gf::TargetLiftProposalRequest proposalRequest() {
    gf::TargetLiftProposalRequest request;
    request.topology=request4p2();
    request.transition.mobile_ids={1,2,3,4};
    request.transition.fixed_ids={100,101};
    request.transition.old_edges=request.topology.old_edges;
    request.transition.fixed_targets={{100,{-100.0,0.0}},{101,{100.0,0.0}}};
    request.transition.raw_targets={{1,{0.0,500.0}},{2,{300.0,600.0}},
        {3,{-300.0,600.0}},{4,{500.0,700.0}}};
    request.transition.add_distance_m=849.0;
    request.transition.target_margin_m=1.0;
    request.transition.topology_version=3;
    request.transition.estimator_version=7;
    request.transition.raw_ledger_version=2;
    request.transition.config_version=1;
    request.geometry.position_support_m={{1,0.1},{2,0.1},{3,0.1},{4,0.1},
        {100,0.0},{101,0.0}};
    request.geometry.posterior_valid={{1,true},{2,true},{3,true},{4,true}};
    for (const auto& edge : request.topology.eligible_edges) {
        request.geometry.edge_information_valid[edge.id()]=true;
        request.geometry.range_variances_m2[
            gf::UndirectedEdge::canonical(edge.owner,edge.reference).id()]=100.0;
    }
    request.geometry.minimum_collision_distance_m=0.0;
    request.geometry.minimum_target_fim_planning_score=0.0;
    request.geometry.minimum_target_cone_planning_score=0.0;
    request.geometry.maximum_target_deformation_m=1e6;
    request.maximum_attempts=16;
    return request;
}

}

TEST_CASE("Union target-gate rejection adds request-local no-good and reproposes") {
    gf::GurobiTopologySolver solver;
    auto request=proposalRequest();
    const auto first=solver.solve(gf::TopologyModel(request.topology));
    REQUIRE(first.status==gf::TopologySolveStatus::Optimal);
    auto second_request=request.topology;
    second_request.forbidden_topologies.push_back(first.edges);
    const auto second=solver.solve(gf::TopologyModel(second_request));
    REQUIRE(second.status==gf::TopologySolveStatus::Optimal);
    const auto distinguishing=std::find_if(first.edges.begin(),first.edges.end(),
        [&](const auto& edge) {
            const bool already_old=std::any_of(
                request.topology.old_edges.begin(),request.topology.old_edges.end(),
                [&](const auto& old) { return old.id()==edge.id(); });
            return !already_old && std::none_of(second.edges.begin(),second.edges.end(),
                [&](const auto& candidate) { return candidate.id()==edge.id(); });
        });
    REQUIRE(distinguishing!=first.edges.end());
    request.geometry.edge_information_valid[distinguishing->id()]=false;
    const auto result=gf::proposeTargetGeometryCandidate(request,solver);
    std::string rejection_log;
    for (std::size_t index=0;index<result.rejection_reasons.size();++index) {
        rejection_log+=result.rejection_reasons[index]+":";
        for (const auto& edge : result.request_local_no_goods[index])
            rejection_log+=edge.id()+",";
        rejection_log+=';';
    }
    INFO("reason=",result.reason," rejections=",result.no_good_rejections,
         " log=",rejection_log);
    REQUIRE(result.target_geometry_candidate);
    CHECK(result.no_good_rejections>=1);
    CHECK(result.request_local_no_goods.front()==first.edges);
    CHECK(result.selected_topology!=first.edges);
    CHECK(result.reason=="target_geometry_candidate");
    CHECK(result.frozen_union.union_edges.size()>=result.selected_topology.size());
}

TEST_CASE("Gurobi HiGHS and exhaustive proposer share frozen optimum") {
    const auto request=proposalRequest();
    const gf::TopologyModel model(request.topology);
    gf::GurobiTopologySolver gurobi;
    gf::HighsTopologySolver highs;
    const auto gurobi_solution=gurobi.solve(model);
    const auto highs_solution=highs.solve(model);
    const auto parity=gf::compareTopologyParity(
        model,gurobi_solution,highs_solution,1e-8);
    REQUIRE(parity.exhaustive_feasible);
    CHECK(parity.gurobi_matches);
    CHECK(parity.highs_matches);
    CHECK(parity.selected_edge_tie_matches);
    const auto g=gf::proposeTargetGeometryCandidate(request,gurobi);
    const auto h=gf::proposeTargetGeometryCandidate(request,highs);
    REQUIRE(g.target_geometry_candidate);
    REQUIRE(h.target_geometry_candidate);
    CHECK(g.selected_topology==h.selected_topology);
    CHECK(g.frozen_union.lifted_digest==h.frozen_union.lifted_digest);
}

TEST_CASE("Identical raw ledger and graph ignore topology-strategy label") {
    const auto solution=gf::exhaustiveTopologyOracle(
        gf::TopologyModel(request4p2()),1e-8);
    REQUIRE(solution.feasible);
    std::uint64_t digest=0;
    for (const std::string strategy : {"fixed","greedy","proposed"}) {
        CAPTURE(strategy);
        auto request=proposalRequest().transition;
        request.successor_edges=solution.edges;
        const auto frozen=gf::freezeUnionTargetLift(request);
        REQUIRE(frozen.valid);
        if (digest==0) digest=frozen.lifted_digest;
        CHECK(frozen.lifted_digest==digest);
    }
}
