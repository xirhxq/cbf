#pragma once

#include "grand_finale/Task10p11gReadinessFixture.hpp"

namespace gf {

inline std::map<NodeId,Eigen::Vector2d> task10p11hCoastalAnchors() {
    return {{100,{100.0,-50.0}},
            {101,{250.0,-50.0}},
            {102,{400.0,-50.0}}};
}

inline std::vector<Eigen::Vector2d> task10p11hCoastal14LaunchBand() {
    return {{70.0,50.0},{190.0,50.0},{310.0,50.0},{430.0,50.0},
            {70.0,100.0},{190.0,100.0},{310.0,100.0},{430.0,100.0},
            {100.0,150.0},{220.0,150.0},{340.0,150.0},{420.0,150.0},
            {170.0,200.0},{330.0,200.0}};
}

inline std::vector<DirectedEdge> task10p11hCoastalFixedTopology14() {
    return {{100,1},{101,1},{100,2},{101,2},
            {101,3},{102,3},{101,4},{102,4},
            {100,5},{101,5},{100,6},{101,6},
            {101,7},{102,7},{101,8},{102,8},
            {100,9},{101,9},{100,10},{101,10},
            {101,11},{102,11},{101,12},{102,12},
            {100,13},{101,13},{101,14},{102,14}};
}

inline std::vector<DirectedEdge> task10p11hCoastalRelayDag14() {
    return {{100,1},{101,1},{100,2},{101,2},
            {101,3},{102,3},{101,4},{102,4},
            {1,5},{2,5},{1,6},{2,6},{3,7},{4,7},{3,8},{4,8},
            {5,9},{6,9},{6,10},{7,10},{7,11},{8,11},{7,12},{8,12},
            {9,13},{10,13},{11,14},{12,14}};
}

inline Task10p10Scenario task10p11hCoastalEasyScenario() {
    return {"coastal_easy",400.0,600.0,{1,2,3,4},
        {{100.0,60.0},{200.0,60.0},{300.0,60.0},{380.0,60.0}},
        task10p11hCoastalAnchors(),
        {{100,1},{101,1},{100,2},{101,2},
         {101,3},{102,3},{101,4},{102,4}}};
}

inline Task10p10Scenario task10p11hCoastalLeaderEasyScenario() {
    return {"coastal_leader_easy",400.0,600.0,task10p10MobileIds(14),
        {{50.0,50.0},{150.0,50.0},{250.0,50.0},{350.0,50.0},
         {70.0,100.0},{170.0,100.0},{270.0,100.0},{330.0,100.0},
         {80.0,150.0},{180.0,150.0},{280.0,150.0},{340.0,150.0},
         {140.0,200.0},{260.0,200.0}},
        task10p11hCoastalAnchors(),task10p11hCoastalFixedTopology14()};
}

inline Task10p10Scenario task10p11hCoastalNonbindingScenario() {
    return {"coastal_nonbinding",500.0,700.0,task10p10MobileIds(14),
        task10p11hCoastal14LaunchBand(),task10p11hCoastalAnchors(),
        task10p11hCoastalFixedTopology14()};
}

inline Task10p10Scenario task10p11hCoastalBindingScenario() {
    return {"coastal_binding",500.0,1800.0,task10p10MobileIds(14),
        task10p11hCoastal14LaunchBand(),task10p11hCoastalAnchors(),
        task10p11hCoastalRelayDag14()};
}

inline Task10p10Scenario task10p11hCoastalBindingActiveScenario() {
    return {"coastal_binding_reference_active_mechanism",500.0,1800.0,
        task10p10MobileIds(14),
        {{100.0,50.0},{250.0,50.0},{400.0,50.0},{450.0,50.0},
         {120.0,250.0},{250.0,250.0},{380.0,250.0},{470.0,250.0},
         {150.0,500.0},{280.0,780.0},{150.0,700.0},{350.0,700.0},
         {100.0,830.0},{300.0,1500.0}},
        task10p11hCoastalAnchors(),task10p11hCoastalRelayDag14()};
}

inline std::vector<DirectedEdge> task10p11hCoastalRelayCandidates(
    const GrandFinaleSwarmAdapter& adapter) {
    const auto runtime=adapter.runtimeSnapshot();
    std::map<std::string,RangeLinkState> links;
    for (const auto& [id,link] : runtime.range_links)
        links[id]={link.age_s,link.quality};
    const auto gates=buildEligibility(runtime.estimate,links,
        {adapter.config().add_reference_distance_m,
         adapter.config().reference_distance_m,
         adapter.config().maximum_range_aoi_s,
         adapter.config().maximum_reference_position_eigenvalue_m2,
         adapter.config().minimum_range_quality,
         adapter.config().uncertainty_sigma},{});
    std::set<NodeId> current;
    for (const auto& edge : runtime.topology)
        if (edge.owner==14) current.insert(edge.reference);
    std::vector<DirectedEdge> result;
    for (const auto& gate : gates.candidates)
        if (gate.edge.owner==14 && gate.eligible &&
            !current.count(gate.edge.reference))
            result.push_back(gate.edge);
    std::sort(result.begin(),result.end(),[](const auto& lhs,const auto& rhs) {
        return lhs.id()<rhs.id();
    });
    return result;
}

inline TopologyRequest task10p11hCoastalRelayProposal(
    const GrandFinaleSwarmAdapter& adapter) {
    const auto runtime=adapter.runtimeSnapshot();
    TopologyRequest request;
    request.mobile_ids=runtime.estimate.mobile_ids;
    for (const auto& [id,position] : runtime.estimate.fixed_positions) {
        (void)position;
        request.fixed_ids.push_back(id);
    }
    request.old_edges=runtime.topology;
    for (const auto& edge : runtime.topology)
        if (edge.id()!="11->14") request.eligible_edges.push_back(edge);
    const auto candidates=task10p11hCoastalRelayCandidates(adapter);
    request.eligible_edges.insert(request.eligible_edges.end(),
        candidates.begin(),candidates.end());
    request.min_indegree=2;
    request.max_indegree=2;
    for (const auto& edge : candidates)
        request.progress_coefficients[edge.id()]=
            edge.id()=="13->14"?1.0:0.0;
    return request;
}

}  // namespace gf
