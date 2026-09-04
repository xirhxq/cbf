#pragma once

#include "grand_finale/Task20CoveragePolicy.hpp"
#include "grand_finale/Task22FootprintInsetSweep.hpp"

namespace gf {

enum class Task25DagMode {
    H0Origin=0,
    H0Microfix=10,
    CrossBracedDualFront=11,
    Pinball5432=12,
    LongTriangleSingleLadder=13,
    SplitThreeFront=2
};

namespace task25_detail {

inline std::set<std::string> edgeSet(
    const std::vector<DirectedEdge>& edges) {
    std::set<std::string> result;
    for (const auto& edge:edges) result.insert(edge.id());
    return result;
}

inline std::vector<DirectedEdge> microfixEdges() {
    auto result=task20_lattice_detail::dualLadderEdges();
    for (auto& edge:result) {
        if (edge==DirectedEdge{101,2}) edge=DirectedEdge{100,2};
        else if (edge==DirectedEdge{101,9}) edge=DirectedEdge{102,9};
    }
    return result;
}

inline Task20DagLatticeContract h0Rewired(
    const std::string& id,const std::string& signature,
    const std::vector<DirectedEdge>& edges) {
    auto result=task20DagLatticeContract(Task20LatticeMode::DualLadder);
    result.id=id;
    result.structural_signature=signature;
    result.reference_edges=edges;
    result.topological_order.clear();
    result.reason.clear();
    result.valid=false;
    task20_lattice_detail::finish(result);
    return result;
}

inline Task20DagLatticeContract longTriangle() {
    Task20DagLatticeContract result;
    result.id="long-triangle-single-ladder";
    result.structural_signature=
        "units=14;rows=2-1x12;single-long-triangle";
    result.reference_edges={{100,1},{101,1},{101,2},{102,2},
        {1,3},{2,3}};
    for (NodeId owner=4;owner<=14;++owner) {
        result.reference_edges.emplace_back(owner-2,owner);
        result.reference_edges.emplace_back(owner-1,owner);
    }
    result.coverage_units={{"T",task10p10MobileIds(14),
        {100,101,102},14,{13,14}}};
    std::vector<std::pair<double,double>> roles;
    roles.reserve(14);
    // p = base + axial*v +/- sqrt(3)/(2*13) R90(v).  The generic
    // a*I+t*R60 representation is exact for t=1/13 and
    // a=axial-t/2, so no special runtime lifting branch is required.
    for (NodeId member=1;member<=14;++member) {
        const double axial=member<=2?1.0/13.0:
            static_cast<double>(member-1)/13.0;
        const double triangular=(member%2==0?-1.0:1.0)/13.0;
        roles.emplace_back(axial-0.5/13.0,triangular);
    }
    task20_lattice_detail::addRoles(result,result.coverage_units.front(),roles);
    task20_lattice_detail::finish(result);
    return result;
}

}  // namespace task25_detail

inline Task20DagLatticeContract task25DagContract(Task25DagMode mode) {
    if (mode==Task25DagMode::H0Origin)
        return task20DagLatticeContract(Task20LatticeMode::DualLadder);
    if (mode==Task25DagMode::H0Microfix)
        return task25_detail::h0Rewired("h0-microfix",
            "units=7+7;roles=h0;edges=microfix",
            task25_detail::microfixEdges());
    if (mode==Task25DagMode::CrossBracedDualFront) {
        auto edges=task20_lattice_detail::dualLadderEdges();
        for (auto& edge:edges) {
            // Same H0 depths and member roles.  The two depth-one roots
            // provide symmetric cross-squad references to depth two.
            if (edge==DirectedEdge{101,2}) edge=DirectedEdge{8,2};
            else if (edge==DirectedEdge{101,9}) edge=DirectedEdge{1,9};
        }
        return task25_detail::h0Rewired("cross-braced-dual-front",
            "units=7+7;roles=h0;edges=symmetric-depth1-cross-brace",edges);
    }
    if (mode==Task25DagMode::Pinball5432)
        return task22Pinball5432Contract();
    if (mode==Task25DagMode::LongTriangleSingleLadder)
        return task25_detail::longTriangle();
    if (mode==Task25DagMode::SplitThreeFront)
        return task20DagLatticeContract(Task20LatticeMode::SplitThreeFront);
    Task20DagLatticeContract invalid;
    invalid.reason="unknown_task25_dag_mode";
    return invalid;
}

inline Task20DagLatticeContract task25DagContractFromCode(int code) {
    if (code==0) return task25DagContract(Task25DagMode::H0Origin);
    if (code==2) return task25DagContract(Task25DagMode::SplitThreeFront);
    if (code==10) return task25DagContract(Task25DagMode::H0Microfix);
    if (code==11)
        return task25DagContract(Task25DagMode::CrossBracedDualFront);
    if (code==12) return task25DagContract(Task25DagMode::Pinball5432);
    if (code==13)
        return task25DagContract(Task25DagMode::LongTriangleSingleLadder);
    Task20DagLatticeContract invalid;
    invalid.reason="unknown_task25_dag_code";
    return invalid;
}

inline bool task25CoverageRoleEquivalent(
    const Task20DagLatticeContract& first,
    const Task20DagLatticeContract& second) {
    if (!first.valid||!second.valid||
        first.coverage_units.size()!=second.coverage_units.size()||
        first.member_roles.size()!=second.member_roles.size()) return false;
    for (std::size_t index=0;index<first.coverage_units.size();++index) {
        const auto& a=first.coverage_units[index];
        const auto& b=second.coverage_units[index];
        if (a.id!=b.id||a.members!=b.members||a.base_anchors!=b.base_anchors||
            a.leader!=b.leader||a.front_members!=b.front_members) return false;
    }
    for (const auto& [member,a]:first.member_roles) {
        const auto found=second.member_roles.find(member);
        if (found==second.member_roles.end()) return false;
        const auto& b=found->second;
        if (a.coverage_unit!=b.coverage_unit||
            std::abs(a.axial_fraction-b.axial_fraction)>1.0e-12||
            std::abs(a.triangular_fraction-b.triangular_fraction)>1.0e-12)
            return false;
    }
    return true;
}

inline std::size_t task25DagReplacementCount(
    const Task20DagLatticeContract& first,
    const Task20DagLatticeContract& second) {
    const auto a=task25_detail::edgeSet(first.reference_edges);
    const auto b=task25_detail::edgeSet(second.reference_edges);
    return static_cast<std::size_t>(std::count_if(a.begin(),a.end(),
        [&](const auto& id) { return b.count(id)==0; }));
}

}  // namespace gf
