#pragma once

#include "grand_finale/Task10p11pOperationalEnvelope.hpp"
#include "grand_finale/Task15ForwardCoveragePolicy.hpp"

#include <queue>
#include <set>

namespace gf {

enum class Task20LatticeMode {
    DualLadder,
    MergedStrip,
    SplitThreeFront,
    CrossBracedDiamond
};

struct Task20CoverageUnit {
    std::string id;
    std::vector<NodeId> members;
    std::vector<NodeId> base_anchors;
    NodeId leader=0;
    // Generic front-frame support.  Legacy contracts may leave this empty,
    // in which case the leader is the sole front member.
    std::vector<NodeId> front_members;
};

// Every role is an affine triangular map
//   p_i = b_u + (a_i I + s_i R_60) (g_u - b_u).
// A negative s_i is the deterministic mirror of a positive lattice slot.
struct Task20MemberRole {
    NodeId member=0;
    std::string coverage_unit;
    double axial_fraction=0.0;
    double triangular_fraction=0.0;
};

struct Task20DagLatticeContract {
    bool valid=false;
    std::string reason;
    std::string id;
    std::string structural_signature;
    std::vector<DirectedEdge> reference_edges;
    std::vector<Task20CoverageUnit> coverage_units;
    std::map<NodeId,Task20MemberRole> member_roles;
    std::vector<NodeId> topological_order;
};

struct Task20LiftResult {
    bool valid=false;
    std::string reason;
    std::map<NodeId,Eigen::Vector2d> targets;
};

struct Task20FrontInverseResult {
    bool valid=false;
    std::string reason;
    std::string coverage_unit;
    Eigen::Vector2d front=Eigen::Vector2d::Zero();
};

namespace task20_lattice_detail {

inline std::vector<DirectedEdge> dualLadderEdges() {
    return {{101,1},{100,1},{101,2},{1,2},{1,3},{2,3},{2,4},{3,4},
            {3,5},{4,5},{4,6},{5,6},{5,7},{6,7},
            {101,8},{102,8},{101,9},{8,9},{8,10},{9,10},
            {9,11},{10,11},{10,12},{11,12},{11,13},{12,13},
            {12,14},{13,14}};
}

inline Eigen::Vector2d rotate60(const Eigen::Vector2d& value,double sign) {
    constexpr double cosine=0.5;
    constexpr double sine=0.86602540378443864676;
    return {cosine*value.x()-sign*sine*value.y(),
            sign*sine*value.x()+cosine*value.y()};
}

inline std::vector<DirectedEdge> mergedStripEdges() {
    std::vector<DirectedEdge> result{{100,1},{101,1},{101,8},{102,8}};
    const std::array<std::array<NodeId,2>,7> rows{{
        {{1,8}},{{2,9}},{{3,10}},{{4,11}},{{5,12}},{{6,13}},{{7,14}}}};
    for (std::size_t row=1;row<rows.size();++row)
        for (NodeId owner:rows[row]) {
            result.emplace_back(rows[row-1][0],owner);
            result.emplace_back(rows[row-1][1],owner);
        }
    return result;
}

inline void appendSplitUnit(std::vector<DirectedEdge>& edges,
    const std::vector<NodeId>& members,NodeId left,NodeId right) {
    edges.emplace_back(left,members[0]);
    edges.emplace_back(right,members[0]);
    edges.emplace_back(right,members[1]);
    edges.emplace_back(members[0],members[1]);
    for (std::size_t index=2;index<members.size();++index) {
        edges.emplace_back(members[index-2],members[index]);
        edges.emplace_back(members[index-1],members[index]);
    }
}

inline std::vector<DirectedEdge> splitThreeFrontEdges() {
    std::vector<DirectedEdge> result;
    appendSplitUnit(result,{1,2,3,4,5},100,101);
    appendSplitUnit(result,{6,7,8,9,10},101,102);
    appendSplitUnit(result,{11,12,13,14},100,102);
    return result;
}

inline void appendDiamond(std::vector<DirectedEdge>& edges,
    const std::array<NodeId,7>& m,NodeId left,NodeId right) {
    edges.insert(edges.end(),{{left,m[0]},{right,m[0]},
        {left,m[1]},{m[0],m[1]},{right,m[2]},{m[0],m[2]},
        {m[1],m[3]},{m[2],m[3]},
        {m[1],m[4]},{m[3],m[4]},
        {m[2],m[5]},{m[3],m[5]},
        {m[4],m[6]},{m[5],m[6]}});
}

inline std::vector<DirectedEdge> diamondEdges() {
    std::vector<DirectedEdge> result;
    appendDiamond(result,{1,2,3,4,5,6,7},100,101);
    appendDiamond(result,{8,9,10,11,12,13,14},101,102);
    return result;
}

inline std::vector<NodeId> topologicalOrder(
    const std::vector<DirectedEdge>& edges,std::string& reason) {
    std::set<NodeId> nodes{100,101,102};
    for (NodeId owner=1;owner<=14;++owner) nodes.insert(owner);
    std::map<NodeId,std::size_t> indegree;
    std::map<NodeId,std::vector<NodeId>> successors;
    std::map<NodeId,std::size_t> owner_references;
    std::set<std::string> edge_ids;
    for (NodeId node:nodes) indegree[node]=0;
    for (const auto& edge:edges) {
        if (!nodes.count(edge.reference)||edge.owner<1||edge.owner>14||
            !edge_ids.insert(edge.id()).second) {
            reason="invalid_or_duplicate_edge";
            return {};
        }
        ++indegree[edge.owner];
        ++owner_references[edge.owner];
        successors[edge.reference].push_back(edge.owner);
    }
    for (NodeId owner=1;owner<=14;++owner)
        if (owner_references[owner]!=2) {
            reason="owner_reference_count";
            return {};
        }
    std::priority_queue<NodeId,std::vector<NodeId>,std::greater<NodeId>> ready;
    for (const auto& [node,degree]:indegree)
        if (degree==0) ready.push(node);
    std::vector<NodeId> result;
    while (!ready.empty()) {
        const NodeId node=ready.top();
        ready.pop();
        result.push_back(node);
        auto next=successors[node];
        std::sort(next.begin(),next.end());
        for (NodeId successor:next)
            if (--indegree[successor]==0) ready.push(successor);
    }
    if (result.size()!=nodes.size()) {
        reason="cycle";
        return {};
    }
    return result;
}

inline void addRoles(Task20DagLatticeContract& result,
    const Task20CoverageUnit& unit,
    const std::vector<std::pair<double,double>>& coordinates) {
    if (unit.members.size()!=coordinates.size()) return;
    for (std::size_t index=0;index<unit.members.size();++index)
        result.member_roles.emplace(unit.members[index],Task20MemberRole{
            unit.members[index],unit.id,coordinates[index].first,
            coordinates[index].second});
}

inline void finish(Task20DagLatticeContract& result) {
    if (result.reference_edges.size()!=28||result.member_roles.size()!=14) {
        result.reason="contract_cardinality";
        return;
    }
    std::set<NodeId> assigned;
    for (const auto& unit:result.coverage_units) {
        if (unit.members.empty()||unit.base_anchors.empty()||
            std::find(unit.members.begin(),unit.members.end(),unit.leader)==
                unit.members.end()) {
            result.reason="invalid_coverage_unit";
            return;
        }
        for (NodeId member:unit.members)
            if (!assigned.insert(member).second) {
                result.reason="duplicate_coverage_member";
                return;
            }
    }
    if (assigned.size()!=14) {
        result.reason="missing_coverage_member";
        return;
    }
    result.topological_order=topologicalOrder(result.reference_edges,result.reason);
    result.valid=result.topological_order.size()==17;
}

}  // namespace task20_lattice_detail

inline Task20DagLatticeContract task20DagLatticeContract(
    Task20LatticeMode mode) {
    Task20DagLatticeContract result;
    if (mode==Task20LatticeMode::DualLadder) {
        result.id="dual-ladder";
        result.structural_signature="units=7+7;rows=2x(2-2-2-1);ladder";
        result.reference_edges=task20_lattice_detail::dualLadderEdges();
        result.coverage_units={{"A",{1,2,3,4,5,6,7},{101},7,{7}},
                               {"B",{8,9,10,11,12,13,14},{101},14,{14}}};
        const std::vector<std::pair<double,double>> a{
            {0.25,0.0},{0.25,-0.25},{0.50,0.0},{0.50,-0.25},
            {0.75,0.0},{0.75,-0.25},{1.0,0.0}};
        const std::vector<std::pair<double,double>> b{
            {0.25,0.0},{0.25,0.25},{0.50,0.0},{0.50,0.25},
            {0.75,0.0},{0.75,0.25},{1.0,0.0}};
        task20_lattice_detail::addRoles(result,result.coverage_units[0],a);
        task20_lattice_detail::addRoles(result,result.coverage_units[1],b);
    } else if (mode==Task20LatticeMode::MergedStrip) {
        result.id="merged-strip";
        result.structural_signature="units=14;rows=2-2-2-2-2-2-2;merged-cross";
        result.reference_edges=task20_lattice_detail::mergedStripEdges();
        result.coverage_units={{"M",{1,8,2,9,3,10,4,11,5,12,6,13,7,14},
                                {100,101,102},14,{7,14}}};
        std::vector<std::pair<double,double>> roles;
        for (int depth=1;depth<=7;++depth) {
            roles.emplace_back(depth/7.0,-0.09);
            roles.emplace_back(depth/7.0,0.09);
        }
        task20_lattice_detail::addRoles(result,result.coverage_units[0],roles);
    } else if (mode==Task20LatticeMode::SplitThreeFront) {
        result.id="split-three-front";
        result.structural_signature="units=5+5+4;three-fibonacci-chains";
        result.reference_edges=task20_lattice_detail::splitThreeFrontEdges();
        result.coverage_units={{"L",{1,2,3,4,5},{100,101},5,{5}},
                               {"C",{6,7,8,9,10},{101,102},10,{10}},
                               {"R",{11,12,13,14},{100,102},14,{14}}};
        task20_lattice_detail::addRoles(result,result.coverage_units[0],
            {{0.20,-0.08},{0.40,0.08},{0.60,-0.08},{0.80,0.08},{1.0,0.0}});
        task20_lattice_detail::addRoles(result,result.coverage_units[1],
            {{0.20,-0.08},{0.40,0.08},{0.60,-0.08},{0.80,0.08},{1.0,0.0}});
        task20_lattice_detail::addRoles(result,result.coverage_units[2],
            {{0.25,-0.08},{0.50,0.08},{0.75,-0.08},{1.0,0.0}});
    } else {
        result.id="cross-braced-diamond";
        result.structural_signature="units=7+7;rows=1-2-3-1;diamond";
        result.reference_edges=task20_lattice_detail::diamondEdges();
        result.coverage_units={{"A",{1,2,3,4,5,6,7},{100,101},7,{7}},
                               {"B",{8,9,10,11,12,13,14},{101,102},14,{14}}};
        const std::vector<std::pair<double,double>> roles{
            {0.25,0.0},{0.50,-0.12},{0.50,0.12},{0.75,0.0},
            {0.75,-0.16},{0.75,0.16},{1.0,0.0}};
        task20_lattice_detail::addRoles(result,result.coverage_units[0],roles);
        task20_lattice_detail::addRoles(result,result.coverage_units[1],roles);
    }
    task20_lattice_detail::finish(result);
    return result;
}

inline Task20LiftResult task20LiftTargets(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const std::map<std::string,Eigen::Vector2d>& fronts) {
    Task20LiftResult result;
    if (!contract.valid) {
        result.reason="invalid_contract";
        return result;
    }
    for (const auto& unit:contract.coverage_units) {
        const auto front=fronts.find(unit.id);
        if (front==fronts.end()||!front->second.allFinite()) {
            result.reason="missing_front:"+unit.id;
            return result;
        }
        Eigen::Vector2d base=Eigen::Vector2d::Zero();
        for (NodeId anchor:unit.base_anchors) {
            const auto fixed=fixed_positions.find(anchor);
            if (fixed==fixed_positions.end()||!fixed->second.allFinite()) {
                result.reason="missing_anchor";
                return result;
            }
            base+=fixed->second;
        }
        base/=static_cast<double>(unit.base_anchors.size());
        const Eigen::Vector2d displacement=front->second-base;
        for (NodeId member:unit.members) {
            const auto& role=contract.member_roles.at(member);
            const double sign=role.triangular_fraction<0.0?-1.0:1.0;
            const Eigen::Vector2d triangular=
                task20_lattice_detail::rotate60(displacement,sign);
            result.targets[member]=base+role.axial_fraction*displacement+
                std::abs(role.triangular_fraction)*triangular;
        }
    }
    result.valid=result.targets.size()==14;
    if (!result.valid) result.reason="incomplete_lifting";
    return result;
}

inline Task20FrontInverseResult task20FrontForMemberPose(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    NodeId member,const Eigen::Vector2d& service_pose) {
    Task20FrontInverseResult result;
    if (!contract.valid||!service_pose.allFinite()) {
        result.reason="invalid_inverse_request";
        return result;
    }
    const auto role=contract.member_roles.find(member);
    if (role==contract.member_roles.end()) {
        result.reason="unknown_member";
        return result;
    }
    const auto unit=std::find_if(contract.coverage_units.begin(),
        contract.coverage_units.end(),[&](const auto& value) {
            return value.id==role->second.coverage_unit;
        });
    if (unit==contract.coverage_units.end()) {
        result.reason="missing_member_unit";
        return result;
    }
    Eigen::Vector2d base=Eigen::Vector2d::Zero();
    for (NodeId anchor:unit->base_anchors) {
        const auto fixed=fixed_positions.find(anchor);
        if (fixed==fixed_positions.end()) {
            result.reason="missing_anchor";
            return result;
        }
        base+=fixed->second;
    }
    base/=static_cast<double>(unit->base_anchors.size());
    const double a=role->second.axial_fraction;
    const double t=std::abs(role->second.triangular_fraction);
    const double sign=role->second.triangular_fraction<0.0?-1.0:1.0;
    constexpr double cosine=0.5;
    constexpr double sine=0.86602540378443864676;
    Eigen::Matrix2d matrix;
    matrix<<a+t*cosine,-sign*t*sine,
            sign*t*sine,a+t*cosine;
    if (std::abs(matrix.determinant())<1.0e-12) {
        result.reason="singular_member_role";
        return result;
    }
    result.coverage_unit=unit->id;
    result.front=base+matrix.inverse()*(service_pose-base);
    result.valid=result.front.allFinite();
    if (!result.valid) result.reason="nonfinite_inverse";
    return result;
}

}  // namespace gf
