#pragma once

#include "grand_finale/SharedCollisionViableFrontierAllocator.hpp"

#include <cstdint>
#include <sstream>

namespace gf {

struct LeaderCoverageBranchSpec {
    std::vector<NodeId> members;
    NodeId leader=0;
    Eigen::Vector2d coverage_origin=Eigen::Vector2d::Zero();
    double rotation_rad=0.0;
    std::vector<NodeId> preferred_fixed_roots;
    // Task 13 B0-a v3 forward compatibility: ladder segment count
    // parameterized for B1's component-wise CVT / lattice-band layouts.
    int ladder_segments=4;
};

inline std::vector<LeaderCoverageBranchSpec>
task10p11hLeaderCoverageSpec() {
    return {{{1,2,3,4,5,6,7},7,{250.0,-50.0},-M_PI/3.0,{101,100}},
            {{8,9,10,11,12,13,14},14,{250.0,-50.0},M_PI/3.0,{101,102}}};
}

struct LeaderCoverageRequest {
    std::vector<FrontierAgentState> agents;
    std::vector<FrontierCell> uncovered_cells;
    std::vector<FrontierCell> domain_cells;
    std::vector<LeaderCoverageBranchSpec> branches=
        task10p11hLeaderCoverageSpec();
    // Task 13 B0-a v3: leader target = centroid of the uncovered CVT
    // share (nearest cell only as the near-empty closing degradation).
    bool leader_centroid_primary=false;
    int leader_centroid_near_empty_cells=2;
    // Task 13 B0-a v5: hard reachability filter - a leader candidate must
    // lie within R_eff of EVERY branch reference (disk-intersection
    // convexity => the root->leader segment inherits reachability).  An
    // empty filtered set is the topology-health signal (B1 trigger) and
    // degrades to the nearest cell.
    bool leader_reachability_filter=false;
    std::map<NodeId,std::vector<std::pair<Eigen::Vector2d,double>>>
        leader_reference_disks;
    // Corrected (sensing) semantics: hard bound on the distance to the
    // NEAREST reference; the per-reference R_eff radii in the disks act
    // only as the sorting weight.
    double leader_reachability_sensing_bound_m = 0.0;
    double leader_reachability_weight = 0.25;
};

struct LeaderCoverageResult {
    bool valid=false;
    std::string reason;
    std::map<NodeId,FrontierCell> leader_targets;
    std::map<NodeId,FrontierCell> targets;
    std::map<std::string,NodeId> voronoi_owner;
    std::set<NodeId> centroid_fallback_leaders;
    // Task 13 B0-a v5: leaders whose reachable-candidate set was empty -
    // the topology-health signal wired to the B1 trigger.
    std::set<NodeId> leader_reachability_fallback_leaders;
    std::uint64_t request_digest=0;
};

namespace leader_coverage_detail {

inline std::uint64_t hashText(const std::string& value) {
    std::uint64_t hash=1469598103934665603ULL;
    for (const unsigned char byte : value) {
        hash^=byte;
        hash*=1099511628211ULL;
    }
    return hash==0?1:hash;
}

inline Eigen::Vector2d clipForLegacyAblation(
    Eigen::Vector2d point,const Eigen::Vector2d& minimum,
    const Eigen::Vector2d& maximum) {
    point.x()=std::max(minimum.x(),std::min(maximum.x(),point.x()));
    point.y()=std::max(minimum.y(),std::min(maximum.y(),point.y()));
    return point;
}

inline const FrontierAgentState& leaderState(
    const std::vector<FrontierAgentState>& agents,NodeId leader) {
    const auto found=std::find_if(agents.begin(),agents.end(),
        [&](const auto& value) { return value.id==leader; });
    if (found==agents.end())
        throw std::invalid_argument("missing coverage leader");
    return *found;
}

}  // namespace leader_coverage_detail

inline LeaderCoverageResult allocateLeaderCoverageTargets(
    LeaderCoverageRequest request) {
    LeaderCoverageResult result;
    if (request.domain_cells.empty()) {
        result.reason="invalid_leader_coverage_request";
        return result;
    }
    std::sort(request.agents.begin(),request.agents.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id<rhs.id; });
    std::sort(request.uncovered_cells.begin(),request.uncovered_cells.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
    std::sort(request.domain_cells.begin(),request.domain_cells.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
    std::set<NodeId> ids;
    for (const auto& agent : request.agents)
        if (agent.id<=0 || !agent.position.allFinite() ||
            !agent.velocity.allFinite() || !ids.insert(agent.id).second) {
            result.reason="invalid_leader_coverage_agent";
            return result;
        }
    std::sort(request.branches.begin(),request.branches.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.leader<rhs.leader; });
    std::set<NodeId> branch_members;
    std::set<NodeId> branch_leaders;
    for (const auto& branch : request.branches) {
        if (branch.leader==0 || branch.members.empty() ||
            !branch_leaders.insert(branch.leader).second ||
            std::find(branch.members.begin(),branch.members.end(),
                      branch.leader)==branch.members.end() ||
            !branch.coverage_origin.allFinite() ||
            !std::isfinite(branch.rotation_rad)) {
            result.reason="invalid_leader_coverage_branch";
            return result;
        }
        for (NodeId member : branch.members)
            if (!ids.count(member) || !branch_members.insert(member).second) {
                result.reason="invalid_leader_coverage_branch";
                return result;
            }
    }
    if (branch_members!=ids) {
        result.reason="leader_coverage_branch_partition";
        return result;
    }
    const auto& branches=request.branches;
    std::vector<FrontierAgentState> leaders;
    try {
        for (const auto& branch : branches)
            leaders.push_back(
                leader_coverage_detail::leaderState(request.agents,branch.leader));
    } catch (const std::invalid_argument&) {
        result.reason="missing_coverage_leader";
        return result;
    }

    std::map<NodeId,std::vector<FrontierCell>> domain;
    std::set<std::string> domain_ids;
    for (const auto& cell : request.domain_cells) {
        if (!cell.center.allFinite() || !domain_ids.insert(cell.id()).second) {
            result.reason="invalid_leader_coverage_domain";
            return result;
        }
        const NodeId owner=deterministicVoronoiOwner(cell,leaders);
        result.voronoi_owner[cell.id()]=owner;
        domain[owner].push_back(cell);
    }
    for (const auto& leader : leaders)
        if (domain[leader.id].empty()) {
            result.reason="leader_voronoi_cell_empty";
            return result;
        }

    std::map<NodeId,std::vector<FrontierCell>> uncovered;
    std::set<std::string> uncovered_ids;
    for (const auto& cell : request.uncovered_cells) {
        if (!cell.center.allFinite() ||
            !uncovered_ids.insert(cell.id()).second ||
            !domain_ids.count(cell.id())) {
            result.reason="invalid_leader_uncovered_cell";
            return result;
        }
        uncovered[deterministicVoronoiOwner(cell,leaders)].push_back(cell);
    }

    for (const auto& leader : leaders) {
        auto& candidates=uncovered[leader.id];
        if (!candidates.empty()) {
            if (request.leader_centroid_primary &&
                static_cast<int>(candidates.size())>
                    request.leader_centroid_near_empty_cells) {
                // Task 13 B0-a v5: hard reachability filter - a candidate
                // must lie within R_eff of every branch reference (disk
                // intersection is convex, so the root->leader segment
                // inherits reachability).  An empty filtered set is the
                // topology-health signal and degrades to the nearest
                // cell.
                std::vector<FrontierCell> reachable;
                const auto disk_it=
                    request.leader_reference_disks.find(leader.id);
                if (!request.leader_reachability_filter ||
                    disk_it==request.leader_reference_disks.end() ||
                    disk_it->second.empty() ||
                    request.leader_reachability_sensing_bound_m<=0.0) {
                    reachable=candidates;
                } else {
                    // Corrected (sensing) semantics: only the distance to
                    // the NEAREST reference is bounded; the per-reference
                    // R_eff radii act solely as the sorting weight.
                    for (const auto& cell : candidates) {
                        double d_min=std::numeric_limits<double>::infinity();
                        for (const auto& disk : disk_it->second)
                            d_min=std::min(d_min,
                                (cell.center-disk.first).norm());
                        if (d_min<=request.
                                leader_reachability_sensing_bound_m)
                            reachable.push_back(cell);
                    }
                    if (reachable.empty())
                        result.leader_reachability_fallback_leaders.insert(
                            leader.id);
                }
                auto& scored_set=reachable.empty()?candidates:reachable;
                const bool degraded=reachable.empty();
                if (degraded) {
                    std::sort(scored_set.begin(),scored_set.end(),
                        [&](const auto& lhs,const auto& rhs) {
                            const double dl=(lhs.center-leader.position).squaredNorm();
                            const double dr=(rhs.center-leader.position).squaredNorm();
                            return dl<dr || (dl==dr && lhs.id()<rhs.id());
                        });
                    result.leader_targets[leader.id]=scored_set.front();
                } else {
                // Task 13 B0-a v4 (three-strikes round): nearest-cell
                // frontier pacing + centroid direction-preference scoring.
                // The classic nearest-cell walk is an implicit
                // reachability pacer (v3 showed pure centroid targets
                // burn the ladder budget at 28.8 s); the scoring adds a
                // centroid-direction bias without abandoning pacing:
                //   score = dist * (1 + w_dir*(1 - cos(theta)))
                // with theta the angle between (cell-leader) and the
                // share centroid direction, w_dir = 0.5 frozen.
                Eigen::Vector2d centroid=Eigen::Vector2d::Zero();
                for (const auto& cell : scored_set) centroid+=cell.center;
                centroid/=static_cast<double>(scored_set.size());
                const Eigen::Vector2d centroid_dir=
                    centroid-leader.position;
                const double centroid_norm=centroid_dir.norm();
                std::vector<std::pair<double,const FrontierCell*>> scored;
                scored.reserve(scored_set.size());
                for (const auto& cell : scored_set) {
                    const Eigen::Vector2d delta=cell.center-leader.position;
                    const double dist=delta.norm();
                    double score=dist;
                    if (dist>1e-9 && centroid_norm>1e-9) {
                        const double cos_theta=std::clamp(
                            delta.dot(centroid_dir)/(dist*centroid_norm),
                            -1.0,1.0);
                        score=dist*(1.0+0.5*(1.0-cos_theta));
                    }
                    if (request.leader_reachability_weight>0.0) {
                        double d_min=std::numeric_limits<double>::infinity();
                        for (const auto& disk : disk_it->second)
                            d_min=std::min(d_min,
                                (cell.center-disk.first).norm());
                        score+=request.leader_reachability_weight*d_min;
                    }
                    scored.emplace_back(score,&cell);
                }
                std::sort(scored.begin(),scored.end(),
                    [](const auto& lhs,const auto& rhs) {
                        return lhs.first<rhs.first ||
                            (lhs.first==rhs.first &&
                             lhs.second->id()<rhs.second->id());
                    });
                result.leader_targets[leader.id]=*scored.front().second;
                }
            } else {
            Eigen::Vector2d centroid=Eigen::Vector2d::Zero();
            for (const auto& cell : domain[leader.id]) centroid+=cell.center;
            centroid/=static_cast<double>(domain[leader.id].size());
            result.leader_targets[leader.id]={-leader.id,-1,centroid};
            result.centroid_fallback_leaders.insert(leader.id);
        }
    }
    }

    for (const auto& branch : branches) {
        // map::at robustness (microfix v5 crash fix): a missing leader
        // entry degrades to the branch coverage origin instead of
        // throwing.
        const auto leader_it=result.leader_targets.find(branch.leader);
        const FrontierCell leader_target=
            leader_it!=result.leader_targets.end()
                ?leader_it->second
                :FrontierCell{-branch.leader,-1,branch.coverage_origin};
        const Eigen::Vector2d section=
            (leader_target.center-branch.coverage_origin)/
                static_cast<double>(branch.ladder_segments);
        const Eigen::Rotation2Dd rotate(branch.rotation_rad);
        for (std::size_t index=0;index<branch.members.size();++index) {
            const int local=static_cast<int>(index)+1;
            const int sections=(8-local)/2;
            Eigen::Vector2d target=
                leader_target.center-static_cast<double>(sections)*section;
            if ((7-local)%2==1) target+=rotate*section;
            const NodeId owner=branch.members[index];
            result.targets[owner]=owner==branch.leader
                ? leader_target:FrontierCell{-owner,-2,target};
        }
    }

    std::ostringstream canonical;
    canonical.precision(17);
    for (const auto& agent : request.agents)
        canonical<<agent.id<<':'<<agent.position.transpose()<<';';
    canonical<<"|domain=";
    for (const auto& cell : request.domain_cells)
        canonical<<cell.id()<<':'<<cell.center.transpose()<<';';
    canonical<<"|uncovered=";
    for (const auto& cell : request.uncovered_cells)
        canonical<<cell.id()<<':'<<cell.center.transpose()<<';';
    canonical<<"|branches=";
    for (const auto& branch : branches) {
        canonical<<branch.leader<<':'<<branch.coverage_origin.transpose()<<':'
                 <<branch.rotation_rad<<':';
        for (NodeId member : branch.members) canonical<<member<<',';
        canonical<<';';
    }
    result.request_digest=leader_coverage_detail::hashText(canonical.str());
    result.valid=true;
    result.reason=request.uncovered_cells.empty()
        ?"leader_centroid_fallback":"leader_search_targets";
    return result;
}

inline LeaderCoverageResult legacyProjectLeaderCoverageTargetsToSearchPolygon(
    LeaderCoverageResult result,const Eigen::Vector2d& minimum,
    const Eigen::Vector2d& maximum) {
    if (!result.valid || !minimum.allFinite() || !maximum.allFinite() ||
        (minimum.array()>maximum.array()).any())
        throw std::invalid_argument("invalid legacy search polygon projection");
    for (auto& [owner,target] : result.targets) {
        (void)owner;
        target.center=leader_coverage_detail::clipForLegacyAblation(
            target.center,minimum,maximum);
    }
    return result;
}

}  // namespace gf
