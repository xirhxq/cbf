#pragma once

#include "grand_finale/SharedCollisionViableFrontierAllocator.hpp"

#include <cstdint>
#include <limits>

namespace gf {

enum class TargetProjection {
    None,
    LegacySearchPolygonClippingAblation
};

struct SimpleCoveragePolicyConfig {
    std::size_t allocation_epoch_cycles=10;
    std::size_t fairness_promotion_age=3;
    std::size_t maximum_certification_failures=3;
    double comparison_tolerance=1.0e-12;
    TargetProjection target_projection=TargetProjection::None;
};

struct SimpleCoveragePolicyRequest {
    std::vector<FrontierAgentState> agents;
    std::vector<FrontierCell> uncovered_cells;
    std::vector<FrontierCell> domain_cells;
    std::map<std::string,std::size_t> fairness_ages;
    std::size_t priority_epoch=0;
};

struct SimpleCoveragePolicyResult {
    bool valid=false;
    std::string reason;
    std::map<NodeId,FrontierCell> targets;
    std::map<std::string,NodeId> voronoi_owner;
    std::set<NodeId> cvt_fallback_owners;
    std::size_t uncovered_input_count=0;
    std::size_t remaining_unassigned_count=0;
    std::uint64_t request_digest=0;
};

namespace simple_coverage_detail {

inline std::uint64_t hashText(const std::string& text) {
    std::uint64_t hash=1469598103934665603ULL;
    for (const unsigned char value : text) {
        hash^=value;
        hash*=1099511628211ULL;
    }
    return hash==0?1:hash;
}

inline std::size_t ageOf(
    const std::map<std::string,std::size_t>& ages,const FrontierCell& cell) {
    const auto found=ages.find(cell.id());
    return found==ages.end()?0:found->second;
}

inline bool cellBetter(
    const FrontierCell& candidate,const FrontierCell& incumbent,
    NodeId owner,const std::map<NodeId,Eigen::Vector2d>& position,
    const std::map<std::string,std::size_t>& ages,
    const SimpleCoveragePolicyConfig& config) {
    const std::size_t candidate_age=ageOf(ages,candidate);
    const std::size_t incumbent_age=ageOf(ages,incumbent);
    const bool candidate_promoted=
        candidate_age>=config.fairness_promotion_age;
    const bool incumbent_promoted=
        incumbent_age>=config.fairness_promotion_age;
    if (candidate_promoted!=incumbent_promoted) return candidate_promoted;
    if (candidate_promoted && candidate_age!=incumbent_age)
        return candidate_age>incumbent_age;
    const double candidate_distance=
        (candidate.center-position.at(owner)).squaredNorm();
    const double incumbent_distance=
        (incumbent.center-position.at(owner)).squaredNorm();
    if (candidate_distance+config.comparison_tolerance<incumbent_distance)
        return true;
    if (incumbent_distance+config.comparison_tolerance<candidate_distance)
        return false;
    return candidate.id()<incumbent.id();
}

}  // namespace simple_coverage_detail

inline SimpleCoveragePolicyResult allocateSimpleCoverageTargets(
    SimpleCoveragePolicyRequest request,
    SimpleCoveragePolicyConfig config={}) {
    SimpleCoveragePolicyResult result;
    result.uncovered_input_count=request.uncovered_cells.size();
    if (request.agents.empty() || config.allocation_epoch_cycles==0 ||
        config.fairness_promotion_age==0 ||
        config.maximum_certification_failures==0 ||
        !std::isfinite(config.comparison_tolerance) ||
        config.comparison_tolerance<0.0) {
        result.reason="invalid_simple_policy_request";
        return result;
    }
    std::sort(request.agents.begin(),request.agents.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id<rhs.id; });
    std::sort(request.uncovered_cells.begin(),request.uncovered_cells.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
    std::sort(request.domain_cells.begin(),request.domain_cells.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
    std::map<NodeId,Eigen::Vector2d> positions;
    for (const auto& agent : request.agents) {
        if (agent.id<=0 || !agent.position.allFinite() ||
            !agent.velocity.allFinite() || !positions.emplace(
                agent.id,agent.position).second) {
            result.reason="invalid_simple_policy_agent";
            return result;
        }
    }
    std::set<std::string> cell_ids;
    std::map<NodeId,std::vector<FrontierCell>> partition;
    for (const auto& value : request.uncovered_cells) {
        if (!value.center.allFinite() || !cell_ids.insert(value.id()).second) {
            result.reason="invalid_simple_policy_cell";
            return result;
        }
        const NodeId owner=deterministicVoronoiOwner(value,request.agents);
        result.voronoi_owner[value.id()]=owner;
        partition[owner].push_back(value);
    }
    std::set<std::string> domain_ids;
    std::map<NodeId,std::vector<FrontierCell>> domain_partition;
    for (const auto& value : request.domain_cells) {
        if (!value.center.allFinite() || !domain_ids.insert(value.id()).second) {
            result.reason="invalid_simple_policy_domain";
            return result;
        }
        domain_partition[deterministicVoronoiOwner(
            value,request.agents)].push_back(value);
    }
    if (request.domain_cells.empty()) {
        result.reason="empty_simple_policy_domain";
        return result;
    }
    std::ostringstream canonical;
    canonical.precision(17);
    for (const auto& agent : request.agents)
        canonical<<agent.id<<':'<<agent.position.x()<<','<<agent.position.y()<<';';
    canonical<<'|';
    for (const auto& value : request.uncovered_cells)
        canonical<<value.id()<<':'<<value.center.x()<<','<<value.center.y()<<';';
    canonical<<"|domain=";
    for (const auto& value : request.domain_cells)
        canonical<<value.id()<<':'<<value.center.x()<<','<<value.center.y()<<';';
    canonical<<"|epoch="<<request.priority_epoch;
    for (const auto& [id,age] : request.fairness_ages)
        canonical<<'|'<<id<<'='<<age;
    result.request_digest=simple_coverage_detail::hashText(canonical.str());

    std::vector<NodeId> order;
    for (const auto& agent : request.agents) order.push_back(agent.id);
    std::rotate(order.begin(),
        order.begin()+static_cast<std::ptrdiff_t>(request.priority_epoch%order.size()),
        order.end());
    std::set<std::string> available=cell_ids;
    const auto choose=[&](NodeId owner,const std::vector<FrontierCell>& pool)
        -> std::optional<FrontierCell> {
        std::optional<FrontierCell> best;
        for (const auto& value : pool) {
            if (!available.count(value.id())) continue;
            if (!best.has_value() || simple_coverage_detail::cellBetter(
                    value,*best,owner,positions,request.fairness_ages,config))
                best=value;
        }
        return best;
    };
    for (NodeId owner : order) {
        auto selected=choose(owner,partition[owner]);
        if (selected.has_value()) {
            result.targets[owner]=*selected;
            available.erase(selected->id());
            continue;
        }
        Eigen::Vector2d centroid=positions.at(owner);
        const auto& cells=domain_partition[owner];
        if (!cells.empty()) {
            centroid.setZero();
            for (const auto& value : cells) centroid+=value.center;
            centroid/=static_cast<double>(cells.size());
        }
        result.targets[owner]={-owner,-1,centroid};
        result.cvt_fallback_owners.insert(owner);
    }
    result.remaining_unassigned_count=available.size();
    result.valid=true;
    result.reason=request.uncovered_cells.empty()?"full_domain_cvt":
        (result.cvt_fallback_owners.empty()?"allocated":"allocated_with_cvt");
    return result;
}

class SimpleCoverageFairnessLedger {
public:
    void advanceUncovered(const std::vector<FrontierCell>& cells) {
        for (const auto& cell : cells) {
            denominator_ids_.insert(cell.id());
            ++ages_[cell.id()];
        }
    }
    void recordServed(const std::string& id) {
        denominator_ids_.insert(id);
        ages_[id]=0;
    }
    const std::map<std::string,std::size_t>& ages() const { return ages_; }
    const std::set<std::string>& denominatorIds() const {
        return denominator_ids_;
    }
private:
    std::map<std::string,std::size_t> ages_;
    std::set<std::string> denominator_ids_;
};

}  // namespace gf
