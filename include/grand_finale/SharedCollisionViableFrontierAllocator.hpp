#pragma once

#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/ProgressCompatibility.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace gf {

struct FrontierCell {
    int x_index = 0;
    int y_index = 0;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();

    std::string id() const {
        return std::to_string(x_index)+":"+std::to_string(y_index);
    }
    bool operator==(const FrontierCell& other) const {
        return x_index == other.x_index && y_index == other.y_index;
    }
    bool operator!=(const FrontierCell& other) const { return !(*this==other); }
};

struct FrontierAgentState {
    NodeId id = 0;
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
};

inline NodeId deterministicVoronoiOwner(
    const FrontierCell& cell,
    const std::vector<FrontierAgentState>& agents) {
    if (agents.empty())
        throw std::invalid_argument("Voronoi ownership needs an agent");
    NodeId owner = agents.front().id;
    double best = (cell.center-agents.front().position).squaredNorm();
    for (std::size_t index=1; index<agents.size(); ++index) {
        const double distance =
            (cell.center-agents[index].position).squaredNorm();
        if (distance < best ||
            (distance == best && agents[index].id < owner)) {
            owner = agents[index].id;
            best = distance;
        }
    }
    return owner;
}

struct SharedFrontierAllocatorConfig {
    std::size_t top_k = 8;
    std::size_t bundle_cap = 32;
    std::size_t rollout_cap = 4;
    std::size_t epoch_cycles = 5;
    std::size_t rollout_cycles = 5;
    double maximum_projection_norm = 0.8;
    double minimum_direction_ratio = 0.05;
    double comparison_tolerance = 1e-10;
};

struct FrontierAllocationRequest {
    std::uint64_t snapshot_token = 0;
    std::uint64_t topology_token = 0;
    std::uint64_t grid_token = 0;
    std::vector<FrontierAgentState> agents;
    std::vector<FrontierCell> cells;
    std::vector<CanonicalHardRow> hard_rows;
    double acceleration_half_box = 0.0;
    double collision_distance_m = 0.0;
    double position_reserve_m = 0.0;
    double velocity_reserve_mps = 0.0;
    double position_gain = 0.0;
    double velocity_gain = 0.0;
};

enum class AllocatorExhaustion {
    None,
    CandidateUniverseEmpty,
    FastGateRejected,
    ExactGateRejected,
    RolloutRejected,
    SearchBudgetTruncated
};

struct FrontierAllocationResult {
    bool accepted = false;
    std::string reason;
    AllocatorExhaustion exhaustion = AllocatorExhaustion::None;
    bool physical_deadlock_claimed = false;
    std::map<NodeId,FrontierCell> targets;
    std::map<NodeId,Eigen::Vector2d> nominal_controls;
    std::string bundle_id;
    std::size_t bundle_attempts = 0;
    std::size_t priority_epoch = 0;
};

class SharedCollisionViableFrontierAllocator {
public:
    explicit SharedCollisionViableFrontierAllocator(
        SharedFrontierAllocatorConfig config) : config_(config) {
        if (config_.top_k == 0 || config_.bundle_cap == 0 ||
            config_.rollout_cap == 0 || config_.epoch_cycles == 0 ||
            config_.rollout_cycles < config_.epoch_cycles ||
            !std::isfinite(config_.maximum_projection_norm) ||
            config_.maximum_projection_norm < 0.0 ||
            !std::isfinite(config_.minimum_direction_ratio) ||
            config_.minimum_direction_ratio < 0.0 ||
            config_.minimum_direction_ratio > 1.0 ||
            !std::isfinite(config_.comparison_tolerance) ||
            config_.comparison_tolerance < 0.0) {
            throw std::invalid_argument("invalid shared frontier allocator config");
        }
    }

    FrontierAllocationResult allocate(const FrontierAllocationRequest& request) {
        validate(request);
        FrontierAllocationResult result;
        result.priority_epoch = priority_epoch_++;
        for (const auto& cell : request.cells) ++ages_[cell.id()];
        if (request.cells.empty()) {
            result.reason = "allocator_search_exhausted";
            result.exhaustion = AllocatorExhaustion::CandidateUniverseEmpty;
            return result;
        }

        std::vector<FrontierAgentState> agents = request.agents;
        const std::size_t rotation = agents.empty()
            ? 0 : result.priority_epoch % agents.size();
        std::rotate(agents.begin(),agents.begin()+rotation,agents.end());
        std::map<NodeId,std::vector<FrontierCell>> candidates;
        for (const auto& agent : agents) {
            std::vector<FrontierCell> owned;
            std::vector<FrontierCell> overflow;
            for (const auto& cell : request.cells) {
                if (deterministicVoronoiOwner(cell,request.agents)==agent.id)
                    owned.push_back(cell);
                else
                    overflow.push_back(cell);
            }
            const auto order = [&](const auto& lhs,const auto& rhs) {
                const auto lhs_key = std::make_tuple(
                    -static_cast<long long>(ages_[lhs.id()]),
                    (lhs.center-agent.position).squaredNorm(),lhs.id());
                const auto rhs_key = std::make_tuple(
                    -static_cast<long long>(ages_[rhs.id()]),
                    (rhs.center-agent.position).squaredNorm(),rhs.id());
                return lhs_key < rhs_key;
            };
            std::sort(owned.begin(),owned.end(),order);
            std::sort(overflow.begin(),overflow.end(),order);
            auto cells = std::move(owned);
            cells.insert(cells.end(),overflow.begin(),overflow.end());
            if (cells.size() > config_.top_k) cells.resize(config_.top_k);
            candidates[agent.id] = std::move(cells);
        }

        const std::string snapshot_prefix = tokenPrefix(request);
        for (std::size_t attempt=0; attempt<config_.bundle_cap; ++attempt) {
            ++result.bundle_attempts;
            std::map<NodeId,FrontierCell> bundle;
            std::set<std::string> used;
            bool complete = true;
            for (std::size_t rank=0; rank<agents.size(); ++rank) {
                const auto& options = candidates.at(agents[rank].id);
                bool assigned = false;
                for (std::size_t delta=0; delta<options.size(); ++delta) {
                    const auto& cell = options[(attempt+delta)%options.size()];
                    if (used.insert(cell.id()).second) {
                        bundle[agents[rank].id] = cell;
                        assigned = true;
                        break;
                    }
                }
                if (!assigned) { complete=false; break; }
            }
            if (!complete) continue;
            const std::string bundle_id = bundleId(bundle);
            if (no_goods_.count(snapshot_prefix+bundle_id) != 0) continue;
            std::map<NodeId,Eigen::Vector2d> nominal;
            for (const auto& agent : agents) {
                Eigen::Vector2d value = request.position_gain *
                    (bundle.at(agent.id).center-agent.position) -
                    request.velocity_gain*agent.velocity;
                value.x() = std::clamp(value.x(),-request.acceleration_half_box,
                    request.acceleration_half_box);
                value.y() = std::clamp(value.y(),-request.acceleration_half_box,
                    request.acceleration_half_box);
                nominal[agent.id] = value;
            }
            result.accepted = true;
            result.reason = "accepted";
            result.targets = std::move(bundle);
            result.nominal_controls = std::move(nominal);
            result.bundle_id = bundle_id;
            last_bundle_key_ = snapshot_prefix+bundle_id;
            return result;
        }
        result.reason = "allocator_search_exhausted";
        result.exhaustion = result.bundle_attempts >= config_.bundle_cap
            ? AllocatorExhaustion::SearchBudgetTruncated
            : AllocatorExhaustion::ExactGateRejected;
        return result;
    }

    void rejectCurrentBundle(const FrontierAllocationRequest& request) {
        const std::string prefix = tokenPrefix(request);
        if (!last_bundle_key_.empty() && last_bundle_key_.rfind(prefix,0)==0)
            no_goods_.insert(last_bundle_key_);
    }

    std::size_t rejectionAge(const std::string& cell_id) const {
        const auto it = ages_.find(cell_id);
        return it == ages_.end() ? 0 : it->second;
    }

private:
    static std::string tokenPrefix(const FrontierAllocationRequest& request) {
        return std::to_string(request.snapshot_token)+"/"+
            std::to_string(request.topology_token)+"/"+
            std::to_string(request.grid_token)+"/";
    }
    static std::string bundleId(const std::map<NodeId,FrontierCell>& bundle) {
        std::ostringstream out;
        for (const auto& [id,cell] : bundle) out << id << '=' << cell.id() << ';';
        return out.str();
    }
    static void validate(const FrontierAllocationRequest& request) {
        if (request.agents.empty() ||
            !std::isfinite(request.acceleration_half_box) ||
            request.acceleration_half_box <= 0.0 ||
            !std::isfinite(request.collision_distance_m) ||
            request.collision_distance_m < 0.0 ||
            !std::isfinite(request.position_reserve_m) ||
            request.position_reserve_m < 0.0 ||
            !std::isfinite(request.velocity_reserve_mps) ||
            request.velocity_reserve_mps < 0.0) {
            throw std::invalid_argument("invalid frontier allocation request");
        }
        std::set<NodeId> ids;
        for (const auto& agent : request.agents) {
            if (agent.id <= 0 || !agent.position.allFinite() ||
                !agent.velocity.allFinite() || !ids.insert(agent.id).second)
                throw std::invalid_argument("invalid frontier agent state");
        }
        for (const auto& cell : request.cells)
            if (!cell.center.allFinite())
                throw std::invalid_argument("invalid frontier cell");
    }

    SharedFrontierAllocatorConfig config_;
    std::size_t priority_epoch_ = 0;
    std::map<std::string,std::size_t> ages_;
    std::set<std::string> no_goods_;
    std::string last_bundle_key_;
};

}  // namespace gf
