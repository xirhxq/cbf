#pragma once

#include "Swarm.hpp"
#include "grand_finale/FullStateCheckpoint.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/RouteBClosedLoopAudit.hpp"
#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"
#include "grand_finale/RouteBPolarEntry.hpp"
#include "grand_finale/SearchPolygonDomain.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"
#include "models/DoubleIntegrate2D.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gf {

struct RouteBConvexDomainSpec {
    std::string name;
    std::string description;
    std::vector<Eigen::Vector2d> vertices;
    RouteBTriangulation initial_tri;
    RouteBTriangulation target_tri;
    bool use_direct_matched = false;
};

struct RouteBMissionConfig {
    std::string domain_name = "narrow_deep_offshore";
    std::optional<RouteBConvexDomainSpec> custom_domain;
    double cell_size_m = 10.0;
    double sensor_radius_m = 400.0;
    bool dynamic_chain_sweep = false;
    double search_speed_cap_mps = 0.0;
    bool record_trajectory_samples = false;
    double trajectory_sample_interval_s = 10.0;
    bool record_mission_snapshots = false;
    std::vector<double> snapshot_times_s;
    bool skip_stay_arm = false;
    bool skip_switch_arm = false;
    double dt_s = 0.1;
    double t_fork_s = 15.0;
    double max_mission_s = 1350.0;
    double patrol_radius_m = 105.0;
    double v_cruise_mps = 15.0;
    double accel_mps2 = 2.0;
    double a_max_mps2 = 4.0;
    double max_yaw_rate_radps = 1.0;
    double d_safe_m = 10.0;
    double d_dep_m = 850.0;
    double d_pipe_m = 60.0;
    double d_clear_m = 32.0;
    double t_settle_s = 1.0;
    double uwb_d_r_m = 850.0;
    double uwb_sigma0_m = 0.5;
    double uwb_d_max_m = 1200.0;
    unsigned int link_seed = 20261123;
    double maximum_range_aoi_s = 3.5;
    double curve_sample_interval_s = 5.0;
};

struct RouteBCoverageSample {
    double time_s = 0.0;
    double truth_coverage = 0.0;
    double certified_coverage = 0.0;
};

struct RouteBMissionSnapshot {
    double time_s = 0.0;
    std::size_t tick = 0;
    std::string stage;
    double truth_coverage = 0.0;
    double certified_coverage = 0.0;
    int certified_count = 0;
    std::map<NodeId, Eigen::Vector2d> positions;
    std::map<NodeId, Eigen::Vector2d> beacons;
    std::vector<std::pair<NodeId, NodeId>> active_edges;
    std::vector<std::pair<int, int>> certified_rle;
};

struct RouteBGridMetadata {
    double x_min_m = 0.0;
    double x_max_m = 0.0;
    double y_min_m = 0.0;
    double y_max_m = 0.0;
    int x_num = 0;
    int y_num = 0;
    double cell_size_m = 0.0;
    int valid_cells = 0;
    int bbox_ix_min = 0;
    int bbox_iy_min = 0;
    int bbox_ix_max = -1;
    int bbox_iy_max = -1;
    std::vector<std::pair<int, int>> valid_rle;
};

struct RouteBMissionArmResult {
    std::string arm_name;
    bool reconfig_completed = true;
    std::string status_reason;
    double reconfig_entry_s = 0.0;
    double reconfig_complete_s = 0.0;
    double final_time_s = 0.0;
    double wall_clock_s = 0.0;
    bool reached_t95 = false;
    bool reached_t100 = false;
    std::optional<double> t95_certified_s;
    std::optional<double> t100_certified_s;
    double final_truth_coverage = 0.0;
    double final_certified_coverage = 0.0;
    int false_certified_count = 0;
    bool certified_subset_of_truth = true;
    std::size_t static_parent_violations = 0;
    std::size_t dag_violations = 0;
    std::size_t total_mbb_swaps = 0;
    RouteBClosedLoopAuditSummary audit;
    std::vector<RouteBCoverageSample> coverage_curve;
    std::vector<RouteBTrajectorySample> trajectory_samples;
    std::vector<RouteBMissionSnapshot> mission_snapshots;
};

struct RouteBMissionComparison {
    std::string domain_name;
    int valid_cells = 0;
    double area_m2 = 0.0;
    double fork_time_s = 0.0;
    bool checkpoint_byte_identical = false;
    double fork_truth_coverage = 0.0;
    double fork_certified_coverage = 0.0;
    RouteBGridMetadata grid_metadata;
    RouteBMissionArmResult stay;
    RouteBMissionArmResult switch_arm;
};

/**
 * Computes bounded velocity-aligned yaw rate with speed and angular deadbands
 * to eliminate constant spinning and stationary heading chatter.
 */
inline double computeVelocityAlignedYawRate(
    const Eigen::Vector2d& vel,
    double yaw_rad,
    double max_yaw_rate_radps,
    double speed_deadband_mps = 2.0,
    double yaw_gain = 1.8,
    double angle_deadband_rad = 0.05) {
    if (!std::isfinite(vel.x()) || !std::isfinite(vel.y()) ||
        !std::isfinite(yaw_rad) || !std::isfinite(max_yaw_rate_radps) ||
        max_yaw_rate_radps <= 0.0 || vel.norm() < speed_deadband_mps) {
        return 0.0;
    }
    const double desired = std::atan2(vel.y(), vel.x());
    const double err = wrapDoubleIntegratorYawRad(desired - yaw_rad);
    if (std::abs(err) < angle_deadband_rad) {
        return 0.0;
    }
    return std::clamp(yaw_gain * err, -max_yaw_rate_radps, max_yaw_rate_radps);
}

inline std::vector<RouteBConvexDomainSpec> standardRouteBConvexDomains() {
    std::vector<RouteBConvexDomainSpec> domains;

    {
        RouteBConvexDomainSpec d;
        d.name = "wide_shallow_coastal";
        d.description = "Wide shallow coastal convex polygon favoring parallel multi-interval coverage";
        d.vertices = {{0.0, 220.0}, {1600.0, 220.0}, {1480.0, 760.0}, {120.0, 760.0}};
        const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0, 1600.0};
        d.initial_tri = buildStaggeredTriangulation(
            b_xs, {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}, 320.0, 80.0, 100.0, 140.0);
        d.target_tri = buildStaggeredTriangulation(
            b_xs, {{1, 2, 3, 4}, {5, 6}, {7, 8}, {9, 10, 11, 12}}, 320.0, 80.0, 100.0, 140.0);
        d.use_direct_matched = false;
        domains.push_back(std::move(d));
    }

    {
        RouteBConvexDomainSpec d;
        d.name = "narrow_deep_offshore";
        d.description = "Narrow deep offshore convex polygon triggering deep-reach merging";
        d.vertices = {{380.0, 250.0}, {820.0, 250.0}, {760.0, 1550.0}, {440.0, 1550.0}};
        const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0};
        d.initial_tri = buildStaggeredTriangulation(
            b_xs, {{1, 2, 3, 4, 5, 6, 7}, {}, {8, 9, 10, 11, 12, 13, 14}},
            380.0, 90.0, 100.0, 140.0);
        d.target_tri = buildStaggeredTriangulation(
            b_xs, {{}, {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {}},
            380.0, 90.0, 100.0, 140.0);
        d.use_direct_matched = true;
        domains.push_back(std::move(d));
    }

    {
        RouteBConvexDomainSpec d;
        d.name = "oblique_asymmetric";
        d.description = "Oblique asymmetric convex pentagon skewed across offshore intervals";
        d.vertices = {
            {100.0, 240.0}, {920.0, 260.0}, {1420.0, 880.0},
            {960.0, 1280.0}, {320.0, 920.0}};
        d.initial_tri = buildPinballTriangulation(3, 400.0, 100.0, 140.0);
        d.target_tri = buildStaggeredTriangulation(
            {0.0, 400.0, 800.0, 1200.0}, {{}, {1, 2, 3}, {4, 5, 6}},
            380.0, 90.0, 100.0, 140.0);
        d.use_direct_matched = false;
        domains.push_back(std::move(d));
    }

    return domains;
}

inline RouteBConvexDomainSpec findRouteBDomain(const std::string& name) {
    for (const auto& d : standardRouteBConvexDomains()) {
        if (d.name == name) return d;
    }
    throw std::invalid_argument("unknown Route B convex domain: " + name);
}

namespace route_b_mission_detail {

struct UncoveredCell {
    int index = -1;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
};

/**
 * Computes bounded target-aiming yaw rate with distance and angular deadbands
 * so approaching an aim cell never triggers the 1/r atan2 spin singularity.
 */
inline double computeTargetAlignedYawRate(
    const Eigen::Vector2d& aim_vec,
    double yaw_rad,
    double max_yaw_rate_radps,
    double dist_deadband_m = 5.0,
    double yaw_gain = 1.8,
    double angle_deadband_rad = 0.05) {
    if (!std::isfinite(aim_vec.x()) || !std::isfinite(aim_vec.y()) ||
        !std::isfinite(yaw_rad) || !std::isfinite(max_yaw_rate_radps) ||
        max_yaw_rate_radps <= 0.0 || aim_vec.norm() < dist_deadband_m) {
        return 0.0;
    }
    const double desired = std::atan2(aim_vec.y(), aim_vec.x());
    const double err = wrapDoubleIntegratorYawRad(desired - yaw_rad);
    if (std::abs(err) < angle_deadband_rad) {
        return 0.0;
    }
    return std::clamp(yaw_gain * err, -max_yaw_rate_radps, max_yaw_rate_radps);
}

inline std::vector<std::pair<int, int>> encodeGridMaskRle(
    const GridWorld& grid,
    bool certified_only) {
    std::vector<std::pair<int, int>> rle;
    const int total = grid.xNum * grid.yNum;
    int run_start = -1;
    for (int idx = 0; idx < total; ++idx) {
        const std::size_t uidx = static_cast<std::size_t>(idx);
        const bool bit = certified_only
            ? (grid.valid[uidx] && grid.vis[uidx])
            : grid.valid[uidx];
        if (bit) {
            if (run_start < 0) run_start = idx;
        } else if (run_start >= 0) {
            rle.emplace_back(run_start, idx - run_start);
            run_start = -1;
        }
    }
    if (run_start >= 0) {
        rle.emplace_back(run_start, total - run_start);
    }
    return rle;
}

class FrontierSweepPolicy {
public:
    FrontierSweepPolicy(
        const RouteBTriangulation& active_tri,
        double patrol_radius_m,
        double kp,
        double kd,
        double a_max_mps2,
        double max_yaw_rate_radps,
        double sensor_radius_m = 400.0,
        bool dynamic_chain_sweep = false,
        double search_speed_cap_mps = 0.0,
        double d_dep_m = 850.0)
        : mobile_ids_(active_tri.mobile_ids),
          slots_(active_tri.positions),
          beacons_(active_tri.beacons),
          parents_(active_tri.parents),
          patrol_radius_m_(patrol_radius_m),
          kp_(kp),
          kd_(kd),
          a_max_(a_max_mps2),
          max_yaw_rate_(max_yaw_rate_radps),
          sensor_radius_(sensor_radius_m),
          dynamic_chain_sweep_(dynamic_chain_sweep),
          search_speed_cap_mps_(search_speed_cap_mps),
          d_dep_m_(d_dep_m) {
        std::map<NodeId, int> hop;
        std::map<NodeId, Eigen::Vector2d> root_pt;
        for (const auto& [bid, bp] : beacons_) {
            hop[bid] = 0;
            root_pt[bid] = bp;
        }
        for (int iter = 0; iter < static_cast<int>(mobile_ids_.size()) + 2; ++iter) {
            for (NodeId id : mobile_ids_) {
                const auto ps = parents_.at(id);
                if (hop.count(ps.first) && hop.count(ps.second)) {
                    const int h1 = hop.at(ps.first);
                    const int h2 = hop.at(ps.second);
                    hop[id] = 1 + std::min(h1, h2);
                    root_pt[id] = (h1 <= h2) ? root_pt.at(ps.first) : root_pt.at(ps.second);
                }
            }
        }
        for (NodeId id : mobile_ids_) {
            beacon_hop_[id] = hop.count(id) ? hop.at(id) : 1;
            root_anchor_[id] = root_pt.count(id) ? root_pt.at(id) : slots_.at(id);
            targets_[id] = slots_.at(id);
            aim_points_[id] = slots_.at(id) + Eigen::Vector2d(0.0, 200.0);
        }
    }

    void computeCommands(
        const Swarm& swarm,
        const GrandFinaleSwarmAdapter& adapter,
        const JointEstimateSnapshot& estimate,
        std::size_t tick,
        std::map<NodeId, Eigen::Vector2d>& out_nominal,
        std::map<NodeId, double>& out_yaw_rates) {
        std::map<NodeId, Eigen::Vector2d> est_pos, est_vel;
        for (std::size_t i = 0; i < mobile_ids_.size(); ++i) {
            const NodeId id = estimate.mobile_ids[i];
            est_pos[id] = estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i));
            est_vel[id] = estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i) + 2);
        }

        if (!targets_initialized_ || tick % 10 == 0) {
            updateTargets(adapter.coverage().certifiedGrid(), est_pos);
            targets_initialized_ = true;
        }

        for (std::size_t i = 0; i < mobile_ids_.size(); ++i) {
            const NodeId id = mobile_ids_[i];
            const Eigen::Vector2d& p = est_pos.at(id);
            const Eigen::Vector2d& v = est_vel.at(id);

            Eigen::Vector2d w = targets_.at(id);
            if (!dynamic_chain_sweep_) {
                w = slots_.at(id);
                const Eigen::Vector2d delta = targets_.at(id) - slots_.at(id);
                const double dnorm = delta.norm();
                if (dnorm > 1.0e-6) {
                    w += delta * (std::min(dnorm, patrol_radius_m_) / dnorm);
                }
            }

            if (search_speed_cap_mps_ > 0.0) {
                Eigen::Vector2d v_des = 0.65 * (w - p);
                const double sp = v_des.norm();
                if (sp > search_speed_cap_mps_) {
                    v_des *= (search_speed_cap_mps_ / sp);
                }
                out_nominal[id] = (1.6 * (v_des - v))
                                      .cwiseMax(-a_max_)
                                      .cwiseMin(a_max_);
            } else {
                out_nominal[id] = (kp_ * (w - p) - kd_ * v)
                                      .cwiseMax(-a_max_)
                                      .cwiseMin(a_max_);
            }

            const double yaw = swarm.robots[i]->model->getStateVariable("yawRad");
            const Eigen::Vector2d aim_vec = aim_points_.at(id) - p;
            out_yaw_rates[id] = computeTargetAlignedYawRate(
                aim_vec, yaw, max_yaw_rate_,
                kSearchYawDistDeadbandM, kSearchYawGain, kSearchYawAngleDeadbandRad);
        }
    }

    static constexpr double kLookaheadMinDistM = 30.0;
    static constexpr double kSearchYawDistDeadbandM = 5.0;
    static constexpr double kSearchYawAngleDeadbandRad = 0.05;
    static constexpr double kSearchYawGain = 1.8;

    std::size_t cachedUnsearchedCount() const { return unsearched_indices_.size(); }
    double searchAimDistanceDeadbandM() const { return kSearchYawDistDeadbandM; }
    std::optional<Eigen::Vector2d> aimPoint(NodeId id) const {
        const auto it = aim_points_.find(id);
        if (it == aim_points_.end()) return std::nullopt;
        return it->second;
    }

private:
    bool hasValidUncertifiedAim(NodeId id, const GridWorld& cert_grid) const {
        const auto it = aim_cell_indices_.find(id);
        if (it == aim_cell_indices_.end()) return false;
        const int prev_idx = it->second;
        return prev_idx >= 0 &&
               static_cast<std::size_t>(prev_idx) < cert_grid.vis.size() &&
               cert_grid.valid[static_cast<std::size_t>(prev_idx)] &&
               !cert_grid.vis[static_cast<std::size_t>(prev_idx)];
    }

    void selectAimPointWithHysteresis(
        NodeId id,
        const std::vector<UncoveredCell>& pool,
        const GridWorld& cert_grid,
        const Eigen::Vector2d& cur) {
        if (hasValidUncertifiedAim(id, cert_grid)) {
            return;
        }
        if (pool.empty()) {
            aim_cell_indices_.erase(id);
            return;
        }
        constexpr double kLookaheadMinDist2 = kLookaheadMinDistM * kLookaheadMinDistM;
        const UncoveredCell* best_far = nullptr;
        double best_far_d2 = std::numeric_limits<double>::infinity();
        const UncoveredCell* best_near = nullptr;
        double best_near_d2 = std::numeric_limits<double>::infinity();
        for (const auto& cell : pool) {
            const double d2 = (cell.center - cur).squaredNorm();
            if (d2 >= kLookaheadMinDist2) {
                if (d2 < best_far_d2) {
                    best_far_d2 = d2;
                    best_far = &cell;
                }
            } else if (d2 < best_near_d2) {
                best_near_d2 = d2;
                best_near = &cell;
            }
        }
        const UncoveredCell& chosen = (best_far != nullptr) ? *best_far : *best_near;
        aim_cell_indices_[id] = chosen.index;
        aim_points_[id] = chosen.center;
    }

    void updateTargets(
        const GridWorld& cert_grid,
        const std::map<NodeId, Eigen::Vector2d>& est_pos) {
        std::vector<UncoveredCell> uncovered;
        if (!unsearched_initialized_) {
            const double x0 = cert_grid.xLim.first;
            const double x1 = cert_grid.xLim.second;
            const double y0 = cert_grid.yLim.first;
            const double y1 = cert_grid.yLim.second;
            unsearched_indices_.reserve(
                static_cast<std::size_t>(std::max(0, cert_grid.validCount)));
            unsearched_centers_.reserve(
                static_cast<std::size_t>(std::max(0, cert_grid.validCount)));
            for (int ix = 0; ix < cert_grid.xNum; ++ix) {
                const double rx = (static_cast<double>(ix) + 0.5) / static_cast<double>(cert_grid.xNum);
                const double cx = x0 * (1.0 - rx) + x1 * rx;
                for (int iy = 0; iy < cert_grid.yNum; ++iy) {
                    const int idx = cert_grid.getIndex(ix, iy);
                    if (cert_grid.valid[idx] && !cert_grid.vis[idx]) {
                        const double ry = (static_cast<double>(iy) + 0.5) / static_cast<double>(cert_grid.yNum);
                        unsearched_indices_.push_back(idx);
                        unsearched_centers_.emplace_back(cx, y0 * (1.0 - ry) + y1 * ry);
                        uncovered.push_back({idx, unsearched_centers_.back()});
                    }
                }
            }
            unsearched_initialized_ = true;
        } else {
            std::size_t write = 0;
            uncovered.reserve(unsearched_indices_.size());
            for (std::size_t k = 0; k < unsearched_indices_.size(); ++k) {
                const int idx = unsearched_indices_[k];
                if (!cert_grid.vis[static_cast<std::size_t>(idx)]) {
                    unsearched_indices_[write] = idx;
                    unsearched_centers_[write] = unsearched_centers_[k];
                    uncovered.push_back({idx, unsearched_centers_[k]});
                    ++write;
                }
            }
            unsearched_indices_.resize(write);
            unsearched_centers_.resize(write);
        }
        if (uncovered.empty()) {
            for (NodeId id : mobile_ids_) {
                aim_cell_indices_.erase(id);
                targets_[id] = slots_.at(id);
                aim_points_[id] = slots_.at(id) + Eigen::Vector2d(0.0, 200.0);
            }
            return;
        }

        if (dynamic_chain_sweep_) {
            const double edge_step_m = 0.858 * d_dep_m_;
            auto projectOntoTwoDisks = [](const Eigen::Vector2d& w,
                                          const Eigen::Vector2d& c1,
                                          const Eigen::Vector2d& c2,
                                          double R) -> Eigen::Vector2d {
                auto clampDisk = [&](const Eigen::Vector2d& p, const Eigen::Vector2d& c) -> Eigen::Vector2d {
                    const Eigen::Vector2d d = p - c;
                    const double n = d.norm();
                    if (n <= R || n < 1.0e-9) return p;
                    return c + d * (R / n);
                };
                if ((w - c1).norm() <= R && (w - c2).norm() <= R) {
                    return w;
                }
                const Eigen::Vector2d p1 = clampDisk(w, c1);
                if ((p1 - c2).norm() <= R + 1.0e-6) return p1;
                const Eigen::Vector2d p2 = clampDisk(w, c2);
                if ((p2 - c1).norm() <= R + 1.0e-6) return p2;

                const Eigen::Vector2d dc = c2 - c1;
                const double dist = dc.norm();
                if (dist < 1.0e-9) return p1;
                if (dist >= 2.0 * R - 1.0e-6) {
                    return 0.5 * (c1 + c2);
                }
                const double a = 0.5 * dist;
                const double h = std::sqrt(std::max(0.0, R * R - a * a));
                const Eigen::Vector2d u = dc / dist;
                const Eigen::Vector2d perp(-u.y(), u.x());
                const Eigen::Vector2d mid = 0.5 * (c1 + c2);
                const Eigen::Vector2d v1 = mid + h * perp;
                const Eigen::Vector2d v2 = mid - h * perp;
                return ((w - v1).squaredNorm() <= (w - v2).squaredNorm()) ? v1 : v2;
            };
            // Group sub-chains by nearest root beacon x so each sub-chain sweeps coherently
            std::map<NodeId, double> node_group_x;
            for (NodeId id : mobile_ids_) {
                node_group_x[id] = root_anchor_.at(id).x();
            }
            // Cluster nodes connected in mobile-mobile parent graph so left/right beacons of same root stay one group
            std::map<NodeId, int> comp;
            for (NodeId id : mobile_ids_) comp[id] = static_cast<int>(id);
            for (int iter = 0; iter < static_cast<int>(mobile_ids_.size()); ++iter) {
                for (NodeId id : mobile_ids_) {
                    const auto ps = parents_.at(id);
                    for (NodeId p : {ps.first, ps.second}) {
                        if (beacons_.count(p) == 0) {
                            const int m = std::min(comp[id], comp[p]);
                            comp[id] = m;
                            comp[p] = m;
                        }
                    }
                }
            }
            std::map<int, std::vector<NodeId>> groups;
            std::map<int, Eigen::Vector2d> group_origin;
            for (NodeId id : mobile_ids_) {
                groups[comp[id]].push_back(id);
            }
            for (auto& [gid, gids] : groups) {
                Eigen::Vector2d org = Eigen::Vector2d::Zero();
                for (NodeId id : gids) org += root_anchor_.at(id);
                group_origin[gid] = org / static_cast<double>(gids.size());
            }

            std::map<int, std::vector<UncoveredCell>> group_cells;
            for (const auto& cell : uncovered) {
                int best_g = groups.begin()->first;
                double best_d2 = std::numeric_limits<double>::infinity();
                bool found_feas = false;
                for (const auto& [gid, gids] : groups) {
                    int max_hop = 1;
                    for (NodeId id : gids) max_hop = std::max(max_hop, beacon_hop_.at(id));
                    const double g_reach = static_cast<double>(max_hop) * edge_step_m + sensor_radius_ - 30.0;
                    const double d_org = (cell.center - group_origin.at(gid)).norm();
                    const bool feas = d_org <= g_reach;
                    const double d2 = (cell.center - group_origin.at(gid)).squaredNorm();
                    if (feas && (!found_feas || d2 < best_d2)) {
                        found_feas = true;
                        best_d2 = d2;
                        best_g = gid;
                    } else if (!found_feas && d2 < best_d2) {
                        best_d2 = d2;
                        best_g = gid;
                    }
                }
                group_cells[best_g].push_back(cell);
            }

            for (const auto& [gid, gids] : groups) {
                const auto& gpool = group_cells[gid].empty() ? uncovered : group_cells[gid];
                const Eigen::Vector2d& org = group_origin.at(gid);
                // Sweep along the leftmost remaining angular sector (monotonic bearing sweep)
                double min_ang = std::numeric_limits<double>::infinity();
                for (const auto& c : gpool) {
                    const double ang = std::atan2(c.center.x() - org.x(), std::max(50.0, c.center.y()));
                    min_ang = std::min(min_ang, ang);
                }
                std::vector<UncoveredCell> sector_pool;
                const double ang_band = 0.28;
                for (const auto& c : gpool) {
                    const double ang = std::atan2(c.center.x() - org.x(), std::max(50.0, c.center.y()));
                    if (ang <= min_ang + ang_band) {
                        sector_pool.push_back(c);
                    }
                }
                if (sector_pool.empty()) sector_pool = gpool;

                std::vector<NodeId> hop_sorted = gids;
                std::sort(hop_sorted.begin(), hop_sorted.end(), [&](NodeId a, NodeId b) {
                    if (beacon_hop_.at(a) != beacon_hop_.at(b)) return beacon_hop_.at(a) < beacon_hop_.at(b);
                    return a < b;
                });
                std::map<NodeId, double> node_side_sign;
                std::map<int, int> hop_seen_count;
                for (NodeId id : hop_sorted) {
                    const int h = beacon_hop_.at(id);
                    const int k = hop_seen_count[h]++;
                    node_side_sign[id] = (k % 2 == 0) ? -1.0 : 1.0;
                }
                double sum_ang = 0.0;
                for (const auto& c : sector_pool) {
                    sum_ang += std::atan2(c.center.x() - org.x(), std::max(50.0, c.center.y()));
                }
                const double mean_ang = sector_pool.empty()
                    ? min_ang + 0.12
                    : (sum_ang / static_cast<double>(sector_pool.size()));

                std::map<NodeId, std::vector<UncoveredCell>> band_cells;
                for (const auto& c : sector_pool) {
                    const double r = (c.center - org).norm();
                    const double ang = std::atan2(c.center.x() - org.x(), std::max(50.0, c.center.y()));
                    NodeId best_id = hop_sorted.front();
                    double best_score = std::numeric_limits<double>::infinity();
                    for (NodeId id : hop_sorted) {
                        const double r_min = static_cast<double>(beacon_hop_.at(id) - 1) * edge_step_m;
                        const double r_max = static_cast<double>(beacon_hop_.at(id)) * edge_step_m + sensor_radius_ - 30.0;
                        const double r_mid = 0.5 * (r_min + r_max);
                        const double lat_pref = 180.0 * std::abs(ang - (mean_ang + node_side_sign[id] * 0.08));
                        const double pen = (r <= r_max)
                            ? (std::abs(r - r_mid) + lat_pref)
                            : (1.0e5 + r - r_max + lat_pref);
                        if (pen < best_score) {
                            best_score = pen;
                            best_id = id;
                        }
                    }
                    band_cells[best_id].push_back(c);
                }

                // Choose bridge/nearest uncovered fallback so parents extend smoothly along chain without overshooting polygon
                std::map<NodeId, std::vector<NodeId>> group_children;
                for (NodeId id : gids) {
                    const auto ps = parents_.at(id);
                    if (beacons_.count(ps.first) == 0) group_children[ps.first].push_back(id);
                    if (beacons_.count(ps.second) == 0) group_children[ps.second].push_back(id);
                }
                const Eigen::Vector2d sector_dir(std::sin(min_ang + 0.10), std::cos(min_ang + 0.10));
                const Eigen::Vector2d sector_perp(-sector_dir.y(), sector_dir.x());
                for (auto it = hop_sorted.rbegin(); it != hop_sorted.rend(); ++it) {
                    const NodeId id = *it;
                    const Eigen::Vector2d& cur = est_pos.at(id);
                    const double max_r = static_cast<double>(beacon_hop_.at(id)) * edge_step_m;
                    if (!band_cells[id].empty()) {
                        const auto& pool = band_cells[id];
                        Eigen::Vector2d best_pt = pool.front().center;
                        double best_d = std::numeric_limits<double>::infinity();
                        for (const auto& c : pool) {
                            const double d = (c.center - cur).norm();
                            if (d < best_d) {
                                best_d = d;
                                best_pt = c.center;
                            }
                        }
                        selectAimPointWithHysteresis(id, pool, cert_grid, cur);
                        const Eigen::Vector2d diff = best_pt - cur;
                        const double dist = diff.norm();
                        const double standoff = std::max(0.0, sensor_radius_ - 170.0);
                        Eigen::Vector2d w = (dist <= standoff)
                            ? best_pt
                            : (cur + diff * ((dist - standoff) / dist)).eval();
                        const Eigen::Vector2d off = w - root_anchor_.at(id);
                        if (off.norm() > max_r && off.norm() > 1.0e-6) {
                            w = root_anchor_.at(id) + off * (max_r / off.norm());
                        }
                        targets_[id] = w;
                    } else {
                        // Pick nearest reachable cell in sector_pool/gpool or bridge toward active child
                        Eigen::Vector2d desired = cur;
                        bool have_child_target = false;
                        Eigen::Vector2d child_avg = Eigen::Vector2d::Zero();
                        int child_cnt = 0;
                        int child_max_hop = beacon_hop_.at(id) + 1;
                        for (NodeId cid : group_children[id]) {
                            if (!band_cells[cid].empty() || (targets_.at(cid) - root_anchor_.at(cid)).norm() > 1.0) {
                                child_avg += targets_.at(cid);
                                child_max_hop = std::max(child_max_hop, beacon_hop_.at(cid));
                                ++child_cnt;
                                have_child_target = true;
                            }
                        }
                        if (have_child_target) {
                            child_avg /= static_cast<double>(child_cnt);
                            const Eigen::Vector2d span_vec = child_avg - root_anchor_.at(id);
                            const double span_len = span_vec.norm();
                            const double frac = static_cast<double>(beacon_hop_.at(id)) /
                                                static_cast<double>(std::max(beacon_hop_.at(id) + 1, child_max_hop));
                            const double want_r = std::min(max_r, std::max(0.76 * span_len, frac * span_len));
                            desired = (span_len > 1.0e-6)
                                ? (root_anchor_.at(id) + span_vec * (want_r / span_len)).eval()
                                : cur;
                            if (!hasValidUncertifiedAim(id, cert_grid)) {
                                aim_cell_indices_.erase(id);
                                aim_points_[id] = child_avg;
                            }
                        } else {
                            const auto& pool = sector_pool.empty() ? gpool : sector_pool;
                            Eigen::Vector2d best_pt = pool.front().center;
                            double best_d = std::numeric_limits<double>::infinity();
                            for (const auto& c : pool) {
                                const double d = (c.center - cur).norm();
                                if (d < best_d) {
                                    best_d = d;
                                    best_pt = c.center;
                                }
                            }
                            desired = best_pt;
                            selectAimPointWithHysteresis(id, pool, cert_grid, cur);
                        }
                        const double parity_shift = (id % 2 == 0) ? 45.0 : -45.0;
                        desired += parity_shift * sector_perp;
                        desired.y() = std::max(desired.y(), 160.0);
                        const Eigen::Vector2d off = desired - root_anchor_.at(id);
                        if (off.norm() > max_r && off.norm() > 1.0e-6) {
                            desired = root_anchor_.at(id) + off * (max_r / off.norm());
                        }
                        if (beacon_hop_.at(id) == 1) {
                            const auto ps = parents_.at(id);
                            for (NodeId p : {ps.first, ps.second}) {
                                if (beacons_.count(p) != 0) {
                                    const Eigen::Vector2d boff = desired - beacons_.at(p);
                                    if (boff.norm() > edge_step_m && boff.norm() > 1.0e-6) {
                                        desired = beacons_.at(p) + boff * (edge_step_m / boff.norm());
                                    }
                                }
                            }
                            desired.y() = std::max(desired.y(), 160.0);
                        }
                        targets_[id] = desired;
                    }
                }
            }

            // Enforce parent-edge feasibility top-down in topological DAG order
            std::vector<NodeId> topo_order;
            topo_order.reserve(mobile_ids_.size());
            std::map<NodeId, bool> placed;
            while (topo_order.size() < mobile_ids_.size()) {
                bool progressed = false;
                for (NodeId id : mobile_ids_) {
                    if (placed[id]) continue;
                    const auto ps = parents_.at(id);
                    const bool p1_ok = (beacons_.count(ps.first) != 0) || placed[ps.first];
                    const bool p2_ok = (beacons_.count(ps.second) != 0) || placed[ps.second];
                    if (p1_ok && p2_ok) {
                        topo_order.push_back(id);
                        placed[id] = true;
                        progressed = true;
                    }
                }
                if (!progressed) {
                    for (NodeId id : mobile_ids_) {
                        if (!placed[id]) topo_order.push_back(id);
                    }
                    break;
                }
            }

            for (std::size_t ti = 0; ti < topo_order.size(); ++ti) {
                const NodeId id = topo_order[ti];
                const auto ps = parents_.at(id);
                const Eigen::Vector2d c1 = beacons_.count(ps.first) ? beacons_.at(ps.first) : targets_.at(ps.first);
                const Eigen::Vector2d c2 = beacons_.count(ps.second) ? beacons_.at(ps.second) : targets_.at(ps.second);
                Eigen::Vector2d q = targets_.at(id);
                q.y() = std::max(q.y(), 120.0);
                q = projectOntoTwoDisks(q, c1, c2, edge_step_m);
                // Keep non-degenerate triangular baseline (>= 115 m) away from both parent anchors
                const Eigen::Vector2d dc = c2 - c1;
                const Eigen::Vector2d perp_dc = (dc.norm() > 1.0e-6)
                    ? Eigen::Vector2d(-dc.y() / dc.norm(), dc.x() / dc.norm())
                    : Eigen::Vector2d(0.0, 1.0);
                for (int pass = 0; pass < 2; ++pass) {
                    for (const Eigen::Vector2d& pc : {c1, c2}) {
                        const Eigen::Vector2d diff = q - pc;
                        const double d = diff.norm();
                        if (d < 115.0) {
                            const Eigen::Vector2d dir = (d > 1.0e-6) ? (diff / d).eval() : perp_dc;
                            q = projectOntoTwoDisks(pc + dir * 120.0, c1, c2, edge_step_m);
                        }
                    }
                }
                for (std::size_t tj = 0; tj < ti; ++tj) {
                    const NodeId other = topo_order[tj];
                    const Eigen::Vector2d diff = q - targets_.at(other);
                    const double sep = diff.norm();
                    const double min_sep = (beacon_hop_.at(other) == beacon_hop_.at(id) &&
                                            (root_anchor_.at(other) - root_anchor_.at(id)).norm() < 1.0)
                        ? 120.0
                        : 45.0;
                    if (sep < min_sep) {
                        const Eigen::Vector2d dir = (sep > 1.0e-6)
                            ? (diff / sep).eval()
                            : perp_dc;
                        q = projectOntoTwoDisks(targets_.at(other) + dir * (min_sep + 5.0), c1, c2, edge_step_m);
                    }
                }
                q.y() = std::max(q.y(), 120.0);
                q = projectOntoTwoDisks(q, c1, c2, edge_step_m);
                const double alt = (q - c1).dot(perp_dc);
                if (dc.norm() > 1.0e-6 && std::abs(alt) < 120.0) {
                    const double sign = (std::abs(alt) > 15.0)
                        ? (alt >= 0.0 ? 1.0 : -1.0)
                        : (perp_dc.y() >= 0.0 ? 1.0 : -1.0);
                    Eigen::Vector2d q_lifted = q + perp_dc * (sign * 125.0 - alt);
                    if (q_lifted.y() < 120.0) {
                        q_lifted = q + perp_dc * (-sign * 125.0 - alt);
                    }
                    q_lifted.y() = std::max(q_lifted.y(), 120.0);
                    q = projectOntoTwoDisks(q_lifted, c1, c2, edge_step_m);
                }
                targets_[id] = q;
            }
            return;
        }

        // Voronoi assignment by slot position so each UAV sweeps its local neighborhood first
        std::map<NodeId, std::vector<UncoveredCell>> owned;
        for (const auto& cell : uncovered) {
            NodeId best_id = mobile_ids_.front();
            double best_d2 = std::numeric_limits<double>::infinity();
            for (NodeId id : mobile_ids_) {
                const double d2 = (cell.center - slots_.at(id)).squaredNorm();
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_id = id;
                }
            }
            owned[best_id].push_back(cell);
        }

        for (NodeId id : mobile_ids_) {
            const auto& pool = owned[id].empty() ? uncovered : owned[id];
            // Pick furthest reachable inside (patrol_radius + sensor_radius), else closest uncovered
            const Eigen::Vector2d& slot = slots_.at(id);
            const Eigen::Vector2d& cur = est_pos.at(id);
            const double reach_limit = patrol_radius_m_ + sensor_radius_;
            Eigen::Vector2d best_pt = pool.front().center;
            double best_score = std::numeric_limits<double>::infinity();
            bool found_reachable = false;
            for (const auto& c : pool) {
                const double d_slot = (c.center - slot).norm();
                const double d_cur = (c.center - cur).norm();
                if (d_slot <= reach_limit) {
                    if (!found_reachable || d_cur < best_score) {
                        found_reachable = true;
                        best_score = d_cur;
                        best_pt = c.center;
                    }
                } else if (!found_reachable && d_cur < best_score) {
                    best_score = d_cur;
                    best_pt = c.center;
                }
            }
            selectAimPointWithHysteresis(id, pool, cert_grid, cur);
            // Stand off slightly so sensor footprint comfortably covers cell center
            const Eigen::Vector2d diff = best_pt - slot;
            const double dist = diff.norm();
            if (dist <= sensor_radius_ - 60.0) {
                targets_[id] = best_pt;
            } else {
                const double want = std::min(patrol_radius_m_, std::max(0.0, dist - (sensor_radius_ - 70.0)));
                targets_[id] = slot + (dist > 1.0e-6 ? (diff * (want / dist)).eval() : Eigen::Vector2d::Zero());
            }
        }
    }

    std::vector<NodeId> mobile_ids_;
    std::map<NodeId, Eigen::Vector2d> slots_;
    std::map<NodeId, Eigen::Vector2d> beacons_;
    std::map<NodeId, std::pair<NodeId, NodeId>> parents_;
    std::map<NodeId, int> beacon_hop_;
    std::map<NodeId, Eigen::Vector2d> root_anchor_;
    double patrol_radius_m_ = 105.0;
    double kp_ = 0.8;
    double kd_ = 1.8;
    double a_max_ = 4.0;
    double max_yaw_rate_ = 1.0;
    double sensor_radius_ = 400.0;
    bool dynamic_chain_sweep_ = false;
    double search_speed_cap_mps_ = 0.0;
    double d_dep_m_ = 850.0;
    bool targets_initialized_ = false;
    bool unsearched_initialized_ = false;
    std::vector<int> unsearched_indices_;
    std::vector<Eigen::Vector2d> unsearched_centers_;
    std::map<NodeId, Eigen::Vector2d> targets_;
    std::map<NodeId, Eigen::Vector2d> aim_points_;
    std::map<NodeId, int> aim_cell_indices_;
};

}  // namespace route_b_mission_detail

inline RouteBMissionComparison runTwoRegimeMissionComparison(
    const RouteBMissionConfig& mcfg) {
    const auto dom = mcfg.custom_domain.has_value()
        ? *mcfg.custom_domain
        : findRouteBDomain(mcfg.domain_name);

    auto make_scenario_settings = [&](json& out_settings,
                                      GrandFinaleSwarmAdapterConfig& out_acfg,
                                      Task10p10Scenario& out_scen) {
        out_scen.id = "route_b_mission_" + dom.name;
        out_scen.width_m = 1600.0;
        out_scen.height_m = 1600.0;
        for (const auto& v : dom.vertices) {
            out_scen.width_m = std::max(out_scen.width_m, v.x() + 200.0);
            out_scen.height_m = std::max(out_scen.height_m, v.y() + 200.0);
        }
        out_scen.mobile_ids = dom.initial_tri.mobile_ids;
        for (NodeId id : out_scen.mobile_ids) {
            out_scen.mobile_positions.push_back(dom.initial_tri.positions.at(id));
        }
        out_scen.fixed_positions = dom.initial_tri.beacons;
        out_scen.initial_topology = dom.initial_tri.initialTopologyEdges();

        out_settings = task10p10SwarmSettings(out_scen, SolverProfile::OpenSource);
        std::vector<int> base_indices;
        for (std::size_t i = 0; i < dom.initial_tri.beacon_ids.size(); ++i) {
            base_indices.push_back(static_cast<int>(i));
        }
        out_settings["formation"]["bases-id"] = {base_indices};
        out_settings["execute"]["time-total"] = mcfg.max_mission_s + 50.0;
        out_settings["execute"]["time-step"] = mcfg.dt_s;
        out_settings["searching"]["front-sector"]["outer-radius"] = mcfg.sensor_radius_m;

        json site = {
            {"schema", "gf-search-polygon-v1"},
            {"id", dom.name},
            {"vertices_m", json::array()},
            {"cell_size_m", mcfg.cell_size_m}
        };
        for (const auto& v : dom.vertices) {
            site["vertices_m"].push_back({v.x(), v.y()});
        }
        if (std::abs(mcfg.cell_size_m - 10.0) <= 1.0e-9) {
            out_settings = searchPolygonSettings(out_settings, site);
        } else {
            if (dom.vertices.size() < 3) {
                throw std::invalid_argument("search polygon needs at least three vertices");
            }
            out_settings["world"]["spacing"] = mcfg.cell_size_m;
            out_settings["world"]["boundary"] = site.at("vertices_m");
            out_settings["world"]["cell-domain"] = "closed-polygon-centers-v1";
        }

        out_acfg.solver_profile = SolverProfile::OpenSource;
        out_acfg.dt_s = mcfg.dt_s;
        out_acfg.minimum_dwell_s = mcfg.dt_s;
        out_acfg.acceleration_half_box = mcfg.a_max_mps2;
        out_acfg.collision_distance_m = mcfg.d_safe_m;
        out_acfg.reference_distance_m = mcfg.d_dep_m;
        out_acfg.add_reference_distance_m = mcfg.d_dep_m - 1.0;
        out_acfg.reference_uncertainty_m = 0.0;
        out_acfg.uncertainty_sigma = 3.0;
        out_acfg.uncertainty_scale_mode = UncertaintyScaleMode::TraceRoot;
        out_acfg.axis_velocity_limit_mps = 25.0;
        out_acfg.axis_velocity_cbf_gain = 5.0;
        out_acfg.estimator_acceleration_variance = 0.25 * 0.25;
        out_acfg.initial_position_variance_m2 = 0.04;
        out_acfg.initial_velocity_variance_m2 = 0.01;
        out_acfg.maximum_range_aoi_s = mcfg.maximum_range_aoi_s;
        out_acfg.normalized_range_innovation_gate_sigma = 4.0;
        out_acfg.minimal_runtime_gating = true;
        out_acfg.tau_margin_gate_enabled = true;
        out_acfg.tau_analytic_first_order = true;
        out_acfg.cbf2026_uwb_channel.enabled = true;
        out_acfg.cbf2026_uwb_channel.d_r_m = mcfg.uwb_d_r_m;
        out_acfg.cbf2026_uwb_channel.sigma0_m = mcfg.uwb_sigma0_m;
        out_acfg.cbf2026_uwb_channel.d_max_m = mcfg.uwb_d_max_m;
        out_acfg.cbf2026_uwb_channel.link_seed = mcfg.link_seed;
        out_acfg.sensor_radius_m = mcfg.sensor_radius_m;
        out_acfg.coverage_footprint_kind = CoverageFootprintKind::ForwardSector;
        out_acfg.coverage_half_angle_rad = M_PI / 3.0;
        out_acfg.maximum_yaw_rate_radps = mcfg.max_yaw_rate_radps;
    };

    json base_settings;
    GrandFinaleSwarmAdapterConfig acfg;
    Task10p10Scenario scen;
    try {
        make_scenario_settings(base_settings, acfg, scen);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("make_scenario_settings: ") + e.what());
    }

    // 1. Common moving-search prefix up to t_fork_s
    json settings_prefix = base_settings;
    Swarm swarm_prefix(settings_prefix);
    GrandFinaleSwarmAdapter adapter_prefix(
        swarm_prefix, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
    try {
        if (!adapter_prefix.initializeStageZero().initialized) {
            throw std::runtime_error("Route B mission prefix stage-zero failed");
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("prefix_stage0: ") + e.what());
    }

    route_b_mission_detail::FrontierSweepPolicy prefix_sweep(
        dom.initial_tri, mcfg.patrol_radius_m, 0.8, 1.8, mcfg.a_max_mps2,
        mcfg.max_yaw_rate_radps, acfg.sensor_radius_m,
        mcfg.dynamic_chain_sweep, mcfg.search_speed_cap_mps, mcfg.d_dep_m);

    const std::size_t fork_ticks = static_cast<std::size_t>(std::llround(mcfg.t_fork_s / mcfg.dt_s));
    std::vector<RouteBCoverageSample> prefix_curve;
    prefix_curve.push_back({0.0, adapter_prefix.coverage().truthFraction(),
                            adapter_prefix.coverage().certifiedFraction()});
    std::vector<RouteBTrajectorySample> prefix_traj;
    std::vector<RouteBMissionSnapshot> prefix_snapshots;
    auto capture_traj = [&](const Swarm& sw, double t_s, std::vector<RouteBTrajectorySample>& dest) {
        RouteBTrajectorySample ts;
        ts.time_s = t_s;
        for (std::size_t i = 0; i < scen.mobile_ids.size(); ++i) {
            const Point pt = sw.robots[i]->model->xy();
            ts.positions[scen.mobile_ids[i]] = Eigen::Vector2d(pt.x, pt.y);
        }
        dest.push_back(std::move(ts));
    };
    auto should_capture_snapshot = [&](double t_s, bool force_event) -> bool {
        if (!mcfg.record_mission_snapshots) return false;
        if (force_event) return true;
        for (double req_t : mcfg.snapshot_times_s) {
            if (std::abs(t_s - req_t) <= 0.5 * mcfg.dt_s + 1.0e-9) {
                return true;
            }
        }
        return false;
    };
    auto capture_snapshot = [&](const Swarm& sw,
                                const GrandFinaleSwarmAdapter& ad,
                                double t_s,
                                std::size_t tick_idx,
                                const std::string& stg,
                                std::vector<RouteBMissionSnapshot>& dest) {
        if (!dest.empty() && dest.back().tick == tick_idx &&
            std::abs(dest.back().time_s - t_s) <= 1.0e-9) {
            return;
        }
        RouteBMissionSnapshot ms;
        ms.time_s = t_s;
        ms.tick = tick_idx;
        ms.stage = stg;
        ms.truth_coverage = ad.coverage().truthFraction();
        ms.certified_coverage = ad.coverage().certifiedFraction();
        ms.certified_count = ad.coverage().certifiedCoveredCount();
        for (std::size_t i = 0; i < scen.mobile_ids.size(); ++i) {
            const Point pt = sw.robots[i]->model->xy();
            ms.positions[scen.mobile_ids[i]] = Eigen::Vector2d(pt.x, pt.y);
        }
        ms.beacons = scen.fixed_positions;
        for (const auto& edge : ad.supervisor().topology()) {
            ms.active_edges.emplace_back(edge.reference, edge.owner);
        }
        std::sort(ms.active_edges.begin(), ms.active_edges.end());
        ms.certified_rle = route_b_mission_detail::encodeGridMaskRle(
            ad.coverage().certifiedGrid(), true);
        dest.push_back(std::move(ms));
    };
    const std::size_t sample_every = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(mcfg.curve_sample_interval_s / mcfg.dt_s)));
    const std::size_t traj_every = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(mcfg.trajectory_sample_interval_s / mcfg.dt_s)));
    if (mcfg.record_trajectory_samples) {
        capture_traj(swarm_prefix, 0.0, prefix_traj);
    }
    if (should_capture_snapshot(0.0, false)) {
        capture_snapshot(swarm_prefix, adapter_prefix, 0.0, 0, "prefix", prefix_snapshots);
    }

    for (std::size_t tick = 0; tick < fork_ticks; ++tick) {
        const auto snap = adapter_prefix.runtimeSnapshot();
        std::map<NodeId, Eigen::Vector2d> nom;
        std::map<NodeId, double> yaw;
        try {
            prefix_sweep.computeCommands(swarm_prefix, adapter_prefix, snap.estimate, tick, nom, yaw);
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("prefix_sweep: ") + e.what());
        }
        try {
            const auto step = adapter_prefix.stepWithNominalAndYawRates(nom, yaw);
            if (!step.advanced) {
                throw std::runtime_error("Route B prefix step aborted: " + step.reason);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("prefix_step: ") + e.what());
        }
        const double t_s = static_cast<double>(tick + 1) * mcfg.dt_s;
        if ((tick + 1) % sample_every == 0 || tick + 1 == fork_ticks) {
            prefix_curve.push_back({
                t_s,
                adapter_prefix.coverage().truthFraction(),
                adapter_prefix.coverage().certifiedFraction()});
        }
        if (mcfg.record_trajectory_samples &&
            ((tick + 1) % traj_every == 0 || tick + 1 == fork_ticks)) {
            capture_traj(swarm_prefix, t_s, prefix_traj);
        }
        if (should_capture_snapshot(t_s, false)) {
            capture_snapshot(swarm_prefix, adapter_prefix, t_s, tick + 1, "prefix", prefix_snapshots);
        }
    }

    const json identity = {{"domain", dom.name}, {"t_fork_s", mcfg.t_fork_s}};
    json saved_ckpt;
    try {
        saved_ckpt = FullStateCheckpoint::captureRouteB(swarm_prefix, adapter_prefix, identity);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("captureRouteB: ") + e.what());
    }

    // 2. Restore two fresh arms from byte-identical checkpoint
    json settings_stay = base_settings;
    json settings_sw = base_settings;
    Swarm swarm_stay(settings_stay), swarm_sw(settings_sw);
    GrandFinaleSwarmAdapter adapter_stay(
        swarm_stay, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
    GrandFinaleSwarmAdapter adapter_sw(
        swarm_sw, scen.mobile_ids, scen.fixed_positions, scen.initial_topology, acfg);
    try {
        FullStateCheckpoint::restoreRouteB(swarm_stay, adapter_stay, saved_ckpt, identity);
        FullStateCheckpoint::restoreRouteB(swarm_sw, adapter_sw, saved_ckpt, identity);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("restoreRouteB: ") + e.what());
    }

    const std::string dump_stay =
        FullStateCheckpoint::captureRouteB(swarm_stay, adapter_stay, identity).dump();
    const std::string dump_sw =
        FullStateCheckpoint::captureRouteB(swarm_sw, adapter_sw, identity).dump();

    RouteBMissionComparison out;
    out.domain_name = dom.name;
    out.valid_cells = adapter_prefix.coverage().truthGrid().validCount;
    out.area_m2 = static_cast<double>(out.valid_cells) * mcfg.cell_size_m * mcfg.cell_size_m;
    out.fork_time_s = mcfg.t_fork_s;
    out.checkpoint_byte_identical = (dump_stay == dump_sw && dump_stay == saved_ckpt.dump());
    out.fork_truth_coverage = adapter_prefix.coverage().truthFraction();
    out.fork_certified_coverage = adapter_prefix.coverage().certifiedFraction();
    if (mcfg.record_mission_snapshots) {
        const GridWorld& vg = adapter_prefix.coverage().truthGrid();
        out.grid_metadata.x_min_m = vg.xLim.first;
        out.grid_metadata.x_max_m = vg.xLim.second;
        out.grid_metadata.y_min_m = vg.yLim.first;
        out.grid_metadata.y_max_m = vg.yLim.second;
        out.grid_metadata.x_num = vg.xNum;
        out.grid_metadata.y_num = vg.yNum;
        out.grid_metadata.cell_size_m = mcfg.cell_size_m;
        out.grid_metadata.valid_cells = vg.validCount;
        out.grid_metadata.valid_rle = route_b_mission_detail::encodeGridMaskRle(vg, false);
        int ix_min = vg.xNum;
        int iy_min = vg.yNum;
        int ix_max = -1;
        int iy_max = -1;
        for (int ix = 0; ix < vg.xNum; ++ix) {
            for (int iy = 0; iy < vg.yNum; ++iy) {
                if (vg.valid[static_cast<std::size_t>(vg.getIndex(ix, iy))]) {
                    ix_min = std::min(ix_min, ix);
                    iy_min = std::min(iy_min, iy);
                    ix_max = std::max(ix_max, ix);
                    iy_max = std::max(iy_max, iy);
                }
            }
        }
        out.grid_metadata.bbox_ix_min = (ix_max >= ix_min) ? ix_min : 0;
        out.grid_metadata.bbox_iy_min = (iy_max >= iy_min) ? iy_min : 0;
        out.grid_metadata.bbox_ix_max = ix_max;
        out.grid_metadata.bbox_iy_max = iy_max;
    }

    const std::size_t max_ticks = static_cast<std::size_t>(
        std::llround(mcfg.max_mission_s / mcfg.dt_s));

    auto record_coverage_milestones = [&](RouteBMissionArmResult& arm,
                                          const CertifiedCoverageTracker& cov,
                                          double t_s,
                                          std::size_t tick_idx) {
        const double cf = cov.certifiedFraction();
        const double tf = cov.truthFraction();
        if (!arm.reached_t95 && cf >= 0.95 - 1.0e-12) {
            arm.reached_t95 = true;
            arm.t95_certified_s = t_s;
        }
        if (!arm.reached_t100 && cov.reachedCertifiedT100()) {
            arm.reached_t100 = true;
            arm.t100_certified_s = t_s;
        }
        if (tick_idx % sample_every == 0) {
            arm.coverage_curve.push_back({t_s, tf, cf});
        }
    };

    // 3. Arm A: Stay in initial formation
    if (!mcfg.skip_stay_arm) {
        const auto wt0 = std::chrono::steady_clock::now();
        out.stay.arm_name = "stay";
        out.stay.reconfig_completed = true;
        out.stay.coverage_curve = prefix_curve;
        if (mcfg.record_trajectory_samples) {
            out.stay.trajectory_samples = prefix_traj;
        }
        RouteBClosedLoopAuditor auditor(
            scen.mobile_ids, scen.fixed_positions, mcfg.d_safe_m, mcfg.d_dep_m, 25.0,
            3.0, UncertaintyScaleMode::TraceRoot, mcfg.dt_s);
        route_b_mission_detail::FrontierSweepPolicy stay_sweep(
            dom.initial_tri, mcfg.patrol_radius_m, 0.8, 1.8, mcfg.a_max_mps2,
            mcfg.max_yaw_rate_radps, acfg.sensor_radius_m,
            mcfg.dynamic_chain_sweep, mcfg.search_speed_cap_mps, mcfg.d_dep_m);

        for (std::size_t tick = fork_ticks; tick < max_ticks; ++tick) {
            const double t_s = static_cast<double>(tick + 1) * mcfg.dt_s;
            const auto snap = adapter_stay.runtimeSnapshot();
            std::map<NodeId, Eigen::Vector2d> nom;
            std::map<NodeId, double> yaw;
            try {
                stay_sweep.computeCommands(swarm_stay, adapter_stay, snap.estimate, tick, nom, yaw);
            } catch (const std::exception& e) {
                throw std::runtime_error(std::string("stay_sweep: ") + e.what());
            }
            try {
                const auto step = adapter_stay.stepWithNominalAndYawRates(nom, yaw);
                if (!step.advanced) {
                    out.stay.status_reason = "step_aborted:" + step.reason;
                    out.stay.final_time_s = t_s;
                    break;
                }
                auditor.observe(swarm_stay, adapter_stay.runtimeSnapshot());
            } catch (const std::exception& e) {
                throw std::runtime_error(std::string("stay_step_obs: ") + e.what());
            }
            record_coverage_milestones(out.stay, adapter_stay.coverage(), t_s, tick + 1);
            if (mcfg.record_trajectory_samples && (tick + 1) % traj_every == 0) {
                capture_traj(swarm_stay, t_s, out.stay.trajectory_samples);
            }
            out.stay.final_time_s = t_s;
            if (out.stay.reached_t100) {
                if (out.stay.coverage_curve.empty() ||
                    std::abs(out.stay.coverage_curve.back().time_s - t_s) > 1.0e-9) {
                    out.stay.coverage_curve.push_back({
                        t_s, adapter_stay.coverage().truthFraction(),
                        adapter_stay.coverage().certifiedFraction()});
                }
                if (mcfg.record_trajectory_samples) {
                    capture_traj(swarm_stay, t_s, out.stay.trajectory_samples);
                }
                break;
            }
        }
        out.stay.final_truth_coverage = adapter_stay.coverage().truthFraction();
        out.stay.final_certified_coverage = adapter_stay.coverage().certifiedFraction();
        out.stay.false_certified_count = adapter_stay.coverage().falseCertifiedCount();
        out.stay.certified_subset_of_truth = adapter_stay.coverage().certifiedSubsetOfTruth();
        out.stay.audit = auditor.summarize();
        out.stay.wall_clock_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - wt0).count();
    }

    // 4. Arm B: Polar Entry -> Pipelined Reconfiguration -> Post-Switch Search
    if (!mcfg.skip_switch_arm) {
        const auto wt0 = std::chrono::steady_clock::now();
        out.switch_arm.arm_name = "switch";
        out.switch_arm.reconfig_completed = false;
        out.switch_arm.reconfig_entry_s = mcfg.t_fork_s;
        out.switch_arm.coverage_curve = prefix_curve;
        if (mcfg.record_trajectory_samples) {
            out.switch_arm.trajectory_samples = prefix_traj;
        }
        if (mcfg.record_mission_snapshots) {
            out.switch_arm.mission_snapshots = prefix_snapshots;
        }
        bool t95_snap_captured = out.switch_arm.reached_t95;
        bool t100_snap_captured = out.switch_arm.reached_t100;

        RouteBClosedLoopAuditor auditor(
            scen.mobile_ids, scen.fixed_positions, mcfg.d_safe_m, mcfg.d_dep_m, 25.0,
            3.0, UncertaintyScaleMode::TraceRoot, mcfg.dt_s);

        auto compute_vel_yaw_rates = [&](const GrandFinaleRuntimeSnapshot& snap) {
            std::map<NodeId, double> cur_yaw;
            for (const auto& robot : swarm_sw.robots) {
                cur_yaw[robot->id] = robot->model->getStateVariable("yawRad");
            }
            std::map<NodeId, double> yaw;
            for (std::size_t i = 0; i < snap.estimate.mobile_ids.size(); ++i) {
                const NodeId id = snap.estimate.mobile_ids[i];
                const Eigen::Vector2d v_est =
                    snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i) + 2);
                yaw[id] = computeVelocityAlignedYawRate(
                    v_est, cur_yaw.at(id), mcfg.max_yaw_rate_radps);
            }
            return yaw;
        };

        // Compute transfer plan (direct-matched or berth-mediated)
        std::vector<RouteBMove> sw_moves;
        RouteBTriangulation resolved_target = dom.target_tri;
        try {
            if (dom.use_direct_matched) {
                std::tie(sw_moves, resolved_target) =
                    planDirectMatchedTransfer(dom.initial_tri, dom.target_tri);
            } else {
                sw_moves = planLabelledTransfer(dom.initial_tri, dom.target_tri);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("switch_plan: ") + e.what());
        }

        // Phase B1: Similarity-preserving polar entry back to source_tri.positions
        const auto entry_snap = adapter_sw.runtimeSnapshot();
        std::map<NodeId, Eigen::Vector2d> p0, v0;
        for (std::size_t i = 0; i < scen.mobile_ids.size(); ++i) {
            const NodeId id = entry_snap.estimate.mobile_ids[i];
            p0[id] = entry_snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i));
            v0[id] = entry_snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i) + 2);
        }
        Eigen::Vector2d pivot = Eigen::Vector2d::Zero();
        for (const auto& [bid, bp] : dom.initial_tri.beacons) {
            (void)bid;
            pivot += bp;
        }
        pivot /= static_cast<double>(dom.initial_tri.beacons.size());

        RouteBPolarEntryConfig entry_cfg;
        entry_cfg.accel_budget_mps2 = 2.0;
        entry_cfg.axis_speed_budget_mps = mcfg.v_cruise_mps;
        entry_cfg.brake_accel_mps2 = 2.0;
        entry_cfg.arrival_pos_tol_m = 3.0;
        entry_cfg.arrival_speed_tol_mps = 0.5;
        std::optional<RouteBPolarEntryExecutor> entry_exec_opt;
        try {
            entry_exec_opt.emplace(p0, v0, dom.initial_tri.positions, pivot, entry_cfg);
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("entry_ctor: ") + e.what());
        }
        auto& entry_exec = *entry_exec_opt;

        RouteBPipelinedConfig pipe_cfg;
        pipe_cfg.serial_mode = false;
        pipe_cfg.dt_s = mcfg.dt_s;
        pipe_cfg.v_cruise_mps = mcfg.v_cruise_mps;
        pipe_cfg.accel_mps2 = mcfg.accel_mps2;
        pipe_cfg.a_max_mps2 = mcfg.a_max_mps2;
        pipe_cfg.d_safe_m = mcfg.d_safe_m;
        pipe_cfg.d_dep_m = mcfg.d_dep_m;
        pipe_cfg.d_pipe_m = mcfg.d_pipe_m;
        pipe_cfg.d_clear_m = mcfg.d_clear_m;
        pipe_cfg.t_settle_s = mcfg.t_settle_s;
        pipe_cfg.uwb_d_r_m = mcfg.uwb_d_r_m;
        pipe_cfg.uwb_sigma0_m = mcfg.uwb_sigma0_m;
        pipe_cfg.uwb_d_max_m = mcfg.uwb_d_max_m;
        pipe_cfg.link_seed = mcfg.link_seed;
        pipe_cfg.maximum_range_aoi_s = mcfg.maximum_range_aoi_s;

        std::optional<RouteBPipelinedSession> pipe_session;
        std::optional<route_b_mission_detail::FrontierSweepPolicy> post_sweep;

        enum class SwitchStage { Entry, Reconfig, Sweep };
        SwitchStage stage = SwitchStage::Entry;
        double entry_elapsed_s = 0.0;

        for (std::size_t tick = fork_ticks; tick < max_ticks; ++tick) {
            const double t_s = static_cast<double>(tick + 1) * mcfg.dt_s;
            out.switch_arm.final_time_s = t_s;

            if (stage == SwitchStage::Entry) {
                const auto snap = adapter_sw.runtimeSnapshot();
                std::map<NodeId, Eigen::Vector2d> nom;
                try {
                    nom = entry_exec.nominalControls(snap.estimate, entry_elapsed_s);
                } catch (const std::exception& e) {
                    throw std::runtime_error(std::string("entry_nom: ") + e.what());
                }
                const auto yaw = compute_vel_yaw_rates(snap);
                try {
                    const auto step = adapter_sw.stepWithNominalAndYawRates(nom, yaw);
                    if (!step.advanced) {
                        out.switch_arm.status_reason = "entry_aborted:" + step.reason;
                        break;
                    }
                    auditor.observe(swarm_sw, adapter_sw.runtimeSnapshot());
                } catch (const std::exception& e) {
                    throw std::runtime_error(std::string("entry_step_obs: ") + e.what());
                }
                entry_elapsed_s += mcfg.dt_s;
                record_coverage_milestones(out.switch_arm, adapter_sw.coverage(), t_s, tick + 1);
                const bool t95_event = out.switch_arm.reached_t95 && !t95_snap_captured;
                if (t95_event) t95_snap_captured = true;
                const bool t100_event = out.switch_arm.reached_t100 && !t100_snap_captured;
                if (t100_event) t100_snap_captured = true;
                if (should_capture_snapshot(t_s, t95_event || t100_event)) {
                    capture_snapshot(swarm_sw, adapter_sw, t_s, tick + 1, "entry", out.switch_arm.mission_snapshots);
                }
                if (entry_exec.arrived(adapter_sw.runtimeSnapshot().estimate, entry_elapsed_s)) {
                    stage = SwitchStage::Reconfig;
                    try {
                        pipe_session.emplace(
                            swarm_sw, adapter_sw, dom.initial_tri, resolved_target,
                            sw_moves, pipe_cfg, &auditor);
                    } catch (const std::exception& e) {
                        throw std::runtime_error(std::string("pipe_session_init: ") + e.what());
                    }
                }
            } else if (stage == SwitchStage::Reconfig) {
                auto yaw = compute_vel_yaw_rates(adapter_sw.runtimeSnapshot());
                bool done = false;
                try {
                    done = pipe_session->stepOnce(&yaw);
                } catch (const std::exception& e) {
                    throw std::runtime_error(std::string("pipe_step: ") + e.what());
                }
                if (pipe_session->finished() && !pipe_session->completed()) {
                    out.switch_arm.status_reason = "reconfig_failed:" + pipe_session->reason();
                    break;
                }
                record_coverage_milestones(out.switch_arm, adapter_sw.coverage(), t_s, tick + 1);
                const bool t95_event = out.switch_arm.reached_t95 && !t95_snap_captured;
                if (t95_event) t95_snap_captured = true;
                const bool t100_event = out.switch_arm.reached_t100 && !t100_snap_captured;
                if (t100_event) t100_snap_captured = true;
                if (should_capture_snapshot(t_s, done || t95_event || t100_event)) {
                    capture_snapshot(swarm_sw, adapter_sw, t_s, tick + 1, "reconfig", out.switch_arm.mission_snapshots);
                }
                if (done) {
                    out.switch_arm.reconfig_completed = true;
                    out.switch_arm.reconfig_complete_s = t_s;
                    out.switch_arm.static_parent_violations = pipe_session->staticParentViolations();
                    out.switch_arm.dag_violations = pipe_session->dagViolations();
                    out.switch_arm.total_mbb_swaps = pipe_session->totalMbbSwaps();
                    stage = SwitchStage::Sweep;
                    post_sweep.emplace(
                        resolved_target, mcfg.patrol_radius_m, 0.8, 1.8,
                        mcfg.a_max_mps2, mcfg.max_yaw_rate_radps, acfg.sensor_radius_m,
                        mcfg.dynamic_chain_sweep, mcfg.search_speed_cap_mps, mcfg.d_dep_m);
                }
            } else {
                const auto snap = adapter_sw.runtimeSnapshot();
                std::map<NodeId, Eigen::Vector2d> nom;
                std::map<NodeId, double> yaw;
                try {
                    post_sweep->computeCommands(swarm_sw, adapter_sw, snap.estimate, tick, nom, yaw);
                } catch (const std::exception& e) {
                    throw std::runtime_error(std::string("post_sweep: ") + e.what());
                }
                GrandFinaleSwarmStep step;
                try {
                    step = adapter_sw.stepWithNominalAndYawRates(nom, yaw);
                } catch (const std::exception& e) {
                    throw std::runtime_error(
                        std::string("post_sweep_step(tick=") + std::to_string(tick) +
                        ",nom=" + std::to_string(nom.size()) + "): " + e.what());
                }
                if (!step.advanced) {
                    out.switch_arm.status_reason = "post_sweep_aborted:" + step.reason;
                    break;
                }
                try {
                    auditor.observe(swarm_sw, adapter_sw.runtimeSnapshot());
                } catch (const std::exception& e) {
                    throw std::runtime_error(std::string("post_sweep_obs: ") + e.what());
                }
                record_coverage_milestones(out.switch_arm, adapter_sw.coverage(), t_s, tick + 1);
                const bool t95_event = out.switch_arm.reached_t95 && !t95_snap_captured;
                if (t95_event) t95_snap_captured = true;
                const bool t100_event = out.switch_arm.reached_t100 && !t100_snap_captured;
                if (t100_event) t100_snap_captured = true;
                if (should_capture_snapshot(t_s, t95_event || t100_event)) {
                    capture_snapshot(swarm_sw, adapter_sw, t_s, tick + 1, "sweep", out.switch_arm.mission_snapshots);
                }
                if (out.switch_arm.reached_t100) {
                    if (out.switch_arm.coverage_curve.empty() ||
                        std::abs(out.switch_arm.coverage_curve.back().time_s - t_s) > 1.0e-9) {
                        out.switch_arm.coverage_curve.push_back({
                            t_s, adapter_sw.coverage().truthFraction(),
                            adapter_sw.coverage().certifiedFraction()});
                    }
                    if (mcfg.record_trajectory_samples) {
                        capture_traj(swarm_sw, t_s, out.switch_arm.trajectory_samples);
                    }
                    break;
                }
            }
            if (mcfg.record_trajectory_samples && (tick + 1) % traj_every == 0) {
                capture_traj(swarm_sw, t_s, out.switch_arm.trajectory_samples);
            }
        }
        if (pipe_session.has_value() && !out.switch_arm.reconfig_completed) {
            out.switch_arm.static_parent_violations = pipe_session->staticParentViolations();
            out.switch_arm.dag_violations = pipe_session->dagViolations();
            out.switch_arm.total_mbb_swaps = pipe_session->totalMbbSwaps();
        }
        out.switch_arm.final_truth_coverage = adapter_sw.coverage().truthFraction();
        out.switch_arm.final_certified_coverage = adapter_sw.coverage().certifiedFraction();
        out.switch_arm.false_certified_count = adapter_sw.coverage().falseCertifiedCount();
        out.switch_arm.certified_subset_of_truth = adapter_sw.coverage().certifiedSubsetOfTruth();
        out.switch_arm.audit = auditor.summarize();
        out.switch_arm.wall_clock_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - wt0).count();
    }

    return out;
}

}  // namespace gf
