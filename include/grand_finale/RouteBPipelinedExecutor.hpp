#pragma once

#include "Swarm.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/RouteBClosedLoopAudit.hpp"
#include "grand_finale/RouteBPlanner.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"
#include "grand_finale/Types.hpp"

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
#include <tuple>
#include <utility>
#include <vector>

namespace gf {

struct RouteBScaleRadiusSample {
    double time_s = 0.0;
    double max_scale_radius_m = 0.0;
    double mean_scale_radius_m = 0.0;
    double position_rmse_m = 0.0;
};

struct RouteBTrajectorySample {
    double time_s = 0.0;
    std::map<NodeId, Eigen::Vector2d> positions;
};

struct RouteBPipelinedConfig {
    bool serial_mode = false;
    double dt_s = 0.1;
    double v_cruise_mps = 14.0;
    double accel_mps2 = 2.0;
    double a_max_mps2 = 4.0;
    double d_safe_m = 10.0;
    double d_dep_m = 850.0;
    double d_pipe_m = 60.0;
    double d_clear_m = 32.0;
    double t_settle_s = 1.0;
    double kp = 0.8;
    double kd = 1.8;
    double waypoint_pos_tol_m = 4.0;
    double waypoint_speed_tol_mps = 0.8;
    double terminal_pos_tol_m = 3.5;
    double terminal_speed_tol_mps = 0.5;
    double max_sim_time_s = 2500.0;
    double uwb_d_r_m = 850.0;
    double uwb_sigma0_m = 0.5;
    double uwb_d_max_m = 1200.0;
    unsigned int link_seed = 20261123;
    double maximum_range_aoi_s = 3.5;
    RangeMeasurementScope range_measurement_scope = RangeMeasurementScope::AllWithinCutoff;
    bool enable_auxiliary_handover_ranges = true;
    bool strict_two_ref_collinear_baseline = false;
    bool record_scale_radius_series = false;
    double scale_series_interval_s = 1.0;
    bool record_trajectory_samples = false;
    double trajectory_sample_interval_s = 5.0;
    double range_positive_bias_m = 0.0;
    Eigen::Vector2d plant_wind_disturbance_mps2 = Eigen::Vector2d::Zero();
    std::map<NodeId, Eigen::Vector2d> beacon_truth_offsets_m;
    bool record_timing_stats = false;
};

struct RouteBGanttEntry {
    NodeId node = 0;
    double start_s = 0.0;
    double finish_s = 0.0;
};

struct RouteBPipelinedRunResult {
    bool completed = false;
    std::string reason;
    double makespan_s = 0.0;
    double wall_clock_s = 0.0;
    std::size_t ticks = 0;
    std::size_t static_parent_violations = 0;
    std::size_t dag_violations = 0;
    std::size_t max_concurrent_moving = 0;
    std::size_t total_mbb_swaps = 0;
    std::size_t mbb_retries = 0;
    double min_inter_moving_distance_m = std::numeric_limits<double>::infinity();
    double final_position_error_m = 0.0;
    bool final_topology_matched = false;
    RouteBClosedLoopAuditSummary audit;
    std::vector<RouteBGanttEntry> gantt;
    std::vector<RouteBScaleRadiusSample> scale_radius_series;
    std::vector<RouteBTrajectorySample> trajectory_samples;
    std::vector<double> per_uav_qp_solve_s;
    std::vector<double> per_uav_qp_total_s;
    std::vector<double> estimator_samples_s;
    std::vector<double> tick_total_samples_s;
    std::size_t total_scheduled_range_packets = 0;
    std::size_t total_accepted_range_packets = 0;
};

namespace route_b_exec_detail {

enum class AgentExecState {
    QueuedWithdraw,
    Moving,
    Settling,
    Berthed,
    SeatedTarget
};

struct AgentPlan {
    NodeId node = 0;
    std::vector<RouteBMove> moves;
    std::size_t move_idx = 0;
    std::size_t leg_idx = 0;
    double s_prog = 0.0;
    double v_ref = 0.0;
    AgentExecState state = AgentExecState::QueuedWithdraw;
    double settle_remaining_s = 0.0;
    int launch_rank = 0;
    double first_move_s = -1.0;
    double last_finish_s = -1.0;
    bool awaiting_mbb_union_finish = false;
    std::optional<NodeId> stationary_handover_probe_ref;
};

inline std::size_t firstInstallLegIndex(
    const RouteBMove& mv,
    const RouteBTriangulation& tgt) {
    bool has_coast = false;
    std::size_t last_coast = 0;
    for (std::size_t i = 0; i < mv.legs.size(); ++i) {
        if (mv.legs[i].kind == "coast") {
            has_coast = true;
            last_coast = i;
        }
    }
    if (has_coast) return last_coast + 1;
    if (mv.legs.size() >= 2 && (mv.legs.front().start - tgt.positions.at(mv.node)).norm() > 1.0e-6) {
        for (std::size_t i = 1; i < mv.legs.size(); ++i) {
            if (mv.legs[i].kind == "root") return i;
        }
    }
    return 0;
}

inline std::vector<NodeId> legRefs(const RouteBLeg& leg) {
    std::vector<NodeId> refs{leg.parents.first, leg.parents.second};
    if (leg.auxiliary.has_value()) refs.push_back(*leg.auxiliary);
    if (leg.prerequest_next_parent.has_value()) refs.push_back(*leg.prerequest_next_parent);
    return refs;
}

inline double pointSegmentDist(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b) {
    const Eigen::Vector2d ab = b - a;
    const double den = ab.squaredNorm();
    if (den <= 1.0e-12) return (p - a).norm();
    const double u = std::clamp((p - a).dot(ab) / den, 0.0, 1.0);
    return (p - (a + u * ab)).norm();
}

inline bool dagCheck(
    const std::vector<NodeId>& mobile_ids,
    const std::map<NodeId, Eigen::Vector2d>& fixed_positions,
    const std::vector<DirectedEdge>& edges,
    std::size_t max_indegree = 3) {
    std::map<NodeId, std::vector<NodeId>> children;
    std::map<NodeId, int> indeg;
    for (NodeId id : mobile_ids) indeg[id] = 0;
    for (const auto& e : edges) {
        if (indeg.count(e.owner) == 0) return false;
        indeg[e.owner]++;
        if (fixed_positions.count(e.reference) == 0) {
            children[e.reference].push_back(e.owner);
        }
    }
    for (NodeId id : mobile_ids) {
        if (indeg[id] < 2 || static_cast<std::size_t>(indeg[id]) > max_indegree) {
            return false;
        }
    }
    std::map<NodeId, int> non_beacon_indeg;
    for (NodeId id : mobile_ids) non_beacon_indeg[id] = 0;
    for (const auto& e : edges) {
        if (fixed_positions.count(e.reference) == 0) non_beacon_indeg[e.owner]++;
    }
    std::deque<NodeId> q;
    for (const auto& [id, d] : non_beacon_indeg) {
        if (d == 0) q.push_back(id);
    }
    std::size_t visited = 0;
    while (!q.empty()) {
        const NodeId u = q.front();
        q.pop_front();
        ++visited;
        for (NodeId ch : children[u]) {
            if (--non_beacon_indeg[ch] == 0) q.push_back(ch);
        }
    }
    return visited == mobile_ids.size();
}

}  // namespace route_b_exec_detail

class RouteBPipelinedSession {
public:
    RouteBPipelinedSession(
        Swarm& swarm,
        GrandFinaleSwarmAdapter& adapter,
        const RouteBTriangulation& source_tri,
        const RouteBTriangulation& target_tri,
        const std::vector<RouteBMove>& moves,
        const RouteBPipelinedConfig& exec_cfg,
        RouteBClosedLoopAuditor* auditor = nullptr)
        : swarm_(swarm),
          adapter_(adapter),
          source_tri_(source_tri),
          target_tri_(target_tri),
          exec_cfg_(exec_cfg),
          auditor_(auditor) {
        if (exec_cfg_.v_cruise_mps <= 0.0 || exec_cfg_.v_cruise_mps > 25.0) {
            throw std::invalid_argument("Route B v_cruise must be in (0, 25.0]");
        }
        for (std::size_t i = 0; i < swarm_.robots.size(); ++i) {
            robot_idx_by_id_[static_cast<NodeId>(swarm_.robots[i]->id)] = i;
        }
        std::map<NodeId, std::vector<RouteBMove>> moves_by_node;
        std::vector<NodeId> launch_order;
        for (const auto& mv : moves) {
            if (moves_by_node.count(mv.node) == 0) launch_order.push_back(mv.node);
            moves_by_node[mv.node].push_back(mv);
        }
        const auto prefix = maximalSharedPrefix(source_tri_, target_tri_);
        for (NodeId nid : source_tri_.mobile_ids) {
            route_b_exec_detail::AgentPlan ap;
            ap.node = nid;
            if (prefix.count(nid) != 0 || moves_by_node.count(nid) == 0) {
                ap.state = route_b_exec_detail::AgentExecState::SeatedTarget;
                ap.first_move_s = 0.0;
                ap.last_finish_s = 0.0;
            } else {
                ap.moves = moves_by_node.at(nid);
                ap.state = route_b_exec_detail::AgentExecState::QueuedWithdraw;
                const auto it = std::find(launch_order.begin(), launch_order.end(), nid);
                ap.launch_rank = static_cast<int>(it - launch_order.begin());
            }
            plans_[nid] = std::move(ap);
            static_anchors_[nid] = source_tri_.positions.at(nid);
        }

        const double corridor_top = source_tri_.h_c_m + exec_cfg_.d_pipe_m;
        auto find_rp = [&](NodeId nid) {
            std::vector<NodeId> bset;
            std::deque<NodeId> stack;
            const auto p0 = target_tri_.parents.at(nid);
            stack.push_back(p0.first);
            stack.push_back(p0.second);
            std::set<NodeId> seen;
            while (!stack.empty()) {
                NodeId cur = stack.front();
                stack.pop_front();
                if (!seen.insert(cur).second) continue;
                if (target_tri_.isBeacon(cur)) {
                    bset.push_back(cur);
                } else {
                    const auto ps = target_tri_.parents.at(cur);
                    stack.push_back(ps.first);
                    stack.push_back(ps.second);
                }
            }
            std::sort(bset.begin(), bset.end(), [&](NodeId a, NodeId b) {
                return target_tri_.beacons.at(a).x() < target_tri_.beacons.at(b).x();
            });
            return std::make_pair(bset.front(), bset.back());
        };

        for (const auto& [nid, ap] : plans_) {
            if (ap.moves.empty()) continue;
            const auto& mv = ap.moves.back();
            if ((mv.legs.back().end - target_tri_.positions.at(nid)).norm() > 1.0e-5) continue;
            const auto rp = find_rp(nid);
            target_root_pair_[nid] = rp;
            const std::size_t start_i = route_b_exec_detail::firstInstallLegIndex(mv, target_tri_);
            for (std::size_t idx = start_i; idx < mv.legs.size(); ++idx) {
                const auto& lg = mv.legs[idx];
                for (int s_i = 0; s_i <= 10; ++s_i) {
                    const double frac = 0.1 * static_cast<double>(s_i);
                    const Eigen::Vector2d pt = lg.start + frac * (lg.end - lg.start);
                    if (pt.y() <= corridor_top + 1.0e-9) {
                        const double dx_pad = std::sqrt(std::max(
                            0.0, exec_cfg_.d_pipe_m * exec_cfg_.d_pipe_m -
                                 (pt.y() - source_tri_.h_c_m) * (pt.y() - source_tri_.h_c_m)));
                        const double lo = pt.x() - dx_pad;
                        const double hi = pt.x() + dx_pad;
                        if (climb_bands_.count(rp) == 0) {
                            climb_bands_[rp] = {lo, hi};
                        } else {
                            climb_bands_[rp].first = std::min(climb_bands_[rp].first, lo);
                            climb_bands_[rp].second = std::max(climb_bands_[rp].second, hi);
                        }
                    }
                }
            }
        }
    }

    bool stepOnce(const std::map<NodeId, double>* yaw_rates = nullptr) {
        using namespace route_b_exec_detail;
        if (finished_) return true;

        const double now_s = static_cast<double>(ticks_) * exec_cfg_.dt_s;
        auto snap = adapter_.runtimeSnapshot();

        bool just_finished_mbb = false;
        if (snap.adapter_transition_pending) {
            if (!adapter_.finishReplacementAfterFreshCycle(true)) {
                ++mbb_retries_;
            } else {
                just_finished_mbb = true;
                for (auto& [nid, ap] : plans_) {
                    (void)nid;
                    if (ap.awaiting_mbb_union_finish) {
                        ap.awaiting_mbb_union_finish = false;
                        ++ap.leg_idx;
                        ap.s_prog = 0.0;
                        ap.v_ref = 0.0;
                    }
                }
            }
            snap = adapter_.runtimeSnapshot();
        }

        std::map<NodeId, Eigen::Vector2d> est_pos;
        std::map<NodeId, Eigen::Vector2d> est_vel;
        for (std::size_t i = 0; i < source_tri_.mobile_ids.size(); ++i) {
            const NodeId id = snap.estimate.mobile_ids[i];
            est_pos[id] = snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i));
            est_vel[id] = snap.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(i) + 2);
        }

        bool all_seated = true;
        for (const auto& [nid, ap] : plans_) {
            (void)nid;
            if (ap.state != AgentExecState::SeatedTarget) {
                all_seated = false;
                break;
            }
        }
        if (all_seated && !snap.adapter_transition_pending) {
            completed_ = true;
            finished_ = true;
            makespan_s_ = now_s;
            return true;
        }

        for (auto& [nid, ap] : plans_) {
            if (ap.state == AgentExecState::Settling) {
                ap.settle_remaining_s -= exec_cfg_.dt_s;
                if (ap.settle_remaining_s <= 1.0e-9) {
                    const bool is_last_move = (ap.move_idx + 1 == ap.moves.size());
                    const Eigen::Vector2d leg_end = ap.moves[ap.move_idx].legs.back().end;
                    if (is_last_move && (leg_end - target_tri_.positions.at(nid)).norm() <= 1.0e-5) {
                        ap.state = AgentExecState::SeatedTarget;
                        ap.last_finish_s = now_s;
                        static_anchors_[nid] = target_tri_.positions.at(nid);
                    } else {
                        ap.state = AgentExecState::Berthed;
                        ++ap.move_idx;
                        ap.leg_idx = 0;
                        ap.s_prog = 0.0;
                        ap.v_ref = 0.0;
                        static_anchors_[nid] = leg_end;
                    }
                }
            }
        }

        const double corridor_top = source_tri_.h_c_m + exec_cfg_.d_pipe_m;

        auto is_stationary = [&](NodeId v) {
            if (source_tri_.isBeacon(v)) return true;
            return plans_.at(v).state != AgentExecState::Moving;
        };

        auto ref_ready_for_leg = [&](const AgentPlan& ap, const RouteBLeg& leg, std::size_t leg_i) {
            for (NodeId r : legRefs(leg)) {
                if (source_tri_.isBeacon(r)) continue;
                if (!is_stationary(r)) return false;
                if ((ap.state == AgentExecState::Berthed || ap.state == AgentExecState::Moving) &&
                    ap.move_idx + 1 == ap.moves.size()) {
                    const auto& mv = ap.moves[ap.move_idx];
                    if ((mv.legs.back().end - target_tri_.positions.at(ap.node)).norm() <= 1.0e-5) {
                        const std::size_t first_inst = firstInstallLegIndex(mv, target_tri_);
                        if (leg_i >= first_inst &&
                            (leg.kind == "root" || leg.kind == "bisector" || leg.kind == "tip")) {
                            const auto tp = target_tri_.parents.at(ap.node);
                            if (r == tp.first || r == tp.second) {
                                if (plans_.at(r).state != AgentExecState::SeatedTarget) return false;
                            }
                        }
                    }
                }
            }
            return true;
        };

        auto can_release_from_source = [&](const AgentPlan& ag) {
            const NodeId nid = ag.node;
            for (const auto& [oid, other] : plans_) {
                if (oid == nid || other.state == AgentExecState::SeatedTarget ||
                    other.state == AgentExecState::Berthed) {
                    continue;
                }
                if (other.moves.size() == 2 && other.move_idx == 1) continue;
                const auto& mv = other.moves[other.move_idx];
                std::size_t stop_idx = mv.legs.size();
                if ((mv.legs.back().end - target_tri_.positions.at(oid)).norm() <= 1.0e-5) {
                    for (std::size_t i = 0; i < mv.legs.size(); ++i) {
                        if (mv.legs[i].kind == "coast") {
                            stop_idx = i;
                            break;
                        }
                    }
                }
                const std::size_t start_i =
                    (other.state == AgentExecState::Moving) ? other.leg_idx : 0;
                for (std::size_t i = start_i; i < stop_idx; ++i) {
                    for (NodeId r : legRefs(mv.legs[i])) {
                        if (r == nid) return false;
                    }
                }
            }
            return true;
        };

        auto target_parents_seated = [&](const AgentPlan& ag) {
            if (ag.state != AgentExecState::Berthed) return true;
            const auto tp = target_tri_.parents.at(ag.node);
            for (NodeId p : {tp.first, tp.second}) {
                if (!target_tri_.isBeacon(p) && plans_.at(p).state != AgentExecState::SeatedTarget) {
                    return false;
                }
            }
            return true;
        };

        auto target_slot_occupied_by_source = [&](const AgentPlan& ag) {
            if (ag.state != AgentExecState::Berthed) return false;
            const auto& mv = ag.moves[ag.move_idx];
            for (const auto& [oid, other] : plans_) {
                if (oid == ag.node || other.state != AgentExecState::QueuedWithdraw) continue;
                const auto& opt = est_pos.at(oid);
                for (const auto& lg : mv.legs) {
                    if (pointSegmentDist(opt, lg.start, lg.end) < exec_cfg_.d_clear_m + 1.0) {
                        return true;
                    }
                }
            }
            return false;
        };

        auto funnel_corridor_conflict = [&](const AgentPlan& ag,
                                            const Eigen::Vector2d& cand_pt,
                                            std::size_t entering_leg_idx) {
            const auto& mv = ag.moves[ag.move_idx];
            const auto& leg = mv.legs[entering_leg_idx];
            const bool is_wd_descent =
                (leg.kind == "tip" || leg.kind == "bisector" || leg.kind == "root") &&
                (leg.end.y() < leg.start.y() - 1.0e-6) &&
                (cand_pt.y() < corridor_top - 1.0e-6) &&
                (est_pos.at(ag.node).y() >= corridor_top - 25.0);
            if (!is_wd_descent) return false;
            double fx_lo = std::numeric_limits<double>::infinity();
            double fx_hi = -std::numeric_limits<double>::infinity();
            for (std::size_t i = entering_leg_idx; i < mv.legs.size(); ++i) {
                fx_lo = std::min({fx_lo, mv.legs[i].start.x(), mv.legs[i].end.x()});
                fx_hi = std::max({fx_hi, mv.legs[i].start.x(), mv.legs[i].end.x()});
                if (mv.legs[i].kind == "root") break;
            }
            fx_lo -= exec_cfg_.d_pipe_m;
            fx_hi += exec_cfg_.d_pipe_m;
            for (const auto& [oid, oag] : plans_) {
                if (oid == ag.node || oag.state != AgentExecState::Moving) continue;
                const auto& opt = est_pos.at(oid);
                if (opt.y() >= corridor_top) continue;
                if (fx_lo <= opt.x() && opt.x() <= fx_hi) return true;
                const auto& omv = oag.moves[oag.move_idx];
                for (std::size_t i = oag.leg_idx; i < omv.legs.size(); ++i) {
                    const auto& olg = omv.legs[i];
                    if (olg.kind == "coast") {
                        const double xs = (i == oag.leg_idx) ? opt.x() : olg.start.x();
                        const double ox_lo = std::min(xs, olg.end.x());
                        const double ox_hi = std::max(xs, olg.end.x());
                        if (std::min(fx_hi, ox_hi) >= std::max(fx_lo, ox_lo)) return true;
                    } else if (i > oag.leg_idx) {
                        break;
                    }
                }
            }
            return false;
        };

        auto climbing_past_corridor = [&](NodeId pid) {
            const auto& pag = plans_.at(pid);
            if (pag.state == AgentExecState::SeatedTarget) return true;
            if (pag.state == AgentExecState::Settling &&
                (est_pos.at(pid) - target_tri_.positions.at(pid)).norm() <=
                    exec_cfg_.terminal_pos_tol_m + 2.0) {
                return true;
            }
            if (pag.state == AgentExecState::Moving) {
                const auto& pmv = pag.moves[pag.move_idx];
                if ((pmv.legs.back().end - target_tri_.positions.at(pid)).norm() <= 1.0e-5) {
                    const std::size_t fi = firstInstallLegIndex(pmv, target_tri_);
                    if (pag.leg_idx >= fi && est_pos.at(pid).y() >= corridor_top - 2.0) {
                        return true;
                    }
                }
            }
            return false;
        };

        auto install_funnel_conflict = [&](const AgentPlan& ag, const Eigen::Vector2d& cand_pt) {
            const auto& mv = ag.moves[ag.move_idx];
            const auto& leg = mv.legs[ag.leg_idx];
            if (leg.kind != "coast") return false;
            if ((mv.legs.back().end - target_tri_.positions.at(ag.node)).norm() > 1.0e-5) return false;
            const auto trp = target_root_pair_.find(ag.node);
            if (trp == target_root_pair_.end()) return false;
            const auto band_it = climb_bands_.find(trp->second);
            if (band_it == climb_bands_.end()) return false;
            const auto [bx_lo, bx_hi] = band_it->second;
            const double cur_x = est_pos.at(ag.node).x();
            const bool entering =
                (cur_x < bx_lo || cur_x > bx_hi) && (bx_lo <= cand_pt.x() && cand_pt.x() <= bx_hi);
            if (!entering) return false;
            const auto tp = target_tri_.parents.at(ag.node);
            for (NodeId p : {tp.first, tp.second}) {
                if (!target_tri_.isBeacon(p) && !climbing_past_corridor(p)) return true;
            }
            const double port_x = mv.legs[firstInstallLegIndex(mv, target_tri_)].start.x();
            const bool from_west = cur_x < port_x;
            for (const auto& [oid, oag] : plans_) {
                if (oid == ag.node || oag.state != AgentExecState::Moving) continue;
                const auto& opt = est_pos.at(oid);
                if (opt.y() >= corridor_top) continue;
                if (from_west) {
                    if (port_x - exec_cfg_.d_pipe_m <= opt.x() &&
                        opt.x() <= port_x + exec_cfg_.d_pipe_m) {
                        return true;
                    }
                } else {
                    if (bx_lo <= opt.x() && opt.x() <= bx_hi) return true;
                }
            }
            return false;
        };

        auto berth_undock_conflict = [&](const AgentPlan& ag) {
            if (ag.state != AgentExecState::Berthed) return false;
            const auto& mv = ag.moves[ag.move_idx];
            double ux_lo = est_pos.at(ag.node).x();
            double ux_hi = ux_lo;
            for (const auto& lg : mv.legs) {
                if (lg.kind == "dock" || lg.kind == "coast") {
                    ux_lo = std::min({ux_lo, lg.start.x(), lg.end.x()});
                    ux_hi = std::max({ux_hi, lg.start.x(), lg.end.x()});
                } else {
                    break;
                }
            }
            ux_lo -= exec_cfg_.d_pipe_m;
            ux_hi += exec_cfg_.d_pipe_m;
            for (const auto& [oid, oag] : plans_) {
                if (oid == ag.node || oag.state != AgentExecState::Moving) continue;
                const auto& opt = est_pos.at(oid);
                if (opt.y() < corridor_top && ux_lo <= opt.x() && opt.x() <= ux_hi) return true;
                if (oag.moves.size() == 2 && oag.move_idx == 0) {
                    const auto& omv = oag.moves[0];
                    for (std::size_t i = oag.leg_idx; i < omv.legs.size(); ++i) {
                        const auto& olg = omv.legs[i];
                        if (olg.kind == "coast" || olg.kind == "root") {
                            const double ox_lo = std::min(olg.start.x(), olg.end.x());
                            const double ox_hi = std::max(olg.start.x(), olg.end.x());
                            if (std::min(ux_hi, ox_hi) >= std::max(ux_lo, ox_lo)) return true;
                        }
                    }
                }
            }
            return false;
        };

        auto coast_direction_conflict = [&](const AgentPlan& ag, const RouteBLeg& leg) {
            if (leg.kind != "coast") return false;
            const double dx = leg.end.x() - leg.start.x();
            if (std::abs(dx) < 1.0e-6) return false;
            const double x_lo = std::min(leg.start.x(), leg.end.x());
            const double x_hi = std::max(leg.start.x(), leg.end.x());
            for (const auto& [oid, oag] : plans_) {
                if (oid == ag.node || oag.state != AgentExecState::Moving) continue;
                const auto& oleg = oag.moves[oag.move_idx].legs[oag.leg_idx];
                if (oleg.kind != "coast") continue;
                const double odx = oleg.end.x() - oleg.start.x();
                if (dx * odx < 0.0) {
                    const double ox_lo = std::min(oleg.start.x(), oleg.end.x());
                    const double ox_hi = std::max(oleg.start.x(), oleg.end.x());
                    if (std::min(x_hi, ox_hi) - 1.0e-6 > std::max(x_lo, ox_lo)) return true;
                }
            }
            return false;
        };

        auto launch_spacing_ok = [&](NodeId nid) {
            const auto& my_p = est_pos.at(nid);
            for (const auto& [oid, oag] : plans_) {
                if (oid == nid || oag.state != AgentExecState::Moving) continue;
                if ((my_p - est_pos.at(oid)).norm() < exec_cfg_.d_pipe_m) return false;
            }
            return true;
        };

        auto any_agent_busy = [&]() {
            for (const auto& [nid, ap] : plans_) {
                (void)nid;
                if (ap.state == AgentExecState::Moving || ap.state == AgentExecState::Settling) {
                    return true;
                }
            }
            return false;
        };

        std::vector<NodeId> launch_candidates;
        for (const auto& [nid, ap] : plans_) {
            (void)ap;
            launch_candidates.push_back(nid);
        }
        std::sort(launch_candidates.begin(), launch_candidates.end(), [&](NodeId a, NodeId b) {
            const auto& pa = plans_.at(a);
            const auto& pb = plans_.at(b);
            const int ga = (pa.state == AgentExecState::Berthed) ? 0 : 1;
            const int gb = (pb.state == AgentExecState::Berthed) ? 0 : 1;
            if (ga != gb) return ga < gb;
            if (pa.launch_rank != pb.launch_rank) return pa.launch_rank < pb.launch_rank;
            return a < b;
        });

        for (NodeId nid : launch_candidates) {
            auto& ap = plans_.at(nid);
            if (exec_cfg_.serial_mode && any_agent_busy()) break;
            if (ap.state == AgentExecState::QueuedWithdraw) {
                const auto& leg0 = ap.moves[0].legs[0];
                if (can_release_from_source(ap) &&
                    ref_ready_for_leg(ap, leg0, 0) &&
                    !funnel_corridor_conflict(ap, est_pos.at(nid), 0) &&
                    !coast_direction_conflict(ap, leg0) &&
                    launch_spacing_ok(nid)) {
                    ap.state = AgentExecState::Moving;
                    ap.leg_idx = 0;
                    ap.s_prog = 0.0;
                    ap.v_ref = 0.0;
                    if (ap.first_move_s < 0.0) ap.first_move_s = now_s;
                }
            } else if (ap.state == AgentExecState::Berthed) {
                const auto& leg0 = ap.moves[ap.move_idx].legs[0];
                if (target_parents_seated(ap) &&
                    ref_ready_for_leg(ap, leg0, 0) &&
                    !berth_undock_conflict(ap) &&
                    !target_slot_occupied_by_source(ap) &&
                    !coast_direction_conflict(ap, leg0) &&
                    launch_spacing_ok(nid)) {
                    ap.state = AgentExecState::Moving;
                    ap.leg_idx = 0;
                    ap.s_prog = 0.0;
                    ap.v_ref = 0.0;
                }
            }
        }

        std::vector<NodeId> moving_list;
        for (const auto& [nid, ap] : plans_) {
            if (ap.state == AgentExecState::Moving) moving_list.push_back(nid);
        }
        max_concurrent_moving_ = std::max(max_concurrent_moving_, moving_list.size());

        auto moving_prio = [&](NodeId id) {
            const auto& ap = plans_.at(id);
            const std::size_t rem_legs = ap.moves[ap.move_idx].legs.size() - ap.leg_idx;
            const int last_mv = (ap.move_idx + 1 == ap.moves.size()) ? 0 : 1;
            return std::make_tuple(last_mv, rem_legs, ap.launch_rank, id);
        };
        std::sort(moving_list.begin(), moving_list.end(), [&](NodeId a, NodeId b) {
            return moving_prio(a) < moving_prio(b);
        });

        std::map<NodeId, std::set<NodeId>> active_parents_map;
        for (const auto& e : snap.topology) active_parents_map[e.owner].insert(e.reference);

        std::map<NodeId, Eigen::Vector2d> nominal;
        for (NodeId nid : source_tri_.mobile_ids) {
            const auto& ap = plans_.at(nid);
            if (ap.state != AgentExecState::Moving) {
                nominal[nid] = (exec_cfg_.kp * (static_anchors_.at(nid) - est_pos.at(nid)) -
                                exec_cfg_.kd * est_vel.at(nid))
                                   .cwiseMax(-exec_cfg_.a_max_mps2)
                                   .cwiseMin(exec_cfg_.a_max_mps2);
            }
        }

        bool supervisor_busy =
            snap.adapter_transition_pending || just_finished_mbb ||
            (snap.timer_since_supervisor_transition_s + 1.0e-12 < exec_cfg_.dt_s);

        for (std::size_t rank_idx = 0; rank_idx < moving_list.size(); ++rank_idx) {
            const NodeId nid = moving_list[rank_idx];
            auto& ap = plans_.at(nid);
            auto& mv = ap.moves[ap.move_idx];
            const auto& leg = mv.legs[ap.leg_idx];

            if (ap.awaiting_mbb_union_finish) {
                nominal[nid] = (exec_cfg_.kp * (leg.end - est_pos.at(nid)) -
                                exec_cfg_.kd * est_vel.at(nid))
                                   .cwiseMax(-exec_cfg_.a_max_mps2)
                                   .cwiseMin(exec_cfg_.a_max_mps2);
                continue;
            }

            const double seg_len = (leg.end - leg.start).norm();
            const Eigen::Vector2d tangent =
                seg_len > 1.0e-9 ? ((leg.end - leg.start) / seg_len).eval() : Eigen::Vector2d(1.0, 0.0);

            const bool at_end_geom = ap.s_prog >= seg_len - 1.0e-6;
            const bool at_terminal_leg = (ap.leg_idx + 1 == mv.legs.size());
            const double pos_tol =
                at_terminal_leg ? exec_cfg_.terminal_pos_tol_m : exec_cfg_.waypoint_pos_tol_m;
            const double spd_tol =
                at_terminal_leg ? exec_cfg_.terminal_speed_tol_mps : exec_cfg_.waypoint_speed_tol_mps;
            const bool arrived_wp =
                at_end_geom &&
                (est_pos.at(nid) - leg.end).norm() <= pos_tol &&
                est_vel.at(nid).norm() <= spd_tol;

            if (arrived_wp) {
                if (at_terminal_leg) {
                    ap.state = AgentExecState::Settling;
                    ap.settle_remaining_s = exec_cfg_.t_settle_s;
                    static_anchors_[nid] = leg.end;
                    nominal[nid] = (exec_cfg_.kp * (leg.end - est_pos.at(nid)) -
                                    exec_cfg_.kd * est_vel.at(nid))
                                       .cwiseMax(-exec_cfg_.a_max_mps2)
                                       .cwiseMin(exec_cfg_.a_max_mps2);
                    continue;
                }

                const auto& nxt_leg = mv.legs[ap.leg_idx + 1];
                const bool guard_ok =
                    ref_ready_for_leg(ap, nxt_leg, ap.leg_idx + 1) &&
                    !funnel_corridor_conflict(ap, leg.end, ap.leg_idx + 1) &&
                    !coast_direction_conflict(ap, nxt_leg);

                if (guard_ok) {
                    const auto& cur_set = active_parents_map.at(nid);
                    const std::set<NodeId> nxt_set{nxt_leg.parents.first, nxt_leg.parents.second};
                    std::vector<NodeId> adds, rems;
                    for (NodeId p : nxt_set) {
                        if (cur_set.count(p) == 0) adds.push_back(p);
                    }
                    for (NodeId p : cur_set) {
                        if (nxt_set.count(p) == 0) rems.push_back(p);
                    }
                    if (adds.empty() && rems.empty()) {
                        ap.stationary_handover_probe_ref.reset();
                        ++ap.leg_idx;
                        ap.s_prog = 0.0;
                        ap.v_ref = 0.0;
                    } else if (adds.size() == 1 && rems.size() == 1 && !supervisor_busy) {
                        if (adapter_.beginReplacement({adds[0], nid}, {rems[0], nid}, false)) {
                            ap.stationary_handover_probe_ref.reset();
                            supervisor_busy = true;
                            ap.awaiting_mbb_union_finish = true;
                            ++total_mbb_swaps_;
                        } else {
                            if (exec_cfg_.strict_two_ref_collinear_baseline) {
                                ap.stationary_handover_probe_ref = adds[0];
                            }
                            ++mbb_retries_;
                        }
                    }
                }
                nominal[nid] = (exec_cfg_.kp * (leg.end - est_pos.at(nid)) -
                                exec_cfg_.kd * est_vel.at(nid))
                                   .cwiseMax(-exec_cfg_.a_max_mps2)
                                   .cwiseMin(exec_cfg_.a_max_mps2);
                continue;
            }

            const double d_stop =
                (ap.v_ref * ap.v_ref) / (2.0 * exec_cfg_.accel_mps2) + 12.0;
            const double look_prog = std::min(seg_len, ap.s_prog + d_stop);
            const Eigen::Vector2d cand_pt = leg.start + look_prog * tangent;
            bool hold = !ref_ready_for_leg(ap, leg, ap.leg_idx) ||
                        funnel_corridor_conflict(ap, cand_pt, ap.leg_idx) ||
                        install_funnel_conflict(ap, cand_pt);

            if (!hold &&
                (leg.kind == "tip" || leg.kind == "bisector" || leg.kind == "root") &&
                leg.end.y() < leg.start.y() - 1.0e-6 &&
                cand_pt.y() < corridor_top - 1.0e-6 &&
                est_pos.at(nid).y() >= corridor_top - 25.0) {
                double fx_lo = std::numeric_limits<double>::infinity();
                double fx_hi = -std::numeric_limits<double>::infinity();
                for (std::size_t i = ap.leg_idx; i < mv.legs.size(); ++i) {
                    fx_lo = std::min({fx_lo, mv.legs[i].start.x(), mv.legs[i].end.x()});
                    fx_hi = std::max({fx_hi, mv.legs[i].start.x(), mv.legs[i].end.x()});
                    if (mv.legs[i].kind == "root") break;
                }
                fx_lo -= exec_cfg_.d_pipe_m;
                fx_hi += exec_cfg_.d_pipe_m;
                for (std::size_t h_idx = 0; h_idx < rank_idx; ++h_idx) {
                    const NodeId leader = moving_list[h_idx];
                    const auto& l_pos = est_pos.at(leader);
                    if (l_pos.x() >= fx_lo && l_pos.x() <= fx_hi &&
                        l_pos.y() <= corridor_top + exec_cfg_.d_pipe_m) {
                        hold = true;
                        break;
                    }
                }
            }

            if (!hold) {
                for (std::size_t h_idx = 0; h_idx < rank_idx; ++h_idx) {
                    const NodeId leader = moving_list[h_idx];
                    const double sep_now = (est_pos.at(nid) - est_pos.at(leader)).norm();
                    const double sep_look = (cand_pt - est_pos.at(leader)).norm();
                    if (sep_now < exec_cfg_.d_pipe_m || sep_look < exec_cfg_.d_pipe_m) {
                        hold = true;
                        break;
                    }
                }
            }

            const double rem = std::max(0.0, seg_len - ap.s_prog);
            const double v_stop = std::sqrt(2.0 * exec_cfg_.accel_mps2 * rem);
            const double v_target = hold ? 0.0 : std::min(exec_cfg_.v_cruise_mps, v_stop);
            const double dv = std::clamp(
                v_target - ap.v_ref, -exec_cfg_.accel_mps2 * exec_cfg_.dt_s,
                exec_cfg_.accel_mps2 * exec_cfg_.dt_s);
            ap.v_ref = std::max(0.0, ap.v_ref + dv);
            ap.s_prog = std::min(seg_len, ap.s_prog + ap.v_ref * exec_cfg_.dt_s);

            const Eigen::Vector2d p_ref = leg.start + ap.s_prog * tangent;
            const Eigen::Vector2d v_ref = ap.v_ref * tangent;
            nominal[nid] = (exec_cfg_.kp * (p_ref - est_pos.at(nid)) +
                            exec_cfg_.kd * (v_ref - est_vel.at(nid)))
                               .cwiseMax(-exec_cfg_.a_max_mps2)
                               .cwiseMin(exec_cfg_.a_max_mps2);
        }

        const auto pre_topo = adapter_.supervisor().topology();
        for (const auto& e : pre_topo) {
            if (!source_tri_.isBeacon(e.reference)) {
                if (plans_.at(e.reference).state == AgentExecState::Moving) {
                    ++static_parent_violations_;
                }
            }
        }
        if (!dagCheck(source_tri_.mobile_ids, source_tri_.beacons, pre_topo, 3)) {
            ++dag_violations_;
        }

        for (std::size_t a = 0; a < moving_list.size(); ++a) {
            for (std::size_t b = a + 1; b < moving_list.size(); ++b) {
                const Point pa = swarm_.robots[robot_idx_by_id_.at(moving_list[a])]->model->xy();
                const Point pb = swarm_.robots[robot_idx_by_id_.at(moving_list[b])]->model->xy();
                min_inter_moving_distance_m_ = std::min(
                    min_inter_moving_distance_m_, std::hypot(pa.x - pb.x, pa.y - pb.y));
            }
        }

        if (exec_cfg_.range_measurement_scope ==
            RangeMeasurementScope::ActiveTopologyAndAuxiliary) {
            std::vector<DirectedEdge> aux_edges;
            for (const auto& [nid, ap] : plans_) {
                if (ap.state == AgentExecState::Moving) {
                    const auto& leg = ap.moves[ap.move_idx].legs[ap.leg_idx];
                    if (exec_cfg_.enable_auxiliary_handover_ranges) {
                        if (leg.auxiliary.has_value()) {
                            aux_edges.push_back({*leg.auxiliary, nid});
                        }
                        if (leg.prerequest_next_parent.has_value()) {
                            aux_edges.push_back({*leg.prerequest_next_parent, nid});
                        }
                    } else if (ap.stationary_handover_probe_ref.has_value()) {
                        aux_edges.push_back({*ap.stationary_handover_probe_ref, nid});
                    }
                }
            }
            adapter_.setAuxiliaryRangeEdges(std::move(aux_edges));
        }

        const auto step = (yaw_rates != nullptr)
            ? adapter_.stepWithNominalAndYawRates(nominal, *yaw_rates)
            : adapter_.stepWithNominal(nominal);
        ++ticks_;
        if (exec_cfg_.record_timing_stats && step.advanced) {
            per_uav_qp_solve_s_.insert(
                per_uav_qp_solve_s_.end(),
                step.per_uav_qp_solve_s.begin(),
                step.per_uav_qp_solve_s.end());
            per_uav_qp_total_s_.insert(
                per_uav_qp_total_s_.end(),
                step.per_uav_qp_total_s.begin(),
                step.per_uav_qp_total_s.end());
            estimator_samples_s_.push_back(step.estimator_wall_s);
            tick_total_samples_s_.push_back(step.tick_wall_s);
            total_scheduled_range_packets_ += step.scheduled_range_packet_count;
            total_accepted_range_packets_ += step.accepted_range_packet_count;
        }
        if (!step.advanced) {
            reason_ = "step_aborted:" + step.reason;
            makespan_s_ = now_s;
            finished_ = true;
            return false;
        }
        if (auditor_ != nullptr) {
            auditor_->observe(swarm_, adapter_.runtimeSnapshot());
        }
        return false;
    }

    bool finished() const { return finished_; }
    bool completed() const { return completed_; }
    const std::string& reason() const { return reason_; }
    double makespanS() const { return makespan_s_; }
    std::size_t ticks() const { return ticks_; }
    std::size_t staticParentViolations() const { return static_parent_violations_; }
    std::size_t dagViolations() const { return dag_violations_; }
    std::size_t maxConcurrentMoving() const { return max_concurrent_moving_; }
    std::size_t totalMbbSwaps() const { return total_mbb_swaps_; }
    std::size_t mbbRetries() const { return mbb_retries_; }
    double minInterMovingDistanceM() const { return min_inter_moving_distance_m_; }
    const std::vector<double>& perUavQpSolveS() const { return per_uav_qp_solve_s_; }
    const std::vector<double>& perUavQpTotalS() const { return per_uav_qp_total_s_; }
    const std::vector<double>& estimatorSamplesS() const { return estimator_samples_s_; }
    const std::vector<double>& tickTotalSamplesS() const { return tick_total_samples_s_; }
    std::size_t totalScheduledRangePackets() const { return total_scheduled_range_packets_; }
    std::size_t totalAcceptedRangePackets() const { return total_accepted_range_packets_; }

    std::vector<RouteBGanttEntry> gantt() const {
        std::vector<RouteBGanttEntry> out;
        for (const auto& [nid, ap] : plans_) {
            if (ap.first_move_s >= 0.0 && ap.last_finish_s >= 0.0 && !ap.moves.empty()) {
                out.push_back({nid, ap.first_move_s, ap.last_finish_s});
            }
        }
        return out;
    }

private:
    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    RouteBTriangulation source_tri_;
    RouteBTriangulation target_tri_;
    RouteBPipelinedConfig exec_cfg_;
    RouteBClosedLoopAuditor* auditor_ = nullptr;
    std::map<NodeId, std::size_t> robot_idx_by_id_;
    std::map<NodeId, route_b_exec_detail::AgentPlan> plans_;
    std::map<NodeId, std::pair<NodeId, NodeId>> target_root_pair_;
    std::map<std::pair<NodeId, NodeId>, std::pair<double, double>> climb_bands_;
    std::map<NodeId, Eigen::Vector2d> static_anchors_;

    bool finished_ = false;
    bool completed_ = false;
    std::string reason_;
    double makespan_s_ = 0.0;
    std::size_t ticks_ = 0;
    std::size_t static_parent_violations_ = 0;
    std::size_t dag_violations_ = 0;
    std::size_t max_concurrent_moving_ = 0;
    std::size_t total_mbb_swaps_ = 0;
    std::size_t mbb_retries_ = 0;
    double min_inter_moving_distance_m_ = std::numeric_limits<double>::infinity();
    std::vector<double> per_uav_qp_solve_s_;
    std::vector<double> per_uav_qp_total_s_;
    std::vector<double> estimator_samples_s_;
    std::vector<double> tick_total_samples_s_;
    std::size_t total_scheduled_range_packets_ = 0;
    std::size_t total_accepted_range_packets_ = 0;
};

inline RouteBPipelinedRunResult runClosedLoopReconfiguration(
    const RouteBTriangulation& source_tri,
    const RouteBTriangulation& target_tri,
    const std::vector<RouteBMove>& moves,
    const RouteBPipelinedConfig& exec_cfg = {}) {
    const auto wall_t0 = std::chrono::steady_clock::now();

    RouteBPipelinedRunResult res;
    double max_x = 1600.0;
    double max_y = 1600.0;
    for (const auto& [id, p] : source_tri.beacons) {
        (void)id;
        max_x = std::max(max_x, p.x() + 200.0);
    }
    for (const auto& [id, p] : source_tri.positions) {
        (void)id;
        max_x = std::max(max_x, p.x() + 200.0);
        max_y = std::max(max_y, p.y() + 200.0);
    }
    for (const auto& [id, p] : target_tri.positions) {
        (void)id;
        max_x = std::max(max_x, p.x() + 200.0);
        max_y = std::max(max_y, p.y() + 200.0);
    }

    Task10p10Scenario scenario;
    scenario.id = "route_b_tdd5_closed_loop";
    scenario.width_m = max_x;
    scenario.height_m = max_y;
    scenario.mobile_ids = source_tri.mobile_ids;
    for (NodeId id : scenario.mobile_ids) {
        scenario.mobile_positions.push_back(source_tri.positions.at(id));
    }
    scenario.fixed_positions = source_tri.beacons;
    scenario.initial_topology = source_tri.initialTopologyEdges();

    json settings = task10p10SwarmSettings(scenario, SolverProfile::OpenSource);
    std::vector<int> base_indices;
    for (std::size_t i = 0; i < source_tri.beacon_ids.size(); ++i) {
        base_indices.push_back(static_cast<int>(i));
    }
    settings["formation"]["bases-id"] = {base_indices};
    settings["world"]["spacing"] = 200.0;
    settings["execute"]["time-total"] = exec_cfg.max_sim_time_s + 10.0;
    settings["execute"]["time-step"] = exec_cfg.dt_s;

    Swarm swarm(settings);

    GrandFinaleSwarmAdapterConfig cfg;
    cfg.solver_profile = SolverProfile::OpenSource;
    cfg.dt_s = exec_cfg.dt_s;
    cfg.minimum_dwell_s = exec_cfg.dt_s;
    cfg.acceleration_half_box = exec_cfg.a_max_mps2;
    cfg.collision_distance_m = exec_cfg.d_safe_m;
    cfg.reference_distance_m = exec_cfg.d_dep_m;
    cfg.add_reference_distance_m = exec_cfg.d_dep_m - 1.0;
    cfg.reference_uncertainty_m = 0.0;
    cfg.uncertainty_sigma = 3.0;
    cfg.uncertainty_scale_mode = UncertaintyScaleMode::TraceRoot;
    cfg.axis_velocity_limit_mps = 25.0;
    cfg.axis_velocity_cbf_gain = 5.0;
    cfg.estimator_acceleration_variance = 0.25 * 0.25;
    cfg.initial_position_variance_m2 = 0.04;
    cfg.initial_velocity_variance_m2 = 0.01;
    cfg.maximum_range_aoi_s = exec_cfg.maximum_range_aoi_s;
    cfg.normalized_range_innovation_gate_sigma = 4.0;
    cfg.minimal_runtime_gating = true;
    cfg.tau_margin_gate_enabled = true;
    cfg.tau_analytic_first_order = true;
    cfg.cbf2026_uwb_channel.enabled = true;
    cfg.cbf2026_uwb_channel.d_r_m = exec_cfg.uwb_d_r_m;
    cfg.cbf2026_uwb_channel.sigma0_m = exec_cfg.uwb_sigma0_m;
    cfg.cbf2026_uwb_channel.d_max_m = exec_cfg.uwb_d_max_m;
    cfg.cbf2026_uwb_channel.link_seed = exec_cfg.link_seed;
    cfg.range_measurement_scope = exec_cfg.range_measurement_scope;
    cfg.sensor_radius_m = 400.0;
    cfg.coverage_footprint_kind = CoverageFootprintKind::ForwardSector;
    cfg.coverage_half_angle_rad = M_PI / 3.0;
    cfg.range_positive_bias_m = exec_cfg.range_positive_bias_m;
    cfg.plant_wind_disturbance_mps2 = exec_cfg.plant_wind_disturbance_mps2;
    cfg.beacon_truth_offsets_m = exec_cfg.beacon_truth_offsets_m;

    GrandFinaleSwarmAdapter adapter(
        swarm, scenario.mobile_ids, scenario.fixed_positions,
        scenario.initial_topology, cfg);
    if (!adapter.initializeStageZero().initialized) {
        res.reason = "stage_zero_init_failed";
        return res;
    }

    RouteBClosedLoopAuditor auditor(
        scenario.mobile_ids, scenario.fixed_positions,
        exec_cfg.d_safe_m, exec_cfg.d_dep_m, 25.0, 3.0);

    RouteBPipelinedSession session(
        swarm, adapter, source_tri, target_tri, moves, exec_cfg, &auditor);

    const std::size_t max_ticks = static_cast<std::size_t>(
        std::llround(exec_cfg.max_sim_time_s / exec_cfg.dt_s));
    const std::size_t scale_every = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(exec_cfg.scale_series_interval_s / exec_cfg.dt_s)));
    const std::size_t traj_every = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(exec_cfg.trajectory_sample_interval_s / exec_cfg.dt_s)));

    auto record_traj = [&](double t_s) {
        RouteBTrajectorySample ts;
        ts.time_s = t_s;
        for (std::size_t i = 0; i < scenario.mobile_ids.size(); ++i) {
            const Point pt = swarm.robots[i]->model->xy();
            ts.positions[scenario.mobile_ids[i]] = Eigen::Vector2d(pt.x, pt.y);
        }
        res.trajectory_samples.push_back(std::move(ts));
    };

    if (exec_cfg.record_trajectory_samples) {
        record_traj(0.0);
    }

    for (std::size_t tick = 0; tick < max_ticks; ++tick) {
        const bool done = session.stepOnce();
        const double t_s = static_cast<double>(session.ticks()) * exec_cfg.dt_s;
        if (exec_cfg.record_scale_radius_series &&
            auditor.observationCount() > 0 &&
            (session.ticks() % scale_every == 0 || done || session.finished())) {
            res.scale_radius_series.push_back({
                t_s,
                auditor.lastMaxPositionScaleRadiusM(),
                auditor.lastMeanPositionScaleRadiusM(),
                auditor.runningPositionRmseM()});
        }
        if (exec_cfg.record_trajectory_samples &&
            (session.ticks() % traj_every == 0 || done || session.finished())) {
            record_traj(t_s);
        }
        if (done) break;
        if (session.finished()) break;
    }

    res.completed = session.completed();
    res.reason = session.reason();
    res.makespan_s = session.completed() ? session.makespanS() : exec_cfg.max_sim_time_s;
    if (!res.completed && res.reason.empty()) res.reason = "timeout";
    res.ticks = session.ticks();
    res.static_parent_violations = session.staticParentViolations();
    res.dag_violations = session.dagViolations();
    res.max_concurrent_moving = session.maxConcurrentMoving();
    res.total_mbb_swaps = session.totalMbbSwaps();
    res.mbb_retries = session.mbbRetries();
    res.min_inter_moving_distance_m = session.minInterMovingDistanceM();
    res.audit = auditor.summarize();

    const auto final_snap = adapter.runtimeSnapshot();
    for (std::size_t i = 0; i < scenario.mobile_ids.size(); ++i) {
        const NodeId nid = scenario.mobile_ids[i];
        const Point pt = swarm.robots[i]->model->xy();
        const double err = (Eigen::Vector2d(pt.x, pt.y) - target_tri.positions.at(nid)).norm();
        res.final_position_error_m = std::max(res.final_position_error_m, err);
    }

    std::set<std::string> actual_edges, target_edges;
    for (const auto& e : final_snap.topology) actual_edges.insert(e.id());
    for (const auto& e : target_tri.initialTopologyEdges()) target_edges.insert(e.id());
    res.final_topology_matched = (actual_edges == target_edges);
    res.gantt = session.gantt();
    if (exec_cfg.record_timing_stats) {
        res.per_uav_qp_solve_s = session.perUavQpSolveS();
        res.per_uav_qp_total_s = session.perUavQpTotalS();
        res.estimator_samples_s = session.estimatorSamplesS();
        res.tick_total_samples_s = session.tickTotalSamplesS();
        res.total_scheduled_range_packets = session.totalScheduledRangePackets();
        res.total_accepted_range_packets = session.totalAcceptedRangePackets();
    }
    res.wall_clock_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_t0).count();
    return res;
}

}  // namespace gf
