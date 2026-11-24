#pragma once

#include "Swarm.hpp"
#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/ReferenceEligibility.hpp"
#include "grand_finale/Types.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace gf {

struct RouteBClosedLoopAuditSummary {
    std::size_t observation_count = 0;
    std::size_t total_agent_samples = 0;
    std::size_t collision_violations = 0;
    std::size_t reference_violations = 0;
    std::size_t axis_speed_violations = 0;
    double min_inter_node_distance_m = std::numeric_limits<double>::infinity();
    double max_active_reference_distance_m = 0.0;
    double max_axis_speed_mps = 0.0;
    double max_position_error_m = 0.0;
    double max_velocity_error_mps = 0.0;
    double position_rmse_m = 0.0;
    double velocity_rmse_mps = 0.0;
    double peak_position_scale_radius_m = 0.0;
    double mean_position_scale_radius_m = 0.0;
    double peak_position_covariance_trace_m2 = 0.0;
    double overall_envelope_coverage = 0.0;
    double overall_velocity_envelope_coverage = 0.0;
    std::map<NodeId, double> per_agent_position_rmse_m;
    std::map<NodeId, double> per_agent_velocity_rmse_mps;
    std::map<NodeId, double> per_agent_peak_position_scale_radius_m;
    std::map<NodeId, double> per_agent_envelope_coverage;
    std::map<NodeId, double> per_agent_velocity_envelope_coverage;
};

class RouteBClosedLoopAuditor {
public:
    RouteBClosedLoopAuditor(
        std::vector<NodeId> mobile_ids,
        std::map<NodeId, Eigen::Vector2d> fixed_positions,
        double collision_distance_m,
        double reference_distance_m,
        double axis_velocity_limit_mps,
        double uncertainty_sigma = 3.0,
        UncertaintyScaleMode scale_mode = UncertaintyScaleMode::TraceRoot)
        : mobile_ids_(std::move(mobile_ids)),
          fixed_positions_(std::move(fixed_positions)),
          collision_distance_m_(collision_distance_m),
          reference_distance_m_(reference_distance_m),
          axis_velocity_limit_mps_(axis_velocity_limit_mps),
          uncertainty_sigma_(uncertainty_sigma),
          scale_mode_(scale_mode) {
        if (mobile_ids_.empty() ||
            !std::isfinite(collision_distance_m_) || collision_distance_m_ <= 0.0 ||
            !std::isfinite(reference_distance_m_) || reference_distance_m_ <= 0.0 ||
            !std::isfinite(axis_velocity_limit_mps_) || axis_velocity_limit_mps_ <= 0.0 ||
            !std::isfinite(uncertainty_sigma_) || uncertainty_sigma_ < 0.0) {
            throw std::invalid_argument("invalid RouteBClosedLoopAuditor configuration");
        }
        std::sort(mobile_ids_.begin(), mobile_ids_.end());
        for (NodeId id : mobile_ids_) {
            pos_sq_err_sum_[id] = 0.0;
            vel_sq_err_sum_[id] = 0.0;
            pos_scale_sum_[id] = 0.0;
            pos_scale_peak_[id] = 0.0;
            pos_envelope_hits_[id] = 0;
            vel_envelope_hits_[id] = 0;
        }
    }

    void observe(const Swarm& swarm, const GrandFinaleRuntimeSnapshot& snapshot) {
        std::map<NodeId, Eigen::Vector2d> truth_pos;
        std::map<NodeId, Eigen::Vector2d> truth_vel;
        for (const auto& robot : swarm.robots) {
            const NodeId id = static_cast<NodeId>(robot->id);
            const Point pt = robot->model->xy();
            const Eigen::VectorXd vel = robot->model->getVelocity();
            truth_pos[id] = Eigen::Vector2d(pt.x, pt.y);
            truth_vel[id] = Eigen::Vector2d(vel(0), vel(1));
        }

        auto truth_node_pos = [&](NodeId id) -> const Eigen::Vector2d& {
            const auto mit = truth_pos.find(id);
            if (mit != truth_pos.end()) return mit->second;
            return fixed_positions_.at(id);
        };

        ++observation_count_;
        double step_max_scale = 0.0;
        double step_sum_scale = 0.0;

        for (std::size_t i = 0; i < mobile_ids_.size(); ++i) {
            const NodeId id = mobile_ids_[i];
            const Eigen::Vector2d p = truth_pos.at(id);
            const Eigen::Vector2d v = truth_vel.at(id);

            const double axis_speed = std::max(std::abs(v.x()), std::abs(v.y()));
            max_axis_speed_mps_ = std::max(max_axis_speed_mps_, axis_speed);
            if (axis_speed > axis_velocity_limit_mps_ + 1.0e-9) {
                ++axis_speed_violations_;
            }

            const std::size_t est_idx = detail::mobileIndex(snapshot.estimate, id);
            const Eigen::Vector2d p_hat =
                snapshot.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(est_idx));
            const Eigen::Vector2d v_hat =
                snapshot.estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(est_idx) + 2);

            const double pos_err = (p_hat - p).norm();
            const double vel_err = (v_hat - v).norm();
            max_position_error_m_ = std::max(max_position_error_m_, pos_err);
            max_velocity_error_mps_ = std::max(max_velocity_error_mps_, vel_err);
            pos_sq_err_sum_[id] += pos_err * pos_err;
            vel_sq_err_sum_[id] += vel_err * vel_err;
            step_sq_err_sum_ += pos_err * pos_err;

            const double pos_radius = detail::positionScaleRadius(
                snapshot.estimate, id, uncertainty_sigma_, scale_mode_);
            const double vel_radius = detail::velocityScaleRadius(
                snapshot.estimate, id, uncertainty_sigma_, scale_mode_);
            const double pos_trace = detail::positionTrace(snapshot.estimate, id);
            pos_scale_sum_[id] += pos_radius;
            pos_scale_peak_[id] = std::max(pos_scale_peak_[id], pos_radius);
            peak_pos_scale_m_ = std::max(peak_pos_scale_m_, pos_radius);
            peak_pos_trace_m2_ = std::max(peak_pos_trace_m2_, pos_trace);
            step_max_scale = std::max(step_max_scale, pos_radius);
            step_sum_scale += pos_radius;
            if (pos_err <= pos_radius + 1.0e-12) {
                ++pos_envelope_hits_[id];
            }
            if (vel_err <= vel_radius + 1.0e-12) {
                ++vel_envelope_hits_[id];
            }

            for (std::size_t j = i + 1; j < mobile_ids_.size(); ++j) {
                const double dist = (p - truth_pos.at(mobile_ids_[j])).norm();
                min_inter_node_distance_m_ = std::min(min_inter_node_distance_m_, dist);
                if (dist < collision_distance_m_ - 1.0e-9) {
                    ++collision_violations_;
                }
            }
            for (const auto& [fixed_id, fixed_pt] : fixed_positions_) {
                (void)fixed_id;
                const double dist = (p - fixed_pt).norm();
                min_inter_node_distance_m_ = std::min(min_inter_node_distance_m_, dist);
                if (dist < collision_distance_m_ - 1.0e-9) {
                    ++collision_violations_;
                }
            }
        }
        last_max_pos_scale_m_ = step_max_scale;
        last_mean_pos_scale_m_ = step_sum_scale / static_cast<double>(mobile_ids_.size());

        for (const DirectedEdge& edge : snapshot.topology) {
            const double dist =
                (truth_node_pos(edge.owner) - truth_node_pos(edge.reference)).norm();
            max_active_reference_distance_m_ =
                std::max(max_active_reference_distance_m_, dist);
            if (dist > reference_distance_m_ + 1.0e-9) {
                ++reference_violations_;
            }
        }
    }

    RouteBClosedLoopAuditSummary summarize() const {
        RouteBClosedLoopAuditSummary summary;
        summary.observation_count = observation_count_;
        summary.total_agent_samples = observation_count_ * mobile_ids_.size();
        summary.collision_violations = collision_violations_;
        summary.reference_violations = reference_violations_;
        summary.axis_speed_violations = axis_speed_violations_;
        summary.min_inter_node_distance_m = min_inter_node_distance_m_;
        summary.max_active_reference_distance_m = max_active_reference_distance_m_;
        summary.max_axis_speed_mps = max_axis_speed_mps_;
        summary.max_position_error_m = max_position_error_m_;
        summary.max_velocity_error_mps = max_velocity_error_mps_;

        if (observation_count_ == 0) {
            return summary;
        }

        double total_pos_sq = 0.0;
        double total_vel_sq = 0.0;
        double total_pos_scale = 0.0;
        std::size_t total_pos_hits = 0;
        std::size_t total_vel_hits = 0;
        const double n_obs = static_cast<double>(observation_count_);
        for (NodeId id : mobile_ids_) {
            const double pos_sq = pos_sq_err_sum_.at(id);
            const double vel_sq = vel_sq_err_sum_.at(id);
            const std::size_t pos_hits = pos_envelope_hits_.at(id);
            const std::size_t vel_hits = vel_envelope_hits_.at(id);
            total_pos_sq += pos_sq;
            total_vel_sq += vel_sq;
            total_pos_scale += pos_scale_sum_.at(id);
            total_pos_hits += pos_hits;
            total_vel_hits += vel_hits;
            summary.per_agent_position_rmse_m[id] = std::sqrt(pos_sq / n_obs);
            summary.per_agent_velocity_rmse_mps[id] = std::sqrt(vel_sq / n_obs);
            summary.per_agent_peak_position_scale_radius_m[id] = pos_scale_peak_.at(id);
            summary.per_agent_envelope_coverage[id] =
                static_cast<double>(pos_hits) / n_obs;
            summary.per_agent_velocity_envelope_coverage[id] =
                static_cast<double>(vel_hits) / n_obs;
        }
        const double total_samples = static_cast<double>(summary.total_agent_samples);
        summary.position_rmse_m = std::sqrt(total_pos_sq / total_samples);
        summary.velocity_rmse_mps = std::sqrt(total_vel_sq / total_samples);
        summary.peak_position_scale_radius_m = peak_pos_scale_m_;
        summary.mean_position_scale_radius_m = total_pos_scale / total_samples;
        summary.peak_position_covariance_trace_m2 = peak_pos_trace_m2_;
        summary.overall_envelope_coverage =
            static_cast<double>(total_pos_hits) / total_samples;
        summary.overall_velocity_envelope_coverage =
            static_cast<double>(total_vel_hits) / total_samples;
        return summary;
    }

    std::size_t observationCount() const { return observation_count_; }
    double lastMaxPositionScaleRadiusM() const { return last_max_pos_scale_m_; }
    double lastMeanPositionScaleRadiusM() const { return last_mean_pos_scale_m_; }
    double runningPositionRmseM() const {
        if (observation_count_ == 0) return 0.0;
        return std::sqrt(
            step_sq_err_sum_ /
            static_cast<double>(observation_count_ * mobile_ids_.size()));
    }

private:
    std::vector<NodeId> mobile_ids_;
    std::map<NodeId, Eigen::Vector2d> fixed_positions_;
    double collision_distance_m_ = 10.0;
    double reference_distance_m_ = 850.0;
    double axis_velocity_limit_mps_ = 25.0;
    double uncertainty_sigma_ = 3.0;
    UncertaintyScaleMode scale_mode_ = UncertaintyScaleMode::TraceRoot;

    std::size_t observation_count_ = 0;
    std::size_t collision_violations_ = 0;
    std::size_t reference_violations_ = 0;
    std::size_t axis_speed_violations_ = 0;
    double min_inter_node_distance_m_ = std::numeric_limits<double>::infinity();
    double max_active_reference_distance_m_ = 0.0;
    double max_axis_speed_mps_ = 0.0;
    double max_position_error_m_ = 0.0;
    double max_velocity_error_mps_ = 0.0;
    double peak_pos_scale_m_ = 0.0;
    double peak_pos_trace_m2_ = 0.0;
    double last_max_pos_scale_m_ = 0.0;
    double last_mean_pos_scale_m_ = 0.0;
    double step_sq_err_sum_ = 0.0;
    std::map<NodeId, double> pos_sq_err_sum_;
    std::map<NodeId, double> vel_sq_err_sum_;
    std::map<NodeId, double> pos_scale_sum_;
    std::map<NodeId, double> pos_scale_peak_;
    std::map<NodeId, std::size_t> pos_envelope_hits_;
    std::map<NodeId, std::size_t> vel_envelope_hits_;
};

}  // namespace gf
