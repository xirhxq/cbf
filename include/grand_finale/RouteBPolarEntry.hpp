#pragma once

#include "grand_finale/CentralizedEkfOracle.hpp"
#include "grand_finale/ReferenceEligibility.hpp"
#include "grand_finale/Types.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace gf {

struct RouteBPolarEntryConfig {
    double accel_budget_mps2 = 2.0;
    double axis_speed_budget_mps = 16.0;
    double brake_accel_mps2 = 1.8;
    double kp = 0.6;
    double kd = 1.4;
    double arrival_pos_tol_m = 2.5;
    double arrival_speed_tol_mps = 0.4;
    double max_acceleration_cmd_mps2 = 4.0;
    double velocity_capture_tol_mps = 0.25;
    double min_alpha = 0.25;
};

struct RouteBPolarEntrySample {
    double s = 0.0;
    double alpha = 1.0;
    double theta_rad = 0.0;
    std::map<NodeId, Eigen::Vector2d> positions;
    std::map<NodeId, Eigen::Vector2d> velocities;
    std::map<NodeId, Eigen::Vector2d> accelerations;
};

class RouteBPolarEntryPlanner {
public:
    RouteBPolarEntryPlanner(
        const std::map<NodeId, Eigen::Vector2d>& start_positions,
        const std::map<NodeId, Eigen::Vector2d>& start_velocities,
        const std::map<NodeId, Eigen::Vector2d>& target_positions,
        const Eigen::Vector2d& pivot,
        RouteBPolarEntryConfig config = {})
        : config_(config),
          pivot_(pivot),
          target_positions_(target_positions) {
        if (!pivot_.allFinite() ||
            !std::isfinite(config_.accel_budget_mps2) || config_.accel_budget_mps2 <= 0.0 ||
            !std::isfinite(config_.axis_speed_budget_mps) || config_.axis_speed_budget_mps <= 0.0 ||
            !std::isfinite(config_.brake_accel_mps2) || config_.brake_accel_mps2 <= 0.0 ||
            !std::isfinite(config_.min_alpha) || config_.min_alpha <= 0.0 ||
            start_positions.empty() || start_positions.size() != target_positions_.size()) {
            throw std::invalid_argument("invalid RouteBPolarEntryPlanner configuration");
        }

        double max_v_axis = 0.0;
        for (const auto& [id, q] : target_positions_) {
            const auto sp = start_positions.find(id);
            if (sp == start_positions.end() || !sp->second.allFinite() || !q.allFinite()) {
                throw std::invalid_argument("mismatch or non-finite entry waypoint");
            }
            ids_.push_back(id);
            brake_start_pos_[id] = sp->second;
            Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
            const auto sv = start_velocities.find(id);
            if (sv != start_velocities.end()) {
                if (!sv->second.allFinite()) {
                    throw std::invalid_argument("non-finite start velocity");
                }
                v0 = sv->second;
            }
            brake_start_vel_[id] = v0;
            max_v_axis = std::max(max_v_axis, v0.cwiseAbs().maxCoeff());
        }

        if (max_v_axis > 1.0e-9) {
            brake_duration_s_ = max_v_axis / config_.brake_accel_mps2;
        }
        std::map<NodeId, Eigen::Vector2d> homotopy_start;
        for (NodeId id : ids_) {
            homotopy_start[id] =
                brake_start_pos_[id] + 0.5 * brake_start_vel_[id] * brake_duration_s_;
        }

        // Centroid-centered 2D Procrustes similarity fit so anchor-pivoted pure
        // similarity has c0 = 0 and delta_i = 0, and translated+rotated+scaled
        // formations still preserve exact pairwise interior angles.
        Eigen::Vector2d mean_u = Eigen::Vector2d::Zero();
        Eigen::Vector2d mean_w = Eigen::Vector2d::Zero();
        for (NodeId id : ids_) {
            mean_u += target_positions_.at(id) - pivot_;
            mean_w += homotopy_start.at(id) - pivot_;
        }
        mean_u /= static_cast<double>(ids_.size());
        mean_w /= static_cast<double>(ids_.size());

        double num_cos = 0.0;
        double num_sin = 0.0;
        double denom = 0.0;
        for (NodeId id : ids_) {
            const Eigen::Vector2d du = (target_positions_.at(id) - pivot_) - mean_u;
            const Eigen::Vector2d dw = (homotopy_start.at(id) - pivot_) - mean_w;
            num_cos += du.x() * dw.x() + du.y() * dw.y();
            num_sin += du.x() * dw.y() - du.y() * dw.x();
            denom += du.squaredNorm();
        }
        if (denom > 1.0e-12) {
            const double a = num_cos / denom;
            const double b = num_sin / denom;
            alpha0_ = std::max(config_.min_alpha, std::hypot(a, b));
            theta0_rad_ = std::atan2(b, a);
        } else {
            alpha0_ = 1.0;
            theta0_rad_ = 0.0;
        }

        const Eigen::Matrix2d r0 = rotation2d(theta0_rad_);
        translation0_ = mean_w - alpha0_ * (r0 * mean_u);
        for (NodeId id : ids_) {
            const Eigen::Vector2d u = target_positions_.at(id) - pivot_;
            const Eigen::Vector2d w = homotopy_start.at(id) - pivot_;
            deltas_[id] = w - (translation0_ + alpha0_ * (r0 * u));
        }

        double m1 = 0.0;
        double m2 = 0.0;
        const double d_alpha = 1.0 - alpha0_;
        const double d_theta = -theta0_rad_;
        for (int k = 0; k <= 64; ++k) {
            const double s = static_cast<double>(k) / 64.0;
            const double alpha = (1.0 - s) * alpha0_ + s;
            const double theta = (1.0 - s) * theta0_rad_;
            const Eigen::Matrix2d rot = rotation2d(theta);
            Eigen::Matrix2d j_mat;
            j_mat << 0.0, -1.0, 1.0, 0.0;
            const Eigen::Matrix2d d1_mat =
                (d_alpha * Eigen::Matrix2d::Identity() + alpha * d_theta * j_mat) * rot;
            const Eigen::Matrix2d d2_mat =
                (2.0 * d_alpha * d_theta * j_mat - alpha * d_theta * d_theta * Eigen::Matrix2d::Identity()) * rot;
            for (NodeId id : ids_) {
                const Eigen::Vector2d u = target_positions_.at(id) - pivot_;
                const Eigen::Vector2d rp = -translation0_ - deltas_.at(id) + d1_mat * u;
                const Eigen::Vector2d rpp = d2_mat * u;
                m1 = std::max(m1, rp.cwiseAbs().maxCoeff());
                m2 = std::max(m2, rpp.cwiseAbs().maxCoeff());
            }
        }

        if (m1 > 1.0e-9) {
            s_ddot_max_ = (0.50 * config_.accel_budget_mps2) / m1;
            double v_cap = config_.axis_speed_budget_mps / m1;
            if (m2 > 1.0e-12) {
                v_cap = std::min(v_cap, std::sqrt((0.45 * config_.accel_budget_mps2) / m2));
            }
            s_dot_max_ = std::max(1.0e-6, v_cap);
            const double t_ramp = s_dot_max_ / s_ddot_max_;
            const double dist_ramp = s_dot_max_ * t_ramp;
            if (dist_ramp >= 1.0) {
                // Triangular profile
                s_dot_peak_ = std::sqrt(s_ddot_max_);
                t_accel_s_ = s_dot_peak_ / s_ddot_max_;
                t_cruise_s_ = 0.0;
                homotopy_duration_s_ = 2.0 * t_accel_s_;
            } else {
                // Trapezoidal profile
                s_dot_peak_ = s_dot_max_;
                t_accel_s_ = t_ramp;
                t_cruise_s_ = (1.0 - dist_ramp) / s_dot_peak_;
                homotopy_duration_s_ = 2.0 * t_accel_s_ + t_cruise_s_;
            }
        } else {
            homotopy_duration_s_ = 0.0;
        }
    }

    double brakeDurationS() const { return brake_duration_s_; }
    double homotopyDurationS() const { return homotopy_duration_s_; }
    double totalDurationS() const { return brake_duration_s_ + homotopy_duration_s_; }
    double alpha0() const { return alpha0_; }
    double theta0Rad() const { return theta0_rad_; }

    RouteBPolarEntrySample sampleAt(double t_s) const {
        RouteBPolarEntrySample sample;
        if (t_s < 0.0) t_s = 0.0;
        if (brake_duration_s_ > 0.0 && t_s < brake_duration_s_) {
            sample.s = 0.0;
            sample.alpha = alpha0_;
            sample.theta_rad = theta0_rad_;
            for (NodeId id : ids_) {
                const Eigen::Vector2d& p0 = brake_start_pos_.at(id);
                const Eigen::Vector2d& v0 = brake_start_vel_.at(id);
                const Eigen::Vector2d a = -v0 / brake_duration_s_;
                sample.positions[id] = p0 + v0 * t_s + 0.5 * a * t_s * t_s;
                sample.velocities[id] = v0 + a * t_s;
                sample.accelerations[id] = a;
            }
            return sample;
        }

        const double tau = t_s - brake_duration_s_;
        double s = 1.0;
        double s_dot = 0.0;
        double s_ddot = 0.0;
        if (homotopy_duration_s_ > 0.0 && tau < homotopy_duration_s_) {
            if (tau < t_accel_s_) {
                s_ddot = s_ddot_max_;
                s_dot = s_ddot_max_ * tau;
                s = 0.5 * s_ddot_max_ * tau * tau;
            } else if (tau < t_accel_s_ + t_cruise_s_) {
                const double dt_c = tau - t_accel_s_;
                s_ddot = 0.0;
                s_dot = s_dot_peak_;
                s = 0.5 * s_ddot_max_ * t_accel_s_ * t_accel_s_ + s_dot_peak_ * dt_c;
            } else {
                const double dt_d = homotopy_duration_s_ - tau;
                s_ddot = -s_ddot_max_;
                s_dot = s_ddot_max_ * dt_d;
                s = 1.0 - 0.5 * s_ddot_max_ * dt_d * dt_d;
            }
        }
        s = std::clamp(s, 0.0, 1.0);

        const double alpha = (1.0 - s) * alpha0_ + s;
        const double theta = (1.0 - s) * theta0_rad_;
        const double d_alpha = 1.0 - alpha0_;
        const double d_theta = -theta0_rad_;
        const Eigen::Matrix2d rot = rotation2d(theta);
        Eigen::Matrix2d j_mat;
        j_mat << 0.0, -1.0, 1.0, 0.0;
        const Eigen::Matrix2d d1_mat =
            (d_alpha * Eigen::Matrix2d::Identity() + alpha * d_theta * j_mat) * rot;
        const Eigen::Matrix2d d2_mat =
            (2.0 * d_alpha * d_theta * j_mat - alpha * d_theta * d_theta * Eigen::Matrix2d::Identity()) * rot;

        sample.s = s;
        sample.alpha = alpha;
        sample.theta_rad = theta;
        for (NodeId id : ids_) {
            const Eigen::Vector2d u = target_positions_.at(id) - pivot_;
            const Eigen::Vector2d& delta = deltas_.at(id);
            const Eigen::Vector2d rp = -translation0_ - delta + d1_mat * u;
            const Eigen::Vector2d rpp = d2_mat * u;
            sample.positions[id] =
                pivot_ + (1.0 - s) * translation0_ + alpha * (rot * u) + (1.0 - s) * delta;
            sample.velocities[id] = rp * s_dot;
            sample.accelerations[id] = rp * s_ddot + rpp * (s_dot * s_dot);
        }
        return sample;
    }

private:
    static Eigen::Matrix2d rotation2d(double rad) {
        const double c = std::cos(rad);
        const double s = std::sin(rad);
        Eigen::Matrix2d r;
        r << c, -s, s, c;
        return r;
    }

    RouteBPolarEntryConfig config_;
    Eigen::Vector2d pivot_;
    std::map<NodeId, Eigen::Vector2d> target_positions_;
    std::vector<NodeId> ids_;

    std::map<NodeId, Eigen::Vector2d> brake_start_pos_;
    std::map<NodeId, Eigen::Vector2d> brake_start_vel_;
    double brake_duration_s_ = 0.0;

    double alpha0_ = 1.0;
    double theta0_rad_ = 0.0;
    Eigen::Vector2d translation0_ = Eigen::Vector2d::Zero();
    std::map<NodeId, Eigen::Vector2d> deltas_;

    double s_ddot_max_ = 0.0;
    double s_dot_max_ = 0.0;
    double s_dot_peak_ = 0.0;
    double t_accel_s_ = 0.0;
    double t_cruise_s_ = 0.0;
    double homotopy_duration_s_ = 0.0;
};

class RouteBPolarEntryExecutor {
public:
    RouteBPolarEntryExecutor(
        const std::map<NodeId, Eigen::Vector2d>& start_positions,
        const std::map<NodeId, Eigen::Vector2d>& start_velocities,
        const std::map<NodeId, Eigen::Vector2d>& target_positions,
        const Eigen::Vector2d& pivot,
        RouteBPolarEntryConfig config = {})
        : config_(config),
          target_positions_(target_positions),
          pivot_(pivot),
          brake_planner_(start_positions, start_velocities, start_positions, pivot, config) {
        if (brake_planner_.brakeDurationS() <= 1.0e-9) {
            homotopy_started_ = true;
            homotopy_start_t_ = 0.0;
            homotopy_planner_.emplace(
                start_positions, std::map<NodeId, Eigen::Vector2d>{},
                target_positions_, pivot_, config_);
        }
    }

    std::map<NodeId, Eigen::Vector2d> nominalControls(
        const JointEstimateSnapshot& estimate,
        double elapsed_s) {
        if (!homotopy_started_) {
            double max_speed = 0.0;
            std::map<NodeId, Eigen::Vector2d> est_pos;
            for (std::size_t idx = 0; idx < estimate.mobile_ids.size(); ++idx) {
                const NodeId id = estimate.mobile_ids[idx];
                est_pos[id] = estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(idx));
                const Eigen::Vector2d v =
                    estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(idx) + 2);
                max_speed = std::max(max_speed, v.norm());
            }
            if (elapsed_s >= brake_planner_.brakeDurationS() &&
                max_speed <= config_.velocity_capture_tol_mps) {
                homotopy_started_ = true;
                homotopy_start_t_ = elapsed_s;
                homotopy_planner_.emplace(
                    est_pos, std::map<NodeId, Eigen::Vector2d>{},
                    target_positions_, pivot_, config_);
            }
        }

        const RouteBPolarEntrySample ref = homotopy_started_
            ? homotopy_planner_->sampleAt(std::max(0.0, elapsed_s - homotopy_start_t_))
            : brake_planner_.sampleAt(std::min(elapsed_s, brake_planner_.brakeDurationS()));

        std::map<NodeId, Eigen::Vector2d> commands;
        for (std::size_t idx = 0; idx < estimate.mobile_ids.size(); ++idx) {
            const NodeId id = estimate.mobile_ids[idx];
            const Eigen::Vector2d p =
                estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(idx));
            const Eigen::Vector2d v =
                estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(idx) + 2);
            const Eigen::Vector2d u =
                ref.accelerations.at(id) +
                config_.kp * (ref.positions.at(id) - p) +
                config_.kd * (ref.velocities.at(id) - v);
            commands[id] = u.cwiseMax(-config_.max_acceleration_cmd_mps2)
                             .cwiseMin(config_.max_acceleration_cmd_mps2);
        }
        return commands;
    }

    bool arrived(const JointEstimateSnapshot& estimate, double elapsed_s) const {
        if (!homotopy_started_) {
            return false;
        }
        if (elapsed_s - homotopy_start_t_ + 1.0e-9 < homotopy_planner_->homotopyDurationS()) {
            return false;
        }
        for (std::size_t idx = 0; idx < estimate.mobile_ids.size(); ++idx) {
            const NodeId id = estimate.mobile_ids[idx];
            const Eigen::Vector2d p =
                estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(idx));
            const Eigen::Vector2d v =
                estimate.mean.segment<2>(4 * static_cast<Eigen::Index>(idx) + 2);
            if ((p - target_positions_.at(id)).norm() > config_.arrival_pos_tol_m ||
                v.norm() > config_.arrival_speed_tol_mps) {
                return false;
            }
        }
        return true;
    }

    bool homotopyStarted() const { return homotopy_started_; }

private:
    RouteBPolarEntryConfig config_;
    std::map<NodeId, Eigen::Vector2d> target_positions_;
    Eigen::Vector2d pivot_;
    RouteBPolarEntryPlanner brake_planner_;
    bool homotopy_started_ = false;
    double homotopy_start_t_ = 0.0;
    std::optional<RouteBPolarEntryPlanner> homotopy_planner_;
};

}  // namespace gf
