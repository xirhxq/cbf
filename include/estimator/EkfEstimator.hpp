#ifndef CBF_ESTIMATOR_EKF_ESTIMATOR_HPP
#define CBF_ESTIMATOR_EKF_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "ComputingGeometry/Point.hpp"

// Basic extended Kalman filter for range-only cooperative localization.
//
// This is the C++ counterpart of scripts/diagnostics/ekf_estimator_service.py
// (the paper's estimator-in-loop EKF). It runs fully in-process, eliminating
// the file-IPC multi-process timing of the Python resident service.
//
// Per robot, a 2D position estimate (mean) and 2x2 covariance are maintained.
// Each control frame:
//   1. predict:  mean += v_cmd * dt,  cov += (q*dt)^2 * I
//   2. for each reference (base or lower-index UAV anchor), generate a noisy
//      range measurement (true distance + sigma(d)*Gaussian, admitted with
//      link-budget availability p(d)) and apply a sequential scalar range
//      update with 3sigma innovation gating.
//   3. epsilon = 3 * sqrt(lambda_max(cov))
//
// Parameters mirror the estimator spec in the MBZIRC handoff §3.1:
// sigma(d) = sigma0 * (1 + (d/d0)^2), p(d) = 1/(1+(d/d0)^2), kappa, q, p0, dt.
class EkfEstimator {
public:
    struct Params {
        double sigma0 = 0.5;            // short-range ranging noise std (m)
        double range0 = 850.0;          // distance scale for noise/availability
        double process_noise_mps = 1.0; // process-noise speed q (m/s)
        double p0_std = 1.0;            // initial per-axis position std (m)
        double dt = 0.5;                // control/estimate period (s)
        double innovation_gate = 3.0;   // reject measurements beyond N sigma
        double anchor_covariance_scale = 3.0;  // kappa (anchor cov scaling)
        unsigned seed = 2026081301u;    // deterministic ranging-noise seed
    };

    // One per-robot reference (base station or UAV anchor).
    struct Reference {
        bool is_base;        // true = base, false = UAV anchor
        int identifier;      // base index or robot id
        Point position;      // anchor truth position (this frame)
    };

    struct RobotState {
        Eigen::Vector2d mean;
        Eigen::Matrix2d cov;
        int updates = 0;     // accepted measurements this frame
        int rejected = 0;
    };

    EkfEstimator(const std::vector<Point>& bases,
                 const std::vector<Point>& deployment_positions,
                 const Params& params)
        : bases_(bases), params_(params), rng_(params.seed) {
        states_.resize(deployment_positions.size());
        for (size_t i = 0; i < deployment_positions.size(); ++i) {
            states_[i].mean = Eigen::Vector2d(
                deployment_positions[i].x, deployment_positions[i].y);
            states_[i].cov = Eigen::Matrix2d::Identity() * params_.p0_std * params_.p0_std;
        }
    }

    // Advance one frame.
    //   references_by_robot[i] = references for robot index i (robot id = i+1).
    //   velocities[i] = (vx, vy) used for the predict step. To avoid a positive
    //     feedback loop (CBF command -> EKF predict -> estimate -> CBF command),
    //     callers should pass the ACTUAL velocity (e.g. truth frame-to-frame
    //     difference), not the commanded velocity.
    //   truth_positions[i] = robot i's TRUE position this frame (used only to
    //     generate the noisy range measurement; the EKF state still tracks the
    //     estimate). This mirrors the Python design where build_ekf_raw_references
    //     uses truth distances + noise, separate from the EKF update.
    void step(const std::vector<std::vector<Reference>>& references_by_robot,
              const std::vector<std::pair<double, double>>& velocities,
              const std::vector<Point>& truth_positions) {
        const double dt = params_.dt;
        const double proc = (params_.process_noise_mps * dt) *
                            (params_.process_noise_mps * dt);
        const Eigen::Matrix2d procCov = Eigen::Matrix2d::Identity() * proc;

        for (size_t i = 0; i < states_.size(); ++i) {
            RobotState& s = states_[i];
            Eigen::Vector2d mean = s.mean;
            Eigen::Matrix2d cov = s.cov + procCov;

            if (i < velocities.size()) {
                mean(0) += velocities[i].first * dt;
                mean(1) += velocities[i].second * dt;
            }

            Eigen::Vector2d truth(i < truth_positions.size()
                ? Eigen::Vector2d(truth_positions[i].x, truth_positions[i].y)
                : mean);

            int updates = 0, rejected = 0;
            if (i < references_by_robot.size()) {
                for (const Reference& ref : references_by_robot[i]) {
                    Eigen::Vector2d anchorMean;
                    Eigen::Matrix2d anchorCov = Eigen::Matrix2d::Zero();
                    Eigen::Vector2d anchorTruth;
                    if (ref.is_base) {
                        anchorMean << ref.position.x, ref.position.y;
                        anchorTruth = anchorMean;
                    } else {
                        int idx = ref.identifier - 1;
                        if (idx < 0 || idx >= (int)states_.size()) continue;
                        anchorMean = states_[idx].mean;
                        anchorCov = states_[idx].cov;
                        // Truth anchor position is in ref.position (passed by caller).
                        anchorTruth << ref.position.x, ref.position.y;
                    }

                    // True range (truth -> truth) drives the noisy measurement.
                    double trueRange = (truth - anchorTruth).norm();
                    if (trueRange < 1e-6) continue;
                    double sigma = rangingSigma(trueRange);
                    double avail = availability(trueRange);
                    if (uniform_(rng_) > avail) continue;
                    double meas = trueRange + normal_(rng_) * sigma;

                    // Predicted range from the EKF estimate (mean -> anchorMean).
                    Eigen::Vector2d diff = mean - anchorMean;
                    double predRange = diff.norm();
                    if (predRange < 1e-6) continue;
                    Eigen::Vector2d direction = diff / predRange;

                    double innovation = meas - predRange;
                    double measCov = sigma * sigma +
                        params_.anchor_covariance_scale *
                        params_.anchor_covariance_scale *
                        direction.dot(anchorCov * direction);
                    double innovCov = direction.dot(cov * direction) + measCov;

                    if (std::abs(innovation) >
                        params_.innovation_gate * std::sqrt(innovCov)) {
                        ++rejected;
                        continue;
                    }
                    Eigen::Vector2d gain = (cov * direction) / innovCov;
                    mean = mean + gain * innovation;
                    cov = (Eigen::Matrix2d::Identity() - gain * direction.transpose()) * cov;
                    cov = 0.5 * (cov + cov.transpose().eval());
                    ++updates;
                }
            }

            s.mean = mean;
            s.cov = cov;
            s.updates = updates;
            s.rejected = rejected;
        }
    }

    void reset(const std::vector<Point>& deployment_positions) {
        for (size_t i = 0; i < states_.size() && i < deployment_positions.size(); ++i) {
            states_[i].mean = Eigen::Vector2d(
                deployment_positions[i].x, deployment_positions[i].y);
            states_[i].cov = Eigen::Matrix2d::Identity() * params_.p0_std * params_.p0_std;
            states_[i].updates = 0;
            states_[i].rejected = 0;
        }
        rng_.seed(params_.seed);
    }

    const RobotState& state(int robot_id) const { return states_.at(robot_id - 1); }
    Point estimate(int robot_id) const {
        return Point(states_.at(robot_id - 1).mean(0),
                     states_.at(robot_id - 1).mean(1));
    }
    double epsilon(int robot_id) const {
        const Eigen::Matrix2d& cov = states_.at(robot_id - 1).cov;
        // lambda_max of a symmetric 2x2 [[a,b],[b,d]]:
        // (a+d)/2 + sqrt(((a-d)/2)^2 + b^2)
        double a = cov(0, 0), b = cov(0, 1), d = cov(1, 1);
        double lam = 0.5 * (a + d) + std::sqrt(0.25 * (a - d) * (a - d) + b * b);
        return 3.0 * std::sqrt(std::max(lam, 0.0));
    }
    bool fresh(int robot_id) const { return states_.at(robot_id - 1).updates > 0; }
    const std::vector<Point>& bases() const { return bases_; }

private:
    double rangingSigma(double distance) const {
        double r = distance / params_.range0;
        return params_.sigma0 * (1.0 + r * r);
    }
    double availability(double distance) const {
        double r = distance / params_.range0;
        return 1.0 / (1.0 + r * r);
    }

    std::vector<Point> bases_;
    Params params_;
    std::vector<RobotState> states_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};
};

#endif  // CBF_ESTIMATOR_EKF_ESTIMATOR_HPP
