#pragma once

#include "grand_finale/RangeBatch.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

namespace gf {

struct MobileEstimate {
    NodeId id;
    Eigen::Vector4d mean;
    Eigen::Matrix4d covariance;
};

struct JointEstimateSnapshot {
    std::vector<NodeId> mobile_ids;
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;
    std::map<NodeId, Eigen::Vector2d> fixed_positions;

    Eigen::Vector2d fixed_position(NodeId id) const {
        return fixed_positions.at(id);
    }
};

class CentralizedEkfOracle {
public:
    CentralizedEkfOracle(
        std::vector<MobileEstimate> mobile_estimates,
        std::map<NodeId, Eigen::Vector2d> fixed_positions)
        : fixed_positions_(std::move(fixed_positions)) {
        std::sort(
            mobile_estimates.begin(), mobile_estimates.end(),
            [](const MobileEstimate& lhs, const MobileEstimate& rhs) {
                return lhs.id < rhs.id;
            });

        const Eigen::Index dimension =
            static_cast<Eigen::Index>(4 * mobile_estimates.size());
        mean_ = Eigen::VectorXd::Zero(dimension);
        covariance_ = Eigen::MatrixXd::Zero(dimension, dimension);

        for (std::size_t index = 0; index < mobile_estimates.size(); ++index) {
            const MobileEstimate& estimate = mobile_estimates[index];
            if (index > 0 && mobile_estimates[index - 1].id == estimate.id) {
                throw std::invalid_argument("duplicate mobile node id");
            }
            if (fixed_positions_.count(estimate.id) != 0) {
                throw std::invalid_argument("node cannot be mobile and fixed");
            }
            if (!estimate.mean.allFinite() || !estimate.covariance.allFinite() ||
                !estimate.covariance.isApprox(
                    estimate.covariance.transpose(), 1e-12) ||
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(
                    estimate.covariance)
                        .eigenvalues()
                        .minCoeff() < -1e-12) {
                throw std::invalid_argument(
                    "mobile estimate covariance must be symmetric positive semidefinite");
            }

            mobile_ids_.push_back(estimate.id);
            const Eigen::Index offset = static_cast<Eigen::Index>(4 * index);
            mean_.segment<4>(offset) = estimate.mean;
            covariance_.block<4, 4>(offset, offset) = estimate.covariance;
        }

        for (const auto& [id, position] : fixed_positions_) {
            (void)id;
            if (!position.allFinite()) {
                throw std::invalid_argument("fixed anchor position must be finite");
            }
        }
    }

    void propagate(
        const std::vector<Eigen::Vector2d>& accelerations,
        double dt_s,
        double acceleration_variance) {
        if (accelerations.size() != mobile_ids_.size()) {
            throw std::invalid_argument(
                "one acceleration is required for each mobile node");
        }
        if (!std::isfinite(dt_s) || dt_s <= 0.0) {
            throw std::invalid_argument("dt_s must be positive and finite");
        }
        if (!std::isfinite(acceleration_variance) ||
            acceleration_variance < 0.0) {
            throw std::invalid_argument(
                "acceleration_variance must be non-negative and finite");
        }

        Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
        transition(0, 2) = dt_s;
        transition(1, 3) = dt_s;

        Eigen::Matrix<double, 4, 2> input;
        input << 0.5 * dt_s * dt_s, 0.0,
                 0.0, 0.5 * dt_s * dt_s,
                 dt_s, 0.0,
                 0.0, dt_s;

        Eigen::MatrixXd joint_transition =
            Eigen::MatrixXd::Identity(mean_.size(), mean_.size());
        Eigen::MatrixXd process_covariance =
            Eigen::MatrixXd::Zero(mean_.size(), mean_.size());

        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            const Eigen::Index offset = static_cast<Eigen::Index>(4 * index);
            mean_.segment<4>(offset) =
                transition * mean_.segment<4>(offset) +
                input * accelerations[index];
            joint_transition.block<4, 4>(offset, offset) = transition;
            process_covariance.block<4, 4>(offset, offset) =
                acceleration_variance * input * input.transpose();
        }

        covariance_ = joint_transition * covariance_ *
                          joint_transition.transpose() +
                      process_covariance;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    }

    void update(std::vector<RangeMeasurement> measurements) {
        const std::vector<RangeMeasurement> batch =
            canonicalizeRangeBatch(std::move(measurements));
        for (const RangeMeasurement& measurement : batch) {
            updateOne(measurement);
        }
    }

    JointEstimateSnapshot snapshot() const {
        return JointEstimateSnapshot{
            mobile_ids_, mean_, covariance_, fixed_positions_};
    }

private:
    Eigen::Index mobileOffset(NodeId id) const {
        const auto iterator =
            std::lower_bound(mobile_ids_.begin(), mobile_ids_.end(), id);
        if (iterator == mobile_ids_.end() || *iterator != id) {
            return -1;
        }
        return static_cast<Eigen::Index>(
            4 * std::distance(mobile_ids_.begin(), iterator));
    }

    Eigen::Vector2d position(NodeId id, Eigen::Index mobile_offset) const {
        if (mobile_offset >= 0) {
            return mean_.segment<2>(mobile_offset);
        }
        const auto fixed = fixed_positions_.find(id);
        if (fixed == fixed_positions_.end()) {
            throw std::invalid_argument("range sample contains unknown node");
        }
        return fixed->second;
    }

    void updateOne(const RangeMeasurement& measurement) {
        const Eigen::Index first_offset = mobileOffset(measurement.edge.first);
        const Eigen::Index second_offset = mobileOffset(measurement.edge.second);
        if (first_offset < 0 && second_offset < 0) {
            position(measurement.edge.first, first_offset);
            position(measurement.edge.second, second_offset);
            throw std::invalid_argument(
                "range sample must include a mobile node");
        }

        const Eigen::Vector2d delta =
            position(measurement.edge.first, first_offset) -
            position(measurement.edge.second, second_offset);
        const double predicted_range = delta.norm();
        if (!std::isfinite(predicted_range) || predicted_range <= 1e-12) {
            throw std::invalid_argument(
                "range Jacobian is undefined at coincident positions");
        }
        const Eigen::Vector2d direction = delta / predicted_range;

        Eigen::RowVectorXd jacobian = Eigen::RowVectorXd::Zero(mean_.size());
        if (first_offset >= 0) {
            jacobian.segment<2>(first_offset) = direction.transpose();
        }
        if (second_offset >= 0) {
            jacobian.segment<2>(second_offset) = -direction.transpose();
        }

        const double innovation_variance =
            (jacobian * covariance_ * jacobian.transpose())(0, 0) +
            measurement.variance_m2;
        if (!std::isfinite(innovation_variance) ||
            innovation_variance <= 0.0) {
            throw std::invalid_argument(
                "range innovation variance must be positive and finite");
        }

        const Eigen::VectorXd gain =
            covariance_ * jacobian.transpose() / innovation_variance;
        mean_ += gain * (measurement.range_m - predicted_range);

        const Eigen::MatrixXd identity =
            Eigen::MatrixXd::Identity(mean_.size(), mean_.size());
        const Eigen::MatrixXd correction = identity - gain * jacobian;
        covariance_ = correction * covariance_ * correction.transpose() +
                      gain * measurement.variance_m2 * gain.transpose();
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    }

    std::vector<NodeId> mobile_ids_;
    Eigen::VectorXd mean_;
    Eigen::MatrixXd covariance_;
    std::map<NodeId, Eigen::Vector2d> fixed_positions_;
};

}  // namespace gf
