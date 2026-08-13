#pragma once

#include "grand_finale/CentralizedEkfOracle.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace gf {

struct MarginalEstimate {
    NodeId id;
    Eigen::Vector4d mean;
    Eigen::Matrix4d covariance;
};

struct InterimMasterMessage {
    std::uint64_t source_version;
    std::uint64_t message_id;
    std::int64_t timestamp_ns;
    UndirectedEdge measured_edge;
    NodeId master;
    double innovation;
    double innovation_variance;
    std::vector<Eigen::Vector4d> gamma_by_mobile;
};

class InterimMasterDekf {
public:
    InterimMasterDekf(
        std::vector<MobileEstimate> mobile_estimates,
        std::map<NodeId, Eigen::Vector2d> fixed_positions)
        : fixed_positions_(std::move(fixed_positions)) {
        std::sort(
            mobile_estimates.begin(), mobile_estimates.end(),
            [](const MobileEstimate& lhs, const MobileEstimate& rhs) {
                return lhs.id < rhs.id;
            });

        correlation_rows_.resize(
            mobile_estimates.size(),
            std::vector<Eigen::Matrix4d>(
                mobile_estimates.size(), Eigen::Matrix4d::Zero()));

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
            means_.push_back(estimate.mean);
            propagation_factors_.push_back(Eigen::Matrix4d::Identity());
            correlation_rows_[index][index] = estimate.covariance;
        }

        for (const auto& [id, position] : fixed_positions_) {
            (void)id;
            if (!position.allFinite()) {
                throw std::invalid_argument("fixed anchor position must be finite");
            }
        }
    }

    void propagateLocal(
        NodeId agent,
        const Eigen::Vector2d& acceleration,
        double dt_s,
        double acceleration_variance) {
        const std::size_t index = mobileIndex(agent);
        if (!acceleration.allFinite()) {
            throw std::invalid_argument("acceleration must be finite");
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

        const Eigen::Matrix4d old_phi = propagation_factors_[index];
        const Eigen::Matrix4d old_covariance =
            old_phi * correlationBlock(index, index) * old_phi.transpose();
        means_[index] = transition * means_[index] + input * acceleration;
        propagation_factors_[index] = transition * old_phi;

        const Eigen::Matrix4d propagated_covariance =
            transition * old_covariance * transition.transpose() +
            acceleration_variance * input * input.transpose();
        const Eigen::Matrix4d inverse_phi =
            propagation_factors_[index].inverse();
        correlationBlock(index, index) =
            inverse_phi * propagated_covariance * inverse_phi.transpose();
        correlationBlock(index, index) =
            0.5 * (correlationBlock(index, index) +
                   correlationBlock(index, index).transpose());
        ++version_;
    }

    InterimMasterMessage makeUpdate(
        NodeId master,
        const RangeMeasurement& raw_measurement) const {
        const RangeMeasurement measurement =
            canonicalizeRangeBatch({raw_measurement}).front();
        const MeasurementKey key{
            measurement.timestamp_ns,
            measurement.edge.first,
            measurement.edge.second};
        if (last_measurement_.has_value() && !(last_measurement_.value() < key)) {
            throw std::invalid_argument(
                "range measurements must be applied in canonical order");
        }

        const std::optional<std::size_t> first_index =
            findMobileIndex(measurement.edge.first);
        const std::optional<std::size_t> second_index =
            findMobileIndex(measurement.edge.second);
        const bool master_is_first =
            first_index.has_value() && measurement.edge.first == master;
        const bool master_is_second =
            second_index.has_value() && measurement.edge.second == master;
        if (!master_is_first && !master_is_second) {
            throw std::invalid_argument(
                "interim master must be a mobile endpoint of the measured edge");
        }

        const Eigen::Vector2d delta =
            position(measurement.edge.first, first_index) -
            position(measurement.edge.second, second_index);
        const double predicted_range = delta.norm();
        if (!std::isfinite(predicted_range) || predicted_range <= 1e-12) {
            throw std::invalid_argument(
                "range Jacobian is undefined at coincident positions");
        }
        const Eigen::Vector2d direction = delta / predicted_range;

        Eigen::RowVector4d first_hbar = Eigen::RowVector4d::Zero();
        Eigen::RowVector4d second_hbar = Eigen::RowVector4d::Zero();
        if (first_index.has_value()) {
            Eigen::RowVector4d jacobian = Eigen::RowVector4d::Zero();
            jacobian.head<2>() = direction.transpose();
            first_hbar = jacobian * propagation_factors_[*first_index];
        }
        if (second_index.has_value()) {
            Eigen::RowVector4d jacobian = Eigen::RowVector4d::Zero();
            jacobian.head<2>() = -direction.transpose();
            second_hbar = jacobian * propagation_factors_[*second_index];
        }

        std::vector<Eigen::Vector4d> gamma_by_mobile(
            mobile_ids_.size(), Eigen::Vector4d::Zero());
        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            if (first_index.has_value()) {
                gamma_by_mobile[index] +=
                    correlationBlock(index, *first_index) *
                    first_hbar.transpose();
            } else {
                requireFixed(measurement.edge.first);
            }
            if (second_index.has_value()) {
                gamma_by_mobile[index] +=
                    correlationBlock(index, *second_index) *
                    second_hbar.transpose();
            } else {
                requireFixed(measurement.edge.second);
            }
        }

        double innovation_variance = measurement.variance_m2;
        if (first_index.has_value()) {
            innovation_variance +=
                (first_hbar * gamma_by_mobile[*first_index])(0, 0);
        }
        if (second_index.has_value()) {
            innovation_variance +=
                (second_hbar * gamma_by_mobile[*second_index])(0, 0);
        }
        if (!std::isfinite(innovation_variance) ||
            innovation_variance <= 0.0) {
            throw std::invalid_argument(
                "range innovation variance must be positive and finite");
        }

        return InterimMasterMessage{
            version_,
            version_ + 1,
            measurement.timestamp_ns,
            measurement.edge,
            master,
            measurement.range_m - predicted_range,
            innovation_variance,
            gamma_by_mobile};
    }

    void applyUpdate(const InterimMasterMessage& message) {
        if (message.source_version != version_ ||
            message.message_id != version_ + 1) {
            throw std::invalid_argument(
                "stale or repeated interim-master message");
        }
        if (message.gamma_by_mobile.size() != mobile_ids_.size()) {
            throw std::invalid_argument(
                "interim-master message dimension mismatch");
        }
        if (!std::isfinite(message.innovation) ||
            !std::isfinite(message.innovation_variance) ||
            message.innovation_variance <= 0.0) {
            throw std::invalid_argument("invalid interim-master innovation");
        }

        for (const Eigen::Vector4d& gamma : message.gamma_by_mobile) {
            if (!gamma.allFinite()) {
                throw std::invalid_argument(
                    "invalid interim-master correlation factor");
            }
        }

        for (std::size_t index = 0; index < mobile_ids_.size(); ++index) {
            means_[index] +=
                propagation_factors_[index] *
                message.gamma_by_mobile[index] *
                (message.innovation / message.innovation_variance);
        }
        for (std::size_t row = 0; row < mobile_ids_.size(); ++row) {
            for (std::size_t column = 0; column < mobile_ids_.size(); ++column) {
                correlation_rows_[row][column] -=
                    message.gamma_by_mobile[row] *
                    message.gamma_by_mobile[column].transpose() /
                    message.innovation_variance;
            }
        }
        for (std::size_t row = 0; row < mobile_ids_.size(); ++row) {
            correlation_rows_[row][row] =
                0.5 * (correlation_rows_[row][row] +
                       correlation_rows_[row][row].transpose());
            for (std::size_t column = row + 1;
                 column < mobile_ids_.size(); ++column) {
                const Eigen::Matrix4d symmetric_pair =
                    0.5 * (correlation_rows_[row][column] +
                           correlation_rows_[column][row].transpose());
                correlation_rows_[row][column] = symmetric_pair;
                correlation_rows_[column][row] = symmetric_pair.transpose();
            }
        }

        last_measurement_ = MeasurementKey{
            message.timestamp_ns,
            message.measured_edge.first,
            message.measured_edge.second};
        ++version_;
    }

    MarginalEstimate marginal(NodeId agent) const {
        const std::size_t index = mobileIndex(agent);
        const Eigen::Matrix4d covariance =
            propagation_factors_[index] * correlationBlock(index, index) *
            propagation_factors_[index].transpose();
        return MarginalEstimate{agent, means_[index], covariance};
    }

    Eigen::Matrix4d crossCovariance(NodeId first, NodeId second) const {
        const std::size_t first_index = mobileIndex(first);
        const std::size_t second_index = mobileIndex(second);
        return propagation_factors_[first_index] *
               correlationBlock(first_index, second_index) *
               propagation_factors_[second_index].transpose();
    }

    JointEstimateSnapshot reconstructForAudit() const {
        const Eigen::Index dimension =
            static_cast<Eigen::Index>(4 * mobile_ids_.size());
        Eigen::VectorXd joint_mean = Eigen::VectorXd::Zero(dimension);
        Eigen::MatrixXd joint_covariance =
            Eigen::MatrixXd::Zero(dimension, dimension);
        for (std::size_t row = 0; row < mobile_ids_.size(); ++row) {
            joint_mean.segment<4>(static_cast<Eigen::Index>(4 * row)) =
                means_[row];
            for (std::size_t column = 0; column < mobile_ids_.size(); ++column) {
                joint_covariance.block<4, 4>(
                    static_cast<Eigen::Index>(4 * row),
                    static_cast<Eigen::Index>(4 * column)) =
                    propagation_factors_[row] *
                    correlationBlock(row, column) *
                    propagation_factors_[column].transpose();
            }
        }
        joint_covariance =
            0.5 * (joint_covariance + joint_covariance.transpose());
        return JointEstimateSnapshot{
            mobile_ids_, joint_mean, joint_covariance, fixed_positions_};
    }

private:
    using MeasurementKey = std::tuple<std::int64_t, NodeId, NodeId>;

    std::optional<std::size_t> findMobileIndex(NodeId id) const {
        const auto iterator =
            std::lower_bound(mobile_ids_.begin(), mobile_ids_.end(), id);
        if (iterator == mobile_ids_.end() || *iterator != id) {
            return std::nullopt;
        }
        return static_cast<std::size_t>(
            std::distance(mobile_ids_.begin(), iterator));
    }

    std::size_t mobileIndex(NodeId id) const {
        const std::optional<std::size_t> index = findMobileIndex(id);
        if (!index.has_value()) {
            throw std::invalid_argument("unknown mobile node");
        }
        return *index;
    }

    void requireFixed(NodeId id) const {
        if (fixed_positions_.count(id) == 0) {
            throw std::invalid_argument("range sample contains unknown node");
        }
    }

    Eigen::Vector2d position(
        NodeId id,
        const std::optional<std::size_t>& mobile_index) const {
        if (mobile_index.has_value()) {
            return means_[*mobile_index].head<2>();
        }
        requireFixed(id);
        return fixed_positions_.at(id);
    }

    Eigen::Matrix4d& correlationBlock(
        std::size_t row,
        std::size_t column) {
        return correlation_rows_[row][column];
    }

    const Eigen::Matrix4d& correlationBlock(
        std::size_t row,
        std::size_t column) const {
        return correlation_rows_[row][column];
    }

    std::vector<NodeId> mobile_ids_;
    std::vector<Eigen::Vector4d> means_;
    std::vector<Eigen::Matrix4d> propagation_factors_;
    std::vector<std::vector<Eigen::Matrix4d>> correlation_rows_;
    std::map<NodeId, Eigen::Vector2d> fixed_positions_;
    std::uint64_t version_ = 0;
    std::optional<MeasurementKey> last_measurement_;
};

}  // namespace gf
