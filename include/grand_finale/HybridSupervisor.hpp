#pragma once

#include "grand_finale/TransitionCertifier.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace gf {

enum class SupervisorMode { Search, Reform, Retreat, Hold };

struct SupervisorThresholds {
    double minimum_dwell_s;
    double gamma_trigger;
    double gamma_accept;
};

class HybridSupervisor {
public:
    explicit HybridSupervisor(SupervisorThresholds thresholds)
        : thresholds_(thresholds) {
        if (!std::isfinite(thresholds_.minimum_dwell_s) ||
            thresholds_.minimum_dwell_s < 0.0 ||
            !std::isfinite(thresholds_.gamma_trigger) ||
            !std::isfinite(thresholds_.gamma_accept) ||
            thresholds_.gamma_trigger <= 0.0 ||
            thresholds_.gamma_accept <= thresholds_.gamma_trigger) {
            throw std::invalid_argument("invalid supervisor thresholds");
        }
    }

    SupervisorMode observeGamma(
        double now_s,
        double minimum_gamma,
        bool candidate_available,
        bool reverse_available) {
        if (!std::isfinite(now_s) || !std::isfinite(minimum_gamma) ||
            now_s < last_transition_s_) {
            mode_ = SupervisorMode::Hold;
            return mode_;
        }
        if (now_s - last_transition_s_ < thresholds_.minimum_dwell_s)
            return mode_;
        if (mode_ == SupervisorMode::Search &&
            minimum_gamma <= thresholds_.gamma_trigger) {
            mode_ = candidate_available
                ? SupervisorMode::Reform
                : (reverse_available
                    ? SupervisorMode::Retreat
                    : SupervisorMode::Hold);
            last_transition_s_ = now_s;
        } else if (mode_ == SupervisorMode::Reform &&
                   minimum_gamma >= thresholds_.gamma_accept) {
            mode_ = SupervisorMode::Search;
            last_transition_s_ = now_s;
        }
        return mode_;
    }

    SupervisorMode requestReformation(
        double now_s,
        bool candidate_available,
        bool reverse_available) {
        if (!std::isfinite(now_s) || now_s < last_transition_s_) {
            mode_ = SupervisorMode::Hold;
            return mode_;
        }
        if (now_s - last_transition_s_ < thresholds_.minimum_dwell_s)
            return mode_;
        mode_ = candidate_available
            ? SupervisorMode::Reform
            : (reverse_available
                ? SupervisorMode::Retreat
                : SupervisorMode::Hold);
        last_transition_s_ = now_s;
        return mode_;
    }

    void initializeTopology(
        std::vector<DirectedEdge> topology,
        std::uint64_t topology_version) {
        if (!topology_.empty() || topology_version_ != 0 || pending_.has_value())
            throw std::logic_error("supervisor topology is already initialized");
        topology_ = std::move(topology);
        topology_version_ = topology_version;
    }

    bool beginMakeBeforeBreak(
        const TransitionCertificate& certificate,
        std::uint64_t current_topology_version,
        std::uint64_t current_estimator_version,
        double now_s) {
        if (mode_ != SupervisorMode::Reform || !certificate.valid ||
            !certificate.forward_valid || !certificate.reverse_valid ||
            certificate.topology_version != current_topology_version ||
            certificate.estimator_version != current_estimator_version) {
            mode_ = SupervisorMode::Hold;
            last_transition_s_ = now_s;
            return false;
        }
        topology_ = certificate.union_edges;
        topology_version_ = current_topology_version + 1;
        pending_ = certificate;
        mode_ = SupervisorMode::Reform;
        last_transition_s_ = now_s;
        return true;
    }

    bool finishMakeBeforeBreak(
        std::uint64_t current_topology_version,
        std::uint64_t current_estimator_version,
        double now_s) {
        if (!pending_.has_value() || mode_ != SupervisorMode::Reform ||
            current_topology_version != topology_version_ ||
            current_estimator_version != pending_->estimator_version) {
            mode_ = SupervisorMode::Hold;
            last_transition_s_ = now_s;
            return false;
        }
        topology_ = pending_->successor_edges;
        topology_version_ = current_topology_version + 1;
        pending_.reset();
        mode_ = SupervisorMode::Search;
        last_transition_s_ = now_s;
        return true;
    }

    SupervisorMode mode() const { return mode_; }
    const std::vector<DirectedEdge>& topology() const { return topology_; }
    std::uint64_t topologyVersion() const { return topology_version_; }

private:
    SupervisorThresholds thresholds_;
    SupervisorMode mode_ = SupervisorMode::Search;
    double last_transition_s_ = 0.0;
    std::vector<DirectedEdge> topology_;
    std::uint64_t topology_version_ = 0;
    std::optional<TransitionCertificate> pending_;
};

}  // namespace gf
