#pragma once

#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/HybridSupervisor.hpp"
#include "grand_finale/ReferenceEligibility.hpp"
#include "grand_finale/SharedCollisionViableFrontierAllocator.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>

namespace gf {

struct CanonicalPredictionBlueprint {
    CanonicalHardRowRequest hard_template;
    double uncertainty_sigma = 0.0;
    double shadow_single_position_support_m = 0.0;
    double shadow_relative_position_support_m = 0.0;
    double shadow_relative_velocity_support_mps = 0.0;

    CanonicalHardRowRequest build(JointEstimateSnapshot snapshot) const {
        if (!std::isfinite(uncertainty_sigma) || uncertainty_sigma < 0.0 ||
            !std::isfinite(shadow_single_position_support_m) ||
            shadow_single_position_support_m < 0.0 ||
            !std::isfinite(shadow_relative_position_support_m) ||
            shadow_relative_position_support_m < 0.0 ||
            !std::isfinite(shadow_relative_velocity_support_mps) ||
            shadow_relative_velocity_support_mps < 0.0) {
            throw std::invalid_argument("invalid canonical prediction blueprint");
        }
        if (snapshot.mobile_ids != hard_template.mobile_ids ||
            snapshot.mean.size()!=static_cast<Eigen::Index>(4*snapshot.mobile_ids.size()) ||
            snapshot.covariance.rows()!=snapshot.mean.size() ||
            snapshot.covariance.cols()!=snapshot.mean.size()) {
            throw std::invalid_argument("prediction snapshot does not match blueprint");
        }
        auto request=hard_template;
        request.reference_snapshot_tubes.clear();
        request.collision_snapshot_tubes.clear();
        request.workspace_snapshot_tubes.clear();
        for (const auto& [id,state] : hard_template.states) {
            if (std::find(snapshot.mobile_ids.begin(),snapshot.mobile_ids.end(),id)==
                snapshot.mobile_ids.end()) {
                snapshot.fixed_positions[id]={state.position.x,state.position.y};
            }
        }
        for (std::size_t index=0;index<snapshot.mobile_ids.size();++index) {
            const NodeId id=snapshot.mobile_ids[index];
            const Eigen::Vector4d state=snapshot.mean.segment<4>(4*index);
            request.states[id]={{state.x(),state.y()},state.tail<2>(),
                                Eigen::Vector2d::Zero()};
            request.workspace_snapshot_tubes[id]={
                uncertainty_sigma*std::sqrt(std::max(
                    0.0,detail::maximumPositionEigenvalue(snapshot,id)))+
                    shadow_single_position_support_m,
                uncertainty_sigma*std::sqrt(std::max(
                    0.0,detail::maximumVelocityEigenvalue(snapshot,id))),
                SnapshotTubeProvenance::CovarianceSigmaDevelopment};
        }
        const auto pair_tube=[&](NodeId first,NodeId second) {
            return PairwiseSnapshotTube{
                uncertainty_sigma*(std::sqrt(std::max(0.0,
                    detail::maximumPositionEigenvalue(snapshot,first)))+
                    std::sqrt(std::max(0.0,
                    detail::maximumPositionEigenvalue(snapshot,second))))+
                    shadow_relative_position_support_m,
                uncertainty_sigma*(std::sqrt(std::max(0.0,
                    detail::maximumVelocityEigenvalue(snapshot,first)))+
                    std::sqrt(std::max(0.0,
                    detail::maximumVelocityEigenvalue(snapshot,second))))+
                    shadow_relative_velocity_support_mps,
                SnapshotTubeProvenance::CovarianceSigmaDevelopment};
        };
        for (const auto& edge : request.reference_edges)
            request.reference_snapshot_tubes[edge.id()]=
                pair_tube(edge.owner,edge.reference);
        for (const auto& edge : request.collision_pairs)
            request.collision_snapshot_tubes[edge.id()]=
                pair_tube(edge.first,edge.second);
        return request;
    }
};

struct AsyncFrontierProposalRequest {
    std::uint64_t request_id = 0;
    std::uint64_t launch_tick = 0;
    std::uint64_t logical_ready_tick = 0;
    std::uint64_t estimator_version = 0;
    double estimator_time_s = 0.0;
    std::uint64_t topology_version = 0;
    std::string topology_digest;
    SupervisorMode mode = SupervisorMode::Search;
    std::uint64_t target_epoch = 0;
    std::uint64_t denominator_version = 0;
    std::uint64_t frontier_mark_version = 0;
    std::uint64_t tube_policy_version = 0;
    std::uint64_t config_version = 0;
    SolverProfile profile = SolverProfile::OpenSource;
    std::uint64_t tie_break_seed = 0;
    JointEstimateSnapshot estimator_snapshot;
    CanonicalPredictionBlueprint canonical_blueprint;
    double planner_period_s = 0.0;

    void validate() const {
        if (request_id==0 || topology_version==0 || denominator_version==0 ||
            tube_policy_version==0 || config_version==0 ||
            topology_digest.empty() || !std::isfinite(estimator_time_s) ||
            !std::isfinite(planner_period_s) || planner_period_s<=0.0)
            throw std::invalid_argument("invalid async proposal provenance");
        if (logical_ready_tick<=launch_tick)
            throw std::invalid_argument("async proposal ready tick must follow launch");
        (void)canonical_blueprint.build(estimator_snapshot);
    }
};

enum class AsyncProposalClassification {
    None,
    NotReady,
    Ready,
    ProposalDeadlineMissed,
    ProposalStale,
    ProposalCancelled,
    FreshCommitRejected,
    AllocatorSearchExhausted,
    OutOfOrder,
    InFlightExists
};

struct AsyncFrontierProposalResult {
    std::uint64_t request_id = 0;
    bool accepted = false;
    std::string reason;
    std::string bundle_id;
    std::map<NodeId,FrontierCell> targets;
    AsyncFrontierProposalRequest provenance;

    static AsyncFrontierProposalResult acceptedFor(
        const AsyncFrontierProposalRequest& request,std::string bundle) {
        AsyncFrontierProposalResult result;
        result.request_id=request.request_id;
        result.accepted=true;
        result.reason="accepted";
        result.bundle_id=std::move(bundle);
        result.provenance=request;
        return result;
    }
};

struct AsyncProposalTransition {
    bool accepted = false;
    AsyncProposalClassification classification =
        AsyncProposalClassification::None;
    std::string reason;
    std::optional<AsyncFrontierProposalResult> result;
};

struct AsyncProposalTimingConfig {
    std::uint64_t planner_period_ticks = 10;
    std::uint64_t maximum_estimator_age_ticks = 10;
    std::size_t maximum_consecutive_failures = 3;
};

class AsyncProposalStateMachine {
public:
    explicit AsyncProposalStateMachine(AsyncProposalTimingConfig config)
        : config_(config) {
        if (config_.planner_period_ticks==0 ||
            config_.maximum_estimator_age_ticks==0 ||
            config_.maximum_consecutive_failures==0)
            throw std::invalid_argument("invalid async proposal timing config");
    }

    AsyncProposalTransition launch(const AsyncFrontierProposalRequest& request) {
        if (boundedFailureReached())
            return {false,last_failure_classification_,
                    "bounded_proposal_failure"};
        request.validate();
        if (in_flight_.has_value() || completed_.has_value())
            return {false,AsyncProposalClassification::InFlightExists,
                    "proposal_in_flight"};
        if (request.request_id<=last_launched_request_id_)
            return {false,AsyncProposalClassification::OutOfOrder,
                    "proposal_request_id_not_monotonic"};
        in_flight_=request;
        last_launched_request_id_=request.request_id;
        return {true,AsyncProposalClassification::None,"launched"};
    }

    AsyncProposalTransition complete(
        AsyncFrontierProposalResult result,double wall_s) {
        if (cancelled_ids_.count(result.request_id)!=0)
            return {false,AsyncProposalClassification::ProposalCancelled,
                    "proposal_cancelled"};
        if (!in_flight_.has_value() ||
            result.request_id!=in_flight_->request_id)
            return {false,AsyncProposalClassification::OutOfOrder,
                    "proposal_out_of_order"};
        if (!std::isfinite(wall_s) || wall_s<0.0)
            throw std::invalid_argument("invalid proposal wall time");
        completed_=std::move(result);
        in_flight_.reset();
        completed_classification_=wall_s>completed_->provenance.planner_period_s
            ? AsyncProposalClassification::ProposalDeadlineMissed
            : AsyncProposalClassification::Ready;
        return {completed_classification_==AsyncProposalClassification::Ready,
                completed_classification_,
                completed_classification_==AsyncProposalClassification::Ready
                    ? "completed":"proposal_deadline_missed"};
    }

    AsyncProposalTransition observe(
        std::uint64_t tick,const AsyncFrontierProposalRequest& current) {
        if (!completed_.has_value())
            return {false,AsyncProposalClassification::NotReady,"not_ready"};
        if (tick<completed_->provenance.logical_ready_tick)
            return {false,AsyncProposalClassification::NotReady,"not_ready"};
        auto result=*completed_;
        completed_.reset();
        if (completed_classification_==
            AsyncProposalClassification::ProposalDeadlineMissed)
            return {false,completed_classification_,"proposal_deadline_missed"};
        if (!structurallyFresh(result.provenance,current) ||
            current.estimator_time_s-result.provenance.estimator_time_s>
                0.1*static_cast<double>(config_.maximum_estimator_age_ticks))
            return {false,AsyncProposalClassification::ProposalStale,
                    "proposal_stale"};
        return {true,AsyncProposalClassification::Ready,"ready",result};
    }

    void cancel(std::string reason) {
        (void)reason;
        if (in_flight_.has_value()) cancelled_ids_.insert(in_flight_->request_id);
        if (completed_.has_value()) cancelled_ids_.insert(completed_->request_id);
        in_flight_.reset();
        completed_.reset();
    }

    void recordFailure(AsyncProposalClassification classification) {
        if (classification==AsyncProposalClassification::Ready ||
            classification==AsyncProposalClassification::None ||
            classification==AsyncProposalClassification::NotReady ||
            classification==AsyncProposalClassification::InFlightExists)
            return;
        ++consecutive_failures_;
        last_failure_classification_=classification;
    }

    void recordCommit() {
        consecutive_failures_=0;
        last_failure_classification_=AsyncProposalClassification::None;
    }
    std::size_t consecutiveFailures() const { return consecutive_failures_; }
    bool boundedFailureReached() const {
        return consecutive_failures_>=config_.maximum_consecutive_failures;
    }

private:
    static bool structurallyFresh(
        const AsyncFrontierProposalRequest& proposal,
        const AsyncFrontierProposalRequest& current) {
        return proposal.topology_version==current.topology_version &&
            proposal.topology_digest==current.topology_digest &&
            proposal.mode==current.mode &&
            proposal.target_epoch==current.target_epoch &&
            proposal.denominator_version==current.denominator_version &&
            proposal.tube_policy_version==current.tube_policy_version &&
            proposal.config_version==current.config_version;
    }

    AsyncProposalTimingConfig config_;
    std::optional<AsyncFrontierProposalRequest> in_flight_;
    std::optional<AsyncFrontierProposalResult> completed_;
    AsyncProposalClassification completed_classification_=
        AsyncProposalClassification::None;
    std::set<std::uint64_t> cancelled_ids_;
    std::size_t consecutive_failures_=0;
    std::uint64_t last_launched_request_id_=0;
    AsyncProposalClassification last_failure_classification_=
        AsyncProposalClassification::None;
};

}  // namespace gf
