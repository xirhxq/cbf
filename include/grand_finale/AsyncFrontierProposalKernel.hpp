#pragma once

#include "grand_finale/AsyncFrontierProposal.hpp"
#include "grand_finale/SharedFrontierAllocationPipeline.hpp"

#include <chrono>
#include <future>

namespace gf {

struct AsyncFrontierWorkRequest {
    AsyncFrontierProposalRequest provenance;
    SharedFrontierAllocatorConfig allocator_config;
    FrontierAllocationRequest allocation;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    double dt_s = 0.0;
    double estimator_acceleration_variance = 0.0;
    CanonicalGammaFeedbackConfig gamma_feedback_config;
};

struct AsyncFrontierWorkResult {
    AsyncFrontierProposalResult proposal;
    std::size_t candidate_bundles = 0;
    std::size_t recursive_allocator_calls = 0;
    SharedFrontierPipelineResult diagnostic;
};

inline AsyncFrontierWorkResult runAsyncFrontierProposal(
    const AsyncFrontierWorkRequest& work) {
    work.provenance.validate();
    SharedFrontierPipelineRequest request;
    request.allocation=work.allocation;
    request.profile=work.provenance.profile;
    request.dt_s=work.dt_s;
    request.fixed_positions=work.fixed_positions;
    request.estimator_snapshot=work.provenance.estimator_snapshot;
    request.estimator_acceleration_variance=
        work.estimator_acceleration_variance;
    request.gamma_feedback_config=work.gamma_feedback_config;
    request.hard_row_request=work.provenance.canonical_blueprint.build(
        work.provenance.estimator_snapshot);
    const auto blueprint=work.provenance.canonical_blueprint;
    request.canonical_request_builder=[blueprint](
        const JointEstimateSnapshot& snapshot) {
        return blueprint.build(snapshot);
    };
    SharedFrontierAllocationPipeline pipeline(work.allocator_config);
    AsyncFrontierWorkResult result;
    result.diagnostic=pipeline.choose(std::move(request));
    result.candidate_bundles=result.diagnostic.candidate_bundles;
    result.proposal.request_id=work.provenance.request_id;
    result.proposal.provenance=work.provenance;
    result.proposal.accepted=result.diagnostic.accepted;
    result.proposal.reason=result.diagnostic.reason;
    result.proposal.bundle_id=result.diagnostic.allocation.bundle_id;
    result.proposal.targets=result.diagnostic.allocation.targets;
    return result;
}

struct AsyncWorkerCompletion {
    AsyncFrontierWorkResult work;
    double wall_s = 0.0;
};

class AsyncFrontierProposalWorker {
public:
    bool launch(AsyncFrontierWorkRequest request) {
        if (future_.valid()) return false;
        launched_=std::chrono::steady_clock::now();
        future_=std::async(std::launch::async,
            [request=std::move(request)]() {
                return runAsyncFrontierProposal(request);
            });
        return true;
    }

    std::optional<AsyncWorkerCompletion> tryCollect() {
        if (!future_.valid() ||
            future_.wait_for(std::chrono::seconds(0))!=
                std::future_status::ready)
            return std::nullopt;
        AsyncWorkerCompletion result;
        result.work=future_.get();
        result.wall_s=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-launched_).count();
        return result;
    }

private:
    std::future<AsyncFrontierWorkResult> future_;
    std::chrono::steady_clock::time_point launched_;
};

}  // namespace gf
