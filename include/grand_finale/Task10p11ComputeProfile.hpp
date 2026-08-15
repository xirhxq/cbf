#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

enum class Task10p11ComputePhase {
    CandidateBundleConstruction,
    EstimatorPropagation,
    CanonicalRowRebuild,
    ExactHardProjection,
    CurrentGamma,
    PredictedGamma,
    RobustQpSetup,
    RobustQpSolve,
    ResidualTokenAudit,
    DiagnosticSerialization,
    SolverInitialization,
    SolverModelUpdate
};

inline std::string task10p11ComputePhaseName(Task10p11ComputePhase phase) {
    switch (phase) {
    case Task10p11ComputePhase::CandidateBundleConstruction:
        return "candidate_bundle_construction";
    case Task10p11ComputePhase::EstimatorPropagation:
        return "estimator_propagation";
    case Task10p11ComputePhase::CanonicalRowRebuild:
        return "canonical_row_rebuild";
    case Task10p11ComputePhase::ExactHardProjection:
        return "exact_hard_projection";
    case Task10p11ComputePhase::CurrentGamma:
        return "current_gamma";
    case Task10p11ComputePhase::PredictedGamma:
        return "predicted_gamma";
    case Task10p11ComputePhase::RobustQpSetup:
        return "robust_qp_setup";
    case Task10p11ComputePhase::RobustQpSolve:
        return "robust_qp_solve";
    case Task10p11ComputePhase::ResidualTokenAudit:
        return "residual_token_audit";
    case Task10p11ComputePhase::DiagnosticSerialization:
        return "diagnostic_serialization";
    case Task10p11ComputePhase::SolverInitialization:
        return "solver_initialization";
    case Task10p11ComputePhase::SolverModelUpdate:
        return "solver_model_update";
    }
    throw std::invalid_argument("unknown Task 10.11 compute phase");
}

struct Task10p11ComputeSummary {
    std::size_t calls = 0;
    std::size_t cold_calls = 0;
    std::size_t steady_calls = 0;
    double total_s = 0.0;
    double median_s = 0.0;
    double p95_s = 0.0;
    double maximum_s = 0.0;
};

class Task10p11ComputeProfile {
public:
    void record(Task10p11ComputePhase phase,double seconds,bool steady) {
        if (!std::isfinite(seconds) || seconds < 0.0)
            throw std::invalid_argument("invalid compute-profile sample");
        samples_[phase].push_back({seconds,steady});
    }

    void merge(const Task10p11ComputeProfile& other) {
        for (const auto& [phase,samples] : other.samples_)
            samples_[phase].insert(
                samples_[phase].end(),samples.begin(),samples.end());
    }

    Task10p11ComputeSummary summary(Task10p11ComputePhase phase) const {
        Task10p11ComputeSummary result;
        const auto found=samples_.find(phase);
        if (found==samples_.end()) return result;
        std::vector<double> ordered;
        ordered.reserve(found->second.size());
        for (const auto& sample : found->second) {
            ordered.push_back(sample.seconds);
            result.total_s+=sample.seconds;
            if (sample.steady) ++result.steady_calls;
            else ++result.cold_calls;
        }
        result.calls=ordered.size();
        std::sort(ordered.begin(),ordered.end());
        const std::size_t middle=ordered.size()/2;
        result.median_s=ordered.size()%2==0
            ? 0.5*(ordered[middle-1]+ordered[middle]) : ordered[middle];
        const std::size_t p95=std::min(
            ordered.size()-1,static_cast<std::size_t>(
                std::ceil(0.95*static_cast<double>(ordered.size())))-1);
        result.p95_s=ordered[p95];
        result.maximum_s=ordered.back();
        return result;
    }

private:
    struct Sample {
        double seconds = 0.0;
        bool steady = false;
    };
    std::map<Task10p11ComputePhase,std::vector<Sample>> samples_;
};

}  // namespace gf
