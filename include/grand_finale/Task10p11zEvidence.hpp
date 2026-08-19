#pragma once

#include "grand_finale/Task10p11yEvidence.hpp"
#include "grand_finale/Task10p11tOnlinePairResponsibility.hpp"

#include <filesystem>
#include <map>
#include <string>

namespace gf {

enum class Task10p11zTerminationBoundary {
    PreAdvance,
    GuardRejection,
    PostAdvance,
    Exception,
    NormalEnd
};

inline std::string task10p11zTerminationBoundaryName(
    Task10p11zTerminationBoundary boundary) {
    switch (boundary) {
        case Task10p11zTerminationBoundary::PreAdvance:
            return "pre_advance";
        case Task10p11zTerminationBoundary::GuardRejection:
            return "guard_rejection";
        case Task10p11zTerminationBoundary::PostAdvance:
            return "post_advance";
        case Task10p11zTerminationBoundary::Exception:
            return "exception";
        case Task10p11zTerminationBoundary::NormalEnd:
            return "normal_end";
    }
    throw std::logic_error("unknown Task 10.11z termination boundary");
}

inline bool task10p11zAppliedControlsMatch(
    const std::map<NodeId,Eigen::Vector2d>& first,
    const std::map<NodeId,Eigen::Vector2d>& second,double tolerance) {
    if (!std::isfinite(tolerance) || tolerance<0.0 ||
        first.size()!=second.size()) return false;
    for (const auto& [owner,control]:first) {
        const auto found=second.find(owner);
        if (found==second.end() || !control.allFinite() ||
            !found->second.allFinite() ||
            (control-found->second).norm()>tolerance) return false;
    }
    return true;
}

struct Task10p11zPreparedBaseline {
    bool valid=false;
    std::string reason;
    std::optional<std::string> active_pair;
    SimpleCoverageControlStep control;
    std::map<NodeId,Eigen::Vector2d> nominal_controls;
};

inline Task10p11zPreparedBaseline task10p11zPrepareNativeBaseline(
    const Task10p11rFixedBaselineFixture& source,
    GammaFeedbackSelectionMode selection,double tau,
    std::optional<std::string> active_pair) {
    Task10p11zPreparedBaseline result;
    try {
        auto shadow=makeTask10p11rFixedBaselineFixture(selection,tau);
        const auto initialized=shadow->adapter.initializeStageZero();
        if (!initialized.initialized) {
            result.reason="shadow_stage_zero_initialization_failed:"+
                initialized.reason;
            return result;
        }
        const auto restart=makeTask10p11vSparseRestartCheckpoint(
            source,active_pair,source.controller.successfulControlCycles());
        const auto restored=restoreTask10p11vSparseRestartCheckpoint(
            *shadow,restart);
        if (restored.active_pair!=active_pair) {
            result.reason="shadow_active_pair_restore_mismatch";
            return result;
        }
        const auto runtime=shadow->adapter.runtimeSnapshot();
        if (!active_pair.has_value()) {
            const auto rows=buildCanonicalHardRows(
                shadow->adapter.snapshotHardRowRequest(
                    runtime.estimate,runtime.topology));
            const auto diagnostic=diagnoseTask10p11tOnlineConflicts(
                rows,runtime.estimate.mobile_ids,runtime.runtime_s,
                runtime.mode,runtime.topology,
                shadow->adapter.config().acceleration_half_box);
            if (!diagnostic.valid) {
                result.reason=diagnostic.reason;
                return result;
            }
            if (diagnostic.infeasible) {
                active_pair=task10p11tUniqueOnlinePair(diagnostic);
                if (!active_pair.has_value()) {
                    result.reason=diagnostic.mobile_pair_base_ids.size()>1
                        ?"multiple_mobile_pair_conflict":
                        "dynamic_pair_conflict_not_unique";
                    return result;
                }
            }
        }
        result.active_pair=active_pair;
        result.control=active_pair.has_value()
            ?shadow->controller.advanceWithDynamicPairResponsibility(
                *active_pair)
            :shadow->controller.advance();
        result.nominal_controls=shadow->controller.lastNominalControls();
        result.valid=true;
        result.reason=result.control.reason;
    } catch (const std::exception& error) {
        result.reason=error.what();
    }
    return result;
}

struct Task10p11zTerminationEvidence {
    Task10p11zTerminationBoundary boundary=
        Task10p11zTerminationBoundary::PreAdvance;
    bool valid=false;
    bool advanced=false;
    std::string reason;
    nlohmann::json metadata=nlohmann::json::object();
};

inline void writeTask10p11zTerminationEvidence(
    const std::filesystem::path& result_path,
    const std::filesystem::path& checkpoint_path,
    const Task10p11rFixedBaselineFixture& fixture,
    const Task10p11zTerminationEvidence& evidence) {
    if (evidence.reason.empty())
        throw std::invalid_argument("Task 10.11z termination reason is empty");
    const std::string boundary=task10p11zTerminationBoundaryName(
        evidence.boundary);
    const bool packed=fixture.controller.lastNominalControls().size()==14;
    nlohmann::json checkpoint=packed
        ?makeTask10p11yCurrentPackedCheckpoint(
            fixture,"task10p11z_"+boundary)
        :makeTask10p11vSparseRestartCheckpoint(
            fixture,std::nullopt,
            fixture.controller.successfulControlCycles());
    checkpoint["task10p11z"]={{"protocol",
        "task10p11z-termination-evidence-v1"},
        {"termination_boundary",boundary},{"valid",evidence.valid},
        {"advanced",evidence.advanced},{"reason",evidence.reason},
        {"metadata",evidence.metadata},{"checkpoint_kind",
            packed?"packed":"sparse_recoverable"}};
    writeTask10p11vJson(checkpoint_path,checkpoint);

    const double runtime=fixture.swarm.robots.empty()?0.0:
        fixture.swarm.robots.front()->runtime;
    const nlohmann::json result={{"protocol",
        "task10p11z-termination-result-v1"},
        {"valid",evidence.valid},{"advanced",evidence.advanced},
        {"termination_boundary",boundary},{"reason",evidence.reason},
        {"runtime_s",runtime},{"checkpoint_published",true},
        {"checkpoint_kind",packed?"packed":"sparse_recoverable"},
        {"checkpoint_path",checkpoint_path.filename().string()},
        {"metadata",evidence.metadata},
        {"claim_boundary",{{"finite_trajectory_only",true},
            {"recursive_feasibility_claimed",false},
            {"task_11_entered",false}}}};
    writeTask10p11vJson(result_path,result);
}

}  // namespace gf
