#pragma once

#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

namespace gf {

inline nlohmann::json task10p11yMetric(
    const std::optional<double>& value,const std::string& reason) {
    if (value.has_value() && !std::isfinite(*value))
        throw std::invalid_argument("Task 10.11y metric is non-finite: "+reason);
    return {{"status",value.has_value()?"valid":"not_applicable"},
        {"value",value.has_value()?nlohmann::json(*value):nlohmann::json(nullptr)},
        {"reason",reason}};
}

struct Task10p11yCycleStatistics {
    std::size_t advanced_cycles=0;
    std::size_t intervention_cycles=0;
    double cumulative_control_deviation=0.0;

    void observe(bool advanced,bool intervention,double control_deviation) {
        if (!advanced) return;
        if (!std::isfinite(control_deviation) || control_deviation<0.0)
            throw std::invalid_argument(
                "Task 10.11y control deviation must be finite and nonnegative");
        ++advanced_cycles;
        if (intervention) {
            ++intervention_cycles;
            cumulative_control_deviation+=control_deviation;
        }
    }
};

inline nlohmann::json makeTask10p11yCurrentPackedCheckpoint(
    const Task10p11rFixedBaselineFixture& fixture,
    const std::string& capture_event) {
    const auto runtime=fixture.adapter.runtimeSnapshot();
    const auto request=fixture.adapter.snapshotHardRowRequest(
        runtime.estimate,runtime.topology);
    auto offline=makeTask10p11sSnapshot(runtime,request,
        fixture.controller.lastNominalControls(),fixture.adapter.config());
    return makeTask10p11vRestartCheckpoint(std::move(offline),fixture,
        capture_event);
}

inline void writeTask10p11yPostAdvanceFailureCheckpoint(
    const std::filesystem::path& path,
    const Task10p11rFixedBaselineFixture& fixture,
    const std::string& hard_gate_reason,nlohmann::json metadata={}) {
    if (hard_gate_reason.empty())
        throw std::invalid_argument(
            "Task 10.11y post-advance hard-gate reason is empty");
    auto checkpoint=makeTask10p11yCurrentPackedCheckpoint(
        fixture,"post_advance_hard_gate_failure");
    checkpoint["task10p11y"]={{"hard_gate_reason",hard_gate_reason},
        {"advanced",true},{"metadata",std::move(metadata)}};
    writeTask10p11vJson(path,checkpoint);
}

}  // namespace gf
