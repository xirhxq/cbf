#pragma once

#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11sSnapshotCapture.hpp"

#include <cmath>
#include <set>
#include <filesystem>
#include <string>
#include <vector>

namespace gf {

struct Task10p11vRestartAudit {
    bool offline_oracle_complete=false;
    bool capture_fields_complete=false;
    bool deterministic_restart_complete=false;
    std::vector<std::string> missing_fields;
    std::vector<std::string> restart_blockers;
    std::string reason;
};

namespace task10p11v_restart_detail {

inline std::string bitHex(const std::vector<bool>& values) {
    static constexpr char digits[]="0123456789abcdef";
    std::string result((values.size()+3)/4,'0');
    for (std::size_t index=0;index<values.size();++index) {
        if (!values[index]) continue;
        const std::size_t digit=index/4;
        const unsigned shift=static_cast<unsigned>(index%4);
        const unsigned value=static_cast<unsigned>(
            std::isdigit(static_cast<unsigned char>(result[digit]))
                ?result[digit]-'0':result[digit]-'a'+10);
        result[digit]=digits[value|(1U<<shift)];
    }
    return result;
}

inline nlohmann::json gridJson(const GridWorld& grid) {
    return {{"x_limits",{grid.xLim.first,grid.xLim.second}},
        {"y_limits",{grid.yLim.first,grid.yLim.second}},
        {"x_cells",grid.xNum},{"y_cells",grid.yNum},
        {"valid_count",grid.validCount},{"valid_bits_hex",bitHex(grid.valid)},
        {"visited_bits_hex",bitHex(grid.vis)}};
}

inline bool validGridJson(const nlohmann::json& value,
    std::size_t expected_cells) {
    try {
        const auto x=value.at("x_cells").get<std::size_t>();
        const auto y=value.at("y_cells").get<std::size_t>();
        const auto hex_length=(expected_cells+3)/4;
        return x*y==expected_cells &&
            value.at("valid_bits_hex").get<std::string>().size()==hex_length &&
            value.at("visited_bits_hex").get<std::string>().size()==hex_length;
    } catch (...) { return false; }
}

inline nlohmann::json targetsJson(
    const std::map<NodeId,FrontierCell>& targets) {
    nlohmann::json result=nlohmann::json::object();
    for (const auto& [owner,target]:targets)
        result[std::to_string(owner)]={{"x_index",target.x_index},
            {"y_index",target.y_index},
            {"center",{target.center.x(),target.center.y()}}};
    return result;
}

inline nlohmann::json plantJson(const Swarm& swarm) {
    nlohmann::json robots=nlohmann::json::array();
    for (const auto& robot:swarm.robots) {
        const Eigen::VectorXd state=robot->model->getX();
        const Eigen::VectorXd acceleration=robot->model->getAcceleration();
        robots.push_back({{"id",robot->id},{"runtime_s",robot->runtime},
            {"state",task10p11s_capture_detail::matrixJson(state)},
            {"applied_acceleration",{
                acceleration(0),acceleration(1)}}});
    }
    return {{"robots",std::move(robots)}};
}

inline nlohmann::json coverageJson(
    const Task10p11rFixedBaselineFixture& fixture) {
    nlohmann::json local=nlohmann::json::object();
    for (const auto& robot:fixture.swarm.robots)
        local[std::to_string(robot->id)]=gridJson(robot->gridWorld);
    const auto& tracker=fixture.adapter.coverage();
    const auto cell_count=static_cast<std::size_t>(
        fixture.swarm.gridWorldGroundTruth.xNum)*
        static_cast<std::size_t>(fixture.swarm.gridWorldGroundTruth.yNum);
    return {{"cell_count",cell_count},
        {"swarm_ground_truth",gridJson(fixture.swarm.gridWorldGroundTruth)},
        {"robot_local",std::move(local)},
        {"certified_tracker_truth",gridJson(tracker.truthGrid())},
        {"certified_tracker_certified",gridJson(tracker.certifiedGrid())},
        {"truth_fraction",tracker.truthFraction()},
        {"certified_fraction",tracker.certifiedFraction()}};
}

inline nlohmann::json controllerJson(
    const Task10p11hSimpleCoverageController& controller) {
    return {{"target_epoch",controller.targetEpoch()},
        {"targets",targetsJson(controller.committedTargets())},
        {"successful_control_cycles",controller.successfulControlCycles()},
        {"phase",static_cast<int>(controller.phase())},
        {"settling_dwell_cycles",controller.settlingDwellCycles()},
        {"t100_coverage_s",controller.t100CoverageS().has_value()
            ?nlohmann::json(*controller.t100CoverageS())
            :nlohmann::json(nullptr)}};
}

}  // namespace task10p11v_restart_detail

inline nlohmann::json captureTask10p11vRestartFields(
    const Task10p11rFixedBaselineFixture& fixture) {
    return {{"plant",task10p11v_restart_detail::plantJson(fixture.swarm)},
        {"coverage",task10p11v_restart_detail::coverageJson(fixture)}};
}

inline nlohmann::json makeTask10p11vRestartCheckpoint(
    nlohmann::json offline_snapshot,
    nlohmann::json captured_fields,
    const Task10p11hSimpleCoverageController& controller,
    const std::string& capture_event) {
    if (capture_event.empty())
        throw std::invalid_argument("capture event is empty");
    nlohmann::json checkpoint{
        {"protocol","task10p11v-minimal-restart-checkpoint-v1"},
        {"capture_event",capture_event},
        {"runtime_s",offline_snapshot.at("runtime").at("runtime_s")},
        {"plant",std::move(captured_fields.at("plant"))},
        {"coverage",std::move(captured_fields.at("coverage"))},
        {"controller",task10p11v_restart_detail::controllerJson(controller)},
        {"estimator_dekf",{{"estimator_token",
                offline_snapshot.at("runtime").at("estimator_token")},
            {"estimate",offline_snapshot.at("estimator")},
            {"dekf_internal",offline_snapshot.at("dekf_internal")}}},
        {"topology",offline_snapshot.at("runtime").at("fixed_topology")},
        {"exact_zoh",offline_snapshot.at("successor_parameters")},
        {"restart_capability",{{"supported",false},
            {"blockers",nlohmann::json::array({
                "no_import_api_for_estimator_dekf_state",
                "no_import_api_for_adapter_rng_and_range_history",
                "no_import_api_for_supervisor_pending_state",
                "no_import_api_for_controller_and_coverage_state"})}}}};
    offline_snapshot["restart_checkpoint"]=std::move(checkpoint);
    return offline_snapshot;
}

inline nlohmann::json makeTask10p11vRestartCheckpoint(
    nlohmann::json offline_snapshot,
    const Task10p11rFixedBaselineFixture& fixture,
    const std::string& capture_event) {
    return makeTask10p11vRestartCheckpoint(std::move(offline_snapshot),
        captureTask10p11vRestartFields(fixture),fixture.controller,
        capture_event);
}

inline Task10p11vRestartAudit auditTask10p11vRestartCheckpoint(
    const nlohmann::json& snapshot) {
    Task10p11vRestartAudit result;
    try {
        const auto offline=validateTask10p11sSnapshot(snapshot);
        result.offline_oracle_complete=offline.complete;
    } catch (...) {
        result.offline_oracle_complete=false;
    }
    const std::vector<std::string> required={"plant","coverage","controller",
        "estimator_dekf","topology","exact_zoh","restart_capability"};
    if (!snapshot.contains("restart_checkpoint")) {
        for (const auto& field:required)
            result.missing_fields.push_back("restart_checkpoint."+field);
    } else {
        const auto& checkpoint=snapshot.at("restart_checkpoint");
        for (const auto& field:required)
            if (!checkpoint.contains(field))
                result.missing_fields.push_back("restart_checkpoint."+field);
        try {
            const double runtime_s=checkpoint.at("runtime_s").get<double>();
            const auto& robots=checkpoint.at("plant").at("robots");
            std::set<NodeId> plant_ids;
            bool plant_shape=robots.size()==14 && std::isfinite(runtime_s);
            for (const auto& robot:robots) {
                plant_ids.insert(robot.at("id").get<NodeId>());
                const auto& state=robot.at("state");
                const auto& acceleration=robot.at("applied_acceleration");
                plant_shape=plant_shape && state.size()>=6 &&
                    acceleration.size()==2 &&
                    std::abs(robot.at("runtime_s").get<double>()-runtime_s)
                        <=1e-12;
                for (const auto& scalar:state)
                    plant_shape=plant_shape && scalar.is_array() &&
                        scalar.size()==1 &&
                        std::isfinite(scalar.at(0).get<double>());
                for (const auto& scalar:acceleration)
                    plant_shape=plant_shape &&
                        std::isfinite(scalar.get<double>());
            }
            if (!plant_shape || plant_ids.size()!=14)
                result.missing_fields.push_back(
                    "restart_checkpoint.plant.14_owner_states");
            const auto& coverage=checkpoint.at("coverage");
            const auto cells=coverage.at("cell_count").get<std::size_t>();
            bool grids=task10p11v_restart_detail::validGridJson(
                coverage.at("swarm_ground_truth"),cells);
            grids=grids && task10p11v_restart_detail::validGridJson(
                coverage.at("certified_tracker_truth"),cells) &&
                task10p11v_restart_detail::validGridJson(
                    coverage.at("certified_tracker_certified"),cells) &&
                coverage.at("robot_local").size()==14;
            for (const auto& item:coverage.at("robot_local").items())
                grids=grids && task10p11v_restart_detail::validGridJson(
                    item.value(),cells);
            if (!grids)
                result.missing_fields.push_back("restart_checkpoint.coverage.maps");
            const auto& controller=checkpoint.at("controller");
            if (!controller.at("target_epoch").is_number_unsigned() ||
                controller.at("targets").size()!=14)
                result.missing_fields.push_back(
                    "restart_checkpoint.controller.targets_epoch");
            if (checkpoint.at("topology").size()!=28 ||
                checkpoint.at("exact_zoh")!=snapshot.at("successor_parameters"))
                result.missing_fields.push_back(
                    "restart_checkpoint.topology_exact_zoh");
        } catch (...) {
            result.missing_fields.push_back("restart_checkpoint.capture_shape");
        }
        try {
            const auto& capability=checkpoint.at("restart_capability");
            result.deterministic_restart_complete=
                capability.at("supported").get<bool>();
            result.restart_blockers=capability.at("blockers").get<
                std::vector<std::string>>();
        } catch (...) {
            result.deterministic_restart_complete=false;
        }
    }
    result.capture_fields_complete=result.missing_fields.empty();
    if (!result.offline_oracle_complete)
        result.reason="offline_oracle_snapshot_incomplete";
    else if (!result.capture_fields_complete)
        result.reason="runtime_restart_fields_missing";
    else if (!result.deterministic_restart_complete)
        result.reason="runtime_restore_api_unavailable";
    else result.reason="complete";
    return result;
}

inline nlohmann::json task10p11vRestartAuditJson(
    const Task10p11vRestartAudit& audit) {
    return {{"offline_oracle_complete",audit.offline_oracle_complete},
        {"capture_fields_complete",audit.capture_fields_complete},
        {"deterministic_restart_complete",audit.deterministic_restart_complete},
        {"missing_fields",audit.missing_fields},
        {"restart_blockers",audit.restart_blockers},{"reason",audit.reason}};
}

}  // namespace gf
