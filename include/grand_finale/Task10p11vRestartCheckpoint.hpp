#pragma once

#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11sSnapshotCapture.hpp"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
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

inline std::vector<bool> bitsFromHex(
    const std::string& value,std::size_t count) {
    if (value.size()!=(count+3)/4)
        throw std::invalid_argument("packed bitset length mismatch");
    std::vector<bool> result(count,false);
    for (std::size_t digit=0;digit<value.size();++digit) {
        const char character=value[digit];
        const unsigned nibble=character>='0' && character<='9'
            ?static_cast<unsigned>(character-'0'):
            character>='a' && character<='f'
                ?static_cast<unsigned>(character-'a'+10):16U;
        if (nibble>15U) throw std::invalid_argument("invalid packed bitset");
        for (unsigned shift=0;shift<4;++shift) {
            const std::size_t index=4*digit+shift;
            if (index<count) result[index]=(nibble&(1U<<shift))!=0;
            else if ((nibble&(1U<<shift))!=0)
                throw std::invalid_argument("nonzero packed bitset padding");
        }
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
        const Eigen::VectorXd control=robot->model->getControlInput();
        robots.push_back({{"id",robot->id},{"runtime_s",robot->runtime},
            {"state",task10p11s_capture_detail::matrixJson(state)},
            {"control",task10p11s_capture_detail::matrixJson(control)}});
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
    const auto state=controller.restartState();
    nlohmann::json outside=nlohmann::json::object();
    for (const auto& [owner,duration]:
         state.boundary_excursion.owner_outside_duration_s)
        outside[std::to_string(owner)]=duration;
    return {{"target_epoch",state.target_epoch},
        {"targets",targetsJson(state.targets)},
        {"consecutive_failures",state.consecutive_failures},
        {"successful_control_cycles",state.successful_control_cycles},
        {"control_boundaries",state.control_boundaries},
        {"phase",static_cast<int>(state.phase)},
        {"settling_dwell_cycles",state.settling_dwell_cycles},
        {"t100_coverage_s",state.t100_coverage_s.has_value()
            ?nlohmann::json(*state.t100_coverage_s):nlohmann::json(nullptr)},
        {"last_nominal_controls",
            task10p11s_capture_detail::nominalJson(
                state.last_nominal_controls)},
        {"boundary_excursion",{
            {"maximum_outside_distance_m",
                state.boundary_excursion.maximum_outside_distance_m},
            {"any_outside_duration_s",
                state.boundary_excursion.any_outside_duration_s},
            {"owner_outside_duration_s",outside},
            {"maximum_simultaneous_outside",
                state.boundary_excursion.maximum_simultaneous_outside},
            {"maximum_position_norm_m",
                state.boundary_excursion.maximum_position_norm_m},
            {"maximum_position",task10p11s_capture_detail::vector2Json(
                state.boundary_excursion.maximum_position)},
            {"outside_observer_new_truth_cells",
                state.boundary_excursion.outside_observer_new_truth_cells}}}};
}

inline SimpleCoverageControllerRestartState controllerFromJson(
    const nlohmann::json& value) {
    SimpleCoverageControllerRestartState state;
    for (const auto& item:value.at("targets").items()) {
        const auto center=task10p11s_capture_detail::vector2FromJson(
            item.value().at("center"));
        state.targets.emplace(static_cast<NodeId>(std::stoul(item.key())),
            FrontierCell{item.value().at("x_index").get<int>(),
                item.value().at("y_index").get<int>(),center});
    }
    state.target_epoch=value.at("target_epoch").get<std::size_t>();
    state.consecutive_failures=
        value.at("consecutive_failures").get<std::size_t>();
    state.successful_control_cycles=
        value.at("successful_control_cycles").get<std::size_t>();
    state.control_boundaries=value.at("control_boundaries").get<std::size_t>();
    state.phase=static_cast<SimpleCoveragePhase>(value.at("phase").get<int>());
    state.settling_dwell_cycles=
        value.at("settling_dwell_cycles").get<std::size_t>();
    if (!value.at("t100_coverage_s").is_null())
        state.t100_coverage_s=value.at("t100_coverage_s").get<double>();
    state.last_nominal_controls=
        task10p11s_capture_detail::nominalFromJson(
            value.at("last_nominal_controls"));
    const auto& boundary=value.at("boundary_excursion");
    state.boundary_excursion.maximum_outside_distance_m=
        boundary.at("maximum_outside_distance_m").get<double>();
    state.boundary_excursion.any_outside_duration_s=
        boundary.at("any_outside_duration_s").get<double>();
    for (const auto& item:boundary.at("owner_outside_duration_s").items())
        state.boundary_excursion.owner_outside_duration_s.emplace(
            static_cast<NodeId>(std::stoul(item.key())),
            item.value().get<double>());
    state.boundary_excursion.maximum_simultaneous_outside=
        boundary.at("maximum_simultaneous_outside").get<std::size_t>();
    state.boundary_excursion.maximum_position_norm_m=
        boundary.at("maximum_position_norm_m").get<double>();
    state.boundary_excursion.maximum_position=
        task10p11s_capture_detail::vector2FromJson(
            boundary.at("maximum_position"));
    state.boundary_excursion.outside_observer_new_truth_cells=
        boundary.at("outside_observer_new_truth_cells").get<std::size_t>();
    return state;
}

inline void restoreGridJson(const nlohmann::json& value,GridWorld& grid) {
    const auto x_limits=value.at("x_limits");
    const auto y_limits=value.at("y_limits");
    if (value.at("x_cells").get<int>()!=grid.xNum ||
        value.at("y_cells").get<int>()!=grid.yNum ||
        x_limits.size()!=2 || y_limits.size()!=2 ||
        std::abs(x_limits.at(0).get<double>()-grid.xLim.first)>1e-12 ||
        std::abs(x_limits.at(1).get<double>()-grid.xLim.second)>1e-12 ||
        std::abs(y_limits.at(0).get<double>()-grid.yLim.first)>1e-12 ||
        std::abs(y_limits.at(1).get<double>()-grid.yLim.second)>1e-12)
        throw std::invalid_argument("grid restart geometry mismatch");
    const std::size_t count=static_cast<std::size_t>(grid.xNum)*
        static_cast<std::size_t>(grid.yNum);
    const auto valid=bitsFromHex(
        value.at("valid_bits_hex").get<std::string>(),count);
    if (valid!=grid.valid ||
        value.at("valid_count").get<std::size_t>()!=grid.validCount)
        throw std::invalid_argument("grid restart valid mask mismatch");
    grid.vis=bitsFromHex(
        value.at("visited_bits_hex").get<std::string>(),count);
}

inline nlohmann::json doubleMapJson(
    const std::map<std::string,double>& values) {
    nlohmann::json result=nlohmann::json::object();
    for (const auto& [key,value]:values) result[key]=value;
    return result;
}

inline std::map<std::string,double> doubleMapFromJson(
    const nlohmann::json& value) {
    std::map<std::string,double> result;
    for (const auto& item:value.items()) {
        const double scalar=item.value().get<double>();
        if (!std::isfinite(scalar))
            throw std::invalid_argument("nonfinite restart map value");
        result.emplace(item.key(),scalar);
    }
    return result;
}

inline nlohmann::json dekfJson(const DekfRestartState& state) {
    nlohmann::json means=nlohmann::json::array();
    nlohmann::json factors=nlohmann::json::array();
    nlohmann::json correlations=nlohmann::json::array();
    for (const auto& mean:state.means)
        means.push_back(task10p11s_capture_detail::matrixJson(mean));
    for (const auto& factor:state.propagation_factors)
        factors.push_back(task10p11s_capture_detail::matrixJson(factor));
    for (const auto& row:state.correlation_rows) {
        nlohmann::json stored_row=nlohmann::json::array();
        for (const auto& block:row)
            stored_row.push_back(
                task10p11s_capture_detail::matrixJson(block));
        correlations.push_back(std::move(stored_row));
    }
    nlohmann::json fixed=nlohmann::json::object();
    for (const auto& [id,position]:state.fixed_positions)
        fixed[std::to_string(id)]=
            task10p11s_capture_detail::vector2Json(position);
    nlohmann::json measurement=nullptr;
    if (state.last_measurement.has_value()) {
        const auto& [timestamp,first,second]=*state.last_measurement;
        measurement={timestamp,first,second};
    }
    return {{"mobile_ids",state.mobile_ids},{"means",std::move(means)},
        {"propagation_factors",std::move(factors)},
        {"correlation_rows",std::move(correlations)},
        {"fixed_positions",std::move(fixed)},{"version",state.version},
        {"last_measurement",std::move(measurement)}};
}

template<int Rows,int Columns>
inline Eigen::Matrix<double,Rows,Columns> fixedMatrixFromJson(
    const nlohmann::json& value) {
    const Eigen::MatrixXd matrix=
        task10p11s_capture_detail::matrixFromJson(value);
    if (matrix.rows()!=Rows || matrix.cols()!=Columns)
        throw std::invalid_argument("restart matrix dimension mismatch");
    return matrix;
}

inline DekfRestartState dekfFromJson(const nlohmann::json& value) {
    DekfRestartState state;
    state.mobile_ids=value.at("mobile_ids").get<std::vector<NodeId>>();
    for (const auto& item:value.at("means"))
        state.means.push_back(fixedMatrixFromJson<4,1>(item));
    for (const auto& item:value.at("propagation_factors"))
        state.propagation_factors.push_back(fixedMatrixFromJson<4,4>(item));
    for (const auto& stored_row:value.at("correlation_rows")) {
        std::vector<Eigen::Matrix4d> row;
        for (const auto& item:stored_row)
            row.push_back(fixedMatrixFromJson<4,4>(item));
        state.correlation_rows.push_back(std::move(row));
    }
    for (const auto& item:value.at("fixed_positions").items())
        state.fixed_positions.emplace(
            static_cast<NodeId>(std::stoul(item.key())),
            task10p11s_capture_detail::vector2FromJson(item.value()));
    state.version=value.at("version").get<std::uint64_t>();
    if (!value.at("last_measurement").is_null()) {
        const auto& measurement=value.at("last_measurement");
        if (!measurement.is_array() || measurement.size()!=3)
            throw std::invalid_argument("invalid last measurement key");
        state.last_measurement=std::make_tuple(
            measurement.at(0).get<std::int64_t>(),
            measurement.at(1).get<NodeId>(),
            measurement.at(2).get<NodeId>());
    }
    return state;
}

inline nlohmann::json topologyJson(const std::vector<DirectedEdge>& edges) {
    nlohmann::json result=nlohmann::json::array();
    for (const auto& edge:edges)
        result.push_back({{"reference",edge.reference},{"owner",edge.owner}});
    return result;
}

inline std::vector<DirectedEdge> topologyFromJson(
    const nlohmann::json& value) {
    std::vector<DirectedEdge> result;
    for (const auto& edge:value)
        result.emplace_back(edge.at("reference").get<NodeId>(),
            edge.at("owner").get<NodeId>());
    return result;
}

inline nlohmann::json adapterJson(
    const GrandFinaleFixedRestartState& state) {
    return {{"estimator",dekfJson(state.estimator)},
        {"topology",topologyJson(state.topology)},
        {"mode",static_cast<int>(state.mode)},
        {"topology_version",state.topology_version},
        {"supervisor_last_transition_s",state.supervisor_last_transition_s},
        {"stage_zero_initialized",state.stage_zero_initialized},
        {"range_last_observation_s",doubleMapJson(
            state.range_last_observation_s)},
        {"range_quality",doubleMapJson(state.range_quality)},
        {"range_variance_m2",doubleMapJson(state.range_variance_m2)},
        {"range_batch_count",state.range_batch_count},
        {"last_certification_reason",state.last_certification_reason}};
}

inline GrandFinaleFixedRestartState adapterFromJson(
    const nlohmann::json& value,
    CertifiedCoverageRestartState coverage) {
    GrandFinaleFixedRestartState state;
    state.estimator=dekfFromJson(value.at("estimator"));
    state.coverage=std::move(coverage);
    state.topology=topologyFromJson(value.at("topology"));
    state.mode=static_cast<SupervisorMode>(value.at("mode").get<int>());
    state.topology_version=value.at("topology_version").get<std::uint64_t>();
    state.supervisor_last_transition_s=
        value.at("supervisor_last_transition_s").get<double>();
    state.stage_zero_initialized=
        value.at("stage_zero_initialized").get<bool>();
    state.range_last_observation_s=
        doubleMapFromJson(value.at("range_last_observation_s"));
    state.range_quality=doubleMapFromJson(value.at("range_quality"));
    state.range_variance_m2=
        doubleMapFromJson(value.at("range_variance_m2"));
    state.range_batch_count=value.at("range_batch_count").get<std::size_t>();
    state.last_certification_reason=
        value.at("last_certification_reason").get<std::string>();
    return state;
}

}  // namespace task10p11v_restart_detail

inline nlohmann::json captureTask10p11vRestartFields(
    const Task10p11rFixedBaselineFixture& fixture) {
    return {{"plant",task10p11v_restart_detail::plantJson(fixture.swarm)},
        {"coverage",task10p11v_restart_detail::coverageJson(fixture)},
        {"controller",task10p11v_restart_detail::controllerJson(
            fixture.controller)},
        {"adapter",task10p11v_restart_detail::adapterJson(
            fixture.adapter.fixedRestartState())}};
}

inline nlohmann::json task10p11vRestartStateJson(
    const Task10p11rFixedBaselineFixture& fixture) {
    return captureTask10p11vRestartFields(fixture);
}

inline void restoreTask10p11vRestartState(
    Task10p11rFixedBaselineFixture& fixture,const nlohmann::json& value) {
    const auto& robots=value.at("plant").at("robots");
    if (robots.size()!=fixture.swarm.robots.size())
        throw std::invalid_argument("restart plant owner count mismatch");
    std::set<NodeId> restored_ids;
    for (const auto& stored:robots) {
        const NodeId id=stored.at("id").get<NodeId>();
        auto found=std::find_if(fixture.swarm.robots.begin(),
            fixture.swarm.robots.end(),[id](const auto& robot) {
                return robot->id==id;
            });
        if (found==fixture.swarm.robots.end() || !restored_ids.insert(id).second)
            throw std::invalid_argument("restart plant owner mismatch");
        const Eigen::MatrixXd state_matrix=
            task10p11s_capture_detail::matrixFromJson(stored.at("state"));
        const Eigen::MatrixXd control_matrix=
            task10p11s_capture_detail::matrixFromJson(stored.at("control"));
        if (state_matrix.cols()!=1 || control_matrix.cols()!=1)
            throw std::invalid_argument("restart plant vector shape mismatch");
        (*found)->model->setStateVector(state_matrix.col(0));
        (*found)->model->setControlInput(control_matrix.col(0));
        const double runtime_s=stored.at("runtime_s").get<double>();
        if (!std::isfinite(runtime_s))
            throw std::invalid_argument("nonfinite restart runtime");
        (*found)->runtime=runtime_s;
    }

    const auto& coverage=value.at("coverage");
    task10p11v_restart_detail::restoreGridJson(
        coverage.at("swarm_ground_truth"),fixture.swarm.gridWorldGroundTruth);
    for (const auto& robot:fixture.swarm.robots) {
        const auto key=std::to_string(robot->id);
        task10p11v_restart_detail::restoreGridJson(
            coverage.at("robot_local").at(key),robot->gridWorld);
    }
    CertifiedCoverageRestartState tracker=
        fixture.adapter.coverage().restartState();
    GridWorld tracker_truth=fixture.adapter.coverage().truthGrid();
    GridWorld tracker_certified=fixture.adapter.coverage().certifiedGrid();
    task10p11v_restart_detail::restoreGridJson(
        coverage.at("certified_tracker_truth"),tracker_truth);
    task10p11v_restart_detail::restoreGridJson(
        coverage.at("certified_tracker_certified"),tracker_certified);
    tracker.truth=std::move(tracker_truth.vis);
    tracker.certified=std::move(tracker_certified.vis);
    fixture.adapter.restoreFixedRestartState(
        task10p11v_restart_detail::adapterFromJson(
            value.at("adapter"),std::move(tracker)));
    fixture.controller.restoreRestartState(
        task10p11v_restart_detail::controllerFromJson(
            value.at("controller")));
    const nlohmann::json expected{{"plant",value.at("plant")},
        {"coverage",value.at("coverage")},
        {"controller",value.at("controller")},
        {"adapter",value.at("adapter")}};
    if (task10p11vRestartStateJson(fixture)!=expected)
        throw std::runtime_error("restart state readback mismatch");
}

inline nlohmann::json readTask10p11vJson(
    const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error(
        "failed to open Task 10.11v JSON: "+path.string());
    nlohmann::json result;
    input>>result;
    if (!input && !input.eof())
        throw std::runtime_error(
            "failed to read Task 10.11v JSON: "+path.string());
    return result;
}

inline void validateTask10p11vFiniteJson(const nlohmann::json& value,
    const std::string& path="$") {
    if (value.is_number_float() &&
        !std::isfinite(value.get<double>()))
        throw std::invalid_argument(
            "non-finite Task 10.11v JSON value at "+path);
    if (value.is_array()) {
        for (std::size_t index=0;index<value.size();++index)
            validateTask10p11vFiniteJson(value.at(index),path+"["+
                std::to_string(index)+"]");
    } else if (value.is_object()) {
        for (auto item=value.begin();item!=value.end();++item)
            validateTask10p11vFiniteJson(item.value(),path+"."+item.key());
    }
}

inline void writeTask10p11vJson(const std::filesystem::path& path,
    const nlohmann::json& value) {
    validateTask10p11vFiniteJson(value);
    if (std::filesystem::exists(path))
        throw std::runtime_error(
            "refusing to overwrite Task 10.11v evidence: "+path.string());
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    const auto nonce=std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path temporary=path.string()+".tmp."+
        std::to_string(nonce);
    {
        std::ofstream output(temporary,std::ios::binary|std::ios::trunc);
        if (!output) throw std::runtime_error(
            "failed to create Task 10.11v temporary file");
        output<<value.dump(2)<<'\n';
        output.flush();
        if (!output) throw std::runtime_error(
            "failed to flush Task 10.11v temporary file");
    }
    if (readTask10p11vJson(temporary)!=value)
        throw std::runtime_error(
            "Task 10.11v temporary-file readback mismatch");
    std::filesystem::rename(temporary,path);
    if (readTask10p11vJson(path)!=value)
        throw std::runtime_error("Task 10.11v published-file readback mismatch");
}

struct Task10p11vSparseMetadata {
    std::optional<std::string> active_pair;
    std::size_t cycle=0;
};

inline nlohmann::json makeTask10p11vSparseRestartCheckpoint(
    const Task10p11rFixedBaselineFixture& fixture,
    std::optional<std::string> active_pair,std::size_t cycle) {
    return {{"protocol","task10p11v-sparse-restart-v1"},
        {"cycle",cycle},{"active_pair",active_pair.has_value()
            ?nlohmann::json(*active_pair):nlohmann::json(nullptr)},
        {"restart_state",task10p11vRestartStateJson(fixture)}};
}

inline Task10p11vSparseMetadata restoreTask10p11vSparseRestartCheckpoint(
    Task10p11rFixedBaselineFixture& fixture,const nlohmann::json& value) {
    if (value.at("protocol").get<std::string>()!=
        "task10p11v-sparse-restart-v1")
        throw std::invalid_argument("unsupported sparse restart protocol");
    Task10p11vSparseMetadata result;
    result.cycle=value.at("cycle").get<std::size_t>();
    if (!value.at("active_pair").is_null())
        result.active_pair=value.at("active_pair").get<std::string>();
    restoreTask10p11vRestartState(fixture,value.at("restart_state"));
    return result;
}

inline nlohmann::json makeTask10p11vRestartCheckpoint(
    nlohmann::json offline_snapshot,
    nlohmann::json captured_fields,
    const std::string& capture_event) {
    if (capture_event.empty())
        throw std::invalid_argument("capture event is empty");
    nlohmann::json checkpoint{
        {"protocol","task10p11v-minimal-restart-checkpoint-v1"},
        {"capture_event",capture_event},
        {"runtime_s",offline_snapshot.at("runtime").at("runtime_s")},
        {"plant",std::move(captured_fields.at("plant"))},
        {"coverage",std::move(captured_fields.at("coverage"))},
        {"controller",std::move(captured_fields.at("controller"))},
        {"adapter",std::move(captured_fields.at("adapter"))},
        {"estimator_dekf",{{"estimator_token",
                offline_snapshot.at("runtime").at("estimator_token")},
            {"estimate",offline_snapshot.at("estimator")},
            {"dekf_internal",offline_snapshot.at("dekf_internal")}}},
        {"topology",offline_snapshot.at("runtime").at("fixed_topology")},
        {"exact_zoh",offline_snapshot.at("successor_parameters")},
        {"restart_capability",{{"supported",true},
            {"blockers",nlohmann::json::array()}}}};
    offline_snapshot["restart_checkpoint"]=std::move(checkpoint);
    return offline_snapshot;
}

inline nlohmann::json makeTask10p11vRestartCheckpoint(
    nlohmann::json offline_snapshot,
    const Task10p11rFixedBaselineFixture& fixture,
    const std::string& capture_event) {
    return makeTask10p11vRestartCheckpoint(std::move(offline_snapshot),
        captureTask10p11vRestartFields(fixture),capture_event);
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
        "adapter","estimator_dekf","topology","exact_zoh",
        "restart_capability"};
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
                const auto& control=robot.at("control");
                plant_shape=plant_shape && state.size()>=6 &&
                    control.size()>=3 &&
                    std::abs(robot.at("runtime_s").get<double>()-runtime_s)
                        <=1e-12;
                for (const auto& scalar:state)
                    plant_shape=plant_shape && scalar.is_array() &&
                        scalar.size()==1 &&
                        std::isfinite(scalar.at(0).get<double>());
                for (const auto& scalar:control)
                    plant_shape=plant_shape && scalar.is_array() &&
                        scalar.size()==1 &&
                        std::isfinite(scalar.at(0).get<double>());
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
                (controller.at("targets").size()!=0 &&
                 controller.at("targets").size()!=14))
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
