#pragma once

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "json.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

namespace task10p11s_capture_detail {

inline nlohmann::json vector2Json(const Eigen::Vector2d& value) {
    return {value.x(),value.y()};
}

inline Eigen::Vector2d vector2FromJson(const nlohmann::json& value) {
    if (!value.is_array() || value.size()!=2)
        throw std::invalid_argument("expected vector2");
    Eigen::Vector2d result(value.at(0).get<double>(),value.at(1).get<double>());
    if (!result.allFinite()) throw std::invalid_argument("nonfinite vector2");
    return result;
}

template<class Derived>
nlohmann::json matrixJson(const Eigen::MatrixBase<Derived>& value) {
    nlohmann::json result=nlohmann::json::array();
    for (Eigen::Index row=0;row<value.rows();++row) {
        nlohmann::json line=nlohmann::json::array();
        for (Eigen::Index column=0;column<value.cols();++column)
            line.push_back(value(row,column));
        result.push_back(std::move(line));
    }
    return result;
}

inline Eigen::MatrixXd matrixFromJson(const nlohmann::json& value) {
    if (!value.is_array() || value.empty() || !value.front().is_array() ||
        value.front().empty())
        throw std::invalid_argument("expected nonempty matrix");
    const Eigen::Index rows=static_cast<Eigen::Index>(value.size());
    const Eigen::Index columns=
        static_cast<Eigen::Index>(value.front().size());
    Eigen::MatrixXd result(rows,columns);
    for (Eigen::Index row=0;row<rows;++row) {
        if (!value.at(static_cast<std::size_t>(row)).is_array() ||
            value.at(static_cast<std::size_t>(row)).size()!=
                static_cast<std::size_t>(columns))
            throw std::invalid_argument("ragged matrix");
        for (Eigen::Index column=0;column<columns;++column)
            result(row,column)=value.at(static_cast<std::size_t>(row)).at(
                static_cast<std::size_t>(column)).get<double>();
    }
    if (!result.allFinite()) throw std::invalid_argument("nonfinite matrix");
    return result;
}

inline nlohmann::json stateJson(const PairwiseSecondOrderState2D& state) {
    return {{"position",{state.position.x,state.position.y}},
            {"velocity",vector2Json(state.velocity)},
            {"acceleration",vector2Json(state.acceleration)}};
}

inline PairwiseSecondOrderState2D stateFromJson(const nlohmann::json& value) {
    const auto position=vector2FromJson(value.at("position"));
    return {Point(position.x(),position.y()),
            vector2FromJson(value.at("velocity")),
            vector2FromJson(value.at("acceleration"))};
}

inline nlohmann::json tubeJson(const PairwiseSnapshotTube& tube) {
    return {{"position_radius_m",tube.position_radius_m},
            {"velocity_radius_mps",tube.velocity_radius_mps},
            {"provenance",static_cast<int>(tube.provenance)}};
}

inline PairwiseSnapshotTube tubeFromJson(const nlohmann::json& value) {
    PairwiseSnapshotTube result;
    result.position_radius_m=value.at("position_radius_m").get<double>();
    result.velocity_radius_mps=value.at("velocity_radius_mps").get<double>();
    result.provenance=static_cast<SnapshotTubeProvenance>(
        value.at("provenance").get<int>());
    if (!std::isfinite(result.position_radius_m) ||
        !std::isfinite(result.velocity_radius_mps) ||
        result.position_radius_m<0.0 || result.velocity_radius_mps<0.0)
        throw std::invalid_argument("invalid pair tube");
    return result;
}

inline nlohmann::json singleTubeJson(const SingleSnapshotTube2D& tube) {
    return {{"position_radius_m",tube.position_radius_m},
            {"velocity_radius_mps",tube.velocity_radius_mps},
            {"provenance",static_cast<int>(tube.provenance)}};
}

inline SingleSnapshotTube2D singleTubeFromJson(const nlohmann::json& value) {
    SingleSnapshotTube2D result;
    result.position_radius_m=value.at("position_radius_m").get<double>();
    result.velocity_radius_mps=value.at("velocity_radius_mps").get<double>();
    result.provenance=static_cast<SnapshotTubeProvenance>(
        value.at("provenance").get<int>());
    if (!std::isfinite(result.position_radius_m) ||
        !std::isfinite(result.velocity_radius_mps) ||
        result.position_radius_m<0.0 || result.velocity_radius_mps<0.0)
        throw std::invalid_argument("invalid single tube");
    return result;
}

inline nlohmann::json specJson(const PairwiseSecondOrderRowSpec& spec) {
    return {{"kind",static_cast<int>(spec.kind)},
            {"distance_limit",spec.distanceLimit},
            {"uncertainty",spec.uncertainty},{"k",spec.k},
            {"lambda1",spec.lambda1},{"lambda2",spec.lambda2},
            {"total_reserve",spec.totalReserve}};
}

inline PairwiseSecondOrderRowSpec specFromJson(const nlohmann::json& value) {
    return {static_cast<PairwiseSecondOrderBarrierKind>(
                value.at("kind").get<int>()),
            value.at("distance_limit").get<double>(),
            value.at("uncertainty").get<double>(),
            value.at("k").get<double>(),value.at("lambda1").get<double>(),
            value.at("lambda2").get<double>(),
            value.at("total_reserve").get<double>()};
}

inline nlohmann::json rowJson(const CanonicalHardRow& row) {
    const auto scalar=[](double value) {
        return std::isfinite(value)?nlohmann::json(value):nlohmann::json(nullptr);
    };
    nlohmann::json result{{"id",row.id},{"kind",static_cast<int>(row.kind)},
        {"owner",row.owner},{"normal",vector2Json(row.normal)},
        {"control_coefficient",vector2Json(row.control_coefficient)},
        {"constant",row.constant},{"responsibility",row.responsibility},
        {"participates_in_gamma",row.participates_in_gamma},
        {"barrier_h",scalar(row.barrier_h)},
        {"barrier_psi1",scalar(row.barrier_psi1)},
        {"barrier_hdot",scalar(row.barrier_hdot)},
        {"coefficient_uncertainty_reserve",
            row.coefficient_uncertainty_reserve},
        {"position_uncertainty_reserve_m",
            row.position_uncertainty_reserve_m},
        {"velocity_uncertainty_reserve_mps",
            row.velocity_uncertainty_reserve_mps}};
    result["peer"]=row.peer.has_value()?nlohmann::json(*row.peer):
        nlohmann::json(nullptr);
    result["tube_provenance"]=row.tube_provenance.has_value()
        ?nlohmann::json(static_cast<int>(*row.tube_provenance)):
         nlohmann::json(nullptr);
    return result;
}

inline bool sameRow(const CanonicalHardRow& row,const nlohmann::json& stored) {
    const auto close=[](double lhs,const nlohmann::json& rhs) {
        if (!std::isfinite(lhs)) return rhs.is_null();
        return rhs.is_number() &&
            std::abs(lhs-rhs.get<double>())<=1e-12;
    };
    const auto peer=stored.at("peer").is_null()?std::optional<NodeId>{}:
        std::optional<NodeId>{stored.at("peer").get<NodeId>()};
    const auto provenance=stored.at("tube_provenance").is_null()
        ?std::optional<SnapshotTubeProvenance>{}
        :std::optional<SnapshotTubeProvenance>{
            static_cast<SnapshotTubeProvenance>(
                stored.at("tube_provenance").get<int>())};
    return row.id==stored.at("id").get<std::string>() &&
        static_cast<int>(row.kind)==stored.at("kind").get<int>() &&
        row.owner==stored.at("owner").get<NodeId>() && row.peer==peer &&
        row.normal.isApprox(vector2FromJson(stored.at("normal")),1e-12) &&
        row.control_coefficient.isApprox(
            vector2FromJson(stored.at("control_coefficient")),1e-12) &&
        close(row.constant,stored.at("constant")) &&
        close(row.responsibility,stored.at("responsibility")) &&
        row.participates_in_gamma==
            stored.at("participates_in_gamma").get<bool>() &&
        close(row.barrier_h,stored.at("barrier_h")) &&
        close(row.barrier_psi1,stored.at("barrier_psi1")) &&
        close(row.barrier_hdot,stored.at("barrier_hdot")) &&
        close(row.coefficient_uncertainty_reserve,
            stored.at("coefficient_uncertainty_reserve")) &&
        close(row.position_uncertainty_reserve_m,
            stored.at("position_uncertainty_reserve_m")) &&
        close(row.velocity_uncertainty_reserve_mps,
            stored.at("velocity_uncertainty_reserve_mps")) &&
        row.tube_provenance==provenance;
}

inline nlohmann::json requestJson(const CanonicalHardRowRequest& request) {
    nlohmann::json result{{"mobile_ids",request.mobile_ids},
        {"fixed_ids",request.fixed_ids},
        {"reference_edges",nlohmann::json::array()},
        {"collision_pairs",nlohmann::json::array()},
        {"reference_spec",specJson(request.reference_spec)},
        {"collision_spec",specJson(request.collision_spec)},
        {"acceleration_half_box",request.acceleration_half_box},
        {"speed_limit_mps",request.speed_limit_mps},
        {"speed_cbf_gain",request.speed_cbf_gain},
        {"plant_speed_facet_count",request.plant_speed_facet_count},
        {"plant_speed_dt_s",request.plant_speed_dt_s},
        {"require_snapshot_robust_rows",request.require_snapshot_robust_rows},
        {"states",nlohmann::json::object()},
        {"reference_snapshot_tubes",nlohmann::json::object()},
        {"collision_snapshot_tubes",nlohmann::json::object()},
        {"workspace_facets",nlohmann::json::array()},
        {"workspace_snapshot_tubes",nlohmann::json::object()},
        {"speed_snapshot_tubes",nlohmann::json::object()},
        {"plant_speed_snapshot_tubes",nlohmann::json::object()}};
    for (const auto& [id,state]:request.states)
        result["states"][std::to_string(id)]=stateJson(state);
    for (const auto& edge:request.reference_edges)
        result["reference_edges"].push_back(
            {{"reference",edge.reference},{"owner",edge.owner}});
    for (const auto& edge:request.collision_pairs)
        result["collision_pairs"].push_back(
            {{"first",edge.first},{"second",edge.second}});
    for (const auto& [id,tube]:request.reference_snapshot_tubes)
        result["reference_snapshot_tubes"][id]=tubeJson(tube);
    for (const auto& [id,tube]:request.collision_snapshot_tubes)
        result["collision_snapshot_tubes"][id]=tubeJson(tube);
    for (const auto& facet:request.workspace_facets)
        result["workspace_facets"].push_back({{"id",facet.id},
            {"outward_normal",vector2Json(facet.outward_normal)},
            {"offset_m",facet.offset_m}});
    const auto append_single=[&](const char* key,
        const std::map<NodeId,SingleSnapshotTube2D>& values) {
        for (const auto& [id,tube]:values)
            result[key][std::to_string(id)]=singleTubeJson(tube);
    };
    append_single("workspace_snapshot_tubes",request.workspace_snapshot_tubes);
    append_single("speed_snapshot_tubes",request.speed_snapshot_tubes);
    append_single("plant_speed_snapshot_tubes",
        request.plant_speed_snapshot_tubes);
    return result;
}

inline CanonicalHardRowRequest requestFromJson(const nlohmann::json& value) {
    CanonicalHardRowRequest result;
    result.mobile_ids=value.at("mobile_ids").get<std::vector<NodeId>>();
    result.fixed_ids=value.at("fixed_ids").get<std::vector<NodeId>>();
    for (const auto& item:value.at("states").items())
        result.states.emplace(static_cast<NodeId>(std::stoul(item.key())),
            stateFromJson(item.value()));
    for (const auto& edge:value.at("reference_edges"))
        result.reference_edges.emplace_back(
            edge.at("reference").get<NodeId>(),edge.at("owner").get<NodeId>());
    for (const auto& edge:value.at("collision_pairs"))
        result.collision_pairs.push_back(UndirectedEdge::canonical(
            edge.at("first").get<NodeId>(),edge.at("second").get<NodeId>()));
    result.reference_spec=specFromJson(value.at("reference_spec"));
    result.collision_spec=specFromJson(value.at("collision_spec"));
    result.acceleration_half_box=value.at("acceleration_half_box").get<double>();
    result.speed_limit_mps=value.at("speed_limit_mps").get<double>();
    result.speed_cbf_gain=value.at("speed_cbf_gain").get<double>();
    result.plant_speed_facet_count=
        value.at("plant_speed_facet_count").get<std::size_t>();
    result.plant_speed_dt_s=value.at("plant_speed_dt_s").get<double>();
    result.require_snapshot_robust_rows=
        value.at("require_snapshot_robust_rows").get<bool>();
    const auto read_tubes=[&](const char* key,
        std::map<std::string,PairwiseSnapshotTube>& destination) {
        for (const auto& item:value.at(key).items())
            destination.emplace(item.key(),tubeFromJson(item.value()));
    };
    read_tubes("reference_snapshot_tubes",result.reference_snapshot_tubes);
    read_tubes("collision_snapshot_tubes",result.collision_snapshot_tubes);
    for (const auto& facet:value.at("workspace_facets"))
        result.workspace_facets.push_back({facet.at("id").get<std::string>(),
            vector2FromJson(facet.at("outward_normal")),
            facet.at("offset_m").get<double>()});
    const auto read_single=[&](const char* key,
        std::map<NodeId,SingleSnapshotTube2D>& destination) {
        for (const auto& item:value.at(key).items())
            destination.emplace(static_cast<NodeId>(std::stoul(item.key())),
                singleTubeFromJson(item.value()));
    };
    read_single("workspace_snapshot_tubes",result.workspace_snapshot_tubes);
    read_single("speed_snapshot_tubes",result.speed_snapshot_tubes);
    read_single("plant_speed_snapshot_tubes",
        result.plant_speed_snapshot_tubes);
    return result;
}

inline nlohmann::json estimateJson(const JointEstimateSnapshot& estimate) {
    nlohmann::json fixed=nlohmann::json::object();
    for (const auto& [id,position]:estimate.fixed_positions)
        fixed[std::to_string(id)]=vector2Json(position);
    nlohmann::json mean=nlohmann::json::array();
    for (Eigen::Index i=0;i<estimate.mean.size();++i)
        mean.push_back(estimate.mean(i));
    return {{"mobile_ids",estimate.mobile_ids},{"mean",mean},
            {"covariance",matrixJson(estimate.covariance)},
            {"fixed_positions",fixed}};
}

inline JointEstimateSnapshot estimateFromJson(const nlohmann::json& value) {
    JointEstimateSnapshot result;
    result.mobile_ids=value.at("mobile_ids").get<std::vector<NodeId>>();
    result.mean.resize(static_cast<Eigen::Index>(value.at("mean").size()));
    for (Eigen::Index i=0;i<result.mean.size();++i)
        result.mean(i)=value.at("mean").at(static_cast<std::size_t>(i)).
            get<double>();
    result.covariance=matrixFromJson(value.at("covariance"));
    for (const auto& item:value.at("fixed_positions").items())
        result.fixed_positions.emplace(
            static_cast<NodeId>(std::stoul(item.key())),
            vector2FromJson(item.value()));
    if (!result.mean.allFinite() ||
        result.mean.size()!=static_cast<Eigen::Index>(4*result.mobile_ids.size()) ||
        result.covariance.rows()!=result.mean.size() ||
        result.covariance.cols()!=result.mean.size())
        throw std::invalid_argument("invalid estimator snapshot");
    return result;
}

inline nlohmann::json dekfJson(const DekfInternalAudit& audit) {
    nlohmann::json factors=nlohmann::json::array();
    for (const auto& factor:audit.propagation_factors)
        factors.push_back(matrixJson(factor));
    nlohmann::json rows=nlohmann::json::array();
    for (const auto& row:audit.correlation_rows) {
        nlohmann::json stored=nlohmann::json::array();
        for (const auto& block:row) stored.push_back(matrixJson(block));
        rows.push_back(std::move(stored));
    }
    return {{"propagation_factors",factors},{"correlation_rows",rows},
            {"has_last_measurement",audit.has_last_measurement}};
}

inline nlohmann::json nominalJson(
    const std::map<NodeId,Eigen::Vector2d>& nominal) {
    nlohmann::json controls=nlohmann::json::object();
    for (const auto& [id,value]:nominal)
        controls[std::to_string(id)]=vector2Json(value);
    return controls;
}

inline std::map<NodeId,Eigen::Vector2d> nominalFromJson(
    const nlohmann::json& value) {
    std::map<NodeId,Eigen::Vector2d> result;
    for (const auto& item:value.items())
        result.emplace(static_cast<NodeId>(std::stoul(item.key())),
            vector2FromJson(item.value()));
    return result;
}

inline std::vector<double> orderedObjective(
    const std::vector<NodeId>& ids,
    const std::map<NodeId,Eigen::Vector2d>& nominal) {
    std::vector<double> result;
    result.reserve(2*ids.size());
    for (NodeId id:ids) {
        const auto found=nominal.find(id);
        if (found==nominal.end() || !found->second.allFinite())
            throw std::invalid_argument("nominal control missing");
        result.push_back(found->second.x());
        result.push_back(found->second.y());
    }
    return result;
}

}  // namespace task10p11s_capture_detail

struct Task10p11sSnapshotValidation {
    bool complete=false;
    bool rows_match=false;
    bool objective_matches=false;
    std::size_t owner_count=0;
    std::size_t row_count=0;
    std::string reason;
};

inline nlohmann::json makeTask10p11sSnapshot(
    const GrandFinaleRuntimeSnapshot& runtime,
    const CanonicalHardRowRequest& request,
    const std::map<NodeId,Eigen::Vector2d>& nominal_controls,
    const GrandFinaleSwarmAdapterConfig& config) {
    using namespace task10p11s_capture_detail;
    const auto rows=buildCanonicalHardRows(request);
    nlohmann::json stored_rows=nlohmann::json::array();
    std::map<NodeId,std::size_t> counts;
    for (const auto& row:rows) {
        stored_rows.push_back(rowJson(row));
        ++counts[row.owner];
    }
    nlohmann::json owner_counts=nlohmann::json::object();
    for (NodeId id:request.mobile_ids)
        owner_counts[std::to_string(id)]=counts[id];
    nlohmann::json topology=nlohmann::json::array();
    for (const auto& edge:runtime.topology)
        topology.push_back({{"reference",edge.reference},{"owner",edge.owner}});
    nlohmann::json links=nlohmann::json::object();
    for (const auto& [id,link]:runtime.range_links)
        links[id]={{"age_s",link.age_s},{"quality",link.quality},
                   {"variance_m2",link.variance_m2}};
    return {{"protocol","task10p11s-minimal-snapshot-v1"},
        {"runtime",{{"runtime_s",runtime.runtime_s},
            {"mode",static_cast<int>(runtime.mode)},
            {"estimator_token",runtime.estimator_token},
            {"topology_token",runtime.topology_token},
            {"freshness",static_cast<int>(runtime.freshness)},
            {"timer_since_supervisor_transition_s",
                runtime.timer_since_supervisor_transition_s},
            {"supervisor_transition_pending",
                runtime.supervisor_transition_pending},
            {"adapter_transition_pending",runtime.adapter_transition_pending},
            {"pending_is_retreat",runtime.pending_is_retreat},
            {"transition_stack_size",runtime.transition_stack_size},
            {"union_control_cycles",runtime.union_control_cycles},
            {"fixed_topology",topology},{"range_links",links}}},
        {"estimator",estimateJson(runtime.estimate)},
        {"dekf_internal",dekfJson(runtime.dekf)},
        {"canonical_request",requestJson(request)},
        {"actual_rows",stored_rows},{"owner_row_counts",owner_counts},
        {"nominal_controls",nominalJson(nominal_controls)},
        {"objective_28d",orderedObjective(request.mobile_ids,nominal_controls)},
        {"successor_parameters",{{"dt_s",config.dt_s},
            {"estimator_acceleration_variance",
                config.estimator_acceleration_variance},
            {"uncertainty_sigma",config.uncertainty_sigma},
            {"single_position_support_m",
                config.certified_shadow_single_position_support_m},
            {"single_velocity_support_mps",
                config.certified_shadow_single_velocity_support_mps},
            {"relative_position_support_m",
                config.certified_shadow_relative_position_support_m},
            {"relative_velocity_support_mps",
                config.certified_shadow_relative_velocity_support_mps}}}};
}

inline Task10p11sSnapshotValidation validateTask10p11sSnapshot(
    const nlohmann::json& snapshot) {
    using namespace task10p11s_capture_detail;
    Task10p11sSnapshotValidation result;
    try {
        if (snapshot.at("protocol").get<std::string>()!=
            "task10p11s-minimal-snapshot-v1") {
            result.reason="protocol_mismatch";
            return result;
        }
        const auto request=requestFromJson(snapshot.at("canonical_request"));
        const auto estimate=estimateFromJson(snapshot.at("estimator"));
        const auto nominal=nominalFromJson(snapshot.at("nominal_controls"));
        const auto rows=buildCanonicalHardRows(request);
        const auto& stored=snapshot.at("actual_rows");
        result.owner_count=request.mobile_ids.size();
        result.row_count=rows.size();
        result.rows_match=stored.is_array() && stored.size()==rows.size();
        for (std::size_t index=0;result.rows_match && index<rows.size();++index)
            result.rows_match=sameRow(rows[index],stored.at(index));
        const auto objective=orderedObjective(request.mobile_ids,nominal);
        result.objective_matches=
            snapshot.at("objective_28d").get<std::vector<double>>()==objective;
        std::set<NodeId> unique(request.mobile_ids.begin(),request.mobile_ids.end());
        bool owner_rows_complete=true;
        for (NodeId owner:request.mobile_ids)
            owner_rows_complete=owner_rows_complete &&
                snapshot.at("owner_row_counts").at(std::to_string(owner)).
                    get<std::size_t>()>0;
        result.complete=request.mobile_ids.size()==14 && unique.size()==14 &&
            estimate.mobile_ids==request.mobile_ids && nominal.size()==14 &&
            objective.size()==28 && owner_rows_complete &&
            result.rows_match && result.objective_matches &&
            snapshot.at("runtime").at("fixed_topology")==
                snapshot.at("canonical_request").at("reference_edges") &&
            snapshot.at("dekf_internal").contains("propagation_factors") &&
            snapshot.at("successor_parameters").contains("dt_s");
        result.reason=result.complete?"complete":"incomplete";
    } catch (const std::exception& error) {
        result.reason=std::string("invalid_snapshot:")+error.what();
    }
    return result;
}

inline CanonicalHardRowRequest rebuildTask10p11sSuccessorRequest(
    const nlohmann::json& snapshot,
    const std::map<NodeId,Eigen::Vector2d>& controls) {
    using namespace task10p11s_capture_detail;
    CanonicalHardRowRequest result=requestFromJson(
        snapshot.at("canonical_request"));
    const auto current=estimateFromJson(snapshot.at("estimator"));
    const auto& parameters=snapshot.at("successor_parameters");
    const double dt_s=parameters.at("dt_s").get<double>();
    const double acceleration_variance=
        parameters.at("estimator_acceleration_variance").get<double>();
    const double sigma=parameters.at("uncertainty_sigma").get<double>();
    const double single_position_support=
        parameters.at("single_position_support_m").get<double>();
    const double single_velocity_support=
        parameters.at("single_velocity_support_mps").get<double>();
    const double relative_position_support=
        parameters.at("relative_position_support_m").get<double>();
    const double relative_velocity_support=
        parameters.at("relative_velocity_support_mps").get<double>();
    const auto successor=predictNoMeasurementSnapshot(
        current,controls,dt_s,acceleration_variance);
    for (std::size_t index=0;index<successor.mobile_ids.size();++index) {
        const Eigen::Vector4d state=successor.mean.segment<4>(4*index);
        result.states.at(successor.mobile_ids[index])={
            Point(state.x(),state.y()),state.tail<2>(),Eigen::Vector2d::Zero()};
    }
    const auto pair_tube=[&](NodeId first,NodeId second) {
        const double first_position=std::sqrt(std::max(
            0.0,detail::maximumPositionEigenvalue(successor,first)));
        const double second_position=std::sqrt(std::max(
            0.0,detail::maximumPositionEigenvalue(successor,second)));
        const double first_velocity=std::sqrt(std::max(
            0.0,detail::maximumVelocityEigenvalue(successor,first)));
        const double second_velocity=std::sqrt(std::max(
            0.0,detail::maximumVelocityEigenvalue(successor,second)));
        return PairwiseSnapshotTube{
            sigma*(first_position+second_position)+relative_position_support,
            sigma*(first_velocity+second_velocity)+relative_velocity_support,
            SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    };
    result.reference_snapshot_tubes.clear();
    for (const auto& edge:result.reference_edges)
        result.reference_snapshot_tubes.emplace(
            edge.id(),pair_tube(edge.owner,edge.reference));
    result.collision_snapshot_tubes.clear();
    for (const auto& edge:result.collision_pairs)
        result.collision_snapshot_tubes.emplace(
            edge.id(),pair_tube(edge.first,edge.second));
    for (NodeId id:result.mobile_ids) {
        const double position_sigma=std::sqrt(std::max(
            0.0,detail::maximumPositionEigenvalue(successor,id)));
        const double velocity_sigma=std::sqrt(std::max(
            0.0,detail::maximumVelocityEigenvalue(successor,id)));
        if (!result.workspace_snapshot_tubes.empty())
            result.workspace_snapshot_tubes[id]={
                sigma*position_sigma+single_position_support,
                sigma*velocity_sigma,
                SnapshotTubeProvenance::CovarianceSigmaDevelopment};
        if (!result.plant_speed_snapshot_tubes.empty())
            result.plant_speed_snapshot_tubes[id]={0.0,
                sigma*velocity_sigma+single_velocity_support,
                SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    }
    result.plant_speed_dt_s=dt_s;
    return result;
}

inline void writeTask10p11sSnapshot(
    const std::filesystem::path& path,nlohmann::json snapshot) {
    const auto validation=validateTask10p11sSnapshot(snapshot);
    snapshot["preflight"]={{"complete",validation.complete},
        {"rows_match",validation.rows_match},
        {"objective_matches",validation.objective_matches},
        {"owner_count",validation.owner_count},{"row_count",validation.row_count},
        {"reason",validation.reason}};
    if (!validation.complete)
        throw std::runtime_error("snapshot validation failed: "+validation.reason);
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) throw std::runtime_error("cannot write snapshot");
    output<<snapshot.dump(2)<<'\n';
}

inline nlohmann::json readTask10p11sSnapshot(
    const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot read snapshot");
    nlohmann::json result;
    input>>result;
    return result;
}

}  // namespace gf
