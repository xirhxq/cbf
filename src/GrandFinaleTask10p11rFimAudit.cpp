#include "grand_finale/CentralizedEkfOracle.hpp"
#include "grand_finale/Task10p11rFimGraphSemantics.hpp"

#include <fstream>
#include <iostream>

namespace {

using json=nlohmann::json;

std::vector<gf::MobileEstimate> mobileEstimates(
    const gf::JointEstimateSnapshot& snapshot) {
    std::vector<gf::MobileEstimate> result;
    for (std::size_t index=0;index<snapshot.mobile_ids.size();++index) {
        const Eigen::Index offset=static_cast<Eigen::Index>(4*index);
        result.push_back({snapshot.mobile_ids[index],
            snapshot.mean.segment<4>(offset),
            snapshot.covariance.block<4,4>(offset,offset)});
    }
    return result;
}

std::vector<gf::RangeMeasurement> measurements(
    const std::vector<gf::AcceptedRangeUpdateAudit>& batch) {
    std::vector<gf::RangeMeasurement> result;
    for (const auto& update:batch) result.push_back(update.measurement);
    return result;
}

json matrix2(const Eigen::Matrix2d& value) {
    return json::array({value(0,0),value(0,1),value(1,0),value(1,1)});
}

json eigenvalues(const Eigen::Matrix2d& value) {
    const auto eigen=Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(value)
        .eigenvalues();
    return json::array({eigen.x(),eigen.y()});
}

json edgeSet(const gf::FimEdgeSetAudit& value) {
    json edges=json::array();
    for (const auto& edge:value.edges)
        edges.push_back({{"id",edge.id()},{"reference",edge.reference},
            {"owner",edge.owner}});
    return {{"edges",edges},{"edge_count",edges.size()},
        {"range_variances_m2",value.range_variances_m2},
        {"nominal_eigenvalue",value.nominal_eigenvalue},
        {"posterior_proxy_eigenvalue",value.posterior_proxy_eigenvalue},
        {"robust_cone_lower_bound",value.robust_cone_lower_bound},
        {"robust_gate_valid",value.robust_gate_valid}};
}

json batchAudit(const gf::GrandFinaleRuntimeSnapshot& runtime,
    const std::vector<gf::AcceptedRangeUpdateAudit>& batch) {
    json result=json::array();
    for (const auto& update:batch) {
        const auto& measurement=update.measurement;
        const auto first=gf::detail::nodePosition(
            runtime.estimate,measurement.edge.first);
        const auto second=gf::detail::nodePosition(
            runtime.estimate,measurement.edge.second);
        const auto history=runtime.range_links.at(measurement.edge.id());
        result.push_back({{"timestamp_ns",measurement.timestamp_ns},
            {"edge",measurement.edge.id()},
            {"first",measurement.edge.first},
            {"second",measurement.edge.second},
            {"interim_master",update.master},
            {"accepted",true},
            {"measured_range_m",measurement.range_m},
            {"posterior_mean_range_m",(first-second).norm()},
            {"measurement_variance_m2",measurement.variance_m2},
            {"innovation_m",update.innovation},
            {"innovation_variance_m2",update.innovation_variance},
            {"range_history_age_s",history.age_s},
            {"range_history_quality",history.quality},
            {"range_history_variance_m2",history.variance_m2}});
    }
    return result;
}

json posteriorAudit(const gf::JointEstimateSnapshot& centralized,
    const gf::JointEstimateSnapshot& distributed,gf::NodeId owner) {
    const auto central_cov=gf::marginalPositionCovariance(centralized,owner);
    const auto dekf_cov=gf::marginalPositionCovariance(distributed,owner);
    const auto central_info=
        gf::schurEffectivePositionInformation(centralized,owner);
    const auto dekf_info=gf::schurEffectivePositionInformation(distributed,owner);
    return {{"centralized_position_covariance",matrix2(central_cov)},
        {"centralized_position_covariance_eigenvalues",eigenvalues(central_cov)},
        {"centralized_schur_information",matrix2(central_info)},
        {"centralized_schur_information_eigenvalues",eigenvalues(central_info)},
        {"dekf_position_covariance",matrix2(dekf_cov)},
        {"dekf_position_covariance_eigenvalues",eigenvalues(dekf_cov)},
        {"dekf_schur_information",matrix2(dekf_info)},
        {"dekf_schur_information_eigenvalues",eigenvalues(dekf_info)},
        {"joint_mean_max_abs_difference",
            (centralized.mean-distributed.mean).cwiseAbs().maxCoeff()},
        {"joint_covariance_max_abs_difference",
            (centralized.covariance-distributed.covariance)
                .cwiseAbs().maxCoeff()}};
}

json formalAudit(const gf::CurrentReferenceAudit& value) {
    return {{"minimum_effective_reference_count",
                value.minimum_effective_reference_count},
        {"minimum_effective_reference_owner",
                value.minimum_effective_reference_owner},
        {"minimum_information_edge_count",value.minimum_information_edge_count},
        {"minimum_information_edge_owner",value.minimum_information_edge_owner},
        {"minimum_information_fim_eigenvalue",value.minimum_fim_eigenvalue},
        {"minimum_information_fim_owner",value.minimum_fim_owner},
        {"minimum_information_robust_cone_lower_bound",
                value.minimum_robust_fim_cone_lower_bound},
        {"minimum_information_robust_owner",value.minimum_robust_fim_owner},
        {"minimum_reference_only_fim_eigenvalue",
                value.minimum_reference_only_fim_eigenvalue},
        {"minimum_reference_only_fim_owner",
                value.minimum_reference_only_fim_owner},
        {"minimum_reference_only_robust_cone_lower_bound",
                value.minimum_reference_only_robust_fim_cone_lower_bound},
        {"minimum_reference_only_robust_owner",
                value.minimum_reference_only_robust_fim_owner}};
}

json snapshotAudit(const std::string& label,
    const gf::GrandFinaleRuntimeSnapshot& runtime,
    const std::vector<gf::AcceptedRangeUpdateAudit>& batch,
    const gf::JointEstimateSnapshot& centralized,
    const gf::FimGraphSemanticsThresholds& thresholds) {
    const auto fim=gf::auditFimGraphSemantics(runtime,batch,12,
        gf::task10p11rAuthorityContract().branches,thresholds);
    return {{"label",label},{"runtime_s",runtime.runtime_s},
        {"estimator_version",runtime.estimator_token},
        {"accepted_measurement_count",batch.size()},
        {"accepted_measurements",batchAudit(runtime,batch)},
        {"reference_only",edgeSet(fim.reference)},
        {"cbf2026_authority_augmented",edgeSet(fim.authority_augmented)},
        {"all_accepted_information_edges",edgeSet(fim.all_accepted)},
        {"rejected_information_edges",fim.rejected_information_edges},
        {"posterior",posteriorAudit(centralized,runtime.estimate,12)}};
}

}  // namespace

int main() {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto prior=fixture->adapter.runtimeSnapshot().estimate;
    gf::CentralizedEkfOracle centralized(
        mobileEstimates(prior),prior.fixed_positions);
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized) {
        std::cerr<<"initialization failed: "<<initialized.reason<<'\n';
        return 1;
    }
    const auto thresholds=gf::FimGraphSemanticsThresholds{
        fixture->adapter.config().reference_distance_m,
        fixture->adapter.config().maximum_range_aoi_s,
        fixture->adapter.config().minimum_range_quality,
        fixture->adapter.config().uncertainty_sigma,1.0e-6};
    auto batch=fixture->adapter.lastAcceptedRangeBatchAudit();
    const auto stage_batch=batch;
    centralized.update(measurements(batch));
    const auto stage_runtime=fixture->adapter.runtimeSnapshot();
    const auto stage_centralized=centralized.snapshot();
    const auto stage_formal=fixture->adapter.currentReferenceAudit();

    const auto step=fixture->controller.advance();
    if (!step.step.advanced) {
        std::cerr<<"first control failed: "<<step.reason<<'\n';
        return 1;
    }
    std::vector<Eigen::Vector2d> controls;
    for (const auto id:prior.mobile_ids)
        controls.push_back(step.step.applied_controls.at(id));
    centralized.propagate(controls,fixture->adapter.config().dt_s,
        fixture->adapter.config().estimator_acceleration_variance);
    batch=fixture->adapter.lastAcceptedRangeBatchAudit();
    centralized.update(measurements(batch));
    const auto first_runtime=fixture->adapter.runtimeSnapshot();
    const auto first_centralized=centralized.snapshot();
    const auto first_formal=fixture->adapter.currentReferenceAudit();

    json report={{"protocol","task10p11r_fim_graph_semantics_v1"},
        {"owner",12},
        {"cbf2026_authority_commit",gf::task10p11rAuthorityContract().source_commit},
        {"grand_finale_reference_topology_semantics","fixed_hard_control_DAG"},
        {"information_graph_semantics",
            "accepted_ranging_edges; cycles permitted; separate from reference DAG"},
        {"global_communication_graph_semantics",
            "not used as FIM edge set in this audit"},
        {"thresholds",{{"maximum_information_distance_m",
            thresholds.maximum_information_distance_m},
            {"maximum_aoi_s",thresholds.maximum_aoi_s},
            {"minimum_quality",thresholds.minimum_quality},
            {"uncertainty_sigma",thresholds.uncertainty_sigma},
            {"minimum_robust_fim",thresholds.minimum_robust_fim}}}};

    report["snapshots"]=json::array({
        snapshotAudit("stage_zero",stage_runtime,stage_batch,
            stage_centralized,thresholds),
        snapshotAudit("after_first_control_zoh_update",first_runtime,batch,
            first_centralized,thresholds)});
    report["formal_current_audit"]={{"stage_zero",formalAudit(stage_formal)},
        {"after_first_control_zoh_update",formalAudit(first_formal)}};
    report["first_control"]={{"minimum_hard_residual",
        step.step.minimum_hard_residual},
        {"minimum_plant_speed_applied_control_residual",
            step.step.minimum_plant_speed_applied_control_residual}};
    report["conclusion"]={{"reference_only_is_information_graph",false},
        {"authority_augmentation_required",true}};

    const std::string path="../docs/evidence/task10p11r/fim-graph-semantics.json";
    std::ofstream output(path);
    if (!output) {
        std::cerr<<"cannot write "<<path<<'\n';
        return 1;
    }
    output<<report.dump(2)<<'\n';
    std::cout<<report.dump(2)<<'\n';
    return 0;
}
