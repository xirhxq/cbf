#include "grand_finale/Task10p11qStandardSafetyOn.hpp"
#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11rFailureWindow.hpp"
#include "grand_finale/Task10p11rStopAttribution.hpp"

#include <chrono>
#include <fstream>
#include <iostream>

namespace {

json edgeJson(const gf::DirectedEdge& edge) {
    return {{"reference",edge.reference},{"owner",edge.owner},{"id",edge.id()}};
}

json vectorJson(const Eigen::Vector2d& value) {
    return json::array({value.x(),value.y()});
}

json targetLedgerJson(const std::map<gf::NodeId,Eigen::Vector2d>& values) {
    json result=json::object();
    for (const auto& [owner,value]:values)
        result[std::to_string(owner)]=vectorJson(value);
    return result;
}

std::string hardRowKindName(gf::CanonicalHardRowKind kind) {
    switch (kind) {
    case gf::CanonicalHardRowKind::ReferenceDistance: return "reference";
    case gf::CanonicalHardRowKind::Collision: return "collision";
    case gf::CanonicalHardRowKind::Workspace: return "workspace";
    case gf::CanonicalHardRowKind::SpeedLimit: return "legacy_speed_hocbf";
    case gf::CanonicalHardRowKind::PlantSpeedAppliedControl:
        return "plant_speed_applied_control";
    case gf::CanonicalHardRowKind::InputBox: return "input_box";
    case gf::CanonicalHardRowKind::Auxiliary: return "auxiliary";
    }
    return "unknown";
}

json hardPolytopeJson(const gf::HardPolytopeCertificate& certificate) {
    json rows=json::array();
    for (const auto& row:certificate.rows) rows.push_back({
        {"id",row.id},{"kind",hardRowKindName(row.kind)},
        {"responsibility_owner",row.responsibility_owner},
        {"peer",row.peer.has_value()?json(*row.peer):json(nullptr)},
        {"normal",vectorJson(row.normal)},{"constant",row.constant},
        {"h",row.barrier_h},{"psi1",row.barrier_psi1},
        {"position_reserve_m",row.position_reserve_m},
        {"velocity_reserve_mps",row.velocity_reserve_mps},
        {"coefficient_reserve_mps2",row.coefficient_reserve}});
    return {{"owner",certificate.owner},{"time_s",certificate.time_s},
        {"mode",static_cast<int>(certificate.mode)},
        {"input_half_box_mps2",certificate.input_half_box},
        {"exact_feasible",certificate.exact_feasible},
        {"minimal_conflict_row_ids",certificate.minimal_conflict_row_ids},
        {"rows",rows}};
}

const std::vector<gf::Task10p11ComputePhase>& profiledPhases() {
    static const std::vector<gf::Task10p11ComputePhase> phases{
        gf::Task10p11ComputePhase::OnlineEstimator,
        gf::Task10p11ComputePhase::GridWorldTarget,
        gf::Task10p11ComputePhase::CurrentCanonicalRowRebuild,
        gf::Task10p11ComputePhase::CurrentGamma,
        gf::Task10p11ComputePhase::EstimatorPropagation,
        gf::Task10p11ComputePhase::PredictedCanonicalRowRebuild,
        gf::Task10p11ComputePhase::PredictedGamma,
        gf::Task10p11ComputePhase::FinalQp,
        gf::Task10p11ComputePhase::RobustQpSolve,
        gf::Task10p11ComputePhase::ResidualTokenAudit,
        gf::Task10p11ComputePhase::InformationAudit,
        gf::Task10p11ComputePhase::TruthOnlyAudit,
        gf::Task10p11ComputePhase::PlantPreflightZoh};
    return phases;
}

json computeProfileJson(const gf::Task10p11ComputeProfile& profile) {
    json output=json::object();
    for (const auto phase:profiledPhases()) {
        const auto summary=profile.summary(phase);
        output[gf::task10p11ComputePhaseName(phase)]={{"calls",summary.calls},
            {"total_s",summary.total_s},{"median_s",summary.median_s},
            {"p95_s",summary.p95_s},{"maximum_s",summary.maximum_s},
            {"cold_calls",summary.cold_calls},
            {"steady_calls",summary.steady_calls}};
    }
    return output;
}

json referenceAttributionJson(const gf::ReferenceOwnerAttribution& value,
    double time_s) {
    json edges=json::array();
    for (const auto& edge:value.edges) edges.push_back({
        {"edge",edgeJson(edge.edge)},{"distance_m",edge.distance_m},
        {"range_variance_m2",edge.range_variance_m2},
        {"position_support_m",edge.position_support_m},
        {"aoi_s",edge.aoi_s},{"quality",edge.quality}});
    json effective=json::array();
    for (const auto& edge:value.effective_edges)
        effective.push_back(edgeJson(edge));
    return {{"time_s",time_s},{"valid",value.valid},{"reason",value.reason},
        {"owner",value.owner},{"effective_edges",effective},
        {"nominal_fim_eigenvalue",value.nominal_fim_eigenvalue},
        {"posterior_fim_proxy_eigenvalue",
            value.posterior_fim_proxy_eigenvalue},
        {"robust_cone_fim_lower_bound",
            value.robust_cone_fim_lower_bound},
        {"reference_angle_rad",value.reference_angle_rad},
        {"owner_position_covariance",json::array({
            value.owner_position_covariance(0,0),
            value.owner_position_covariance(0,1),
            value.owner_position_covariance(1,0),
            value.owner_position_covariance(1,1)})},
        {"edges",edges}};
}

json candidateJson(const gf::ReplacementCandidateAttribution& value) {
    return {{"remove",edgeJson(value.removal)},
        {"add",edgeJson(value.addition)},
        {"dag_valid",value.dag_valid},
        {"keep_distance_valid",value.keep_distance_valid},
        {"add_distance_valid",value.add_distance_valid},
        {"aoi_valid",value.aoi_valid},{"quality_valid",value.quality_valid},
        {"posterior_valid",value.posterior_valid},
        {"fim_valid",value.fim_valid},
        {"reference_lens_valid",value.reference_lens_valid},
        {"nominal_fim_eigenvalue",value.nominal_fim_eigenvalue},
        {"posterior_fim_proxy_eigenvalue",
            value.posterior_fim_proxy_eigenvalue},
        {"robust_cone_fim_lower_bound",value.robust_cone_fim_lower_bound},
        {"target_fim_planning_score",value.target_fim_planning_score},
        {"target_cone_planning_score",value.target_cone_planning_score},
        {"raw_targets",targetLedgerJson(value.raw_targets)},
        {"lifted_targets",targetLedgerJson(value.lifted_targets)},
        {"first_rejection_reason",value.exact_rejection_reason}};
}

json exactAlternatives(gf::GrandFinaleSwarmAdapter& adapter,gf::NodeId owner) {
    const auto runtime=adapter.runtimeSnapshot();
    std::set<gf::NodeId> current;
    std::vector<gf::DirectedEdge> old_edges;
    for (const auto& edge:runtime.topology) if (edge.owner==owner) {
        current.insert(edge.reference);
        old_edges.push_back(edge);
    }
    std::map<std::string,gf::RangeLinkState> links;
    for (const auto& [id,link]:runtime.range_links)
        links[id]={link.age_s,link.quality};
    const auto eligibility=gf::buildEligibility(runtime.estimate,links,
        {adapter.config().add_reference_distance_m,
         adapter.config().reference_distance_m,
         adapter.config().maximum_range_aoi_s,
         adapter.config().maximum_reference_position_eigenvalue_m2,
         adapter.config().minimum_range_quality,
         adapter.config().uncertainty_sigma},{});
    json result=json::array();
    for (const auto& candidate:eligibility.candidates) {
        if (candidate.edge.owner!=owner || !candidate.eligible ||
            current.count(candidate.edge.reference)) continue;
        for (const auto& removal:old_edges) {
            const auto certificate=adapter.auditReplacement(
                candidate.edge,removal);
            result.push_back({{"remove",edgeJson(removal)},
                {"add",edgeJson(candidate.edge)},
                {"add_eligible",candidate.eligible},
                {"robust_distance_m",candidate.robust_distance_m},
                {"exact_certificate_valid",certificate.valid},
                {"exact_certificate_reason",certificate.reason},
                {"reverse_valid",certificate.reverse_valid}});
        }
    }
    return result;
}

json runAttribution(gf::GammaFeedbackSelectionMode selection,
    std::optional<double> tau,std::size_t cycles,const std::string& label) {
    auto fixture=gf::makeTask10p11qSafetyOnFixture(
        gf::SolverProfile::Gurobi,selection,tau);
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized)
        return {{"protocol","task10p11r_attribution_v1"},{"case",label},
            {"outcome","initialization_failed"},{"reason",initialized.reason}};
    json owner8_series=json::array();
    json candidate_series=json::array();
    std::optional<double> first_decrease;
    std::optional<double> first_zero;
    std::optional<double> first_hard_threshold;
    double previous=std::numeric_limits<double>::quiet_NaN();
    std::string failure;
    for (std::size_t cycle=0;cycle<cycles;++cycle) {
        const auto step=fixture->controller.advance();
        for (const auto& candidate:step.topology.candidate_attributions)
            candidate_series.push_back(candidateJson(candidate));
        if (!step.control.step.advanced) {
            failure=step.reason.empty()?step.control.reason:step.reason;
            break;
        }
        const auto value=gf::attributeReferenceOwner(
            fixture->adapter.runtimeSnapshot(),8,
            fixture->adapter.config().uncertainty_sigma);
        owner8_series.push_back(referenceAttributionJson(
            value,step.simulated_time_s));
        if (value.valid) {
            if (std::isfinite(previous) && !first_decrease.has_value() &&
                value.robust_cone_fim_lower_bound<previous-1e-12)
                first_decrease=step.simulated_time_s;
            if (!first_zero.has_value() &&
                value.robust_cone_fim_lower_bound<0.0)
                first_zero=step.simulated_time_s;
            if (!first_hard_threshold.has_value() &&
                value.robust_cone_fim_lower_bound<1e-6)
                first_hard_threshold=step.simulated_time_s;
            previous=value.robust_cone_fim_lower_bound;
        }
        const auto information=fixture->adapter.currentReferenceAudit();
        if (information.minimum_robust_fim_cone_lower_bound<1e-6) {
            failure="information_robust_fim_failure";
            break;
        }
    }
    const auto final_owner=gf::attributeReferenceOwner(
        fixture->adapter.runtimeSnapshot(),8,
        fixture->adapter.config().uncertainty_sigma);
    return {{"protocol","task10p11r_attribution_v1"},{"case",label},
        {"source_task","task10p11q_round3"},
        {"outcome",failure.empty()?"bounded_replay_complete":"failed"},
        {"reason",failure},{"simulated_time_s",fixture->swarm.robots.front()->runtime},
        {"truth_coverage",fixture->adapter.coverage().truthFraction()},
        {"first_robust_fim_decrease_s",first_decrease.has_value()
            ?json(*first_decrease):json(nullptr)},
        {"first_robust_fim_zero_crossing_s",first_zero.has_value()
            ?json(*first_zero):json(nullptr)},
        {"first_robust_fim_hard_threshold_crossing_s",
            first_hard_threshold.has_value()?json(*first_hard_threshold):json(nullptr)},
        {"owner8_series",owner8_series},
        {"final_owner8",referenceAttributionJson(
            final_owner,fixture->swarm.robots.front()->runtime)},
        {"exact_alternative_edge_audit",exactAlternatives(fixture->adapter,8)},
        {"candidate_rejection_ledger",candidate_series}};
}

struct TruthMargins {
    double mm=std::numeric_limits<double>::infinity();
    double mf=std::numeric_limits<double>::infinity();
    double speed_margin=std::numeric_limits<double>::infinity();
};

TruthMargins truthMargins(const gf::Task10p11rFixedBaselineFixture& fixture) {
    TruthMargins result;
    std::map<gf::NodeId,Eigen::Vector2d> positions;
    for (const auto& robot:fixture.swarm.robots) {
        const Point point=robot->model->xy();
        positions[robot->id]={point.x,point.y};
        const Eigen::Vector2d velocity(
            robot->model->getStateVariable("vx"),
            robot->model->getStateVariable("vy"));
        result.speed_margin=std::min(result.speed_margin,30.0-velocity.norm());
    }
    for (std::size_t i=0;i<fixture.scenario.mobile_ids.size();++i) {
        const auto a=fixture.scenario.mobile_ids[i];
        for (std::size_t j=i+1;j<fixture.scenario.mobile_ids.size();++j)
            result.mm=std::min(result.mm,
                (positions.at(a)-positions.at(fixture.scenario.mobile_ids[j])).norm());
        for (const auto& [fixed,p]:fixture.scenario.fixed_positions) {
            (void)fixed;
            result.mf=std::min(result.mf,(positions.at(a)-p).norm());
        }
    }
    return result;
}

double minimumGamma(const gf::GrandFinaleSwarmStep& step,
    double gf::GrandFinaleGammaFeedbackDiagnostic::*field) {
    double value=std::numeric_limits<double>::infinity();
    for (const auto& [owner,gamma]:step.gamma_feedback) {
        (void)owner;
        value=std::min(value,gamma.*field);
    }
    return value;
}

json runFixed(std::size_t cycles,const std::string& label) {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized)
        return {{"protocol","task10p11r_fixed_v1"},{"case",label},
            {"outcome","initialization_failed"},{"reason",initialized.reason}};
    const auto topology=fixture->adapter.runtimeSnapshot().topology;
    std::string topology_canonical;
    for (const auto& edge:gf::canonicalTargetEdges(topology))
        topology_canonical+=edge.id()+";";
    std::string failure;
    std::optional<double> t95,t100;
    double min_current=std::numeric_limits<double>::infinity();
    double min_nominal=std::numeric_limits<double>::infinity();
    double min_maximum=std::numeric_limits<double>::infinity();
    double min_selected=std::numeric_limits<double>::infinity();
    double min_hard=std::numeric_limits<double>::infinity();
    double min_plant=std::numeric_limits<double>::infinity();
    double min_information_nominal_fim=std::numeric_limits<double>::infinity();
    gf::NodeId min_information_nominal_fim_owner=0;
    double min_information_robust_fim=std::numeric_limits<double>::infinity();
    gf::NodeId min_information_robust_fim_owner=0;
    double min_reference_fim_proxy=std::numeric_limits<double>::infinity();
    gf::NodeId min_reference_fim_proxy_owner=0;
    double min_reference_robust_fim_proxy=
        std::numeric_limits<double>::infinity();
    gf::NodeId min_reference_robust_fim_proxy_owner=0;
    double max_posterior=0.0;
    double min_aoi=std::numeric_limits<double>::infinity();
    std::size_t min_effective_references=std::numeric_limits<std::size_t>::max();
    std::size_t min_information_edges=std::numeric_limits<std::size_t>::max();
    double min_collision_h=std::numeric_limits<double>::infinity();
    double min_collision_psi1=std::numeric_limits<double>::infinity();
    double min_collision_residual=std::numeric_limits<double>::infinity();
    double min_reference_h=std::numeric_limits<double>::infinity();
    double min_reference_psi1=std::numeric_limits<double>::infinity();
    double min_reference_residual=std::numeric_limits<double>::infinity();
    double min_mm=std::numeric_limits<double>::infinity();
    double min_mf=std::numeric_limits<double>::infinity();
    double min_speed_margin=std::numeric_limits<double>::infinity();
    std::size_t tau_samples=0,tau_attained=0,fallbacks=0;
    std::size_t stagnant=0;
    int previous_covered=fixture->adapter.coverage().truthCoveredCount();
    const auto wall_start=std::chrono::steady_clock::now();
    for (std::size_t cycle=0;cycle<cycles;++cycle) {
        if (!fixture->topologyFrozen()) { failure="fixed_topology_mutated"; break; }
        const auto step=fixture->controller.advance();
        if (!step.step.advanced) {
            failure=step.reason.empty()?step.step.reason:step.reason;
            break;
        }
        if (!fixture->topologyFrozen()) { failure="fixed_topology_mutated"; break; }
        min_current=std::min(min_current,minimumGamma(step.step,
            &gf::GrandFinaleGammaFeedbackDiagnostic::current_gamma));
        min_nominal=std::min(min_nominal,minimumGamma(step.step,
            &gf::GrandFinaleGammaFeedbackDiagnostic::nominal_predicted_gamma));
        min_maximum=std::min(min_maximum,minimumGamma(step.step,
            &gf::GrandFinaleGammaFeedbackDiagnostic::maximum_margin_candidate_predicted_gamma));
        min_selected=std::min(min_selected,minimumGamma(step.step,
            &gf::GrandFinaleGammaFeedbackDiagnostic::selected_predicted_gamma));
        for (const auto& [owner,gamma]:step.step.gamma_feedback) {
            (void)owner;
            ++tau_samples;
            if (gamma.selected_predicted_gamma+1e-10>=14.0) ++tau_attained;
            if (!gamma.fallback_reason.empty()) ++fallbacks;
        }
        min_hard=std::min(min_hard,step.step.minimum_hard_residual);
        min_plant=std::min(min_plant,
            step.step.minimum_plant_speed_applied_control_residual);
        const auto info=fixture->adapter.currentReferenceAudit();
        if (info.minimum_fim_eigenvalue<min_information_nominal_fim) {
            min_information_nominal_fim=info.minimum_fim_eigenvalue;
            min_information_nominal_fim_owner=info.minimum_fim_owner;
        }
        if (info.minimum_robust_fim_cone_lower_bound<
            min_information_robust_fim) {
            min_information_robust_fim=
                info.minimum_robust_fim_cone_lower_bound;
            min_information_robust_fim_owner=info.minimum_robust_fim_owner;
        }
        if (info.minimum_reference_only_fim_eigenvalue<
            min_reference_fim_proxy) {
            min_reference_fim_proxy=
                info.minimum_reference_only_fim_eigenvalue;
            min_reference_fim_proxy_owner=
                info.minimum_reference_only_fim_owner;
        }
        if (info.minimum_reference_only_robust_fim_cone_lower_bound<
            min_reference_robust_fim_proxy) {
            min_reference_robust_fim_proxy=
                info.minimum_reference_only_robust_fim_cone_lower_bound;
            min_reference_robust_fim_proxy_owner=
                info.minimum_reference_only_robust_fim_owner;
        }
        max_posterior=std::max(max_posterior,info.maximum_posterior_eigenvalue);
        min_aoi=std::min(min_aoi,info.minimum_range_aoi_margin_s);
        min_effective_references=std::min(min_effective_references,
            info.minimum_effective_reference_count);
        min_information_edges=std::min(min_information_edges,
            info.minimum_information_edge_count);
        min_collision_h=std::min(min_collision_h,step.step.minimum_collision_h);
        min_collision_psi1=std::min(min_collision_psi1,
            step.step.minimum_collision_psi1);
        min_collision_residual=std::min(min_collision_residual,
            step.step.minimum_collision_residual);
        min_reference_h=std::min(min_reference_h,step.step.minimum_reference_h);
        min_reference_psi1=std::min(min_reference_psi1,
            step.step.minimum_reference_psi1);
        min_reference_residual=std::min(min_reference_residual,
            step.step.minimum_reference_residual);
        if (info.minimum_effective_reference_count<2)
            failure="information_effective_reference_failure";
        else if (info.minimum_robust_fim_cone_lower_bound<1e-6)
            failure="information_robust_fim_failure";
        else if (info.maximum_posterior_eigenvalue>
            fixture->adapter.config().maximum_posterior_eigenvalue_m2)
            failure="information_posterior_failure";
        else if (info.minimum_range_aoi_margin_s<0.0)
            failure="information_aoi_failure";
        const auto truth=truthMargins(*fixture);
        min_mm=std::min(min_mm,truth.mm);
        min_mf=std::min(min_mf,truth.mf);
        min_speed_margin=std::min(min_speed_margin,truth.speed_margin);
        if (truth.mm<10.0-1e-9 || truth.mf<10.0-1e-9)
            failure="truth_collision";
        if (truth.speed_margin<-1e-9) failure="plant_speed_violation";
        if (!failure.empty()) break;
        const double coverage=fixture->adapter.coverage().truthFraction();
        const double time=fixture->swarm.robots.front()->runtime;
        if (!t95.has_value() && coverage>=0.95-1e-12) t95=time;
        if (!t100.has_value() && coverage>=1.0-1e-12) { t100=time; break; }
        const int covered=fixture->adapter.coverage().truthCoveredCount();
        stagnant=covered==previous_covered?stagnant+1:0;
        previous_covered=covered;
        if (stagnant>=1000 && coverage<1.0-1e-12) {
            failure="coverage_stagnation_100s";
            break;
        }
    }
    const double wall=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-wall_start).count();
    json topology_json=json::array();
    for (const auto& edge:topology) topology_json.push_back(edgeJson(edge));
    std::map<gf::NodeId,Eigen::Vector2d> targets;
    for (const auto& [owner,target]:fixture->controller.committedTargets())
        targets[owner]=target.center;
    const auto limiting_reference=gf::attributeReferenceOwner(
        fixture->adapter.runtimeSnapshot(),min_reference_fim_proxy_owner,
        fixture->adapter.config().uncertainty_sigma);
    return {{"protocol","task10p11r_fixed_v1"},{"case",label},
        {"authority_commit",gf::task10p11rAuthorityContract().source_commit},
        {"outcome",t100.has_value()?"t100":failure.empty()?"bounded_pass":"failed"},
        {"reason",failure},{"simulated_time_s",fixture->swarm.robots.front()->runtime},
        {"wall_time_s",wall},{"truth_coverage",fixture->adapter.coverage().truthFraction()},
        {"covered_cells",fixture->adapter.coverage().truthCoveredCount()},
        {"denominator_cells",90000},{"t95_true_s",t95.has_value()?json(*t95):json(nullptr)},
        {"t100_true_s",t100.has_value()?json(*t100):json(nullptr)},
        {"minimum_current_gamma_mps2",min_current},
        {"minimum_nominal_predicted_gamma_mps2",min_nominal},
        {"minimum_maximum_predicted_gamma_mps2",min_maximum},
        {"minimum_selected_predicted_gamma_mps2",min_selected},
        {"tau14_attainment_fraction",tau_samples==0?json(nullptr):
            json(static_cast<double>(tau_attained)/tau_samples)},
        {"fallback_owner_samples",fallbacks},
        {"minimum_accepted_information_nominal_fim_eigenvalue",
            min_information_nominal_fim},
        {"minimum_accepted_information_nominal_fim_owner",
            min_information_nominal_fim_owner},
        {"minimum_accepted_information_robust_fim_cone_lower_bound",
            min_information_robust_fim},
        {"minimum_accepted_information_robust_fim_owner",
            min_information_robust_fim_owner},
        {"minimum_reference_topology_fim_proxy_eigenvalue",
            min_reference_fim_proxy},
        {"minimum_reference_topology_fim_proxy_owner",
            min_reference_fim_proxy_owner},
        {"minimum_reference_topology_robust_fim_proxy",
            min_reference_robust_fim_proxy},
        {"minimum_reference_topology_robust_fim_proxy_owner",
            min_reference_robust_fim_proxy_owner},
        {"minimum_effective_reference_count",min_effective_references},
        {"minimum_accepted_information_edge_count",min_information_edges},
        {"limiting_reference_topology_attribution",referenceAttributionJson(
            limiting_reference,fixture->swarm.robots.front()->runtime)},
        {"maximum_posterior_eigenvalue_m2",max_posterior},
        {"minimum_aoi_margin_s",min_aoi},
        {"minimum_collision_h_m",min_collision_h},
        {"minimum_collision_psi1_mps",min_collision_psi1},
        {"minimum_collision_residual_mps2",min_collision_residual},
        {"minimum_reference_h_m",min_reference_h},
        {"minimum_reference_psi1_mps",min_reference_psi1},
        {"minimum_reference_residual_mps2",min_reference_residual},
        {"minimum_robust_hard_residual_mps2",min_hard},
        {"minimum_plant_speed_residual_mps2",min_plant},
        {"minimum_truth_mobile_mobile_distance_m",min_mm},
        {"minimum_truth_mobile_fixed_distance_m",min_mf},
        {"minimum_truth_speed_margin_mps",min_speed_margin},
        {"fixed_topology",topology_json},
        {"topology_canonical",topology_canonical},
        {"topology_frozen",fixture->topologyFrozen()},
        {"target_ledger",targetLedgerJson(targets)},
        {"target_epoch",fixture->controller.targetEpoch()},
        {"dynamic_topology_components_enabled",false}};
}

json runFailureWindow() {
    constexpr double window_start_s=131.0;
    constexpr std::size_t maximum_cycles=1330;
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized)
        return {{"protocol","task10p11r_failure_window_v1"},
            {"outcome","initialization_failed"},{"reason",initialized.reason}};
    gf::Task10p11ComputeProfile profile;
    json timeline=json::array();
    std::string failure;
    std::optional<double> failure_time;
    gf::NodeId failure_owner=0;
    double failure_gamma=std::numeric_limits<double>::infinity();
    std::vector<std::string> failure_conflict;
    for (std::size_t cycle=0;cycle<maximum_cycles;++cycle) {
        const auto before=fixture->adapter.runtimeSnapshot();
        const double time_s=before.runtime_s;
        const auto rows=fixture->adapter.currentSnapshotHardRows(before.topology);
        const std::size_t epoch_before=fixture->controller.targetEpoch();
        const auto control=fixture->controller.advance();
        profile.merge(control.compute_profile);

        const auto information_started=std::chrono::steady_clock::now();
        const auto information=fixture->adapter.currentReferenceAudit();
        profile.record(gf::Task10p11ComputePhase::InformationAudit,
            std::chrono::duration<double>(std::chrono::steady_clock::now()-
                information_started).count(),true);
        const auto truth_started=std::chrono::steady_clock::now();
        const auto truth=truthMargins(*fixture);
        profile.record(gf::Task10p11ComputePhase::TruthOnlyAudit,
            std::chrono::duration<double>(std::chrono::steady_clock::now()-
                truth_started).count(),true);

        if (time_s+1e-12>=window_start_s || !control.step.advanced) {
            const auto serialization_started=std::chrono::steady_clock::now();
            const auto audit=gf::auditTask10p11rFailureSnapshot(
                before.estimate.mobile_ids,time_s,before.mode,before.topology,
                rows,fixture->adapter.config().acceleration_half_box);
            json gamma={{"current_mps2",audit.current_gamma},
                {"nominal_predicted_mps2",nullptr},
                {"maximum_predicted_mps2",nullptr},
                {"selected_predicted_mps2",nullptr},
                {"tau14_attained",false},
                {"intervened",false},
                {"fallback_reason",control.step.advanced
                    ?"missing_owner_diagnostic":"current_gamma_negative_before_prediction"},
                {"dominant_row",audit.dominant_row}};
            if (control.step.advanced) {
                const auto found=control.step.gamma_feedback.find(
                    audit.limiting_owner);
                if (found!=control.step.gamma_feedback.end()) {
                    const auto& value=found->second;
                    gamma={{"current_mps2",value.current_gamma},
                        {"nominal_predicted_mps2",value.nominal_predicted_gamma},
                        {"maximum_predicted_mps2",
                            value.maximum_margin_candidate_predicted_gamma},
                        {"selected_predicted_mps2",value.selected_predicted_gamma},
                        {"tau14_attained",value.selected_predicted_gamma+1e-10>=14.0},
                        {"intervened",value.intervened},
                        {"fallback_reason",value.fallback_reason},
                        {"dominant_row",value.dominant_row}};
                }
            }
            std::map<gf::NodeId,Eigen::Vector2d> targets;
            for (const auto& [owner,target]:fixture->controller.committedTargets())
                targets[owner]=target.center;
            timeline.push_back({{"cycle",cycle},{"time_s",time_s},
                {"advanced",control.step.advanced},{"reason",control.reason},
                {"limiting_owner",audit.limiting_owner},
                {"gamma",gamma},{"target_epoch_before",epoch_before},
                {"target_epoch_after",fixture->controller.targetEpoch()},
                {"target_ledger",targetLedgerJson(targets)},
                {"raw_target_equals_committed",true},
                {"lifted_target_enabled",false},
                {"hard_polytope",hardPolytopeJson(audit.hard_polytope)},
                {"accepted_information_robust_fim",
                    information.minimum_robust_fim_cone_lower_bound},
                {"truth_minimum_mobile_mobile_distance_m",truth.mm},
                {"truth_minimum_mobile_fixed_distance_m",truth.mf},
                {"truth_speed_margin_mps",truth.speed_margin}});
            profile.record(gf::Task10p11ComputePhase::DiagnosticSerialization,
                std::chrono::duration<double>(std::chrono::steady_clock::now()-
                    serialization_started).count(),true);
        }
        if (!control.step.advanced) {
            failure=control.reason.empty()?control.step.reason:control.reason;
            failure_time=time_s;
            const auto audit=gf::auditTask10p11rFailureSnapshot(
                before.estimate.mobile_ids,time_s,before.mode,before.topology,
                rows,fixture->adapter.config().acceleration_half_box);
            failure_owner=audit.limiting_owner;
            failure_gamma=audit.current_gamma;
            failure_conflict=audit.hard_polytope.minimal_conflict_row_ids;
            break;
        }
    }
    return {{"protocol","task10p11r_failure_window_v1"},
        {"control_dt_s",0.1},{"window_start_s",window_start_s},
        {"outcome",failure.empty()?"bounded_replay_complete":"failed"},
        {"reason",failure},{"failure_time_s",failure_time.has_value()
            ?json(*failure_time):json(nullptr)},
        {"failure_owner",failure_owner},{"failure_current_gamma_mps2",failure_gamma},
        {"failure_minimal_conflict_row_ids",failure_conflict},
        {"truth_coverage",fixture->adapter.coverage().truthFraction()},
        {"target_epoch",fixture->controller.targetEpoch()},
        {"timeline",timeline},{"phase_profile",computeProfileJson(profile)},
        {"claim_boundary","diagnostic replay; dt unchanged; no failed control applied"}};
}

}

int main(int argc,char** argv) {
    if (argc!=2 && argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11r "
                 <<"attribute-maximum|attribute-li14|stage-zero|prefix|completion|failure-window\n";
        return 2;
    }
    std::optional<std::ofstream> output_stream;
    if (argc==3) {
        output_stream.emplace(argv[2]);
        if (!*output_stream) {
            std::cerr<<"output_preflight_failed:"<<argv[2]<<'\n';
            return 3;
        }
    }
    const std::string mode=argv[1];
    json output;
    if (mode=="attribute-maximum") output=runAttribution(
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,
        std::nullopt,60,"maximum_round3_attribution");
    else if (mode=="attribute-li14") output=runAttribution(
        gf::GammaFeedbackSelectionMode::LeastIntervention,
        14.0,181,"li14_round3_attribution");
    else if (mode=="stage-zero") output=runFixed(1,"fixed_stage_zero");
    else if (mode=="prefix") output=runFixed(600,"fixed_prefix_60s");
    else if (mode=="completion") output=runFixed(5000,"fixed_completion_500s");
    else if (mode=="failure-window") output=runFailureWindow();
    else return 2;
    if (output_stream.has_value()) {
        *output_stream<<output.dump(2)<<'\n';
        if (!*output_stream) return 3;
    } else {
        std::cout<<output.dump(2)<<'\n';
    }
    return 0;
}
