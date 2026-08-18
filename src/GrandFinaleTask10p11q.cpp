#include "grand_finale/Task10p11qStandardSafetyOn.hpp"

#include <chrono>
#include <iostream>

namespace {

struct TruthAudit {
    double minimum_mobile_mobile_m=std::numeric_limits<double>::infinity();
    double minimum_mobile_fixed_m=std::numeric_limits<double>::infinity();
    double maximum_speed_mps=0.0;
    std::size_t collisions=0;
    std::size_t speed_violations=0;
};

TruthAudit truthAudit(const gf::Task10p11qSafetyOnFixture& fixture) {
    TruthAudit audit;
    std::map<gf::NodeId,Eigen::Vector2d> positions;
    for (const auto& robot:fixture.swarm.robots) {
        const auto point=robot->model->xy();
        positions[robot->id]={point.x,point.y};
        const Eigen::Vector2d velocity(
            robot->model->getStateVariable("vx"),
            robot->model->getStateVariable("vy"));
        audit.maximum_speed_mps=std::max(
            audit.maximum_speed_mps,velocity.norm());
        if (velocity.norm()>30.0+1e-9) ++audit.speed_violations;
    }
    const auto& ids=fixture.scenario.mobile_ids;
    for (std::size_t first=0;first<ids.size();++first) {
        for (std::size_t second=first+1;second<ids.size();++second) {
            const double distance=(positions.at(ids[first])-
                positions.at(ids[second])).norm();
            audit.minimum_mobile_mobile_m=std::min(
                audit.minimum_mobile_mobile_m,distance);
            if (distance<10.0-1e-9) ++audit.collisions;
        }
        for (const auto& [fixed,position]:fixture.scenario.fixed_positions) {
            (void)fixed;
            const double distance=(positions.at(ids[first])-position).norm();
            audit.minimum_mobile_fixed_m=std::min(
                audit.minimum_mobile_fixed_m,distance);
            if (distance<10.0-1e-9) ++audit.collisions;
        }
    }
    return audit;
}

double minimumGamma(
    const std::map<gf::NodeId,gf::GrandFinaleGammaFeedbackDiagnostic>& values,
    double gf::GrandFinaleGammaFeedbackDiagnostic::*field) {
    double result=std::numeric_limits<double>::infinity();
    for (const auto& [owner,value]:values) {
        (void)owner;
        result=std::min(result,value.*field);
    }
    return result;
}

json optionalNumber(const std::optional<double>& value) {
    return value.has_value()?json(*value):json(nullptr);
}

json runPrefix(const std::string& label,gf::SolverProfile profile,
    gf::GammaFeedbackSelectionMode selection,
    std::optional<double> tau,std::size_t cycles) {
    auto fixture=gf::makeTask10p11qSafetyOnFixture(profile,selection,tau);
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized)
        return {{"protocol","task10p11q_safety_on_v1"},{"case",label},
            {"outcome","initialization_failed"},{"reason",initialized.reason}};

    gf::Task10p11qRunMetrics coverage;
    std::string failure_reason;
    double min_current=std::numeric_limits<double>::infinity();
    double min_nominal_predicted=std::numeric_limits<double>::infinity();
    double min_maximum_predicted=std::numeric_limits<double>::infinity();
    double min_selected_predicted=std::numeric_limits<double>::infinity();
    double min_hard_residual=std::numeric_limits<double>::infinity();
    double min_plant_residual=std::numeric_limits<double>::infinity();
    double min_reference_keep_margin=std::numeric_limits<double>::infinity();
    double min_reference_add_margin=std::numeric_limits<double>::infinity();
    double min_truth_mm=std::numeric_limits<double>::infinity();
    double min_truth_mf=std::numeric_limits<double>::infinity();
    double max_truth_speed=0.0;
    double min_robust_fim=std::numeric_limits<double>::infinity();
    double max_posterior=0.0;
    double min_aoi_margin=std::numeric_limits<double>::infinity();
    std::size_t min_effective_refs=std::numeric_limits<std::size_t>::max();
    std::size_t proposals=0,target_rejections=0,certifier_rejections=0;
    std::size_t no_goods=0,union_cycles=0,successors=0;
    std::map<std::string,std::size_t> topology_rejection_reasons;
    std::string last_topology_rejection_reason;
    std::size_t tau_owner_samples=0,tau_attained_owner_samples=0;
    std::size_t fallback_owner_samples=0,intervention_owner_samples=0;
    std::size_t exact_gamma_solves=0,canonical_rebuilds=0,qp_solves=0;
    std::optional<double> first_k1_warning_s;
    std::size_t truth_collisions=0,plant_speed_violations=0;
    gf::NodeId limiting_information_owner=0;
    std::string limiting_information_edge;
    std::string limiting_information_gate;
    const auto wall_start=std::chrono::steady_clock::now();

    for (std::size_t cycle=0;cycle<cycles;++cycle) {
        const auto step=fixture->controller.advance();
        if (step.topology_proposal_attempted) {
            ++proposals;
            target_rejections+=step.topology.target_gate_rejections;
            certifier_rejections+=step.topology.exact_certifier_rejections;
            no_goods+=step.topology.no_good_rejections;
            for (const auto& reason:step.topology.rejection_reasons) {
                ++topology_rejection_reasons[reason];
                last_topology_rejection_reason=reason;
            }
        }
        if (step.fresh_successor_completed) ++successors;
        if (!step.control.step.advanced) {
            failure_reason=step.reason.empty()?step.control.reason:step.reason;
            break;
        }
        if (step.control.step.mode==gf::SupervisorMode::Union) ++union_cycles;
        exact_gamma_solves+=step.control.step.gamma_policy_work.exact_gamma_solves;
        canonical_rebuilds+=
            step.control.step.gamma_policy_work.canonical_row_rebuilds;
        qp_solves+=step.control.step.certified_control_count;
        min_current=std::min(min_current,minimumGamma(
            step.control.step.gamma_feedback,
            &gf::GrandFinaleGammaFeedbackDiagnostic::current_gamma));
        min_nominal_predicted=std::min(min_nominal_predicted,minimumGamma(
            step.control.step.gamma_feedback,
            &gf::GrandFinaleGammaFeedbackDiagnostic::nominal_predicted_gamma));
        min_maximum_predicted=std::min(min_maximum_predicted,minimumGamma(
            step.control.step.gamma_feedback,
            &gf::GrandFinaleGammaFeedbackDiagnostic::
                maximum_margin_candidate_predicted_gamma));
        min_selected_predicted=std::min(min_selected_predicted,minimumGamma(
            step.control.step.gamma_feedback,
            &gf::GrandFinaleGammaFeedbackDiagnostic::selected_predicted_gamma));
        for (const auto& [owner,value]:step.control.step.gamma_feedback) {
            (void)owner;
            if (tau.has_value()) {
                ++tau_owner_samples;
                if (value.selected_predicted_gamma+1e-10>=*tau)
                    ++tau_attained_owner_samples;
            }
            if (!value.fallback_reason.empty()) ++fallback_owner_samples;
            if (value.intervened) ++intervention_owner_samples;
            if (!first_k1_warning_s.has_value() &&
                value.maximum_margin_candidate_predicted_gamma<0.0)
                first_k1_warning_s=step.simulated_time_s;
        }
        min_hard_residual=std::min(min_hard_residual,
            step.control.step.minimum_hard_residual);
        min_plant_residual=std::min(min_plant_residual,
            step.control.step.minimum_plant_speed_applied_control_residual);
        const auto truth=truthAudit(*fixture);
        min_truth_mm=std::min(min_truth_mm,truth.minimum_mobile_mobile_m);
        min_truth_mf=std::min(min_truth_mf,truth.minimum_mobile_fixed_m);
        max_truth_speed=std::max(max_truth_speed,truth.maximum_speed_mps);
        truth_collisions+=truth.collisions;
        plant_speed_violations+=truth.speed_violations;
        if (truth.collisions>0) { failure_reason="truth_collision"; break; }
        if (truth.speed_violations>0) {
            failure_reason="plant_speed_violation"; break;
        }
        const auto rows=fixture->adapter.currentSnapshotHardRows(
            fixture->adapter.supervisor().topology());
        for (const auto& row:rows)
            if (row.kind==gf::CanonicalHardRowKind::ReferenceDistance) {
                min_reference_keep_margin=std::min(
                    min_reference_keep_margin,row.barrier_h);
                min_reference_add_margin=std::min(
                    min_reference_add_margin,row.barrier_h-1.0);
            }
        const auto information=fixture->adapter.currentReferenceAudit();
        min_effective_refs=std::min(min_effective_refs,
            information.minimum_effective_reference_count);
        min_robust_fim=std::min(min_robust_fim,
            information.minimum_robust_fim_cone_lower_bound);
        max_posterior=std::max(max_posterior,
            information.maximum_posterior_eigenvalue);
        min_aoi_margin=std::min(min_aoi_margin,
            information.minimum_range_aoi_margin_s);
        if (information.minimum_effective_reference_count<2) {
            failure_reason="information_effective_reference_failure";
            limiting_information_gate="effective_reference_count";
            limiting_information_owner=
                information.minimum_effective_reference_owner;
            break;
        }
        if (information.minimum_robust_fim_cone_lower_bound<1e-6) {
            failure_reason="information_robust_fim_failure";
            limiting_information_gate="robust_fim_cone_lower_bound";
            limiting_information_owner=information.minimum_robust_fim_owner;
            break;
        }
        if (information.maximum_posterior_eigenvalue>
            fixture->adapter.config().maximum_posterior_eigenvalue_m2) {
            failure_reason="information_posterior_failure";
            limiting_information_gate="posterior_eigenvalue";
            limiting_information_owner=information.maximum_posterior_owner;
            break;
        }
        if (information.minimum_range_aoi_margin_s<0.0) {
            failure_reason="information_aoi_failure";
            limiting_information_gate="range_aoi";
            limiting_information_owner=information.minimum_range_aoi_owner;
            limiting_information_edge=information.minimum_range_aoi_edge;
            break;
        }
        if (coverage.observeCoverage(
                step.simulated_time_s,step.control.step.truth_coverage))
            break;
    }
    const double wall_s=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-wall_start).count();
    const bool t100=coverage.t100_s.has_value();
    const bool prefix_pass=failure_reason.empty() &&
        (t100 || fixture->swarm.robots.front()->runtime+1e-9>=
            static_cast<double>(cycles)*0.1);
    return {
        {"protocol","task10p11q_safety_on_v1"},{"case",label},
        {"solver_profile",profile==gf::SolverProfile::Gurobi?
            "gurobi":"open_source"},
        {"selection",selection==gf::GammaFeedbackSelectionMode::
            LeastIntervention?"least_intervention":"maximum_predicted_margin"},
        {"predictive_tau_mps2",tau.has_value()?json(*tau):json(nullptr)},
        {"config_digest",gf::task10p11qConfigDigest(fixture->adapter.config())},
        {"outcome",t100?"t100":prefix_pass?"prefix_pass":"failed"},
        {"reason",failure_reason},{"simulated_time_s",
            fixture->swarm.robots.front()->runtime},{"wall_time_s",wall_s},
        {"t95_true_s",optionalNumber(coverage.t95_s)},
        {"t100_true_s",optionalNumber(coverage.t100_s)},
        {"truth_coverage",fixture->adapter.coverage().truthFraction()},
        {"covered_cells",fixture->adapter.coverage().truthCoveredCount()},
        {"denominator_cells",90000},
        {"minimum_current_gamma_mps2",min_current},
        {"minimum_nominal_predicted_gamma_mps2",min_nominal_predicted},
        {"minimum_maximum_predicted_gamma_mps2",min_maximum_predicted},
        {"minimum_selected_predicted_gamma_mps2",min_selected_predicted},
        {"first_local_k1_warning_s",first_k1_warning_s.has_value()?
            json(*first_k1_warning_s):json(nullptr)},
        {"k1_scope","local_secondorder_surrogate_not_joint_gamma1"},
        {"k2_status","not_computed_no_recursive_claim"},
        {"tau_owner_samples",tau_owner_samples},
        {"tau_attained_owner_samples",tau_attained_owner_samples},
        {"tau_attainment_fraction",tau_owner_samples==0?json(nullptr):
            json(static_cast<double>(tau_attained_owner_samples)/tau_owner_samples)},
        {"fallback_owner_samples",fallback_owner_samples},
        {"intervention_owner_samples",intervention_owner_samples},
        {"minimum_reference_keep_margin_m",min_reference_keep_margin},
        {"minimum_reference_add_margin_m",min_reference_add_margin},
        {"minimum_effective_reference_count",min_effective_refs},
        {"minimum_robust_fim_cone_lower_bound",min_robust_fim},
        {"maximum_posterior_eigenvalue_m2",max_posterior},
        {"minimum_aoi_margin_s",min_aoi_margin},
        {"topology_proposals",proposals},
        {"target_gate_rejections",target_rejections},
        {"exact_certifier_rejections",certifier_rejections},
        {"request_local_no_goods",no_goods},
        {"topology_rejection_reasons",topology_rejection_reasons},
        {"last_topology_rejection_reason",last_topology_rejection_reason},
        {"limiting_information_gate",limiting_information_gate},
        {"limiting_information_owner",limiting_information_owner},
        {"limiting_information_edge",limiting_information_edge},
        {"union_control_cycles",union_cycles},
        {"fresh_successors",successors},
        {"minimum_truth_mobile_mobile_distance_m",min_truth_mm},
        {"minimum_truth_mobile_fixed_distance_m",min_truth_mf},
        {"truth_collision_events",truth_collisions},
        {"maximum_truth_speed_mps",max_truth_speed},
        {"plant_speed_violation_events",plant_speed_violations},
        {"minimum_robust_hard_residual_mps2",min_hard_residual},
        {"minimum_plant_speed_residual_mps2",min_plant_residual},
        {"exact_gamma_solves",exact_gamma_solves},
        {"canonical_row_rebuilds",canonical_rebuilds},
        {"final_qp_controls",qp_solves}
    };
}

}

int main(int argc,char** argv) {
    if (argc!=2) {
        std::cerr<<"usage: GrandFinaleTask10p11q "
            "open-short|prefix-li14|prefix-maximum|completion-li14|completion-maximum\n";
        return 64;
    }
    const std::string mode=argv[1];
    json result;
    if (mode=="open-short")
        result=runPrefix(mode,gf::SolverProfile::OpenSource,
            gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,
            std::nullopt,20);
    else if (mode=="prefix-li14")
        result=runPrefix(mode,gf::SolverProfile::Gurobi,
            gf::GammaFeedbackSelectionMode::LeastIntervention,14.0,600);
    else if (mode=="prefix-maximum")
        result=runPrefix(mode,gf::SolverProfile::Gurobi,
            gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,
            std::nullopt,600);
    else if (mode=="completion-li14")
        result=runPrefix(mode,gf::SolverProfile::Gurobi,
            gf::GammaFeedbackSelectionMode::LeastIntervention,14.0,5000);
    else if (mode=="completion-maximum")
        result=runPrefix(mode,gf::SolverProfile::Gurobi,
            gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,
            std::nullopt,5000);
    else return 64;
    std::cout<<result.dump(2)<<'\n';
    return result.at("outcome")!="failed" &&
        result.at("outcome")!="initialization_failed"?0:3;
}
