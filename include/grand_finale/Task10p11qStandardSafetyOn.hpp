#pragma once

#include "grand_finale/Task10p11pOperationalEnvelope.hpp"
#include "grand_finale/Task10p11qSafetyOnCoordinator.hpp"

namespace gf {

inline GrandFinaleSwarmAdapterConfig task10p11qAdapterConfig(
    SolverProfile profile,GammaFeedbackSelectionMode selection,
    std::optional<double> predictive_tau_mps2) {
    return task10p11pAdapterConfig(
        profile,30.0,selection,predictive_tau_mps2);
}

inline std::uint64_t task10p11qConfigDigest(
    const GrandFinaleSwarmAdapterConfig& config) {
    std::ostringstream stream;
    stream.precision(17);
    stream<<static_cast<int>(config.solver_profile)<<';'
          <<config.dt_s<<';'<<config.acceleration_half_box<<';'
          <<config.speed_limit_mps<<';'<<config.plant_speed_facet_count<<';'
          <<config.reference_distance_m<<';'
          <<config.add_reference_distance_m<<';'
          <<config.collision_distance_m<<';'
          <<config.uncertainty_sigma<<';'
          <<config.maximum_range_aoi_s<<';'
          <<config.minimum_range_quality<<';'
          <<config.maximum_posterior_eigenvalue_m2<<';'
          <<static_cast<int>(config.boundary.policy)<<';'
          <<config.position_gain<<';'<<config.velocity_gain<<';'
          <<config.maximum_yaw_rate_radps<<';';
    return certified_target_lifting_detail::hashText(stream.str());
}

struct Task10p11qRunMetrics {
    std::optional<double> t95_s;
    std::optional<double> t100_s;

    bool observeCoverage(double time_s,double truth_fraction) {
        if (!std::isfinite(time_s) || time_s<0.0 ||
            !std::isfinite(truth_fraction) || truth_fraction<0.0 ||
            truth_fraction>1.0+1e-12)
            throw std::invalid_argument("invalid Task 10.11q coverage sample");
        if (!t95_s.has_value() && truth_fraction>=0.95-1e-12)
            t95_s=time_s;
        if (!t100_s.has_value() && truth_fraction>=1.0-1e-12)
            t100_s=time_s;
        return t100_s.has_value();
    }
};

struct Task10p11qSafetyOnStep {
    SimpleCoverageControlStep control;
    bool topology_audit_due=false;
    bool topology_proposal_attempted=false;
    Task10p11qProposalAudit topology;
    bool fresh_successor_completed=false;
    bool stopped_at_t100=false;
    double simulated_time_s=0.0;
    std::string reason;
};

struct Task10p11qProposalBuildResult {
    bool required=false;
    std::string reason;
    NodeId owner=0;
    std::optional<DirectedEdge> removal;
    std::optional<TargetLiftProposalRequest> request;
};

inline Task10p11qProposalBuildResult task10p11qBuildProposalRequest(
    const GrandFinaleSwarmAdapter& adapter,
    const std::map<NodeId,Eigen::Vector2d>& raw_targets,
    std::uint64_t raw_ledger_version,std::uint64_t config_version) {
    Task10p11qProposalBuildResult result;
    const auto runtime=adapter.runtimeSnapshot();
    if (runtime.mode!=SupervisorMode::Search ||
        runtime.adapter_transition_pending) {
        result.reason="topology_audit_not_in_search";
        return result;
    }
    auto target_of=[&](NodeId id)->std::optional<Eigen::Vector2d> {
        const auto mobile=raw_targets.find(id);
        if (mobile!=raw_targets.end()) return mobile->second;
        const auto fixed=runtime.estimate.fixed_positions.find(id);
        if (fixed!=runtime.estimate.fixed_positions.end()) return fixed->second;
        return std::nullopt;
    };
    double worst_planned_distance=-1.0;
    for (const auto& edge:runtime.topology) {
        const auto owner=target_of(edge.owner);
        const auto reference=target_of(edge.reference);
        if (!owner.has_value() || !reference.has_value()) {
            result.reason="topology_target_missing";
            return result;
        }
        const double distance=(*owner-*reference).norm();
        if (distance>worst_planned_distance) {
            worst_planned_distance=distance;
            result.owner=edge.owner;
            result.removal=edge;
        }
    }
    const double planning_radius=adapter.config().add_reference_distance_m-1.0;

    std::map<std::string,RangeLinkState> links;
    for (const auto& [id,link]:runtime.range_links)
        links[id]={link.age_s,link.quality};
    const auto eligibility=buildEligibility(runtime.estimate,links,
        {adapter.config().add_reference_distance_m,
         adapter.config().reference_distance_m,
         adapter.config().maximum_range_aoi_s,
         adapter.config().maximum_reference_position_eigenvalue_m2,
         adapter.config().minimum_range_quality,
         adapter.config().uncertainty_sigma},{});

    TargetLiftProposalRequest request;
    request.topology.mobile_ids=runtime.estimate.mobile_ids;
    request.transition.mobile_ids=runtime.estimate.mobile_ids;
    request.geometry.mobile_ids=runtime.estimate.mobile_ids;
    request.geometry.raw_targets=raw_targets;
    request.geometry.targets=raw_targets;
    request.geometry.minimum_collision_distance_m=
        adapter.config().collision_distance_m;
    request.geometry.minimum_target_fim_planning_score=0.0;
    request.geometry.minimum_target_cone_planning_score=0.0;
    request.geometry.maximum_target_deformation_m=1.0e6;
    for (const auto& [fixed,position]:runtime.estimate.fixed_positions) {
        request.topology.fixed_ids.push_back(fixed);
        request.transition.fixed_ids.push_back(fixed);
        request.transition.fixed_targets[fixed]=position;
        request.geometry.fixed_ids.push_back(fixed);
        request.geometry.position_support_m[fixed]=0.0;
        request.geometry.targets[fixed]=position;
    }
    for (const auto owner:runtime.estimate.mobile_ids) {
        request.geometry.position_support_m[owner]=
            adapter.config().uncertainty_sigma*std::sqrt(std::max(0.0,
                detail::maximumPositionEigenvalue(runtime.estimate,owner)));
        request.geometry.posterior_valid[owner]=posteriorPositionHealthy(
            runtime.estimate,owner,
            adapter.config().maximum_posterior_eigenvalue_m2);
    }
    std::vector<DirectedEdge> information_edges=runtime.topology;
    for (const auto& candidate:eligibility.candidates)
        if (candidate.eligible) information_edges.push_back(candidate.edge);
    for (const auto& edge:information_edges) {
        const std::string range=UndirectedEdge::canonical(
            edge.owner,edge.reference).id();
        const auto link=runtime.range_links.find(range);
        request.geometry.edge_information_valid[edge.id()]=
            link!=runtime.range_links.end() &&
            link->second.age_s<=adapter.config().maximum_range_aoi_s+1e-12 &&
            link->second.quality+1e-12>=adapter.config().minimum_range_quality;
        if (link!=runtime.range_links.end())
            request.geometry.range_variances_m2[range]=link->second.variance_m2;
    }
    request.geometry.reference_edges=runtime.topology;
    const auto current_target_geometry=evaluateTargetGeometryGates(
        request.geometry);
    const bool distance_pressure=worst_planned_distance>planning_radius;
    const bool cone_pressure=!current_target_geometry.valid &&
        current_target_geometry.reason=="target_cone_planning_score" &&
        current_target_geometry.minimum_target_cone_owner!=0;
    const bool fim_pressure=!current_target_geometry.valid &&
        current_target_geometry.reason=="target_fim_planning_score" &&
        current_target_geometry.minimum_target_fim_owner!=0;
    if (!distance_pressure && !cone_pressure && !fim_pressure) {
        result.reason="topology_audit_not_required";
        return result;
    }
    if (!distance_pressure) {
        result.owner=cone_pressure
            ?current_target_geometry.minimum_target_cone_owner
            :current_target_geometry.minimum_target_fim_owner;
        result.removal.reset();
        double longest=-1.0;
        for (const auto& edge:runtime.topology) {
            if (edge.owner!=result.owner) continue;
            const auto owner=target_of(edge.owner);
            const auto reference=target_of(edge.reference);
            if (!owner.has_value() || !reference.has_value()) continue;
            const double distance=(*owner-*reference).norm();
            if (distance>longest) {
                longest=distance;
                result.removal=edge;
            }
        }
    }
    if (!result.removal.has_value()) {
        result.reason="topology_pressure_owner_has_no_removal";
        return result;
    }
    result.required=true;
    request.topology.old_edges=runtime.topology;
    for (const auto& edge:runtime.topology)
        if (!result.removal.has_value() || edge.id()!=result.removal->id()) {
            request.topology.eligible_edges.push_back(edge);
            request.topology.required_edges.push_back(edge);
        }
    for (const auto& candidate:eligibility.candidates) {
        if (!candidate.eligible || candidate.edge.owner!=result.owner ||
            std::any_of(runtime.topology.begin(),runtime.topology.end(),
                [&](const auto& edge) {
                    return edge.id()==candidate.edge.id();
                }))
            continue;
        request.topology.eligible_edges.push_back(candidate.edge);
        const auto owner=target_of(candidate.edge.owner);
        const auto reference=target_of(candidate.edge.reference);
        if (owner.has_value() && reference.has_value())
            request.topology.progress_coefficients[candidate.edge.id()]=
                planning_radius-(*owner-*reference).norm();
    }
    std::sort(request.topology.eligible_edges.begin(),
        request.topology.eligible_edges.end(),[](const auto& lhs,const auto& rhs) {
            return lhs.id()<rhs.id();
        });
    request.topology.eligible_edges.erase(std::unique(
        request.topology.eligible_edges.begin(),
        request.topology.eligible_edges.end(),[](const auto& lhs,const auto& rhs) {
            return lhs.id()==rhs.id();
        }),request.topology.eligible_edges.end());
    request.topology.min_indegree=2;
    request.topology.max_indegree=2;

    request.transition.old_edges=runtime.topology;
    request.transition.raw_targets=raw_targets;
    request.transition.add_distance_m=adapter.config().add_reference_distance_m;
    request.transition.target_margin_m=1.0;
    request.transition.topology_version=runtime.topology_token;
    request.transition.estimator_version=runtime.estimator_token;
    request.transition.raw_ledger_version=raw_ledger_version;
    request.transition.config_version=config_version;

    request.maximum_attempts=64;
    result.request=std::move(request);
    result.reason=result.request->topology.eligible_edges.size()<
            runtime.topology.size()
        ?"no_add_eligible_candidate":"topology_replacement_required";
    return result;
}

class Task10p11qSafetyOnController {
public:
    Task10p11qSafetyOnController(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter,
        Task10p11hSimpleCoverageController& coverage,
        Task10p11qSafetyOnCoordinator& topology)
        : swarm_(swarm),adapter_(adapter),coverage_(coverage),topology_(topology) {}

    Task10p11qSafetyOnStep advance() {
        Task10p11qSafetyOnStep result;
        result.topology_audit_due=control_boundaries_%10==0;
        if (topology_.activeToken()!=nullptr) {
            if (adapter_.unionControlCycles()>=1) {
                result.fresh_successor_completed=
                    topology_.finishFreshSuccessor();
                if (!result.fresh_successor_completed) {
                    result.reason="fresh_break_rejected:"+
                        adapter_.lastCertificationReason();
                    return result;
                }
            }
        }
        if (topology_.activeToken()==nullptr && result.topology_audit_due &&
            !coverage_.committedTargets().empty() &&
            adapter_.runtimeSnapshot().mode==SupervisorMode::Search) {
            std::map<NodeId,Eigen::Vector2d> raw;
            for (const auto& [owner,target]:coverage_.committedTargets())
                raw[owner]=target.center;
            const auto built=task10p11qBuildProposalRequest(
                adapter_,raw,coverage_.targetEpoch(),config_version_);
            if (built.required) {
                result.topology_proposal_attempted=true;
                if (!built.request.has_value() ||
                    built.reason=="no_add_eligible_candidate") {
                    result.reason=built.reason;
                    return result;
                }
                std::unique_ptr<TopologySolver> solver;
                if (adapter_.config().solver_profile==SolverProfile::Gurobi) {
#ifdef ENABLE_GUROBI
                    solver=std::make_unique<GurobiTopologySolver>();
#else
                    result.reason="gurobi_unavailable";
                    return result;
#endif
                } else {
                    solver=std::make_unique<HighsTopologySolver>();
                }
                result.topology=topology_.proposeAndBegin(
                    *built.request,*solver);
                if (!result.topology.transition_started) {
                    result.reason=result.topology.reason;
                    return result;
                }
            }
        }
        if (topology_.activeToken()!=nullptr) {
            result.control=coverage_.advanceWithFrozenTargetLift(
                *topology_.activeToken());
            if (result.control.step.advanced &&
                !topology_.recordUnionCycle(result.control)) {
                result.reason="union_cycle_evidence_rejected";
                return result;
            }
        } else {
            result.control=coverage_.advance();
        }
        ++control_boundaries_;
        result.simulated_time_s=swarm_.robots.front()->runtime;
        result.stopped_at_t100=
            result.control.t100_coverage_s.has_value();
        result.reason=result.control.reason;
        return result;
    }

private:
    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    Task10p11hSimpleCoverageController& coverage_;
    Task10p11qSafetyOnCoordinator& topology_;
    std::size_t control_boundaries_=0;
    std::uint64_t config_version_=1;
};

struct Task10p11qSafetyOnFixture {
    Task10p10Scenario scenario;
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11hSimpleCoverageController coverage_controller;
    Task10p11qSafetyOnCoordinator topology_coordinator;
    Task10p11qSafetyOnController controller;

    Task10p11qSafetyOnFixture(
        SolverProfile profile,GammaFeedbackSelectionMode selection,
        std::optional<double> predictive_tau_mps2)
        : scenario(task10p11pStandardCoastalScenario()),
          settings(task10p11pSwarmSettings(scenario,profile)),
          swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task10p11qAdapterConfig(
                  profile,selection,predictive_tau_mps2)),
          coverage_controller(swarm,adapter,{}, {},
              task10p11pStandardCoverageBranches()),
          topology_coordinator(swarm,adapter),
          controller(swarm,adapter,coverage_controller,topology_coordinator) {}
};

inline std::unique_ptr<Task10p11qSafetyOnFixture>
makeTask10p11qSafetyOnFixture(
    SolverProfile profile,GammaFeedbackSelectionMode selection,
    std::optional<double> predictive_tau_mps2) {
    return std::make_unique<Task10p11qSafetyOnFixture>(
        profile,selection,predictive_tau_mps2);
}

}  // namespace gf
