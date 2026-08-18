#include "grand_finale/Task10p11pOperationalEnvelope.hpp"

#include <chrono>
#include <iostream>

namespace {

struct TruthAudit {
    double minimum_mobile_mobile_m=std::numeric_limits<double>::infinity();
    double minimum_mobile_fixed_m=std::numeric_limits<double>::infinity();
    double maximum_speed_mps=0.0;
    std::size_t collision_violating_pairs=0;
    std::size_t speed_violating_owners=0;
    std::size_t reference_violating_edges=0;
    double maximum_reference_distance_m=0.0;
};

TruthAudit auditTruth(
    const gf::Task10p11pNominalFixture& fixture,double speed_limit_mps) {
    TruthAudit result;
    std::map<gf::NodeId,Eigen::Vector2d> positions;
    for (const auto& robot:fixture.swarm.robots) {
        const Point point=robot->model->xy();
        positions[static_cast<gf::NodeId>(robot->id)]={point.x,point.y};
        const Eigen::Vector2d velocity(
            robot->model->getStateVariable("vx"),
            robot->model->getStateVariable("vy"));
        result.maximum_speed_mps=std::max(
            result.maximum_speed_mps,velocity.norm());
        if (velocity.norm()>speed_limit_mps+1e-9)
            ++result.speed_violating_owners;
    }
    const auto ids=fixture.scenario.mobile_ids;
    for (std::size_t first=0;first<ids.size();++first) {
        for (std::size_t second=first+1;second<ids.size();++second) {
            const double distance=(positions.at(ids[first])-
                                   positions.at(ids[second])).norm();
            result.minimum_mobile_mobile_m=std::min(
                result.minimum_mobile_mobile_m,distance);
            if (distance<10.0) ++result.collision_violating_pairs;
        }
        for (const auto& [fixed,position]:fixture.scenario.fixed_positions) {
            (void)fixed;
            const double distance=(positions.at(ids[first])-position).norm();
            result.minimum_mobile_fixed_m=std::min(
                result.minimum_mobile_fixed_m,distance);
            if (distance<10.0) ++result.collision_violating_pairs;
        }
    }
    for (const auto& edge:fixture.adapter.supervisor().topology()) {
        const Eigen::Vector2d reference=edge.reference>=100
            ?fixture.scenario.fixed_positions.at(edge.reference)
            :positions.at(edge.reference);
        const double distance=(positions.at(edge.owner)-reference).norm();
        result.maximum_reference_distance_m=std::max(
            result.maximum_reference_distance_m,distance);
        if (distance>850.0) ++result.reference_violating_edges;
    }
    return result;
}

double finiteMinimum(const std::map<gf::NodeId,double>& values) {
    double result=std::numeric_limits<double>::infinity();
    for (const auto& [id,value]:values) {
        (void)id;
        if (std::isfinite(value)) result=std::min(result,value);
    }
    return std::isfinite(result)
        ?result:std::numeric_limits<double>::quiet_NaN();
}

json optionalNumber(const std::optional<double>& value) {
    return value.has_value()?json(*value):json(nullptr);
}

int runNominal() {
    constexpr double speed_limit_mps=30.0;
    constexpr std::size_t maximum_cycles=5000;
    auto fixture=gf::makeTask10p11pNominalFixture(
        gf::SolverProfile::Gurobi,speed_limit_mps);
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized) {
        std::cout<<json{{"outcome","initialization_failed"},
                        {"reason",initialized.reason}}.dump(2)<<'\n';
        return 2;
    }
    gf::Task10p11pNominalMetrics metrics;
    std::size_t collision_pair_ticks=0;
    std::size_t reference_edge_ticks=0;
    std::size_t speed_owner_ticks=0;
    std::size_t information_failures=0;
    std::size_t information_hard_gate_failure_ticks=0;
    double first_information_hard_gate_failure_s=
        std::numeric_limits<double>::quiet_NaN();
    double minimum_robust_fim=std::numeric_limits<double>::infinity();
    double maximum_posterior=0.0;
    double minimum_aoi_margin=std::numeric_limits<double>::infinity();
    double maximum_reference_distance=0.0;
    double last_progress_time=0.0;
    std::size_t previous_covered=fixture->adapter.coverage().truthCoveredCount();
    std::string failure_reason;
    json failure_certificate=nullptr;
    const auto wall_start=std::chrono::steady_clock::now();
    for (std::size_t cycle=0;cycle<maximum_cycles;++cycle) {
        const auto step=fixture->controller.advance();
        if (!step.diagnostic.advanced) {
            failure_reason=step.diagnostic.reason;
            try {
                const auto rows=fixture->adapter.currentSnapshotHardRows(
                    fixture->adapter.supervisor().topology());
                const gf::CanonicalHardRow* minimum_speed=nullptr;
                for (const auto& row:rows)
                    if (row.kind==gf::CanonicalHardRowKind::SpeedLimit &&
                        (minimum_speed==nullptr ||
                         row.barrier_h<minimum_speed->barrier_h))
                        minimum_speed=&row;
                if (minimum_speed!=nullptr) {
                    const auto runtime=fixture->adapter.runtimeSnapshot();
                    const auto found=std::find(
                        runtime.estimate.mobile_ids.begin(),
                        runtime.estimate.mobile_ids.end(),minimum_speed->owner);
                    const auto index=static_cast<std::size_t>(std::distance(
                        runtime.estimate.mobile_ids.begin(),found));
                    const Eigen::Vector2d estimated_velocity=
                        runtime.estimate.mean.segment<2>(4*index+2);
                    const auto robot=std::find_if(
                        fixture->swarm.robots.begin(),fixture->swarm.robots.end(),
                        [&](const auto& value) {
                            return value->id==minimum_speed->owner;
                        });
                    const Eigen::Vector2d truth_velocity(
                        (*robot)->model->getStateVariable("vx"),
                        (*robot)->model->getStateVariable("vy"));
                    failure_certificate={
                        {"row_id",minimum_speed->id},
                        {"owner",minimum_speed->owner},
                        {"barrier_h",minimum_speed->barrier_h},
                        {"constant",minimum_speed->constant},
                        {"coefficient",{minimum_speed->control_coefficient.x(),
                                        minimum_speed->control_coefficient.y()}},
                        {"velocity_tube_radius_mps",
                            minimum_speed->velocity_uncertainty_reserve_mps},
                        {"estimated_velocity_mps",{estimated_velocity.x(),
                                                    estimated_velocity.y()}},
                        {"estimated_speed_mps",estimated_velocity.norm()},
                        {"robust_speed_mps",estimated_velocity.norm()+
                            minimum_speed->velocity_uncertainty_reserve_mps},
                        {"truth_velocity_mps",{truth_velocity.x(),
                                                truth_velocity.y()}},
                        {"truth_speed_mps",truth_velocity.norm()},
                        {"current_gamma_mps2",
                            step.diagnostic.current_gamma.at(
                                minimum_speed->owner)},
                        {"full_hard_row_count",step.diagnostic.full_hard_row_count}
                    };
                }
            } catch (...) {
                failure_certificate={{"audit_error",true}};
            }
            break;
        }
        const double simulated_time=fixture->swarm.robots.front()->runtime;
        const auto truth=auditTruth(*fixture,speed_limit_mps);
        collision_pair_ticks+=truth.collision_violating_pairs;
        reference_edge_ticks+=truth.reference_violating_edges;
        speed_owner_ticks+=truth.speed_violating_owners;
        maximum_reference_distance=std::max(
            maximum_reference_distance,truth.maximum_reference_distance_m);
        const auto covered=fixture->adapter.coverage().truthCoveredCount();
        if (covered>previous_covered) {
            previous_covered=covered;
            last_progress_time=simulated_time;
        }
        const double current_gamma=finiteMinimum(step.diagnostic.current_gamma);
        const double local_predicted=finiteMinimum(
            step.diagnostic.local_maximum_predicted_gamma);
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const auto current_request=fixture->adapter.currentSnapshotHardRowRequest(
            runtime.topology);
        double maximum_estimated_speed=0.0;
        double maximum_tube_robust_speed=0.0;
        for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index) {
            const auto owner=runtime.estimate.mobile_ids[index];
            const double speed=
                runtime.estimate.mean.segment<2>(4*index+2).norm();
            maximum_estimated_speed=std::max(maximum_estimated_speed,speed);
            maximum_tube_robust_speed=std::max(
                maximum_tube_robust_speed,speed+
                    current_request.plant_speed_snapshot_tubes.at(owner)
                        .velocity_radius_mps);
        }
        metrics.observe({simulated_time,step.diagnostic.truth_coverage,
            truth.minimum_mobile_mobile_m,truth.minimum_mobile_fixed_m,
            truth.maximum_speed_mps,current_gamma,local_predicted,
            step.diagnostic.minimum_full_hard_residual,
            static_cast<std::size_t>(covered),0.1,
            step.diagnostic.minimum_plant_speed_applied_control_residual,
            maximum_estimated_speed,maximum_tube_robust_speed});
        try {
            const auto information=fixture->adapter.currentReferenceAudit();
            const bool information_gate_failed=
                information.minimum_effective_reference_count<2 ||
                !std::isfinite(
                    information.minimum_robust_fim_cone_lower_bound) ||
                information.minimum_robust_fim_cone_lower_bound<1.0e-6 ||
                information.maximum_posterior_eigenvalue>
                    fixture->adapter.config().maximum_posterior_eigenvalue_m2 ||
                information.minimum_range_aoi_margin_s<0.0;
            if (information_gate_failed) {
                ++information_hard_gate_failure_ticks;
                if (!std::isfinite(first_information_hard_gate_failure_s))
                    first_information_hard_gate_failure_s=simulated_time;
            }
            if (std::isfinite(
                    information.minimum_robust_fim_cone_lower_bound))
                minimum_robust_fim=std::min(minimum_robust_fim,
                    information.minimum_robust_fim_cone_lower_bound);
            maximum_posterior=std::max(maximum_posterior,
                information.maximum_posterior_eigenvalue);
            minimum_aoi_margin=std::min(minimum_aoi_margin,
                information.minimum_range_aoi_margin_s);
        } catch (...) {
            ++information_failures;
        }
        if (cycle%100==99)
            std::cerr<<"TASK10P11P_NOMINAL t="<<simulated_time
                     <<" coverage="<<step.diagnostic.truth_coverage
                     <<" min_mm="<<truth.minimum_mobile_mobile_m
                     <<" gamma="<<current_gamma<<'\n';
        if (metrics.t100_s.has_value()) break;
    }
    const double wall_s=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-wall_start).count();
    const double simulated_time=fixture->swarm.robots.front()->runtime;
    const bool completed=metrics.t100_s.has_value();
    const std::string outcome=completed?"nominal_t100":
        (!failure_reason.empty()?"nominal_control_failure":"nominal_timeout");
    json output{
        {"protocol","task10p11p_nominal_only_causal_v1"},
        {"cbf2026_source_commit",gf::task10p11pCbf2026SourceCommit()},
        {"outcome",outcome},{"failure_reason",failure_reason},
        {"failure_certificate",failure_certificate},
        {"simulated_time_s",simulated_time},{"wall_time_s",wall_s},
        {"t95_true_s",optionalNumber(metrics.t95_s)},
        {"t100_true_s",optionalNumber(metrics.t100_s)},
        {"truth_coverage",fixture->adapter.coverage().truthFraction()},
        {"covered_cells",fixture->adapter.coverage().truthCoveredCount()},
        {"denominator_cells",90000},
        {"last_new_coverage_time_s",last_progress_time},
        {"minimum_truth_mobile_mobile_distance_m",
            metrics.minimum_truth_mobile_distance_m},
        {"minimum_truth_mobile_fixed_distance_m",
            metrics.minimum_truth_mobile_fixed_distance_m},
        {"collision_violation_pair_ticks",collision_pair_ticks},
        {"collision_any_violation_ticks",
            metrics.collision_violation_pair_ticks},
        {"deepest_collision_intrusion_m",
            metrics.deepest_collision_intrusion_m},
        {"maximum_speed_mps",metrics.maximum_speed_mps},
        {"maximum_estimated_speed_mps",metrics.maximum_estimated_speed_mps},
        {"maximum_tube_robust_speed_mps",
            metrics.maximum_tube_robust_speed_mps},
        {"speed_violation_owner_ticks",speed_owner_ticks},
        {"reference_violation_edge_ticks",reference_edge_ticks},
        {"maximum_reference_distance_m",maximum_reference_distance},
        {"minimum_current_gamma_mps2",metrics.minimum_current_gamma_mps2},
        {"first_current_gamma_negative_s",
            std::isfinite(metrics.first_current_gamma_negative_s)
                ?json(metrics.first_current_gamma_negative_s):json(nullptr)},
        {"minimum_local_maximum_predicted_gamma_mps2",
            metrics.minimum_local_predicted_gamma_mps2},
        {"first_local_predicted_gamma_negative_s",
            std::isfinite(metrics.first_local_predicted_gamma_negative_s)
                ?json(metrics.first_local_predicted_gamma_negative_s):json(nullptr)},
        {"local_predicted_gamma_invalid_ticks",
            metrics.local_predicted_gamma_invalid_ticks},
        {"true_joint_gamma1_status","unknown_not_computed"},
        {"minimum_full_hard_residual_mps2",
            metrics.minimum_full_hard_residual_mps2},
        {"minimum_plant_speed_applied_control_residual_mps2",
            metrics.minimum_plant_speed_applied_control_residual_mps2},
        {"minimum_finite_robust_fim_cone_lower_bound",
            std::isfinite(minimum_robust_fim)
                ?json(minimum_robust_fim):json(nullptr)},
        {"information_hard_gate_failure_ticks",
            information_hard_gate_failure_ticks},
        {"first_information_hard_gate_failure_s",
            std::isfinite(first_information_hard_gate_failure_s)
                ?json(first_information_hard_gate_failure_s):json(nullptr)},
        {"maximum_posterior_eigenvalue",maximum_posterior},
        {"minimum_aoi_margin_s",minimum_aoi_margin},
        {"information_audit_failures",information_failures},
        {"target_epochs",fixture->controller.targetEpoch()}
    };
    std::cout<<output.dump(2)<<'\n';
    return completed?0:3;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=2 || std::string(argv[1])!="nominal") {
        std::cerr<<"usage: GrandFinaleTask10p11p nominal\n";
        return 64;
    }
    return runNominal();
}
