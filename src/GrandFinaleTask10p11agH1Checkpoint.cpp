#include "grand_finale/Task10p11agComponentRecovery.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <filesystem>
#include <iostream>

namespace {

using json=nlohmann::json;

gf::SimpleCoverageControlStep advanceNative(
    gf::Task10p11rFixedBaselineFixture& fixture,
    std::optional<std::string>& active_pair) {
    const auto runtime=fixture.adapter.runtimeSnapshot();
    if (!active_pair.has_value()) {
        const auto rows=gf::buildCanonicalHardRows(
            fixture.adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology));
        const auto diagnostic=gf::diagnoseTask10p11tOnlineConflicts(
            rows,runtime.estimate.mobile_ids,runtime.runtime_s,
            runtime.mode,runtime.topology,
            fixture.adapter.config().acceleration_half_box);
        if (!diagnostic.valid) throw std::runtime_error(diagnostic.reason);
        if (diagnostic.infeasible) {
            active_pair=gf::task10p11tUniqueOnlinePair(diagnostic);
            if (!active_pair.has_value())
                throw std::runtime_error("nonunique_dynamic_pair_conflict");
        }
    }
    return active_pair.has_value()
        ?fixture.controller.advanceWithDynamicPairResponsibility(*active_pair)
        :fixture.controller.advance();
}

json controlsJson(const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    json result=json::object();
    for (const auto& [owner,control]:controls)
        result[std::to_string(owner)]={control.x(),control.y()};
    return result;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=5) {
        std::cerr<<"usage: GrandFinaleTask10p11agH1Checkpoint "
            "INPUT_PACKED INIT_MANIFEST OUTPUT_JSON CHECKPOINT_DIR\n";
        return 2;
    }
    try {
        const auto input=gf::readTask10p11vJson(argv[1]);
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const auto initialization=gf::task10p11acInitializationFromManifest(
            manifest,"P3");
        auto scenario=gf::task10p11rFixedBaselineScenario();
        scenario.mobile_positions=initialization.positions;
        auto settings=gf::task10p11acSwarmSettings(
            scenario,initialization.velocities);
        auto fixture=gf::makeTask10p11rFixedBaselineFixture(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
        if (!fixture->adapter.initializeStageZero().initialized)
            throw std::runtime_error("stage_zero_initialization_failed");
        gf::restoreTask10p11vRestartState(
            *fixture,input.at("restart_checkpoint"));
        if (!fixture->topologyFrozen())
            throw std::runtime_error("fixed_topology_identity_failed");
        std::optional<std::string> active_pair;
        std::filesystem::create_directories(argv[4]);
        json trace=json::array();
        std::size_t advanced=0,takeovers=0,checkpoint_count=0;
        double deviation=0.0;
        const double start=fixture->swarm.robots.front()->runtime;
        const double initial_coverage=fixture->adapter.coverage().truthFraction();
        std::string stop_reason;
        for (std::size_t cycle=0;cycle<100;++cycle) {
            const auto boundary=gf::task10p11zCaptureBeforeOverride(*fixture);
            const auto prepared=gf::task10p11zPrepareNativeBaseline(
                *fixture,gf::GammaFeedbackSelectionMode::LeastIntervention,
                22.0,active_pair);
            if (!prepared.valid || !prepared.control.step.advanced) {
                stop_reason="distributed_prepare_failed:"+prepared.reason;
                break;
            }
            active_pair=prepared.active_pair;
            auto snapshot=gf::makeTask10p11sSnapshot(
                boundary.runtime,boundary.request,prepared.nominal_controls,
                fixture->adapter.config());
            const auto distributed=gf::task10p11sOrderedControls(
                boundary.request.mobile_ids,
                prepared.control.step.applied_controls);
            const auto context=gf::task10p11ag_detail::makeContext(
                snapshot,distributed);
            const auto baseline=gf::task10p11ag_detail::evaluate(
                context,distributed);
            const bool trigger=baseline.solved && !baseline.witness;
            bool takeover=false;
            gf::task10p11ag_component_detail::Attempt component;
            std::map<gf::NodeId,Eigen::Vector2d> selected=
                prepared.control.step.applied_controls;
            if (trigger) {
                component=gf::task10p11ag_component_detail::evaluateComponent(
                    snapshot,distributed,{2,9});
                if (!component.successor_feasible) {
                    auto checkpoint=gf::makeTask10p11vRestartCheckpoint(
                        snapshot,boundary.restart_fields,
                        "task10p11ag_h1_fail_closed");
                    checkpoint["task10p11ag"]={{"protocol",
                            "task10p11ag-h1-checkpoint-v1"},
                        {"advanced",false},{"trigger",true},
                        {"component",json::array({2,9})},
                        {"reason","component_successor_infeasible"}};
                    const auto path=std::filesystem::path(argv[4])/
                        "checkpoint-fail-closed.json";
                    gf::writeTask10p11vJson(path,checkpoint);
                    ++checkpoint_count;
                    trace.push_back({{"time_s",boundary.runtime.runtime_s},
                        {"advanced",false},{"trigger",true},
                        {"baseline_successor_gamma_mps2",
                            baseline.successor_gamma},
                        {"component_successor_feasible",false},
                        {"checkpoint",path.filename().string()}});
                    stop_reason="component_successor_infeasible_fail_closed";
                    break;
                }
                takeover=true;
                selected=gf::task10p11sControlMap(
                    boundary.request.mobile_ids,component.current.controls);
                deviation+=(component.current.controls-distributed).norm();
                ++takeovers;
            }
            gf::SimpleCoverageControlStep step;
            if (!takeover) {
                step=advanceNative(*fixture,active_pair);
            } else {
                step=fixture->controller.advanceWithDevelopmentControlOverride(
                    [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
                        const std::map<gf::NodeId,Eigen::Vector2d>&,
                        const std::map<gf::NodeId,double>& yaw_rates) {
                        return fixture->adapter.
                            stepWithDevelopmentFullPairCertifiedControls(
                                selected,yaw_rates,runtime.estimator_token,
                                runtime.topology_token,true,
                                component.successor_full_margin.minimum_residual);
                    });
            }
            if (!step.step.advanced) {
                stop_reason=step.reason.empty()?step.step.reason:step.reason;
                break;
            }
            ++advanced;
            trace.push_back({{"time_s",boundary.runtime.runtime_s},
                {"advanced",true},{"trigger",trigger},
                {"takeover",takeover},
                {"baseline_successor_gamma_mps2",baseline.successor_gamma},
                {"current_minimum_residual_mps2",
                    baseline.current_minimum_residual},
                {"coverage_after",step.step.truth_coverage},
                {"selected_controls",controlsJson(selected)}});
            if (fixture->swarm.robots.front()->runtime>158.1+1.0e-9) {
                stop_reason="crossed_original_failure_interval";
                break;
            }
        }
        if (stop_reason.empty()) stop_reason="registered_10s_limit";
        const double final_time=fixture->swarm.robots.front()->runtime;
        const json output={{"protocol","task10p11ag-h1-checkpoint-result-v1"},
            {"valid",true},{"start_time_s",start},{"final_time_s",final_time},
            {"advanced_cycles",advanced},{"takeover_cycles",takeovers},
            {"component",json::array({2,9})},
            {"crossed_original_failure_interval",final_time>158.1+1.0e-9},
            {"coverage_increment",fixture->adapter.coverage().truthFraction()-
                initial_coverage},
            {"cumulative_coverage_control_deviation_l2_mps2",deviation},
            {"checkpoint_count",checkpoint_count},
            {"stop_reason",stop_reason},{"trace",std::move(trace)},
            {"claim_boundary",{{"finite_checkpoint_branch",true},
                {"h1_successor_recovery",true},
                {"recursive_feasibility_claimed",false},
                {"stage_zero_trajectory_run",false}}}};
        gf::writeTask10p11vJson(argv[3],output);
        std::cout<<output.dump(2)<<'\n';
        return output.at("crossed_original_failure_interval").get<bool>()?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag H1 checkpoint failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
