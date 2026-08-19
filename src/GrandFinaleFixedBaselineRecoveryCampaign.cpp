#include "grand_finale/Task10p11xRecoveryCampaign.hpp"
#include "grand_finale/Task10p11qStandardSafetyOn.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

using json = nlohmann::json;

struct TruthMargins {
    double mobile_mobile = std::numeric_limits<double>::infinity();
    double mobile_fixed = std::numeric_limits<double>::infinity();
    double speed = std::numeric_limits<double>::infinity();
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
        result.speed=std::min(result.speed,30.0-velocity.norm());
    }
    for (std::size_t index=0;index<fixture.scenario.mobile_ids.size();++index) {
        const auto owner=fixture.scenario.mobile_ids[index];
        for (std::size_t peer=index+1;
             peer<fixture.scenario.mobile_ids.size();++peer)
            result.mobile_mobile=std::min(result.mobile_mobile,
                (positions.at(owner)-positions.at(
                    fixture.scenario.mobile_ids[peer])).norm());
        for (const auto& [fixed,position]:fixture.scenario.fixed_positions) {
            (void)fixed;
            result.mobile_fixed=std::min(result.mobile_fixed,
                (positions.at(owner)-position).norm());
        }
    }
    return result;
}

std::string eventName(std::size_t index,double time,const std::string& event) {
    std::ostringstream value;
    value<<"checkpoint-"<<std::setw(4)<<std::setfill('0')<<index
         <<"-t"<<std::fixed<<std::setprecision(1)<<time<<"-"<<event<<".json";
    return value.str();
}

json run(const std::string& mode,
         const std::optional<std::filesystem::path>& restore_path,
         const std::filesystem::path& checkpoint_directory,
         const json& source) {
    const bool short_branch=mode=="short";
    const std::size_t maximum_cycles=short_branch?100:5000;
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        gf::GammaFeedbackSelectionMode::LeastIntervention,14.0);
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized)
        throw std::runtime_error("stage_zero_initialization_failed:"+
                                 initialized.reason);
    if (restore_path.has_value()) {
        const auto checkpoint=gf::readTask10p11vJson(*restore_path);
        const auto audit=gf::auditTask10p11vRestartCheckpoint(checkpoint);
        if (!audit.deterministic_restart_complete)
            throw std::runtime_error("restart_checkpoint_not_recoverable:"+
                                     audit.reason);
        gf::restoreTask10p11vRestartState(
            *fixture,checkpoint.at("restart_checkpoint"));
    }
    if (fixture->adapter.config().range_noise_std_m!=0.0 ||
        fixture->adapter.config().range_dropout_probability!=0.0 ||
        !fixture->topologyFrozen())
        throw std::runtime_error("frozen_identity_preflight_failed");

    std::filesystem::create_directories(checkpoint_directory);
    const double initial_time=fixture->swarm.robots.front()->runtime;
    const double initial_coverage=fixture->adapter.coverage().truthFraction();
    std::optional<double> first_intervention;
    std::optional<double> t95;
    std::optional<double> t100;
    bool component_active=false;
    std::size_t component_cycles=0;
    std::size_t component_entries=0;
    std::size_t checkpoint_index=0;
    std::size_t sparse_index=0;
    std::size_t stagnant_cycles=0;
    int previous_covered=fixture->adapter.coverage().truthCoveredCount();
    double minimum_full_residual=std::numeric_limits<double>::infinity();
    double minimum_successor_full_residual=
        std::numeric_limits<double>::infinity();
    double minimum_mobile_mobile=std::numeric_limits<double>::infinity();
    double minimum_mobile_fixed=std::numeric_limits<double>::infinity();
    double minimum_speed=std::numeric_limits<double>::infinity();
    double minimum_fim=std::numeric_limits<double>::infinity();
    double maximum_posterior=0.0;
    double minimum_aoi=std::numeric_limits<double>::infinity();
    std::size_t minimum_references=std::numeric_limits<std::size_t>::max();
    std::string stop_reason;
    json trace=json::array();
    json coverage_progress=json::array({{{"time_s",initial_time},
        {"truth_coverage",initial_coverage},
        {"covered_cells",fixture->adapter.coverage().truthCoveredCount()}}});
    const auto wall_start=std::chrono::steady_clock::now();

    for (std::size_t cycle=0;cycle<maximum_cycles;++cycle) {
        if (!fixture->topologyFrozen()) {
            stop_reason="fixed_topology_mutated";
            break;
        }
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const double decision_time=runtime.runtime_s;
        if (cycle%100==0) {
            auto sparse=gf::makeTask10p11vSparseRestartCheckpoint(
                *fixture,component_active
                    ?std::optional<std::string>("component:2--4")
                    :std::nullopt,cycle);
            sparse["campaign"]={{"protocol",
                "fixed-baseline-multi-path-recovery-v1"},
                {"mode",mode},{"route","pair_2_4_component"}};
            sparse["source"]=source;
            sparse["config_digest"]=gf::task10p11qConfigDigest(
                fixture->adapter.config());
            std::ostringstream name;
            name<<"sparse-"<<std::setw(4)<<std::setfill('0')<<sparse_index++
                <<"-t"<<std::fixed<<std::setprecision(1)<<decision_time
                <<".json";
            gf::writeTask10p11vJson(checkpoint_directory/name.str(),sparse);
        }

        const auto request=fixture->adapter.snapshotHardRowRequest(
            runtime.estimate,runtime.topology);
        const auto restart_fields=gf::captureTask10p11vRestartFields(*fixture);
        const bool active_before=component_active;
        std::optional<gf::task10p11x_detail::ComponentFallbackDecision> decision;
        const auto control=fixture->controller.advanceWithDevelopmentControlOverride(
            [&](const gf::GrandFinaleRuntimeSnapshot& callback_runtime,
                const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
                const std::map<gf::NodeId,double>& yaw_rates) {
                const auto snapshot=gf::makeTask10p11sSnapshot(
                    callback_runtime,
                    fixture->adapter.snapshotHardRowRequest(
                        callback_runtime.estimate,callback_runtime.topology),
                    nominal,fixture->adapter.config());
                decision=gf::task10p11x_detail::decidePairComponentFallback(
                    snapshot,component_active);
                if (!decision->valid) {
                    gf::GrandFinaleSwarmStep rejected;
                    rejected.reason=decision->reason;
                    return rejected;
                }
                component_active=decision->use_component;
                return fixture->adapter.
                    stepWithDevelopmentFullPairCertifiedControls(
                        decision->controls,yaw_rates,
                        callback_runtime.estimator_token,
                        callback_runtime.topology_token,
                        decision->successor_audit.full_pair,
                        decision->successor_audit.full_residual);
            });
        if (!decision.has_value())
            throw std::runtime_error("component decision callback missing");
        const bool mode_changed=component_active!=active_before;
        if (component_active) {
            ++component_cycles;
            if (!active_before) {
                ++component_entries;
                if (!first_intervention.has_value())
                    first_intervention=decision_time;
            }
        }
        minimum_full_residual=std::min(minimum_full_residual,
            decision->current_full_row_residual_mps2);
        minimum_successor_full_residual=std::min(
            minimum_successor_full_residual,
            decision->successor_audit.full_residual);
        const bool final_planned=cycle+1==maximum_cycles;
        const bool save_event=mode_changed || !control.step.advanced ||
            final_planned;
        if (save_event) {
            const auto base=gf::makeTask10p11sSnapshot(runtime,request,
                fixture->controller.lastNominalControls(),
                fixture->adapter.config());
            std::string event=!control.step.advanced?"fail_closed":
                mode_changed?(component_active?"component_enter":"component_exit"):
                "branch_limit";
            auto checkpoint=gf::makeTask10p11vRestartCheckpoint(
                base,restart_fields,event);
            checkpoint["campaign"]={{"protocol",
                "fixed-baseline-multi-path-recovery-v1"},
                {"mode",mode},{"route","pair_2_4_component"},
                {"decision",gf::task10p11x_detail::
                    componentFallbackDecisionJson(*decision)},
                {"source",source}};
            gf::writeTask10p11vJson(checkpoint_directory/
                eventName(checkpoint_index++,decision_time,event),checkpoint);
        }
        trace.push_back({{"decision_time_s",decision_time},
            {"advanced",control.step.advanced},
            {"component_active",component_active},
            {"decision",gf::task10p11x_detail::
                componentFallbackDecisionJson(*decision)},
            {"coverage_after",control.step.truth_coverage},
            {"reason",control.reason}});
        if (!control.step.advanced) {
            stop_reason=control.reason.empty()?control.step.reason:control.reason;
            break;
        }
        const auto information=fixture->adapter.currentReferenceAudit();
        minimum_fim=std::min(minimum_fim,
            information.minimum_robust_fim_cone_lower_bound);
        maximum_posterior=std::max(maximum_posterior,
            information.maximum_posterior_eigenvalue);
        minimum_aoi=std::min(minimum_aoi,
            information.minimum_range_aoi_margin_s);
        minimum_references=std::min(minimum_references,
            information.minimum_effective_reference_count);
        if (information.minimum_effective_reference_count<2)
            stop_reason="information_effective_reference_failure";
        else if (information.minimum_robust_fim_cone_lower_bound<1.0e-6)
            stop_reason="information_robust_fim_failure";
        else if (information.maximum_posterior_eigenvalue>
                 fixture->adapter.config().maximum_posterior_eigenvalue_m2)
            stop_reason="information_posterior_failure";
        else if (information.minimum_range_aoi_margin_s<0.0)
            stop_reason="information_aoi_failure";
        const auto truth=truthMargins(*fixture);
        minimum_mobile_mobile=std::min(minimum_mobile_mobile,truth.mobile_mobile);
        minimum_mobile_fixed=std::min(minimum_mobile_fixed,truth.mobile_fixed);
        minimum_speed=std::min(minimum_speed,truth.speed);
        if (truth.mobile_mobile<10.0-1e-9 || truth.mobile_fixed<10.0-1e-9)
            stop_reason="truth_collision";
        else if (truth.speed<-1e-9)
            stop_reason="plant_speed_violation";
        if (!stop_reason.empty()) break;
        const double time=fixture->swarm.robots.front()->runtime;
        const double coverage=fixture->adapter.coverage().truthFraction();
        if (!t95.has_value() && coverage>=0.95-1e-12) t95=time;
        if (!t100.has_value() && coverage>=1.0-1e-12) t100=time;
        if ((cycle+1)%10==0 || t95.has_value() || t100.has_value())
            coverage_progress.push_back({{"time_s",time},
                {"truth_coverage",coverage},
                {"covered_cells",fixture->adapter.coverage().truthCoveredCount()},
                {"component_active",component_active}});
        if (t100.has_value()) {
            stop_reason="t100";
            break;
        }
        const int covered=fixture->adapter.coverage().truthCoveredCount();
        stagnant_cycles=covered==previous_covered?stagnant_cycles+1:0;
        previous_covered=covered;
        if (!short_branch && stagnant_cycles>=1000) {
            stop_reason="coverage_stagnation_100s";
            break;
        }
    }
    if (stop_reason.empty()) stop_reason=short_branch?"short_branch_10s_limit":
        "long_horizon_500s_limit";
    const double final_time=fixture->swarm.robots.front()->runtime;
    const double final_coverage=fixture->adapter.coverage().truthFraction();
    const double wall=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-wall_start).count();
    return {{"protocol","fixed-baseline-multi-path-recovery-v1"},
        {"mode",mode},{"route","pair_2_4_component"},
        {"restored_from_checkpoint",restore_path.has_value()},
        {"initial_time_s",initial_time},{"final_time_s",final_time},
        {"crossed_132p9",final_time>132.9+1e-9},
        {"initial_coverage",initial_coverage},
        {"final_coverage",final_coverage},
        {"coverage_increment",final_coverage-initial_coverage},
        {"t95_s",t95.has_value()?json(*t95):json(nullptr)},
        {"t100_s",t100.has_value()?json(*t100):json(nullptr)},
        {"first_intervention_s",first_intervention.has_value()
            ?json(*first_intervention):json(nullptr)},
        {"component_cycles",component_cycles},
        {"component_duration_s",0.1*component_cycles},
        {"component_entries",component_entries},
        {"minimum_independent_full_row_residual_mps2",minimum_full_residual},
        {"minimum_successor_full_row_residual_mps2",
            minimum_successor_full_residual},
        {"hard_gates",{{"minimum_mobile_mobile_distance_m",minimum_mobile_mobile},
            {"minimum_mobile_fixed_distance_m",minimum_mobile_fixed},
            {"minimum_speed_margin_mps",minimum_speed},
            {"minimum_robust_fim",minimum_fim},
            {"maximum_posterior_eigenvalue_m2",maximum_posterior},
            {"minimum_aoi_margin_s",minimum_aoi},
            {"minimum_effective_references",minimum_references},
            {"topology_frozen",fixture->topologyFrozen()}}},
        {"stop_reason",stop_reason},{"wall_time_s",wall},
        {"coverage_progress",coverage_progress},{"trace",trace},
        {"source",source},
        {"claim_boundary",{{"finite_trajectory_only",true},
            {"recursive_feasibility_claimed",false},
            {"task_11_entered",false}}}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=9) {
        std::cerr<<"usage: GrandFinaleFixedBaselineRecoveryCampaign "
            "short|long CHECKPOINT_OR_DASH CHECKPOINT_DIR RESULT_JSON "
            "PARENT_COMMIT PARENT_TREE CBF_COMMIT CBF_TREE\n";
        return 2;
    }
    try {
        const std::string mode=argv[1];
        if (mode!="short" && mode!="long")
            throw std::invalid_argument("mode must be short or long");
        const std::optional<std::filesystem::path> restore=
            std::string(argv[2])=="-"?std::nullopt:
                std::optional<std::filesystem::path>(argv[2]);
        if ((mode=="short")!=restore.has_value())
            throw std::invalid_argument("short requires checkpoint; long requires dash");
        const json source={{"parent_commit",argv[5]},
            {"parent_tree",argv[6]},{"cbf_commit",argv[7]},
            {"cbf_tree",argv[8]}};
        const auto output=run(mode,restore,argv[3],source);
        gf::writeTask10p11vJson(argv[4],output);
        std::cout<<output.dump(2)<<'\n';
        return output.at("stop_reason") == "t100" ||
            (mode=="short" && output.at("stop_reason")==
                "short_branch_10s_limit") ?0:3;
    } catch (const std::exception& error) {
        std::cerr<<"campaign failed: "<<error.what()<<'\n';
        return 4;
    }
}
