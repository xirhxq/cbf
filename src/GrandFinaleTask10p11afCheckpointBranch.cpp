#include "grand_finale/Task10p11afSuccessorRecovery.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

using json=nlohmann::json;
constexpr std::size_t kMaximumCycles=50;

json controlsJson(const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    json result=json::object();
    for (const auto& [owner,control]:controls)
        result[std::to_string(owner)]={control.x(),control.y()};
    return result;
}

std::string checkpointName(std::size_t index,double time,
    const std::string& event) {
    std::ostringstream stream;
    stream<<"checkpoint-"<<std::setw(4)<<std::setfill('0')<<index
        <<"-t"<<std::fixed<<std::setprecision(1)<<time<<'-'<<event<<".json";
    return stream.str();
}

void decorate(json& checkpoint,const std::string& profile,
    const gf::Task10p11afTakeoverDecision& decision,bool advanced) {
    checkpoint["task10p11af"]={{"protocol","task10p11af-checkpoint-branch-v1"},
        {"profile",profile},{"tau_mps2",22.0},{"fixed_topology",true},
        {"advanced",advanced},{"decision",gf::task10p11afTakeoverJson(decision)},
        {"selected_controls",controlsJson(decision.selected_controls)},
        {"development_oracle",true},{"recursive_feasibility_claimed",false}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=6) {
        std::cerr<<"usage: GrandFinaleTask10p11afCheckpointBranch "
            "G1|G2 INPUT_PACKED INIT_MANIFEST OUTPUT_JSON CHECKPOINT_DIR\n";
        return 2;
    }
    const std::string profile=argv[1];
    if (profile!="G1" && profile!="G2") return 2;
    const auto depth=profile=="G1"?gf::Task10p11aaGraphDepth::H1:
        gf::Task10p11aaGraphDepth::H2FiniteWitness;
    const std::filesystem::path output_path=argv[4];
    const std::filesystem::path directory=argv[5];
    try {
        const auto input=gf::readTask10p11vJson(argv[2]);
        const auto manifest=gf::readTask10p11vJson(argv[3]);
        const auto initialization=gf::task10p11acInitializationFromManifest(
            manifest,"P3");
        auto scenario=gf::task10p11rFixedBaselineScenario();
        scenario.mobile_positions=initialization.positions;
        auto settings=gf::task10p11acSwarmSettings(
            scenario,initialization.velocities);
        auto fixture=gf::makeTask10p11rFixedBaselineFixture(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage zero initialization failed");
        gf::restoreTask10p11vRestartState(
            *fixture,input.at("restart_checkpoint"));
        if (!fixture->topologyFrozen())
            throw std::runtime_error("fixed topology identity failed");
        const auto saved_controls=
            gf::task10p11afAppliedControlsFromCheckpoint(input);
        std::optional<std::string> active_pair;
        std::size_t advanced=0;
        std::size_t takeover_cycles=0;
        std::size_t entries=0,exits=0,checkpoint_index=0;
        double cumulative_deviation=0.0;
        bool previously_active=false;
        std::string stop_reason;
        json trace=json::array();
        std::filesystem::create_directories(directory);
        for (std::size_t cycle=0;cycle<kMaximumCycles;++cycle) {
            const auto boundary=gf::task10p11zCaptureBeforeOverride(*fixture);
            const auto prepared=gf::task10p11zPrepareNativeBaseline(
                *fixture,gf::GammaFeedbackSelectionMode::LeastIntervention,
                22.0,active_pair);
            if (!prepared.valid || !prepared.control.step.advanced) {
                stop_reason="distributed_prepare_failed:"+prepared.reason;
                break;
            }
            active_pair=prepared.active_pair;
            if (cycle==0 && !gf::task10p11zAppliedControlsMatch(
                    saved_controls,prepared.control.step.applied_controls,1e-10))
                throw std::runtime_error("restored_P3_distributed_control_mismatch");
            const auto snapshot=gf::makeTask10p11sSnapshot(
                boundary.runtime,boundary.request,prepared.nominal_controls,
                fixture->adapter.config());
            auto decision=gf::task10p11afDecideTakeover(snapshot,
                prepared.control.step.applied_controls,depth,true);
            const double time=boundary.runtime.runtime_s;
            const bool active=decision.takeover_applied;
            if (active && !previously_active) ++entries;
            if (!active && previously_active) ++exits;
            if (!decision.valid) {
                auto checkpoint=gf::makeTask10p11yCurrentPackedCheckpoint(
                    *fixture,"task10p11af_pre_advance_fail_closed");
                decorate(checkpoint,profile,decision,false);
                const auto name=checkpointName(checkpoint_index++,time,
                    "fail_closed");
                gf::writeTask10p11vJson(directory/name,checkpoint);
                trace.push_back({{"time_s",time},{"advanced",false},
                    {"decision",gf::task10p11afTakeoverJson(decision)},
                    {"checkpoint",name}});
                stop_reason=decision.reason;
                break;
            }
            const auto distributed=gf::task10p11sOrderedControls(
                boundary.request.mobile_ids,
                prepared.control.step.applied_controls);
            const auto selected=gf::task10p11sOrderedControls(
                boundary.request.mobile_ids,decision.selected_controls);
            cumulative_deviation+=(selected-distributed).norm();
            std::optional<gf::SimpleCoverageControlStep> committed;
            committed=fixture->controller.advanceWithDevelopmentControlOverride(
                [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
                    const std::map<gf::NodeId,Eigen::Vector2d>& nominal,
                    const std::map<gf::NodeId,double>& yaw_rates) {
                    if (!gf::task10p11zAppliedControlsMatch(
                            nominal,prepared.nominal_controls,1e-12)) {
                        gf::GrandFinaleSwarmStep rejected;
                        rejected.reason="prepared_native_nominal_mismatch";
                        return rejected;
                    }
                    const double successor_residual=decision.takeover_applied
                        ?decision.oracle.candidates.at(
                            decision.oracle.selected_index).
                            successor_full_row_residual_mps2
                        :decision.successor_distributed_residual_mps2;
                    return fixture->adapter.
                        stepWithDevelopmentFullPairCertifiedControls(
                            decision.selected_controls,yaw_rates,
                            runtime.estimator_token,runtime.topology_token,
                            true,successor_residual);
                });
            if (!committed->step.advanced) {
                stop_reason=committed->reason.empty()?committed->step.reason:
                    committed->reason;
                break;
            }
            ++advanced;
            if (active) ++takeover_cycles;
            const std::string event=active!=previously_active
                ?(active?"enter_takeover":"exit_takeover"):"advanced";
            auto checkpoint=gf::makeTask10p11yCurrentPackedCheckpoint(
                *fixture,"task10p11af_"+event);
            decorate(checkpoint,profile,decision,true);
            const auto name=checkpointName(checkpoint_index++,
                fixture->swarm.robots.front()->runtime,event);
            gf::writeTask10p11vJson(directory/name,checkpoint);
            trace.push_back({{"time_s",time},{"advanced",true},
                {"coverage_after",committed->step.truth_coverage},
                {"decision",gf::task10p11afTakeoverJson(decision)},
                {"checkpoint",name}});
            previously_active=active;
        }
        if (stop_reason.empty()) stop_reason="registered_5s_limit";
        const bool crossed_original_failure=
            fixture->swarm.robots.front()->runtime>158.1+1e-9;
        const json result={{"protocol","task10p11af-checkpoint-branch-result-v1"},
            {"valid",true},{"profile",profile},{"tau_mps2",22.0},
            {"start_time_s",input.at("runtime").at("runtime_s")},
            {"final_time_s",fixture->swarm.robots.front()->runtime},
            {"advanced_cycles",advanced},{"takeover_cycles",takeover_cycles},
            {"takeover_entries",entries},{"takeover_exits",exits},
            {"cumulative_coverage_control_deviation_l2_mps2",
                cumulative_deviation},{"crossed_original_P3_failure",
                crossed_original_failure},{"stop_reason",stop_reason},
            {"trace",std::move(trace)},{"checkpoint_count",checkpoint_index},
            {"claim_boundary",{{"fixed_topology_development_oracle",true},
                {"candidate_limited_H2",profile=="G2"},
                {"recursive_feasibility_claimed",false},
                {"production_controller",false}}}};
        gf::writeTask10p11vJson(output_path,result);
        std::cout<<result.dump(2)<<'\n';
        return crossed_original_failure?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11af checkpoint branch failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
