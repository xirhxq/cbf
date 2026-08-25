#include "grand_finale/Task10p11ahTerminalRecoveryOptimizer.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

using json=nlohmann::json;

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeP3(
    const json& manifest) {
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
        throw std::runtime_error("P3 stage-zero initialization failed");
    return fixture;
}

gf::Task10p11ahComponentPlan componentPlan(
    const json& first,const json& second,
    const std::vector<gf::NodeId>& owners) {
    const auto u0=gf::task10p11ah_detail::controlsFromJson(first,owners);
    const auto u1=gf::task10p11ah_detail::controlsFromJson(second,owners);
    gf::Task10p11ahComponentPlan plan;
    plan.owner2_u0=u0.at(2); plan.owner9_u0=u0.at(9);
    plan.owner2_u1=u1.at(2); plan.owner9_u1=u1.at(9);
    return plan;
}

gf::Task10p11ahComponentPlan nativePlan(
    const gf::Task10p11rFixedBaselineFixture& source) {
    auto shadow=gf::task10p11ah_optimizer_detail::cloneFixture(
        source,std::nullopt);
    auto frame0=gf::task10p11ah_optimizer_detail::nativeFrame(
        *shadow,std::nullopt);
    const auto successor=gf::task10p11af_detail::successorFullPair(
        frame0.snapshot,frame0.prepared.control.step.applied_controls);
    if (!successor.feasible)
        throw std::runtime_error("native registered start has no successor");
    const auto applied=gf::task10p11ah_optimizer_detail::applyVerified(
        *shadow,frame0,frame0.prepared.control.step.applied_controls,
        successor.minimum_residual);
    if (!applied.step.advanced)
        throw std::runtime_error("native registered start failed to advance");
    auto frame1=gf::task10p11ah_optimizer_detail::nativeFrame(
        *shadow,frame0.prepared.active_pair);
    gf::Task10p11ahComponentPlan plan;
    plan.owner2_u0=frame0.prepared.control.step.applied_controls.at(2);
    plan.owner9_u0=frame0.prepared.control.step.applied_controls.at(9);
    plan.owner2_u1=frame1.prepared.control.step.applied_controls.at(2);
    plan.owner9_u1=frame1.prepared.control.step.applied_controls.at(9);
    return plan;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=7) {
        std::cerr<<"usage: GrandFinaleTask10p11ahTerminalRecoveryOptimizer "
            "PACKED_157P8 INIT_MANIFEST FULL28_H2 LEGACY_COMPONENT "
            "OUTPUT_JSON PROGRESS_DIRECTORY\n";
        return 2;
    }
    try {
        const auto packed=gf::readTask10p11vJson(argv[1]);
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const auto full=gf::readTask10p11vJson(argv[3]);
        const auto legacy=gf::readTask10p11vJson(argv[4]);
        auto fixture=makeP3(manifest);
        gf::restoreTask10p11vRestartState(
            *fixture,packed.at("restart_checkpoint"));
        if (!fixture->topologyFrozen())
            throw std::runtime_error("fixed topology identity failed");
        const auto owners=fixture->adapter.runtimeSnapshot().estimate.mobile_ids;
        const std::vector<gf::Task10p11ahComponentPlan> starts{
            componentPlan(legacy.at("x0").at("controls"),
                legacy.at("x1").at("controls"),owners),
            componentPlan(full.at("U0"),full.at("U1"),owners),
            nativePlan(*fixture)};
        const std::filesystem::path progress_directory=argv[6];
        std::filesystem::create_directories(progress_directory);
        json progress_index={{"protocol",
            "task10p11ah-posthoc-terminal-optimizer-progress-v2"},
            {"complete",false},{"evaluation_files",json::array()}};
        std::size_t index=0;
        const auto evaluator=[&](const gf::Task10p11ahComponentPlan& plan) {
            const auto evaluation=gf::evaluateTask10p11ahTerminalRecoveryPlan(
                *fixture,std::nullopt,plan);
            const auto vector=gf::task10p11ahPlanVector(plan);
            const json record={{"index",++index},
                {"plan",std::vector<double>(vector.data(),
                    vector.data()+vector.size())},
                {"result",{{"valid",evaluation.valid},
                    {"reason",evaluation.reason},
                    {"classification",
                        gf::task10p11ahOptimizerClassificationName(
                            evaluation.classification)},
                    {"full_rows_feasible",
                        evaluation.score.full_rows_feasible},
                    {"terminal_recovered",
                        evaluation.score.terminal_recovered},
                    {"minimum_full_row_residual_mps2",
                        gf::task10p11w_detail::number(
                            evaluation.score.
                                minimum_full_row_residual_mps2)},
                    {"terminal_recovery_margin_mps2",
                        gf::task10p11w_detail::number(
                            evaluation.score.
                                terminal_recovery_margin_mps2)},
                    {"coverage_deviation_l2_mps2",
                        gf::task10p11w_detail::number(
                            evaluation.score.
                                cumulative_coverage_deviation_l2_mps2)}}}};
            std::ostringstream name;
            name<<"evaluation-"<<std::setw(4)<<std::setfill('0')
                <<index<<".json";
            gf::writeTask10p11vJson(progress_directory/name.str(),record);
            progress_index["evaluation_files"].push_back(name.str());
            std::cout<<"evaluation "<<index<<" classification="
                <<gf::task10p11ahOptimizerClassificationName(
                    evaluation.classification)<<" terminal_margin="
                <<evaluation.score.terminal_recovery_margin_mps2
                <<" deviation="
                <<evaluation.score.cumulative_coverage_deviation_l2_mps2
                <<'\n'<<std::flush;
            return evaluation;
        };
        gf::Task10p11ahPatternSearchSettings settings;
        const auto result=gf::solveTask10p11ahTerminalRecovery(
            starts,evaluator,settings);
        progress_index["complete"]=true;
        progress_index["result_classification"]=
            gf::task10p11ahOptimizerClassificationName(result.classification);
        gf::writeTask10p11vJson(
            progress_directory/"progress-index.json",progress_index);
        const auto output=gf::task10p11ahOptimizerResultJson(result);
        gf::writeTask10p11vJson(argv[5],output);
        std::cout<<output.dump(2)<<'\n';
        return result.terminal_recovery_witness_found?0:
            (result.valid?5:4);
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ah terminal optimizer failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
