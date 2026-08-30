#include "grand_finale/Task10p11aiTerminalCertificate.hpp"

#include <chrono>
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
    if (argc!=8&&argc!=10) {
        std::cerr<<"usage: GrandFinaleTask10p11aiWitnessSearch PACKED_157P8 "
            "INIT_MANIFEST FULL28_H2 LEGACY_COMPONENT STAGEA_RESULT OUTPUT_JSON "
            "PROGRESS_DIRECTORY [MAX_EVALUATIONS WALL_CLOCK_S]\n";
        return 2;
    }
    try {
        const auto protocol=gf::task10p11aiProtocol();
        const auto packed=gf::readTask10p11vJson(argv[1]);
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const auto full=gf::readTask10p11vJson(argv[3]);
        const auto legacy=gf::readTask10p11vJson(argv[4]);
        const auto stage_a=gf::readTask10p11vJson(argv[5]);
        const std::filesystem::path progress_directory=argv[7];
        std::filesystem::create_directories(progress_directory);
        const std::size_t maximum_evaluations=argc==10?
            static_cast<std::size_t>(std::stoull(argv[8])):520;
        const double wall_clock_limit_s=argc==10?std::stod(argv[9]):7200.0;
        auto fixture=makeP3(manifest);
        gf::restoreTask10p11vRestartState(*fixture,
            packed.at("restart_checkpoint"));
        if (!fixture->topologyFrozen())
            throw std::runtime_error("fixed topology identity failed");
        const auto owners=fixture->adapter.runtimeSnapshot().estimate.
            mobile_ids;
        // Anchor: the frozen plan must reproduce the 10.11ah terminal before
        // any search begins.
        gf::Task10p11ahComponentPlan frozen_plan;
        frozen_plan.owner2_u0=Eigen::Vector2d(-4,4);
        frozen_plan.owner9_u0=Eigen::Vector2d(4,-4);
        frozen_plan.owner2_u1=Eigen::Vector2d(-4,4);
        frozen_plan.owner9_u1=Eigen::Vector2d(4,-4);
        const auto anchor=gf::evaluateTask10p11ahTerminalRecoveryPlan(
            *fixture,std::nullopt,frozen_plan);
        if (!anchor.valid||std::abs(
                anchor.terminal_native_successor_residual_mps2-
                    protocol.anchor_successor_residual_mps2)>
                protocol.anchor_tolerance_mps2)
            throw std::runtime_error("anchor mismatch before witness search");
        auto native=nativePlan(*fixture);
        const auto starts=gf::task10p11aiWitnessStartPlans(
            componentPlan(legacy.at("x0").at("controls"),
                legacy.at("x1").at("controls"),owners),
            componentPlan(full.at("U0"),full.at("U1"),owners),
            native,
            gf::task10p11aiChainIncumbentsFromStageA(stage_a));
        json start_index=json::array();
        for (const auto& plan:starts) {
            const auto vector=gf::task10p11ahPlanVector(plan);
            start_index.push_back(std::vector<double>(vector.data(),
                vector.data()+vector.size()));
        }
        std::size_t index=0;
        const auto evaluator=[&](const gf::Task10p11ahComponentPlan& plan) {
            const auto evaluation=gf::evaluateTask10p11ahTerminalRecoveryPlan(
                *fixture,std::nullopt,plan);
            const auto vector=gf::task10p11ahPlanVector(plan);
            const json record={{"index",++index},
                {"plan",std::vector<double>(vector.data(),
                    vector.data()+vector.size())},
                {"result",{{"valid",evaluation.valid},
                    {"classification",
                        gf::task10p11ahOptimizerClassificationName(
                            evaluation.classification)},
                    {"terminal_recovered",
                        evaluation.score.terminal_recovered},
                    {"terminal_recovery_margin_mps2",
                        gf::task10p11w_detail::number(
                            evaluation.score.terminal_recovery_margin_mps2)},
                    {"minimum_full_row_residual_mps2",
                        gf::task10p11w_detail::number(
                            evaluation.score.
                                minimum_full_row_residual_mps2)}}}};
            std::ostringstream name;
            name<<"evaluation-"<<std::setw(4)<<std::setfill('0')
                <<index<<".json";
            gf::writeTask10p11vJson(progress_directory/name.str(),record);
            std::cout<<"evaluation "<<index<<" "
                <<gf::task10p11ahOptimizerClassificationName(
                    evaluation.classification)<<" margin="
                <<evaluation.score.terminal_recovery_margin_mps2<<'\n'
                <<std::flush;
            return evaluation;
        };
        gf::Task10p11ahPatternSearchSettings settings;
        settings.maximum_evaluations=maximum_evaluations;
        settings.wall_clock_limit_s=wall_clock_limit_s;
        const auto result=gf::solveTask10p11ahTerminalRecovery(
            starts,evaluator,settings);
        const auto vector=gf::task10p11ahPlanVector(result.plan);
        const json output={{"protocol",
            "task10p11ai-extended-witness-search-v1"},
            {"preregistration",protocol.preregistration},
            {"stage","B"},
            {"valid",result.valid},{"reason",result.reason},
            {"terminal_recovery_witness_found",
                result.terminal_recovery_witness_found},
            {"optimizer_classification",
                gf::task10p11ahOptimizerClassificationName(
                    result.classification)},
            {"timed_out",result.timed_out},
            {"evaluations",result.evaluations},
            {"solve_time_s",result.solve_time_s},
            {"registered_starts",start_index},
            {"start_count",starts.size()},
            {"maximum_evaluations",maximum_evaluations},
            {"wall_clock_limit_s",wall_clock_limit_s},
            {"selected_plan_vector",std::vector<double>(vector.data(),
                vector.data()+vector.size())},
            {"evaluation",task10p11ahPlanEvaluationJson(result.evaluation)},
            {"claim_boundary",{{"witness_releases_checkpoint_branch",true},
                {"conditional_preauthorization_short_branch_only",true},
                {"no_witness_is_not_infeasibility",true},
                {"stage_zero_authorized",false},
                {"component_expansion",false},{"horizon_expansion",false},
                {"dynamic_topology",false},
                {"recursive_feasibility_claimed",false}}}};
        gf::writeTask10p11vJson(argv[6],output);
        std::cout<<output.dump(2)<<'\n';
        return result.terminal_recovery_witness_found?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ai witness search failed: "<<error.what()<<'\n';
        return 4;
    }
}
