#include "grand_finale/Task10p11ahTerminalRecoveryOptimizer.hpp"

#include <iostream>

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

json residualAudit(const json& packed,const json& encoded_estimate,
    const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    const auto estimate=gf::task10p11s_capture_detail::estimateFromJson(
        encoded_estimate);
    const auto request=gf::task10p11x_detail::requestAtEstimate(
        packed,estimate);
    const auto problem=gf::buildTask10p11sRows28d(
        gf::buildCanonicalHardRows(request),request.mobile_ids,true);
    const auto ordered=gf::task10p11sOrderedControls(
        request.mobile_ids,controls);
    const double minimum=gf::task10p11af_detail::independentMinimumResidual(
        problem,ordered);
    return {{"row_count",problem.rows.size()},
        {"minimum_residual_mps2",minimum},
        {"all_rows",gf::task10p11ag_detail::residualsJson(
            problem,ordered)}};
}

bool close(double first,double second,double tolerance=1.0e-8) {
    return std::isfinite(first) && std::isfinite(second) &&
        std::abs(first-second)<=tolerance;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=5) {
        std::cerr<<"usage: GrandFinaleTask10p11ahTerminalRecoveryVerifier "
            "PACKED_157P8 INIT_MANIFEST OPTIMIZER_RESULT OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto packed=gf::readTask10p11vJson(argv[1]);
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const auto optimizer=gf::readTask10p11vJson(argv[3]);
        const auto encoded=optimizer.at("selected_plan_vector")
            .get<std::vector<double>>();
        if (encoded.size()!=8)
            throw std::runtime_error("optimizer plan dimension mismatch");
        Eigen::Matrix<double,8,1> vector;
        for (Eigen::Index index=0;index<8;++index)
            vector(index)=encoded.at(static_cast<std::size_t>(index));
        auto fixture=makeP3(manifest);
        gf::restoreTask10p11vRestartState(
            *fixture,packed.at("restart_checkpoint"));
        const auto rebuilt=gf::evaluateTask10p11ahTerminalRecoveryPlan(
            *fixture,std::nullopt,gf::task10p11ahPlanFromVector(vector));
        if (!rebuilt.valid)
            throw std::runtime_error("independent plan rebuild failed:"+
                                     rebuilt.reason);
        const auto rows0=residualAudit(
            packed,rebuilt.x0_estimator,rebuilt.selected_u0);
        const auto rows1=residualAudit(
            packed,rebuilt.x1_estimator,rebuilt.selected_u1);
        const auto rows2=residualAudit(
            packed,rebuilt.x2_estimator,rebuilt.distributed_u2);
        const double independent_deviation=
            gf::task10p11ah_optimizer_detail::deviation(
                rebuilt.selected_u0,rebuilt.distributed_u0)+
            gf::task10p11ah_optimizer_detail::deviation(
                rebuilt.selected_u1,rebuilt.distributed_u1);
        const auto& saved=optimizer.at("evaluation");
        const bool summaries_match=
            saved.at("full_rows_feasible").get<bool>()==
                rebuilt.score.full_rows_feasible &&
            saved.at("terminal_recovered").get<bool>()==
                rebuilt.score.terminal_recovered &&
            close(saved.at("minimum_full_row_residual_mps2").get<double>(),
                rebuilt.score.minimum_full_row_residual_mps2) &&
            close(saved.at("terminal_recovery_margin_mps2").get<double>(),
                rebuilt.score.terminal_recovery_margin_mps2) &&
            close(saved.at("cumulative_coverage_deviation_l2_mps2")
                    .get<double>(),independent_deviation);
        const bool all_rows_nonnegative=
            rows0.at("row_count")==1113 && rows1.at("row_count")==1113 &&
            rows2.at("row_count")==1113 &&
            rows0.at("minimum_residual_mps2").get<double>()>=-1.0e-8 &&
            rows1.at("minimum_residual_mps2").get<double>()>=-1.0e-8 &&
            rows2.at("minimum_residual_mps2").get<double>()>=-1.0e-8;
        const bool valid=summaries_match && all_rows_nonnegative;
        const json output={{"protocol",
            "task10p11ah-terminal-recovery-independent-verifier-v1"},
            {"valid",valid},{"saved_summary_matches_rebuild",summaries_match},
            {"all_current_rows_nonnegative",all_rows_nonnegative},
            {"independent_cumulative_coverage_deviation_l2_mps2",
                independent_deviation},
            {"rebuilt",gf::task10p11ahPlanEvaluationJson(rebuilt)},
            {"independent_rows",{{"x0",rows0},{"x1",rows1},{"x2",rows2}}},
            {"claim_boundary",{{"rows_rebuilt_read_only",true},
                {"optimizer_rerun",false},
                {"local_no_witness_is_not_infeasibility",true},
                {"recursive_feasibility_claimed",false}}}};
        gf::writeTask10p11vJson(argv[4],output);
        std::cout<<json{{"valid",valid},
            {"saved_summary_matches_rebuild",summaries_match},
            {"all_current_rows_nonnegative",all_rows_nonnegative},
            {"independent_cumulative_coverage_deviation_l2_mps2",
                independent_deviation}}.dump(2)<<'\n';
        return valid?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ah terminal verifier failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
