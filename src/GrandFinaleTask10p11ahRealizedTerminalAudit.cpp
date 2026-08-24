#include "grand_finale/Task10p11ahEarlyH2Recovery.hpp"

#include <filesystem>
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

double maximumAbsoluteDifference(const Eigen::VectorXd& first,
                                 const Eigen::VectorXd& second) {
    if (first.size()!=second.size())
        return std::numeric_limits<double>::infinity();
    return (first-second).cwiseAbs().maxCoeff();
}

double maximumAbsoluteDifference(const Eigen::MatrixXd& first,
                                 const Eigen::MatrixXd& second) {
    if (first.rows()!=second.rows() || first.cols()!=second.cols())
        return std::numeric_limits<double>::infinity();
    return (first-second).cwiseAbs().maxCoeff();
}

json controlsJson(const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    json result=json::object();
    for (const auto& [owner,control]:controls)
        result[std::to_string(owner)]={control.x(),control.y()};
    return result;
}

json applyWitnessControl(gf::Task10p11rFixedBaselineFixture& fixture,
                         const std::map<gf::NodeId,Eigen::Vector2d>& controls,
                         std::optional<std::string>& active_pair) {
    const auto boundary=gf::task10p11zCaptureBeforeOverride(fixture);
    const auto prepared=gf::task10p11zPrepareNativeBaseline(
        fixture,gf::GammaFeedbackSelectionMode::LeastIntervention,22.0,
        active_pair);
    if (!prepared.valid || !prepared.control.step.advanced)
        throw std::runtime_error("native proposal unavailable:"+prepared.reason);
    active_pair=prepared.active_pair;
    const auto snapshot=gf::makeTask10p11sSnapshot(
        boundary.runtime,boundary.request,prepared.nominal_controls,
        fixture.adapter.config());
    const auto problem=gf::buildTask10p11sRows28d(
        gf::buildCanonicalHardRows(boundary.request),
        boundary.request.mobile_ids,true);
    const auto ordered=gf::task10p11sOrderedControls(
        boundary.request.mobile_ids,controls);
    const double current_residual=
        gf::task10p11af_detail::independentMinimumResidual(problem,ordered);
    const auto successor=gf::task10p11af_detail::successorFullPair(
        snapshot,controls);
    if (current_residual<-gf::task10p11ag_detail::kTolerance ||
        !successor.feasible)
        return {{"advanced",false},{"reason","witness_invalid_at_realized_boundary"},
            {"time_s",boundary.runtime.runtime_s},
            {"current_minimum_residual_mps2",gf::task10p11w_detail::number(
                current_residual)},
            {"successor_minimum_residual_mps2",gf::task10p11w_detail::number(
                successor.minimum_residual)}};
    const auto step=fixture.controller.advanceWithDevelopmentControlOverride(
        [&](const gf::GrandFinaleRuntimeSnapshot& runtime,
            const std::map<gf::NodeId,Eigen::Vector2d>&,
            const std::map<gf::NodeId,double>& yaw_rates) {
            return fixture.adapter.stepWithDevelopmentFullPairCertifiedControls(
                controls,yaw_rates,runtime.estimator_token,
                runtime.topology_token,true,successor.minimum_residual);
        });
    return {{"advanced",step.step.advanced},
        {"reason",step.reason.empty()?step.step.reason:step.reason},
        {"time_s",boundary.runtime.runtime_s},
        {"current_minimum_residual_mps2",current_residual},
        {"successor_minimum_residual_mps2",successor.minimum_residual},
        {"coverage_after",step.step.truth_coverage}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=6) {
        std::cerr<<"usage: GrandFinaleTask10p11ahRealizedTerminalAudit "
            "PACKED_157P8 H2_WITNESS INIT_MANIFEST OUTPUT_JSON OUTPUT_PACKED\n";
        return 2;
    }
    try {
        const auto packed=gf::readTask10p11vJson(argv[1]);
        const auto witness=gf::readTask10p11vJson(argv[2]);
        const auto manifest=gf::readTask10p11vJson(argv[3]);
        auto fixture=makeP3(manifest);
        gf::restoreTask10p11vRestartState(
            *fixture,packed.at("restart_checkpoint"));
        if (!fixture->topologyFrozen())
            throw std::runtime_error("fixed topology identity failed");
        const auto owners=fixture->adapter.runtimeSnapshot().estimate.mobile_ids;
        const bool full_domain_witness=witness.contains("U0");
        const auto u0=gf::task10p11ah_detail::controlsFromJson(
            full_domain_witness?witness.at("U0"):
                witness.at("x0").at("controls"),owners);
        const auto u1=gf::task10p11ah_detail::controlsFromJson(
            full_domain_witness?witness.at("U1"):
                witness.at("x1").at("controls"),owners);
        std::optional<std::string> active_pair;
        const auto step0=applyWitnessControl(*fixture,u0,active_pair);
        json step1={{"advanced",false},{"reason","not_applicable"}};
        if (step0.at("advanced").get<bool>())
            step1=applyWitnessControl(*fixture,u1,active_pair);

        json terminal={{"audited",false}};
        if (step1.at("advanced").get<bool>()) {
            const auto boundary=gf::task10p11zCaptureBeforeOverride(*fixture);
            const auto prepared=gf::task10p11zPrepareNativeBaseline(
                *fixture,gf::GammaFeedbackSelectionMode::LeastIntervention,
                22.0,active_pair);
            if (!prepared.valid || !prepared.control.step.advanced)
                throw std::runtime_error("terminal native proposal unavailable:"+
                                         prepared.reason);
            const auto snapshot=gf::makeTask10p11sSnapshot(
                boundary.runtime,boundary.request,prepared.nominal_controls,
                fixture->adapter.config());
            const auto rows=gf::buildCanonicalHardRows(boundary.request);
            const auto problem=gf::buildTask10p11sRows28d(
                rows,boundary.request.mobile_ids,true);
            const auto native=gf::task10p11sOrderedControls(
                boundary.request.mobile_ids,
                prepared.control.step.applied_controls);
            const double current_residual=
                gf::task10p11af_detail::independentMinimumResidual(
                    problem,native);
            const auto successor=gf::task10p11af_detail::successorFullPair(
                snapshot,prepared.control.step.applied_controls);
            const auto signed_transfer=gf::solveTask10p11tDynamicPair(
                rows,boundary.request.mobile_ids,prepared.nominal_controls,
                boundary.request.acceleration_half_box,"collision:2--9");
            terminal={{"audited",true},
                {"time_s",boundary.runtime.runtime_s},
                {"owner_local_gamma_mps2",
                    gf::task10p11af_detail::minimumOwnerLocalGamma(
                        rows,boundary.request)},
                {"signed_transfer_feasible",signed_transfer.feasible},
                {"native_current_full_pair_feasible",
                    current_residual>=-gf::task10p11ag_detail::kTolerance},
                {"native_current_minimum_residual_mps2",current_residual},
                {"native_successor_full_pair_feasible",successor.feasible},
                {"native_successor_gamma_mps2",successor.recomputed_gamma},
                {"native_successor_minimum_residual_mps2",
                    successor.minimum_residual}};
            if (full_domain_witness) {
                const auto oracle_x2=
                    gf::task10p11s_capture_detail::estimateFromJson(
                        witness.at("x2"));
                terminal["oracle_vs_realized_mean_max_abs"]=
                    maximumAbsoluteDifference(
                        oracle_x2.mean,boundary.runtime.estimate.mean);
                terminal["oracle_vs_realized_covariance_max_abs"]=
                    maximumAbsoluteDifference(
                        oracle_x2.covariance,
                        boundary.runtime.estimate.covariance);
            }
        }
        auto output_checkpoint=gf::makeTask10p11yCurrentPackedCheckpoint(
            *fixture,"task10p11ah_realized_terminal_audit");
        output_checkpoint["task10p11ac"]={{"decision",{
            {"applied_controls",controlsJson(
                step1.at("advanced").get<bool>()?u1:u0)}}}};
        output_checkpoint["task10p11ah"]={{"protocol",
            "task10p11ah-realized-terminal-audit-v1"},
            {"step0",step0},{"step1",step1},{"terminal",terminal},
            {"development_oracle",true},
            {"stage_zero_trajectory_run",false},
            {"recursive_feasibility_claimed",false}};
        gf::writeTask10p11vJson(argv[5],output_checkpoint);
        const json result={{"protocol",
            "task10p11ah-realized-terminal-audit-result-v1"},
            {"valid",true},{"step0",step0},{"step1",step1},
            {"terminal",terminal},
            {"packed_checkpoint",std::filesystem::path(argv[5]).filename().string()},
            {"claim_boundary",{{"offline_fixture",true},
            {"old_full_14_owner_witness",full_domain_witness},
            {"component_plan",!full_domain_witness},
                {"recursive_feasibility_claimed",false}}}};
        gf::writeTask10p11vJson(argv[4],result);
        std::cout<<result.dump(2)<<'\n';
        return step1.at("advanced").get<bool>()?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ah realized terminal audit failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
