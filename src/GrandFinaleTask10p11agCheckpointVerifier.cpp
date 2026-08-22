#include "grand_finale/Task10p11acCampaign.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=4) return 2;
    try {
        const auto checkpoint=gf::readTask10p11vJson(argv[1]);
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
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage_zero_initialization_failed");
        const auto audit=gf::auditTask10p11vRestartCheckpoint(checkpoint);
        if (!audit.offline_oracle_complete || !audit.capture_fields_complete ||
            !audit.deterministic_restart_complete)
            throw std::runtime_error("packed_audit_failed:"+audit.reason);
        gf::restoreTask10p11vRestartState(
            *fixture,checkpoint.at("restart_checkpoint"));
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const auto request=fixture->adapter.snapshotHardRowRequest(
            runtime.estimate,runtime.topology);
        const auto rebuilt=gf::makeTask10p11sSnapshot(
            runtime,request,gf::task10p11s_capture_detail::nominalFromJson(
                checkpoint.at("nominal_controls")),fixture->adapter.config());
        const bool request_equal=rebuilt.at("canonical_request")==
            checkpoint.at("canonical_request");
        const bool rows_equal=rebuilt.at("actual_rows")==
            checkpoint.at("actual_rows");
        const bool objective_equal=rebuilt.at("objective_28d")==
            checkpoint.at("objective_28d");
        const bool valid=request_equal && rows_equal && objective_equal &&
            fixture->topologyFrozen();
        const nlohmann::json output={{"protocol",
            "task10p11ag-independent-P3-checkpoint-verifier-v1"},
            {"valid",valid},{"input",argv[1]},
            {"runtime_s",runtime.runtime_s},{"request_equal",request_equal},
            {"actual_rows_equal",rows_equal},{"objective_28d_equal",objective_equal},
            {"fixed_topology",fixture->topologyFrozen()},
            {"audit",gf::task10p11vRestartAuditJson(audit)}};
        gf::writeTask10p11vJson(argv[3],output);
        std::cout<<output.dump(2)<<'\n';
        return valid?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag checkpoint verification failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
