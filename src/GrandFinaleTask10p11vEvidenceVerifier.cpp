#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11vEvidenceVerifier "
            "INPUT_JSON OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto input=gf::readTask10p11vJson(argv[1]);
        auto fixture=gf::makeTask10p11rFixedBaselineFixture();
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage zero failed: "+initialized.reason);
        nlohmann::json output{{"protocol",
            "task10p11v-independent-checkpoint-verifier-v1"},
            {"input",argv[1]},{"restore_equal",false},
            {"rows_objective_rebuild_equal",false}};
        const std::string protocol=input.at("protocol").get<std::string>();
        if (protocol=="task10p11v-sparse-restart-v1") {
            const auto metadata=gf::restoreTask10p11vSparseRestartCheckpoint(
                *fixture,input);
            output["kind"]="sparse";
            output["cycle"]=metadata.cycle;
            output["restore_equal"]=true;
            output["rows_objective_rebuild_equal"]=nullptr;
        } else if (protocol=="task10p11s-minimal-snapshot-v1") {
            const auto audit=gf::auditTask10p11vRestartCheckpoint(input);
            if (!audit.offline_oracle_complete ||
                !audit.capture_fields_complete ||
                !audit.deterministic_restart_complete)
                throw std::runtime_error("packed audit failed: "+audit.reason);
            gf::restoreTask10p11vRestartState(
                *fixture,input.at("restart_checkpoint"));
            const auto runtime=fixture->adapter.runtimeSnapshot();
            const auto request=fixture->adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology);
            const auto rebuilt=gf::makeTask10p11sSnapshot(runtime,request,
                gf::task10p11s_capture_detail::nominalFromJson(
                    input.at("nominal_controls")),
                fixture->adapter.config());
            const bool equal=rebuilt.at("actual_rows")==
                    input.at("actual_rows") &&
                rebuilt.at("objective_28d")==input.at("objective_28d") &&
                rebuilt.at("canonical_request")==
                    input.at("canonical_request");
            if (!equal)
                throw std::runtime_error("packed rows/objective mismatch");
            output["kind"]="packed";
            output["restore_equal"]=true;
            output["rows_objective_rebuild_equal"]=true;
            output["audit"]=gf::task10p11vRestartAuditJson(audit);
        } else {
            throw std::invalid_argument("unsupported checkpoint protocol");
        }
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"evidence verification failed: "<<error.what()<<'\n';
        return 4;
    }
}
