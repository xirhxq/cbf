#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>

int main(int argc,char** argv) {
    if (argc!=2) {
        std::cerr<<"usage: GrandFinaleTask10p11vCheckpointFixture OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto started=std::chrono::steady_clock::now();
        auto fixture=gf::makeTask10p11rFixedBaselineFixture();
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage zero failed: "+initialized.reason);
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const auto request=fixture->adapter.snapshotHardRowRequest(
            runtime.estimate,runtime.topology);
        const auto restart_fields=gf::captureTask10p11vRestartFields(*fixture);
        const auto step=fixture->controller.advance();
        if (!step.step.advanced)
            throw std::runtime_error("short fixture failed: "+step.reason);
        const auto base=gf::makeTask10p11sSnapshot(runtime,request,
            fixture->controller.lastNominalControls(),
            fixture->adapter.config());
        const auto checkpoint=gf::makeTask10p11vRestartCheckpoint(
            base,restart_fields,fixture->controller,"short_fixture");
        gf::writeTask10p11sSnapshot(argv[1],checkpoint);
        const auto audit=gf::auditTask10p11vRestartCheckpoint(checkpoint);
        const double wall=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        std::cout<<nlohmann::json{{"audit",gf::task10p11vRestartAuditJson(audit)},
            {"wall_time_s",wall},
            {"checkpoint_bytes",std::filesystem::file_size(argv[1])},
            {"trajectory_run",false}}.dump(2)<<'\n';
        return audit.offline_oracle_complete && audit.capture_fields_complete
            ?0:3;
    } catch (const std::exception& error) {
        std::cerr<<"checkpoint fixture failed: "<<error.what()<<'\n';
        return 4;
    }
}
