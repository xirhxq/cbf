#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11sSnapshotCapture.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>

int main(int argc,char** argv) {
    if (argc!=2) {
        std::cerr<<"usage: GrandFinaleTask10p11sSnapshotFixture OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto started=std::chrono::steady_clock::now();
        auto fixture=gf::makeTask10p11rFixedBaselineFixture();
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage zero failed: "+initialized.reason);
        const auto before=fixture->adapter.runtimeSnapshot();
        const auto request=fixture->adapter.snapshotHardRowRequest(
            before.estimate,before.topology);
        const auto step=fixture->controller.advance();
        if (!step.step.advanced)
            throw std::runtime_error("short fixture failed: "+step.reason);
        const auto snapshot=gf::makeTask10p11sSnapshot(
            before,request,fixture->controller.lastNominalControls(),
            fixture->adapter.config());
        gf::writeTask10p11sSnapshot(argv[1],snapshot);
        const auto loaded=gf::readTask10p11sSnapshot(argv[1]);
        const auto validation=gf::validateTask10p11sSnapshot(loaded);
        const double wall=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        std::cout<<nlohmann::json{{"complete",validation.complete},
            {"rows_match",validation.rows_match},
            {"objective_matches",validation.objective_matches},
            {"owner_count",validation.owner_count},
            {"row_count",validation.row_count},{"wall_time_s",wall},
            {"snapshot_bytes",std::filesystem::file_size(argv[1])},
            {"long_horizon_run",false},{"gate_b_run",false}}.dump(2)<<'\n';
        return validation.complete?0:3;
    } catch (const std::exception& error) {
        std::cerr<<"snapshot fixture failed: "<<error.what()<<'\n';
        return 4;
    }
}
