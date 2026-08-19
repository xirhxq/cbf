#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11vRestartFixture "
            "CHECKPOINT_JSON EXPECTED_AFTER_JSON\n";
        return 2;
    }
    try {
        auto fixture=gf::makeTask10p11rFixedBaselineFixture();
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage zero failed: "+initialized.reason);
        for (std::size_t cycle=0;cycle<2;++cycle) {
            const auto step=fixture->controller.advance();
            if (!step.step.advanced)
                throw std::runtime_error("prefix failed: "+step.reason);
        }
        gf::writeTask10p11vJson(argv[1],
            gf::makeTask10p11vSparseRestartCheckpoint(
                *fixture,std::nullopt,2));
        const auto next=fixture->controller.advance();
        if (!next.step.advanced)
            throw std::runtime_error("shadow step failed: "+next.reason);
        gf::writeTask10p11vJson(argv[2],{{"protocol",
            "task10p11v-restart-shadow-v1"},{"advanced",true},
            {"restart_state",gf::task10p11vRestartStateJson(*fixture)}});
        std::cout<<nlohmann::json{{"checkpoint",argv[1]},
            {"expected_after",argv[2]},{"prefix_cycles",2},
            {"trajectory_run",false}}.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"restart fixture failed: "<<error.what()<<'\n';
        return 4;
    }
}
