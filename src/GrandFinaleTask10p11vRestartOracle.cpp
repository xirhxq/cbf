#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11vRestartOracle "
            "CHECKPOINT_JSON EXPECTED_AFTER_JSON\n";
        return 2;
    }
    try {
        const auto checkpoint=gf::readTask10p11vJson(argv[1]);
        const auto expected=gf::readTask10p11vJson(argv[2]);
        if (expected.at("protocol").get<std::string>()!=
            "task10p11v-restart-shadow-v1")
            throw std::invalid_argument("unsupported shadow protocol");
        auto fixture=gf::makeTask10p11rFixedBaselineFixture();
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage zero failed: "+initialized.reason);
        const auto metadata=gf::restoreTask10p11vSparseRestartCheckpoint(
            *fixture,checkpoint);
        const auto next=fixture->controller.advance();
        if (!next.step.advanced)
            throw std::runtime_error("restored step failed: "+next.reason);
        const auto actual=gf::task10p11vRestartStateJson(*fixture);
        if (actual!=expected.at("restart_state"))
            throw std::runtime_error("restored next-boundary shadow mismatch");
        std::cout<<nlohmann::json{{"protocol",
            "task10p11v-independent-restart-oracle-v1"},
            {"cycle",metadata.cycle},{"restored",true},
            {"next_boundary_equal",true},{"trajectory_run",false}}.dump(2)
            <<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"restart oracle failed: "<<error.what()<<'\n';
        return 4;
    }
}
