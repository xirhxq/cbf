#include "grand_finale/Task10p11agComponentRecovery.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11agComponentGate "
            "INPUT_PACKED OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto controls=gf::task10p11afAppliedControlsFromCheckpoint(
            snapshot);
        const auto gate=gf::runTask10p11agComponentGate(snapshot,controls);
        const auto output=gf::task10p11agComponentGateJson(gate);
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return gate.valid?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag component gate failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
