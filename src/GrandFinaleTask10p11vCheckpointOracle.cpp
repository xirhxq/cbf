#include "grand_finale/Task10p11sFull28dGateA.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <chrono>
#include <fstream>
#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask10p11vCheckpointOracle INPUT_JSON OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto started=std::chrono::steady_clock::now();
        const auto checkpoint=gf::readTask10p11sSnapshot(argv[1]);
        const auto audit=gf::auditTask10p11vRestartCheckpoint(checkpoint);
        if (!audit.offline_oracle_complete || !audit.capture_fields_complete)
            throw std::runtime_error("checkpoint preflight failed: "+audit.reason);
        const auto oracle=gf::runTask10p11sFull28dGateA(checkpoint);
        const double wall=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        nlohmann::json output{{"protocol","task10p11v-independent-oracle-v1"},
            {"checkpoint_audit",gf::task10p11vRestartAuditJson(audit)},
            {"gate_a",oracle},{"wall_time_s",wall},
            {"trajectory_run",false}};
        std::ofstream destination(argv[2]);
        if (!destination) throw std::runtime_error("cannot open output path");
        destination<<output.dump(2)<<'\n';
        std::cout<<output.dump(2)<<'\n';
        return oracle.at("snapshot_complete").get<bool>() &&
            oracle.at("full_pair").at("feasible").get<bool>() &&
            oracle.at("successor").at("performed").get<bool>() &&
            oracle.at("successor").at("feasible").get<bool>() ?0:3;
    } catch (const std::exception& error) {
        std::cerr<<"checkpoint oracle failed: "<<error.what()<<'\n';
        return 4;
    }
}
