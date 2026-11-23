#include "grand_finale/CheckpointFile.hpp"
#include <iostream>
int main(int argc,char** argv) {
    if(argc!=2&&argc!=3)return 2;
    const auto saved=gf::CheckpointFile::read(argv[1]);
    if(argc==3&&std::string(argv[2])=="--full"){std::cout<<saved.dump()<<'\n';return 0;}
    const auto& program=saved.at("metrics").at("role_program_state");
    nlohmann::json out={{"tick",saved.at("tick")},{"runtime_s",saved.at("runtime_s")},
        {"checkpoint_sha256",gf::CheckpointFile::fileSha256(argv[1])},
        {"requests",nlohmann::json::array()},{"terminal",saved.at("terminal")}};
    if(!program.is_null())for(const auto& r:program.at("requests")) {
        auto compact=r;for(const auto& key:{"bridge","endpoint","endpoint_fronts","real_task_ledger"})compact.erase(key);
        out["requests"].push_back(compact);
    }
    std::cout<<out.dump(2)<<'\n';return 0;
}
