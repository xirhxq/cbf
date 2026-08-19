#include "grand_finale/Task10p11xRecoveryCampaign.hpp"

#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: GrandFinaleTask10p11xStageA CHECKPOINT_DIR OUTPUT_JSON\n";
        return 2;
    }
    try {
        const std::filesystem::path directory = argv[1];
        const std::vector<std::filesystem::path> paths = {
            directory / "checkpoint-000-t132.4-first_dynamic_intervention.json",
            directory / "checkpoint-001-t132.5-dynamic_intervention.json",
            directory / "checkpoint-002-t132.6-dynamic_intervention.json",
            directory / "checkpoint-003-t132.7-dynamic_intervention.json",
            directory / "checkpoint-004-t132.8-dynamic_intervention.json",
            directory / "checkpoint-005-t132.9-fail_closed.json"};
        auto output = gf::runTask10p11xStageAPreregistration(paths);
        const std::filesystem::path destination = argv[2];
        if (!destination.parent_path().empty())
            std::filesystem::create_directories(destination.parent_path());
        gf::writeTask10p11vJson(destination, output);
        std::cout << output.at("routes").dump(2) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Stage A failed: " << error.what() << '\n';
        return 4;
    }
}
