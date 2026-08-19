#include "grand_finale/Task10p11wConflictComponentOracle.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>

namespace {

bool digest(const std::string& value, std::size_t size) {
    return value.size() == size && std::all_of(
        value.begin(), value.end(), [](unsigned char character) {
            return std::isxdigit(character) != 0;
        });
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 10) {
        std::cerr << "usage: GrandFinaleTask10p11w CHECKPOINT_DIR OUTPUT_JSON "
                     "PARENT_COMMIT PARENT_TREE CBF_COMMIT CBF_TREE "
                     "BINARY_SHA256 CHECKPOINT_MANIFEST_SHA256 CONFIG_DIGEST\n";
        return 2;
    }
    if (!digest(argv[3], 40) || !digest(argv[4], 40) ||
        !digest(argv[5], 40) || !digest(argv[6], 40) ||
        !digest(argv[7], 64) || !digest(argv[8], 64)) {
        std::cerr << "provenance_preflight_failed\n";
        return 3;
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
        auto output = gf::runTask10p11wOfflineOracle(paths);
        output["source"] = {{"parent_commit", argv[3]},
            {"parent_tree", argv[4]}, {"cbf_commit", argv[5]},
            {"cbf_tree", argv[6]}, {"binary_sha256", argv[7]},
            {"checkpoint_manifest_sha256", argv[8]},
            {"frozen_model_config_digest", argv[9]}};
        output["checkpoint_directory"] = directory.string();
        const std::filesystem::path destination_path = argv[2];
        if (!destination_path.parent_path().empty())
            std::filesystem::create_directories(destination_path.parent_path());
        std::ofstream destination(destination_path);
        if (!destination) throw std::runtime_error("cannot open output path");
        destination << output.dump(2) << '\n';
        std::cout << output.at("summary").dump(2) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Task 10.11w offline oracle failed: "
                  << error.what() << '\n';
        return 4;
    }
}
