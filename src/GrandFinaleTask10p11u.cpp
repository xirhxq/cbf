#include "grand_finale/Task10p11uFrozenFailureSnapshot.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

bool hexDigest(const std::string& value, std::size_t length) {
    return value.size() == length && std::all_of(
        value.begin(), value.end(), [](unsigned char character) {
            return std::isxdigit(character) != 0;
        });
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 9) {
        std::cerr << "usage: GrandFinaleTask10p11u INPUT_JSON OUTPUT_JSON "
                     "PARENT_COMMIT PARENT_TREE CBF_COMMIT CBF_TREE "
                     "BINARY_SHA256 INPUT_SHA256\n";
        return 2;
    }
    if (!hexDigest(argv[3], 40) || !hexDigest(argv[4], 40) ||
        !hexDigest(argv[5], 40) || !hexDigest(argv[6], 40) ||
        !hexDigest(argv[7], 64) || !hexDigest(argv[8], 64)) {
        std::cerr << "provenance_preflight_failed\n";
        return 3;
    }
    try {
        const std::filesystem::path input_path = argv[1];
        const std::filesystem::path output_path = argv[2];
        std::ifstream input(input_path);
        if (!input) throw std::runtime_error("cannot open frozen input");
        nlohmann::json frozen;
        input >> frozen;
        nlohmann::json output =
            gf::runTask10p11uFrozenFailureAudit(frozen);
        output["source"] = {
            {"parent_commit", argv[3]}, {"parent_tree", argv[4]},
            {"cbf_commit", argv[5]}, {"cbf_tree", argv[6]},
            {"binary_sha256", argv[7]}, {"input_sha256", argv[8]}};
        output["input_path"] = input_path.string();
        if (!output_path.parent_path().empty())
            std::filesystem::create_directories(output_path.parent_path());
        std::ofstream destination(output_path);
        if (!destination) throw std::runtime_error("cannot open output path");
        destination << output.dump(2) << '\n';
        std::cout << output.dump(2) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Task 10.11u offline audit failed: "
                  << error.what() << '\n';
        return 4;
    }
}
