#include "grand_finale/Task10p11sFull28dGateA.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: GrandFinaleTask10p11sFull28d "
                     "FAILURE_SNAPSHOT_JSON OUTPUT_JSON\n";
        return 2;
    }
    try {
        const std::filesystem::path input_path = argv[1];
        const std::filesystem::path output_path = argv[2];
        const nlohmann::json snapshot = gf::readTask10p11sSnapshot(input_path);
        nlohmann::json output = gf::runTask10p11sFull28dGateA(snapshot);
        output["input_snapshot_path"] = input_path.string();
        if (!output_path.parent_path().empty()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        std::ofstream destination(output_path);
        if (!destination) throw std::runtime_error("cannot open output path");
        destination << output.dump(2) << '\n';
        std::cout << output.dump(2) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Task 10.11s full 28D Gate A failed: "
                  << error.what() << '\n';
        return 3;
    }
}
