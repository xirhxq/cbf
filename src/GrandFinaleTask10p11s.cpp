#include "grand_finale/Task10p11sCentralizedDiagnostic.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: GrandFinaleTask10p11s INPUT_JSON OUTPUT_JSON "
                     "PROVENANCE_JSON\n";
        return 2;
    }

    const std::filesystem::path input_path = argv[1];
    const std::filesystem::path output_path = argv[2];
    const std::filesystem::path provenance_path = argv[3];
    try {
        std::ifstream input(input_path);
        if (!input) {
            throw std::runtime_error("cannot open frozen evidence input");
        }
        nlohmann::json evidence;
        input >> evidence;

        std::ifstream provenance_input(provenance_path);
        if (!provenance_input) {
            throw std::runtime_error("cannot open provenance input");
        }
        nlohmann::json provenance;
        provenance_input >> provenance;
        if (!gf::task10p11sValidProvenance(provenance)) {
            throw std::runtime_error("provenance schema incomplete");
        }

        nlohmann::json output = gf::auditTask10p11sEvidenceJson(
            evidence,
            provenance.at("snapshot_input_sha256").get<std::string>(),
            provenance.at("config_digest").get<std::string>());
        output["outcome"] = output.at("gate_b_authorized").get<bool>()
            ? "gate_a_complete_and_passed"
            : "stopped_at_gate_a";
        output["input_evidence_path"] = input_path.string();
        output["provenance"] = provenance;
        output["long_horizon_rerun_performed"] = false;
        output["gate_b_run_performed"] = false;

        if (!output_path.parent_path().empty()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        std::ofstream destination(output_path);
        if (!destination) {
            throw std::runtime_error("cannot open Gate A output path");
        }
        destination << output.dump(2) << '\n';
        std::cout << output.dump(2) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Task 10.11s Gate A failed: " << error.what() << '\n';
        try {
            if (!output_path.parent_path().empty()) {
                std::filesystem::create_directories(output_path.parent_path());
            }
            std::ofstream destination(output_path);
            destination << nlohmann::json{
                {"protocol", "task10p11s-gate-a-v1"},
                {"outcome", "runner_schema_or_io_failure"},
                {"reason", "schema_integrity_failure"},
                {"detail", error.what()},
                {"input_evidence_path", input_path.string()},
                {"provenance_path", provenance_path.string()},
                {"gate_b_authorized", false},
                {"gate_b_run_performed", false},
                {"long_horizon_rerun_performed", false}}.dump(2) << '\n';
        } catch (const std::exception&) {
            std::cerr << "Task 10.11s could not retain failure evidence\n";
        }
        return 3;
    }
}
