#include "grand_finale/Task10p11wConflictComponentOracle.hpp"

#include <fstream>
#include <iostream>

namespace {

Eigen::VectorXd controlsFromJson(
    const nlohmann::json& controls,
    const std::vector<gf::NodeId>& mobile_ids) {
    std::map<gf::NodeId, Eigen::Vector2d> mapped;
    for (gf::NodeId owner : mobile_ids) {
        const auto values = controls.at(std::to_string(owner)).get<std::vector<double>>();
        if (values.size() != 2)
            throw std::invalid_argument("control is not 2D");
        mapped.emplace(owner, Eigen::Vector2d(values[0], values[1]));
    }
    return gf::task10p11sOrderedControls(mobile_ids, mapped);
}

std::pair<double, std::string> recompute(
    const gf::Task10p11sRows28d& rows, const nlohmann::json& controls) {
    const Eigen::VectorXd vector = controlsFromJson(controls, rows.mobile_ids);
    double minimum = std::numeric_limits<double>::infinity();
    std::string limiting;
    for (const auto& row : rows.rows) {
        double residual = row.constant;
        for (Eigen::Index index = 0; index < row.coefficient.size(); ++index)
            residual += row.coefficient(index) * vector(index);
        if (!std::isfinite(residual))
            throw std::runtime_error("nonfinite independently recomputed residual");
        if (residual < minimum) {
            minimum = residual;
            limiting = row.id;
        }
    }
    return {minimum, limiting};
}

void checkSolution(nlohmann::json& records, const std::string& label,
                   const gf::Task10p11sRows28d& rows,
                   const nlohmann::json& solution,
                   const std::string& minimum_key) {
    if (!solution.at("feasible").get<bool>()) return;
    const auto independent = recompute(rows, solution.at("controls"));
    const double compiled = solution.at(minimum_key).get<double>();
    records.push_back({{"label", label},
        {"minimum_residual_mps2", independent.first},
        {"limiting_row_id", independent.second},
        {"compiled_minimum_residual_mps2", compiled},
        {"nonnegative_within_tolerance", independent.first >= -1.0e-8},
        {"compiled_match", std::abs(independent.first - compiled) <= 1.0e-10}});
}

gf::Task10p11sRows28d successorRows(
    const nlohmann::json& checkpoint, const nlohmann::json& applied_controls,
    const std::vector<gf::NodeId>& mobile_ids) {
    const Eigen::VectorXd vector = controlsFromJson(applied_controls, mobile_ids);
    const auto request = gf::rebuildTask10p11sSuccessorRequest(
        checkpoint, gf::task10p11sControlMap(mobile_ids, vector));
    return gf::buildTask10p11sRows28d(
        gf::buildCanonicalHardRows(request), request.mobile_ids, true);
}

void checkSuccessor(nlohmann::json& records, const std::string& label,
                    const nlohmann::json& checkpoint,
                    const std::vector<gf::NodeId>& mobile_ids,
                    const nlohmann::json& applied_controls,
                    const nlohmann::json& successor) {
    if (!successor.value("performed", false)) return;
    const auto rows = successorRows(checkpoint, applied_controls, mobile_ids);
    checkSolution(records, label + "/component", rows,
        successor.at("component_feasibility"),
        "minimum_full_row_residual_mps2");
    checkSolution(records, label + "/component-margin", rows,
        successor.at("component_maximum_margin"),
        "minimum_full_row_residual_mps2");
    checkSolution(records, label + "/full", rows,
        successor.at("full_pair_feasibility"),
        "minimum_compiled_residual_mps2");
    checkSolution(records, label + "/full-margin", rows,
        successor.at("full_pair_maximum_margin"),
        "minimum_full_row_residual_mps2");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: GrandFinaleTask10p11wResidualVerifier "
                     "ORACLE_JSON CHECKPOINT_DIR OUTPUT_JSON\n";
        return 2;
    }
    try {
        std::ifstream source(argv[1]);
        nlohmann::json oracle;
        source >> oracle;
        if (oracle.at("protocol") != "task10p11w-offline-oracle-v1")
            throw std::invalid_argument("oracle protocol mismatch");
        const std::filesystem::path checkpoint_directory = argv[2];
        nlohmann::json records = nlohmann::json::array();
        for (const auto& frame : oracle.at("frames")) {
            std::ifstream input(checkpoint_directory /
                                frame.at("checkpoint").get<std::string>());
            nlohmann::json checkpoint;
            input >> checkpoint;
            const auto request = gf::task10p11s_capture_detail::requestFromJson(
                checkpoint.at("canonical_request"));
            const auto current_rows = gf::buildTask10p11sRows28d(
                gf::buildCanonicalHardRows(request), request.mobile_ids, true);
            const std::string prefix = "t" +
                std::to_string(frame.at("time_s").get<double>());
            checkSolution(records, prefix + "/current/full28d", current_rows,
                frame.at("full_28d_oracle"),
                "minimum_compiled_residual_mps2");
            const auto& component = frame.at("component_search");
            checkSolution(records, prefix + "/current/pair2-4", current_rows,
                component.at("pair_2_4_4d"),
                "minimum_full_row_residual_mps2");
            checkSolution(records, prefix + "/current/min-component", current_rows,
                component.at("minimum_component_solution"),
                "minimum_full_row_residual_mps2");
            const auto& homotopy = frame.at("preventive_homotopy");
            checkSolution(records, prefix + "/current/max-margin-endpoint",
                current_rows, homotopy.at("current_maximum_margin_endpoint"),
                "minimum_full_row_residual_mps2");
            checkSuccessor(records, prefix + "/pair2-4/successor", checkpoint,
                request.mobile_ids, component.at("pair_2_4_4d").at("controls"),
                component.at("pair_2_4_successor"));
            checkSuccessor(records, prefix + "/min-component/successor", checkpoint,
                request.mobile_ids,
                component.at("minimum_component_solution").at("controls"),
                component.at("minimum_component_successor"));
            checkSuccessor(records, prefix + "/full28d/successor", checkpoint,
                request.mobile_ids, frame.at("full_28d_oracle").at("controls"),
                frame.at("full_28d_successor"));
            for (const auto& candidate : homotopy.at("candidates")) {
                const auto independent = recompute(current_rows,
                    candidate.at("controls"));
                const double compiled = candidate.at(
                    "current_minimum_full_row_residual_mps2").get<double>();
                const std::string candidate_label = prefix + "/candidate-" +
                    std::to_string(candidate.at("index").get<std::size_t>());
                records.push_back({{"label", candidate_label + "/current"},
                    {"minimum_residual_mps2", independent.first},
                    {"limiting_row_id", independent.second},
                    {"compiled_minimum_residual_mps2", compiled},
                    {"nonnegative_within_tolerance",
                        independent.first >= -1.0e-8},
                    {"compiled_match",
                        std::abs(independent.first - compiled) <= 1.0e-10}});
                checkSuccessor(records, candidate_label + "/successor", checkpoint,
                    request.mobile_ids, candidate.at("controls"),
                    candidate.at("successor"));
            }
        }
        double minimum = std::numeric_limits<double>::infinity();
        bool all_nonnegative = true;
        bool all_match = true;
        for (const auto& record : records) {
            minimum = std::min(minimum,
                record.at("minimum_residual_mps2").get<double>());
            all_nonnegative = all_nonnegative &&
                record.at("nonnegative_within_tolerance").get<bool>();
            all_match = all_match && record.at("compiled_match").get<bool>();
        }
        nlohmann::json output{{"protocol",
            "task10p11w-independent-full-row-residual-v1"},
            {"solution_count", records.size()},
            {"minimum_residual_mps2", minimum},
            {"all_full_row_residuals_nonnegative_within_tolerance",
                all_nonnegative},
            {"all_compiled_independent_minima_match", all_match},
            {"tolerance_mps2", 1.0e-8},
            {"match_tolerance_mps2", 1.0e-10},
            {"records", std::move(records)},
            {"trajectory_run_performed", false}};
        std::ofstream destination(argv[3]);
        destination << output.dump(2) << '\n';
        std::cout << nlohmann::json{{"solution_count", output.at("solution_count")},
            {"minimum_residual_mps2", minimum},
            {"all_nonnegative", all_nonnegative},
            {"all_match", all_match}}.dump(2) << '\n';
        return all_nonnegative && all_match ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Task 10.11w residual verification failed: "
                  << error.what() << '\n';
        return 4;
    }
}
