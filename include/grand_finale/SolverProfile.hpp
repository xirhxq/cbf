#pragma once

#include "grand_finale/Types.hpp"
#include "json.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace gf {

struct SolverSelection {
    SolverProfile profile;
    std::string topology;
    std::string control;
    std::string reserve;
    std::uint32_t seed;
};

inline SolverSelection loadSolverSelection(const nlohmann::json& config) {
    const nlohmann::json& solver = config.at("solver");
    if (!solver.contains("seed")) {
        throw std::invalid_argument("solver.seed is required");
    }

    const std::string profile = solver.at("profile").get<std::string>();
    const std::string topology = solver.at("topology").get<std::string>();
    const std::string control = solver.at("control").get<std::string>();
    const std::string reserve = solver.at("reserve").get<std::string>();
    if (!solver.at("seed").is_number_integer()) {
        throw std::invalid_argument("solver.seed must be a uint32 integer");
    }
    const std::int64_t seed_value = solver.at("seed").get<std::int64_t>();
    if (seed_value < 0 ||
        seed_value > static_cast<std::int64_t>(
                         std::numeric_limits<std::uint32_t>::max())) {
        throw std::invalid_argument("solver.seed must be a uint32 integer");
    }
    const std::uint32_t seed = static_cast<std::uint32_t>(seed_value);

    if (profile == "gurobi") {
        if (topology != "gurobi_miqp" || control != "gurobi_qp" ||
            reserve != "gurobi_lp") {
            throw std::invalid_argument(
                "gurobi profile requires gurobi_miqp/gurobi_qp/gurobi_lp");
        }
        return SolverSelection{SolverProfile::Gurobi, topology, control,
                               reserve, seed};
    }

    if (profile == "open_source") {
        if (topology != "highs_milp" || control != "osqp_qp" ||
            reserve != "highs_lp") {
            throw std::invalid_argument(
                "open_source profile requires highs_milp/osqp_qp/highs_lp");
        }
        return SolverSelection{SolverProfile::OpenSource, topology, control,
                               reserve, seed};
    }

    throw std::invalid_argument("solver.profile must be gurobi or open_source");
}

}  // namespace gf
