#pragma once

#include "grand_finale/TopologyModel.hpp"

#include <map>
#include <string>
#include <vector>

namespace gf {

inline constexpr double kTopologyFeasibilityTolerance = 1.0e-9;
inline constexpr double kTopologyLexicographicFreezeTolerance = 1.0e-8;

enum class TopologySolveStatus { Optimal, Feasible, Infeasible, Error };

struct TopologySolution {
    TopologySolveStatus status = TopologySolveStatus::Error;
    std::vector<DirectedEdge> edges;
    std::map<NodeId, std::size_t> levels;
    LexicographicObjective objective;
    std::string detail;
};

class TopologySolver {
public:
    virtual ~TopologySolver() = default;
    virtual TopologySolution solve(const TopologyModel& model) = 0;
};

}  // namespace gf
