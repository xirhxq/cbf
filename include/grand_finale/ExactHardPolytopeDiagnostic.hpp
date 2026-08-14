#pragma once

#include "grand_finale/HybridSupervisor.hpp"
#include "grand_finale/ProgressCompatibility.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace gf {

struct HardPolytopeRowRecord {
    std::string id;
    CanonicalHardRowKind kind = CanonicalHardRowKind::Auxiliary;
    NodeId responsibility_owner = 0;
    std::optional<NodeId> peer;
    Eigen::Vector2d normal = Eigen::Vector2d::Zero();
    double constant = 0.0;
    double barrier_h = std::numeric_limits<double>::infinity();
    double barrier_psi1 = std::numeric_limits<double>::infinity();
    double position_reserve_m = 0.0;
    double velocity_reserve_mps = 0.0;
    double coefficient_reserve = 0.0;
};

struct HardPolytopeCertificate {
    NodeId owner = 0;
    double time_s = 0.0;
    SupervisorMode mode = SupervisorMode::Search;
    std::vector<DirectedEdge> topology;
    double input_half_box = 0.0;
    bool exact_feasible = false;
    std::vector<std::string> minimal_conflict_row_ids;
    std::vector<HardPolytopeRowRecord> rows;
};

namespace exact_hard_polytope_detail {
inline bool feasible(
    const std::vector<CanonicalHardRow>& rows, NodeId owner,
    double input_half_box) {
    return evaluateProgressCompatibility(
        rows, owner, Eigen::Vector2d::Zero(), input_half_box,
        ProgressCompatibilityConfig{
            std::numeric_limits<double>::max(), 0.0, 1.0e-10, true})
        .polytope_nonempty;
}
}  // namespace exact_hard_polytope_detail

inline HardPolytopeCertificate diagnoseHardPolytope(
    NodeId owner,
    double time_s,
    SupervisorMode mode,
    std::vector<DirectedEdge> topology,
    const std::vector<CanonicalHardRow>& all_rows,
    double input_half_box) {
    if (owner <= 0 || !std::isfinite(time_s) || time_s < 0.0 ||
        !std::isfinite(input_half_box) || input_half_box <= 0.0) {
        throw std::invalid_argument("invalid hard-polytope diagnostic input");
    }
    HardPolytopeCertificate result;
    result.owner = owner;
    result.time_s = time_s;
    result.mode = mode;
    result.topology = std::move(topology);
    result.input_half_box = input_half_box;
    std::vector<CanonicalHardRow> rows;
    for (const auto& row : all_rows) {
        if (row.owner != owner || row.kind == CanonicalHardRowKind::InputBox)
            continue;
        rows.push_back(row);
        result.rows.push_back({
            row.id, row.kind, row.owner, row.peer, row.control_coefficient,
            row.constant, row.barrier_h, row.barrier_psi1,
            row.position_uncertainty_reserve_m,
            row.velocity_uncertainty_reserve_mps,
            row.coefficient_uncertainty_reserve});
    }
    std::sort(rows.begin(), rows.end(),
        [](const auto& a, const auto& b) { return a.id < b.id; });
    std::sort(result.rows.begin(), result.rows.end(),
        [](const auto& a, const auto& b) { return a.id < b.id; });
    result.exact_feasible =
        exact_hard_polytope_detail::feasible(rows, owner, input_half_box);
    if (result.exact_feasible) return result;

    for (std::size_t count = 1; count <= rows.size(); ++count) {
        std::vector<std::size_t> selected;
        bool found = false;
        std::function<void(std::size_t)> search = [&](std::size_t start) {
            if (found) return;
            if (selected.size() == count) {
                std::vector<CanonicalHardRow> subset;
                for (std::size_t index : selected) subset.push_back(rows[index]);
                if (exact_hard_polytope_detail::feasible(
                        subset, owner, input_half_box)) return;
                for (std::size_t remove = 0; remove < subset.size(); ++remove) {
                    auto reduced = subset;
                    reduced.erase(reduced.begin() +
                        static_cast<std::ptrdiff_t>(remove));
                    if (!exact_hard_polytope_detail::feasible(
                            reduced, owner, input_half_box)) return;
                }
                result.minimal_conflict_row_ids.clear();
                if (subset.size() == 1)
                    result.minimal_conflict_row_ids.push_back("$input_box");
                for (const auto& row : subset)
                    result.minimal_conflict_row_ids.push_back(row.id);
                found = true;
                return;
            }
            for (std::size_t i = start;
                 i + (count - selected.size()) <= rows.size(); ++i) {
                selected.push_back(i);
                search(i + 1);
                selected.pop_back();
                if (found) return;
            }
        };
        search(0);
        if (found) break;
    }
    return result;
}

}  // namespace gf
