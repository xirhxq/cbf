#pragma once

#include "bridge/ExactGammaStar2D.hpp"
#include "cbf/PairwiseSecondOrderCBF.hpp"
#include "grand_finale/Types.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gf {

enum class CanonicalHardRowKind {
    ReferenceDistance,
    Collision,
    InputBox,
    Auxiliary
};

struct CanonicalHardRow {
    std::string id;
    CanonicalHardRowKind kind = CanonicalHardRowKind::Auxiliary;
    NodeId owner = 0;
    std::optional<NodeId> peer;
    Eigen::Vector2d normal = Eigen::Vector2d::Zero();
    Eigen::Vector2d control_coefficient = Eigen::Vector2d::Zero();
    double constant = 0.0;
    double responsibility = 1.0;
    bool participates_in_gamma = true;

    double margin(const Eigen::Vector2d& control) const {
        return control_coefficient.dot(control) + constant;
    }
};

struct CanonicalHardRowRequest {
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::map<NodeId, PairwiseSecondOrderState2D> states;
    std::vector<DirectedEdge> reference_edges;
    std::vector<UndirectedEdge> collision_pairs;
    PairwiseSecondOrderRowSpec reference_spec;
    PairwiseSecondOrderRowSpec collision_spec;
    double acceleration_half_box = 0.0;
};

inline CanonicalHardRow makeCanonicalGammaRow(
    std::string id,
    NodeId owner,
    Eigen::Vector2d control_coefficient,
    double constant) {
    if (id.empty() || !control_coefficient.allFinite() ||
        !std::isfinite(constant)) {
        throw std::invalid_argument("canonical gamma row must be finite and named");
    }
    return CanonicalHardRow{
        std::move(id), CanonicalHardRowKind::Auxiliary, owner, std::nullopt,
        control_coefficient, control_coefficient, constant, 1.0, true};
}

namespace canonical_hard_row_detail {

inline void canonicalizeNodes(std::vector<NodeId>& ids, const char* kind) {
    std::sort(ids.begin(), ids.end());
    if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
        throw std::invalid_argument(std::string("duplicate ") + kind + " node");
    }
}

inline CanonicalHardRow physicalRow(
    std::string id,
    CanonicalHardRowKind kind,
    NodeId owner,
    NodeId peer,
    const Eigen::Vector2d& normal,
    const Eigen::Vector2d& coefficient,
    double central_constant,
    double responsibility) {
    return CanonicalHardRow{
        std::move(id), kind, owner, peer, normal, coefficient,
        responsibility * central_constant, responsibility, true};
}

inline void appendSharedPairRows(
    std::vector<CanonicalHardRow>& rows,
    const std::string& prefix,
    CanonicalHardRowKind kind,
    NodeId first,
    NodeId second,
    const PairwiseSecondOrderState2D& first_state,
    const PairwiseSecondOrderState2D& second_state,
    const PairwiseSecondOrderRowSpec& spec) {
    PairwiseSecondOrderState2D zero_first = first_state;
    PairwiseSecondOrderState2D zero_second = second_state;
    zero_first.acceleration.setZero();
    zero_second.acceleration.setZero();
    const auto central = buildPairwiseSecondOrderRow(
        zero_first, zero_second, spec);
    const auto kinematics = computePairwiseDistanceKinematics(
        zero_first.position, zero_second.position,
        zero_first.velocity, zero_second.velocity);
    rows.push_back(physicalRow(
        prefix + ":owner:" + std::to_string(first), kind,
        first, second, kinematics.normal, central.uCoe,
        central.constTerm, 0.5));
    rows.push_back(physicalRow(
        prefix + ":owner:" + std::to_string(second), kind,
        second, first, -kinematics.normal, -central.uCoe,
        central.constTerm, 0.5));
}

inline void appendFixedPairRow(
    std::vector<CanonicalHardRow>& rows,
    const std::string& prefix,
    CanonicalHardRowKind kind,
    NodeId mobile,
    NodeId fixed,
    const PairwiseSecondOrderState2D& mobile_state,
    const PairwiseSecondOrderState2D& fixed_state,
    const PairwiseSecondOrderRowSpec& spec) {
    PairwiseSecondOrderState2D zero_mobile = mobile_state;
    PairwiseSecondOrderState2D zero_fixed = fixed_state;
    zero_mobile.acceleration.setZero();
    zero_fixed.acceleration.setZero();
    const auto central = buildPairwiseSecondOrderRow(
        zero_mobile, zero_fixed, spec);
    const auto kinematics = computePairwiseDistanceKinematics(
        zero_mobile.position, zero_fixed.position,
        zero_mobile.velocity, zero_fixed.velocity);
    rows.push_back(physicalRow(
        prefix + ":owner:" + std::to_string(mobile), kind,
        mobile, fixed, kinematics.normal, central.uCoe,
        central.constTerm, 1.0));
}

}  // namespace canonical_hard_row_detail

inline std::vector<CanonicalHardRow> buildCanonicalHardRows(
    CanonicalHardRowRequest request) {
    using namespace canonical_hard_row_detail;
    canonicalizeNodes(request.mobile_ids, "mobile");
    canonicalizeNodes(request.fixed_ids, "fixed");
    if (!std::isfinite(request.acceleration_half_box) ||
        request.acceleration_half_box <= 0.0) {
        throw std::invalid_argument("acceleration half box must be positive");
    }
    const std::set<NodeId> mobiles(
        request.mobile_ids.begin(), request.mobile_ids.end());
    const std::set<NodeId> fixed(request.fixed_ids.begin(), request.fixed_ids.end());
    std::set<NodeId> known = mobiles;
    for (NodeId id : fixed) {
        if (!known.insert(id).second)
            throw std::invalid_argument("mobile and fixed nodes must be disjoint");
    }
    for (NodeId id : known) {
        if (request.states.count(id) == 0)
            throw std::invalid_argument("missing pairwise state");
    }
    for (NodeId id : fixed) {
        const auto& state = request.states.at(id);
        if (state.velocity.norm() != 0.0 || state.acceleration.norm() != 0.0)
            throw std::invalid_argument("fixed node kinematics must be zero");
    }
    if (request.reference_spec.kind !=
            PairwiseSecondOrderBarrierKind::CommunicationUpper ||
        request.collision_spec.kind !=
            PairwiseSecondOrderBarrierKind::CollisionLower) {
        throw std::invalid_argument("canonical hard-row barrier kinds are fixed");
    }

    std::vector<CanonicalHardRow> rows;
    std::set<std::string> reference_ids;
    for (const DirectedEdge& edge : request.reference_edges) {
        if (mobiles.count(edge.owner) == 0 || known.count(edge.reference) == 0 ||
            !reference_ids.insert(edge.id()).second) {
            throw std::invalid_argument("invalid or duplicate reference edge");
        }
        const std::string prefix = "reference:" + edge.id();
        if (mobiles.count(edge.reference) != 0) {
            appendSharedPairRows(
                rows, prefix, CanonicalHardRowKind::ReferenceDistance,
                edge.owner, edge.reference, request.states.at(edge.owner),
                request.states.at(edge.reference), request.reference_spec);
        } else {
            appendFixedPairRow(
                rows, prefix, CanonicalHardRowKind::ReferenceDistance,
                edge.owner, edge.reference, request.states.at(edge.owner),
                request.states.at(edge.reference), request.reference_spec);
        }
    }

    std::set<std::string> collision_ids;
    for (const UndirectedEdge& raw_edge : request.collision_pairs) {
        const UndirectedEdge edge = UndirectedEdge::canonical(
            raw_edge.first, raw_edge.second);
        if (known.count(edge.first) == 0 || known.count(edge.second) == 0 ||
            (mobiles.count(edge.first) == 0 && mobiles.count(edge.second) == 0) ||
            !collision_ids.insert(edge.id()).second) {
            throw std::invalid_argument("invalid or duplicate collision pair");
        }
        const std::string prefix = "collision:" + edge.id();
        if (mobiles.count(edge.first) != 0 && mobiles.count(edge.second) != 0) {
            appendSharedPairRows(
                rows, prefix, CanonicalHardRowKind::Collision,
                edge.first, edge.second, request.states.at(edge.first),
                request.states.at(edge.second), request.collision_spec);
        } else {
            const NodeId mobile = mobiles.count(edge.first) ? edge.first : edge.second;
            const NodeId anchor = mobile == edge.first ? edge.second : edge.first;
            appendFixedPairRow(
                rows, prefix, CanonicalHardRowKind::Collision,
                mobile, anchor, request.states.at(mobile),
                request.states.at(anchor), request.collision_spec);
        }
    }

    for (NodeId owner : request.mobile_ids) {
        const double bound = request.acceleration_half_box;
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ax:lower",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            Eigen::Vector2d::UnitX(), Eigen::Vector2d::UnitX(),
            bound, 1.0, false});
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ax:upper",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            -Eigen::Vector2d::UnitX(), -Eigen::Vector2d::UnitX(),
            bound, 1.0, false});
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ay:lower",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            Eigen::Vector2d::UnitY(), Eigen::Vector2d::UnitY(),
            bound, 1.0, false});
        rows.push_back(CanonicalHardRow{
            "input:" + std::to_string(owner) + ":ay:upper",
            CanonicalHardRowKind::InputBox, owner, std::nullopt,
            -Eigen::Vector2d::UnitY(), -Eigen::Vector2d::UnitY(),
            bound, 1.0, false});
    }
    std::sort(rows.begin(), rows.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.id < rhs.id;
    });
    if (std::adjacent_find(rows.begin(), rows.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.id == rhs.id;
        }) != rows.end()) {
        throw std::invalid_argument("duplicate canonical hard-row id");
    }
    return rows;
}

inline BridgeGammaStarSolution2D solveCanonicalGammaStar(
    const std::vector<CanonicalHardRow>& rows,
    NodeId owner,
    double acceleration_half_box) {
    std::vector<BridgeGammaStarResidual2D> residuals;
    for (const CanonicalHardRow& row : rows) {
        if (row.owner != owner || !row.participates_in_gamma) continue;
        residuals.push_back(bridgeGammaStarResidualFromAffineMargin(
            row.control_coefficient.x(), row.control_coefficient.y(),
            row.constant));
    }
    return solveExactBridgeGammaStar2D(residuals, acceleration_half_box);
}

}  // namespace gf
