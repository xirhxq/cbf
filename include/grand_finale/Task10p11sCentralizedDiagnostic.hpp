#pragma once

#include "grand_finale/CanonicalHardRows.hpp"
#include "json.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gf {

struct Task10p11sFullPairRow {
    std::string id;
    NodeId owner = 0;
    NodeId peer = 0;
    Eigen::Vector2d owner_coefficient = Eigen::Vector2d::Zero();
    Eigen::Vector2d peer_coefficient = Eigen::Vector2d::Zero();
    double constant = 0.0;

    double margin(const Eigen::Vector2d& owner_control,
                  const Eigen::Vector2d& peer_control) const {
        return owner_coefficient.dot(owner_control) +
            peer_coefficient.dot(peer_control) + constant;
    }
};

inline std::optional<Task10p11sFullPairRow>
reconstructTask10p11sFullPairRow(const CanonicalHardRow& owner_half) {
    if (!owner_half.peer.has_value() ||
        std::abs(owner_half.responsibility - 0.5) > 1e-12 ||
        !owner_half.control_coefficient.allFinite() ||
        !std::isfinite(owner_half.constant) ||
        !std::isfinite(owner_half.coefficient_uncertainty_reserve) ||
        owner_half.coefficient_uncertainty_reserve < 0.0) {
        return std::nullopt;
    }
    return Task10p11sFullPairRow{
        owner_half.id + ":once-reserve-full-pair",
        owner_half.owner,
        *owner_half.peer,
        owner_half.control_coefficient,
        -owner_half.control_coefficient,
        2.0 * owner_half.constant +
            owner_half.coefficient_uncertainty_reserve};
}

struct Task10p11sFrozenSnapshot {
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::map<NodeId, std::vector<CanonicalHardRow>> owner_rows;
    std::map<NodeId, PairwiseSecondOrderState2D> mobile_states;
    std::map<NodeId, PairwiseSecondOrderState2D> fixed_states;
    std::map<NodeId, Eigen::Vector2d> nominal_controls;
    std::vector<DirectedEdge> fixed_topology;
    std::map<std::string, PairwiseSnapshotTube> pair_tubes;
    double input_half_box_mps2 = 0.0;
    double plant_speed_limit_mps = 0.0;
    std::size_t plant_speed_facet_count = 0;
    double plant_speed_dt_s = 0.0;
    std::string relaxed_full_pair_row_id;
    bool coherent_peer_half_verified = false;
    std::string snapshot_digest;
    std::string config_digest;
};

struct Task10p11sGateAResult {
    bool snapshot_complete = false;
    std::string reason;
    bool same_half_decidable = false;
    bool same_half_feasible = false;
    NodeId limiting_owner = 0;
    double limiting_gamma_mps2 =
        -std::numeric_limits<double>::infinity();
    bool full_pair_relaxation_decidable = false;
    bool full_pair_relaxation_feasible = false;
    std::string full_pair_status = "full_pair_not_audited";
    bool full_pair_complete_28d_decidable = false;
    bool successor_audit_authorized = false;
    bool gate_b_authorized = false;
};

inline Task10p11sGateAResult auditTask10p11sGateA(
    const Task10p11sFrozenSnapshot& request) {
    Task10p11sGateAResult result;

    bool observed_infeasible_owner = false;
    std::set<NodeId> audited_owners;
    for (const auto& [owner, rows] : request.owner_rows) {
        if (rows.empty()) continue;
        const auto gamma = solveCanonicalGammaStar(
            rows, owner, request.input_half_box_mps2);
        if (!gamma.valid || !std::isfinite(gamma.gamma)) continue;
        audited_owners.insert(owner);
        if (result.limiting_owner == 0 ||
            gamma.gamma < result.limiting_gamma_mps2) {
            result.limiting_owner = owner;
            result.limiting_gamma_mps2 = gamma.gamma;
        }
        observed_infeasible_owner = observed_infeasible_owner || gamma.gamma < 0.0;
    }

    std::set<NodeId> unique_mobile_ids(
        request.mobile_ids.begin(), request.mobile_ids.end());
    std::set<NodeId> unique_fixed_ids(
        request.fixed_ids.begin(), request.fixed_ids.end());
    const std::set<NodeId> canonical_mobile_ids = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    const std::set<NodeId> canonical_fixed_ids = {100, 101, 102};
    const bool valid_mobile_ids = request.mobile_ids.size() == 14 &&
        unique_mobile_ids == canonical_mobile_ids;
    const bool valid_fixed_ids = request.fixed_ids.size() == 3 &&
        unique_fixed_ids == canonical_fixed_ids;

    if (observed_infeasible_owner) {
        result.same_half_decidable = true;
        result.same_half_feasible = false;
    } else if (valid_mobile_ids && audited_owners == unique_mobile_ids) {
        result.same_half_decidable = true;
        result.same_half_feasible = true;
    }

    if (!request.relaxed_full_pair_row_id.empty()) {
        for (const auto& [owner, rows] : request.owner_rows) {
            const auto selected = std::find_if(
                rows.begin(), rows.end(), [&](const CanonicalHardRow& row) {
                    return row.id == request.relaxed_full_pair_row_id;
                });
            if (selected == rows.end()) continue;
            const auto coupled = reconstructTask10p11sFullPairRow(*selected);
            if (!coupled.has_value()) break;

            std::vector<CanonicalHardRow> relaxed_rows;
            relaxed_rows.reserve(rows.size());
            for (const CanonicalHardRow& row : rows) {
                if (row.id != selected->id) relaxed_rows.push_back(row);
            }
            CanonicalHardRow relaxed = *selected;
            relaxed.id = coupled->id + ":peer-input-box-relaxation";
            relaxed.constant = coupled->constant +
                request.input_half_box_mps2 *
                coupled->peer_coefficient.cwiseAbs().sum();
            relaxed.responsibility = 1.0;
            relaxed.coefficient_uncertainty_reserve = 0.0;
            relaxed_rows.push_back(std::move(relaxed));

            const auto gamma = solveCanonicalGammaStar(
                relaxed_rows, owner, request.input_half_box_mps2);
            if (!gamma.valid || !std::isfinite(gamma.gamma)) break;
            result.full_pair_relaxation_decidable = true;
            result.full_pair_relaxation_feasible = gamma.gamma >= 0.0;
            if (result.full_pair_relaxation_feasible) {
                result.full_pair_status =
                    "relaxed_full_pair_feasible_full_28d_undetermined";
            } else {
                if (request.coherent_peer_half_verified) {
                    result.full_pair_status =
                        "relaxed_full_pair_infeasible_implies_full_28d_infeasible";
                    result.full_pair_complete_28d_decidable = true;
                } else {
                    result.full_pair_status =
                        "one_sided_relaxation_infeasible_coherence_unverified";
                }
            }
            break;
        }
    }

    const auto finite_state = [](const PairwiseSecondOrderState2D& state) {
        return std::isfinite(state.position.x) &&
            std::isfinite(state.position.y) && state.velocity.allFinite() &&
            state.acceleration.allFinite();
    };
    std::set<std::pair<NodeId, NodeId>> unique_topology_edges;
    bool topology_endpoints_valid = true;
    for (const DirectedEdge& edge : request.fixed_topology) {
        topology_endpoints_valid = topology_endpoints_valid &&
            canonical_mobile_ids.count(edge.owner) != 0 &&
            (canonical_mobile_ids.count(edge.reference) != 0 ||
             canonical_fixed_ids.count(edge.reference) != 0);
        unique_topology_edges.emplace(edge.reference, edge.owner);
    }

    bool has_all_owner_fields = valid_mobile_ids && valid_fixed_ids &&
        topology_endpoints_valid &&
        unique_topology_edges.size() == request.fixed_topology.size();
    for (NodeId owner : request.mobile_ids) {
        has_all_owner_fields = has_all_owner_fields &&
            request.owner_rows.count(owner) != 0 &&
            !request.owner_rows.at(owner).empty() &&
            request.mobile_states.count(owner) != 0 &&
            finite_state(request.mobile_states.at(owner)) &&
            request.nominal_controls.count(owner) != 0 &&
            request.nominal_controls.at(owner).allFinite();
        bool has_collision_row = false;
        std::set<NodeId> reference_row_peers;
        if (request.owner_rows.count(owner) != 0) {
            for (const CanonicalHardRow& row : request.owner_rows.at(owner)) {
                const bool pair_kind =
                    row.kind == CanonicalHardRowKind::ReferenceDistance ||
                    row.kind == CanonicalHardRowKind::Collision;
                const bool valid_pair_peer = row.peer.has_value() &&
                    *row.peer != owner &&
                    (canonical_mobile_ids.count(*row.peer) != 0 ||
                     canonical_fixed_ids.count(*row.peer) != 0);
                has_collision_row = has_collision_row ||
                    (row.kind == CanonicalHardRowKind::Collision &&
                     valid_pair_peer);
                has_all_owner_fields = has_all_owner_fields &&
                    row.owner == owner && !row.id.empty() &&
                    row.control_coefficient.allFinite() &&
                    std::isfinite(row.constant) &&
                    std::isfinite(row.responsibility) &&
                    std::isfinite(row.coefficient_uncertainty_reserve) &&
                    row.coefficient_uncertainty_reserve >= 0.0 &&
                    (!pair_kind || valid_pair_peer);
                if (pair_kind && valid_pair_peer) {
                    const auto tube = request.pair_tubes.find(row.id);
                    const bool valid_tube = tube != request.pair_tubes.end() &&
                        std::isfinite(tube->second.position_radius_m) &&
                        tube->second.position_radius_m >= 0.0 &&
                        std::isfinite(tube->second.velocity_radius_mps) &&
                        tube->second.velocity_radius_mps >= 0.0 &&
                        tube->second.provenance ==
                            SnapshotTubeProvenance::ExternallyCertified;
                    has_all_owner_fields = has_all_owner_fields && valid_tube;
                    if (row.kind == CanonicalHardRowKind::ReferenceDistance) {
                        reference_row_peers.insert(*row.peer);
                    }
                }
                if (pair_kind && valid_pair_peer &&
                    canonical_mobile_ids.count(*row.peer) != 0) {
                    has_all_owner_fields = has_all_owner_fields &&
                        std::abs(row.responsibility - 0.5) <= 1e-12;
                    const auto peer_rows = request.owner_rows.find(*row.peer);
                    const bool coherent_peer = peer_rows != request.owner_rows.end() &&
                        std::any_of(peer_rows->second.begin(), peer_rows->second.end(),
                            [&](const CanonicalHardRow& peer_row) {
                                return peer_row.peer == std::optional<NodeId>{owner} &&
                                    peer_row.kind == row.kind &&
                                    std::abs(peer_row.responsibility - 0.5) <= 1e-12 &&
                                    (peer_row.control_coefficient +
                                     row.control_coefficient).norm() <= 1e-9 &&
                                    std::abs(
                                        2.0 * (peer_row.constant +
                                               peer_row.coefficient_uncertainty_reserve) -
                                        2.0 * (row.constant +
                                               row.coefficient_uncertainty_reserve)) <= 1e-8;
                            });
                    has_all_owner_fields = has_all_owner_fields && coherent_peer;
                }
                if (pair_kind && valid_pair_peer &&
                    canonical_fixed_ids.count(*row.peer) != 0) {
                    has_all_owner_fields = has_all_owner_fields &&
                        std::abs(row.responsibility - 1.0) <= 1e-12;
                }
            }
        }
        std::set<NodeId> incoming_references;
        for (const DirectedEdge& edge : request.fixed_topology) {
            if (edge.owner == owner) incoming_references.insert(edge.reference);
        }
        has_all_owner_fields = has_all_owner_fields &&
            has_collision_row && incoming_references.size() >= 2 &&
            reference_row_peers == incoming_references;
    }
    for (NodeId fixed : request.fixed_ids) {
        has_all_owner_fields = has_all_owner_fields &&
            request.fixed_states.count(fixed) != 0 &&
            finite_state(request.fixed_states.at(fixed)) &&
            request.fixed_states.at(fixed).velocity.norm() <= 1e-12 &&
            request.fixed_states.at(fixed).acceleration.norm() <= 1e-12;
    }
    result.snapshot_complete = has_all_owner_fields &&
        !request.snapshot_digest.empty() && !request.config_digest.empty() &&
        std::isfinite(request.input_half_box_mps2) &&
        request.input_half_box_mps2 > 0.0 &&
        std::isfinite(request.plant_speed_limit_mps) &&
        request.plant_speed_limit_mps > 0.0 &&
        request.plant_speed_facet_count > 0 &&
        std::isfinite(request.plant_speed_dt_s) && request.plant_speed_dt_s > 0.0;

    if (!result.snapshot_complete) {
        result.reason = "frozen_snapshot_incomplete_for_full_28d";
        return result;
    }

    result.reason = "full_28d_audit_not_implemented_after_completeness_gate";
    return result;
}

inline CanonicalHardRowKind task10p11sRowKind(const std::string& kind) {
    if (kind == "reference") return CanonicalHardRowKind::ReferenceDistance;
    if (kind == "collision") return CanonicalHardRowKind::Collision;
    if (kind == "workspace") return CanonicalHardRowKind::Workspace;
    if (kind == "plant_speed_applied_control") {
        return CanonicalHardRowKind::PlantSpeedAppliedControl;
    }
    if (kind == "input_box") return CanonicalHardRowKind::InputBox;
    throw std::invalid_argument("unknown frozen hard-row kind: " + kind);
}

inline nlohmann::json auditTask10p11sEvidenceJson(
    const nlohmann::json& evidence,
    const std::string& snapshot_digest,
    const std::string& config_digest) {
    const auto incomplete = [&]() {
        return nlohmann::json{
            {"protocol", "task10p11s-gate-a-v1"},
            {"reason", "frozen_snapshot_incomplete_for_full_28d"},
            {"snapshot_complete", false},
            {"same_half_feasible", false},
            {"full_pair_status", "full_pair_not_audited"},
            {"successor_audit_authorized", false},
            {"gate_b_authorized", false}};
    };
    if (!evidence.contains("timeline") || evidence.at("timeline").empty() ||
        !evidence.at("timeline").back().contains("hard_polytope") ||
        !evidence.at("timeline").back().at("hard_polytope").contains("owner") ||
        !evidence.at("timeline").back().at("hard_polytope").contains(
            "input_half_box_mps2") ||
        !evidence.at("timeline").back().at("hard_polytope").contains("rows")) {
        return incomplete();
    }

    try {
        Task10p11sFrozenSnapshot request;
        request.snapshot_digest = snapshot_digest;
        request.config_digest = config_digest;
        const auto& hard = evidence.at("timeline").back().at("hard_polytope");
        const NodeId owner = hard.at("owner").get<NodeId>();
        request.input_half_box_mps2 =
            hard.at("input_half_box_mps2").get<double>();
        std::vector<CanonicalHardRow> rows;
        for (const auto& value : hard.at("rows")) {
            if (!value.contains("id") || !value.contains("kind") ||
                !value.contains("normal") || !value.contains("constant")) {
                return incomplete();
            }
        CanonicalHardRow row;
        row.id = value.at("id").get<std::string>();
        row.kind = task10p11sRowKind(value.at("kind").get<std::string>());
        row.owner = owner;
        if (value.contains("peer") && !value.at("peer").is_null()) {
            row.peer = value.at("peer").get<NodeId>();
        }
        const auto normal = value.at("normal");
        row.normal = {normal.at(0).get<double>(), normal.at(1).get<double>()};
        row.control_coefficient = row.normal;
        row.constant = value.at("constant").get<double>();
        row.coefficient_uncertainty_reserve =
            value.value("coefficient_reserve_mps2", 0.0);
        const bool mobile_pair = row.peer.has_value() && *row.peer >= 1 &&
            *row.peer <= 14 &&
            (row.kind == CanonicalHardRowKind::ReferenceDistance ||
             row.kind == CanonicalHardRowKind::Collision);
        row.responsibility = mobile_pair ? 0.5 : 1.0;
        rows.push_back(std::move(row));
        }
        request.owner_rows.emplace(owner, std::move(rows));

        if (evidence.contains("failure_minimal_conflict_row_ids")) {
            for (const auto& id_value :
                 evidence.at("failure_minimal_conflict_row_ids")) {
            const std::string id = id_value.get<std::string>();
            const auto& owner_rows = request.owner_rows.at(owner);
            const auto row = std::find_if(
                owner_rows.begin(), owner_rows.end(),
                [&](const CanonicalHardRow& candidate) {
                    return candidate.id == id &&
                        std::abs(candidate.responsibility - 0.5) <= 1e-12;
                });
            if (row != owner_rows.end()) {
                request.relaxed_full_pair_row_id = id;
                break;
            }
            }
        }

        const auto result = auditTask10p11sGateA(request);
        return nlohmann::json{
        {"protocol", "task10p11s-gate-a-v1"},
        {"reason", result.reason},
        {"snapshot_complete", result.snapshot_complete},
        {"same_half_decidable", result.same_half_decidable},
        {"same_half_feasible", result.same_half_feasible},
        {"limiting_owner", result.limiting_owner},
        {"limiting_gamma_mps2", result.limiting_gamma_mps2},
        {"full_pair_relaxation_decidable",
         result.full_pair_relaxation_decidable},
        {"full_pair_relaxation_feasible",
         result.full_pair_relaxation_feasible},
        {"full_pair_status", result.full_pair_status},
        {"full_pair_complete_28d_decidable",
         result.full_pair_complete_28d_decidable},
        {"successor_audit_authorized", result.successor_audit_authorized},
        {"gate_b_authorized", result.gate_b_authorized},
        {"snapshot_digest", snapshot_digest},
        {"config_digest", config_digest}};
    } catch (const std::exception&) {
        nlohmann::json failure = incomplete();
        failure["reason"] = "schema_integrity_failure";
        return failure;
    }
}

inline bool task10p11sValidProvenance(const nlohmann::json& metadata) {
    static const std::vector<std::string> string_fields = {
        "parent_commit", "parent_tree", "parent_gitlink", "parent_branch",
        "parent_upstream", "parent_worktree_status", "cbf_commit", "cbf_tree",
        "cbf_branch", "cbf_upstream", "cbf_worktree_status", "binary_sha256",
        "solver_profile", "solver_version", "snapshot_input_sha256",
        "config_digest"};
    for (const std::string& field : string_fields) {
        if (!metadata.contains(field) || !metadata.at(field).is_string() ||
            metadata.at(field).get<std::string>().empty()) {
            return false;
        }
    }
    if (!metadata.contains("independent_verifier") ||
        !metadata.at("independent_verifier").is_object()) {
        return false;
    }
    const auto& verifier = metadata.at("independent_verifier");
    static const std::vector<std::string> verifier_fields = {
        "result", "source_sha256", "tool_version", "output_path"};
    for (const std::string& field : verifier_fields) {
        if (!verifier.contains(field) || !verifier.at(field).is_string() ||
            verifier.at(field).get<std::string>().empty()) {
            return false;
        }
    }
    if (verifier.at("result").get<std::string>() != "pending") {
        return false;
    }
    return metadata.contains("source_file_sha256") &&
        metadata.at("source_file_sha256").is_object() &&
        !metadata.at("source_file_sha256").empty() &&
        metadata.contains("seed");
}

}  // namespace gf
