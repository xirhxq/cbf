#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11sCentralizedDiagnostic.hpp"
#include "json.hpp"

namespace {

gf::Task10p11sFrozenSnapshot completeSnapshotFixture() {
    gf::Task10p11sFrozenSnapshot request;
    request.input_half_box_mps2 = 4.0;
    request.plant_speed_limit_mps = 30.0;
    request.plant_speed_facet_count = 64;
    request.plant_speed_dt_s = 0.1;
    request.snapshot_digest = "snapshot";
    request.config_digest = "config";
    request.fixed_ids = {100, 101, 102};
    for (gf::NodeId fixed : request.fixed_ids) {
        request.fixed_states.emplace(fixed, PairwiseSecondOrderState2D{});
    }
    for (gf::NodeId owner = 1; owner <= 14; ++owner) {
        request.mobile_ids.push_back(owner);
        request.mobile_states.emplace(owner, PairwiseSecondOrderState2D{});
        request.nominal_controls.emplace(owner, Eigen::Vector2d::Zero());
        request.fixed_topology.emplace_back(100, owner);
        request.fixed_topology.emplace_back(101, owner);

        std::vector<gf::CanonicalHardRow> rows;
        for (gf::NodeId fixed : {gf::NodeId{100}, gf::NodeId{101}}) {
            gf::CanonicalHardRow reference;
            reference.id = "reference:" + std::to_string(fixed) + "->" +
                std::to_string(owner) + ":owner:" + std::to_string(owner);
            reference.kind = gf::CanonicalHardRowKind::ReferenceDistance;
            reference.owner = owner;
            reference.peer = fixed;
            reference.control_coefficient = {1.0, 0.0};
            reference.constant = 8.0;
            reference.responsibility = 1.0;
            rows.push_back(reference);
            request.pair_tubes.emplace(
                reference.id,
                gf::PairwiseSnapshotTube{
                    0.1, 0.1, gf::SnapshotTubeProvenance::ExternallyCertified});
        }

        const gf::NodeId peer = owner % 2 == 0 ? owner - 1 : owner + 1;
        gf::CanonicalHardRow collision;
        collision.id = "collision:" + std::to_string(std::min(owner, peer)) +
            "--" + std::to_string(std::max(owner, peer)) + ":owner:" +
            std::to_string(owner);
        collision.kind = gf::CanonicalHardRowKind::Collision;
        collision.owner = owner;
        collision.peer = peer;
        collision.control_coefficient = owner < peer
            ? Eigen::Vector2d{1.0, 0.0} : Eigen::Vector2d{-1.0, 0.0};
        collision.constant = 8.0;
        collision.responsibility = 0.5;
        rows.push_back(collision);
        request.pair_tubes.emplace(
            collision.id,
            gf::PairwiseSnapshotTube{
                0.1, 0.1, gf::SnapshotTubeProvenance::ExternallyCertified});
        request.owner_rows.emplace(owner, std::move(rows));
    }
    return request;
}

}  // namespace

TEST_CASE("incomplete frozen snapshot preserves strict same-half infeasibility") {
    gf::Task10p11sFrozenSnapshot request;
    request.input_half_box_mps2 = 4.0;

    gf::CanonicalHardRow lower;
    lower.id = "reference:101->2:owner:2";
    lower.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    lower.owner = 2;
    lower.peer = 101;
    lower.control_coefficient = {1.0, 0.0};
    lower.constant = -5.0;

    gf::CanonicalHardRow upper = lower;
    upper.id = "reference:2->4:owner:2";
    upper.peer = 4;
    upper.control_coefficient = {-1.0, 0.0};
    request.owner_rows.emplace(2, std::vector<gf::CanonicalHardRow>{lower, upper});

    const auto result = gf::auditTask10p11sGateA(request);

    CHECK_FALSE(result.snapshot_complete);
    CHECK(result.reason == "frozen_snapshot_incomplete_for_full_28d");
    CHECK(result.same_half_decidable);
    CHECK_FALSE(result.same_half_feasible);
    CHECK(result.limiting_owner == 2);
    CHECK_FALSE(result.full_pair_complete_28d_decidable);
    CHECK_FALSE(result.successor_audit_authorized);
    CHECK_FALSE(result.gate_b_authorized);
}

TEST_CASE("full-pair reconstruction deducts coefficient reserve exactly once") {
    gf::CanonicalHardRow owner_half;
    owner_half.id = "reference:2->4:owner:2";
    owner_half.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    owner_half.owner = 2;
    owner_half.peer = 4;
    owner_half.control_coefficient = {-1.0, 0.0};
    owner_half.constant = 1.5;
    owner_half.responsibility = 0.5;
    owner_half.coefficient_uncertainty_reserve = 0.25;

    const auto coupled = gf::reconstructTask10p11sFullPairRow(owner_half);

    REQUIRE(coupled.has_value());
    CHECK(coupled->owner == 2);
    CHECK(coupled->peer == 4);
    CHECK(coupled->owner_coefficient.x() == doctest::Approx(-1.0));
    CHECK(coupled->peer_coefficient.x() == doctest::Approx(1.0));
    CHECK(coupled->constant == doctest::Approx(3.25));
    CHECK(coupled->margin({2.0, 0.0}, {-1.0, 0.0}) ==
          doctest::Approx(0.25));

    gf::CanonicalHardRow fixed_full = owner_half;
    fixed_full.id = "reference:101->2:owner:2";
    fixed_full.peer = 101;
    fixed_full.responsibility = 1.0;
    CHECK_FALSE(gf::reconstructTask10p11sFullPairRow(fixed_full).has_value());
}

TEST_CASE("relaxed full-pair feasibility does not authorize complete 28D or successor") {
    gf::Task10p11sFrozenSnapshot request;
    request.input_half_box_mps2 = 4.0;
    request.relaxed_full_pair_row_id = "reference:2->4:owner:2";

    gf::CanonicalHardRow fixed;
    fixed.id = "reference:101->2:owner:2";
    fixed.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    fixed.owner = 2;
    fixed.peer = 101;
    fixed.control_coefficient = {1.0, 0.0};
    fixed.constant = -3.0;

    gf::CanonicalHardRow mobile = fixed;
    mobile.id = request.relaxed_full_pair_row_id;
    mobile.peer = 4;
    mobile.control_coefficient = {-1.0, 0.0};
    mobile.constant = 1.0;
    mobile.responsibility = 0.5;
    mobile.coefficient_uncertainty_reserve = 0.25;
    request.owner_rows.emplace(
        2, std::vector<gf::CanonicalHardRow>{fixed, mobile});

    const auto result = gf::auditTask10p11sGateA(request);

    CHECK_FALSE(result.snapshot_complete);
    CHECK_FALSE(result.same_half_feasible);
    CHECK(result.full_pair_relaxation_decidable);
    CHECK(result.full_pair_relaxation_feasible);
    CHECK(result.full_pair_status ==
          "relaxed_full_pair_feasible_full_28d_undetermined");
    CHECK_FALSE(result.full_pair_complete_28d_decidable);
    CHECK_FALSE(result.successor_audit_authorized);
    CHECK_FALSE(result.gate_b_authorized);
}

TEST_CASE("frozen evidence JSON cannot authorize Gate B without full snapshot") {
    const nlohmann::json evidence = {
        {"failure_minimal_conflict_row_ids",
         {"reference:101->2:owner:2", "reference:2->4:owner:2"}},
        {"timeline", {{{"hard_polytope", {
            {"owner", 2},
            {"input_half_box_mps2", 4.0},
            {"rows", {
                {{"id", "reference:101->2:owner:2"},
                 {"kind", "reference"}, {"peer", 101},
                 {"normal", {1.0, 0.0}}, {"constant", -3.0},
                 {"coefficient_reserve_mps2", 0.0}},
                {{"id", "reference:2->4:owner:2"},
                 {"kind", "reference"}, {"peer", 4},
                 {"normal", {-1.0, 0.0}}, {"constant", 1.0},
                 {"coefficient_reserve_mps2", 0.25}}
            }}
        }}}}}
    };

    const auto output = gf::auditTask10p11sEvidenceJson(
        evidence, "snapshot-sha256", "");

    CHECK(output.at("reason") ==
          "frozen_snapshot_incomplete_for_full_28d");
    CHECK(output.at("snapshot_complete") == false);
    CHECK(output.at("same_half_feasible") == false);
    CHECK(output.at("full_pair_status") ==
          "relaxed_full_pair_feasible_full_28d_undetermined");
    CHECK(output.at("successor_audit_authorized") == false);
    CHECK(output.at("gate_b_authorized") == false);
}

TEST_CASE("partial feasible owner ledger is not a decidable same-half result") {
    gf::Task10p11sFrozenSnapshot request;
    request.input_half_box_mps2 = 4.0;
    gf::CanonicalHardRow row;
    row.id = "reference:101->2:owner:2";
    row.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    row.owner = 2;
    row.peer = 101;
    row.control_coefficient = {1.0, 0.0};
    row.constant = 1.0;
    request.owner_rows.emplace(2, std::vector<gf::CanonicalHardRow>{row});

    const auto result = gf::auditTask10p11sGateA(request);
    CHECK_FALSE(result.snapshot_complete);
    CHECK_FALSE(result.same_half_decidable);
    CHECK_FALSE(result.same_half_feasible);
}

TEST_CASE("duplicate mobile identifiers cannot satisfy snapshot completeness") {
    gf::Task10p11sFrozenSnapshot request;
    request.mobile_ids.assign(14, 1);
    request.fixed_ids = {100, 101, 102};
    request.input_half_box_mps2 = 4.0;
    request.snapshot_digest = "snapshot";
    request.config_digest = "config";
    request.fixed_topology = {{101, 1}, {102, 1}};
    request.mobile_states.emplace(1, PairwiseSecondOrderState2D{});
    request.nominal_controls.emplace(1, Eigen::Vector2d::Zero());
    gf::CanonicalHardRow row;
    row.id = "reference:101->1:owner:1";
    row.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    row.owner = 1;
    row.peer = 101;
    row.control_coefficient = {1.0, 0.0};
    row.constant = 1.0;
    request.owner_rows.emplace(1, std::vector<gf::CanonicalHardRow>{row});
    request.pair_tubes.emplace(row.id, gf::PairwiseSnapshotTube{});

    const auto result = gf::auditTask10p11sGateA(request);
    CHECK_FALSE(result.snapshot_complete);
    CHECK(result.reason == "frozen_snapshot_incomplete_for_full_28d");
}

TEST_CASE("complete snapshot rejects pseudo collision invalid tube and bypassed references") {
    const auto complete = completeSnapshotFixture();
    CHECK(gf::auditTask10p11sGateA(complete).snapshot_complete);

    auto no_collision_peer = complete;
    no_collision_peer.owner_rows.at(1).back().peer.reset();
    CHECK_FALSE(gf::auditTask10p11sGateA(no_collision_peer).snapshot_complete);

    auto invalid_tube = complete;
    invalid_tube.pair_tubes.begin()->second.position_radius_m =
        std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(gf::auditTask10p11sGateA(invalid_tube).snapshot_complete);

    auto nonfinite_nominal = complete;
    nonfinite_nominal.nominal_controls.at(1).x() =
        std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(gf::auditTask10p11sGateA(nonfinite_nominal).snapshot_complete);

    auto full_responsibility_mobile_pair = complete;
    full_responsibility_mobile_pair.owner_rows.at(1).back().responsibility = 1.0;
    CHECK_FALSE(gf::auditTask10p11sGateA(full_responsibility_mobile_pair)
                    .snapshot_complete);

    auto half_responsibility_fixed_pair = complete;
    half_responsibility_fixed_pair.owner_rows.at(1).front().responsibility = 0.5;
    CHECK_FALSE(gf::auditTask10p11sGateA(half_responsibility_fixed_pair)
                    .snapshot_complete);

    auto missing_reference_rows = complete;
    missing_reference_rows.owner_rows.at(1).erase(
        missing_reference_rows.owner_rows.at(1).begin(),
        missing_reference_rows.owner_rows.at(1).begin() + 2);
    CHECK_FALSE(gf::auditTask10p11sGateA(missing_reference_rows)
                    .snapshot_complete);
}

TEST_CASE("unknown frozen row kind fails schema integrity instead of becoming auxiliary") {
    const nlohmann::json evidence = {
        {"timeline", {{{"hard_polytope", {
            {"owner", 2}, {"input_half_box_mps2", 4.0},
            {"rows", {{{"id", "mystery:2"}, {"kind", "mystery"},
                       {"peer", nullptr}, {"normal", {1.0, 0.0}},
                       {"constant", 1.0}}}}
        }}}}}
    };
    const auto output = gf::auditTask10p11sEvidenceJson(evidence, "sha", "");
    CHECK(output.at("reason") == "schema_integrity_failure");
    CHECK(output.at("gate_b_authorized") == false);
}

TEST_CASE("runner provenance requires version binary profile seed and verifier fields") {
    nlohmann::json metadata = {
        {"parent_commit", "parent"}, {"parent_tree", "parent-tree"},
        {"parent_gitlink", "gitlink"}, {"parent_branch", "branch"},
        {"parent_upstream", "origin/branch"},
        {"parent_worktree_status", "dirty"}, {"cbf_commit", "cbf"},
        {"cbf_tree", "cbf-tree"}, {"cbf_branch", "cbf-branch"},
        {"cbf_upstream", "none"},
        {"cbf_worktree_status", "dirty"}, {"binary_sha256", "binary"},
        {"source_file_sha256", {{"header", "hash"}}},
        {"solver_profile", "exact-2d"}, {"solver_version", "in-tree"},
        {"seed", nullptr},
        {"independent_verifier", {
            {"result", "pending"}, {"source_sha256", "verifier-source"},
            {"tool_version", "scipy"}, {"output_path", "verifier.json"}
        }},
        {"snapshot_input_sha256", "snapshot"},
        {"config_digest", "config"}
    };
    CHECK(gf::task10p11sValidProvenance(metadata));
    metadata.erase("binary_sha256");
    CHECK_FALSE(gf::task10p11sValidProvenance(metadata));
    metadata["binary_sha256"] = "binary";
    metadata["config_digest"] = "";
    CHECK_FALSE(gf::task10p11sValidProvenance(metadata));
    metadata["config_digest"] = "config";
    metadata["independent_verifier"]["result"] = "passed";
    CHECK_FALSE(gf::task10p11sValidProvenance(metadata));
}
