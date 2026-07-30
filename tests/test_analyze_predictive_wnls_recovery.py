"""Contracts for the exact Stage-1 predictive-recovery analyzer."""

from __future__ import annotations

import copy
import gzip
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from scripts.diagnostics import analyze_predictive_wnls_recovery as analyzer
from scripts.diagnostics import replay_predictive_wnls_recovery as replay
from scripts.diagnostics.run_diagnostic import START_BYTES, DiskSpaceError


def baseline_row(
    seed: int,
    frame: int,
    robot: int,
    *,
    attempt: str = "converged",
    status: str = "converged",
    error: float | None = 1.0,
) -> dict:
    return {
        "graph_case": "dynamic_dag_wnls",
        "seed": seed,
        "frame_index": frame,
        "robot_id": robot,
        "squad_local_index": robot,
        "attempt_status": attempt,
        "status": status,
        "error_norm": error,
        "finite": error is not None,
    }


def development_row(
    variant: str,
    seed: int,
    frame: int,
    robot: int,
    *,
    output: str = "fresh",
    attempt: str = "accepted",
    error: float | None = 1.0,
    prediction_age: int | None = 0,
) -> dict:
    values = {field: None for field in replay.ROW_FIELDS}
    values.update(
        {
            "variant": variant,
            "seed": seed,
            "frame_index": frame,
            "robot_id": robot,
            "squad_local_index": robot,
            "applied_command_source_frame": None if frame == 0 else frame - 1,
            "applied_command": None if frame == 0 else [1.0, 0.0],
            "legacy_numeric_status": "converged",
            "legacy_initial_estimate_source": "test",
            "attempt_status": attempt,
            "attempt_failure_reason": None,
            "output_status": output,
            "prediction_age": prediction_age,
            "estimate": None if output == "unavailable" else [error or 0.0, 0.0],
            "fresh_modeled_covariance": (
                [[1.0, 0.0], [0.0, 1.0]] if output == "fresh" else None
            ),
            "fresh_epsilon": 3.0 if output == "fresh" else None,
            "aged_modeled_covariance": (
                [[2.0, 0.0], [0.0, 2.0]] if output == "predicted" else None
            ),
            "aged_modeled_radius": (
                3.0 * (2.0**0.5) if output == "predicted" else None
            ),
            "private_reacquisition_seed": None,
            "attempt_base_anchor_provenance": [0, 1],
            "base_anchor_provenance": [0, 1] if output == "fresh" else [],
            "mandatory_references": {"base_ids": [0, 1], "uav_ids": []},
            "optional_candidates": [],
            "active_references": [["base", 0], ["base", 1]],
            "reference_evidence": [
                [
                    "base",
                    base_id,
                    "mandatory",
                    True,
                    1.0,
                    seed + base_id,
                    "fresh",
                    True,
                    attempt != "reference_unavailable",
                    None,
                    [base_id],
                ]
                for base_id in (0, 1)
            ],
            "reference_freshness": [
                ["base", base_id, "fresh"] for base_id in (0, 1)
            ],
            "excluded_references": [],
            "reference_violations": [],
            "candidates": (
                [
                    [
                        "test",
                        [0.0, 0.0],
                        "converged",
                        [error or 0.0, 0.0],
                        [[1.0, 0.0], [0.0, 1.0]],
                        0.0,
                        True,
                        None,
                        1.0,
                        ["normalized", 1.0, "accepted", True, None, 0.0],
                        [],
                    ]
                ]
                if attempt == "accepted"
                else []
            ),
            "selected_candidate_source": "test" if attempt == "accepted" else None,
            "offline_truth_position": [0.0, 0.0],
            "offline_error_norm": error if output != "unavailable" else None,
            "offline_fresh_containment": (
                error <= 3.0 if output == "fresh" and error is not None else None
            ),
            "offline_aged_radius_containment": (
                error <= 3.0 * (2.0**0.5)
                if output == "predicted" and error is not None
                else None
            ),
            "offline_fresh_q_error": (
                error * error if output == "fresh" and error is not None else None
            ),
            "offline_aged_q_error": (
                error * error / 2.0
                if output == "predicted" and error is not None
                else None
            ),
        }
    )
    return values


def truth_data() -> dict:
    def robot(identifier: int, vx: float) -> dict:
        return {
            "id": identifier,
            "state": {"x": 0.0, "y": 0.0},
            "opt": {"result": {"vx": vx, "vy": 0.0}},
        }

    return {
        "state": [
            {"frame_index": 0, "robots": [robot(1, 1.0), robot(2, 1.0)]},
            {"frame_index": 1, "robots": [robot(1, 30.0), robot(2, 1.0)]},
            {"frame_index": 2, "robots": [robot(1, 1.0), robot(2, 1.0)]},
        ]
    }


def protocol() -> dict:
    return {
        "experiment": {
            "variants": list(replay.DEVELOPMENT_VARIANTS),
            "frame_dt_seconds": 0.5,
        },
        "ablation_contracts": copy.deepcopy(replay.ABLATION_CONTRACTS),
        "estimator_constants": copy.deepcopy(replay.ESTIMATOR_CONSTANTS),
        "gates": copy.deepcopy(replay.GATES),
    }


def paired_fixture() -> tuple[list[dict], list[dict]]:
    baseline = [
        baseline_row(11, 0, 1, error=1.0),
        baseline_row(11, 0, 2, status="stale", attempt="failed", error=60.0),
        baseline_row(
            11, 1, 1, status="failed", attempt="invalid", error=None
        ),
        baseline_row(11, 1, 2, error=4.0),
        baseline_row(11, 2, 1, error=6.0),
    ]
    rows = []
    for variant in replay.DEVELOPMENT_VARIANTS:
        rows.extend(
            [
                development_row(variant, 11, 0, 1, error=0.5),
                development_row(
                    variant,
                    11,
                    0,
                    2,
                    output="predicted",
                    attempt="rejected",
                    error=3.0,
                    prediction_age=1,
                ),
                development_row(
                    variant,
                    11,
                    1,
                    1,
                    output="unavailable",
                    attempt="invalid",
                    error=None,
                    prediction_age=None,
                ),
                development_row(
                    variant,
                    11,
                    1,
                    2,
                    output=(
                        "unavailable"
                        if variant == "predictive_multistart"
                        else "fresh"
                    ),
                    attempt=(
                        "reference_unavailable"
                        if variant == "predictive_multistart"
                        else "accepted"
                    ),
                    error=None if variant == "predictive_multistart" else 2.0,
                    prediction_age=(
                        None if variant == "predictive_multistart" else 0
                    ),
                ),
                development_row(
                    variant,
                    11,
                    2,
                    1,
                    output="predicted",
                    attempt="rejected",
                    error=5.0,
                    prediction_age=2,
                ),
            ]
        )
    rejected = rows[-4]
    rejected["candidates"] = [
        [
            "algebraic",
            [0.0, 0.0],
            "converged",
            [9.0, 0.0],
            [[1.0, 0.0], [0.0, 1.0]],
            1.0,
            False,
            "innovation_gate",
            12.0,
            ["normalized", 12.0, "rejected", True, None, 1.0],
            [],
        ]
    ]
    accepted = rows[-5]
    accepted["candidates"] = [
        [
            "prediction",
            [0.0, 0.0],
            "converged",
            [0.5, 0.0],
            [[1.0, 0.0], [0.0, 1.0]],
            0.1,
            True,
            None,
            2.5,
            ["normalized", 2.5, "accepted", True, None, 0.1],
            [],
        ]
    ]
    accepted["selected_candidate_source"] = "prediction"
    for row in rows:
        if row["frame_index"] == 2 and row["robot_id"] == 1:
            row["applied_command"] = [30.0, 0.0]
    return baseline, rows


class PureAggregationTests(unittest.TestCase):
    def test_status_mapping_exact_cohorts_and_denominators(self):
        """Breaks if stale or unavailable baseline rows enter fresh errors."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        self.assertEqual(result["baseline"]["output_counts"], {
            "fresh": 3,
            "legacy_published": 1,
            "unavailable": 1,
        })
        complete = result["variants"]["predictive_multistart"]
        self.assertEqual(
            complete["output_counts"],
            {"fresh": 1, "predicted": 2, "unavailable": 2},
        )
        self.assertEqual(complete["budgets"], {"attempted": 5, "outputs": 5})
        self.assertEqual(
            sum(complete["attempt_counts"].values()), complete["budgets"]["attempted"]
        )
        self.assertEqual(
            complete["errors"]["fresh"]["denominator"], 1
        )
        self.assertEqual(
            complete["errors"]["all_published"]["denominator"], 3
        )
        self.assertEqual(
            result["prediction_transition_audit"]["unique_predecessor_rows"], 3
        )
        self.assertEqual(
            set(complete["errors"]["all_published"]),
            {"denominator", "p50", "p95", "p99", "maximum"},
        )
        self.assertEqual(complete["errors"]["all_published"]["p50"], 3.0)
        self.assertEqual(complete["errors"]["all_published"]["maximum"], 5.0)

    def test_pair_audits_separate_attrition_downgrade_and_improvement(self):
        """Breaks if excluded rows are disguised as paired improvements."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        paired = result["variants"]["predictive_multistart"]["paired"]
        published = paired["baseline_published"]
        self.assertEqual(
            published["transition_counts"],
            {"fresh": 1, "predicted": 2, "unavailable": 1},
        )
        self.assertEqual(published["newly_unavailable"]["count"], 1)
        self.assertEqual(published["newly_unavailable"]["baseline_errors"]["maximum"], 4.0)
        self.assertEqual(published["both_published_error_change"]["denominator"], 3)
        fresh = paired["baseline_fresh"]
        self.assertEqual(
            fresh["transition_counts"],
            {"fresh": 1, "predicted": 1, "unavailable": 1},
        )
        self.assertEqual(fresh["both_fresh_error_change"]["denominator"], 1)
        self.assertEqual(fresh["downgraded_to_predicted"]["count"], 1)
        self.assertEqual(fresh["newly_unavailable"]["count"], 1)
        self.assertEqual(fresh["excluded_from_both_fresh"]["count"], 2)

    def test_calibration_rejected_candidates_catastrophic_and_strata(self):
        """Breaks if diagnostic families share denominators or strata disappear."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        complete = result["variants"]["predictive_multistart"]
        self.assertEqual(
            {
                key: complete["catastrophic"]["all_published"][key]
                for key in ("numerator", "denominator")
            },
            {"numerator": 0, "denominator": 3},
        )
        calibration = complete["calibration"]
        self.assertEqual(calibration["fresh_containment"]["denominator"], 1)
        self.assertEqual(calibration["fresh_epsilon"]["denominator"], 1)
        self.assertEqual(calibration["fresh_q_error"]["denominator"], 1)
        self.assertEqual(calibration["fresh_q_error"]["above_5_991464547107979"], 0)
        self.assertEqual(calibration["fresh_q_error"]["above_9"], 0)
        self.assertEqual(calibration["aged_containment"]["denominator"], 2)
        self.assertEqual(calibration["aged_radius"]["denominator"], 2)
        self.assertEqual(calibration["aged_q_error"]["above_5_991464547107979"], 1)
        self.assertEqual(calibration["aged_q_error"]["above_9"], 1)
        self.assertEqual(calibration["online_q_innovation"]["accepted"]["denominator"], 1)
        self.assertEqual(calibration["online_q_innovation"]["rejected"]["denominator"], 1)
        rejected = complete["rejected_candidate_offline_errors"]
        self.assertEqual(rejected["denominator"], 1)
        self.assertEqual(rejected["maximum"], 9.0)
        self.assertEqual(
            set(complete["strata"]),
            {"depth", "squad", "time", "seed"},
        )
        self.assertEqual(complete["strata"]["seed"]["11"]["rows"], 5)
        self.assertEqual(complete["strata"]["squad"]["1"]["rows"], 5)
        self.assertEqual(complete["strata"]["time"]["0"]["rows"], 5)
        self.assertEqual(complete["strata"]["depth"]["1"]["rows"], 3)

    def test_truth_input_limit_audit_deduplicates_variants_and_noise_seeds(self):
        """Breaks if repeated raw rows inflate the physical command denominator."""
        audit = analyzer.audit_input_limits(truth_data(), component_limit=25.0)
        self.assertEqual(audit["unique_physical_rows"], 6)
        self.assertEqual(audit["component_bound_violations"], 1)
        self.assertEqual(audit["maximum_applied_component"], 30.0)
        self.assertEqual(audit["maximum_planar_norm"], 30.0)
        self.assertEqual(audit["maximum_displacement"], 15.0)

    def test_production_shaped_input_audit_is_exactly_243_of_7000(self):
        """Breaks if Stage 1 audits predecessor rows or repeated noise rows."""
        states = []
        remaining = 243
        for frame in range(500):
            robots = []
            for robot_id in range(1, 15):
                violating = remaining > 0
                if violating:
                    remaining -= 1
                robots.append(
                    {
                        "id": robot_id,
                        "state": {"x": 0.0, "y": 0.0},
                        "opt": {
                            "result": {
                                "vx": 25.5 if violating else 25.0,
                                "vy": 0.0,
                            }
                        },
                    }
                )
            states.append({"frame_index": frame, "robots": robots})
        audit = analyzer.audit_input_limits(
            {"state": states}, component_limit=25.0
        )
        self.assertEqual(
            (
                audit["component_bound_violations"],
                audit["unique_physical_rows"],
            ),
            (243, 7000),
        )

    def test_production_summary_requires_complete_6986_predecessor_coverage(self):
        """Breaks if production can publish with missing prediction transitions."""
        summary = {
            "mechanism_records": {
                "record": {
                    "baseline": {"error": 1.0},
                    "variants": {
                        variant: {"error": 1.0}
                        for variant in replay.DEVELOPMENT_VARIANTS
                    },
                }
            },
            "input_limits": {
                "unique_physical_rows": 7000,
                "component_bound_violations": 243,
            },
            "prediction_transition_audit": {
                "unique_predecessor_rows": 6986,
                "expected_unique_predecessor_rows": 6986,
                "coverage_complete": True,
            },
        }
        analyzer._validate_production_summary(summary)
        mutations = (
            ("unique_predecessor_rows", 6985),
            ("expected_unique_predecessor_rows", 6985),
            ("coverage_complete", False),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                changed = copy.deepcopy(summary)
                changed["prediction_transition_audit"][field] = value
                with self.assertRaises(ValueError):
                    analyzer._validate_production_summary(changed)

    def test_production_raw_disk_metrics_reconcile_to_pinned_leaf(self):
        """Breaks if replay disk claims are trusted without raw-leaf reconciliation."""
        metrics = {
            "free_bytes_before": START_BYTES,
            "free_bytes_after": replay.HARD_FLOOR_BYTES + 1,
            "allocated_bytes": 4096,
            "raw_bundle_cap_bytes": replay.RAW_BUNDLE_CAP_BYTES,
        }
        analyzer._validate_production_disk_metrics(
            metrics, actual_allocated_bytes=4096
        )
        mutations = (
            ("free_bytes_before", START_BYTES - 1),
            ("free_bytes_after", replay.HARD_FLOOR_BYTES - 1),
            ("allocated_bytes", 8192),
            ("allocated_bytes", True),
            ("raw_bundle_cap_bytes", replay.RAW_BUNDLE_CAP_BYTES - 1),
        )
        for field, value in mutations:
            with self.subTest(field=field, value=value):
                changed = dict(metrics)
                changed[field] = value
                with self.assertRaises(ValueError):
                    analyzer._validate_production_disk_metrics(
                        changed, actual_allocated_bytes=4096
                    )

    def test_exact_key_contract_rejects_duplicate_missing_extra_and_order_drift(self):
        """Breaks if positional pairing can silently change a denominator."""
        baseline, rows = paired_fixture()
        mutations = [
            rows + [copy.deepcopy(rows[0])],
            rows[:-1],
            rows + [development_row("predictive_multistart", 99, 0, 1)],
            [rows[1], rows[0], *rows[2:]],
        ]
        wrong_schema = copy.deepcopy(rows)
        wrong_schema[0].pop("offline_fresh_q_error")
        mutations.append(wrong_schema)
        semantic_drift = copy.deepcopy(rows)
        semantic_drift[0]["offline_error_norm"] = 0.75
        mutations.append(semantic_drift)
        epsilon_drift = copy.deepcopy(rows)
        epsilon_drift[0]["fresh_epsilon"] = 4.0
        mutations.append(epsilon_drift)
        covariance_drift = copy.deepcopy(rows)
        covariance_drift[0]["fresh_modeled_covariance"] = [
            [1.0, 0.1],
            [0.0, 1.0],
        ]
        mutations.append(covariance_drift)
        truth_drift = copy.deepcopy(rows)
        truth_drift[0]["offline_truth_position"] = [1.0, 0.0]
        truth_drift[0]["estimate"] = [1.5, 0.0]
        mutations.append(truth_drift)
        compact_drift = copy.deepcopy(rows)
        compact_drift[-5]["candidates"][0][9] = ["truncated"]
        mutations.append(compact_drift)
        attempt_output_drift = copy.deepcopy(rows)
        attempt_output_drift[0]["attempt_status"] = "failed"
        attempt_output_drift[0]["selected_candidate_source"] = None
        attempt_output_drift[0]["candidates"][0][6] = False
        mutations.append(attempt_output_drift)
        accepted_predicted_drift = copy.deepcopy(rows)
        accepted_predicted_drift[1]["attempt_status"] = "accepted"
        mutations.append(accepted_predicted_drift)
        selected_source_drift = copy.deepcopy(rows)
        selected_source_drift[0]["selected_candidate_source"] = "not_a_candidate"
        mutations.append(selected_source_drift)
        accepted_candidate_drift = copy.deepcopy(rows)
        accepted_candidate_drift[0]["candidates"][0][6] = False
        mutations.append(accepted_candidate_drift)
        squad_local_drift = copy.deepcopy(rows)
        squad_local_drift[0]["squad_local_index"] = 7
        mutations.append(squad_local_drift)
        squad_local_bool = copy.deepcopy(rows)
        squad_local_bool[0]["squad_local_index"] = True
        mutations.append(squad_local_bool)
        squad_local_string = copy.deepcopy(rows)
        squad_local_string[0]["squad_local_index"] = "1"
        mutations.append(squad_local_string)
        for changed in mutations:
            with self.subTest(rows=len(changed)):
                with self.assertRaises(ValueError):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth_data(),
                        protocol=protocol(),
                    )

    def test_baseline_filter_and_native_order_are_enforced(self):
        """Breaks if fixed-graph rows enter pairs or dynamic rows reorder."""
        baseline, rows = paired_fixture()
        fixed = dict(baseline[0])
        fixed["graph_case"] = "fixed_refs_wnls"
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=[fixed, *baseline],
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        self.assertEqual(result["baseline"]["budgets"]["attempted"], 5)
        reordered = [baseline[2], baseline[0], baseline[1], *baseline[3:]]
        with self.assertRaisesRegex(ValueError, "out of order"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=reordered,
                development_rows=rows,
                truth_data=truth_data(),
                protocol=protocol(),
            )

    def test_protocol_integrity_violations_fail_closed(self):
        """Breaks if aged anchors, age three, or cycles can enter evidence."""
        baseline, rows = paired_fixture()
        mutations = []
        age = copy.deepcopy(rows)
        age[-1]["prediction_age"] = 3
        mutations.append(age)
        dag = copy.deepcopy(rows)
        dag[-1]["active_references"].append(["uav", dag[-1]["robot_id"]])
        mutations.append(dag)
        anchor = copy.deepcopy(rows)
        anchor[-1]["reference_evidence"] = [
            [
                "uav",
                1,
                "optional",
                True,
                1.0,
                2,
                "predicted",
                True,
                True,
                None,
                [0, 1],
            ]
        ]
        mutations.append(anchor)
        provenance = copy.deepcopy(rows)
        provenance[-1]["reference_violations"] = [
            ["uav", 1, "provenance"]
        ]
        mutations.append(provenance)
        current_chain = copy.deepcopy(rows)
        current_chain[5]["base_anchor_provenance"] = [2, 3]
        mutations.append(current_chain)
        attempt_chain = copy.deepcopy(rows)
        attempt_chain[6]["attempt_base_anchor_provenance"] = [99]
        mutations.append(attempt_chain)
        timing = copy.deepcopy(rows)
        timing[-1]["applied_command"] = [1.0, 0.0]
        mutations.append(timing)
        for changed in mutations:
            with self.subTest():
                with self.assertRaises(ValueError):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth_data(),
                        protocol=protocol(),
                    )

    def test_prediction_expiry_retains_diagnostic_reference_violations(self):
        """Breaks if the sole fresh-qualification ablation is rejected or hidden."""
        baseline, rows = paired_fixture()
        diagnostic = ["uav", 1, "stale_or_predicted_anchor_used"]
        rows[1]["active_references"].append(["uav", 1])
        rows[1]["reference_evidence"].append(
            [
                "uav",
                1,
                "optional",
                True,
                1.0,
                12,
                "predicted",
                False,
                True,
                None,
                [0, 1],
            ]
        )
        rows[1]["reference_freshness"].append(["uav", 1, "predicted"])
        rows[1]["reference_violations"] = [diagnostic]
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        integrity = result["variants"]["prediction_expiry"]["integrity"]
        self.assertEqual(integrity["reference_violation_count"], 1)
        self.assertEqual(
            integrity["reference_violation_reasons"],
            {"stale_or_predicted_anchor_used": 1},
        )
        qualified = copy.deepcopy(rows)
        qualified[1]["active_references"].pop()
        qualified[1]["reference_evidence"].pop()
        qualified[1]["reference_freshness"].pop()
        qualified[1]["reference_violations"] = []
        qualified[6]["active_references"].append(["uav", 1])
        qualified[6]["reference_evidence"].append(
            [
                "uav",
                1,
                "optional",
                True,
                1.0,
                12,
                "predicted",
                False,
                True,
                None,
                [0, 1],
            ]
        )
        qualified[6]["reference_freshness"].append(["uav", 1, "predicted"])
        qualified[6]["reference_violations"] = [diagnostic]
        with self.assertRaises(ValueError):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=qualified,
                truth_data=truth_data(),
                protocol=protocol(),
            )
        arbitrary = copy.deepcopy(rows)
        arbitrary[1]["reference_violations"][0][2] = "arbitrary_reason"
        with self.assertRaises(ValueError):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=arbitrary,
                truth_data=truth_data(),
                protocol=protocol(),
            )

    def test_mechanisms_are_resolved_by_exact_key_not_maximum(self):
        """Breaks if the historical mechanisms are found by approximate tails."""
        baseline = {
            (20260736, 138, 14): baseline_row(
                20260736,
                138,
                14,
                attempt="failed",
                status="stale",
                error=999.3318962079554,
            ),
            (20260730, 177, 14): baseline_row(
                20260730, 177, 14, error=168.90169712504604
            ),
            (20260736, 44, 14): baseline_row(
                20260736, 44, 14, attempt="failed", status="stale", error=12.0
            ),
            (1, 1, 1): baseline_row(1, 1, 1, error=2000.0),
        }
        variants = {
            name: {
                key: development_row(name, *key, error=1.0)
                for key in baseline
            }
            for name in replay.DEVELOPMENT_VARIANTS
        }
        records = analyzer.resolve_mechanism_records(baseline, variants)
        self.assertEqual(
            records["legacy_999m_stale"]["key"],
            {"seed": 20260736, "frame_index": 138, "robot_id": 14},
        )
        self.assertEqual(
            records["legacy_168m_fresh"]["key"],
            {"seed": 20260730, "frame_index": 177, "robot_id": 14},
        )
        self.assertEqual(
            records["frame44_recovery"]["key"]["frame_index"], 44
        )

    def test_markdown_exposes_every_diagnostic_family_with_denominators(self):
        """Breaks if compact JSON evidence is hidden from the human report."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        markdown = analyzer._markdown(result)
        for label in (
            "Baseline budgets and tails",
            "Attempt counts and output tails",
            "Both-published error change",
            "Excluded baseline-error tails",
            "Catastrophic errors and availability",
            "Prediction lifecycle and integrity",
            "Reference summaries",
            "FIM, epsilon, radius, and q diagnostics",
            "Rejected candidate offline errors",
            "Stratified diagnostics",
            "Prediction-transition coverage: 3/4",
            "p50 / p95 / p99 / max / n",
        ):
            with self.subTest(label=label):
                self.assertIn(label, markdown)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


class AnalyzerLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.output = self.root.parent / f"{self.root.name}-analysis"

    def tearDown(self):
        if self.output.is_symlink():
            self.output.unlink()
        elif self.output.exists():
            for path in self.output.iterdir():
                path.unlink()
            self.output.rmdir()
        self.temporary.cleanup()

    def build_bundle(self, rows=None):
        baseline, default_rows = paired_fixture()
        rows = default_rows if rows is None else rows
        baseline_path = self.root / "baseline.jsonl.gz"
        raw_root = self.root / "raw"
        raw_root.mkdir()
        raw_path = raw_root / replay.RAW_PROCESS_NAME
        manifest_path = raw_root / replay.TERMINAL_MANIFEST_NAME
        protocol_path = self.root / "protocol.json"
        truth_path = self.root / "truth.json"
        with gzip.open(baseline_path, "wt") as target:
            for row in baseline:
                target.write(json.dumps(row) + "\n")
        decompressed = b"".join(
            (json.dumps(row) + "\n").encode() for row in rows
        )
        with raw_path.open("wb") as raw_file:
            with gzip.GzipFile(
                filename="", fileobj=raw_file, mode="wb", mtime=0
            ) as target:
                target.write(decompressed)
        truth_path.write_text(json.dumps(truth_data()))
        protocol_value = {
            "schema_id": analyzer.ANALYZER_TEST_SCHEMA_ID,
            "sources": {
                "truth_data": {
                    "path": str(truth_path),
                    "sha256": sha256(truth_path),
                },
                "baseline_process": {
                    "path": str(baseline_path),
                    "sha256": sha256(baseline_path),
                },
            },
            "experiment": {
                **protocol()["experiment"],
                "evidence_class": "hermetic_non_evidence_only",
            },
            "ablation_contracts": protocol()["ablation_contracts"],
            "estimator_constants": protocol()["estimator_constants"],
            "gates": protocol()["gates"],
            "raw_schema": copy.deepcopy(replay.RAW_SCHEMA_DECLARATION),
            "analysis_schema": copy.deepcopy(replay.ANALYSIS_SCHEMA),
            "disk_contract": copy.deepcopy(replay.DISK_CONTRACT),
            "invocations": {
                "analyzer_test": {
                    "kind": "analyzer_test_only",
                    "development_manifest_path": str(manifest_path),
                    "output_root": str(self.output),
                    "expected_baseline_sha256": sha256(baseline_path),
                }
            },
        }
        protocol_path.write_text(json.dumps(protocol_value, sort_keys=True))
        manifest = {
            "status": "completed",
            "schema_id": replay.RAW_SCHEMA_ID,
            "raw_schema": replay.RAW_SCHEMA_DECLARATION,
            "rows_written": len(rows),
            "expected_rows": len(rows),
            "compressed_process_sha256": sha256(raw_path),
            "decompressed_process_sha256": hashlib.sha256(decompressed).hexdigest(),
        }
        manifest_path.write_text(json.dumps(manifest))
        return {
            "baseline": baseline_path,
            "raw": raw_path,
            "manifest": manifest_path,
            "protocol": protocol_path,
            "truth": truth_path,
            "manifest_value": manifest,
        }

    def execute(self, bundle):
        return analyzer.analyze_predictive_recovery(
            baseline_process_path=bundle["baseline"],
            development_manifest_path=bundle["manifest"],
            protocol_path=bundle["protocol"],
            expected_baseline_sha256=sha256(bundle["baseline"]),
            output_root=self.output,
        )

    def test_exclusive_root_success_cap_and_terminal_manifest(self):
        """Breaks if compact evidence can overwrite or publish without a terminal."""
        bundle = self.build_bundle()
        completed = self.execute(bundle)
        self.assertEqual(completed["status"], "completed")
        self.assertEqual(
            set(path.name for path in self.output.iterdir()),
            {
                analyzer.OUTPUT_JSON_NAME,
                analyzer.OUTPUT_MARKDOWN_NAME,
                analyzer.ANALYZER_MANIFEST_NAME,
            },
        )
        self.assertLessEqual(
            completed["allocated_bytes"], analyzer.COMPACT_OUTPUT_CAP_BYTES
        )
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "completed")
        markdown = (self.output / analyzer.OUTPUT_MARKDOWN_NAME).read_text()
        for label in (
            "Baseline-published attrition",
            "Baseline-fresh transitions",
            "Calibration denominators",
            "Input-limit audit: 1/6",
            "frame44_recovery",
            "legacy_999m_stale",
            "legacy_168m_fresh",
        ):
            self.assertIn(label, markdown)
        with self.assertRaises(FileExistsError):
            self.execute(bundle)

    def test_failure_after_allocation_writes_failed_terminal(self):
        """Breaks if an allocated failed analysis erases failure evidence."""
        _, rows = paired_fixture()
        rows[-1]["prediction_age"] = 3
        bundle = self.build_bundle(rows)
        with self.assertRaises(ValueError):
            self.execute(bundle)
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_authority_hash_row_count_and_decompressed_hash_fail_preallocation(self):
        """Breaks if manifest/source drift can allocate an evidence root."""
        mutations = (
            ("expected_rows", 1),
            ("compressed_process_sha256", "0" * 64),
            ("decompressed_process_sha256", "0" * 64),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                bundle = self.build_bundle()
                manifest = json.loads(bundle["manifest"].read_text())
                manifest[field] = value
                bundle["manifest"].write_text(json.dumps(manifest))
                with self.assertRaises(ValueError):
                    self.execute(bundle)
                self.assertFalse(self.output.exists())
                bundle["raw"].parent.joinpath(replay.TERMINAL_MANIFEST_NAME).unlink()
                bundle["raw"].unlink()
                bundle["raw"].parent.rmdir()

    def test_compact_cap_failure_retains_failed_terminal(self):
        """Breaks if the JSON/Markdown/manifest aggregate can exceed 10 MB."""
        bundle = self.build_bundle()
        with mock.patch.object(analyzer, "COMPACT_OUTPUT_CAP_BYTES", 1):
            with self.assertRaises(DiskSpaceError):
                self.execute(bundle)
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_symlink_and_preexisting_targets_fail_closed(self):
        """Breaks if exact-root allocation follows or reuses a target."""
        bundle = self.build_bundle()
        self.output.symlink_to(self.root)
        with self.assertRaises((FileExistsError, ValueError)):
            self.execute(bundle)
        self.assertTrue(self.output.is_symlink())

    def test_raw_identity_drift_before_completed_becomes_failed_terminal(self):
        """Breaks if final source revalidation is omitted."""
        bundle = self.build_bundle()
        original = analyzer._reverify_inputs
        mutated = False

        def mutate_then_verify(*args, **kwargs):
            nonlocal mutated
            if not mutated:
                mutated = True
                bundle["raw"].write_bytes(bundle["raw"].read_bytes() + b"x")
            return original(*args, **kwargs)

        with mock.patch.object(
            analyzer, "_reverify_inputs", side_effect=mutate_then_verify
        ):
            with self.assertRaises(ValueError):
                self.execute(bundle)
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_durable_completed_manifest_wins_post_commit_interrupt(self):
        """Breaks if an asynchronous delivery fault rewrites valid success."""
        bundle = self.build_bundle()
        with mock.patch.object(
            replay, "_post_commit_boundary", side_effect=KeyboardInterrupt
        ):
            completed = self.execute(bundle)
        self.assertEqual(completed["status"], "completed")
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "completed")
        self.assertFalse(
            any(
                path.name.startswith("manifest.failed")
                for path in self.output.iterdir()
            )
        )

    def test_link_then_interrupt_reconcile_cleans_retained_stage(self):
        """Breaks if completed reconciliation leaks its parent staging entry."""
        bundle = self.build_bundle()
        original = replay._link_stage

        def link_then_interrupt(transaction, stage):
            original(transaction, stage)
            if stage["name"].endswith(".completed"):
                raise KeyboardInterrupt

        with mock.patch.object(replay, "_link_stage", side_effect=link_then_interrupt):
            completed = self.execute(bundle)
        self.assertEqual(completed["status"], "completed")
        self.assertFalse(
            any(
                self.output.parent.glob(
                    f".{self.output.name}.manifest.completed*"
                )
            )
        )


if __name__ == "__main__":
    unittest.main()
