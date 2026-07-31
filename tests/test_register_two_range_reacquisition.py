import copy
import gzip
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

from scripts.diagnostics import register_two_range_reacquisition as registrar


def source_record(path, seed):
    return {
        "path": path,
        "device": 100 + seed,
        "inode": 200 + seed,
        "size": 300 + seed,
        "mtime_ns": 400 + seed,
        "sha256": f"{seed:064x}",
    }


def canonical_sources():
    return {
        name: source_record(f"/private/tmp/sources/{name}", index + 1)
        for index, name in enumerate(registrar.SOURCE_MEMBER_NAMES)
    }


def canonical_comparators():
    expected = registrar.EXPECTED_COMPARATOR_BINDINGS
    replay_root = expected["v4_replay_root"]
    analysis_root = expected["v4_analysis_root"]
    process = source_record(
        f"{replay_root}/predictive-wnls-development.jsonl.gz",
        30,
    )
    process["sha256"] = expected["v4_compressed_sha256"]
    return {
        "v4_replay_root": replay_root,
        "v4_replay_manifest": {
            **source_record(f"{replay_root}/manifest.json", 29),
            "sha256": expected["v4_manifest_sha256"],
        },
        "v4_compressed_process": process,
        "v4_decompressed_process": {
            **process,
            "sha256": expected["v4_decompressed_sha256"],
        },
        "v4_analysis_root": analysis_root,
        "v4_analysis_manifest": {
            **source_record(f"{analysis_root}/manifest.json", 32),
            "sha256": expected["v4_analysis_manifest_sha256"],
        },
        "v4_analysis_json": {
            **source_record(
                f"{analysis_root}/predictive-wnls-development.json",
                33,
            ),
            "sha256": expected["v4_analysis_json_sha256"],
        },
        "v4_analysis_markdown": {
            **source_record(
                f"{analysis_root}/predictive-wnls-development.md",
                34,
            ),
            "sha256": expected["v4_analysis_markdown_sha256"],
        },
        "legacy_baseline_process": {
            **source_record(
                "/private/tmp/frozen/baseline/calibration.jsonl.gz",
                35,
            ),
            "sha256": expected["legacy_baseline_process_sha256"],
        },
        "legacy_baseline_protocol_json_sha256": expected[
            "legacy_baseline_protocol_json_sha256"
        ],
    }


def canonical_protocol():
    return registrar._build_protocol(
        head="a" * 40,
        sources=canonical_sources(),
        comparators=canonical_comparators(),
    )


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ProtocolSchemaTests(unittest.TestCase):
    def test_direct_script_cli_help_runs_from_repository_root(self):
        repository_root = Path(registrar.__file__).resolve().parents[2]
        completed = subprocess.run(
            [
                sys.executable,
                "scripts/diagnostics/register_two_range_reacquisition.py",
                "--help",
            ],
            cwd=repository_root,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--repository-root", completed.stdout)
        self.assertIn("--output-markdown", completed.stdout)
        self.assertIn("--output-json", completed.stdout)

    def test_top_level_field_order(self):
        self.assertEqual(
            registrar.PROTOCOL_FIELDS,
            (
                "schema_id",
                "registration_schema_id",
                "protocol_id",
                "implementation_parent_commit",
                "binding_design",
                "sources",
                "comparators",
                "experiment",
                "method_contract",
                "estimator_constants",
                "status_contract",
                "raw_schema",
                "analysis_schema",
                "gates",
                "disk_contract",
                "invocations",
                "evidence_lifecycle",
                "authorization",
                "commands",
            ),
        )

    def test_source_member_order(self):
        self.assertEqual(
            registrar.SOURCE_MEMBER_NAMES,
            (
                "implementation_plan",
                "two_range_reacquisition_source",
                "predictive_wnls_source",
                "fixture_extractor_source",
                "replay_source",
                "analyzer_source",
                "registrar_source",
                "mechanism_fixture",
                "mechanism_fixture_manifest",
                "truth_data",
                "input_manifest",
            ),
        )

    def test_invocation_member_order(self):
        self.assertEqual(
            registrar.INVOCATION_MEMBER_NAMES,
            (
                "smoke_a",
                "smoke_b",
                "smoke_analyzer_a",
                "smoke_analyzer_b",
                "registered_replay",
                "registered_analyzer",
            ),
        )

    def test_protocol_section_field_orders(self):
        source_members = (
            "implementation_plan",
            "two_range_reacquisition_source",
            "predictive_wnls_source",
            "fixture_extractor_source",
            "replay_source",
            "analyzer_source",
            "registrar_source",
            "mechanism_fixture",
            "mechanism_fixture_manifest",
            "truth_data",
            "input_manifest",
        )
        invocation_members = (
            "smoke_a",
            "smoke_b",
            "smoke_analyzer_a",
            "smoke_analyzer_b",
            "registered_replay",
            "registered_analyzer",
        )
        self.assertEqual(
            registrar.PROTOCOL_SECTION_FIELDS,
            {
                "binding_design": (
                    "path",
                    "commit",
                    "sha256",
                    "review_path",
                    "review_sha256",
                ),
                "source_record": (
                    "path",
                    "device",
                    "inode",
                    "size",
                    "mtime_ns",
                    "sha256",
                ),
                "sources": source_members,
                "comparators": (
                    "v4_replay_root",
                    "v4_replay_manifest",
                    "v4_compressed_process",
                    "v4_decompressed_process",
                    "v4_analysis_root",
                    "v4_analysis_manifest",
                    "v4_analysis_json",
                    "v4_analysis_markdown",
                    "legacy_baseline_process",
                    "legacy_baseline_protocol_json_sha256",
                ),
                "experiment": (
                    "method",
                    "seeds",
                    "frames",
                    "robots",
                    "expected_rows",
                    "key_order",
                    "ranging_sigma_m",
                    "frame_dt_seconds",
                    "measurement_seed_contract",
                    "evidence_class",
                ),
                "method_contract": (
                    "structural_conditions",
                    "branch_ids",
                    "continuous_starts",
                    "private_prior_allowed_roles",
                    "private_prior_forbidden_roles",
                    "branch_score_rule",
                    "publication_rule",
                    "failure_rule",
                    "synthetic_declaration",
                    "synthetic_declaration_sha256",
                ),
                "estimator_constants": (
                    "maximum_public_prediction_age",
                    "innovation_reference_quantile",
                    "candidate_dedup_m",
                    "motion_covariance_per_frame",
                    "reacquisition_reduced_cost_max",
                    "maximum_error_m",
                ),
                "status_contract": (
                    "attempt_statuses",
                    "output_statuses",
                    "private_statuses",
                    "prior_used_semantics",
                ),
                "raw_schema": (
                    "schema_id",
                    "row_fields",
                    "row_scalar_contracts",
                    "row_array_contracts",
                    "nested_field_orders",
                    "branch_field_contracts",
                    "manifest_fields",
                    "null_rules",
                ),
                "analysis_schema": (
                    "schema_id",
                    "analysis_fields",
                    "nested_field_orders",
                    "value_contracts",
                    "list_order_and_cardinality",
                    "semantic_payload_fields",
                    "manifest_fields",
                    "manifest_nested_field_orders",
                    "manifest_source_member_names",
                    "manifest_null_rules",
                ),
                "gates": (
                    "scientific_gate_order",
                    "scientific_gate_records",
                    "integrity_gate_order",
                    "integrity_gate_records",
                    "aggregate_decision_rule",
                ),
                "disk_contract": (
                    "launch_minimum_free_bytes",
                    "live_minimum_free_bytes",
                    "raw_bundle_max_allocated_bytes",
                    "compact_bundle_max_allocated_bytes",
                ),
                "invocation_record": (
                    "invocation_name",
                    "input_root",
                    "output_root",
                    "expected_rows",
                    "authorization_required",
                    "retry_allowed",
                ),
                "invocations": invocation_members,
                "evidence_lifecycle": (
                    "preexisting_target_allowed",
                    "no_follow",
                    "descriptor_pinning",
                    "transactional_publication",
                    "fsync_required",
                    "terminal_manifest_required",
                    "failure_retained",
                    "paper_gate",
                ),
                "authorization": (
                    "implementation_plan_approved",
                    "protocol_preflight_required",
                    "deterministic_smoke_review_required",
                    "registered_full_grid_authorization",
                    "authorization_record_schema",
                    "authorization_record_path",
                ),
            },
        )

    def test_command_field_order(self):
        self.assertEqual(
            registrar.COMMAND_FIELDS,
            (
                "smoke_a",
                "smoke_b",
                "smoke_analyzer_a",
                "smoke_analyzer_b",
                "registered_replay",
                "registered_analyzer",
            ),
        )

    def test_binding_design_is_exact(self):
        self.assertEqual(
            registrar.BINDING_DESIGN,
            {
                "path": (
                    "docs/superpowers/specs/"
                    "2026-07-30-cbf2026-two-range-reacquisition-design.md"
                ),
                "commit": "20a61aad96af35ee7e16434fab0a5edaaea38ef0",
                "sha256": (
                    "d28d6bfe297a6e37b0a878dc716bd5b91f26c5b11568c3f6fc42bd07760a266b"
                ),
                "review_path": (
                    "docs/superpowers/specs/reviews/"
                    "2026-07-30-cbf2026-two-range-reacquisition-"
                    "design-review.md"
                ),
                "review_sha256": (
                    "6d857be965de5b83bb05ce06381b79108fdc4abbccf6b96fe4ec456d7a7df570"
                ),
            },
        )

    def test_comparator_bindings_are_exact(self):
        self.assertEqual(
            registrar.EXPECTED_COMPARATOR_BINDINGS,
            {
                "v4_replay_root": (
                    "/private/tmp/"
                    "cbf2026-predictive-wnls-development/stage1-v4"
                ),
                "v4_manifest_sha256": (
                    "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223"
                ),
                "v4_compressed_sha256": (
                    "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b"
                ),
                "v4_decompressed_sha256": (
                    "7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1"
                ),
                "v4_analysis_root": (
                    "/private/tmp/"
                    "cbf2026-predictive-wnls-development-analysis/stage1-v4"
                ),
                "v4_analysis_manifest_sha256": (
                    "e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238"
                ),
                "v4_analysis_json_sha256": (
                    "8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e"
                ),
                "v4_analysis_markdown_sha256": (
                    "986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b"
                ),
                "legacy_baseline_process_sha256": (
                    "c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003"
                ),
                "legacy_baseline_protocol_json_sha256": (
                    "09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0"
                ),
            },
        )

    def test_output_roots_are_exact(self):
        self.assertEqual(
            registrar.ROOTS,
            {
                "smoke_a": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-v1-a"
                ),
                "smoke_b": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-v1-b"
                ),
                "smoke_analyzer_a": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-analysis-v1-a"
                ),
                "smoke_analyzer_b": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-analysis-v1-b"
                ),
                "registered_replay": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-development/v1"
                ),
                "registered_analyzer": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-analysis/v1"
                ),
            },
        )

    def test_schema_and_method_ids_bind_producer_and_analyzer(self):
        self.assertEqual(
            (
                registrar.PROTOCOL_SCHEMA_ID,
                registrar.REGISTRATION_SCHEMA_ID,
                registrar.RAW_SCHEMA_ID,
                registrar.ANALYSIS_SCHEMA_ID,
                registrar.METHOD_ID,
            ),
            (
                "cbf2026-two-range-reacquisition-protocol-v1",
                "cbf2026-two-range-reacquisition-registration-v1",
                "cbf2026-two-range-reacquisition-raw-v1",
                "cbf2026-two-range-reacquisition-analysis-v1",
                "two_range_private_branch_reacquisition",
            ),
        )

    def test_invocation_contract_freezes_smoke_and_registered_cardinality(self):
        expected = {
            "smoke_a": {
                "invocation_name": "smoke_a",
                "input_root": (
                    "tests/fixtures/cbf2026_two_range_reacquisition"
                ),
                "output_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-v1-a"
                ),
                "expected_rows": 18,
                "authorization_required": False,
                "retry_allowed": False,
            },
            "smoke_b": {
                "invocation_name": "smoke_b",
                "input_root": (
                    "tests/fixtures/cbf2026_two_range_reacquisition"
                ),
                "output_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-v1-b"
                ),
                "expected_rows": 18,
                "authorization_required": False,
                "retry_allowed": False,
            },
            "smoke_analyzer_a": {
                "invocation_name": "smoke_analyzer_a",
                "input_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-v1-a"
                ),
                "output_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-analysis-v1-a"
                ),
                "expected_rows": 18,
                "authorization_required": False,
                "retry_allowed": False,
            },
            "smoke_analyzer_b": {
                "invocation_name": "smoke_analyzer_b",
                "input_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-v1-b"
                ),
                "output_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-analysis-v1-b"
                ),
                "expected_rows": 18,
                "authorization_required": False,
                "retry_allowed": False,
            },
            "registered_replay": {
                "invocation_name": "registered_replay",
                "input_root": (
                    "/private/tmp/cbf2026-results/"
                    "mc-first-mechanism-250s/R/"
                    "20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725"
                ),
                "output_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-development/v1"
                ),
                "expected_rows": 140000,
                "authorization_required": True,
                "retry_allowed": False,
            },
            "registered_analyzer": {
                "invocation_name": "registered_analyzer",
                "input_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-development/v1"
                ),
                "output_root": (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-analysis/v1"
                ),
                "expected_rows": 140000,
                "authorization_required": True,
                "retry_allowed": False,
            },
        }

        observed = registrar.production_invocation_contract()

        self.assertEqual(tuple(observed), tuple(expected))
        self.assertEqual(observed, expected)
        for record in observed.values():
            self.assertEqual(
                tuple(record),
                (
                    "invocation_name",
                    "input_root",
                    "output_root",
                    "expected_rows",
                    "authorization_required",
                    "retry_allowed",
                ),
            )

    def test_command_contract_emits_six_literal_non_shell_argv_arrays(self):
        sources = {
            "truth_data": {"path": "/private/tmp/frozen/truth/data.json"},
            "input_manifest": {
                "path": "/private/tmp/frozen/truth/manifest.json"
            },
        }
        protocol_path = (
            "docs/diagnostics/"
            "2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json"
        )
        replay_prefix = [
            "conda",
            "run",
            "-n",
            "cbf_env",
            "python",
            "scripts/diagnostics/replay_two_range_reacquisition.py",
            "--protocol-path",
            protocol_path,
            "--data-path",
            "/private/tmp/frozen/truth/data.json",
            "--input-manifest-path",
            "/private/tmp/frozen/truth/manifest.json",
        ]
        analyzer_prefix = [
            "conda",
            "run",
            "-n",
            "cbf_env",
            "python",
            "scripts/diagnostics/analyze_two_range_reacquisition.py",
            "--protocol-path",
            protocol_path,
        ]
        authorization = (
            "docs/diagnostics/reviews/"
            "2026-07-30-cbf2026-two-range-reacquisition-"
            "registered-authorization.json"
        )
        expected = {
            "smoke_a": replay_prefix
            + [
                "--output-root",
                "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a",
                "--run-seeds",
                "--max-frames",
                "0",
                "--invocation-name",
                "smoke_a",
            ],
            "smoke_b": replay_prefix
            + [
                "--output-root",
                "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b",
                "--run-seeds",
                "--max-frames",
                "0",
                "--invocation-name",
                "smoke_b",
            ],
            "smoke_analyzer_a": analyzer_prefix
            + [
                "--raw-root",
                "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a",
                "--output-root",
                (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-analysis-v1-a"
                ),
                "--invocation-name",
                "smoke_analyzer_a",
            ],
            "smoke_analyzer_b": analyzer_prefix
            + [
                "--raw-root",
                "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b",
                "--output-root",
                (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-smoke-analysis-v1-b"
                ),
                "--invocation-name",
                "smoke_analyzer_b",
            ],
            "registered_replay": replay_prefix
            + [
                "--output-root",
                (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-development/v1"
                ),
                "--run-seeds",
                "20260727",
                "20260728",
                "20260729",
                "20260730",
                "20260731",
                "20260732",
                "20260733",
                "20260734",
                "20260735",
                "20260736",
                "20260737",
                "20260738",
                "20260739",
                "20260740",
                "20260741",
                "20260742",
                "20260743",
                "20260744",
                "20260745",
                "20260746",
                "--max-frames",
                "500",
                "--invocation-name",
                "registered_replay",
                "--authorization-json",
                authorization,
            ],
            "registered_analyzer": analyzer_prefix
            + [
                "--raw-root",
                (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-development/v1"
                ),
                "--output-root",
                (
                    "/private/tmp/"
                    "cbf2026-two-range-reacquisition-analysis/v1"
                ),
                "--invocation-name",
                "registered_analyzer",
                "--authorization-json",
                authorization,
            ],
        }

        observed = registrar.production_command_contract(sources)

        self.assertEqual(tuple(observed), tuple(expected))
        self.assertEqual(observed, expected)
        for argv in observed.values():
            self.assertTrue(argv)
            self.assertTrue(all(type(token) is str and token for token in argv))
            self.assertFalse(
                any(
                    any(symbol in token for symbol in ("*", "?", ";", "|", "&"))
                    for token in argv
                )
            )

    def test_canonical_protocol_embeds_all_bound_contracts_in_strict_order(self):
        sources = {
            name: source_record(f"/private/tmp/sources/{name}", index + 1)
            for index, name in enumerate(
                (
                    "implementation_plan",
                    "two_range_reacquisition_source",
                    "predictive_wnls_source",
                    "fixture_extractor_source",
                    "replay_source",
                    "analyzer_source",
                    "registrar_source",
                    "mechanism_fixture",
                    "mechanism_fixture_manifest",
                    "truth_data",
                    "input_manifest",
                )
            )
        }
        process = source_record(
            (
                "/private/tmp/cbf2026-predictive-wnls-development/"
                "stage1-v4/predictive-wnls-development.jsonl.gz"
            ),
            30,
        )
        comparators = {
            "v4_replay_root": (
                "/private/tmp/"
                "cbf2026-predictive-wnls-development/stage1-v4"
            ),
            "v4_replay_manifest": source_record(
                (
                    "/private/tmp/cbf2026-predictive-wnls-development/"
                    "stage1-v4/manifest.json"
                ),
                29,
            ),
            "v4_compressed_process": process,
            "v4_decompressed_process": {
                **process,
                "sha256": f"{31:064x}",
            },
            "v4_analysis_root": (
                "/private/tmp/"
                "cbf2026-predictive-wnls-development-analysis/stage1-v4"
            ),
            "v4_analysis_manifest": source_record(
                (
                    "/private/tmp/"
                    "cbf2026-predictive-wnls-development-analysis/"
                    "stage1-v4/manifest.json"
                ),
                32,
            ),
            "v4_analysis_json": source_record(
                (
                    "/private/tmp/"
                    "cbf2026-predictive-wnls-development-analysis/"
                    "stage1-v4/predictive-wnls-development.json"
                ),
                33,
            ),
            "v4_analysis_markdown": source_record(
                (
                    "/private/tmp/"
                    "cbf2026-predictive-wnls-development-analysis/"
                    "stage1-v4/predictive-wnls-development.md"
                ),
                34,
            ),
            "legacy_baseline_process": source_record(
                "/private/tmp/frozen/baseline/calibration.jsonl.gz",
                35,
            ),
            "legacy_baseline_protocol_json_sha256": f"{36:064x}",
        }

        protocol = registrar._build_protocol(
            head="a" * 40,
            sources=sources,
            comparators=comparators,
        )

        self.assertEqual(
            tuple(protocol),
            (
                "schema_id",
                "registration_schema_id",
                "protocol_id",
                "implementation_parent_commit",
                "binding_design",
                "sources",
                "comparators",
                "experiment",
                "method_contract",
                "estimator_constants",
                "status_contract",
                "raw_schema",
                "analysis_schema",
                "gates",
                "disk_contract",
                "invocations",
                "evidence_lifecycle",
                "authorization",
                "commands",
            ),
        )
        self.assertEqual(protocol["implementation_parent_commit"], "a" * 40)
        self.assertEqual(protocol["sources"], sources)
        self.assertEqual(protocol["comparators"], comparators)
        for section in (
            "binding_design",
            "sources",
            "comparators",
            "experiment",
            "method_contract",
            "estimator_constants",
            "status_contract",
            "raw_schema",
            "analysis_schema",
            "gates",
            "disk_contract",
            "invocations",
            "evidence_lifecycle",
            "authorization",
            "commands",
        ):
            self.assertIsNotNone(protocol[section])
        self.assertEqual(
            protocol["experiment"],
            {
                "method": "two_range_private_branch_reacquisition",
                "seeds": list(range(20260727, 20260747)),
                "frames": list(range(500)),
                "robots": list(range(1, 15)),
                "expected_rows": 140000,
                "key_order": [
                    "method",
                    "seed",
                    "frame_index",
                    "robot_id",
                ],
                "ranging_sigma_m": 0.5,
                "frame_dt_seconds": 0.5,
                "measurement_seed_contract": "cbf2026-range-v1",
                "evidence_class": (
                    "paired_single_trajectory_development_only"
                ),
            },
        )
        self.assertEqual(
            protocol["estimator_constants"],
            {
                "maximum_public_prediction_age": 2,
                "innovation_reference_quantile": 11.829007011943707,
                "candidate_dedup_m": 1e-9,
                "motion_covariance_per_frame": 0.25,
                "reacquisition_reduced_cost_max": 9.0,
                "maximum_error_m": 50.0,
            },
        )
        self.assertEqual(
            protocol["disk_contract"],
            {
                "launch_minimum_free_bytes": 8_000_000_000,
                "live_minimum_free_bytes": 6_000_000_000,
                "raw_bundle_max_allocated_bytes": 2_000_000_000,
                "compact_bundle_max_allocated_bytes": 10_000_000,
            },
        )
        self.assertEqual(
            protocol["evidence_lifecycle"],
            {
                "preexisting_target_allowed": False,
                "no_follow": True,
                "descriptor_pinning": True,
                "transactional_publication": True,
                "fsync_required": True,
                "terminal_manifest_required": True,
                "failure_retained": True,
                "paper_gate": "CLOSED",
            },
        )
        self.assertEqual(
            protocol["authorization"],
            {
                "implementation_plan_approved": True,
                "protocol_preflight_required": True,
                "deterministic_smoke_review_required": True,
                "registered_full_grid_authorization": (
                    "pending_external_record"
                ),
                "authorization_record_schema": (
                    "cbf2026-two-range-reacquisition-registration-v1"
                ),
                "authorization_record_path": (
                    "docs/diagnostics/reviews/"
                    "2026-07-30-cbf2026-two-range-reacquisition-"
                    "registered-authorization.json"
                ),
            },
        )
        self.assertEqual(
            protocol["analysis_schema"]["manifest_fields"],
            [
                "schema_id",
                "protocol_id",
                "invocation_name",
                "status",
                "method",
                "output_root",
                "protocol_identity",
                "authorization_identity",
                "source_identities",
                "output_identities",
                "expected_rows",
                "observed_rows",
                "disk_contract",
                "started_at",
                "completed_at",
                "error",
            ],
        )
        self.assertEqual(
            tuple(protocol["commands"]),
            (
                "smoke_a",
                "smoke_b",
                "smoke_analyzer_a",
                "smoke_analyzer_b",
                "registered_replay",
                "registered_analyzer",
            ),
        )
        serialized_contracts = repr(
            (
                protocol["raw_schema"]["row_scalar_contracts"],
                protocol["raw_schema"]["row_array_contracts"],
                protocol["raw_schema"]["nested_field_orders"],
                protocol["raw_schema"]["branch_field_contracts"],
                protocol["analysis_schema"]["nested_field_orders"],
                protocol["analysis_schema"]["value_contracts"],
            )
        )
        for symbolic_name in (
            "ROW_INVOCATION_NAMES",
            "SMOKE_CASE_IDS",
            "BRANCH_IDS",
            "BRANCH_FIELDS",
            "SOLVER_RESULT_FIELDS",
            "ANALYSIS_FIELDS",
            "GATE_RECORD_FIELDS",
        ):
            self.assertNotIn(symbolic_name, serialized_contracts)

    def test_analysis_schema_literals_distinguish_registered_and_smoke(self):
        analysis_schema = canonical_protocol()["analysis_schema"]

        self.assertEqual(
            analysis_schema["value_contracts"]["decision"],
            (
                "registered_enum:pass,fail;"
                "smoke_enum:smoke_pass,smoke_fail"
            ),
        )
        self.assertEqual(
            analysis_schema["value_contracts"]["scientific_gates"],
            (
                "registered_ordered_list:exactly_nine_gate_records;"
                "smoke_ordered_list:empty"
            ),
        )
        list_contract = analysis_schema["list_order_and_cardinality"]
        self.assertEqual(
            list_contract["registered_scientific_gate_count"],
            9,
        )
        self.assertEqual(list_contract["smoke_scientific_gate_count"], 0)
        self.assertNotIn("scientific_gate_count", list_contract)

    def test_strict_validator_rejects_reordered_top_level_or_section(self):
        protocol = canonical_protocol()
        registrar._validate_protocol(protocol)

        reordered_top = copy.deepcopy(protocol)
        schema = reordered_top.pop("schema_id")
        reordered_top["schema_id"] = schema
        with self.assertRaisesRegex(ValueError, "top-level order"):
            registrar._validate_protocol(reordered_top)

        reordered_section = copy.deepcopy(protocol)
        experiment = reordered_section["experiment"]
        method_id = experiment.pop("method")
        experiment["method"] = method_id
        with self.assertRaisesRegex(ValueError, "experiment.*order"):
            registrar._validate_protocol(reordered_section)

    def test_value_validator_rejects_coercions_nonfinite_and_unsafe_values(self):
        mutations = (
            (
                "boolean integer",
                lambda value: value["sources"]["truth_data"].__setitem__(
                    "size", True
                ),
                "non-Boolean integer",
            ),
            (
                "nonfinite estimator constant",
                lambda value: value["estimator_constants"].__setitem__(
                    "candidate_dedup_m", float("inf")
                ),
                "finite",
            ),
            (
                "noncanonical hash",
                lambda value: value["sources"]["truth_data"].__setitem__(
                    "sha256", "A" * 64
                ),
                "SHA-256",
            ),
            (
                "noncanonical OID",
                lambda value: value.__setitem__(
                    "implementation_parent_commit", "A" * 40
                ),
                "OID",
            ),
            (
                "relative source path",
                lambda value: value["sources"]["truth_data"].__setitem__(
                    "path", "relative/data.json"
                ),
                "absolute",
            ),
            (
                "boolean coercion",
                lambda value: value["evidence_lifecycle"].__setitem__(
                    "no_follow", 1
                ),
                "Boolean",
            ),
            (
                "shell wildcard",
                lambda value: value["commands"]["smoke_a"].append("*"),
                "shell",
            ),
        )
        registrar._validate_protocol(canonical_protocol())
        for label, mutate, message in mutations:
            with self.subTest(label=label):
                protocol = canonical_protocol()
                mutate(protocol)
                with self.assertRaisesRegex(ValueError, message):
                    registrar._validate_protocol(protocol)

    def test_cardinality_validator_rejects_incomplete_grid_gates_or_smoke(self):
        mutations = (
            (
                "grid",
                lambda value: value["experiment"]["frames"].pop(),
            ),
            (
                "scientific gates",
                lambda value: value["gates"]["scientific_gate_order"].pop(),
            ),
            (
                "integrity gates",
                lambda value: value["gates"]["integrity_gate_order"].pop(),
            ),
            (
                "smoke rows",
                lambda value: value["invocations"]["smoke_a"].__setitem__(
                    "expected_rows", 17
                ),
            ),
        )
        registrar._validate_protocol(canonical_protocol())
        for label, mutate in mutations:
            with self.subTest(label=label):
                protocol = canonical_protocol()
                mutate(protocol)
                with self.assertRaisesRegex(ValueError, "cardinality"):
                    registrar._validate_protocol(protocol)

    def test_wrong_protocol_registration_raw_or_analysis_schema_id_rejects(self):
        mutations = (
            lambda value: value.__setitem__("schema_id", "wrong"),
            lambda value: value.__setitem__(
                "registration_schema_id", "wrong"
            ),
            lambda value: value["raw_schema"].__setitem__(
                "schema_id", "wrong"
            ),
            lambda value: value["analysis_schema"].__setitem__(
                "schema_id", "wrong"
            ),
        )
        for mutate in mutations:
            protocol = canonical_protocol()
            mutate(protocol)
            with self.assertRaisesRegex(ValueError, "schema"):
                registrar._validate_protocol(protocol)

    def test_changed_frozen_threshold_rejects(self):
        protocol = canonical_protocol()
        protocol["estimator_constants"]["innovation_reference_quantile"] += 1.0

        with self.assertRaisesRegex(ValueError, "threshold"):
            registrar._validate_protocol(protocol)

    def test_changed_seed_frame_or_robot_grid_rejects(self):
        mutations = (
            lambda value: value["experiment"]["seeds"].__setitem__(
                10, 20260800
            ),
            lambda value: value["experiment"]["frames"].__setitem__(250, 249),
            lambda value: value["experiment"]["robots"].__setitem__(13, 15),
        )
        for mutate in mutations:
            protocol = canonical_protocol()
            mutate(protocol)
            with self.assertRaisesRegex(ValueError, "grid"):
                registrar._validate_protocol(protocol)

    def test_reordered_producer_or_serialized_smoke_cases_rejects(self):
        producer_cases = registrar.replay.SMOKE_CASE_IDS
        with mock.patch.object(
            registrar.replay,
            "SMOKE_CASE_IDS",
            (producer_cases[1], producer_cases[0], *producer_cases[2:]),
        ):
            with self.assertRaisesRegex(ValueError, "smoke case"):
                registrar.production_invocation_contract()

        protocol = canonical_protocol()
        synthetic = protocol["method_contract"]["synthetic_declaration"]
        cases = list(synthetic["synthetic_cases"])
        cases[0], cases[1] = cases[1], cases[0]
        synthetic["synthetic_cases"] = tuple(cases)
        with self.assertRaisesRegex(ValueError, "smoke case"):
            registrar._validate_protocol(protocol)

    def test_changed_comparator_path_or_hash_rejects(self):
        mutations = (
            lambda value: value["comparators"].__setitem__(
                "v4_replay_root",
                "/private/tmp/hash-matching-relocated-copy",
            ),
            lambda value: value["comparators"][
                "v4_analysis_json"
            ].__setitem__("sha256", "f" * 64),
        )
        for mutate in mutations:
            protocol = canonical_protocol()
            mutate(protocol)
            with self.assertRaisesRegex(ValueError, "comparator"):
                registrar._validate_protocol(protocol)

    def test_changed_authorization_record_schema_or_path_rejects(self):
        mutations = (
            (
                "authorization_record_schema",
                "cbf2026-two-range-reacquisition-registration-v2",
            ),
            (
                "authorization_record_path",
                (
                    "docs/diagnostics/reviews/"
                    "replacement-registered-authorization.json"
                ),
            ),
        )
        for field, replacement in mutations:
            with self.subTest(field=field):
                protocol = canonical_protocol()
                protocol["authorization"][field] = replacement
                with self.assertRaisesRegex(ValueError, "authorization"):
                    registrar._validate_protocol(protocol)

    def test_extra_or_renamed_comparator_binding_key_rejects(self):
        extra = dict(registrar.EXPECTED_COMPARATOR_BINDINGS)
        extra["unexpected_v4_copy"] = "0" * 64
        renamed = dict(registrar.EXPECTED_COMPARATOR_BINDINGS)
        renamed["renamed_v4_manifest_sha256"] = renamed.pop(
            "v4_manifest_sha256"
        )
        for label, declaration in (
            ("extra", extra),
            ("renamed", renamed),
        ):
            with self.subTest(label=label):
                with mock.patch.object(
                    registrar,
                    "EXPECTED_COMPARATOR_BINDINGS",
                    declaration,
                ):
                    with self.assertRaisesRegex(
                        ValueError,
                        "comparator binding",
                    ):
                        registrar._resolve_comparators()

    def test_changed_nested_or_lifecycle_contract_rejects(self):
        mutations = (
            lambda value: value["status_contract"][
                "prior_used_semantics"
            ].__setitem__("fim", True),
            lambda value: value["raw_schema"]["row_fields"].__setitem__(
                0,
                "wrong_field",
            ),
            lambda value: value["analysis_schema"][
                "analysis_fields"
            ].__setitem__(0, "wrong_field"),
            lambda value: value["gates"].__setitem__(
                "aggregate_decision_rule",
                "replacement_rule",
            ),
            lambda value: value["disk_contract"].__setitem__(
                "launch_minimum_free_bytes",
                8_000_000_001,
            ),
            lambda value: value["invocations"]["registered_replay"].__setitem__(
                "retry_allowed",
                True,
            ),
            lambda value: value["evidence_lifecycle"].__setitem__(
                "paper_gate",
                "OPEN",
            ),
            lambda value: value["commands"]["registered_replay"].__setitem__(
                value["commands"]["registered_replay"].index("--output-root")
                + 1,
                "/private/tmp/replacement-registered-root",
            ),
        )
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                protocol = canonical_protocol()
                mutate(protocol)
                with self.assertRaisesRegex(ValueError, "exact protocol"):
                    registrar._validate_protocol(protocol)

    def test_nested_false_cannot_replace_zero_integrity_threshold(self):
        protocol = canonical_protocol()
        first_gate = protocol["gates"]["integrity_gate_order"][0]
        protocol["gates"]["integrity_gate_records"][first_gate][
            "threshold"
        ] = False

        with self.assertRaisesRegex(
            ValueError,
            "non-Boolean|exact protocol",
        ):
            registrar._validate_protocol(protocol)


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir="/private/tmp")
        self.root = Path(self.temporary.name).absolute()
        self.repository = self.root / "repository"
        self.implementation_root = Path(registrar.__file__).resolve().parents[2]
        subprocess.run(
            [
                "git",
                "clone",
                "-q",
                "--shared",
                str(self.implementation_root),
                str(self.repository),
            ],
            check=True,
        )
        subprocess.run(
            [
                "git",
                "-C",
                str(self.repository),
                "config",
                "user.email",
                "tests@example.invalid",
            ],
            check=True,
        )
        subprocess.run(
            [
                "git",
                "-C",
                str(self.repository),
                "config",
                "user.name",
                "Registrar Tests",
            ],
            check=True,
        )
        relative = Path("scripts/diagnostics/register_two_range_reacquisition.py")
        shutil.copy2(
            self.implementation_root / relative,
            self.repository / relative,
        )
        subprocess.run(
            ["git", "-C", str(self.repository), "add", str(relative)],
            check=True,
        )
        subprocess.run(
            [
                "git",
                "-C",
                str(self.repository),
                "commit",
                "--allow-empty",
                "-qm",
                "registrar test fixture",
            ],
            check=True,
        )
        self.docs = self.repository / "docs" / "diagnostics"

    def tearDown(self):
        self.temporary.cleanup()

    def register(self, stem="protocol"):
        markdown = self.docs / f"{stem}.md"
        output_json = self.docs / f"{stem}.json"
        result = registrar.register_two_range_protocol(
            repository_root=self.repository,
            output_markdown=markdown,
            output_json=output_json,
        )
        return result, markdown, output_json

    def test_registers_bound_sources_and_comparators_to_paired_outputs(self):
        result, markdown, output_json = self.register()

        self.assertEqual(result, (markdown, output_json))
        payload = json.loads(output_json.read_bytes())
        head = subprocess.run(
            ["git", "-C", str(self.repository), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        self.assertEqual(payload["implementation_parent_commit"], head)
        self.assertEqual(tuple(payload["sources"]), registrar.SOURCE_MEMBER_NAMES)
        for record in payload["sources"].values():
            self.assertEqual(
                tuple(record),
                ("path", "device", "inode", "size", "mtime_ns", "sha256"),
            )
            self.assertEqual(record["sha256"], sha256(Path(record["path"])))
        comparators = payload["comparators"]
        self.assertEqual(
            tuple(comparators),
            registrar.PROTOCOL_SECTION_FIELDS["comparators"],
        )
        self.assertEqual(
            comparators["v4_replay_manifest"]["sha256"],
            (
                "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223"
            ),
        )
        self.assertEqual(
            comparators["v4_compressed_process"]["sha256"],
            (
                "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b"
            ),
        )
        self.assertEqual(
            comparators["v4_decompressed_process"]["sha256"],
            (
                "7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1"
            ),
        )
        for field in ("path", "device", "inode", "size", "mtime_ns"):
            self.assertEqual(
                comparators["v4_compressed_process"][field],
                comparators["v4_decompressed_process"][field],
            )
        self.assertTrue(
            markdown.read_text().startswith(
                "# CBF2026 Two-Range Reacquisition Protocol\n"
            )
        )

    def test_dirty_required_source_rejects_before_outputs(self):
        source = (
            self.repository / "scripts/diagnostics/predictive_wnls.py"
        )
        source.write_bytes(source.read_bytes() + b"\n# dirty test mutation\n")
        markdown = self.docs / "dirty.md"
        output_json = self.docs / "dirty.json"

        with self.assertRaisesRegex(ValueError, "dirty"):
            registrar.register_two_range_protocol(
                repository_root=self.repository,
                output_markdown=markdown,
                output_json=output_json,
            )

        self.assertFalse(markdown.exists())
        self.assertFalse(output_json.exists())

    def test_source_path_size_and_sha_drift_between_reads_removes_outputs(self):
        source = self.repository / (
            "scripts/diagnostics/two_range_reacquisition.py"
        )
        markdown = self.docs / "drift.md"
        output_json = self.docs / "drift.json"
        real_serializer = registrar._strict_json_bytes

        def replace_source_after_initial_binding(value):
            payload = real_serializer(value)
            replacement = source.with_name("replacement.py")
            replacement.write_bytes(source.read_bytes() + b"\n# drift\n")
            replacement.replace(source)
            return payload

        with mock.patch.object(
            registrar,
            "_strict_json_bytes",
            side_effect=replace_source_after_initial_binding,
        ):
            with self.assertRaisesRegex(ValueError, "changed|dirty"):
                registrar.register_two_range_protocol(
                    repository_root=self.repository,
                    output_markdown=markdown,
                    output_json=output_json,
                )

        self.assertFalse(markdown.exists())
        self.assertFalse(output_json.exists())

    def test_required_source_symlink_rejects_before_outputs(self):
        source = self.repository / (
            "scripts/diagnostics/two_range_reacquisition.py"
        )
        relocated = self.root / "relocated-two-range-reacquisition.py"
        relocated.write_bytes(source.read_bytes())
        source.unlink()
        source.symlink_to(relocated)
        markdown = self.docs / "source-symlink.md"
        output_json = self.docs / "source-symlink.json"

        with self.assertRaisesRegex(ValueError, "symbolic-link"):
            registrar.register_two_range_protocol(
                repository_root=self.repository,
                output_markdown=markdown,
                output_json=output_json,
            )

        self.assertFalse(markdown.exists())
        self.assertFalse(output_json.exists())

    def test_same_name_identical_inode_swap_before_open_rejects(self):
        source = self.repository / (
            "scripts/diagnostics/two_range_reacquisition.py"
        )
        payload = source.read_bytes()
        replacement = source.with_name("identical-replacement.py")
        replacement.write_bytes(payload)
        original_inode = source.stat().st_ino
        replacement_inode = replacement.stat().st_ino
        self.assertNotEqual(original_inode, replacement_inode)
        real_open = registrar.os.open
        swapped = False

        def swap_between_observation_and_open(
            candidate,
            flags,
            *args,
            **kwargs,
        ):
            nonlocal swapped
            if not swapped and Path(candidate).name == source.name:
                replacement.replace(source)
                swapped = True
            return real_open(candidate, flags, *args, **kwargs)

        with mock.patch.object(
            registrar.os,
            "open",
            side_effect=swap_between_observation_and_open,
        ):
            with self.assertRaisesRegex(ValueError, "identity changed"):
                registrar._read_bound_source(
                    source,
                    expected_sha256=hashlib.sha256(payload).hexdigest(),
                )

        self.assertTrue(swapped)
        self.assertEqual(source.stat().st_ino, replacement_inode)
        self.assertEqual(source.read_bytes(), payload)

    def test_comparator_relocation_symlink_and_hash_copy_cannot_rebind(self):
        expected = registrar.EXPECTED_COMPARATOR_BINDINGS
        authoritative_root = Path(expected["v4_replay_root"])
        authoritative_manifest = authoritative_root / "manifest.json"
        copied_root = self.root / "relocated-v4-copy"
        copied_root.mkdir()
        copied_manifest = copied_root / "manifest.json"
        shutil.copy2(authoritative_manifest, copied_manifest)
        self.assertEqual(
            sha256(copied_manifest),
            expected["v4_manifest_sha256"],
        )
        symlink_root = self.root / "symlink-v4-root"
        symlink_root.symlink_to(
            authoritative_root,
            target_is_directory=True,
        )
        missing_root = self.root / "unresolved-v4-root"

        for label, replacement in (
            ("hash_copy", copied_root),
            ("symlink", symlink_root),
            ("unresolved", missing_root),
        ):
            declaration = dict(expected)
            declaration["v4_replay_root"] = str(replacement)
            with self.subTest(label=label):
                with mock.patch.object(
                    registrar,
                    "EXPECTED_COMPARATOR_BINDINGS",
                    declaration,
                ):
                    with self.assertRaisesRegex(
                        ValueError,
                        "comparator binding",
                    ):
                        registrar._resolve_comparators()

        with self.assertRaisesRegex(ValueError, "symbolic-link"):
            registrar._read_bound_source(
                symlink_root / "manifest.json",
                expected_sha256=expected["v4_manifest_sha256"],
            )

    def test_comparator_gzip_hash_domains_share_one_pinned_descriptor(self):
        process = self.root / "comparator.jsonl.gz"
        with process.open("wb") as raw:
            with gzip.GzipFile(
                filename="",
                fileobj=raw,
                mode="wb",
                mtime=0,
            ) as stream:
                stream.write(b'{"row":1}\n')
        replacement = self.root / "replacement.jsonl.gz"
        replacement.write_bytes(process.read_bytes())
        compressed_sha256 = sha256(process)
        decompressed_sha256 = hashlib.sha256(b'{"row":1}\n').hexdigest()
        real_fdopen = registrar.os.fdopen

        def replace_path_after_descriptor_pin(descriptor, *args, **kwargs):
            replacement.replace(process)
            return real_fdopen(descriptor, *args, **kwargs)

        with mock.patch.object(
            registrar.os,
            "fdopen",
            side_effect=replace_path_after_descriptor_pin,
        ):
            with self.assertRaisesRegex(ValueError, "identity changed"):
                registrar._read_bound_gzip_source(
                    process,
                    expected_compressed_sha256=compressed_sha256,
                    expected_decompressed_sha256=decompressed_sha256,
                )

    def test_preexisting_registered_root_rejects_before_outputs(self):
        root = Path(registrar.ROOTS["smoke_a"])
        self.assertFalse(root.exists())
        self.assertFalse(root.is_symlink())
        root.mkdir()
        markdown = self.docs / "preexisting.md"
        output_json = self.docs / "preexisting.json"
        try:
            with self.assertRaisesRegex(FileExistsError, "root"):
                registrar.register_two_range_protocol(
                    repository_root=self.repository,
                    output_markdown=markdown,
                    output_json=output_json,
                )
            self.assertFalse(markdown.exists())
            self.assertFalse(output_json.exists())
        finally:
            root.rmdir()

    def test_broken_symlink_registered_root_rejects_before_outputs(self):
        root = Path(registrar.ROOTS["smoke_b"])
        self.assertFalse(root.exists())
        self.assertFalse(root.is_symlink())
        root.symlink_to(self.root / "missing-target", target_is_directory=True)
        markdown = self.docs / "root-symlink.md"
        output_json = self.docs / "root-symlink.json"
        try:
            with self.assertRaisesRegex(ValueError, "symbolic-link"):
                registrar.register_two_range_protocol(
                    repository_root=self.repository,
                    output_markdown=markdown,
                    output_json=output_json,
                )
            self.assertFalse(markdown.exists())
            self.assertFalse(output_json.exists())
        finally:
            root.unlink()

    def test_retired_v2_v3_v4_roots_reject_for_each_invocation(self):
        legacy_roots = (
            "/private/tmp/cbf2026-predictive-wnls-smoke-a",
            "/private/tmp/cbf2026-predictive-wnls-smoke-b",
            "/private/tmp/cbf2026-predictive-wnls-development/stage1-v2",
            (
                "/private/tmp/"
                "cbf2026-predictive-wnls-development-analysis/stage1-v2"
            ),
            "/private/tmp/cbf2026-predictive-wnls-smoke-v3-a",
            "/private/tmp/cbf2026-predictive-wnls-smoke-v3-b",
            "/private/tmp/cbf2026-predictive-wnls-development/stage1-v3",
            (
                "/private/tmp/"
                "cbf2026-predictive-wnls-development-analysis/stage1-v3"
            ),
            "/private/tmp/cbf2026-predictive-wnls-smoke-v4-a",
            "/private/tmp/cbf2026-predictive-wnls-smoke-v4-b",
            "/private/tmp/cbf2026-predictive-wnls-development/stage1-v4",
            (
                "/private/tmp/"
                "cbf2026-predictive-wnls-development-analysis/stage1-v4"
            ),
        )
        for invocation_name in registrar.INVOCATION_MEMBER_NAMES:
            for retired_root in legacy_roots:
                with self.subTest(
                    invocation_name=invocation_name,
                    retired_root=retired_root,
                ):
                    with mock.patch.dict(
                        registrar.ROOTS,
                        {invocation_name: retired_root},
                    ):
                        with self.assertRaisesRegex(
                            ValueError,
                            "retired|root contract",
                        ):
                            registrar._assert_registered_roots_absent()

    def test_all_six_exact_registered_roots_are_guarded(self):
        self.assertEqual(
            tuple(registrar.ROOTS),
            registrar.INVOCATION_MEMBER_NAMES,
        )
        for invocation_name, root_string in registrar.ROOTS.items():
            root = Path(root_string)
            missing_parents = []
            parent = root.parent
            while not parent.exists():
                missing_parents.append(parent)
                parent = parent.parent
            for missing_parent in reversed(missing_parents):
                missing_parent.mkdir()
            root.mkdir()
            try:
                with self.subTest(invocation_name=invocation_name):
                    with self.assertRaisesRegex(
                        FileExistsError,
                        invocation_name,
                    ):
                        registrar._assert_registered_roots_absent()
            finally:
                root.rmdir()
                for missing_parent in missing_parents:
                    missing_parent.rmdir()

    def test_non_ancestor_implementation_parent_rejects_before_outputs(self):
        tree = subprocess.run(
            ["git", "-C", str(self.repository), "write-tree"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        unrelated = subprocess.run(
            ["git", "-C", str(self.repository), "commit-tree", tree],
            check=True,
            capture_output=True,
            input="unrelated implementation parent\n",
            text=True,
        ).stdout.strip()
        subprocess.run(
            [
                "git",
                "-C",
                str(self.repository),
                "switch",
                "-q",
                "-c",
                "unrelated-parent",
                unrelated,
            ],
            check=True,
        )
        markdown = self.docs / "non-ancestor.md"
        output_json = self.docs / "non-ancestor.json"
        try:
            with self.assertRaisesRegex(ValueError, "ancestor"):
                registrar.register_two_range_protocol(
                    repository_root=self.repository,
                    output_markdown=markdown,
                    output_json=output_json,
                )
            self.assertFalse(markdown.exists())
            self.assertFalse(output_json.exists())
        finally:
            markdown.unlink(missing_ok=True)
            output_json.unlink(missing_ok=True)

    def test_detached_implementation_parent_rejects_before_outputs(self):
        subprocess.run(
            [
                "git",
                "-C",
                str(self.repository),
                "switch",
                "-q",
                "--detach",
            ],
            check=True,
        )
        markdown = self.docs / "detached.md"
        output_json = self.docs / "detached.json"
        with self.assertRaisesRegex(ValueError, "detached"):
            registrar.register_two_range_protocol(
                repository_root=self.repository,
                output_markdown=markdown,
                output_json=output_json,
            )
        self.assertFalse(markdown.exists())
        self.assertFalse(output_json.exists())

    def test_circular_protocol_parent_rejects_before_outputs(self):
        generated_protocol = self.repository / (
            "docs/diagnostics/"
            "2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json"
        )
        generated_protocol.write_text("{}\n")
        subprocess.run(
            [
                "git",
                "-C",
                str(self.repository),
                "add",
                str(generated_protocol.relative_to(self.repository)),
            ],
            check=True,
        )
        subprocess.run(
            [
                "git",
                "-C",
                str(self.repository),
                "commit",
                "-qm",
                "circular protocol parent fixture",
            ],
            check=True,
        )
        markdown = self.docs / "circular-parent.md"
        output_json = self.docs / "circular-parent.json"
        try:
            with self.assertRaisesRegex(ValueError, "circular"):
                registrar.register_two_range_protocol(
                    repository_root=self.repository,
                    output_markdown=markdown,
                    output_json=output_json,
                )
            self.assertFalse(markdown.exists())
            self.assertFalse(output_json.exists())
        finally:
            markdown.unlink(missing_ok=True)
            output_json.unlink(missing_ok=True)

    def test_partial_output_write_removes_both_protocol_files(self):
        markdown = self.docs / "partial.md"
        output_json = self.docs / "partial.json"
        real_write = registrar.os.write
        write_calls = 0

        def partial_then_stall(descriptor, payload):
            nonlocal write_calls
            write_calls += 1
            if write_calls == 1:
                prefix_size = max(1, len(payload) // 2)
                return real_write(descriptor, payload[:prefix_size])
            if write_calls == 2:
                return 0
            return real_write(descriptor, payload)

        try:
            with mock.patch.object(
                registrar.os,
                "write",
                side_effect=partial_then_stall,
            ):
                with self.assertRaisesRegex(OSError, "short write|partial"):
                    registrar.register_two_range_protocol(
                        repository_root=self.repository,
                        output_markdown=markdown,
                        output_json=output_json,
                    )
            self.assertFalse(markdown.exists())
            self.assertFalse(output_json.exists())
        finally:
            markdown.unlink(missing_ok=True)
            output_json.unlink(missing_ok=True)

    def test_cleanup_preserves_foreign_inode_swapped_after_ownership_check(self):
        markdown = self.docs / "cleanup-race.md"
        output_json = self.docs / "cleanup-race.json"
        foreign_staging = self.docs / "foreign-cleanup-race.md"
        foreign_payload = b"foreign inode must survive cleanup\n"
        foreign_staging.write_bytes(foreign_payload)
        foreign_inode = foreign_staging.stat().st_ino
        real_matches = registrar._path_matches_descriptor
        real_fsync = registrar.os.fsync
        injected = False
        fsync_faulted = False

        def inject_after_ownership_check(**kwargs):
            nonlocal injected
            matches = real_matches(**kwargs)
            if (
                matches
                and kwargs["name"] == markdown.name
                and not injected
            ):
                foreign_staging.replace(markdown)
                injected = True
            return matches

        def retain_primary_cleanup_note(descriptor):
            nonlocal fsync_faulted
            if injected and not fsync_faulted:
                fsync_faulted = True
                raise OSError("cleanup fsync fault")
            return real_fsync(descriptor)

        with (
            mock.patch.object(
                registrar,
                "_path_matches_descriptor",
                side_effect=inject_after_ownership_check,
            ),
            mock.patch.object(
                registrar.os,
                "fsync",
                side_effect=retain_primary_cleanup_note,
            ),
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "primary final probe failure",
            ) as raised:
                registrar._write_paired_outputs(
                    output_markdown=markdown,
                    output_json=output_json,
                    markdown_payload=b"owned markdown\n",
                    json_payload=b'{"owned":true}\n',
                    final_probe=lambda: (_ for _ in ()).throw(
                        RuntimeError("primary final probe failure"),
                    ),
                )

        self.assertTrue(injected)
        self.assertTrue(fsync_faulted)
        self.assertTrue(markdown.exists())
        self.assertEqual(markdown.stat().st_ino, foreign_inode)
        self.assertEqual(markdown.read_bytes(), foreign_payload)
        self.assertFalse(output_json.exists())
        self.assertTrue(
            any(
                "cleanup fsync fault" in note
                for note in getattr(raised.exception, "__notes__", ())
            )
        )

    def test_output_parent_fault_keeps_primary_error_and_closes_descriptor(self):
        target = self.docs / "parent-fstat.json"
        real_fstat = registrar.os.fstat
        real_close = registrar.os.close
        fstat_faulted = False
        close_faulted = False

        def fail_first_fstat(descriptor):
            nonlocal fstat_faulted
            if not fstat_faulted:
                fstat_faulted = True
                raise OSError("parent fstat fault")
            return real_fstat(descriptor)

        def close_then_fault(descriptor):
            nonlocal close_faulted
            real_close(descriptor)
            if not close_faulted:
                close_faulted = True
                raise OSError("parent close fault")

        with (
            mock.patch.object(
                registrar.os,
                "fstat",
                side_effect=fail_first_fstat,
            ),
            mock.patch.object(
                registrar.os,
                "close",
                side_effect=close_then_fault,
            ),
        ):
            with self.assertRaisesRegex(
                OSError,
                "parent fstat fault",
            ) as raised:
                registrar._open_output_parent(target)

        self.assertTrue(fstat_faulted)
        self.assertTrue(close_faulted)
        self.assertTrue(
            any(
                "parent close fault" in note
                for note in getattr(raised.exception, "__notes__", ())
            )
        )

    def test_markdown_json_mismatch_across_dry_runs_rejects_publication(self):
        markdown = self.docs / "dry-run-mismatch.md"
        output_json = self.docs / "dry-run-mismatch.json"
        real_markdown = registrar._markdown_bytes
        render_count = 0

        def divergent_second_render(protocol, json_payload):
            nonlocal render_count
            render_count += 1
            payload = real_markdown(protocol, json_payload)
            if render_count == 2:
                digest = hashlib.sha256(json_payload).hexdigest().encode()
                payload = payload.replace(digest, b"0" * 64)
            return payload

        try:
            with mock.patch.object(
                registrar,
                "_markdown_bytes",
                side_effect=divergent_second_render,
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "deterministic|Markdown/JSON",
                ):
                    registrar.register_two_range_protocol(
                        repository_root=self.repository,
                        output_markdown=markdown,
                        output_json=output_json,
                    )
            self.assertFalse(markdown.exists())
            self.assertFalse(output_json.exists())
        finally:
            markdown.unlink(missing_ok=True)
            output_json.unlink(missing_ok=True)

    def test_two_dry_runs_are_byte_identical_and_semantically_paired(self):
        _, markdown_a, json_a = self.register("deterministic-a")
        _, markdown_b, json_b = self.register("deterministic-b")

        markdown_a_bytes = markdown_a.read_bytes()
        markdown_b_bytes = markdown_b.read_bytes()
        json_a_bytes = json_a.read_bytes()
        json_b_bytes = json_b.read_bytes()
        self.assertEqual(json_a_bytes, json_b_bytes)
        self.assertEqual(markdown_a_bytes, markdown_b_bytes)

        for markdown_bytes, json_bytes in (
            (markdown_a_bytes, json_a_bytes),
            (markdown_b_bytes, json_b_bytes),
        ):
            payload = json.loads(json_bytes)
            digest = hashlib.sha256(json_bytes).hexdigest().encode()
            self.assertEqual(
                markdown_bytes.count(
                    b"- JSON SHA-256: `" + digest + b"`"
                ),
                1,
            )
            self.assertIn(
                (
                    f"- Protocol ID: `{payload['protocol_id']}`"
                ).encode(),
                markdown_bytes,
            )
            self.assertIn(
                (
                    "- Implementation parent: "
                    f"`{payload['implementation_parent_commit']}`"
                ).encode(),
                markdown_bytes,
            )

    def test_cli_main_registers_paired_outputs(self):
        markdown = self.docs / "cli.md"
        output_json = self.docs / "cli.json"

        result = registrar.main(
            [
                "--repository-root",
                str(self.repository),
                "--output-markdown",
                str(markdown),
                "--output-json",
                str(output_json),
            ]
        )

        self.assertEqual(result, 0)
        self.assertTrue(markdown.is_file())
        self.assertTrue(output_json.is_file())
        payload = json.loads(output_json.read_bytes())
        self.assertEqual(payload["protocol_id"], registrar.PROTOCOL_ID)


if __name__ == "__main__":
    unittest.main()
