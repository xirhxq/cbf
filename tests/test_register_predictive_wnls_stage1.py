import contextlib
import hashlib
import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from scripts.diagnostics import replay_predictive_wnls_recovery as replay
from scripts.diagnostics import register_predictive_wnls_stage1 as registrar


REPOSITORY_SOURCES = {
    "replay_source": "scripts/diagnostics/replay_predictive_wnls_recovery.py",
    "analyzer_source": "scripts/diagnostics/analyze_predictive_wnls_recovery.py",
    "estimator_source": "scripts/diagnostics/predictive_wnls.py",
    "legacy_solver_source": (
        "scripts/diagnostics/replay_localization_calibration.py"
    ),
    "diagnostic_integrity_source": "scripts/diagnostics/run_diagnostic.py",
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def open_fd_count():
    return len(os.listdir("/dev/fd"))


class RegistrarTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir="/private/tmp")
        self.root = Path(self.temporary.name).absolute()
        self.repository = self.root / "repository"
        self.repository.mkdir()
        self.inputs = self.root / "inputs"
        self.inputs.mkdir()
        self.truth = self.inputs / "truth.json"
        self.input_manifest = self.inputs / "manifest.json"
        self.baseline = self.inputs / "baseline.jsonl.gz"
        self.truth.write_bytes(b'{"truth":"literal"}\n')
        self.input_manifest.write_bytes(b'{"manifest":"literal"}\n')
        self.baseline.write_bytes(b"literal-baseline-process\n")
        self.truth_sha256 = sha256(self.truth)
        self.input_manifest_sha256 = sha256(self.input_manifest)
        self.baseline_sha256 = sha256(self.baseline)
        for name, relative in REPOSITORY_SOURCES.items():
            path = self.repository / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"{name}: literal source\n")
        self._git("init", "-q")
        self._git("config", "user.email", "tests@example.invalid")
        self._git("config", "user.name", "Registrar Tests")
        self._git("add", ".")
        self._git("commit", "-qm", "fixture")
        self.docs = self.repository / "docs" / "diagnostics"
        self.docs.mkdir(parents=True)

    def tearDown(self):
        self.temporary.cleanup()

    def _git(self, *arguments, check=True):
        return subprocess.run(
            ["git", "-C", str(self.repository), *arguments],
            check=check,
            capture_output=True,
            text=True,
        )

    @contextlib.contextmanager
    def production_inputs(self):
        patches = (
            mock.patch.object(
                replay, "PRODUCTION_TRUTH_DATA_PATH", str(self.truth)
            ),
            mock.patch.object(
                replay, "PRODUCTION_TRUTH_DATA_SHA256", self.truth_sha256
            ),
            mock.patch.object(
                replay,
                "PRODUCTION_INPUT_MANIFEST_PATH",
                str(self.input_manifest),
            ),
            mock.patch.object(
                replay,
                "PRODUCTION_INPUT_MANIFEST_SHA256",
                self.input_manifest_sha256,
            ),
            mock.patch.object(
                replay,
                "PRODUCTION_BASELINE_PROCESS_PATH",
                str(self.baseline),
            ),
            mock.patch.object(
                replay,
                "PRODUCTION_BASELINE_PROCESS_SHA256",
                self.baseline_sha256,
            ),
            mock.patch.object(
                replay,
                "LEGACY_SOLVER_SHA256",
                sha256(
                    self.repository
                    / REPOSITORY_SOURCES["legacy_solver_source"]
                ),
            ),
        )
        with contextlib.ExitStack() as stack:
            for patch in patches:
                stack.enter_context(patch)
            yield

    def register(self, stem="protocol"):
        markdown = self.docs / f"{stem}.md"
        output_json = self.docs / f"{stem}.json"
        with self.production_inputs():
            result = registrar.register_stage1_protocol(
                repository_root=self.repository,
                output_markdown=markdown,
                output_json=output_json,
            )
        return result, markdown, output_json

    def test_binds_head_hashes_and_exact_frozen_contract(self):
        result, markdown, output_json = self.register()
        payload = json.loads(output_json.read_text())
        head = self._git("rev-parse", "HEAD").stdout.strip()

        self.assertEqual(result, payload)
        self.assertEqual(payload["implementation_parent_commit"], head)
        self.assertEqual(
            set(payload),
            {
                "schema_id",
                "protocol_id",
                "implementation_parent_commit",
                "binding_design",
                "sources",
                "experiment",
                "estimator_constants",
                "status_contract",
                "ablation_contracts",
                "raw_schema",
                "analysis_schema",
                "gates",
                "disk_contract",
                "invocations",
                "evidence_lifecycle",
                "commands",
            },
        )
        expected_paths = {
            "truth_data": self.truth,
            "input_manifest": self.input_manifest,
            "baseline_process": self.baseline,
            **{
                name: self.repository / relative
                for name, relative in REPOSITORY_SOURCES.items()
            },
        }
        self.assertEqual(set(payload["sources"]), set(expected_paths))
        for name, path in expected_paths.items():
            self.assertEqual(payload["sources"][name]["path"], str(path))
            self.assertEqual(payload["sources"][name]["sha256"], sha256(path))

        self.assertEqual(
            payload["experiment"],
            {
                "stage": 1,
                "evidence_class": (
                    "paired_single_trajectory_development_only"
                ),
                "variants": [
                    "prediction_expiry",
                    "fresh_reference_qualification",
                    "predictive_multistart",
                ],
                "frame_dt_seconds": 0.5,
                "ranging_sigma_m": 0.5,
                "measurement_seed_contract": "cbf2026-range-v1",
                "execution_order": [
                    "variant",
                    "seed",
                    "frame_index",
                    "ascending_global_robot_id",
                ],
                "baseline_graph_case": "dynamic_dag_wnls",
                "baseline_initialization_policy": (
                    "deployment_restart_before_first_finite_v1"
                ),
                "range_noise_seeds": list(range(20260727, 20260747)),
                "max_frames": None,
            },
        )
        self.assertEqual(
            payload["estimator_constants"],
            {
                "max_prediction_age_frames": 2,
                "motion_sigma_m_per_frame": 0.5,
                "innovation_q_max": 11.829007011943707,
                "reacquisition_reduced_cost_max": 9.0,
                "catastrophic_error_m": 50.0,
                "max_proposals_per_candidate": 50,
                "initial_damping": 0.001,
                "minimum_damping": 1e-15,
                "maximum_damping": 1e15,
                "damping_factor": 10.0,
                "scale_aware_stationarity": 1e-6,
                "relative_spectral_threshold": 1e-12,
                "representable_step_relative_threshold": 1e-12,
                "candidate_dedup_m": 1e-9,
                "relative_tie_tolerance": 1e-12,
            },
        )
        self.assertEqual(
            payload["gates"],
            {
                "maximum_published_error_m_strictly_below": 50.0,
                "maximum_fresh_error_m_strictly_below": 50.0,
                "paired_both_fresh_p95_must_not_worsen": True,
                "fresh_availability_max_drop_fraction": 0.02,
                "fresh_or_predicted_min_fraction": 0.95,
                "maximum_prediction_age_frames": 2,
                "qualification_anchor_violations_allowed": 0,
                "current_frame_provenance_violations_allowed": 0,
                "ascending_dag_violations_allowed": 0,
            },
        )
        self.assertEqual(
            payload["disk_contract"],
            {
                "launch_minimum_free_bytes": 8_000_000_000,
                "live_minimum_free_bytes": 6_000_000_000,
                "raw_bundle_max_allocated_bytes": 2_000_000_000,
                "compact_bundle_max_allocated_bytes": 10_000_000,
            },
        )
        self.assertEqual(
            payload["evidence_lifecycle"],
            {
                "registered_retry_allowed": False,
                "exact_output_root_required": True,
                "preexisting_target_allowed": False,
                "nested_timestamp_directory_allowed": False,
                "terminal_manifest_required_on_success_and_failure": True,
                "paper_gate": "CLOSED",
            },
        )
        self.assertTrue(markdown.read_text().startswith(
            "# CBF2026 Predictive WNLS Stage 1 Protocol\n"
        ))

    def test_emits_literal_replay_and_analyzer_commands(self):
        payload, _, _ = self.register()
        expected_prefix = [
            "conda",
            "run",
            "-n",
            "cbf_env",
            "python",
            "scripts/diagnostics/replay_predictive_wnls_recovery.py",
            "--data-path",
            str(self.truth),
            "--input-manifest-path",
            str(self.input_manifest),
            "--protocol-json",
            (
                "docs/diagnostics/"
                "2026-07-30-predictive-wnls-stage1-protocol.json"
            ),
        ]
        self.assertEqual(
            payload["commands"]["smoke_a"],
            expected_prefix
            + [
                "--output-root",
                "/private/tmp/cbf2026-predictive-wnls-smoke-a",
                "--run-seeds",
                "20260727",
                "--max-frames",
                "2",
            ],
        )
        self.assertEqual(
            payload["commands"]["registered_replay"],
            expected_prefix
            + [
                "--output-root",
                (
                    "/private/tmp/cbf2026-predictive-wnls-development/"
                    "stage1-v2"
                ),
                "--run-seeds",
                ",".join(str(seed) for seed in range(20260727, 20260747)),
            ],
        )
        self.assertEqual(
            payload["commands"]["registered_analyzer"],
            [
                "conda",
                "run",
                "-n",
                "cbf_env",
                "python",
                "scripts/diagnostics/analyze_predictive_wnls_recovery.py",
                "--baseline-process-path",
                str(self.baseline),
                "--development-manifest-path",
                (
                    "/private/tmp/cbf2026-predictive-wnls-development/"
                    "stage1-v2/manifest.json"
                ),
                "--protocol-json",
                (
                    "docs/diagnostics/"
                    "2026-07-30-predictive-wnls-stage1-protocol.json"
                ),
                "--expected-baseline-sha256",
                sha256(self.baseline),
                "--output-root",
                (
                    "/private/tmp/"
                    "cbf2026-predictive-wnls-development-analysis/stage1-v2"
                ),
            ],
        )

    def test_output_is_deterministic_for_same_bound_repository(self):
        _, markdown_a, json_a = self.register("a")
        _, markdown_b, json_b = self.register("b")
        self.assertEqual(json_a.read_bytes(), json_b.read_bytes())
        self.assertEqual(markdown_a.read_bytes(), markdown_b.read_bytes())

    def test_dirty_tracked_implementation_file_fails_before_outputs(self):
        estimator = self.repository / REPOSITORY_SOURCES["estimator_source"]
        estimator.write_text("dirty tracked implementation\n")
        markdown = self.docs / "dirty.md"
        output_json = self.docs / "dirty.json"
        with self.production_inputs():
            with self.assertRaisesRegex(ValueError, "dirty"):
                registrar.register_stage1_protocol(
                    repository_root=self.repository,
                    output_markdown=markdown,
                    output_json=output_json,
                )
        self.assertFalse(markdown.exists())
        self.assertFalse(output_json.exists())

    def test_missing_external_input_fails_before_outputs(self):
        self.baseline.unlink()
        markdown = self.docs / "missing.md"
        output_json = self.docs / "missing.json"
        with self.production_inputs():
            with self.assertRaisesRegex(ValueError, "missing"):
                registrar.register_stage1_protocol(
                    repository_root=self.repository,
                    output_markdown=markdown,
                    output_json=output_json,
                )
        self.assertFalse(markdown.exists())
        self.assertFalse(output_json.exists())

    def test_preexisting_either_target_fails_without_creating_the_other(self):
        for preexisting_kind in ("markdown", "json"):
            with self.subTest(preexisting_kind=preexisting_kind):
                markdown = self.docs / f"{preexisting_kind}.md"
                output_json = self.docs / f"{preexisting_kind}.json"
                preexisting = (
                    markdown
                    if preexisting_kind == "markdown"
                    else output_json
                )
                other = (
                    output_json
                    if preexisting_kind == "markdown"
                    else markdown
                )
                preexisting.write_text("preserve me")
                with self.production_inputs():
                    with self.assertRaises(FileExistsError):
                        registrar.register_stage1_protocol(
                            repository_root=self.repository,
                            output_markdown=markdown,
                            output_json=output_json,
                        )
                self.assertEqual(preexisting.read_text(), "preserve me")
                self.assertFalse(other.exists())

    def test_symlink_source_or_output_parent_is_rejected(self):
        real_truth = self.truth
        alias = self.inputs / "truth-alias.json"
        alias.symlink_to(real_truth)
        old_truth = self.truth
        self.truth = alias
        try:
            with self.assertRaisesRegex(ValueError, "symbolic-link"):
                self.register("source-symlink")
        finally:
            self.truth = old_truth

        real_output_parent = self.repository / "real-output"
        real_output_parent.mkdir()
        alias_parent = self.repository / "alias-output"
        alias_parent.symlink_to(real_output_parent, target_is_directory=True)
        with self.production_inputs():
            with self.assertRaisesRegex(ValueError, "symbolic-link"):
                registrar.register_stage1_protocol(
                    repository_root=self.repository,
                    output_markdown=alias_parent / "protocol.md",
                    output_json=alias_parent / "protocol.json",
                )

    def test_source_path_drift_during_registration_removes_outputs(self):
        original = registrar._read_bound_source
        reads = 0

        def mutate_after_initial_reads(path, *arguments, **keywords):
            nonlocal reads
            result = original(path, *arguments, **keywords)
            reads += 1
            if reads == len(REPOSITORY_SOURCES) + 3:
                replacement = self.inputs / "truth-replacement.json"
                replacement.write_bytes(real_truth.read_bytes())
                os.replace(replacement, real_truth)
            return result

        real_truth = self.truth
        markdown = self.docs / "drift.md"
        output_json = self.docs / "drift.json"
        with self.production_inputs():
            with mock.patch.object(
                registrar,
                "_read_bound_source",
                side_effect=mutate_after_initial_reads,
            ):
                with self.assertRaisesRegex(ValueError, "changed"):
                    registrar.register_stage1_protocol(
                        repository_root=self.repository,
                        output_markdown=markdown,
                        output_json=output_json,
                    )
        self.assertFalse(markdown.exists())
        self.assertFalse(output_json.exists())

    def test_cli_writes_requested_relative_targets(self):
        markdown = Path("docs/diagnostics/cli.md")
        output_json = Path("docs/diagnostics/cli.json")
        with self.production_inputs():
            exit_code = registrar.main(
                [
                    "--repository-root",
                    str(self.repository),
                    "--output-markdown",
                    str(markdown),
                    "--output-json",
                    str(output_json),
                ]
            )
        self.assertEqual(exit_code, 0)
        self.assertTrue((self.repository / markdown).is_file())
        self.assertTrue((self.repository / output_json).is_file())

    def test_probe_error_survives_transient_cleanup_faults_for_both_layouts(self):
        for same_parent in (True, False):
            with self.subTest(same_parent=same_parent):
                first_parent = self.repository / f"cleanup-{same_parent}-a"
                second_parent = (
                    first_parent
                    if same_parent
                    else self.repository / f"cleanup-{same_parent}-b"
                )
                first_parent.mkdir()
                if second_parent != first_parent:
                    second_parent.mkdir()
                markdown = first_parent / "protocol.md"
                output_json = second_parent / "protocol.json"
                real_unlink = os.unlink
                real_fsync = os.fsync
                unlink_faulted = False
                fsync_faulted = False
                unlink_attempts = []

                def transient_unlink(path, *, dir_fd=None):
                    nonlocal unlink_faulted
                    unlink_attempts.append(path)
                    if not unlink_faulted:
                        unlink_faulted = True
                        raise OSError("transient cleanup unlink")
                    return real_unlink(path, dir_fd=dir_fd)

                def transient_cleanup_fsync(descriptor):
                    nonlocal fsync_faulted
                    try:
                        target_is_open = markdown.exists() or output_json.exists()
                    except OSError:
                        target_is_open = True
                    if unlink_faulted and target_is_open and not fsync_faulted:
                        fsync_faulted = True
                        raise OSError("transient cleanup fsync")
                    return real_fsync(descriptor)

                baseline_fds = open_fd_count()
                with (
                    mock.patch("os.unlink", side_effect=transient_unlink),
                    mock.patch("os.fsync", side_effect=transient_cleanup_fsync),
                ):
                    with self.assertRaisesRegex(
                        RuntimeError, "final probe drift"
                    ) as raised:
                        registrar._write_protocol_outputs(
                            output_markdown=markdown,
                            output_json=output_json,
                            markdown_payload=b"markdown",
                            json_payload=b"json",
                            final_probe=lambda: (_ for _ in ()).throw(
                                RuntimeError("final probe drift")
                            ),
                        )
                self.assertTrue(unlink_faulted)
                self.assertTrue(fsync_faulted)
                self.assertGreaterEqual(len(unlink_attempts), 3)
                self.assertFalse(markdown.exists())
                self.assertFalse(output_json.exists())
                self.assertEqual(open_fd_count(), baseline_fds)
                notes = getattr(raised.exception, "__notes__", ())
                self.assertTrue(
                    any("transient cleanup unlink" in note for note in notes)
                )
                self.assertTrue(
                    any("transient cleanup fsync" in note for note in notes)
                )

    def test_persistent_cleanup_fault_does_not_mask_or_skip_other_target(self):
        first_parent = self.repository / "persistent-a"
        second_parent = self.repository / "persistent-b"
        first_parent.mkdir()
        second_parent.mkdir()
        markdown = first_parent / "protocol.md"
        output_json = second_parent / "protocol.json"
        real_unlink = os.unlink
        attempted = []

        def persistent_markdown_unlink(path, *, dir_fd=None):
            attempted.append(path)
            if path == markdown.name:
                raise OSError("persistent markdown cleanup")
            return real_unlink(path, dir_fd=dir_fd)

        baseline_fds = open_fd_count()
        with mock.patch("os.unlink", side_effect=persistent_markdown_unlink):
            with self.assertRaisesRegex(
                RuntimeError, "persistent probe drift"
            ) as raised:
                registrar._write_protocol_outputs(
                    output_markdown=markdown,
                    output_json=output_json,
                    markdown_payload=b"markdown",
                    json_payload=b"json",
                    final_probe=lambda: (_ for _ in ()).throw(
                        RuntimeError("persistent probe drift")
                    ),
                )
        self.assertTrue(markdown.exists())
        self.assertFalse(output_json.exists())
        self.assertIn(output_json.name, attempted)
        self.assertEqual(open_fd_count(), baseline_fds)
        notes = getattr(raised.exception, "__notes__", ())
        self.assertTrue(
            any("persistent markdown cleanup" in note for note in notes)
        )

    def test_cleanup_preserves_foreign_replacement_and_cleans_other_target(self):
        parent = self.repository / "foreign-cleanup"
        parent.mkdir()
        markdown = parent / "protocol.md"
        output_json = parent / "protocol.json"

        def replace_with_foreign_then_fail():
            markdown.unlink()
            markdown.write_bytes(b"foreign replacement")
            raise RuntimeError("foreign replacement probe drift")

        baseline_fds = open_fd_count()
        with self.assertRaisesRegex(
            RuntimeError, "foreign replacement probe drift"
        ):
            registrar._write_protocol_outputs(
                output_markdown=markdown,
                output_json=output_json,
                markdown_payload=b"owned markdown",
                json_payload=b"owned json",
                final_probe=replace_with_foreign_then_fail,
            )
        self.assertEqual(markdown.read_bytes(), b"foreign replacement")
        self.assertFalse(output_json.exists())
        self.assertEqual(open_fd_count(), baseline_fds)

    def test_output_parent_fstat_failure_closes_new_descriptor(self):
        parent = self.repository / "parent-fstat"
        parent.mkdir()
        target = parent / "protocol.json"
        for close_fault in (False, True):
            with self.subTest(close_fault=close_fault):
                baseline_fds = open_fd_count()
                real_fstat = os.fstat
                real_close = os.close
                fstat_faulted = False
                close_faulted = False

                def fail_first_fstat(descriptor):
                    nonlocal fstat_faulted
                    if not fstat_faulted:
                        fstat_faulted = True
                        raise OSError("parent fstat fault")
                    return real_fstat(descriptor)

                def close_then_optionally_fault(descriptor):
                    nonlocal close_faulted
                    real_close(descriptor)
                    if close_fault and not close_faulted:
                        close_faulted = True
                        raise OSError("parent close fault")

                with (
                    mock.patch("os.fstat", side_effect=fail_first_fstat),
                    mock.patch(
                        "os.close",
                        side_effect=close_then_optionally_fault,
                    ),
                ):
                    with self.assertRaisesRegex(
                        OSError, "parent fstat fault"
                    ) as raised:
                        registrar._open_output_parent(target)
                self.assertTrue(fstat_faulted)
                self.assertEqual(close_faulted, close_fault)
                self.assertEqual(open_fd_count(), baseline_fds)
                notes = getattr(raised.exception, "__notes__", ())
                if close_fault:
                    self.assertTrue(
                        any("parent close fault" in note for note in notes)
                    )
                else:
                    self.assertEqual(notes, ())


if __name__ == "__main__":
    unittest.main()
