import copy
import json
import tempfile
import unittest
import subprocess
import sys
import argparse
import os
import shutil
from contextlib import contextmanager
from pathlib import Path
from unittest import mock

import scripts.diagnostics.register_qualified_closure_campaign as registrar

from scripts.diagnostics.register_qualified_closure_campaign import (
    FROZEN_THRESHOLDS,
    build_qualified_closure_protocol,
    publish_protocol,
    validate_authorization_binding,
    verify_registered_protocol,
    register_from_arguments,
)


class QualifiedClosureRegistrarTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-registrar-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.project = self.root / "repo"
        self.project.mkdir()
        self.frozen_roots = {
            "development": {
                "raw": str(self.root / "raw" / "v1"),
                "analysis": str(self.root / "analysis" / "v1"),
            },
            "confirmatory": {
                "smoke_a_raw": str(self.root / "outputs" / "smoke-a-raw"),
                "smoke_a_analysis": str(self.root / "outputs" / "smoke-a-analysis"),
                "smoke_b_raw": str(self.root / "outputs" / "smoke-b-raw"),
                "smoke_b_analysis": str(self.root / "outputs" / "smoke-b-analysis"),
                "raw": str(self.root / "outputs" / "confirmatory" / "v1"),
                "analysis": str(self.root / "outputs" / "confirmatory-analysis" / "v1"),
            },
        }
        roots_patch = mock.patch.object(
            registrar, "FROZEN_EXECUTION_ROOTS", self.frozen_roots
        )
        roots_patch.start()
        self.addCleanup(roots_patch.stop)
        self.files = {}
        for name in (
            "source.py", "Swarm", "config.json", "primary.json", "ablation.json",
            "dependencies.txt", "schema.json",
        ):
            path = self.project / name
            path.write_text(name)
            self.files[name] = path
        self.primary = {
            "position_covariance": {"reference-selection": "dynamic-lower-index"}
        }
        self.ablation = {
            "position_covariance": {"reference-selection": "fixed-cbf-only"}
        }
        self.files["primary.json"].write_text(json.dumps(self.primary))
        self.files["ablation.json"].write_text(json.dumps(self.ablation))
        self.files["dependencies.txt"].write_text("ENABLE_GUROBI:BOOL=ON\n")

    def roots(self):
        output = self.root / "outputs"
        return {
            "smoke_a_raw": output / "smoke-a-raw",
            "smoke_a_analysis": output / "smoke-a-analysis",
            "smoke_b_raw": output / "smoke-b-raw",
            "smoke_b_analysis": output / "smoke-b-analysis",
            "raw": output / "confirmatory" / "v1",
            "analysis": output / "confirmatory-analysis" / "v1",
        }

    def build(self, *, count=60, **overrides):
        arguments = {
            "kind": "confirmatory",
            "version": "v1",
            "project_root": self.project,
            "trajectory_seeds": list(range(2026082001, 2026082001 + count)),
            "range_noise_seeds": list(range(2026083001, 2026083001 + count)),
            "frames": 1000,
            "smoke_trajectory_seed": 2026089001,
            "smoke_range_noise_seed": 2026089101,
            "smoke_frames": 20,
            "roots": self.roots(),
            "bindings": {
                "source": self.files["source.py"],
                "binary": self.files["Swarm"],
                "base_config": self.files["config.json"],
                "primary_config": self.files["primary.json"],
                "ablation_config": self.files["ablation.json"],
                "dependencies": self.files["dependencies.txt"],
                "schema": self.files["schema.json"],
            },
            "thresholds": copy.deepcopy(FROZEN_THRESHOLDS),
            "dirty_relevant_paths": [],
        }
        arguments.update(overrides)
        return build_qualified_closure_protocol(**arguments)

    def authoritative(self, *, json_path=None, markdown_path=None):
        protocol = self.build()
        protocol_json = Path(json_path or self.root / "confirmatory-protocol.json")
        protocol_markdown = Path(
            markdown_path or self.root / "confirmatory-protocol.md"
        )
        arguments = argparse.Namespace(
            kind="confirmatory", version="v1", protocol_json=protocol_json,
            binary=self.files["Swarm"], base_config=self.files["config.json"],
            primary_config=self.files["primary.json"],
            ablation_config=self.files["ablation.json"],
            trajectory_seeds="2026082001:2026082060",
            range_noise_seeds="2026083001:2026083060", frames=1000,
            smoke_trajectory_seed=2026089001,
            smoke_range_noise_seed=2026089101, smoke_frames=20,
            raw_root=Path(protocol["roots"]["raw"]),
            analysis_root=Path(protocol["roots"]["analysis"]),
            smoke_a_raw_root=Path(protocol["roots"]["smoke_a_raw"]),
            smoke_a_analysis_root=Path(protocol["roots"]["smoke_a_analysis"]),
            smoke_b_raw_root=Path(protocol["roots"]["smoke_b_raw"]),
            smoke_b_analysis_root=Path(protocol["roots"]["smoke_b_analysis"]),
        )
        repository = {
            "root": str(self.project),
            "head": "a" * 40,
            "tree": "b" * 40,
            "dirty_tracked_paths": [],
            "dirty_relevant_paths": [],
            "allowed_untracked_paths": ["build-diagnostic/"],
        }
        command_identity = {
            "argv": ["conda", "list", "--explicit"],
            "sha256": "c" * 64,
            "bytes": 1,
        }
        dependency_identity = {
            "argv": ["ldd", str(self.files["Swarm"])],
            "returncode": 0,
            "sha256": "d" * 64,
            "bytes": 1,
        }
        protocol.update({
            "repository": repository,
            "review_artifacts": {
                "development_protocol": registrar._file_identity(self.files["source.py"]),
                "development_report": registrar._file_identity(self.files["config.json"]),
                "development_review": registrar._file_identity(self.files["schema.json"]),
            },
            "build": {
                "cmake_cache": registrar._file_identity(self.files["dependencies.txt"]),
                "gurobi_enabled": True,
                "binary_dependencies": dependency_identity,
                "conda_explicit": command_identity,
            },
            "tooling": {
                name: registrar._file_identity(self.files["source.py"])
                for name in (
                    "run_qualified_closure_campaign.py",
                    "generate_qualified_measurements.py",
                    "analyze_qualified_closure_campaign.py",
                    "register_qualified_closure_campaign.py",
                    "replay_qualified_estimator.py",
                    "analyze_qualified_estimator.py",
                )
            },
            "publication": {
                "json_path": str(protocol_json),
                "markdown_path": str(protocol_markdown),
            },
            "supervision": {
                "wallclock_timeout_s": 3600.0,
                "line_stall_timeout_s": 300.0,
                "termination_grace_s": 5.0,
            },
        })
        commands = registrar._registered_argv(protocol)
        protocol["runner_argv"] = commands["runner"]
        protocol["analyzer_argv"] = commands["analyzer"]
        protocol["smoke_argv"] = commands["smoke"]
        protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
        return protocol

    @contextmanager
    def authoritative_probes(self, protocol):
        with (
            mock.patch.object(
                registrar, "_repository_identity",
                return_value=copy.deepcopy(protocol["repository"]),
            ),
            mock.patch.object(
                registrar, "_dependency_identity",
                return_value=copy.deepcopy(protocol["build"]["binary_dependencies"]),
            ),
            mock.patch.object(
                registrar, "_command_identity",
                return_value=copy.deepcopy(protocol["build"]["conda_explicit"]),
            ),
            mock.patch.object(
                registrar.subprocess, "run",
                return_value=subprocess.CompletedProcess([], 0, b"", b""),
            ),
        ):
            yield

    def test_exact_protocol_schema_rejects_deleted_and_extra_fields_after_rehash(self):
        protocol = self.authoritative()
        with self.authoritative_probes(protocol):
            verify_registered_protocol(protocol)

        mutations = []
        missing = copy.deepcopy(protocol)
        del missing["repository"]
        mutations.append(missing)
        extra = copy.deepcopy(protocol)
        extra["self_consistent_but_unregistered"] = True
        mutations.append(extra)
        nested_extra = copy.deepcopy(protocol)
        nested_extra["schedule"]["unregistered"] = 1
        mutations.append(nested_extra)
        nested_missing = copy.deepcopy(protocol)
        del nested_missing["authorization"]["must_bind_implementation_identity"]
        mutations.append(nested_missing)

        for mutated in mutations:
            with self.subTest(keys=set(mutated)):
                mutated["semantic_sha256"] = registrar._semantic_sha256(mutated)
                with self.authoritative_probes(protocol):
                    with self.assertRaisesRegex(ValueError, "schema is not exact"):
                        verify_registered_protocol(mutated)

    def test_self_rehashed_nonliteral_execution_root_is_rejected(self):
        protocol = self.authoritative()
        protocol["roots"]["analysis"] = str(
            self.root / "self-consistent-but-unregistered-analysis"
        )
        protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)

        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(ValueError, "literal execution roots"):
                verify_registered_protocol(protocol)

    def test_self_rehashed_runner_argv_substitution_is_rejected(self):
        protocol = self.authoritative()
        protocol["runner_argv"][-1] = str(self.root / "other-raw-root")
        protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)

        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(ValueError, "runner/analyzer argv"):
                verify_registered_protocol(protocol)

    def test_self_rehashed_supervision_or_authorization_semantics_are_rejected(self):
        for section, field, value in (
            ("supervision", "line_stall_timeout_s", 301.0),
            ("authorization", "must_bind_implementation_identity", False),
        ):
            with self.subTest(section=section, field=field):
                protocol = self.authoritative()
                protocol[section][field] = value
                protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
                with self.authoritative_probes(protocol):
                    with self.assertRaisesRegex(ValueError, "frozen contract"):
                        verify_registered_protocol(protocol)

    def test_repository_identity_reports_untracked_relevant_files_but_allows_build_cache(self):
        repository = self.root / "status-repo"
        repository.mkdir()
        (repository / "tracked.txt").write_text("tracked\n")
        subprocess.run(["git", "init", "-q"], cwd=repository, check=True)
        subprocess.run(["git", "add", "tracked.txt"], cwd=repository, check=True)
        environment = {
            **os.environ,
            "GIT_AUTHOR_NAME": "Test",
            "GIT_AUTHOR_EMAIL": "test@example.invalid",
            "GIT_COMMITTER_NAME": "Test",
            "GIT_COMMITTER_EMAIL": "test@example.invalid",
        }
        subprocess.run(
            ["git", "commit", "-qm", "fixture"], cwd=repository,
            check=True, env=environment,
        )
        (repository / "build-diagnostic").mkdir()
        (repository / "build-diagnostic" / "cache.bin").write_bytes(b"cache")
        (repository / "scripts").mkdir()
        (repository / "scripts" / "new.py").write_text("pass\n")

        identity = registrar._repository_identity(repository)

        self.assertEqual(identity["dirty_tracked_paths"], [])
        self.assertEqual(identity.get("dirty_relevant_paths"), ["scripts/new.py"])
        self.assertEqual(identity.get("allowed_untracked_paths"), ["build-diagnostic/cache.bin"])

    def test_verifier_rejects_post_registration_head_even_when_original_is_ancestor(self):
        protocol = self.authoritative()
        observed = copy.deepcopy(protocol["repository"])
        observed["head"] = "e" * 40
        observed["tree"] = "f" * 40

        with self.authoritative_probes(protocol):
            with mock.patch.object(registrar, "_repository_identity", return_value=observed):
                with self.assertRaisesRegex(ValueError, "exact current HEAD"):
                    verify_registered_protocol(protocol)

    def test_exact_sixty_pair_schedule_and_universes_are_bound(self):
        protocol = self.build()

        self.assertEqual(len(protocol["schedule"]["missions"]), 60)
        self.assertEqual(
            protocol["schedule"]["trajectory_seeds"],
            list(range(2026082001, 2026082061)),
        )
        self.assertEqual(
            protocol["schedule"]["range_noise_seeds"],
            list(range(2026083001, 2026083061)),
        )
        self.assertEqual(protocol["universes"], {
            "initialization": 840,
            "estimator_per_condition": 839160,
            "estimator_total": 1678320,
            "controller": 60000,
            "endpoint": 13920000,
            "reconstructed": 7140000,
            "mission": 60,
        })
        self.assertEqual(set(protocol["bindings"]), {
            "source", "binary", "base_config", "primary_config",
            "ablation_config", "dependencies", "schema",
        })
        self.assertEqual(protocol["thresholds"], FROZEN_THRESHOLDS)
        self.assertEqual(protocol["no_retry"], True)

    def test_fifty_nine_or_sixty_one_missions_are_rejected(self):
        for count in (59, 61):
            with self.subTest(count=count):
                with self.assertRaisesRegex(ValueError, "exactly 60"):
                    self.build(count=count)

    def test_duplicate_colliding_and_development_seeds_are_rejected(self):
        trajectory = list(range(2026082001, 2026082061))
        trajectory[-1] = trajectory[0]
        with self.assertRaisesRegex(ValueError, "distinct"):
            self.build(trajectory_seeds=trajectory)
        with self.assertRaisesRegex(ValueError, "disjoint"):
            self.build(
                range_noise_seeds=list(range(2026082001, 2026082061))
            )
        with self.assertRaisesRegex(ValueError, "development"):
            self.build(
                trajectory_seeds=[2026080101] + list(range(2026082002, 2026082061))
            )

    def test_smoke_schedule_is_exact_reused_and_excluded_from_denominator(self):
        protocol = self.build()

        smoke = protocol["smoke_schedule"]
        self.assertEqual(smoke["trajectory_seed"], 2026089001)
        self.assertEqual(smoke["range_noise_seed"], 2026089101)
        self.assertEqual(smoke["frames"], 20)
        self.assertEqual(smoke["semantic_schedule_a"], smoke["semantic_schedule_b"])
        self.assertEqual(smoke["universes"], {
            "initialization": 14,
            "estimator_per_condition": 266,
            "estimator_total": 532,
            "controller": 20,
            "endpoint": 4640,
            "reconstructed": 2380,
            "mission": 1,
        })
        self.assertFalse(smoke["included_in_scientific_denominator"])

    def test_dirty_source_existing_or_symlinked_root_fails_before_protocol(self):
        with self.assertRaisesRegex(ValueError, "dirty relevant"):
            self.build(dirty_relevant_paths=["source.py"])
        roots = self.roots()
        roots["raw"].mkdir(parents=True)
        with self.assertRaisesRegex(FileExistsError, "absent"):
            self.build(roots=roots)
        roots["raw"].rmdir()
        roots = self.roots()
        target = self.root / "target"
        target.mkdir()
        roots["analysis"].parent.mkdir(parents=True, exist_ok=True)
        roots["analysis"].symlink_to(target, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "symbolic"):
            self.build(roots=roots)

    def test_root_with_symbolic_ancestor_is_rejected(self):
        actual = self.root / "actual-output"
        actual.mkdir()
        alias = self.root / "output-alias"
        alias.symlink_to(actual, target_is_directory=True)
        roots = self.roots()
        roots["raw"] = alias / "confirmatory" / "v1"

        with self.assertRaisesRegex(ValueError, "symbolic ancestor"):
            self.build(roots=roots)

    def test_post_registration_symbolic_root_ancestor_is_rejected(self):
        protocol = self.authoritative()
        target = self.root / "redirected-output"
        target.mkdir()
        (self.root / "outputs").symlink_to(target, target_is_directory=True)

        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(ValueError, "symbolic ancestor"):
                verify_registered_protocol(protocol)

    def test_threshold_and_primary_ablation_mode_mismatch_are_rejected(self):
        thresholds = copy.deepcopy(FROZEN_THRESHOLDS)
        thresholds["aggregate_containment"] = 0.5
        with self.assertRaisesRegex(ValueError, "threshold"):
            self.build(thresholds=thresholds)
        self.files["primary.json"].write_text(json.dumps(self.ablation))
        with self.assertRaisesRegex(ValueError, "primary config"):
            self.build()

    def test_post_registration_mutation_is_detected(self):
        protocol = self.authoritative()
        with self.authoritative_probes(protocol):
            verify_registered_protocol(protocol)
        self.files["Swarm"].write_text("mutated")
        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(ValueError, "binding mutated"):
                verify_registered_protocol(protocol)

    def test_self_rehashed_derived_schedule_or_universe_mutation_is_detected(self):
        for field in ("schedule", "universes"):
            with self.subTest(field=field):
                protocol = self.authoritative()
                if field == "schedule":
                    protocol[field]["missions"][0]["frames"] = 999
                else:
                    protocol[field]["controller"] -= 1
                protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
                with self.authoritative_probes(protocol):
                    with self.assertRaisesRegex(ValueError, "derived protocol contract"):
                        verify_registered_protocol(protocol)

    def test_analyzer_may_reverify_only_the_registered_claimed_raw_root(self):
        protocol = self.authoritative()
        raw = Path(protocol["roots"]["raw"])
        raw.mkdir(parents=True)
        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(FileExistsError, "remain absent"):
                verify_registered_protocol(protocol)
            verify_registered_protocol(protocol, allowed_claimed_roots={"raw"})

    def test_protocol_json_and_markdown_publish_atomically_without_replace(self):
        output = self.root / "protocol"
        output.mkdir()
        json_path = output / "fixture-protocol.json"
        markdown_path = output / "fixture-protocol.md"
        protocol = self.authoritative(json_path=json_path, markdown_path=markdown_path)

        with self.authoritative_probes(protocol):
            publish_protocol(protocol, json_path, markdown_path)

        self.assertEqual(json.loads(json_path.read_text()), protocol)
        self.assertIn("Qualified Closure", markdown_path.read_text())
        before = (json_path.read_bytes(), markdown_path.read_bytes())
        with self.authoritative_probes(protocol):
            with self.assertRaises(FileExistsError):
                publish_protocol(protocol, json_path, markdown_path)
        self.assertEqual((json_path.read_bytes(), markdown_path.read_bytes()), before)

    def test_publication_rejects_paths_other_than_registered_pair(self):
        declared = self.root / "declared"
        declared.mkdir()
        protocol = self.authoritative(
            json_path=declared / "fixture-protocol.json",
            markdown_path=declared / "fixture-protocol.md",
        )
        substituted = self.root / "substituted"
        substituted.mkdir()

        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(ValueError, "registered publication paths"):
                publish_protocol(
                    protocol,
                    substituted / "fixture-protocol.json",
                    substituted / "fixture-protocol.md",
                )
        self.assertEqual(list(substituted.iterdir()), [])

    def test_protocol_publication_failure_leaves_no_completed_pair(self):
        output = self.root / "protocol-transaction"
        output.mkdir()
        json_path = output / "fixture-protocol.json"
        markdown_path = output / "fixture-protocol.md"
        protocol = self.authoritative(json_path=json_path, markdown_path=markdown_path)
        observed = []
        def fail_after_markdown(_stage):
            observed.append((json_path.exists(), markdown_path.exists()))
            raise RuntimeError("injected")

        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(RuntimeError, "injected"):
                publish_protocol(
                    protocol, json_path, markdown_path,
                    publish_hook=fail_after_markdown,
                )
        self.assertEqual(observed, [(False, True)])
        self.assertFalse(json_path.exists())
        self.assertFalse(markdown_path.exists())

    def test_confirmatory_scientific_and_smoke_argv_are_exact(self):
        arguments = argparse.Namespace(
            kind="confirmatory", version="v1",
            protocol_json=Path("docs/diagnostics/frozen-protocol.json"),
            binary=Path("build-diagnostic/Swarm"),
            base_config=Path("config/config.json"),
            primary_config=Path("config/diagnostics/primary.json"),
            ablation_config=Path("config/diagnostics/ablation.json"),
            trajectory_seeds="2026082001:2026082060",
            range_noise_seeds="2026083001:2026083060", frames=1000,
            smoke_trajectory_seed=2026089001,
            smoke_range_noise_seed=2026089101, smoke_frames=20,
            raw_root=Path("/private/tmp/scientific-raw"),
            analysis_root=Path("/private/tmp/scientific-analysis"),
            smoke_a_raw_root=Path("/private/tmp/smoke-a-raw"),
            smoke_a_analysis_root=Path("/private/tmp/smoke-a-analysis"),
            smoke_b_raw_root=Path("/private/tmp/smoke-b-raw"),
            smoke_b_analysis_root=Path("/private/tmp/smoke-b-analysis"),
        )
        authorization = "docs/diagnostics/reviews/frozen-authorization.json"

        self.assertEqual(registrar._analyzer_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python",
            "scripts/diagnostics/analyze_qualified_closure_campaign.py",
            "--kind", "confirmatory", "--version", "v1",
            "--protocol", "docs/diagnostics/frozen-protocol.json",
            "--authorization", authorization,
            "--input-root", "/private/tmp/scientific-raw",
            "--output-root", "/private/tmp/scientific-analysis",
        ])
        expected_smoke = {
            "a": ("/private/tmp/smoke-a-raw", "/private/tmp/smoke-a-analysis"),
            "b": ("/private/tmp/smoke-b-raw", "/private/tmp/smoke-b-analysis"),
        }
        smoke_argv = registrar._smoke_argv(arguments)
        for smoke_id, (raw, analysis) in expected_smoke.items():
            self.assertEqual(smoke_argv[smoke_id]["runner"], [
                "conda", "run", "-n", "cbf_env", "python",
                "scripts/diagnostics/run_qualified_closure_campaign.py",
                "--kind", "confirmatory-smoke", "--smoke-id", smoke_id,
                "--protocol", "docs/diagnostics/frozen-protocol.json",
                "--authorization", authorization,
                "--binary", "build-diagnostic/Swarm",
                "--base-config", "config/config.json",
                "--primary-config", "config/diagnostics/primary.json",
                "--ablation-config", "config/diagnostics/ablation.json",
                "--trajectory-seeds", "2026089001:2026089001",
                "--range-noise-seeds", "2026089101:2026089101",
                "--frames", "20", "--output-root", raw,
            ])
            self.assertEqual(smoke_argv[smoke_id]["analyzer"], [
                "conda", "run", "-n", "cbf_env", "python",
                "scripts/diagnostics/analyze_qualified_closure_campaign.py",
                "--kind", "confirmatory-smoke", "--version", "v1",
                "--smoke-id", smoke_id,
                "--protocol", "docs/diagnostics/frozen-protocol.json",
                "--authorization", authorization,
                "--input-root", raw, "--output-root", analysis,
            ])

    def test_development_analyzer_argv_binds_version_and_ablation(self):
        arguments = argparse.Namespace(
            kind="development", version="v1",
            protocol_json=Path("docs/diagnostics/development-protocol.json"),
            ablation_config=Path("config/diagnostics/ablation.json"),
            raw_root=Path("/private/tmp/development-raw"),
            analysis_root=Path("/private/tmp/development-analysis"),
        )

        self.assertEqual(registrar._analyzer_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python",
            "scripts/diagnostics/analyze_qualified_closure_campaign.py",
            "--kind", "development", "--version", "v1",
            "--protocol", "docs/diagnostics/development-protocol.json",
            "--authorization",
            "docs/diagnostics/reviews/development-authorization.json",
            "--input-root", "/private/tmp/development-raw",
            "--ablation-config", "config/diagnostics/ablation.json",
            "--output-root", "/private/tmp/development-analysis",
        ])

    def test_protocol_publication_recovers_only_a_proven_orphaned_markdown(self):
        output = self.root / "protocol-orphan"
        output.mkdir()
        json_path = output / "fixture-protocol.json"
        markdown_path = output / "fixture-protocol.md"
        protocol = self.authoritative(json_path=json_path, markdown_path=markdown_path)

        expected_markdown = registrar._protocol_markdown_bytes(protocol)
        markdown_path.write_bytes(expected_markdown)
        with self.authoritative_probes(protocol):
            publish_protocol(protocol, json_path, markdown_path)
        self.assertTrue(json_path.is_file())
        self.assertEqual(markdown_path.read_bytes(), expected_markdown)

        json_path.unlink()
        markdown_path.write_text("not the registered companion\n")
        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(RuntimeError, "unrecoverable orphan"):
                publish_protocol(protocol, json_path, markdown_path)
        self.assertFalse(json_path.exists())
        self.assertEqual(
            markdown_path.read_text(), "not the registered companion\n"
        )

    def test_dependency_probe_nonzero_is_fatal(self):
        completed = subprocess.CompletedProcess(
            ["ldd", "Swarm"], returncode=1, stdout=b"", stderr=b"bad binary"
        )
        with mock.patch.object(registrar.subprocess, "run", return_value=completed):
            with self.assertRaisesRegex(ValueError, "dependency identity command failed"):
                registrar._dependency_identity(self.files["Swarm"])

    def test_authorization_must_bind_exact_protocol_bytes_and_kind(self):
        protocol_path = self.root / "registered-protocol.json"
        protocol = self.authoritative(
            json_path=protocol_path, markdown_path=protocol_path.with_suffix(".md")
        )
        with self.authoritative_probes(protocol):
            publish_protocol(protocol, protocol_path, protocol_path.with_suffix(".md"))
        import hashlib
        authorization_path = self.root / "authorization.json"
        authorization = {
            "schema_version": "cbf2026-qualified-authorization-v1",
            "authorized": True,
            "kind": "confirmatory",
            "version": "v1",
            "protocol_sha256": hashlib.sha256(
                protocol_path.read_bytes()
            ).hexdigest(),
            "implementation_identity": protocol["repository"]["head"],
        }
        authorization_path.write_text(json.dumps(authorization))

        with self.authoritative_probes(protocol):
            validated = validate_authorization_binding(
                protocol_path, authorization_path
            )
        self.assertTrue(validated["authorized"])
        authorization["implementation_identity"] = "f" * 40
        authorization_path.write_text(json.dumps(authorization))
        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(ValueError, "implementation identity"):
                validate_authorization_binding(protocol_path, authorization_path)
        authorization["implementation_identity"] = protocol["repository"]["head"]
        authorization["protocol_sha256"] = "0" * 64
        authorization_path.write_text(json.dumps(authorization))
        with self.authoritative_probes(protocol):
            with self.assertRaisesRegex(ValueError, "protocol SHA-256"):
                validate_authorization_binding(protocol_path, authorization_path)

    def test_authorization_rejects_missing_or_mismatched_protocol_companion(self):
        protocol_path = self.root / "paired-protocol.json"
        markdown_path = protocol_path.with_suffix(".md")
        protocol = self.authoritative(
            json_path=protocol_path, markdown_path=markdown_path
        )
        with self.authoritative_probes(protocol):
            publish_protocol(protocol, protocol_path, markdown_path)
        import hashlib
        authorization_path = self.root / "paired-authorization.json"
        authorization_path.write_text(json.dumps({
            "schema_version": "cbf2026-qualified-authorization-v1",
            "authorized": True,
            "kind": "confirmatory",
            "version": "v1",
            "protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest(),
            "implementation_identity": protocol["repository"]["head"],
        }))

        markdown_path.unlink()
        with self.assertRaisesRegex(ValueError, "companion Markdown"):
            validate_authorization_binding(protocol_path, authorization_path)
        markdown_path.write_text("mismatched companion\n")
        with self.assertRaisesRegex(ValueError, "companion Markdown"):
            validate_authorization_binding(protocol_path, authorization_path)

    def test_cli_help_exposes_development_and_confirmatory_contract_options(self):
        script = Path(__file__).resolve().parents[1] / "scripts" / "diagnostics" / "register_qualified_closure_campaign.py"
        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        for option in (
            "--kind", "--version", "--implementation-report",
            "--implementation-review", "--development-protocol",
            "--development-report", "--development-review", "--binary",
            "--base-config", "--primary-config", "--ablation-config",
            "--trajectory-seeds", "--range-noise-seeds", "--frames",
            "--smoke-trajectory-seed", "--smoke-range-noise-seed",
            "--smoke-frames", "--raw-root", "--analysis-root",
            "--protocol-json", "--protocol-md",
        ):
            self.assertIn(option, result.stdout)

    def test_authoritative_development_registration_collects_git_and_exact_argv(self):
        repository = self.root / "authoritative-repo"
        repository.mkdir()
        base = repository / "config.json"
        primary = repository / "primary.json"
        ablation = repository / "ablation.json"
        report = repository / "implementation.md"
        review = repository / "review.md"
        binary = repository / "build" / "Swarm"
        binary.parent.mkdir()
        base.write_text("{}\n")
        primary.write_text(json.dumps(self.primary))
        ablation.write_text(json.dumps(self.ablation))
        report.write_text("reviewed implementation\n")
        review.write_text("C0 I0\n")
        shutil.copy2(sys.executable, binary)
        binary.chmod(0o755)
        (binary.parent / "CMakeCache.txt").write_text("ENABLE_GUROBI:BOOL=ON\n")
        subprocess.run(["git", "init", "-q"], cwd=repository, check=True)
        subprocess.run(["git", "add", "."], cwd=repository, check=True)
        environment = {**os.environ, "GIT_AUTHOR_NAME": "Test", "GIT_AUTHOR_EMAIL": "test@example.invalid",
                       "GIT_COMMITTER_NAME": "Test", "GIT_COMMITTER_EMAIL": "test@example.invalid"}
        subprocess.run(["git", "commit", "-qm", "fixture"], cwd=repository,
                       check=True, env=environment)
        output = self.root / "registered"
        output.mkdir()
        arguments = argparse.Namespace(
            kind="development", version="v1",
            implementation_report=report, implementation_review=review,
            development_protocol=None, development_report=None,
            development_review=None, binary=binary, base_config=base,
            primary_config=primary, ablation_config=ablation,
            trajectory_seeds="2026080101:2026080110",
            range_noise_seeds="2026081101:2026081110", frames=1000,
            smoke_trajectory_seed=None, smoke_range_noise_seed=None,
            smoke_frames=None, smoke_a_raw_root=None,
            smoke_a_analysis_root=None, smoke_b_raw_root=None,
            smoke_b_analysis_root=None,
            raw_root=self.root / "raw" / "v1",
            analysis_root=self.root / "analysis" / "v1",
            protocol_json=output / "fixture-protocol.json",
            protocol_md=output / "fixture-protocol.md",
        )

        protocol = register_from_arguments(arguments, project_root=repository)

        self.assertEqual(protocol["repository"]["head"], subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repository, text=True
        ).strip())
        self.assertEqual(protocol["repository"]["dirty_tracked_paths"], [])
        self.assertEqual(protocol["repository"]["dirty_relevant_paths"], [])
        self.assertTrue(protocol["build"]["gurobi_enabled"])
        expected_top = {
            "schema_version", "kind", "version", "conditions", "schedule",
            "smoke_schedule", "universes", "bindings", "thresholds", "roots",
            "authorization", "supervision", "no_retry", "repository",
            "review_artifacts", "build", "tooling", "publication",
            "runner_argv", "analyzer_argv", "semantic_sha256",
        }
        self.assertEqual(set(protocol), expected_top)
        self.assertEqual(set(protocol["review_artifacts"]), {
            "implementation_report", "implementation_review",
        })
        self.assertEqual(protocol["supervision"], {
            "wallclock_timeout_s": 3600.0,
            "line_stall_timeout_s": 300.0,
            "termination_grace_s": 5.0,
        })
        self.assertEqual(protocol["roots"], self.frozen_roots["development"])
        expected_commands = registrar._registered_argv(protocol)
        self.assertEqual(protocol["runner_argv"], expected_commands["runner"])
        self.assertEqual(protocol["analyzer_argv"], expected_commands["analyzer"])
        self.assertEqual(
            protocol["runner_argv"][protocol["runner_argv"].index("--binary") + 1],
            "build/Swarm",
        )
        self.assertEqual(
            protocol["runner_argv"][protocol["runner_argv"].index("--base-config") + 1],
            "config.json",
        )
        self.assertTrue(arguments.protocol_json.is_file())
        self.assertTrue(arguments.protocol_md.is_file())

        changed_dependency = dict(protocol["build"]["binary_dependencies"])
        changed_dependency["sha256"] = "0" * 64
        with mock.patch.object(
            registrar, "_dependency_identity", return_value=changed_dependency
        ):
            with self.assertRaisesRegex(ValueError, "dependency identity mutated"):
                verify_registered_protocol(protocol)
        changed_conda = dict(protocol["build"]["conda_explicit"])
        changed_conda["sha256"] = "0" * 64
        with mock.patch.object(
            registrar, "_command_identity", return_value=changed_conda
        ):
            with self.assertRaisesRegex(ValueError, "conda identity mutated"):
                verify_registered_protocol(protocol)


if __name__ == "__main__":
    unittest.main()
