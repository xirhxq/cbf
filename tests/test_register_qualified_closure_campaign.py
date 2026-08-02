import copy
import hashlib
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
                "raw": str(self.root / "raw" / "v5"),
                "analysis": str(self.root / "analysis" / "v5"),
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
        self.initial_family = (
            Path(__file__).resolve().parents[1]
            / "config" / "diagnostics" / "qualified_initial_family_v1.json"
        )

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
        if arguments["kind"] == "development":
            arguments["bindings"] = dict(arguments["bindings"])
            arguments["bindings"].setdefault("initial_family", self.initial_family)
        return build_qualified_closure_protocol(**arguments)

    def authoritative(
        self, *, json_path=None, markdown_path=None,
        kind="confirmatory", version="v1",
    ):
        development = kind == "development"
        count = 10 if development else 60
        trajectory_start = 2026080201 if development else 2026082001
        range_start = 2026081201 if development else 2026083001
        roots = {
            label: Path(path)
            for label, path in self.frozen_roots[kind].items()
        }
        protocol = self.build(
            count=count, kind=kind, version=version, roots=roots,
            trajectory_seeds=list(range(trajectory_start, trajectory_start + count)),
            range_noise_seeds=list(range(range_start, range_start + count)),
        )
        protocol_json = Path(json_path or self.root / "confirmatory-protocol.json")
        protocol_markdown = Path(
            markdown_path or self.root / "confirmatory-protocol.md"
        )
        arguments = argparse.Namespace(
            kind=kind, version=version, protocol_json=protocol_json,
            binary=self.files["Swarm"], base_config=self.files["config.json"],
            primary_config=self.files["primary.json"],
            ablation_config=self.files["ablation.json"],
            initial_family=(self.initial_family if development else None),
            trajectory_seeds=(
                "2026080201:2026080210" if development
                else "2026082001:2026082060"
            ),
            range_noise_seeds=(
                "2026081201:2026081210" if development
                else "2026083001:2026083060"
            ), frames=1000,
            smoke_trajectory_seed=2026089001,
            smoke_range_noise_seed=2026089101, smoke_frames=20,
            raw_root=Path(protocol["roots"]["raw"]),
            analysis_root=Path(protocol["roots"]["analysis"]),
            smoke_a_raw_root=(
                None if development else Path(protocol["roots"]["smoke_a_raw"])
            ),
            smoke_a_analysis_root=(
                None if development else Path(protocol["roots"]["smoke_a_analysis"])
            ),
            smoke_b_raw_root=(
                None if development else Path(protocol["roots"]["smoke_b_raw"])
            ),
            smoke_b_analysis_root=(
                None if development else Path(protocol["roots"]["smoke_b_analysis"])
            ),
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
            "review_artifacts": (
                {
                    "implementation_report": registrar._file_identity(
                        self.files["source.py"]
                    ),
                    "implementation_review": registrar._file_identity(
                        self.files["schema.json"]
                    ),
                }
                if development else
                {
                    "development_protocol": registrar._file_identity(
                        self.files["source.py"]
                    ),
                    "development_report": registrar._file_identity(
                        self.files["config.json"]
                    ),
                    "development_review": registrar._file_identity(
                        self.files["schema.json"]
                    ),
                }
            ),
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
                    *(
                        ("qualified_initial_state.py",)
                        if development else ()
                    ),
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
        if kind == "confirmatory":
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

    @contextmanager
    def binding_probes(self, protocol):
        """Keep expensive non-Git identities deterministic in real Git tests."""
        with (
            mock.patch.object(
                registrar, "_dependency_identity",
                return_value=copy.deepcopy(protocol["build"]["binary_dependencies"]),
            ),
            mock.patch.object(
                registrar, "_command_identity",
                return_value=copy.deepcopy(protocol["build"]["conda_explicit"]),
            ),
        ):
            yield

    def git(self, *arguments, check=True):
        environment = {
            **os.environ,
            "GIT_AUTHOR_NAME": "Test",
            "GIT_AUTHOR_EMAIL": "test@example.invalid",
            "GIT_COMMITTER_NAME": "Test",
            "GIT_COMMITTER_EMAIL": "test@example.invalid",
        }
        return subprocess.run(
            ["git", *arguments], cwd=self.project, check=check,
            text=True, capture_output=True, env=environment,
        )

    def prepare_registered_bundle(self):
        """Create the exact four uncommitted Task 10 lifecycle artifacts."""
        self.git("init", "-q")
        self.git("add", ".")
        self.git("commit", "-qm", "implementation")
        publication = self.project / "docs" / "diagnostics"
        protocol_path = publication / "fixture-protocol.json"
        markdown_path = publication / "fixture-protocol.md"
        protocol = self.authoritative(
            json_path=protocol_path, markdown_path=markdown_path,
            kind="development", version="v5",
        )
        protocol["repository"] = registrar._repository_identity(self.project)
        protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
        with self.binding_probes(protocol):
            publish_protocol(protocol, protocol_path, markdown_path)

        review_root = publication / "reviews"
        review_root.mkdir(parents=True)
        preflight_path = review_root / "fixture-preflight.md"
        authorization_path = review_root / "fixture-authorization.json"
        preflight_path.write_text(
            "# Independent preflight\n\nC0/I0/M0. Exact registered identities pass.\n",
            encoding="utf-8",
        )
        authorization_text = "Continue with the registered development-v5 execution."
        authorization = {
            "schema_version": "cbf2026-qualified-authorization-v1",
            "authorized": True,
            "kind": "development",
            "version": "v5",
            "protocol_sha256": hashlib.sha256(
                protocol_path.read_bytes()
            ).hexdigest(),
            "implementation_identity": protocol["repository"]["head"],
            "preflight_sha256": hashlib.sha256(
                preflight_path.read_bytes()
            ).hexdigest(),
            "user_authorization_date": "2026-08-02",
            "user_authorization_text": authorization_text,
            "user_authorization_text_sha256": hashlib.sha256(
                authorization_text.encode("utf-8")
            ).hexdigest(),
        }
        authorization_path.write_text(
            json.dumps(authorization, sort_keys=True) + "\n", encoding="utf-8",
        )
        return {
            "protocol": protocol,
            "protocol_path": protocol_path,
            "markdown_path": markdown_path,
            "preflight_path": preflight_path,
            "authorization_path": authorization_path,
            "authorization": authorization,
        }

    def commit_registered_bundle(self, bundle, *extra_paths):
        paths = [
            bundle["protocol_path"], bundle["markdown_path"],
            bundle["preflight_path"], bundle["authorization_path"],
            *extra_paths,
        ]
        self.git("add", *(str(path.relative_to(self.project)) for path in paths))
        self.git("commit", "-qm", "authorization artifacts")

    def write_authorization(self, bundle, authorization=None):
        authorization = authorization or bundle["authorization"]
        bundle["authorization_path"].write_text(
            json.dumps(authorization, sort_keys=True) + "\n", encoding="utf-8",
        )

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
        for field in (
            "must_bind_implementation_identity",
            "must_bind_preflight_sha256",
            "must_bind_user_authorization",
        ):
            nested_missing = copy.deepcopy(protocol)
            nested_missing["authorization"].pop(field, None)
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

    def test_self_rehashed_nonliteral_initial_family_is_rejected_at_verification(self):
        protocol = self.authoritative(kind="development", version="v5")
        substitute = self.root / "same-bytes-external-initial-family.json"
        substitute.write_bytes(self.initial_family.read_bytes())
        protocol["bindings"]["initial_family"] = registrar._file_identity(
            substitute
        )
        commands = registrar._registered_argv(protocol)
        protocol["runner_argv"] = commands["runner"]
        protocol["analyzer_argv"] = commands["analyzer"]
        protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)

        with (
            self.authoritative_probes(protocol),
            mock.patch.object(
                registrar, "verify_development_predecessor_state",
                return_value=None,
            ),
        ):
            with self.assertRaisesRegex(ValueError, "literal initial-family"):
                verify_registered_protocol(protocol)

    def test_self_rehashed_supervision_or_authorization_semantics_are_rejected(self):
        for section, field, value in (
            ("supervision", "line_stall_timeout_s", 301.0),
            ("authorization", "must_bind_implementation_identity", False),
            ("authorization", "must_bind_preflight_sha256", False),
            ("authorization", "must_bind_user_authorization", False),
        ):
            with self.subTest(section=section, field=field):
                protocol = self.authoritative()
                protocol[section][field] = value
                protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
                with self.authoritative_probes(protocol):
                    with self.assertRaisesRegex(ValueError, "frozen contract"):
                        verify_registered_protocol(protocol)

    def test_protocol_freezes_complete_authorization_requirements(self):
        self.assertEqual(registrar.FROZEN_AUTHORIZATION_REQUIREMENTS, {
            "required": True,
            "must_bind_protocol_sha256": True,
            "must_bind_implementation_identity": True,
            "must_bind_preflight_sha256": True,
            "must_bind_user_authorization": True,
        })
        protocol = self.authoritative()
        self.assertEqual(
            set(protocol["authorization"]),
            set(registrar.FROZEN_AUTHORIZATION_REQUIREMENTS),
        )

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

    def test_exact_four_artifact_direct_child_authorizes_execution(self):
        bundle = self.prepare_registered_bundle()
        self.commit_registered_bundle(bundle)

        with self.binding_probes(bundle["protocol"]):
            verify_registered_protocol(bundle["protocol"])
            validated = validate_authorization_binding(
                bundle["protocol_path"], bundle["authorization_path"],
            )

        self.assertTrue(validated["authorized"])
        self.assertEqual(
            validated["implementation_identity"],
            bundle["protocol"]["repository"]["head"],
        )

    def test_exact_four_artifact_child_accepts_registered_untracked_build(self):
        """The frozen build may be allowed-untracked while source stays committed."""
        (self.project / ".gitignore").write_text("ignored-build.bin\n")
        self.git("init", "-q")
        self.git("add", ".")
        self.git("commit", "-qm", "implementation")

        build = self.project / "build-diagnostic"
        build.mkdir()
        binary = build / "Swarm"
        cmake_cache = build / "CMakeCache.txt"
        binary.write_bytes(b"frozen untracked executable\n")
        cmake_cache.write_text("ENABLE_GUROBI:BOOL=ON\n", encoding="utf-8")

        publication = self.project / "docs" / "diagnostics"
        protocol_path = publication / "fixture-protocol.json"
        markdown_path = publication / "fixture-protocol.md"
        protocol = self.authoritative(
            json_path=protocol_path,
            markdown_path=markdown_path,
            kind="development",
            version="v5",
        )
        protocol["bindings"]["binary"] = registrar._file_identity(binary)
        protocol["bindings"]["dependencies"] = registrar._file_identity(cmake_cache)
        protocol["build"]["cmake_cache"] = registrar._file_identity(cmake_cache)
        protocol["repository"] = registrar._repository_identity(self.project)
        commands = registrar._registered_argv(protocol)
        protocol["runner_argv"] = commands["runner"]
        protocol["analyzer_argv"] = commands["analyzer"]
        protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
        with self.binding_probes(protocol):
            publish_protocol(protocol, protocol_path, markdown_path)

        review_root = publication / "reviews"
        review_root.mkdir(parents=True)
        preflight_path = review_root / "fixture-preflight.md"
        authorization_path = review_root / "fixture-authorization.json"
        preflight_path.write_text("# Independent preflight\n\nC0/I0/M0.\n")
        authorization_text = "Continue with the registered development-v5 execution."
        authorization = {
            "schema_version": "cbf2026-qualified-authorization-v1",
            "authorized": True,
            "kind": "development",
            "version": "v5",
            "protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest(),
            "implementation_identity": protocol["repository"]["head"],
            "preflight_sha256": hashlib.sha256(preflight_path.read_bytes()).hexdigest(),
            "user_authorization_date": "2026-08-02",
            "user_authorization_text": authorization_text,
            "user_authorization_text_sha256": hashlib.sha256(
                authorization_text.encode("utf-8")
            ).hexdigest(),
        }
        authorization_path.write_text(
            json.dumps(authorization, sort_keys=True) + "\n", encoding="utf-8"
        )
        bundle = {
            "protocol": protocol,
            "protocol_path": protocol_path,
            "markdown_path": markdown_path,
            "preflight_path": preflight_path,
            "authorization_path": authorization_path,
        }
        self.commit_registered_bundle(bundle)

        self.assertEqual(
            protocol["repository"]["allowed_untracked_paths"],
            ["build-diagnostic/CMakeCache.txt", "build-diagnostic/Swarm"],
        )
        with self.binding_probes(protocol):
            validated = validate_authorization_binding(
                protocol_path, authorization_path,
            )
        self.assertTrue(validated["authorized"])

        original_binary = binary.read_bytes()
        binary.write_bytes(b"tampered frozen executable\n")
        with self.binding_probes(protocol):
            with self.assertRaisesRegex(ValueError, "binding mutated|allowed-untracked"):
                validate_authorization_binding(protocol_path, authorization_path)
        binary.write_bytes(original_binary)

        extra_build_member = build / "unregistered.cache"
        extra_build_member.write_bytes(b"not registered")
        with self.binding_probes(protocol):
            with self.assertRaisesRegex(ValueError, "protected paths"):
                validate_authorization_binding(protocol_path, authorization_path)
        extra_build_member.unlink()

        ignored_build = self.project / "ignored-build.bin"
        ignored_build.write_bytes(b"not a registered allowed-untracked file")
        non_allowlisted = copy.deepcopy(protocol)
        non_allowlisted["bindings"]["binary"] = registrar._file_identity(
            ignored_build
        )
        with self.assertRaisesRegex(ValueError, "registered Git state is unavailable"):
            registrar._verify_committed_registration_state(
                non_allowlisted,
                registrar._repository_identity(self.project),
            )

    def test_failed_development_v1_through_v4_are_consumed_and_v5_is_non_colliding(self):
        historical_paths = [
            self.root / f"development-{version}-protocol.json"
            for version in ("v1", "v2", "v3", "v4")
        ]
        historical_bytes = b'{"terminal":"failed"}\n'
        for historical_path in historical_paths:
            historical_path.write_bytes(historical_bytes)
        development_arguments = {
            "count": 10,
            "kind": "development",
            "trajectory_seeds": list(range(2026080201, 2026080211)),
            "range_noise_seeds": list(range(2026081201, 2026081211)),
            "roots": {
                label: Path(path)
                for label, path in self.frozen_roots["development"].items()
            },
        }

        for version in ("v1", "v2", "v3", "v4"):
            with self.subTest(version=version):
                with self.assertRaisesRegex(ValueError, "development.*v5"):
                    self.build(version=version, **development_arguments)
        protocol = self.build(version="v5", **development_arguments)

        for historical_path in historical_paths:
            self.assertEqual(historical_path.read_bytes(), historical_bytes)
        self.assertEqual(protocol["version"], "v5")
        self.assertEqual(protocol["roots"], self.frozen_roots["development"])
        self.assertTrue(protocol["roots"]["raw"].endswith("/v5"))
        self.assertTrue(protocol["roots"]["analysis"].endswith("/v5"))

    def test_uncommitted_artifacts_cannot_authorize_execution(self):
        bundle = self.prepare_registered_bundle()

        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "committed|artifact commit"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )

    def test_direct_child_with_extra_path_is_rejected(self):
        bundle = self.prepare_registered_bundle()
        extra = self.project / "docs" / "diagnostics" / "unexpected.md"
        extra.write_text("not part of the registered bundle\n", encoding="utf-8")
        self.commit_registered_bundle(bundle, extra)

        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "exact.*artifact|artifact-only"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )

    def test_direct_child_modifying_existing_source_is_rejected(self):
        bundle = self.prepare_registered_bundle()
        self.files["source.py"].write_text("changed source\n", encoding="utf-8")
        self.commit_registered_bundle(bundle, self.files["source.py"])

        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "binding mutated|artifact-only"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )

    def test_second_descendant_commit_is_rejected(self):
        bundle = self.prepare_registered_bundle()
        self.commit_registered_bundle(bundle)
        self.git("commit", "--allow-empty", "-qm", "later documentation commit")

        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "direct child"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )

    def test_non_descendant_head_is_rejected(self):
        bundle = self.prepare_registered_bundle()
        self.commit_registered_bundle(bundle)
        implementation = bundle["protocol"]["repository"]["head"]
        tree = self.git("rev-parse", f"{implementation}^{{tree}}").stdout.strip()
        unrelated = self.git(
            "commit-tree", tree, "-m", "unrelated history",
        ).stdout.strip()
        self.git("update-ref", "HEAD", unrelated)

        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "descendant"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )

    def test_authorization_preflight_text_and_date_tamper_are_rejected(self):
        bundle = self.prepare_registered_bundle()
        self.commit_registered_bundle(bundle)
        original = bundle["authorization_path"].read_bytes()
        mutations = (
            ("preflight_sha256", "0" * 64, "preflight SHA-256"),
            ("user_authorization_text", "replacement approval", "authorization text"),
            ("user_authorization_date", "02-08-2026", "authorization date"),
        )
        for field, value, error in mutations:
            with self.subTest(field=field):
                authorization = copy.deepcopy(bundle["authorization"])
                authorization[field] = value
                self.write_authorization(bundle, authorization)
                with self.binding_probes(bundle["protocol"]):
                    with self.assertRaisesRegex(ValueError, error):
                        validate_authorization_binding(
                            bundle["protocol_path"], bundle["authorization_path"],
                        )
                bundle["authorization_path"].write_bytes(original)

    def test_committed_artifact_and_markdown_companion_tamper_are_rejected(self):
        bundle = self.prepare_registered_bundle()
        self.commit_registered_bundle(bundle)
        original_authorization = bundle["authorization_path"].read_bytes()
        authorization = copy.deepcopy(bundle["authorization"])
        authorization["user_authorization_text"] = "different standing approval"
        authorization["user_authorization_text_sha256"] = hashlib.sha256(
            authorization["user_authorization_text"].encode("utf-8")
        ).hexdigest()
        self.write_authorization(bundle, authorization)
        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "Git blob|dirty relevant"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )
        bundle["authorization_path"].write_bytes(original_authorization)

        bundle["markdown_path"].write_text("replacement companion\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "companion Markdown"):
            validate_authorization_binding(
                bundle["protocol_path"], bundle["authorization_path"],
            )

    def test_dirty_tracked_and_untracked_relevant_paths_are_rejected(self):
        bundle = self.prepare_registered_bundle()
        self.commit_registered_bundle(bundle)
        source = self.files["source.py"]
        original_source = source.read_bytes()
        source.write_text("dirty tracked source\n", encoding="utf-8")
        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "binding mutated|dirty relevant"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )
        source.write_bytes(original_source)

        scripts = self.project / "scripts"
        scripts.mkdir()
        (scripts / "injected.py").write_text("pass\n", encoding="utf-8")
        with self.binding_probes(bundle["protocol"]):
            with self.assertRaisesRegex(ValueError, "dirty relevant"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"],
                )

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
                trajectory_seeds=[2026080201] + list(range(2026082002, 2026082061))
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

        self.assertEqual(registrar._runner_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.run_qualified_closure_campaign",
            "--kind", "confirmatory", "--version", "v1",
            "--protocol", "docs/diagnostics/frozen-protocol.json",
            "--authorization", authorization,
            "--binary", "build-diagnostic/Swarm",
            "--base-config", "config/config.json",
            "--primary-config", "config/diagnostics/primary.json",
            "--ablation-config", "config/diagnostics/ablation.json",
            "--trajectory-seeds", "2026082001:2026082060",
            "--range-noise-seeds", "2026083001:2026083060",
            "--frames", "1000",
            "--output-root", "/private/tmp/scientific-raw",
        ])
        self.assertEqual(registrar._analyzer_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.analyze_qualified_closure_campaign",
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
                "conda", "run", "-n", "cbf_env", "python", "-m",
                "scripts.diagnostics.run_qualified_closure_campaign",
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
                "conda", "run", "-n", "cbf_env", "python", "-m",
                "scripts.diagnostics.analyze_qualified_closure_campaign",
                "--kind", "confirmatory-smoke", "--version", "v1",
                "--smoke-id", smoke_id,
                "--protocol", "docs/diagnostics/frozen-protocol.json",
                "--authorization", authorization,
                "--input-root", raw, "--output-root", analysis,
            ])

    def test_development_runner_and_analyzer_argv_bind_v5_family_and_ablation(self):
        arguments = argparse.Namespace(
            kind="development", version="v5",
            protocol_json=Path("docs/diagnostics/development-protocol.json"),
            binary=Path("build-diagnostic/Swarm"),
            base_config=Path("config/config.json"),
            primary_config=Path("config/diagnostics/primary.json"),
            ablation_config=Path("config/diagnostics/ablation.json"),
            initial_family=Path("config/diagnostics/qualified_initial_family_v1.json"),
            trajectory_seeds="2026080201:2026080210",
            range_noise_seeds="2026081201:2026081210", frames=1000,
            raw_root=Path("/private/tmp/development-raw"),
            analysis_root=Path("/private/tmp/development-analysis"),
        )

        self.assertEqual(registrar._runner_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.run_qualified_closure_campaign",
            "--kind", "development", "--version", "v5",
            "--protocol", "docs/diagnostics/development-protocol.json",
            "--authorization",
            "docs/diagnostics/reviews/development-authorization.json",
            "--binary", "build-diagnostic/Swarm",
            "--base-config", "config/config.json",
            "--primary-config", "config/diagnostics/primary.json",
            "--ablation-config", "config/diagnostics/ablation.json",
            "--initial-family", "config/diagnostics/qualified_initial_family_v1.json",
            "--trajectory-seeds", "2026080201:2026080210",
            "--range-noise-seeds", "2026081201:2026081210",
            "--frames", "1000",
            "--output-root", "/private/tmp/development-raw",
        ])
        self.assertEqual(registrar._analyzer_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.analyze_qualified_closure_campaign",
            "--kind", "development", "--version", "v5",
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
        bundle = self.prepare_registered_bundle()
        self.commit_registered_bundle(bundle)
        protocol = bundle["protocol"]
        authorization = copy.deepcopy(bundle["authorization"])

        with self.binding_probes(protocol):
            validated = validate_authorization_binding(
                bundle["protocol_path"], bundle["authorization_path"]
            )
        self.assertTrue(validated["authorized"])
        authorization["implementation_identity"] = "f" * 40
        self.write_authorization(bundle, authorization)
        with self.binding_probes(protocol):
            with self.assertRaisesRegex(ValueError, "implementation identity"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"]
                )
        authorization["implementation_identity"] = protocol["repository"]["head"]
        authorization["protocol_sha256"] = "0" * 64
        self.write_authorization(bundle, authorization)
        with self.binding_probes(protocol):
            with self.assertRaisesRegex(ValueError, "protocol SHA-256"):
                validate_authorization_binding(
                    bundle["protocol_path"], bundle["authorization_path"]
                )

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
            kind="development", version="v5",
            implementation_report=report, implementation_review=review,
            development_protocol=None, development_report=None,
            development_review=None, binary=binary, base_config=base,
            primary_config=primary, ablation_config=ablation,
            initial_family=self.initial_family,
            trajectory_seeds="2026080201:2026080210",
            range_noise_seeds="2026081201:2026081210", frames=1000,
            smoke_trajectory_seed=None, smoke_range_noise_seed=None,
            smoke_frames=None, smoke_a_raw_root=None,
            smoke_a_analysis_root=None, smoke_b_raw_root=None,
            smoke_b_analysis_root=None,
            raw_root=self.root / "raw" / "v5",
            analysis_root=self.root / "analysis" / "v5",
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
            "runner_argv", "analyzer_argv", "initial_state", "semantic_sha256",
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

class QualifiedClosureV5InitialFamilyTests(unittest.TestCase):
    """Freeze the development-v5 initial-family and predecessor-state contract."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="qualified-v5-registrar-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.project = self.root / "repo"
        self.project.mkdir()
        self.family_path = (
            Path(__file__).resolve().parents[1]
            / "config" / "diagnostics" / "qualified_initial_family_v1.json"
        )
        self.files = {}
        for name in (
            "source.py", "Swarm", "base.json", "primary.json", "ablation.json",
            "dependencies.txt", "schema.json",
        ):
            path = self.project / name
            path.write_text(name, encoding="utf-8")
            self.files[name] = path
        self.files["primary.json"].write_text(json.dumps({
            "position_covariance": {"reference-selection": "dynamic-lower-index"}
        }), encoding="utf-8")
        self.files["ablation.json"].write_text(json.dumps({
            "position_covariance": {"reference-selection": "fixed-cbf-only"}
        }), encoding="utf-8")
        self.roots = {
            "raw": str(self.root / "development" / "v5"),
            "analysis": str(self.root / "development-analysis" / "v5"),
        }
        self.roots_patch = mock.patch.object(
            registrar, "FROZEN_EXECUTION_ROOTS", {
                **registrar.FROZEN_EXECUTION_ROOTS,
                "development": dict(self.roots),
            },
        )
        self.roots_patch.start()
        self.addCleanup(self.roots_patch.stop)

    def build(self, **overrides):
        arguments = {
            "kind": "development",
            "version": "v5",
            "project_root": self.project,
            "trajectory_seeds": list(range(2026080201, 2026080211)),
            "range_noise_seeds": list(range(2026081201, 2026081211)),
            "frames": 1000,
            "roots": dict(self.roots),
            "bindings": {
                "source": self.files["source.py"],
                "binary": self.files["Swarm"],
                "base_config": self.files["base.json"],
                "primary_config": self.files["primary.json"],
                "ablation_config": self.files["ablation.json"],
                "dependencies": self.files["dependencies.txt"],
                "schema": self.files["schema.json"],
                "initial_family": self.family_path,
            },
            "thresholds": copy.deepcopy(FROZEN_THRESHOLDS),
            "dirty_relevant_paths": [],
        }
        arguments.update(overrides)
        return build_qualified_closure_protocol(**arguments)

    def test_v5_protocol_binds_exact_initial_family_and_per_mission_hashes(self):
        protocol = self.build()

        self.assertEqual(
            protocol["schema_version"], "cbf2026-qualified-closure-protocol-v2"
        )
        self.assertEqual(protocol["version"], "v5")
        initial = protocol["initial_state"]
        self.assertEqual(set(initial), {
            "family_schema_version", "namespace", "family_semantic_sha256",
            "registered_trajectory_seeds", "audit_trajectory_seeds", "missions",
            "frozen_summary", "admission", "perturbation_policy",
        })
        self.assertEqual(initial["namespace"], "cbf2026-v5-initial")
        self.assertEqual(initial["registered_trajectory_seeds"], list(range(2026080201, 2026080211)))
        self.assertEqual(initial["audit_trajectory_seeds"], list(range(2026080201, 2026080301)))
        self.assertEqual(initial["perturbation_policy"], {"clamp": False, "resample": False})
        self.assertEqual(
            protocol["schedule"]["missions"],
            [
                {
                    "mission_id": f"mission-{index:02d}",
                    "trajectory_seed": item["trajectory_seed"],
                    "range_noise_seed": 2026081200 + index,
                    "frames": 1000,
                    "initial_positions_sha256": item["positions_sha256"],
                }
                for index, item in enumerate(initial["missions"], start=1)
            ],
        )

    def test_v4_and_old_development_seed_schedules_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "development.*v5"):
            self.build(version="v4")
        with self.assertRaisesRegex(ValueError, "frozen sequence"):
            self.build(trajectory_seeds=list(range(2026080101, 2026080111)))
        with self.assertRaisesRegex(ValueError, "frozen sequence"):
            self.build(range_noise_seeds=list(range(2026081101, 2026081111)))

    def test_family_duplicate_key_and_self_rehashed_mutations_are_rejected(self):
        duplicate = self.root / "duplicate-family.json"
        text = self.family_path.read_text(encoding="utf-8")
        duplicate.write_text(text.replace(
            '"namespace": "cbf2026-v5-initial",',
            '"namespace": "cbf2026-v5-initial",\n  "namespace": "cbf2026-v5-initial",',
            1,
        ), encoding="utf-8")
        bindings = dict(self.build()["bindings"])
        binding_paths = {
            label: Path(identity["path"]) for label, identity in bindings.items()
        }
        binding_paths["initial_family"] = duplicate
        with mock.patch.object(registrar, "FROZEN_INITIAL_FAMILY_PATH", duplicate):
            with self.assertRaisesRegex(ValueError, "duplicate|unreadable"):
                self.build(bindings=binding_paths)

        family = json.loads(self.family_path.read_text(encoding="utf-8"))
        family["template_positions_m"][0][0] += 0.01
        family["semantic_sha256"] = registrar._initial_family_semantic_sha256(family)
        mutated = self.root / "self-rehashed-family.json"
        mutated.write_text(json.dumps(family), encoding="utf-8")
        binding_paths["initial_family"] = mutated
        with mock.patch.object(registrar, "FROZEN_INITIAL_FAMILY_PATH", mutated):
            with self.assertRaises(ValueError):
                self.build(bindings=binding_paths)

        substitute = self.root / "same-bytes-different-family.json"
        substitute.write_bytes(self.family_path.read_bytes())
        binding_paths["initial_family"] = substitute
        with self.assertRaisesRegex(ValueError, "literal initial-family"):
            self.build(bindings=binding_paths)

        for label, mutate in (
            ("namespace", lambda value: value.__setitem__("namespace", "substitute")),
            ("summary", lambda value: value["frozen_summary"]["registered"].__setitem__(
                "minimum_barrier_m", value["frozen_summary"]["registered"]["minimum_barrier_m"] + 0.01
            )),
            ("admission", lambda value: value["admission"].__setitem__(
                "minimum_qp_margin_mps", 0.69
            )),
            ("seed", lambda value: value["schedule"]["registered_trajectory_seeds"].__setitem__(
                0, 2026080101
            )),
            ("position", lambda value: value["frozen_summary"].__setitem__(
                "representative_positions_sha256", "0" * 64
            )),
        ):
            with self.subTest(label=label):
                candidate = json.loads(self.family_path.read_text(encoding="utf-8"))
                mutate(candidate)
                candidate["semantic_sha256"] = registrar._initial_family_semantic_sha256(candidate)
                candidate_path = self.root / f"self-rehashed-{label}.json"
                candidate_path.write_text(json.dumps(candidate), encoding="utf-8")
                binding_paths["initial_family"] = candidate_path
                with mock.patch.object(
                    registrar, "FROZEN_INITIAL_FAMILY_PATH", candidate_path
                ):
                    with self.assertRaises(ValueError):
                        self.build(bindings=binding_paths)

    def test_self_rehashed_protocol_initial_state_substitution_is_rejected(self):
        protocol = self.build()
        protocol["initial_state"]["missions"][0]["positions_sha256"] = "0" * 64
        protocol["schedule"]["missions"][0]["initial_positions_sha256"] = "0" * 64
        protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
        with self.assertRaisesRegex(ValueError, "initial-state"):
            registrar._verify_initial_state_binding(protocol)

    def test_predecessor_tree_identity_rejects_absence_mutation_and_special_files(self):
        raw = self.root / "v4-raw"
        analysis = self.root / "v4-analysis"
        raw.mkdir()
        analysis.mkdir()
        for index in range(28):
            (raw / f"member-{index:02d}.dat").write_bytes(f"raw-{index}".encode())
        for index in range(3):
            (analysis / f"member-{index:02d}.dat").write_bytes(f"analysis-{index}".encode())
        raw_manifest = raw / "member-00.dat"
        analysis_manifest = analysis / "member-00.dat"
        frozen = {
            "absent": tuple(self.root / f"absent-v{version}-{kind}" for version in range(1, 4) for kind in ("raw", "analysis")),
            "terminal": {
                "raw": {**registrar._directory_tree_identity(raw), "root": str(raw), "manifest_sha256": registrar._sha256(raw_manifest), "manifest_name": raw_manifest.name},
                "analysis": {**registrar._directory_tree_identity(analysis), "root": str(analysis), "manifest_sha256": registrar._sha256(analysis_manifest), "manifest_name": analysis_manifest.name},
            },
        }
        with mock.patch.object(registrar, "FROZEN_DEVELOPMENT_PREDECESSOR_STATE", frozen):
            registrar.verify_development_predecessor_state()
            raw_manifest.write_bytes(b"mutated")
            with self.assertRaisesRegex(ValueError, "predecessor"):
                registrar.verify_development_predecessor_state()

        raw_manifest.write_bytes(b"raw-0")
        frozen["terminal"]["raw"] = {**registrar._directory_tree_identity(raw), "root": str(raw), "manifest_sha256": registrar._sha256(raw_manifest), "manifest_name": raw_manifest.name}
        shutil.rmtree(analysis)
        with mock.patch.object(registrar, "FROZEN_DEVELOPMENT_PREDECESSOR_STATE", frozen):
            with self.assertRaisesRegex(ValueError, "predecessor"):
                registrar.verify_development_predecessor_state()


class QualifiedClosureV6PredecessorTests(unittest.TestCase):
    """Bind v6 registration to the terminal, immutable v5 evidence roots."""

    identity_path = Path(
        "config/diagnostics/qualified_v6_predecessor_v5_identity_v1.json"
    )

    def copy_v6_predecessor_fixture(self):
        return json.loads(self.identity_path.read_text(encoding="utf-8"))

    def test_v6_predecessor_identity_matches_terminal_v5_roots(self):
        self.assertTrue(
            self.identity_path.is_file(),
            "the literal v6 predecessor identity file must exist",
        )
        self.assertTrue(
            callable(getattr(registrar, "load_v6_predecessor_identity", None)),
            "the v6 predecessor loader must exist",
        )
        identity = registrar.load_v6_predecessor_identity(self.identity_path)
        registrar.verify_v6_predecessor_state(identity)

    def test_v6_predecessor_rejects_tree_or_manifest_mutation(self):
        self.assertTrue(
            callable(getattr(registrar, "verify_v6_predecessor_state", None)),
            "the v6 predecessor verifier must exist",
        )
        identity = self.copy_v6_predecessor_fixture()
        identity["terminal"]["raw"]["tree_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "v5 predecessor"):
            registrar.verify_v6_predecessor_state(identity)


if __name__ == "__main__":
    unittest.main()
