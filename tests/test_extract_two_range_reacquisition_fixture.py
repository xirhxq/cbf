import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from scripts.diagnostics.extract_two_range_reacquisition_fixture import (
    FIXTURE_FIELDS,
    MECHANISM_KEY,
    V4_COMPRESSED_SHA256,
    V4_DECOMPRESSED_SHA256,
    V4_MANIFEST_SHA256,
    extract_mechanism_fixture,
    read_v4_manifest,
)


V4_ROOT = Path("/private/tmp/cbf2026-predictive-wnls-development/stage1-v4")


class FixtureIdentityTests(unittest.TestCase):
    def test_v4_manifest_identity(self):
        manifest, identity = read_v4_manifest(V4_ROOT)

        self.assertEqual(
            identity["path"],
            str(V4_ROOT / "manifest.json"),
        )
        self.assertEqual(
            identity["sha256"],
            "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223",
        )
        self.assertEqual(identity["size"], 9560)
        self.assertEqual(
            V4_MANIFEST_SHA256,
            "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223",
        )
        self.assertEqual(
            manifest["compressed_process_sha256"],
            "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b",
        )

    def test_extracts_only_approved_mechanism_deterministically(self):
        with tempfile.TemporaryDirectory() as directory:
            first = Path(directory) / "first.json"
            second = Path(directory) / "second.json"
            extract_mechanism_fixture(v4_root=V4_ROOT, output=first)
            extract_mechanism_fixture(v4_root=V4_ROOT, output=second)

            first_bytes = first.read_bytes()
            self.assertEqual(first_bytes, second.read_bytes())
            fixture = json.loads(first_bytes)
            self.assertEqual(tuple(fixture), FIXTURE_FIELDS)
            self.assertEqual(
                (
                    fixture["key"]["seed"],
                    fixture["key"]["frame_index"],
                    fixture["key"]["robot_id"],
                ),
                MECHANISM_KEY,
            )
            self.assertEqual(
                fixture["mandatory_references"],
                {"base_ids": [], "uav_ids": [10, 11]},
            )
            self.assertEqual(fixture["optional_references"], [])
            self.assertEqual(
                fixture["measurements"]["reference_keys"],
                [["uav", 10], ["uav", 11]],
            )
            self.assertEqual(
                fixture["measurements"]["ranges"],
                [564.4909904970261, 539.4621394498664],
            )
            self.assertEqual(
                fixture["preceding_private_state"]["source_fresh_frame"],
                177,
            )
            self.assertEqual(
                fixture["preceding_private_state"]["propagated_to_frame"],
                180,
            )
            self.assertEqual(
                fixture["preceding_private_state"]["age_frames"],
                3,
            )
            self.assertEqual(
                fixture["preceding_private_state"]["estimate"],
                [-310.7993304339704, 1056.9366015086234],
            )
            self.assertEqual(
                fixture["expected_mechanism"],
                {
                    "active_reference_count": 2,
                    "active_reference_keys": [["uav", 10], ["uav", 11]],
                    "old_failure_reason": (
                        "reacquisition_requires_three_active_references"
                    ),
                    "old_candidate_count": 3,
                    "old_all_candidates_converged": True,
                },
            )
            identities = fixture["source_identities"]
            self.assertEqual(
                identities["v4_manifest"]["sha256"],
                V4_MANIFEST_SHA256,
            )
            self.assertEqual(
                identities["v4_compressed_process"]["sha256"],
                V4_COMPRESSED_SHA256,
            )
            self.assertEqual(
                identities["v4_decompressed_process"]["sha256"],
                V4_DECOMPRESSED_SHA256,
            )
            self.assertEqual(
                identities["truth_data"]["sha256"],
                "3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527",
            )
            self.assertTrue(first_bytes.endswith(b"\n"))
            self.assertFalse(first_bytes.endswith(b"\n\n"))
            self.assertEqual(
                hashlib.sha256(first_bytes).hexdigest(),
                hashlib.sha256(second.read_bytes()).hexdigest(),
            )

    def test_committed_fixture_manifest_binds_exact_bytes_and_sources(self):
        root = Path("tests/fixtures/cbf2026_two_range_reacquisition")
        fixture_bytes = (root / "mechanism_20260727_180_12.json").read_bytes()
        fixture_manifest = json.loads((root / "manifest.json").read_bytes())
        extractor_bytes = Path(
            "scripts/diagnostics/extract_two_range_reacquisition_fixture.py"
        ).read_bytes()

        self.assertEqual(fixture_manifest["fixture_size"], len(fixture_bytes))
        self.assertEqual(
            fixture_manifest["fixture_sha256"],
            hashlib.sha256(fixture_bytes).hexdigest(),
        )
        self.assertEqual(
            fixture_manifest["extractor_identity"]["sha256"],
            hashlib.sha256(extractor_bytes).hexdigest(),
        )
        self.assertEqual(
            fixture_manifest["source_sha256"],
            {
                "v4_manifest": V4_MANIFEST_SHA256,
                "v4_compressed_process": V4_COMPRESSED_SHA256,
                "v4_decompressed_process": V4_DECOMPRESSED_SHA256,
                "truth_data": (
                    "3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527"
                ),
            },
        )


if __name__ == "__main__":
    unittest.main()
