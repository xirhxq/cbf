import json
import copy
import unittest
from pathlib import Path

from scripts.diagnostics.qualified_config import validate_qualified_config


ROOT = Path(__file__).resolve().parents[1]
PRIMARY = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json"
ABLATION = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json"
PRIMARY_V2 = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json"
ABLATION_V2 = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v2.json"


class QualifiedConfigTests(unittest.TestCase):
    def load(self, path: Path):
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)

    def test_registered_primary_overlay_is_strictly_accepted(self):
        self.assertTrue(validate_qualified_config(self.load(PRIMARY)))

    def test_ablation_is_valid_and_changes_only_reference_selection(self):
        primary = self.load(PRIMARY)
        ablation = self.load(ABLATION)

        self.assertTrue(validate_qualified_config(ablation))
        projected = copy.deepcopy(ablation)
        projected["position_covariance"]["reference-selection"] = (
            primary["position_covariance"]["reference-selection"]
        )
        self.assertEqual(projected, primary)
        self.assertEqual(
            ablation["position_covariance"]["reference-selection"],
            "fixed-cbf-only",
        )

    def test_v2_overlays_require_the_exact_interior_selection_contract(self):
        self.assertTrue(PRIMARY_V2.exists())
        self.assertTrue(ABLATION_V2.exists())
        if not PRIMARY_V2.exists() or not ABLATION_V2.exists():
            return

        primary = self.load(PRIMARY_V2)
        ablation = self.load(ABLATION_V2)
        expected_policy = {
            "mode": "planar-chebyshev-fraction-cap-v1",
            "fraction": 0.1,
            "cap-mps": 0.1,
            "feasibility-tolerance-mps": 1e-9,
        }
        expected_alpha = {"coe": 0.1, "pow": 1}
        for candidate in (primary, ablation):
            self.assertTrue(validate_qualified_config(candidate))
            self.assertEqual(
                candidate["cbfs"]["hard-interior-selection"], expected_policy
            )
            self.assertEqual(
                candidate["cbfs"]["without-slack"]["safety"]["alpha"],
                expected_alpha,
            )
            self.assertEqual(
                candidate["cbfs"]["without-slack"]["comm-fixed"]["alpha"],
                expected_alpha,
            )

        projected = copy.deepcopy(ablation)
        projected["position_covariance"]["reference-selection"] = (
            primary["position_covariance"]["reference-selection"]
        )
        self.assertEqual(projected, primary)

    def test_v2_interior_selection_rejects_mutations(self):
        self.assertTrue(PRIMARY_V2.exists())
        if not PRIMARY_V2.exists():
            return

        mutations = []

        def mutate(path, value):
            candidate = self.load(PRIMARY_V2)
            target = candidate
            for key in path[:-1]:
                target = target[key]
            if value is None:
                del target[path[-1]]
            else:
                target[path[-1]] = value
            mutations.append(candidate)

        mutate(("cbfs", "hard-interior-selection", "fraction"), 0.2)
        mutate(("cbfs", "hard-interior-selection", "cap-mps"), 0.5)
        mutate(("cbfs", "hard-interior-selection", "feasibility-tolerance-mps"), 1e-8)
        mutate(("cbfs", "hard-interior-selection", "yaw-in-radius"), True)
        mutate(("cbfs", "without-slack", "safety", "alpha", "coe"), 0.2)
        mutate(("cbfs", "without-slack", "comm-fixed", "alpha", "pow"), 2)
        mutate(("execute", "execution-mode"), "centralized")

        for candidate in mutations:
            with self.subTest(candidate=candidate):
                self.assertFalse(validate_qualified_config(candidate))

    def test_rejects_every_forbidden_contract_mutation(self):
        mutations = []

        def mutate(path, value):
            candidate = self.load(PRIMARY)
            target = candidate
            for key in path[:-1]:
                target = target[key]
            if value is None:
                del target[path[-1]]
            else:
                target[path[-1]] = value
            mutations.append(candidate)

        mutate(("qualified-estimator", "mode-tolerance-m"), None)
        mutate(
            ("qualified-estimator", "sensitivity-tolerances-m"),
            [0.001, 0.0005, 0.002],
        )
        mutate(
            ("qualified-estimator", "deployment", "unit-normal"),
            [2.0, 0.0],
        )
        mutate(
            ("qualified-estimator", "deployment", "unit-normal"),
            [True, False],
        )
        mutate(
            ("qualified-estimator", "deployment", "offset"),
            1549.0,
        )
        mutate(
            ("qualified-estimator", "deployment", "anchor-ids"),
            [False, 2],
        )
        mutate(
            ("qualified-estimator", "deployment", "ocean-side"),
            True,
        )
        mutate(
            ("qualified-estimator", "deployment", "deployment-vertices"),
            [[-1550.0, 0.0], [-1370.0, -200.0], [-1370.0, 200.0], [-1490.0, 200.0]],
        )
        mutate(
            ("qualified-estimator", "history", "q-threshold"),
            11.0,
        )
        mutate(
            ("qualified-estimator", "history", "process-noise-diagonal"),
            [0.5, 0.25],
        )
        mutate(
            ("qualified-estimator", "history", "public-max-age-frames"),
            3,
        )
        mutate(
            ("qualified-estimator", "history", "public-max-age-frames"),
            True,
        )
        mutate(
            ("position_covariance", "reference-selection"),
            "nearest-reference",
        )
        mutate(("cbfs", "input-limits", "planar-component-max"), 24.0)
        mutate(("cbfs", "input-limits", "on"), 1)
        mutate(("cbfs", "without-slack", "safety", "on"), 1)
        mutate(("cbfs", "without-slack", "comm-fixed", "on"), 1)
        mutate(("execute", "time-step"), 0.25)
        candidate = self.load(PRIMARY)
        candidate["unexpected"] = True
        mutations.append(candidate)

        for candidate in mutations:
            with self.subTest(candidate=candidate):
                self.assertFalse(validate_qualified_config(candidate))


if __name__ == "__main__":
    unittest.main()
