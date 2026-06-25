import csv
import pathlib
import sys
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from analysis.second_order_reserve_frontier import summarize_reserve_frontier


FIELDNAMES = [
    "row",
    "trial_count",
    "completion_count",
    "completion_rate",
    "detection_count",
    "detection_rate",
    "mean_final_coverage",
    "min_physical_comm_margin_min",
    "min_hocbf_min",
    "min_control_authority_margin_min",
    "terminal_joint_hocbf_feasible_ratio_min",
    "terminal_hocbf_infeasible_robot_count_max",
    "min_support_chain_margin_after_min",
    "terminal_support_chain_margin_after_min",
    "min_robust_margin_min",
    "min_fim_eigenvalue_min",
]


def write_aggregate(path: pathlib.Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class SecondOrderReserveFrontierTest(unittest.TestCase):
    def test_frontier_keeps_canonical_modes_and_topology_deltas(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            root = pathlib.Path(tmpdir)
            topology_csv = root / "topology.csv"
            edge_csv = root / "edge.csv"
            state_csv = root / "state.csv"
            all_edge_csv = root / "all_edge.csv"

            base_row = {
                "trial_count": 6,
                "detection_count": 3,
                "detection_rate": 0.5,
                "mean_final_coverage": 0.26,
                "min_physical_comm_margin_min": 10.0,
                "min_hocbf_min": -10.0,
                "min_control_authority_margin_min": -2.0,
                "terminal_joint_hocbf_feasible_ratio_min": 0.75,
                "terminal_hocbf_infeasible_robot_count_max": 1,
                "min_support_chain_margin_after_min": -140.0,
                "terminal_support_chain_margin_after_min": -48.0,
                "min_robust_margin_min": -144.0,
                "min_fim_eigenvalue_min": 0.77,
            }
            write_aggregate(
                topology_csv,
                [
                    {
                        **base_row,
                        "row": "LD_HOCBF_PRED",
                        "completion_count": 4,
                        "completion_rate": 4 / 6,
                    }
                ],
            )
            write_aggregate(
                edge_csv,
                [
                    {
                        **base_row,
                        "row": "LD_HOCBF_PRED_EDGE",
                        "completion_count": 1,
                        "completion_rate": 1 / 6,
                        "min_robust_margin_min": 25.0,
                        "min_support_chain_margin_after_min": 22.0,
                    }
                ],
            )
            write_aggregate(
                state_csv,
                [
                    {
                        **base_row,
                        "row": "LD_HOCBF_PRED_SD",
                        "completion_count": 4,
                        "completion_rate": 4 / 6,
                        "min_control_authority_margin_min": -0.6,
                        "min_robust_margin_min": -142.0,
                    }
                ],
            )
            write_aggregate(
                all_edge_csv,
                [
                    {
                        **base_row,
                        "row": "LD_HOCBF_PRED_AE",
                        "completion_count": 4,
                        "completion_rate": 4 / 6,
                        "detection_count": 4,
                        "detection_rate": 4 / 6,
                        "mean_final_coverage": 0.29,
                        "min_physical_comm_margin_min": 23.0,
                        "min_robust_margin_min": -96.0,
                        "min_support_chain_margin_after_min": -95.0,
                    }
                ],
            )

            rows = summarize_reserve_frontier(
                [
                    topology_csv,
                    edge_csv,
                    state_csv,
                    all_edge_csv,
                ]
            )

        self.assertEqual(
            [row["mode"] for row in rows],
            [
                "topology_reserve",
                "constant_edge_reserve",
                "state_dependent_edge_reserve",
                "all_active_edge_reserve",
            ],
        )
        self.assertEqual(rows[1]["completion_delta_vs_topology"], -3.0)
        self.assertEqual(rows[1]["robust_margin_delta_m"], 169.0)
        self.assertEqual(rows[3]["detection_delta_vs_topology"], 1.0)
        self.assertEqual(rows[3]["support_chain_margin_delta_m"], 45.0)


if __name__ == "__main__":
    unittest.main()
