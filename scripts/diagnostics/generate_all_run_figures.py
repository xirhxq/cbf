"""Generate every draw-framework figure for the G1-1303 full-horizon run.

Outputs are written into the run's timestamp folder (the folder that
contains data.json), mirroring the archived 2026-01-28 figure set plus
new estimator-specific components.
"""

from __future__ import annotations

import sys
import traceback
from pathlib import Path


DATA = Path(
    "/private/tmp/r1h-ekf-full/G1/2026081303/"
    "2026-08-04_19-09-15_R1H-EI_seed_20260727_350s/data.json"
)

GLOBAL = [
    "sp",
    "sh",
    "cbf-comm",
    "cbf-comm-energy",
    "comm-uncertainty",
    "comm-uncertainty-maxrange",
    "centralized-cbf",
    "centralized-comm",
    "centralized-cvt",
    "optimization-failure",
    "cvt-center-density",
    "position-uncertainty",
    "uncertainty-heatmap",
    "h_loc",
    "h_safe",
    "position-uncertainty-representative",
    "position-uncertainty-boxplot",
    "estimator-error",
    "constraint-margin",
    "ekf-epsilon",
    "ekf-epsilon-boxplot",
]

SEPARATE_UAV7 = [
    "cbf",
    "cvt",
    "min",
    "energy",
    "u",
    "fix",
    "heat",
    "cbc",
    "cbf-energy",
    "cbc-energy",
    "dh-energy",
    "constraint_cbfs",
    "task_cbfs",
    "slack_vars",
]

GROUP = ["cvt", "min", "energy", "u"]


def main() -> int:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "draw"))
    from drawers.static_global import StaticGlobalPlotDrawer
    from drawers.static_group import StaticGroupPlotDrawer
    from drawers.static_separate import StaticSeparatePlotDrawer

    files = [str(DATA)]
    results: list[tuple[str, str]] = []

    def run(label: str, fn) -> None:
        try:
            fn()
            results.append((label, "ok"))
            print(f"[OK] {label}")
        except Exception as exc:  # noqa: BLE001
            results.append((label, f"FAIL: {exc!r}"))
            print(f"[FAIL] {label}: {exc!r}")
            traceback.print_exc(limit=2)

    for comp in GLOBAL:
        run(
            f"global:{comp}",
            lambda comp=comp: StaticGlobalPlotDrawer(files).draw_plots([comp]),
        )
    for comp in SEPARATE_UAV7:
        run(
            f"uav7:{comp}",
            lambda comp=comp: StaticSeparatePlotDrawer(files).draw_plots(
                [comp], id_list=[7]
            ),
        )
    for comp in GROUP:
        run(
            f"group:{comp}",
            lambda comp=comp: StaticGroupPlotDrawer(files).draw_plots([comp]),
        )

    print("\n=== summary ===")
    for label, status in results:
        print(f"{label:16s} {status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
