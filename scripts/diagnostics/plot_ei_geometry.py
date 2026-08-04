"""Plot EI+joint coverage curves across the G1/G2/G3 geometry matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


TOTAL_CELLS = 90_000


def _coverage(data_path: Path) -> list[float]:
    frames = json.loads(data_path.read_text())["state"]
    covered: set[tuple[int, int]] = set()
    series = []
    for frame in frames:
        for cell in frame.get("update", []):
            if (
                isinstance(cell, list)
                and len(cell) >= 2
                and isinstance(cell[0], int)
                and isinstance(cell[1], int)
            ):
                covered.add((cell[0], cell[1]))
        series.append(len(covered) / TOTAL_CELLS)
    return series


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry-run", action="append", required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("/private/tmp/r1h-figs"))
    arguments = parser.parse_args(argv)

    runs: dict[str, list[tuple[str, Path]]] = {}
    for spec in arguments.geometry_run:
        geometry, seed, run_root = spec.split(":", 2)
        matches = sorted(Path(run_root).glob("*/data.json"))
        if not matches:
            raise SystemExit(f"no data.json under {run_root}")
        runs.setdefault(geometry, []).append((seed, matches[-1]))

    fig, axes = plt.subplots(1, len(runs), figsize=(5.5 * len(runs), 4.4))
    if len(runs) == 1:
        axes = [axes]
    colors = ["tab:blue", "tab:green", "tab:red"]
    for axis, (geometry, entries) in zip(axes, runs.items()):
        for color, (seed, data_path) in zip(colors, entries):
            series = _coverage(data_path)
            t = np.arange(len(series)) * 0.5
            axis.plot(t, np.asarray(series) * 100, lw=1.4, color=color,
                      label=f"noise {seed[-4:]}")
        axis.axhline(100, color="gray", ls=":", lw=1)
        axis.set_xlabel("t (s)")
        axis.set_ylabel("coverage (%)")
        axis.set_title(geometry)
        axis.set_ylim(0, 105)
        axis.legend(fontsize=8)
        axis.grid(alpha=0.3)
    fig.tight_layout()
    out = arguments.out_dir
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "ei-geometry-coverage.png", dpi=160)
    plt.close(fig)
    print("written", out / "ei-geometry-coverage.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
