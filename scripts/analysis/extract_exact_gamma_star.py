"""Extract exact joint gamma-star evidence from simulator ``data.json`` files."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import gzip
import json
import math
from pathlib import Path
from typing import Any, Iterable

from scripts.analysis.gamma_star_lp import ResidualConstraint, solve_gamma_star


@dataclass(frozen=True)
class ExtractionResult:
    records: list[dict[str, Any]]
    total_constraints: int
    skipped_constraints: int
    source: str


def _finite_float(value: Any) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None


def _constraint_from_hocbf(item: dict[str, Any]) -> ResidualConstraint | None:
    coefficient = item.get("coe")
    if not isinstance(coefficient, dict):
        return None
    cx = _finite_float(coefficient.get("ax"))
    cy = _finite_float(coefficient.get("ay"))
    offset = _finite_float(item.get("const"))
    if cx is None or cy is None or offset is None:
        return None
    return ResidualConstraint(
        name=str(item.get("name", "unnamed")),
        cx=cx,
        cy=cy,
        offset=offset,
    )


def _config_value(payload: dict[str, Any], keys: Iterable[str]) -> Any:
    current: Any = payload
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def extract_gamma_star_records(
    payload: dict[str, Any],
    source: str = "",
) -> ExtractionResult:
    """Return one exact joint-budget record per robot/frame with HOCBF rows."""

    half_box_value = _config_value(
        payload,
        ("config", "cbfs", "high-order", "acceleration-bound"),
    )
    half_box = _finite_float(half_box_value)
    if half_box is None or half_box < 0.0:
        raise ValueError("missing finite non-negative HOCBF acceleration-bound")

    feedback_box_value = _config_value(
        payload,
        (
            "config",
            "bridge",
            "nominal",
            "gamma-star-feedback",
            "accel-half-box",
        ),
    )
    feedback_box = _finite_float(feedback_box_value)
    if feedback_box is None:
        feedback_box = math.nan

    row_name = str(
        _config_value(payload, ("bridge", "metadata", "row"))
        or _config_value(payload, ("config", "bridge", "row"))
        or ""
    )
    run_suffix = str(_config_value(payload, ("config", "run_suffix")) or "")

    records: list[dict[str, Any]] = []
    total_constraints = 0
    skipped_constraints = 0
    for frame_index, frame in enumerate(payload.get("state", [])):
        runtime = _finite_float(frame.get("runtime"))
        for robot in frame.get("robots", []):
            opt = robot.get("opt", {})
            stored_groups = [
                (name, opt.get(name, []))
                for name in ("hocbfNoSlack", "hocbfSlack")
                if isinstance(opt.get(name, []), list) and opt.get(name, [])
            ]
            if not stored_groups:
                continue
            items = [
                item
                for _, group in stored_groups
                for item in group
            ]
            constraint_storage = "+".join(name for name, _ in stored_groups)
            constraints: list[ResidualConstraint] = []
            for item in items:
                total_constraints += 1
                constraint = (
                    _constraint_from_hocbf(item)
                    if isinstance(item, dict)
                    else None
                )
                if constraint is None:
                    skipped_constraints += 1
                    continue
                constraints.append(constraint)
            if not constraints:
                continue

            solution = solve_gamma_star(constraints, half_box=half_box)
            per_edge_surrogate = min(
                constraint.offset
                + half_box * (abs(constraint.cx) + abs(constraint.cy))
                for constraint in constraints
            )
            records.append(
                {
                    "source": source,
                    "row": row_name,
                    "run_suffix": run_suffix,
                    "frame_index": frame_index,
                    "runtime_s": runtime if runtime is not None else math.nan,
                    "robot_id": robot.get("id", ""),
                    "constraint_storage": constraint_storage,
                    "active_edge_count": len(constraints),
                    "gamma_star": solution.gamma,
                    "joint_feasible": int(solution.gamma >= -1.0e-9),
                    "optimal_ax": solution.acceleration[0],
                    "optimal_ay": solution.acceleration[1],
                    "per_edge_authority_surrogate": per_edge_surrogate,
                    "feasibility_half_box": half_box,
                    "feedback_score_and_candidate_half_box": feedback_box,
                    "active_residuals": "|".join(solution.active_residuals),
                    "active_box_constraints": "|".join(
                        solution.active_box_constraints
                    ),
                    "dual_multiplier_names_json": json.dumps(
                        solution.dual_multiplier_names,
                        separators=(",", ":"),
                    ),
                    "dual_multipliers_json": json.dumps(
                        solution.dual_multipliers.tolist(),
                        separators=(",", ":"),
                    ),
                    "primal_residual": solution.primal_residual,
                    "stationarity_residual": solution.stationarity_residual,
                    "complementarity_residual": solution.complementarity_residual,
                    "duality_gap": solution.duality_gap,
                }
            )

    return ExtractionResult(
        records=records,
        total_constraints=total_constraints,
        skipped_constraints=skipped_constraints,
        source=source,
    )


def summarize_trial(result: ExtractionResult) -> dict[str, Any]:
    if not result.records:
        raise ValueError("cannot summarize an extraction with no gamma-star records")
    return {
        "source": result.source,
        "record_count": len(result.records),
        "total_constraints": result.total_constraints,
        "skipped_constraints": result.skipped_constraints,
        "min_gamma_star": min(row["gamma_star"] for row in result.records),
        "joint_feasible_all": int(
            all(bool(row["joint_feasible"]) for row in result.records)
        ),
        "min_per_edge_authority_surrogate": min(
            row["per_edge_authority_surrogate"] for row in result.records
        ),
        "max_primal_residual": max(
            row["primal_residual"] for row in result.records
        ),
        "max_stationarity_residual": max(
            row["stationarity_residual"] for row in result.records
        ),
        "max_complementarity_residual": max(
            row["complementarity_residual"] for row in result.records
        ),
        "max_abs_duality_gap": max(
            abs(row["duality_gap"]) for row in result.records
        ),
    }


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError(f"refusing to write empty CSV: {path}")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def extract_paths(paths: list[Path], output_dir: Path) -> tuple[Path, Path]:
    sample_rows: list[dict[str, Any]] = []
    summary_rows: list[dict[str, Any]] = []
    for path in paths:
        if path.suffix == ".gz":
            with gzip.open(path, "rt", encoding="utf-8") as handle:
                payload = json.load(handle)
        else:
            payload = json.loads(path.read_text(encoding="utf-8"))
        result = extract_gamma_star_records(payload, source=str(path))
        sample_rows.extend(result.records)
        summary_rows.append(summarize_trial(result))
    samples_path = output_dir / "exact_gamma_star_samples.csv"
    trials_path = output_dir / "exact_gamma_star_trials.csv"
    _write_rows(samples_path, sample_rows)
    _write_rows(trials_path, summary_rows)
    return samples_path, trials_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    samples_path, trials_path = extract_paths(args.inputs, args.output_dir)
    print(samples_path)
    print(trials_path)


if __name__ == "__main__":
    main()
