"""Build a deterministic SHA-256 manifest for paper evidence artifacts."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class EvidenceEntry:
    logical_role: str
    path: Path
    provenance: str
    artifact_type: str


_FIELDS = (
    "logical_role",
    "path",
    "bytes",
    "sha256",
    "provenance",
    "artifact_type",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def build_manifest_rows(
    root: Path,
    entries: Iterable[EvidenceEntry],
) -> list[dict[str, object]]:
    root = root.resolve()
    rows: list[dict[str, object]] = []
    for entry in entries:
        resolved = entry.path.resolve()
        try:
            relative = resolved.relative_to(root)
        except ValueError as error:
            raise ValueError(f"artifact is outside manifest root: {resolved}") from error
        if not resolved.is_file():
            raise FileNotFoundError(resolved)
        if entry.artifact_type not in {"raw", "derived", "source"}:
            raise ValueError(
                "artifact_type must be one of: raw, derived, source"
            )
        rows.append(
            {
                "logical_role": entry.logical_role,
                "path": relative.as_posix(),
                "bytes": resolved.stat().st_size,
                "sha256": _sha256(resolved),
                "provenance": entry.provenance,
                "artifact_type": entry.artifact_type,
            }
        )
    return sorted(rows, key=lambda row: (str(row["artifact_type"]), str(row["path"])))


def write_manifest(
    root: Path,
    entries: Iterable[EvidenceEntry],
    output: Path,
) -> Path:
    rows = build_manifest_rows(root, entries)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return output


def _entry_from_spec(root: Path, spec: str) -> EvidenceEntry:
    parts = spec.split("::", 3)
    if len(parts) != 4:
        raise ValueError(
            "entry must be TYPE::ROLE::PROVENANCE::RELATIVE_PATH"
        )
    artifact_type, logical_role, provenance, relative_path = parts
    return EvidenceEntry(
        logical_role=logical_role,
        path=root / relative_path,
        provenance=provenance,
        artifact_type=artifact_type,
    )


def read_entry_specs(root: Path, spec_file: Path) -> list[EvidenceEntry]:
    entries: list[EvidenceEntry] = []
    for line in spec_file.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        entries.append(_entry_from_spec(root, stripped))
    return entries


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--entry",
        action="append",
        default=[],
        help="TYPE::ROLE::PROVENANCE::RELATIVE_PATH",
    )
    parser.add_argument(
        "--spec-file",
        type=Path,
        help="newline-delimited entry specs; blank lines and # comments are ignored",
    )
    args = parser.parse_args()
    entries = [_entry_from_spec(args.root, spec) for spec in args.entry]
    if args.spec_file is not None:
        entries.extend(read_entry_specs(args.root, args.spec_file))
    print(write_manifest(args.root, entries, args.output))


if __name__ == "__main__":
    main()
