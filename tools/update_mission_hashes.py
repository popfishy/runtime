#!/usr/bin/env python3
"""Refresh one or more mission manifests after an explicit configuration edit."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Dict, List


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def referenced_files(manifest: Dict[str, object]) -> List[str]:
    required = ["tree_file", "robots_file", "world_file", "bootstrap_file"]
    references = []
    for field in required:
        value = manifest.get(field)
        if not isinstance(value, str) or not value:
            raise ValueError("mission.json requires non-empty %s" % field)
        references.append(value)
    plans = manifest.get("plan_files")
    if not isinstance(plans, list) or not plans or not all(
        isinstance(item, str) and item for item in plans
    ):
        raise ValueError("mission.json requires a non-empty plan_files list")
    references.extend(plans)
    if len(references) != len(set(references)):
        raise ValueError("mission.json contains duplicate referenced files")
    return references


def update_package(package_dir: Path, mark_reviewed: bool) -> None:
    package_dir = package_dir.resolve()
    manifest_path = package_dir / "mission.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    references = referenced_files(manifest)
    hashes = {}
    for relative in references:
        path = (package_dir / relative).resolve()
        if package_dir not in path.parents or not path.is_file():
            raise ValueError("invalid or missing package file: %s" % relative)
        hashes[relative] = sha256_file(path)
    manifest["file_hashes"] = hashes
    manifest["reviewed"] = bool(mark_reviewed)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    state = "reviewed" if mark_reviewed else "NOT reviewed"
    print("updated %s (%s)" % (manifest_path, state))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Update mission file hashes; review stays false unless explicitly approved"
    )
    parser.add_argument("packages", nargs="+", type=Path)
    parser.add_argument(
        "--mark-reviewed",
        action="store_true",
        help="mark the current exact files as human-reviewed",
    )
    args = parser.parse_args()
    try:
        for package in args.packages:
            update_package(package, args.mark_reviewed)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
