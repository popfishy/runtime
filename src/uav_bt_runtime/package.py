"""Reviewed mission-package loading, hashing, and reference-integrity validation."""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Union

from pydantic import Field, JsonValue, field_validator, model_validator

from .actions import build_action_registry
from .bt import (
    BehaviorTreeParseError,
    BehaviorTreeParser,
    ParsedBehaviorTree,
)
from .config import (
    ConfigurationLoadError,
    MissionBootstrapConfig,
    PlanCatalog,
    UavCatalog,
    build_mission_context,
    load_json_model,
)
from .models import Identifier, MissionContext, StrictModel, TaskPlan


PathLike = Union[str, Path]
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class MissionPackageError(ValueError):
    """Raised when a reviewed mission package is unsafe or inconsistent."""


class FlightArea(StrictModel):
    """Rectangular mission boundary and inward safety margin, in mission units."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    safety_margin_m: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_bounds(self) -> "FlightArea":
        if self.max_x <= self.min_x or self.max_y <= self.min_y:
            raise ValueError("flight-area maximums must exceed minimums")
        return self


class WorldConfig(StrictModel):
    schema_version: Identifier = "1.0"
    coordinate_frame: Identifier
    target_config_version: Identifier
    flight_area: FlightArea
    targets: Dict[str, JsonValue] = Field(min_length=1)


class MissionManifest(StrictModel):
    schema_version: Identifier = "1.0"
    package_version: Identifier
    peer_package_version: Identifier
    mission_id: Identifier
    group_id: Identifier
    reviewed: bool
    tree_file: str
    robots_file: str
    world_file: str
    bootstrap_file: str
    plan_files: List[str] = Field(min_length=1)
    target_config_version: Identifier
    file_hashes: Dict[str, str]

    @field_validator(
        "tree_file", "robots_file", "world_file", "bootstrap_file", "plan_files"
    )
    @classmethod
    def reject_absolute_paths(cls, value):
        values = value if isinstance(value, list) else [value]
        for item in values:
            path = Path(item)
            if path.is_absolute() or ".." in path.parts:
                raise ValueError("manifest paths must stay inside the package directory")
        return value

    @field_validator("file_hashes")
    @classmethod
    def validate_sha256_values(cls, value: Dict[str, str]) -> Dict[str, str]:
        for path, digest in value.items():
            if not SHA256_PATTERN.fullmatch(digest):
                raise ValueError(f"invalid SHA-256 digest for {path!r}")
        return value


@dataclass(frozen=True)
class MissionPackage:
    directory: Path
    manifest: MissionManifest
    world: WorldConfig
    context: MissionContext
    parsed_tree: ParsedBehaviorTree


def sha256_file(path: PathLike) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _package_path(directory: Path, relative_path: str) -> Path:
    candidate = (directory / relative_path).resolve()
    root = directory.resolve()
    if candidate != root and root not in candidate.parents:
        raise MissionPackageError(f"package path escapes root: {relative_path!r}")
    if not candidate.is_file():
        raise MissionPackageError(f"referenced package file does not exist: {relative_path!r}")
    return candidate


def load_mission_package(
    directory: PathLike, *, require_reviewed: bool = True
) -> MissionPackage:
    """Load, hash-check, parse, and statically validate one Group package."""

    package_dir = Path(directory).resolve()
    manifest_path = package_dir / "mission.json"
    try:
        manifest = load_json_model(manifest_path, MissionManifest)
    except ConfigurationLoadError as exc:
        raise MissionPackageError(str(exc)) from exc
    if require_reviewed and not manifest.reviewed:
        raise MissionPackageError("mission package is not marked as reviewed")

    referenced = [
        manifest.tree_file,
        manifest.robots_file,
        manifest.world_file,
        manifest.bootstrap_file,
        *manifest.plan_files,
    ]
    if len(referenced) != len(set(referenced)):
        raise MissionPackageError("manifest contains duplicate referenced paths")
    if set(manifest.file_hashes) != set(referenced):
        raise MissionPackageError("file_hashes must exactly cover every referenced package file")

    paths = {relative: _package_path(package_dir, relative) for relative in referenced}
    for relative, path in paths.items():
        actual = sha256_file(path)
        expected = manifest.file_hashes[relative]
        if actual != expected:
            raise MissionPackageError(
                f"SHA-256 mismatch for {relative!r}: expected {expected}, got {actual}"
            )

    try:
        world = load_json_model(paths[manifest.world_file], WorldConfig)
        uavs = load_json_model(paths[manifest.robots_file], UavCatalog)
        bootstrap = load_json_model(
            paths[manifest.bootstrap_file], MissionBootstrapConfig
        )
        plan_lists = [
            load_json_model(paths[plan_file], PlanCatalog).plans
            for plan_file in manifest.plan_files
        ]
    except ConfigurationLoadError as exc:
        raise MissionPackageError(str(exc)) from exc

    plans: List[TaskPlan] = [plan for plan_list in plan_lists for plan in plan_list]
    plan_ids = [plan.plan_id for plan in plans]
    if len(plan_ids) != len(set(plan_ids)):
        raise MissionPackageError("plan IDs must be unique across all plan files")

    if manifest.mission_id != bootstrap.mission_id:
        raise MissionPackageError("manifest and bootstrap mission_id values differ")
    if manifest.group_id != bootstrap.group_id:
        raise MissionPackageError("manifest and bootstrap group_id values differ")
    if manifest.target_config_version != world.target_config_version:
        raise MissionPackageError("manifest and world target_config_version values differ")

    try:
        context = build_mission_context(uavs, bootstrap, PlanCatalog(plans=plans))
    except ConfigurationLoadError as exc:
        raise MissionPackageError(str(exc)) from exc

    parser = BehaviorTreeParser(build_action_registry())
    try:
        parsed_tree = parser.parse(paths[manifest.tree_file])
    except BehaviorTreeParseError as exc:
        raise MissionPackageError(str(exc)) from exc

    for reference in parsed_tree.action_references:
        if reference.plan_id is None:
            continue
        plan = context.plans.get(reference.plan_id)
        if plan is None:
            raise MissionPackageError(
                f"Action {reference.action_id!r} references unknown plan "
                f"{reference.plan_id!r}"
            )
        if plan.group_id != context.group_id:
            raise MissionPackageError(
                f"Action {reference.action_id!r} references another Group's plan"
            )

    package = MissionPackage(package_dir, manifest, world, context, parsed_tree)
    for plan in context.plans.values():
        for assignment in plan.robot_assignments.values():
            target_id = assignment.payload.get("target_id")
            if target_id is not None and target_id not in world.targets:
                raise MissionPackageError(
                    f"plan {plan.plan_id!r} references unknown target_id {target_id!r}"
                )
    _validate_reviewed_scene_coordinates(package)
    return package


def _validate_reviewed_scene_coordinates(package: MissionPackage) -> None:
    """Require explicit flight-plan coordinates to be numeric.

    Real-flight coordinate bounds are owned by each ground station.  Runtime
    forwards the reviewed task plan and must not reject a valid local origin.
    """

    def validate_pose(pose: Mapping[str, object], source: str) -> None:
        x = pose.get("x")
        y = pose.get("y")
        if (
            not isinstance(x, (int, float))
            or isinstance(x, bool)
            or not isinstance(y, (int, float))
            or isinstance(y, bool)
        ):
            raise MissionPackageError(f"{source} must contain numeric x/y coordinates")
    for plan in package.context.plans.values():
        for assignment in plan.robot_assignments.values():
            for key in ("target_pose", "hold_pose"):
                value = assignment.payload.get(key)
                if isinstance(value, Mapping):
                    validate_pose(
                        value,
                        f"plan {plan.plan_id!r} UAV {assignment.robot_id!r} {key}",
                    )
            waypoints = assignment.payload.get("waypoints")
            if isinstance(waypoints, list):
                for index, waypoint in enumerate(waypoints):
                    if isinstance(waypoint, Mapping):
                        validate_pose(
                            waypoint,
                            f"plan {plan.plan_id!r} UAV {assignment.robot_id!r} "
                            f"waypoint {index}",
                        )


def validate_joint_packages(first: MissionPackage, second: MissionPackage) -> None:
    """Validate cross-station invariants before a joint mission starts."""

    packages = {first.context.group_id: first, second.context.group_id: second}
    if set(packages) != {"GroupA", "GroupB"}:
        raise MissionPackageError("joint mission requires exactly GroupA and GroupB packages")
    group_a = packages["GroupA"]
    group_b = packages["GroupB"]
    if group_a.manifest.mission_id != group_b.manifest.mission_id:
        raise MissionPackageError("Group packages have different mission_id values")
    if (
        group_a.manifest.peer_package_version != group_b.manifest.package_version
        or group_b.manifest.peer_package_version != group_a.manifest.package_version
    ):
        raise MissionPackageError("peer package versions do not match")

    group_a_ids = set(group_a.context.uav_specs)
    group_b_ids = set(group_b.context.uav_specs)
    if group_a_ids & group_b_ids:
        raise MissionPackageError("the same UAV ID appears in both Groups")
