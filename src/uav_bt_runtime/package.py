"""Reviewed mission-package loading, hashing, and scenario static validation."""

from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Set, Union

from pydantic import Field, JsonValue, ValidationError, field_validator, model_validator

from .actions import build_action_registry
from .bt import (
    ActionNode,
    BehaviorTreeParseError,
    BehaviorTreeParser,
    ParsedBehaviorTree,
    SequenceNode,
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
        if 2.0 * self.safety_margin_m >= min(
            self.max_x - self.min_x, self.max_y - self.min_y
        ):
            raise ValueError("flight-area safety margin leaves no usable area")
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
    validate_scenario_package(package)
    return package


def _actions_by_id(package: MissionPackage) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for reference in package.parsed_tree.action_references:
        if reference.action_id in result:
            raise MissionPackageError(
                f"semantic Action {reference.action_id!r} appears more than once"
            )
        result[reference.action_id] = reference.plan_id or ""
    return result


def _plan_for_action(
    package: MissionPackage, actions: Dict[str, str], action_id: str
) -> TaskPlan:
    plan_id = actions.get(action_id)
    if not plan_id:
        raise MissionPackageError(f"required Action {action_id!r} has no plan_id")
    return package.context.plans[plan_id]


def _require_exact_assignments(plan: TaskPlan, expected: Set[str], purpose: str) -> None:
    actual = set(plan.robot_assignments)
    if actual != expected:
        raise MissionPackageError(
            f"{purpose} assignments differ: missing={sorted(expected - actual)}, "
            f"unexpected={sorted(actual - expected)}"
        )


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


def validate_scenario_package(package: MissionPackage) -> None:
    """Apply the fixed v1 Group A/Group B task-shape rules."""

    actions = _actions_by_id(package)
    context = package.context
    all_ids = set(context.uav_specs)
    _validate_reviewed_scene_coordinates(package)

    if context.group_id == "GroupA":
        expected_order = [
            "PrepareGroupA",
            "PublishGroupReady",
            "WaitGroupBReady",
            "CoverageSegment1",
            "SimulateDamage",
            "RecoverReconGroup",
            "CoverageSegment2",
            "PublishReconComplete",
            "WaitStrikeComplete",
            "HoldCoverageEnd",
        ]
        root = package.parsed_tree.tree.root
        if not isinstance(root, SequenceNode) or not all(
            isinstance(child, ActionNode) for child in root.children
        ):
            raise MissionPackageError("GroupA tree must be one Sequence of semantic Actions")
        actual_order = [child.action_id for child in root.children]
        if actual_order != expected_order:
            raise MissionPackageError(
                f"GroupA Action order differs: expected={expected_order}, actual={actual_order}"
            )

        initial_active = set(context.roster.active_ids)
        initial_reserves = list(context.roster.reserve_ids)
        prepare = _plan_for_action(package, actions, "PrepareGroupA")
        coverage1 = _plan_for_action(package, actions, "CoverageSegment1")
        fault = _plan_for_action(package, actions, "SimulateDamage")
        recovery = _plan_for_action(package, actions, "RecoverReconGroup")
        coverage2 = _plan_for_action(package, actions, "CoverageSegment2")
        hold_plan = _plan_for_action(package, actions, "HoldCoverageEnd")

        _require_exact_assignments(prepare, initial_active, "GroupA preparation")
        _require_exact_assignments(coverage1, initial_active, "coverage segment 1")
        fault_ids = set(fault.robot_assignments)
        if len(fault_ids) != context.planned_fault_count:
            raise MissionPackageError("fault plan size must equal planned_fault_count")
        if not fault_ids <= initial_active:
            raise MissionPackageError("fault candidates must be initially active")
        if any(context.uav_specs[uav_id].is_leader for uav_id in fault_ids):
            raise MissionPackageError("fault candidates cannot include a Leader")

        activated = set(initial_reserves[: context.planned_recovery_count])
        recovered_active = (initial_active - fault_ids) | activated
        _require_exact_assignments(recovery, recovered_active, "fixed recovery")
        _require_exact_assignments(coverage2, recovered_active, "coverage segment 2")
        _require_exact_assignments(hold_plan, recovered_active, "GroupA end hold")
        coverage_routes: Dict[str, List[JsonValue]] = {}
        for coverage in (coverage1, coverage2):
            route_owners = [
                robot_id
                for robot_id, assignment in coverage.robot_assignments.items()
                if isinstance(assignment.payload.get("waypoints"), list)
                and bool(assignment.payload.get("waypoints"))
            ]
            if len(route_owners) != 1 or not context.uav_specs[route_owners[0]].is_leader:
                raise MissionPackageError(
                    f"coverage plan {coverage.plan_id!r} must contain exactly one Leader route"
                )
            route = coverage.robot_assignments[route_owners[0]].payload.get("waypoints")
            if not isinstance(route, list) or len(route) < 3:
                raise MissionPackageError(
                    f"coverage plan {coverage.plan_id!r} requires at least three waypoints"
                )
            coverage_routes[coverage.plan_id] = route
            for robot_id, assignment in coverage.robot_assignments.items():
                if robot_id == route_owners[0]:
                    continue
                if assignment.payload != {"formation_follow": True}:
                    raise MissionPackageError(
                        f"coverage follower {robot_id!r} must use formation_follow=true"
                    )

        route1 = coverage_routes[coverage1.plan_id]
        route2 = coverage_routes[coverage2.plan_id]

        def route_position(point: JsonValue, source: str) -> tuple:
            if not isinstance(point, Mapping):
                raise MissionPackageError(f"{source} must be a waypoint object")
            values = []
            for key in ("x", "y", "z"):
                value = point.get(key)
                if not isinstance(value, (int, float)) or isinstance(value, bool):
                    raise MissionPackageError(f"{source}.{key} must be numeric")
                values.append(float(value))
            return tuple(values)

        first_end = route_position(route1[-1], "coverage segment 1 endpoint")
        second_start = route_position(route2[0], "coverage segment 2 start")
        if any(
            not math.isclose(left, right, abs_tol=1e-6)
            for left, right in zip(first_end, second_start)
        ):
            raise MissionPackageError(
                "coverage segment 2 must start at segment 1's fault/recovery point"
            )
        split_y = first_end[1]
        first_scan_y = [
            route_position(point, "coverage segment 1 waypoint")[1]
            for point in route1[:-1]
        ]
        second_scan_y = [
            route_position(point, "coverage segment 2 waypoint")[1]
            for point in route2[1:]
        ]
        if any(y > split_y + 1e-6 for y in first_scan_y):
            raise MissionPackageError("coverage segment 1 enters the upper region")
        if any(y < split_y - 1e-6 for y in second_scan_y):
            raise MissionPackageError("coverage segment 2 returns to the scanned lower region")
        if first_scan_y[-1] <= first_scan_y[0] or second_scan_y[-1] <= second_scan_y[0]:
            raise MissionPackageError(
                "coverage routes must progress from the lower area to the upper area"
            )

        leader_ids = [
            robot_id
            for robot_id in initial_active
            if context.uav_specs[robot_id].is_leader
        ]
        if len(leader_ids) != 1:
            raise MissionPackageError("GroupA requires exactly one active Leader")
        leader_id = leader_ids[0]
        recovery_target = recovery.robot_assignments[leader_id].payload.get("target_pose")
        hold_target = hold_plan.robot_assignments[leader_id].payload.get("hold_pose")
        for actual, expected, source in (
            (recovery_target, route2[0], "recovery Leader target"),
            (hold_target, route2[-1], "final Leader hold"),
        ):
            actual_position = route_position(actual, source)
            expected_position = route_position(expected, source + " route reference")
            if any(
                not math.isclose(left, right, abs_tol=1e-6)
                for left, right in zip(actual_position, expected_position)
            ):
                raise MissionPackageError(f"{source} must match its coverage endpoint")
        for robot_id, assignment in fault.robot_assignments.items():
            waypoints = assignment.payload.get("waypoints")
            if not isinstance(waypoints, list) or len(waypoints) < 2:
                raise MissionPackageError(
                    f"fault-exit UAV {robot_id!r} requires descend and exit waypoints"
                )
        return

    if context.group_id == "GroupB":
        root = package.parsed_tree.tree.root
        expected_order = [
            "PublishGroupReady",
            "WaitGroupAReady",
            "WaitReconComplete",
            "StrikeTargets",
            "ReturnStrikeUavs",
            "PublishStrikeComplete",
        ]
        if not isinstance(root, SequenceNode) or not all(
            isinstance(child, ActionNode) for child in root.children
        ):
            raise MissionPackageError("GroupB tree must be one Sequence of semantic Actions")
        actual_order = [child.action_id for child in root.children]
        if actual_order != expected_order:
            raise MissionPackageError(
                f"GroupB Action order differs: expected={expected_order}, actual={actual_order}"
            )

        strike = _plan_for_action(package, actions, "StrikeTargets")
        return_plan = _plan_for_action(package, actions, "ReturnStrikeUavs")
        strike_ids = set(strike.robot_assignments)
        _require_exact_assignments(return_plan, strike_ids, "GroupB strike return")
        if len(strike_ids) != 2:
            raise MissionPackageError("GroupB strike plan must assign exactly two UAVs")
        if not strike_ids <= set(context.roster.active_ids):
            raise MissionPackageError("strike UAVs must be active and flight-eligible")
        target_ids = {
            assignment.payload.get("target_id")
            for assignment in strike.robot_assignments.values()
        }
        if len(target_ids) != 2 or None in target_ids:
            raise MissionPackageError("the two strike UAVs must use two distinct targets")
        if target_ids != set(package.world.targets):
            raise MissionPackageError(
                "strike target IDs must exactly match the two reviewed world targets"
            )
        for robot_id, assignment in return_plan.robot_assignments.items():
            waypoints = assignment.payload.get("waypoints")
            if not isinstance(waypoints, list) or not waypoints:
                raise MissionPackageError(
                    f"return UAV {robot_id!r} requires a reviewed Route"
                )
        return

    raise MissionPackageError(f"unsupported Group {context.group_id!r}")


def validate_joint_packages(
    first: MissionPackage, second: MissionPackage, minimum_uavs: int = 30
) -> None:
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
    if group_a.world.target_config_version != group_b.world.target_config_version:
        raise MissionPackageError("target configuration versions do not match")
    if sha256_file(group_a.directory / group_a.manifest.world_file) != sha256_file(
        group_b.directory / group_b.manifest.world_file
    ):
        raise MissionPackageError("Group packages do not share identical world configuration")

    group_a_ids = set(group_a.context.uav_specs)
    group_b_ids = set(group_b.context.uav_specs)
    if group_a_ids & group_b_ids:
        raise MissionPackageError("the same UAV ID appears in both Groups")
    if len(group_a_ids | group_b_ids) < minimum_uavs:
        raise MissionPackageError(
            f"joint mission requires at least {minimum_uavs} UAVs, "
            f"found {len(group_a_ids | group_b_ids)}"
        )
