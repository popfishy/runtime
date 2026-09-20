"""Lightweight kinematic backend used by the 2-D mission visualizer.

The behavior tree still owns task ordering.  This module only turns the
per-UAV commands emitted by Action handlers into visible, deterministic
motion and returns normal ``CommandFeedback`` objects to the runtime.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .clock import Clock
from .models import CommandEnvelope, CommandFeedback, CommandStatus
from .package import MissionPackage
from .transport import ScriptedBackend, ScriptedOutcome


@dataclass(frozen=True)
class FieldConfig:
    """Physical top-down field dimensions in metres."""

    width_m: float = 100.0
    height_m: float = 150.0
    margin_m: float = 5.0

    def __post_init__(self) -> None:
        if self.width_m <= 0 or self.height_m <= 0:
            raise ValueError("field width and height must be positive")
        if self.margin_m < 0:
            raise ValueError("field margin cannot be negative")


@dataclass
class Point3D:
    x: float
    y: float
    z: float

    def copy(self) -> "Point3D":
        return Point3D(self.x, self.y, self.z)

    def distance_to(self, other: "Point3D") -> float:
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )

    def move_towards(self, other: "Point3D", distance: float) -> float:
        """Move by at most ``distance`` and return unused movement distance."""

        remaining = self.distance_to(other)
        if remaining <= 1e-9:
            self.x, self.y, self.z = other.x, other.y, other.z
            return distance
        if distance >= remaining:
            self.x, self.y, self.z = other.x, other.y, other.z
            return distance - remaining
        ratio = distance / remaining
        self.x += (other.x - self.x) * ratio
        self.y += (other.y - self.y) * ratio
        self.z += (other.z - self.z) * ratio
        return 0.0


@dataclass(frozen=True)
class CoordinateMapper:
    """Uniformly map reviewed plan coordinates inside a physical field."""

    field: FieldConfig
    source_min_x: float
    source_max_x: float
    source_min_y: float
    source_max_y: float
    scale: float
    offset_x: float
    offset_y: float

    @classmethod
    def from_packages(
        cls, packages: Sequence[MissionPackage], field: FieldConfig
    ) -> "CoordinateMapper":
        coordinates = list(_iter_package_coordinates(packages))
        if not coordinates:
            raise ValueError("mission packages contain no mappable coordinates")
        min_x = min(point[0] for point in coordinates)
        max_x = max(point[0] for point in coordinates)
        min_y = min(point[1] for point in coordinates)
        max_y = max(point[1] for point in coordinates)
        # Reviewed coordinates are forwarded 1:1.  Field bounds and any scaling
        # are owned by the ground station, so the visualiser must not rescale.
        return cls(
            field=field,
            source_min_x=min_x,
            source_max_x=max_x,
            source_min_y=min_y,
            source_max_y=max_y,
            scale=1.0,
            offset_x=0.0,
            offset_y=0.0,
        )

    def map_xyz(self, x: float, y: float, z: float) -> Point3D:
        return Point3D(
            x=float(x) * self.scale + self.offset_x,
            y=float(y) * self.scale + self.offset_y,
            z=float(z),
        )

    def map_pose(self, pose: Mapping[str, object]) -> Point3D:
        return self.map_xyz(
            _number(pose, "x"),
            _number(pose, "y"),
            _number(pose, "z", default=12.0),
        )


@dataclass
class KinematicUav:
    uav_id: str
    group_id: str
    role: str
    is_leader: bool
    configured_reserve: bool
    pose: Point3D
    motion_state: str = "INITIAL_HOVER"
    last_result: Optional[CommandStatus] = None
    trail: List[Tuple[float, float]] = field(default_factory=list)

    def record_trail(self) -> None:
        point = (self.pose.x, self.pose.y)
        if not self.trail or math.dist(self.trail[-1], point) >= 0.35:
            self.trail.append(point)
        if len(self.trail) > 350:
            del self.trail[: len(self.trail) - 350]


@dataclass
class _MotionRecord:
    command: CommandEnvelope
    outcome: ScriptedOutcome
    route: List[Point3D]
    failure_delay_s: float
    route_index: int = 0
    elapsed_s: float = 0.0
    running_emitted: bool = False
    terminal: bool = False


class KinematicCommandTransport:
    """Command transport whose feedback is produced by visible UAV movement."""

    def __init__(
        self,
        package: MissionPackage,
        mapper: CoordinateMapper,
        clock: Clock,
        backend: Optional[ScriptedBackend] = None,
        horizontal_speed_mps: float = 12.0,
        vertical_speed_mps: float = 8.0,
    ) -> None:
        if horizontal_speed_mps <= 0 or vertical_speed_mps <= 0:
            raise ValueError("UAV simulation speeds must be positive")
        self.package = package
        self.group_id = package.context.group_id
        self.mapper = mapper
        self.clock = clock
        self.backend = backend or ScriptedBackend()
        self.horizontal_speed_mps = horizontal_speed_mps
        self.vertical_speed_mps = vertical_speed_mps
        self.uavs = _initial_uavs(package, mapper)
        self.route_offsets = _build_route_offsets(package, mapper)
        self._records: Dict[str, _MotionRecord] = {}
        self._pending: List[CommandFeedback] = []
        self.history: List[CommandEnvelope] = []
        self.attack_successes: List[str] = []
        self.closed = False

    def _feedback(
        self,
        command: CommandEnvelope,
        status: CommandStatus,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
    ) -> CommandFeedback:
        return CommandFeedback(
            mission_id=command.mission_id,
            action_instance_id=command.action_instance_id,
            command_id=command.command_id,
            group_id=command.group_id,
            robot_id=command.robot_id,
            status=status,
            error_code=error_code,
            error_message=error_message,
            timestamp=self.clock.time(),
        )

    def send(self, command: CommandEnvelope) -> None:
        if self.closed:
            raise RuntimeError("kinematic command transport is closed")
        if command.group_id != self.group_id:
            raise ValueError(
                f"transport for {self.group_id!r} cannot send command for "
                f"{command.group_id!r}"
            )
        if command.command_id in self._records:
            raise ValueError(f"duplicate command_id {command.command_id!r}")
        uav = self.uavs.get(command.robot_id)
        if uav is None:
            raise ValueError(f"unknown UAV {command.robot_id!r}")
        outcome = self.backend.outcome_for(command)
        route = self._route_for(command, uav) if outcome.status == CommandStatus.SUCCESS else []
        record = _MotionRecord(
            command=command,
            outcome=outcome,
            route=route,
            failure_delay_s=_failure_delay(
                command.command_type,
                self.mapper.field,
                self.vertical_speed_mps,
            ),
        )
        self._records[command.command_id] = record
        self.history.append(command)
        uav.motion_state = command.command_type
        uav.last_result = None
        self._pending.append(self._feedback(command, CommandStatus.ACCEPTED))

    def _route_for(self, command: CommandEnvelope, uav: KinematicUav) -> List[Point3D]:
        payload = command.payload
        if command.command_type == "MOVE_TO":
            target_pose = payload.get("target_pose")
            if not isinstance(target_pose, Mapping):
                raise ValueError(f"{command.command_type} requires target_pose")
            return [self.mapper.map_pose(target_pose)]
        if command.command_type == "FOLLOW_ROUTE":
            waypoints = payload.get("waypoints")
            plan_id = str(payload.get("_plan_id", ""))
            if not isinstance(waypoints, list) or not waypoints:
                plan = self.package.context.plans.get(plan_id)
                if plan is None or not bool(payload.get("formation_follow")):
                    raise ValueError("FOLLOW_ROUTE requires a Leader route")
                leader_routes = [
                    assignment.payload.get("waypoints")
                    for robot_id, assignment in plan.robot_assignments.items()
                    if self.package.context.uav_specs[robot_id].is_leader
                ]
                if len(leader_routes) != 1 or not isinstance(leader_routes[0], list):
                    raise ValueError("FOLLOW_ROUTE plan has no unique Leader route")
                waypoints = leader_routes[0]
            offset_x, offset_y = self.route_offsets.get(
                (plan_id, command.robot_id), (0.0, 0.0)
            )
            route = []
            for waypoint in waypoints:
                if not isinstance(waypoint, Mapping):
                    raise ValueError("FOLLOW_ROUTE waypoints must be objects")
                point = self.mapper.map_pose(waypoint)
                point.x += offset_x
                point.y += offset_y
                route.append(point)
            return route
        if command.command_type in {"FAULT_EXIT", "RETURN"}:
            waypoints = payload.get("waypoints")
            if not isinstance(waypoints, list) or not waypoints:
                raise ValueError(f"{command.command_type} requires non-empty waypoints")
            return [
                self.mapper.map_pose(waypoint)
                for waypoint in waypoints
                if isinstance(waypoint, Mapping)
            ]
        if command.command_type == "ATTACK":
            target_id = payload.get("target_id")
            target = self.package.world.targets.get(str(target_id))
            if not isinstance(target, Mapping):
                raise ValueError(f"ATTACK references unknown target {target_id!r}")
            destination = self.mapper.map_pose(target)
            corridor = Point3D(
                (uav.pose.x + destination.x) / 2.0,
                uav.pose.y,
                destination.z,
            )
            return [corridor, destination]
        if command.command_type in {"HOVER", "SAFE_HOLD"}:
            return []
        raise ValueError(f"unsupported kinematic command {command.command_type!r}")

    def advance(self, seconds: float) -> None:
        if seconds <= 0:
            raise ValueError("simulation step must be positive")
        for record in list(self._records.values()):
            if record.terminal:
                continue
            command = record.command
            uav = self.uavs[command.robot_id]
            if self.clock.time() >= command.expires_at:
                self._finish(record, CommandStatus.TIMEOUT)
                continue
            if not record.running_emitted:
                record.running_emitted = True
                self._pending.append(self._feedback(command, CommandStatus.RUNNING))

            record.elapsed_s += seconds
            if record.outcome.status != CommandStatus.SUCCESS:
                if record.elapsed_s >= record.failure_delay_s:
                    self._finish(
                        record,
                        record.outcome.status,
                        record.outcome.error_code,
                        record.outcome.error_message,
                    )
                continue

            if record.route:
                speed = (
                    self.vertical_speed_mps
                    if command.command_type == "FAULT_EXIT"
                    else self.horizontal_speed_mps
                )
                movement = speed * seconds
                while movement > 1e-9 and record.route_index < len(record.route):
                    movement = uav.pose.move_towards(
                        record.route[record.route_index], movement
                    )
                    if uav.pose.distance_to(record.route[record.route_index]) <= 1e-6:
                        record.route_index += 1
                uav.record_trail()
                if record.route_index >= len(record.route):
                    self._finish(record, CommandStatus.SUCCESS)
            elif record.elapsed_s >= 0.4:
                self._finish(record, CommandStatus.SUCCESS)

    def _finish(
        self,
        record: _MotionRecord,
        status: CommandStatus,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
    ) -> None:
        if record.terminal:
            return
        record.terminal = True
        uav = self.uavs[record.command.robot_id]
        uav.last_result = status
        if status == CommandStatus.SUCCESS:
            if record.command.command_type == "FAULT_EXIT":
                uav.motion_state = "FAULT_EXIT_HOLD"
            elif record.command.command_type == "ATTACK":
                uav.motion_state = "ATTACK_COMPLETE"
                self.attack_successes.append(uav.uav_id)
            else:
                uav.motion_state = "HOVER"
        elif status == CommandStatus.CANCELLED:
            uav.motion_state = "SAFE_HOLD"
        else:
            uav.motion_state = f"{record.command.command_type}_{status.value}"
        self._pending.append(
            self._feedback(record.command, status, error_code, error_message)
        )

    def cancel(self, command_id: str) -> None:
        record = self._records.get(command_id)
        if record is None or record.terminal:
            return
        self._finish(record, CommandStatus.CANCELLED)

    def poll_feedback(self) -> List[CommandFeedback]:
        feedback = list(self._pending)
        self._pending.clear()
        return feedback

    def close(self) -> None:
        self.closed = True


def _iter_package_coordinates(
    packages: Sequence[MissionPackage],
) -> Iterable[Tuple[float, float]]:
    for package in packages:
        for target in package.world.targets.values():
            if isinstance(target, Mapping) and "x" in target and "y" in target:
                yield float(target["x"]), float(target["y"])
        for plan in package.context.plans.values():
            for assignment in plan.robot_assignments.values():
                target_pose = assignment.payload.get("target_pose")
                if isinstance(target_pose, Mapping):
                    yield _number(target_pose, "x"), _number(target_pose, "y")
                hold_pose = assignment.payload.get("hold_pose")
                if isinstance(hold_pose, Mapping):
                    yield _number(hold_pose, "x"), _number(hold_pose, "y")
                waypoints = assignment.payload.get("waypoints", [])
                if isinstance(waypoints, list):
                    for waypoint in waypoints:
                        if isinstance(waypoint, Mapping):
                            yield _number(waypoint, "x"), _number(waypoint, "y")


def _initial_uavs(
    package: MissionPackage, mapper: CoordinateMapper
) -> Dict[str, KinematicUav]:
    result: Dict[str, KinematicUav] = {}
    for index, (uav_id, spec) in enumerate(package.context.uav_specs.items()):
        if package.context.group_id == "GroupA":
            if index < 12:
                pose = Point3D(
                    x=5.0 + (index % 4) * 5.0,
                    y=5.0 + (index // 4) * 5.0,
                    z=12.0,
                )
            else:
                pose = Point3D(x=25.0 + (index - 12) * 5.0, y=10.0, z=0.0)
        else:
            pose = Point3D(
                x=80.0 + (index % 4) * 5.0,
                y=5.0 + (index // 4) * 5.0,
                z=0.0,
            )
        uav = KinematicUav(
            uav_id=uav_id,
            group_id=spec.group_id,
            role=spec.role,
            is_leader=spec.is_leader,
            configured_reserve=spec.is_reserve,
            pose=pose,
        )
        uav.record_trail()
        result[uav_id] = uav
    return result


def _build_route_offsets(
    package: MissionPackage, mapper: CoordinateMapper, spacing_m: float = 5.0
) -> Dict[Tuple[str, str], Tuple[float, float]]:
    """Mirror the lower-layer 3x4/3x3 formation for visual simulation only."""

    del mapper
    offsets: Dict[Tuple[str, str], Tuple[float, float]] = {}
    for plan in package.context.plans.values():
        robot_ids = sorted(
            robot_id
            for robot_id, assignment in plan.robot_assignments.items()
            if assignment.command_type == "FOLLOW_ROUTE"
        )
        if not robot_ids:
            continue
        columns = 4 if len(robot_ids) == 12 else 3
        rows = int(math.ceil(len(robot_ids) / float(columns)))
        ordered = list(robot_ids)
        leaders = [
            robot_id
            for robot_id in ordered
            if package.context.uav_specs[robot_id].is_leader
        ]
        if len(leaders) != 1:
            raise ValueError(f"plan {plan.plan_id!r} requires exactly one Leader")
        leader_id = leaders[0]
        ordered.remove(leader_id)
        leader_index = min(
            (rows // 2) * columns + max(0, (columns - 1) // 2),
            len(robot_ids) - 1,
        )
        ordered.insert(leader_index, leader_id)
        slots = {
            robot_id: (
                (index % columns) * spacing_m,
                (index // columns) * spacing_m,
            )
            for index, robot_id in enumerate(ordered)
        }
        leader_x, leader_y = slots[leader_id]
        for robot_id, (x, y) in slots.items():
            offsets[(plan.plan_id, robot_id)] = (x - leader_x, y - leader_y)
    return offsets


def _number(
    mapping: Mapping[str, object], key: str, default: Optional[float] = None
) -> float:
    value = mapping.get(key, default)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError(f"coordinate {key!r} must be numeric")
    return float(value)


def _numeric_suffix(identifier: str) -> int:
    digits = "".join(character for character in identifier if character.isdigit())
    return int(digits or "0")


def _failure_delay(
    command_type: str,
    field: FieldConfig,
    vertical_speed_mps: float,
) -> float:
    if command_type == "FAULT_EXIT":
        # A partial-failure scenario should preserve siblings that completed
        # their exits.  Bound the delay by the whole-field diagonal instead of
        # coupling it to one particular fault-point coordinate set.
        return math.hypot(field.width_m, field.height_m) / vertical_speed_mps + 2.0
    return 1.2


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
