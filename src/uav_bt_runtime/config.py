"""Fail-closed JSON loading for phase-2 runtime configuration models."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Type, TypeVar, Union

from pydantic import Field, JsonValue, NonNegativeInt, ValidationError, field_validator

from .models import (
    Identifier,
    GroupRoster,
    MissionContext,
    StrictModel,
    TaskPlan,
    UavRuntimeState,
    UavSpec,
)


ModelT = TypeVar("ModelT", bound=StrictModel)
PathLike = Union[str, Path]


class ConfigurationLoadError(ValueError):
    """Raised when a configuration file cannot be read or validated."""


class UavCatalog(StrictModel):
    """Versioned static UAV configuration, optionally shared by both Groups."""

    schema_version: Identifier = "1.0"
    uavs: List[UavSpec] = Field(min_length=1)

    @field_validator("uavs")
    @classmethod
    def reject_duplicate_uavs(cls, value: List[UavSpec]) -> List[UavSpec]:
        uav_ids = [uav.uav_id for uav in value]
        if len(uav_ids) != len(set(uav_ids)):
            raise ValueError("UAV catalog contains duplicate uav_id values")
        return value


class PlanCatalog(StrictModel):
    """Versioned set of reviewed task plans."""

    schema_version: Identifier = "1.0"
    plans: List[TaskPlan] = Field(default_factory=list)

    @field_validator("plans")
    @classmethod
    def reject_duplicate_plans(cls, value: List[TaskPlan]) -> List[TaskPlan]:
        plan_ids = [plan.plan_id for plan in value]
        if len(plan_ids) != len(set(plan_ids)):
            raise ValueError("plan catalog contains duplicate plan_id values")
        return value


class MissionBootstrapConfig(StrictModel):
    """Per-Group startup data used to build a validated MissionContext."""

    schema_version: Identifier = "1.0"
    mission_id: Identifier
    group_id: Identifier
    roster: GroupRoster
    initial_states: Optional[List[UavRuntimeState]] = None
    blackboard: Dict[str, JsonValue] = Field(default_factory=dict)
    current_phase: Optional[Identifier] = None
    planned_fault_count: NonNegativeInt = 0
    planned_recovery_count: NonNegativeInt = 0

    @field_validator("initial_states")
    @classmethod
    def reject_duplicate_states(
        cls, value: Optional[List[UavRuntimeState]]
    ) -> Optional[List[UavRuntimeState]]:
        if value is None:
            return value
        state_ids = [state.uav_id for state in value]
        if len(state_ids) != len(set(state_ids)):
            raise ValueError("initial_states contains duplicate uav_id values")
        return value


def load_json_model(path: PathLike, model_type: Type[ModelT]) -> ModelT:
    """Load one strict Pydantic model from JSON with a path-aware error."""

    config_path = Path(path)
    try:
        raw_text = config_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigurationLoadError(f"cannot read configuration {config_path}: {exc}") from exc

    try:
        raw_data = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise ConfigurationLoadError(
            f"invalid JSON in {config_path} at line {exc.lineno}, column {exc.colno}: {exc.msg}"
        ) from exc

    try:
        return model_type.model_validate(raw_data)
    except ValidationError as exc:
        raise ConfigurationLoadError(f"invalid configuration {config_path}: {exc}") from exc


def load_mission_context(
    robots_path: PathLike,
    mission_path: PathLike,
    plans_path: PathLike,
) -> MissionContext:
    """Load the three phase-2 files and construct one Group's MissionContext."""

    uav_catalog = load_json_model(robots_path, UavCatalog)
    bootstrap = load_json_model(mission_path, MissionBootstrapConfig)
    plan_catalog = load_json_model(plans_path, PlanCatalog)

    return build_mission_context(uav_catalog, bootstrap, plan_catalog)


def build_mission_context(
    uav_catalog: UavCatalog,
    bootstrap: MissionBootstrapConfig,
    plan_catalog: PlanCatalog,
) -> MissionContext:
    """Construct one validated Group context from already loaded catalogs."""

    group_specs = {
        spec.uav_id: spec for spec in uav_catalog.uavs if spec.group_id == bootstrap.group_id
    }
    if not group_specs:
        raise ConfigurationLoadError(
            f"UAV catalog contains no members for Group {bootstrap.group_id!r}"
        )

    if bootstrap.initial_states is None:
        group_states = {
            uav_id: UavRuntimeState(uav_id=uav_id) for uav_id in group_specs
        }
    else:
        group_states = {state.uav_id: state for state in bootstrap.initial_states}

    group_plans = {
        plan.plan_id: plan for plan in plan_catalog.plans if plan.group_id == bootstrap.group_id
    }

    if bootstrap.planned_recovery_count > bootstrap.planned_fault_count:
        raise ConfigurationLoadError(
            "planned_recovery_count cannot exceed planned_fault_count; "
            f"planned_recovery_count={bootstrap.planned_recovery_count}, "
            f"planned_fault_count={bootstrap.planned_fault_count}"
        )
    if len(bootstrap.roster.reserve_ids) < bootstrap.planned_recovery_count:
        raise ConfigurationLoadError(
            "standby Reserve count must cover planned_recovery_count; "
            f"reserves={len(bootstrap.roster.reserve_ids)}, "
            f"planned_recovery_count={bootstrap.planned_recovery_count}"
        )

    try:
        return MissionContext(
            mission_id=bootstrap.mission_id,
            group_id=bootstrap.group_id,
            uav_specs=group_specs,
            uav_states=group_states,
            roster=bootstrap.roster,
            plans=group_plans,
            blackboard=bootstrap.blackboard,
            current_phase=bootstrap.current_phase,
            planned_fault_count=bootstrap.planned_fault_count,
            planned_recovery_count=bootstrap.planned_recovery_count,
        )
    except ValidationError as exc:
        raise ConfigurationLoadError(
            f"inconsistent mission configuration for Group {bootstrap.group_id!r}: {exc}"
        ) from exc
