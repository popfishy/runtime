"""Joint behavior-tree simulation with a dependency-free Tk visualizer."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple, Union

from .bt import ActionNode, FallbackNode, NodeStatus, SequenceNode
from .bt.nodes import BaseNode
from .clock import ManualClock
from .executor import MissionExecutor, RuntimeServices
from .mission_log import MissionLogger, make_logger
from .package import MissionPackage, validate_joint_packages
from .simulation import build_scripted_backend
from .spatial import (
    CoordinateMapper,
    FieldConfig,
    KinematicCommandTransport,
    KinematicUav,
    Point3D,
)
from .transport import InMemoryCoordinationBus, InMemoryCoordinationTransport


SUPPORTED_VISUAL_SCENARIOS = {
    "normal",
    "strike-failure",
    "recovery-failure",
    "partial-damage-failure",
    "coordination-loss",
    "return-failure",
}


ACTION_LABELS = {
    "PrepareGroupA": "准备并建立侦查编队",
    "PublishGroupReady": "发布本组准备完成",
    "WaitGroupAReady": "等待 Group A 准备完成",
    "WaitGroupBReady": "等待 Group B 准备完成",
    "CoverageSegment1": "执行第一段区域覆盖",
    "SimulateDamage": "故障注入与受控退出",
    "RecoverReconGroup": "Reserve 接替并恢复编队",
    "CoverageSegment2": "执行第二段区域覆盖",
    "PublishReconComplete": "发布侦查完成",
    "WaitReconComplete": "等待侦查完成",
    "StrikeTargets": "两架无人机分别执行目标任务",
    "PublishStrikeComplete": "发布打击完成",
    "WaitStrikeComplete": "等待打击完成",
    "HoldCoverageEnd": "上边界终点编队悬停",
    "ReturnStrikeUavs": "两架打击无人机返回初始悬停点",
}


MISSION_FLOW = (
    (
        "准备与双站就绪",
        {
            "PrepareGroupA",
            "PublishGroupReady",
            "WaitGroupAReady",
            "WaitGroupBReady",
        },
    ),
    ("A组下半区覆盖", {"CoverageSegment1"}),
    ("6架故障退出", {"SimulateDamage"}),
    ("3架Reserve补位", {"RecoverReconGroup"}),
    (
        "A组上半区覆盖",
        {"CoverageSegment2", "PublishReconComplete", "WaitReconComplete"},
    ),
    ("B组双目标打击", {"StrikeTargets"}),
    (
        "B组返航 / A组终点悬停",
        {
            "ReturnStrikeUavs",
            "PublishStrikeComplete",
            "WaitStrikeComplete",
            "HoldCoverageEnd",
        },
    ),
)


@dataclass(frozen=True)
class VisualSimulationResult:
    group_a_status: NodeStatus
    group_b_status: NodeStatus
    ticks: int
    simulated_seconds: float
    group_a_commands: int
    group_b_commands: int
    attacks_completed: Tuple[str, ...]

    @property
    def success(self) -> bool:
        return (
            self.group_a_status == NodeStatus.SUCCESS
            and self.group_b_status == NodeStatus.SUCCESS
        )


@dataclass(frozen=True)
class ViewerLayout:
    """Pixel geometry derived from the current resizable canvas size."""

    canvas_width: int
    canvas_height: int
    plot_left: float
    plot_top: float
    plot_width: float
    plot_height: float
    panel_left: float
    panel_width: float
    panel_bottom: float


class JointVisualSimulation:
    """Advance both real BT executors and two visible kinematic backends."""

    terminal_statuses = {
        NodeStatus.SUCCESS,
        NodeStatus.FAILURE,
        NodeStatus.CANCELLED,
    }

    def __init__(
        self,
        group_a: MissionPackage,
        group_b: MissionPackage,
        scenario: str = "normal",
        field: Optional[FieldConfig] = None,
        tick_seconds: float = 0.2,
        max_ticks: int = 2000,
        uav_speed_mps: float = 12.0,
        log_directory: Optional[Union[str, Path]] = None,
    ) -> None:
        if scenario not in SUPPORTED_VISUAL_SCENARIOS:
            raise ValueError(f"unsupported visual scenario {scenario!r}")
        if tick_seconds <= 0 or max_ticks <= 0:
            raise ValueError("tick_seconds and max_ticks must be positive")
        validate_joint_packages(group_a, group_b)
        self.scenario = scenario
        self.tick_seconds = tick_seconds
        self.max_ticks = max_ticks
        self.clock = ManualClock()
        area = group_a.world.flight_area
        self.field = field or FieldConfig(
            width_m=area.max_x - area.min_x,
            height_m=area.max_y - area.min_y,
            margin_m=area.safety_margin_m,
        )
        self.mapper = CoordinateMapper.from_packages(
            [group_a, group_b], self.field
        )
        dropped = ["RECON_COMPLETE"] if scenario == "coordination-loss" else []
        self.bus = InMemoryCoordinationBus(drop_event_types=dropped)
        self.packages: Dict[str, MissionPackage] = {
            group_a.context.group_id: group_a,
            group_b.context.group_id: group_b,
        }
        self.transports: Dict[str, KinematicCommandTransport] = {}
        self.executors: Dict[str, MissionExecutor] = {}
        self.loggers: Dict[str, MissionLogger] = {}
        for group_id, package in self.packages.items():
            transport = KinematicCommandTransport(
                package=package,
                mapper=self.mapper,
                clock=self.clock,
                backend=build_scripted_backend(group_id, scenario),
                horizontal_speed_mps=uav_speed_mps,
            )
            logger = _logger_for(group_id, self.clock, log_directory)
            services = RuntimeServices(
                mission=package.context,
                command_transport=transport,
                coordination_transport=InMemoryCoordinationTransport(group_id, self.bus),
                clock=self.clock,
                logger=logger,
            )
            self.transports[group_id] = transport
            self.loggers[group_id] = logger
            self.executors[group_id] = MissionExecutor(
                package.parsed_tree.tree, services
            )
        self.ticks = 0
        self.phase_labels = {"GroupA": "等待启动", "GroupB": "等待启动"}
        self.phase_history: List[str] = []
        self._closed = False

    @property
    def finished(self) -> bool:
        return all(
            executor.status in self.terminal_statuses
            for executor in self.executors.values()
        )

    @property
    def all_uavs(self) -> Dict[str, KinematicUav]:
        result: Dict[str, KinematicUav] = {}
        for transport in self.transports.values():
            result.update(transport.uavs)
        return result

    def step(self) -> None:
        if self.finished:
            return
        for group_id in ("GroupA", "GroupB"):
            executor = self.executors[group_id]
            if executor.status not in self.terminal_statuses:
                executor.tick()
            self._update_phase(group_id)
        for transport in self.transports.values():
            transport.advance(self.tick_seconds)
        self.ticks += 1
        self.clock.advance(self.tick_seconds)
        if self.ticks >= self.max_ticks and not self.finished:
            for executor in self.executors.values():
                if executor.status not in self.terminal_statuses:
                    executor.abort("VISUAL_SIMULATION_MAX_TICKS")
            for group_id in ("GroupA", "GroupB"):
                self._update_phase(group_id)

    def _update_phase(self, group_id: str) -> None:
        executor = self.executors[group_id]
        action_id = _running_action(executor.tree.root)
        if action_id is not None:
            new_label = ACTION_LABELS.get(action_id, action_id)
        elif executor.status == NodeStatus.SUCCESS:
            new_label = "任务完成"
        elif executor.status in {NodeStatus.FAILURE, NodeStatus.CANCELLED}:
            new_label = "任务中止 / 安全悬停"
        else:
            new_label = "行为树推进中"
        if new_label != self.phase_labels[group_id]:
            self.phase_labels[group_id] = new_label
            self.phase_history.append(
                f"{self.clock.monotonic():6.1f}s  {group_id}: {new_label}"
            )
            if len(self.phase_history) > 10:
                del self.phase_history[: len(self.phase_history) - 10]

    def run_to_completion(self) -> VisualSimulationResult:
        while not self.finished:
            self.step()
        return self.result()

    def cancel(self, reason: str = "VISUALIZER_CLOSED") -> None:
        for executor in self.executors.values():
            if executor.status not in self.terminal_statuses:
                executor.cancel(reason)
        for group_id in ("GroupA", "GroupB"):
            self._update_phase(group_id)

    def result(self) -> VisualSimulationResult:
        attacks: List[str] = []
        for transport in self.transports.values():
            attacks.extend(transport.attack_successes)
        return VisualSimulationResult(
            group_a_status=self.executors["GroupA"].status,
            group_b_status=self.executors["GroupB"].status,
            ticks=self.ticks,
            simulated_seconds=self.clock.monotonic(),
            group_a_commands=len(self.transports["GroupA"].history),
            group_b_commands=len(self.transports["GroupB"].history),
            attacks_completed=tuple(attacks),
        )

    def close(self) -> None:
        if self._closed:
            return
        for executor in self.executors.values():
            executor.close()
        self._closed = True


class TkMissionViewer:
    """Recording-friendly Tk dashboard for the configured joint mission."""

    initial_window_width = 1900
    initial_window_height = 1080
    minimum_window_width = 1360
    minimum_window_height = 960
    canvas_width = 1900
    canvas_height = 1010
    plot_left = 70.0
    plot_top = 82.0
    plot_height = 870.0
    plot_width = plot_height * 100.0 / 150.0
    panel_left = plot_left + plot_width + 60.0
    panel_width = canvas_width - panel_left - 30.0
    panel_bottom = canvas_height - 20.0

    @classmethod
    def layout_for_size(cls, width: int, height: int) -> ViewerLayout:
        """Keep the physical map aspect ratio and expand the status panel."""

        width = max(1000, int(width))
        height = max(700, int(height))
        plot_left = 70.0
        plot_top = 82.0
        plot_height = max(560.0, float(height) - plot_top - 58.0)
        plot_width = plot_height * 100.0 / 150.0
        panel_left = plot_left + plot_width + 60.0
        panel_width = float(width) - panel_left - 30.0
        if panel_width < 700.0:
            panel_width = 700.0
            panel_left = float(width) - panel_width - 30.0
            plot_width = max(360.0, panel_left - plot_left - 60.0)
            plot_height = plot_width * 150.0 / 100.0
        return ViewerLayout(
            canvas_width=width,
            canvas_height=height,
            plot_left=plot_left,
            plot_top=plot_top,
            plot_width=plot_width,
            plot_height=plot_height,
            panel_left=panel_left,
            panel_width=panel_width,
            panel_bottom=float(height) - 20.0,
        )

    def __init__(
        self, simulation: JointVisualSimulation, playback_speed: float = 4.0
    ) -> None:
        if playback_speed <= 0:
            raise ValueError("playback speed must be positive")
        import tkinter as tk
        from tkinter import font as tkfont
        from tkinter import ttk

        self.tk = tk
        self.simulation = simulation
        self.root = tk.Tk()
        # Canvas coordinates and fonts should stay predictable on HiDPI Linux
        # desktops so a 1720x1000 recording has the same layout everywhere.
        self.root.tk.call("tk", "scaling", 1.0)
        self.root.title("四旋翼容错自愈联合任务行为树仿真")
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        initial_width = min(self.initial_window_width, max(1000, screen_width - 40))
        initial_height = min(self.initial_window_height, max(720, screen_height - 80))
        self.root.geometry(f"{initial_width}x{initial_height}")
        self.root.minsize(
            min(self.minimum_window_width, initial_width),
            min(self.minimum_window_height, initial_height),
        )
        self.root.resizable(True, True)
        installed_families = set(tkfont.families(self.root))
        self.font_family = next(
            (
                family
                for family in (
                    "Noto Sans CJK SC",
                    "WenQuanYi Zen Hei",
                    "Droid Sans Fallback",
                    "Sans",
                )
                if family in installed_families
            ),
            "Sans",
        )
        for named_font in (
            "TkDefaultFont",
            "TkTextFont",
            "TkMenuFont",
            "TkHeadingFont",
            "TkCaptionFont",
            "TkSmallCaptionFont",
            "TkIconFont",
        ):
            try:
                tkfont.nametofont(named_font).configure(
                    family=self.font_family, size=11
                )
            except tk.TclError:
                continue
        style = ttk.Style(self.root)
        style.configure("Visual.TButton", font=self._font(12), padding=(12, 7))
        style.configure("Visual.TLabel", font=self._font(11))
        style.configure("Visual.TCombobox", font=self._font(11), padding=5)
        self.root.option_add("*TCombobox*Listbox.font", self._font(11))
        self.canvas = tk.Canvas(
            self.root,
            background="#f3f4f6",
            highlightthickness=0,
        )
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        controls = ttk.Frame(self.root)
        controls.grid(row=1, column=0, sticky="ew", padx=18, pady=(8, 10))
        self.paused = False
        self.pause_button = ttk.Button(
            controls,
            text="暂停",
            width=10,
            command=self._toggle_pause,
            style="Visual.TButton",
        )
        self.pause_button.pack(side="left")
        ttk.Button(
            controls,
            text="单步",
            width=10,
            command=self._single_step,
            style="Visual.TButton",
        ).pack(side="left", padx=(10, 0))
        ttk.Label(
            controls, text="播放速度：", style="Visual.TLabel"
        ).pack(side="left", padx=(24, 6))
        self.speed_var = tk.StringVar(value=f"{playback_speed:g}x")
        ttk.Combobox(
            controls,
            textvariable=self.speed_var,
            values=("1x", "2x", "4x", "8x", "16x"),
            width=6,
            state="readonly",
            style="Visual.TCombobox",
        ).pack(side="left")
        ttk.Label(
            controls,
            text="任务层仿真 · 飞控、避障与防碰撞由底层实现",
            foreground="#555555",
            style="Visual.TLabel",
        ).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._resize_job = None
        self._pending_canvas_size = None
        self._apply_layout(self.layout_for_size(self.canvas_width, self.canvas_height))
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self._draw_static_scene()
        self._render()
        try:
            self.root.attributes("-zoomed", True)
        except tk.TclError:
            pass

    def _font(self, size: int, weight: str = "normal") -> Tuple[str, int, str]:
        return self.font_family, size, weight

    def _apply_layout(self, layout: ViewerLayout) -> None:
        self.canvas_width = layout.canvas_width
        self.canvas_height = layout.canvas_height
        self.plot_left = layout.plot_left
        self.plot_top = layout.plot_top
        self.plot_width = layout.plot_width
        self.plot_height = layout.plot_height
        self.panel_left = layout.panel_left
        self.panel_width = layout.panel_width
        self.panel_bottom = layout.panel_bottom

    def _on_canvas_resize(self, event) -> None:
        if event.width < 100 or event.height < 100:
            return
        self._pending_canvas_size = (event.width, event.height)
        if self._resize_job is not None:
            self.root.after_cancel(self._resize_job)
        self._resize_job = self.root.after(60, self._redraw_after_resize)

    def _redraw_after_resize(self) -> None:
        self._resize_job = None
        if self._pending_canvas_size is None:
            return
        width, height = self._pending_canvas_size
        self._pending_canvas_size = None
        if abs(width - self.canvas_width) < 2 and abs(height - self.canvas_height) < 2:
            return
        self._apply_layout(self.layout_for_size(width, height))
        self.canvas.delete("all")
        self._draw_static_scene()
        self._render()

    def run(self) -> VisualSimulationResult:
        self.root.after(80, self._loop)
        self.root.mainloop()
        return self.simulation.result()

    def _loop(self) -> None:
        if not self.paused and not self.simulation.finished:
            self.simulation.step()
        self._render()
        if self.root.winfo_exists():
            self.root.after(self._frame_delay_ms(), self._loop)

    def _frame_delay_ms(self) -> int:
        value = self.speed_var.get().rstrip("x")
        try:
            speed = float(value)
        except ValueError:
            speed = 4.0
        return max(12, int(self.simulation.tick_seconds * 1000 / speed))

    def _toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_button.configure(text="继续" if self.paused else "暂停")

    def _single_step(self) -> None:
        self.paused = True
        self.pause_button.configure(text="继续")
        self.simulation.step()
        self._render()

    def _close(self) -> None:
        if not self.simulation.finished:
            self.simulation.cancel()
        self.simulation.close()
        self.root.destroy()

    def _draw_static_scene(self) -> None:
        canvas = self.canvas
        left, top = self.plot_left, self.plot_top
        right = left + self.plot_width
        bottom = top + self.plot_height
        canvas.create_text(
            left,
            12,
            anchor="nw",
            text=(
                f"{self.simulation.field.width_m:g} m × "
                f"{self.simulation.field.height_m:g} m 实验场地俯视图"
            ),
            font=self._font(20, "bold"),
            fill="#1f2937",
        )
        canvas.create_text(
            right,
            top - 10,
            anchor="se",
            text="虚线框：5 m安全边界  ·  单位：m",
            font=self._font(10),
            fill="#6b7280",
        )
        canvas.create_rectangle(
            left, top, right, bottom, fill="#ffffff", outline="#374151", width=2
        )
        for x_m in range(0, int(self.simulation.field.width_m) + 1, 10):
            x, _ = self._pixel(float(x_m), 0.0)
            canvas.create_line(x, top, x, bottom, fill="#e5e7eb", tags="static")
            if x_m % 20 == 0:
                canvas.create_text(
                    x,
                    bottom + 19,
                    text=str(x_m),
                    fill="#6b7280",
                    font=self._font(9),
                )
        for y_m in range(0, int(self.simulation.field.height_m) + 1, 10):
            _, y = self._pixel(0.0, float(y_m))
            canvas.create_line(left, y, right, y, fill="#e5e7eb", tags="static")
            if y_m % 20 == 0:
                canvas.create_text(
                    left - 23,
                    y,
                    text=str(y_m),
                    fill="#6b7280",
                    font=self._font(9),
                )
        canvas.create_text(
            right,
            bottom + 39,
            text="X / m",
            anchor="e",
            fill="#4b5563",
            font=self._font(10),
        )
        canvas.create_text(
            left - 48,
            top,
            text="Y / m",
            anchor="w",
            fill="#4b5563",
            font=self._font(10),
        )
        self._draw_fixed_areas()
        self._draw_zones()

    def _draw_fixed_areas(self) -> None:
        field = self.simulation.field
        margin = field.margin_m
        effective_top_left = self._pixel(margin, field.height_m - margin)
        effective_bottom_right = self._pixel(field.width_m - margin, margin)
        self.canvas.create_rectangle(
            *effective_top_left,
            *effective_bottom_right,
            outline="#6b7280",
            width=2,
            dash=(8, 5),
        )
        for bounds, label, fill, outline in (
            ((5.0, 5.0, 35.0, 18.0), "A初始/退出区", "#dbeafe", "#2563eb"),
            (
                (78.0, 5.0, 95.0, 25.0),
                "B初始区",
                "#ffedd5",
                "#ea580c",
            ),
        ):
            min_x, min_y, max_x, max_y = bounds
            x1, y1 = self._pixel(min_x, max_y)
            x2, y2 = self._pixel(max_x, min_y)
            self.canvas.create_rectangle(
                x1,
                y1,
                x2,
                y2,
                fill=fill,
                outline=outline,
                width=1,
                stipple="gray25",
            )
            self.canvas.create_text(
                (x1 + x2) / 2,
                y1 - 4,
                anchor="s",
                text=label,
                fill=outline,
                font=self._font(9, "bold"),
            )

    def _draw_zones(self) -> None:
        zone_specs = [
            ("GroupA", "prepare-a", "A编队集结区", "#dbeafe", "#2563eb", False),
            (
                "GroupA",
                "hold-a-end",
                "A终点悬停区",
                "#e0f2fe",
                "#0284c7",
                True,
            ),
        ]
        for group_id, plan_id, label, fill, outline, label_at_bottom in zone_specs:
            points = _plan_points(
                self.simulation.packages[group_id], plan_id, self.simulation.mapper
            )
            if points:
                self._zone_rectangle(
                    points, label, fill, outline, label_at_bottom=label_at_bottom
                )
        coverage_points = []
        for plan_id in ("coverage-segment-1", "coverage-segment-2"):
            coverage_points.extend(
                _plan_points(
                    self.simulation.packages["GroupA"],
                    plan_id,
                    self.simulation.mapper,
                )
            )
        if coverage_points:
            self._zone_rectangle(
                coverage_points, "覆盖侦查区", "#dcfce7", "#16a34a", 4.0
            )
        for target_id, target in self.simulation.packages["GroupA"].world.targets.items():
            if not isinstance(target, Mapping):
                continue
            point = self.simulation.mapper.map_pose(target)
            x, y = self._pixel(point.x, point.y)
            self.canvas.create_polygon(
                x,
                y - 12,
                x + 12,
                y,
                x,
                y + 12,
                x - 12,
                y,
                fill="#fee2e2",
                outline="#dc2626",
                width=2,
            )
            self.canvas.create_text(
                x + 14,
                y - 14,
                anchor="w",
                text=target_id,
                fill="#991b1b",
                font=self._font(10, "bold"),
            )

    def _zone_rectangle(
        self,
        points: Sequence[Point3D],
        label: str,
        fill: str,
        outline: str,
        padding_m: float = 3.0,
        label_at_bottom: bool = False,
    ) -> None:
        min_x = max(0.0, min(point.x for point in points) - padding_m)
        max_x = min(
            self.simulation.field.width_m,
            max(point.x for point in points) + padding_m,
        )
        min_y = max(0.0, min(point.y for point in points) - padding_m)
        max_y = min(
            self.simulation.field.height_m,
            max(point.y for point in points) + padding_m,
        )
        x1, y1 = self._pixel(min_x, max_y)
        x2, y2 = self._pixel(max_x, min_y)
        self.canvas.create_rectangle(
            x1,
            y1,
            x2,
            y2,
            fill=fill,
            outline=outline,
            width=1,
            stipple="gray25",
        )
        self.canvas.create_text(
            x1 + 5,
            y2 - 5 if label_at_bottom else y1 + 5,
            anchor="sw" if label_at_bottom else "nw",
            text=label,
            fill=outline,
            font=self._font(9, "bold"),
        )

    def _render(self) -> None:
        actual_width = self.canvas.winfo_width()
        actual_height = self.canvas.winfo_height()
        if (
            actual_width > 100
            and actual_height > 100
            and (
                abs(actual_width - self.canvas_width) >= 2
                or abs(actual_height - self.canvas_height) >= 2
            )
        ):
            self._apply_layout(self.layout_for_size(actual_width, actual_height))
            self.canvas.delete("all")
            self._draw_static_scene()
        self.canvas.delete("dynamic")
        self._draw_trails()
        self._draw_uavs()
        self._draw_side_panel()

    def _draw_trails(self) -> None:
        for uav in self.simulation.all_uavs.values():
            if len(uav.trail) < 2:
                continue
            # Showing all 31 trails makes the recording unreadable. Keep the
            # Leader route prominent and retain only exited-aircraft trails as
            # evidence of the fault-handling branch. Followers remain visible
            # as current positions and are controlled by the lower layer.
            if (
                not uav.is_leader
                and uav.uav_id not in {"B01", "B02"}
                and uav.motion_state not in {
                    "FAULT_EXIT",
                    "FAULT_EXIT_HOLD",
                }
            ):
                continue
            coordinates: List[float] = []
            for x_m, y_m in uav.trail:
                x, y = self._pixel(x_m, y_m)
                coordinates.extend((x, y))
            color = "#2563eb" if uav.group_id == "GroupA" else "#ea580c"
            self.canvas.create_line(
                *coordinates,
                fill=color,
                width=3 if uav.is_leader else 1,
                smooth=False,
                tags="dynamic",
            )

    def _draw_uavs(self) -> None:
        occupied_label_boxes: List[Tuple[float, float, float, float]] = []
        for uav_id in sorted(self.simulation.all_uavs):
            uav = self.simulation.all_uavs[uav_id]
            context = self.simulation.packages[uav.group_id].context
            failed = uav_id in context.roster.failed_ids
            standby = uav_id in context.roster.reserve_ids
            inactive = uav_id in context.roster.inactive_ids
            exiting = uav.motion_state == "FAULT_EXIT"
            x, y = self._pixel(uav.pose.x, uav.pose.y)
            group_color = "#2563eb" if uav.group_id == "GroupA" else "#ea580c"
            if inactive:
                self.canvas.create_oval(
                    x - 7,
                    y - 7,
                    x + 7,
                    y + 7,
                    fill="#e5e7eb",
                    outline="#6b7280",
                    width=1,
                    tags="dynamic",
                )
            elif failed:
                self.canvas.create_line(
                    x - 8, y - 8, x + 8, y + 8, fill="#dc2626", width=4, tags="dynamic"
                )
                self.canvas.create_line(
                    x - 8, y + 8, x + 8, y - 8, fill="#dc2626", width=4, tags="dynamic"
                )
            else:
                fill = "#fecaca" if exiting else ("#ffffff" if standby else group_color)
                outline = "#dc2626" if exiting else group_color
                self.canvas.create_oval(
                    x - 7,
                    y - 7,
                    x + 7,
                    y + 7,
                    fill=fill,
                    outline=outline,
                    width=2 if standby or uav.is_leader else 1,
                    dash=(3, 2) if standby else (),
                    tags="dynamic",
                )
                if uav.is_leader:
                    self.canvas.create_oval(
                        x - 11,
                        y - 11,
                        x + 11,
                        y + 11,
                        outline=group_color,
                        width=1,
                        tags="dynamic",
                    )
            if inactive:
                continue
            # Keep the font size fixed, but choose a nearby unoccupied label
            # position. Dense 3x3/3x4 formations otherwise produce unreadable
            # stacks of UAV IDs even when the window itself is enlarged.
            label = f"{uav_id} {uav.pose.z:.0f}m" if exiting else uav_id
            label_x, label_y, label_box = self._place_uav_label(
                x, y, label, occupied_label_boxes, _numeric_suffix(uav_id)
            )
            occupied_label_boxes.append(label_box)
            if abs(label_x - x) > 18 or abs(label_y - y) > 18:
                self.canvas.create_line(
                    x,
                    y,
                    label_x,
                    label_y,
                    fill="#9ca3af",
                    width=1,
                    tags="dynamic",
                )
            self.canvas.create_text(
                label_x,
                label_y,
                text=label,
                anchor="center",
                fill="#111827",
                font=self._font(8, "bold"),
                tags="dynamic",
            )
        # Make the ten grounded/inactive B nodes explicit without drawing a
        # dense stack of labels in the small right-hand launch zone.
        inactive_b = [
            uav_id
            for uav_id, uav in self.simulation.all_uavs.items()
            if uav.group_id == "GroupB"
            and uav_id in self.simulation.packages["GroupB"].context.roster.inactive_ids
        ]
        if inactive_b:
            x, y = self._pixel(87.0, 23.0)
            self.canvas.create_text(
                x,
                y,
                anchor="sw",
                text=f"B07-B16  地面待命（{len(inactive_b)}架）",
                fill="#4b5563",
                font=self._font(8, "bold"),
                tags="dynamic",
            )

    def _place_uav_label(
        self,
        marker_x: float,
        marker_y: float,
        label: str,
        occupied: Sequence[Tuple[float, float, float, float]],
        variant: int,
    ) -> Tuple[float, float, Tuple[float, float, float, float]]:
        candidates = [
            (18.0, -15.0),
            (-18.0, 15.0),
            (18.0, 15.0),
            (-18.0, -15.0),
            (35.0, -23.0),
            (-35.0, -23.0),
            (35.0, 23.0),
            (-35.0, 23.0),
            (0.0, -34.0),
            (0.0, 34.0),
            (52.0, 0.0),
            (-52.0, 0.0),
        ]
        offset = variant % 4
        candidates = candidates[offset:] + candidates[:offset]
        label_width = max(28.0, len(label) * 7.0)
        label_height = 16.0
        plot_right = self.plot_left + self.plot_width
        plot_bottom = self.plot_top + self.plot_height
        fallback = None
        for dx, dy in candidates:
            center_x = marker_x + dx
            center_y = marker_y + dy
            box = (
                center_x - label_width / 2,
                center_y - label_height / 2,
                center_x + label_width / 2,
                center_y + label_height / 2,
            )
            fallback = (center_x, center_y, box)
            if not (
                self.plot_left + 2 <= box[0]
                and box[2] <= plot_right - 2
                and self.plot_top + 2 <= box[1]
                and box[3] <= plot_bottom - 2
            ):
                continue
            if any(_rectangles_overlap(box, other, padding=3.0) for other in occupied):
                continue
            return center_x, center_y, box
        assert fallback is not None
        return fallback

    def _draw_side_panel(self) -> None:
        x = self.panel_left
        result = self.simulation.result()
        inner_width = self.panel_width - 50.0
        column_gap = 44.0
        left_width = max(340.0, inner_width * 0.51)
        right_x = x + left_width + column_gap
        right_width = max(300.0, inner_width - left_width - column_gap)
        self.canvas.create_rectangle(
            x - 20,
            45,
            x + self.panel_width,
            self.panel_bottom,
            fill="#ffffff",
            outline="#d1d5db",
            width=1,
            tags="dynamic",
        )
        self.canvas.create_text(
            x,
            58,
            anchor="nw",
            text="行为树运行状态",
            font=self._font(18, "bold"),
            fill="#1f2937",
            tags="dynamic",
        )
        self.canvas.create_text(
            x,
            91,
            anchor="nw",
            text="GROUND-STATION VIEW",
            font=self._font(10),
            fill="#6b7280",
            tags="dynamic",
        )
        lines = [
            f"场景：{self.simulation.scenario}    仿真时间：{result.simulated_seconds:6.1f} s",
            (
                f"Tick：{result.ticks} / {self.simulation.max_ticks}    "
                f"坐标缩放：1 : {1 / self.simulation.mapper.scale:.2f}"
            ),
        ]
        self.canvas.create_text(
            x,
            122,
            anchor="nw",
            text="\n\n".join(lines),
            font=self._font(11),
            fill="#374151",
            tags="dynamic",
        )
        self._draw_mission_flow(right_x, 145, right_width)
        y = 190
        for group_id, color in (("GroupA", "#2563eb"), ("GroupB", "#ea580c")):
            context = self.simulation.packages[group_id].context
            executor = self.simulation.executors[group_id]
            self.canvas.create_rectangle(
                x,
                y,
                x + left_width,
                y + 112,
                fill="#ffffff",
                outline=color,
                width=2,
                tags="dynamic",
            )
            card_lines = (
                (f"{group_id}   {executor.status.value}", 16, "bold"),
                (f"阶段：{self.simulation.phase_labels[group_id]}", 50, "normal"),
                (
                    f"成员  Active {len(context.roster.active_ids)}  ·  "
                    f"Reserve {len(context.roster.reserve_ids)}  ·  "
                    f"Failed {len(context.roster.failed_ids)}  ·  "
                    f"Inactive {len(context.roster.inactive_ids)}",
                    84,
                    "normal",
                ),
            )
            for line, offset_y, weight in card_lines:
                self.canvas.create_text(
                    x + 16,
                    y + offset_y,
                    anchor="nw",
                    text=line,
                    width=left_width - 32,
                    font=self._font(11, weight),
                    fill="#1f2937",
                    tags="dynamic",
                )
            y += 134

        self.canvas.create_text(
            x,
            480,
            anchor="nw",
            text="阶段事件时间线",
            font=self._font(13, "bold"),
            fill="#1f2937",
            tags="dynamic",
        )
        events = self.simulation.phase_history[-5:] or ["等待任务启动"]
        for index, event_text in enumerate(events):
            self.canvas.create_text(
                x,
                517 + index * 34,
                anchor="nw",
                text=event_text,
                width=left_width,
                font=self._font(9),
                fill="#4b5563",
                tags="dynamic",
            )
        if result.attacks_completed:
            self.canvas.create_text(
                x,
                692,
                anchor="nw",
                text=f"打击完成：{', '.join(result.attacks_completed)}",
                font=self._font(11, "bold"),
                fill="#b91c1c",
                tags="dynamic",
            )
        self.canvas.create_text(
            x,
            734,
            anchor="nw",
            text="图例：  ● A组侦查   ● B组打击/返航   ○ Reserve   ◎ Leader",
            font=self._font(10),
            fill="#374151",
            tags="dynamic",
        )
        self.canvas.create_text(
            x,
            766,
            anchor="nw",
            text="       × 故障退出   灰色节点=本任务不参与飞行",
            font=self._font(10),
            fill="#374151",
            tags="dynamic",
        )
        if self.simulation.finished:
            color = "#15803d" if result.success else "#b91c1c"
            message = "联合任务成功" if result.success else "联合任务失败 / 已安全中止"
            self.canvas.create_text(
                x + self.panel_width / 2,
                self.panel_bottom - 34,
                text=message,
                font=self._font(16, "bold"),
                fill=color,
                tags="dynamic",
            )

    def _draw_mission_flow(self, x: float, y: float, width: float) -> None:
        """Draw a compact, recording-friendly overview of the joint flow."""

        running_actions = {
            action_id
            for executor in self.simulation.executors.values()
            for action_id in (_running_action(executor.tree.root),)
            if action_id is not None
        }
        active_index = next(
            (
                index
                for index, (_, action_ids) in enumerate(MISSION_FLOW)
                if running_actions.intersection(action_ids)
            ),
            len(MISSION_FLOW) if self.simulation.finished else 0,
        )
        self.canvas.create_text(
            x,
            y - 42,
            anchor="nw",
            text="联合任务流程",
            font=self._font(13, "bold"),
            fill="#1f2937",
            tags="dynamic",
        )
        for index, (label, _) in enumerate(MISSION_FLOW):
            completed = index < active_index
            active = index == active_index
            fill = "#dcfce7" if completed else ("#dbeafe" if active else "#f9fafb")
            outline = "#16a34a" if completed else ("#2563eb" if active else "#d1d5db")
            symbol = "✓" if completed else ("▶" if active else str(index + 1))
            box_top = y + index * 75
            self.canvas.create_rectangle(
                x,
                box_top,
                x + width,
                box_top + 55,
                fill=fill,
                outline=outline,
                width=2 if active else 1,
                tags="dynamic",
            )
            self.canvas.create_text(
                x + 18,
                box_top + 27.5,
                text=symbol,
                font=self._font(11, "bold"),
                fill=outline,
                tags="dynamic",
            )
            self.canvas.create_text(
                x + 43,
                box_top + 27.5,
                anchor="w",
                text=label,
                width=max(220.0, width - 58.0),
                font=self._font(10, "bold" if active else "normal"),
                fill="#1f2937",
                tags="dynamic",
            )

    def _pixel(self, x_m: float, y_m: float) -> Tuple[float, float]:
        x = self.plot_left + x_m / self.simulation.field.width_m * self.plot_width
        y = (
            self.plot_top
            + self.plot_height
            - y_m / self.simulation.field.height_m * self.plot_height
        )
        return x, y


def _running_action(node: BaseNode) -> Optional[str]:
    if isinstance(node, ActionNode):
        return node.action_id if node.status == NodeStatus.RUNNING else None
    if isinstance(node, (SequenceNode, FallbackNode)):
        if node.current_index < len(node.children):
            child = node.children[node.current_index]
            action_id = _running_action(child)
            if action_id is not None:
                return action_id
    return None


def _numeric_suffix(identifier: str) -> int:
    digits = "".join(character for character in identifier if character.isdigit())
    return int(digits or "0")


def _rectangles_overlap(
    left: Tuple[float, float, float, float],
    right: Tuple[float, float, float, float],
    *,
    padding: float = 0.0,
) -> bool:
    return not (
        left[2] + padding <= right[0]
        or right[2] + padding <= left[0]
        or left[3] + padding <= right[1]
        or right[3] + padding <= left[1]
    )


def _logger_for(
    group_id: str,
    clock: ManualClock,
    log_directory: Optional[Union[str, Path]],
) -> MissionLogger:
    if log_directory is None:
        return make_logger(None, clock)
    return make_logger(Path(log_directory) / f"{group_id}.jsonl", clock)


def _plan_points(
    package: MissionPackage, plan_id: str, mapper: CoordinateMapper
) -> List[Point3D]:
    plan = package.context.plans.get(plan_id)
    if plan is None:
        return []
    result = []
    for assignment in plan.robot_assignments.values():
        target = assignment.payload.get("target_pose")
        if isinstance(target, Mapping):
            result.append(mapper.map_pose(target))
        hold_pose = assignment.payload.get("hold_pose")
        if isinstance(hold_pose, Mapping):
            result.append(mapper.map_pose(hold_pose))
        waypoints = assignment.payload.get("waypoints")
        if isinstance(waypoints, list):
            result.extend(
                mapper.map_pose(waypoint)
                for waypoint in waypoints
                if isinstance(waypoint, Mapping)
            )
    return result
