# UAV Behavior-Tree Runtime

This directory is an independent Python subproject for the high-level UAV
mission runtime. It does not import LLaMA-Factory, the legacy scenario UI, or
robot-control implementations.

The current implementation covers phases 1 through 7 of the approved plan:

- strict domain models for UAV specifications and runtime state;
- group roster and mission-context consistency checks;
- versioned command, feedback, coordination-event, and task-plan models;
- fail-closed JSON configuration loading.
- reviewed mission-package hashes and strict XML/static validation;
- a tick-based `Sequence`/`Fallback`/`Condition`/`Action` engine;
- non-blocking per-UAV command fan-out and result aggregation;
- two isolated in-memory Group executors and scripted failure injection;
- JSON Lines logs and command-line validation/simulation;
- a newline-delimited JSON/TCP transport that sends one complete Group A or
  Group B task batch to that Group's ground station.

The ground-station TCP Servers and aircraft-control adapters are owned by the
Group A and Group B lower-layer teams and are not implemented in this repository.

## Focused task tests

Run one reviewed task without executing the rest of its Group behavior tree:

```bash
python runtime/task_test.py --group A --task coverage-segment-1
python runtime/task_test.py --group B --task strike-targets
```

The default in-memory backend is suitable for fast checks. To exercise the TCP
wire interface, first start the corresponding ground-station Server and run:

```bash
python runtime/task_test.py --backend tcp --group A --task coverage-segment-1 \
  --tcp-host <group-a-ground-station-ip>
python runtime/task_test.py --backend tcp --group B --task strike-targets \
  --tcp-host <group-b-ground-station-ip>
```

The output is one JSON object containing the terminal status, robot IDs,
command types, individual Route count, target count, and final roster.

## Development

From the repository root, run:

```bash
PYTHONPATH=runtime/src python -m pytest runtime/tests -q
```

## Traditional script entry point

For a conventional `train.py`-style workflow, enter the runtime directory and
run `main.py` directly. No package installation or `PYTHONPATH` is required:

```bash
cd runtime
python main.py --mode validate --allow-unreviewed
python main.py --mode single --auto-peer-events --allow-unreviewed
python main.py --mode joint --scenario normal --allow-unreviewed
python main.py --mode visual --scenario normal --allow-unreviewed
```

## Group A/B TCP integration

Each physical runtime process connects to one Group ground station. The package
is required so the process cannot silently select the wrong Group:

```bash
./runtime/run_tcp.sh \
  --package runtime/examples/joint_mission/group_a \
  --tcp-host <group-a-ground-station-ip> \
  --tcp-port 39001 \
  --auto-peer-events \
  --allow-unreviewed

./runtime/run_tcp.sh \
  --package runtime/examples/joint_mission/group_b \
  --tcp-host <group-b-ground-station-ip> \
  --tcp-port 39001 \
  --auto-peer-events \
  --allow-unreviewed
```

`--auto-peer-events` is only for isolated lower-layer integration. The wire
messages, status rules, and ground-station responsibilities are documented in
`../docs/实物无人机上下层TCP-IP接口对接方案.md`.

The `visual` mode opens a standard-library Tk window and displays both Groups
inside the configured 100 m x 150 m field. It consumes the commands emitted by
the real behavior-tree executors; UAV movement completion generates the normal
command feedback that advances the trees. Use `--headless` to exercise the same
kinematic backend without opening a window:

```bash
python main.py --mode visual --scenario strike-failure --allow-unreviewed
python main.py --mode visual --scenario normal --headless --allow-unreviewed
```

The visualizer is a task-level top-down simulation, not a flight-dynamics,
collision-avoidance, or PX4 simulator.

Validate the included 31-node candidate example:

```bash
PYTHONPATH=runtime/src python -m uav_bt_runtime validate \
  runtime/examples/joint_mission/group_a \
  runtime/examples/joint_mission/group_b \
  --allow-unreviewed
```

Run the complete joint task:

```bash
PYTHONPATH=runtime/src python -m uav_bt_runtime simulate-joint \
  --group-a runtime/examples/joint_mission/group_a \
  --group-b runtime/examples/joint_mission/group_b \
  --allow-unreviewed
```

Exercise a two-aircraft strike failure:

```bash
PYTHONPATH=runtime/src python -m uav_bt_runtime simulate-joint \
  --group-a runtime/examples/joint_mission/group_a \
  --group-b runtime/examples/joint_mission/group_b \
  --scenario strike-failure \
  --allow-unreviewed
```

Configuration files reject unknown fields. `UavSpec` is immutable during a
mission, while `UavRuntimeState` is mutable and is correlated with it by the
same `uav_id`. Reserve priority is the order of `GroupRoster.reserve_ids`.
