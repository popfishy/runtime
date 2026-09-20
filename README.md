# 无人机行为树任务运行时

本目录是一个独立的 Python 子项目，负责无人机的高层任务运行时。它不依赖
LLaMA-Factory、旧的场景 UI，也不依赖各组的机器人控制实现。

当前实现覆盖已批准计划的第 1 至第 7 阶段：

- 无人机规格与运行时状态的严格领域模型；
- 编组花名册与任务上下文的一致性检查；
- 带版本号的指令、反馈、协同事件与任务计划模型；
- 失败即拒（fail-closed）的 JSON 配置加载；
- 任务包哈希校验与严格的 XML/静态校验；
- 基于 tick 的 `Sequence`/`Fallback`/`Condition`/`Action` 执行引擎；
- 面向多机的非阻塞指令下发与结果汇总；
- 两个相互隔离的内存内编组执行器，以及脚本化故障注入；
- JSON Lines 日志与命令行校验/仿真；
- 以换行分隔的 JSON/TCP 传输，把一整批 A 组或 B 组任务指令发往该组地面站。

地面站的 TCP Server 与机载控制适配层由 A 组、B 组下层团队负责，不在本仓库内实现。

## 单任务测试

只执行某一个已审核任务，不跑该组其余行为树：

```bash
python runtime/task_test.py --group A --task coverage-segment-1
python runtime/task_test.py --group B --task strike-targets
```

默认的 `memory` 后端适合快速自检。要验证 TCP 线接口，先启动对应组的地面站 Server：

```bash
python runtime/task_test.py --backend tcp --group A --task coverage-segment-1 \
  --tcp-host <group-a-ground-station-ip>
python runtime/task_test.py --backend tcp --group B --task strike-targets \
  --tcp-host <group-b-ground-station-ip>
```

输出是一个 JSON 对象，包含终止状态、参与无人机 ID、指令类型、各路径点数、
目标数，以及最终的编组花名册。

### 超时控制：`--timeout-s` 与 `--max-ticks`

这两个参数属于不同层，互不相同：

- **`--timeout-s`** 覆盖任务自身的超时。默认值取 `TASK_DEFINITIONS` 中该任务的
  配置（例如 GroupA 的 `prepare` 为 30 秒、`coverage-segment-1` 为 60 秒）。
  它有两个作用：一是决定行为树节点何时判定超时、取消当前批次并返回 `TIMEOUT`，
  二是写进下发给地面站的 `CommandEnvelope.timeout_s`。
- **`--max-ticks`** 只是本测试进程的循环上限。省略时按
  `ceil(timeout_s × tick_hz) + 1` 自动推算，以保证本地等待一定长于任务自身的超时
  ——否则本地会先退出，拿到一个假的 `TIMEOUT`，也观察不到任务自己取消批次的行为。
  tcp 模式下本地等待约 `max_ticks / tick_hz` 秒（默认 20 Hz）。

省略 `--max-ticks`：

```bash
python runtime/task_test.py --backend memory --group A --task prepare
```

覆盖任务超时（`max_ticks` 随之自动推算）：

```bash
python runtime/task_test.py --backend memory --group A --task prepare --timeout-s 12.5
```

显式指定本地循环上限（覆盖自动推算）：

```bash
python runtime/task_test.py --backend tcp --group A --task prepare-two-uav \
  --tcp-host <GCS_A_IP> --tcp-port 39001 --timeout-s 120 --max-ticks 2400
```

### A 组双机 MOVE_TO 冒烟测试

专用的 `prepare-two-uav` 任务走正常运行时流程，只向 A01、A02 下发一批 A 组
`MOVE_TO` 指令。先在无地面站的情况下校验：

```bash
python3 runtime/task_test.py --backend memory --group A \
  --task prepare-two-uav
```

标定完成、确认已起飞并进入 OFFBOARD 后，连接 `tcp_to_ros`：

```bash
python3 runtime/task_test.py --backend tcp --group A \
  --task prepare-two-uav \
  --tcp-host <GCS_A_IP> --tcp-port 39001 \
  --max-ticks 2400
```

该测试用的目标点存放在
`examples/joint_mission/group_a/plans/two_uav_smoke.json`，默认为 A 组任务坐标系下的
A01 `(0,2,5)`、A02 `(2,2,5)`。实飞前请按实际场地复核这些值。

## 修改 JSON 后刷新任务包哈希

`mission.json` 会用 `file_hashes` 记录它引用的每个文件的 SHA-256。改动
`tree.xml`、`robots.json`、`world.json`、`bootstrap.json` 或 `plans/` 下任何被
`plan_files` 列出的 JSON 之后，必须刷新哈希，否则 `load_mission_package` 会拒绝加载。

工具是 `tools/update_mission_hashes.py`：

```bash
python3 tools/update_mission_hashes.py examples/joint_mission/group_a
```

它会重算 `mission.json` 中 `file_hashes` 涉及的全部文件，写回 `file_hashes`，
并把 `reviewed` 置为 `false`。只有在人工逐条确认过航点之后，才显式标记为已审核：

```bash
python3 tools/update_mission_hashes.py \
  examples/joint_mission/group_a --mark-reviewed
```

可一次传入多个任务包目录：

```bash
python3 tools/update_mission_hashes.py \
  examples/joint_mission/group_a \
  examples/joint_mission/group_b
```

要点：

- 该工具**只刷新哈希**，不会修改任何被引用 JSON 的内容，也不会补齐漏登记的文件。
  新增计划文件时必须先把它写进 `mission.json` 的 `plan_files`，工具才会纳入哈希。
- 跨组联合任务还要求两组的 `world.json` **逐字节相同**，并满足
  `mission_id`、双方 `peer_package_version`、`target_config_version` 一致。
  只改其中一组而不改另一组，联合校验会在 `validate_joint_packages` 处报
  "Group packages do not share identical world configuration"。
- 另一条生成示例任务包的路径是 `tools/generate_example_mission.py`，它会直接写出
  带哈希的 `mission.json`。若手工改过示例包，注意别让两者产生分歧。

## 开发

在本仓库根目录（即本目录的上级）执行：

```bash
PYTHONPATH=runtime/src python -m pytest runtime/tests -q
```

## 传统脚本入口

如果习惯 `train.py` 那样的工作流，直接进入 runtime 目录运行 `main.py`，
无需安装包，也不需要设置 `PYTHONPATH`：

```bash
cd runtime
python main.py --mode validate --allow-unreviewed
python main.py --mode single --auto-peer-events --allow-unreviewed
python main.py --mode joint --scenario normal --allow-unreviewed
python main.py --mode visual --scenario normal --allow-unreviewed
```

## A/B 组 TCP 对接

每个实机运行时进程连接一个组的地面站。必须显式给出任务包，进程才无法悄悄选错组：

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

`--auto-peer-events` 仅用于与下层隔离联调。线协议消息格式、状态机规则以及地面站
职责见 `../docs/实物无人机上下层TCP-IP接口对接方案.md`。

`visual` 模式会打开一个标准库 Tk 窗口，在配置好的 100 m × 150 m 场地内同时显示
两组。它消费真实行为树执行器发出的指令；无人机移动完成会生成正常的指令反馈，
从而推进行为树。用 `--headless` 可以不弹窗、只跑同一套运动学后端：

```bash
python main.py --mode visual --scenario strike-failure --allow-unreviewed
python main.py --mode visual --scenario normal --headless --allow-unreviewed
```

可视化器是任务级的俯视仿真，不是飞行动力学、避障或 PX4 仿真器。

校验自带的 31 机示例包：

```bash
PYTHONPATH=runtime/src python -m uav_bt_runtime validate \
  runtime/examples/joint_mission/group_a \
  runtime/examples/joint_mission/group_b \
  --allow-unreviewed
```

跑完整联合任务：

```bash
PYTHONPATH=runtime/src python -m uav_bt_runtime simulate-joint \
  --group-a runtime/examples/joint_mission/group_a \
  --group-b runtime/examples/joint_mission/group_b \
  --allow-unreviewed
```

演练双机打击失败场景：

```bash
PYTHONPATH=runtime/src python -m uav_bt_runtime simulate-joint \
  --group-a runtime/examples/joint_mission/group_a \
  --group-b runtime/examples/joint_mission/group_b \
  --scenario strike-failure \
  --allow-unreviewed
```

配置文件拒绝未知字段。`UavSpec` 在任务期间不可变，`UavRuntimeState` 可变，两者通过
同一个 `uav_id` 关联。备份机的启用优先级即 `GroupRoster.reserve_ids` 的顺序。
