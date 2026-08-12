# A/B 组实物无人机上下层 TCP/IP 接口对接方案

## 1. 交付边界与通信关系

runtime 是上层行为树（Behavior Tree，BT）任务执行程序。它根据任务树决定何时开始一项
任务、等待任务完成、继续执行下一节点或进入失败流程。runtime 不直接控制飞控，也不根据
位置、速度或里程计判断任务是否完成。

A/B 组分别在自己的地面站实现 TCP Server、消息校验、底层命令转换、无人机任务分发和
整组结果汇总：

```text
Group A runtime                    Group B runtime
  行为树 + TCP Client                行为树 + TCP Client
          |                                   |
          | TCP 长连接 / JSON Lines           | TCP 长连接 / JSON Lines
          v                                   v
 A组地面站 TCP Server                B组地面站 TCP Server
          |                                   |
          | A组自有通信桥                     | 例如 swarm_bridge
          v                                   v
      A01...A15                           B01、B02
```

本仓库提供：

- A/B 组通用 TCP Client；
- 行为树任务执行程序；
- A/B 组单任务联调程序；
- Mock TCP 地面站及协议测试。

本仓库不提供 A/B 组正式地面站 TCP Server。两组地面站开发人员需要按照本文实现 Server，
并接入各组已有底层控制接口。

## 2. TCP 连接与消息边界

### 2.1 基本规则

- 地面站作为 TCP Server，runtime 作为 TCP Client。
- 建议地面站监听 `0.0.0.0:39001`，实际 IP 和端口由现场配置决定。
- runtime 启动时建立 TCP 长连接，任务期间复用同一连接。
- 编码固定为 UTF-8。
- 一条消息是一个 JSON 对象，消息末尾必须包含换行符 `\n`。
- 不能用一次 `recv()` 对应一条消息；地面站必须缓存字节流并按 `\n` 拆包。
- 单条消息上限为 64 KiB。
- `version` 固定为字符串 `"1.0"`。
- 一个 runtime 进程只连接本组地面站。
- 同一 runtime 在同一时间最多执行一条整组命令。

例如，实际发送的字节流为：

```text
{"version":"1.0",...}\n
```

### 2.2 通用命令字段

runtime 发给地面站的正常命令使用以下外层结构：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "tcp-0123456789abcdef",
  "command": "ATTACK",
  "timeout_s": 300.0,
  "assignments": []
}
```

| 字段 | 类型 | 必填 | 地面站处理要求 |
|---|---|---|---|
| `version` | string | 是 | 只接受 `"1.0"` |
| `type` | string | 是 | runtime 命令固定为 `"COMMAND"` |
| `mission_id` | string | 是 | 联合任务 ID，状态回复必须原样返回 |
| `group_id` | string | 是 | 只允许 `GroupA` 或 `GroupB`；地面站只接受本组 |
| `command_id` | string | 是 | 本次整组命令唯一 ID，状态回复必须原样返回 |
| `command` | string | 是 | 具体命令，见 A/B 组命令章节 |
| `timeout_s` | number | 是 | 此行为树任务允许的最长执行时间，单位为秒 |
| `assignments` | array | 是 | 本次参与飞机及其任务参数，不能为空 |

一条 TCP 命令代表一次整组行为树任务，而不是一架飞机的一条命令。地面站应当读取整个
`assignments`，再向组内各架无人机分发。

`command_id` 由 runtime 自动生成。相同任务再次执行也会得到新的 `command_id`，因此地面站
不能使用 `command` 名称代替 `command_id` 去重。

## 3. runtime 行为树与状态返回

### 3.1 线上状态与行为树状态映射

地面站在 TCP 上只能发送以下三个 `status`：

| 地面站 TCP 状态 | 含义 | runtime 内部命令状态 | 当前行为树节点 |
|---|---|---|---|
| 尚未回复 | 地面站尚未确认接收 | 等待确认 | `RUNNING` |
| `ACCEPTED` | 格式正确，已接入底层并开始执行 | `ACCEPTED` 后进入 `RUNNING` | `RUNNING` |
| `COMPLETED` | 本次所有参与飞机均完成 | 每架机映射为 `SUCCESS` | `SUCCESS`，继续下一节点 |
| `FAILED` | 至少一架参与飞机失败 | 失败飞机映射为 `FAILURE` | `FAILURE`，当前 Sequence 失败 |
| 5 秒无 `ACCEPTED` | 地面站未及时确认 | `TIMEOUT` | `FAILURE` |
| 接受后超过任务超时 | 任务未按时完成 | `CANCELLED/TIMEOUT` | `FAILURE` |
| TCP 断开 | 无法继续确认任务状态 | `FAILURE` | `FAILURE` |
| 状态消息格式错误 | 回复无法被可信解析 | `FAILURE` | `FAILURE` |

注意：

- `RUNNING`、`SUCCESS`、`FAILURE` 是 runtime 内部行为树状态。
- 地面站不能在 TCP 消息中返回 `RUNNING`、`SUCCESS` 或 `FAILURE`。
- 地面站必须使用线上协议值 `ACCEPTED`、`COMPLETED` 或 `FAILED`。
- runtime 不需要进度百分比，也不会用进度判断成功。
- 收到 `ACCEPTED` 后，可以长时间没有新消息；只要未超过 `timeout_s`，行为树会保持
  `RUNNING`。

### 3.2 正确的状态时序

正常任务必须按照以下顺序：

```text
runtime                                地面站
   |                                      |
   | COMMAND                              |
   |------------------------------------->|
   |                                      | 校验、去重、接入底层
   | STATUS: ACCEPTED（5秒内）             |
   |<-------------------------------------|
   | 行为树保持 RUNNING                    | 飞机执行任务
   |                                      |
   | STATUS: COMPLETED 或 FAILED           |
   |<-------------------------------------|
   | 行为树转为 SUCCESS 或 FAILURE         |
```

`ACCEPTED` 只能在以下条件全部满足后发送：

1. JSON 和所有必填字段校验通过；
2. `group_id` 与本地地面站一致；
3. 命令和飞机 ID 得到支持；
4. 命令已经成功交给本组任务调度或底层控制入口；
5. 地面站有能力继续汇总该任务结果。

仅仅“TCP 已经收到字节”不能视为 `ACCEPTED`。

## 4. 地面站返回消息

### 4.1 ACCEPTED：已接收并开始执行

地面站必须在 runtime 发送命令后的 5 秒内返回：

```json
{
  "version": "1.0",
  "type": "STATUS",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "tcp-0123456789abcdef",
  "status": "ACCEPTED",
  "message": "attack command accepted"
}
```

必填字段为 `version/type/mission_id/group_id/command_id/status`。`message` 可选，仅用于日志和
现场排查。

地面站必须把原命令的 `mission_id`、`group_id`、`command_id` 原样返回。如果 ID 不匹配，
runtime 会把回复视为其他任务的旧消息并忽略，最终可能触发超时。

### 4.2 COMPLETED：整组全部完成

```json
{
  "version": "1.0",
  "type": "STATUS",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "tcp-0123456789abcdef",
  "status": "COMPLETED",
  "completed_uavs": ["B01", "B02"],
  "message": "B01 and B02 completed the attack"
}
```

`completed_uavs` 必填，并且必须满足：

- 与原命令 `assignments` 中的全部 `uav_id` 完全一致；
- 不能缺少飞机；
- 不能包含未参与本次任务的飞机；
- 不能出现重复 ID。

例如原命令包含 B01、B02，只返回 `completed_uavs:["B01"]` 会被 runtime 视为无效状态，
不会使行为树成功。

### 4.3 FAILED：任务失败

```json
{
  "version": "1.0",
  "type": "STATUS",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "tcp-0123456789abcdef",
  "status": "FAILED",
  "failed_uavs": ["B02"]
}
```

`failed_uavs` 必填，并且必须是非空、无重复的数组，只能包含原命令中参与的飞机。
`error_code` 和 `message` 都是可选字段，地面站可以不发送。

runtime 会把 `failed_uavs` 中的飞机标记为本批次失败，并使整个行为树节点返回 `FAILURE`。
没有列入 `failed_uavs` 的参与飞机仅表示本批次未报告失败，不会改变“整组任务失败”的结果。

对于能够解析并取得任务 ID 的非法命令，地面站应返回 `FAILED`，`failed_uavs` 填写该命令
能够识别出的全部参与飞机。对于无法解析的 JSON、非法 UTF-8、消息超长或缺少关联 ID 的
消息，地面站记录错误并关闭当前 TCP 连接，不应猜测 `mission_id` 或 `command_id`。

### 4.4 A 组状态回复示例

A/B 两组使用完全相同的状态结构，但 `group_id` 和飞机列表必须对应本组。例如 A 组
`FAULT_EXIT` 命令开始执行时返回：

```json
{
  "version": "1.0",
  "type": "STATUS",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "tcp-a-exit-example",
  "status": "ACCEPTED",
  "message": "fault exit command accepted"
}
```

A07-A12 全部完成退出航线后返回：

```json
{
  "version": "1.0",
  "type": "STATUS",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "tcp-a-exit-example",
  "status": "COMPLETED",
  "completed_uavs": ["A07", "A08", "A09", "A10", "A11", "A12"],
  "message": "all fault-exit UAVs completed their routes"
}
```

如果 A10 退出失败，不能返回部分 `COMPLETED`，应返回：

```json
{
  "version": "1.0",
  "type": "STATUS",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "tcp-a-exit-example",
  "status": "FAILED",
  "failed_uavs": ["A10"]
}
```

## 5. A 组命令

### 5.1 A 组任务批次

| 单任务测试名 | 线上命令 | 参与飞机 | 地面站返回 `COMPLETED` 的条件 |
|---|---|---|---|
| `prepare` | `MOVE_TO` | A01-A12 | 12 架机全部到达各自目标并稳定悬停 |
| `coverage-segment-1` | `FOLLOW_ROUTE` | A01-A12 | A01 完成航线、其余飞机完成编队跟随，整组稳定悬停 |
| `fault-exit` | `FAULT_EXIT` | A07-A12 | 6 架退出飞机分别完成自己的退出航线 |
| `recovery` | `MOVE_TO` | A01-A06、A13-A15 | 9 架当前活动飞机到达恢复编队位置并稳定悬停 |
| `coverage-segment-2` | `FOLLOW_ROUTE` | A01-A06、A13-A15 | 恢复后的 9 机编队完成航线并稳定悬停 |
| `hold-end` | `HOVER` | A01-A06、A13-A15 | 9 架机全部进入稳定悬停 |

### 5.2 MOVE_TO

每个 assignment 必须包含 `uav_id/x/y/z/yaw`：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "tcp-a-move-example",
  "command": "MOVE_TO",
  "timeout_s": 90.0,
  "assignments": [
    {"uav_id": "A01", "x": 17.5, "y": 50.0, "z": 12.0, "yaw": 0.0},
    {"uav_id": "A02", "x": 12.5, "y": 45.0, "z": 12.0, "yaw": 0.0}
  ]
}
```

上例只展示两个 assignment。实际 `prepare` 消息会在同一个数组中携带 A01-A12 全部 12 架
飞机，实际 `recovery` 消息会携带恢复后的 9 架飞机。

地面站职责：分别下发目标位姿；只有本条消息中的全部飞机到达并稳定悬停后，才返回
`COMPLETED`。

### 5.3 FOLLOW_ROUTE

`leader_id` 必填，当前固定为 `A01`。A01 携带完整 `waypoints`，每个航点包含
`x/y/z/yaw`；其他参与飞机携带 `formation_follow:true`：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "tcp-a-route-example",
  "command": "FOLLOW_ROUTE",
  "timeout_s": 300.0,
  "leader_id": "A01",
  "assignments": [
    {
      "uav_id": "A01",
      "waypoints": [
        {"x": 15.0, "y": 12.0, "z": 12.0, "yaw": 0.0},
        {"x": 80.0, "y": 12.0, "z": 12.0, "yaw": 0.0}
      ]
    },
    {"uav_id": "A02", "formation_follow": true}
  ]
}
```

示例航点和跟随机数量经过缩短；地面站必须处理 runtime 实际消息中的完整数组。地面站负责
将领导机航线转换给底层，并根据本组现有编队算法让其他飞机跟随。只有领导机完成航线、
全部参与飞机完成编队任务且整组稳定悬停后，才返回 `COMPLETED`。

### 5.4 FAULT_EXIT

每架退出飞机都携带自己的非空 `waypoints`，不能只读取第一架飞机的航线后复制：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "tcp-a-exit-example",
  "command": "FAULT_EXIT",
  "timeout_s": 180.0,
  "assignments": [
    {
      "uav_id": "A07",
      "waypoints": [
        {"x": 85.0, "y": 75.0, "z": 8.0, "yaw": 0.0},
        {"x": 5.0, "y": 20.0, "z": 8.0, "yaw": 0.0}
      ]
    },
    {
      "uav_id": "A08",
      "waypoints": [
        {"x": 90.0, "y": 75.0, "z": 8.0, "yaw": 0.0},
        {"x": 10.0, "y": 20.0, "z": 8.0, "yaw": 0.0}
      ]
    }
  ]
}
```

实际任务包含 A07-A12。全部退出飞机执行完自己的航线后返回 `COMPLETED`；任意一架失败，
返回 `FAILED` 并在 `failed_uavs` 中列出对应 ID。

### 5.5 HOVER

`HOVER` 不携带目标坐标，只列出需要悬停的飞机：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "tcp-a-hover-example",
  "command": "HOVER",
  "timeout_s": 60.0,
  "assignments": [
    {"uav_id": "A01"},
    {"uav_id": "A02"}
  ]
}
```

地面站使用底层当前位姿悬停逻辑；全部参与飞机稳定悬停后返回 `COMPLETED`。

## 6. B 组命令

B 组正常任务只有 `ATTACK` 和 `RETURN`。两条正常命令都必须同时包含且只包含 B01、B02。

| 单任务测试名 | 线上命令 | 地面站返回 `COMPLETED` 的条件 |
|---|---|---|
| `strike-targets` | `ATTACK` | B01、B02 均完成前往目标、悬停和打击 |
| `return-strike-uavs` | `RETURN` | B01、B02 均到达各自返航位置并稳定悬停 |

### 6.1 ATTACK

runtime 根据任务包的目标定义，将 `target_id` 转换为目标坐标后发送。每个 assignment 必须
包含 `uav_id/target_id/x/y/z/yaw`：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "tcp-b-attack-example",
  "command": "ATTACK",
  "timeout_s": 300.0,
  "assignments": [
    {
      "uav_id": "B01",
      "target_id": "target-1",
      "x": 65.0,
      "y": 65.0,
      "z": 12.0,
      "yaw": 0.0
    },
    {
      "uav_id": "B02",
      "target_id": "target-2",
      "x": 85.0,
      "y": 105.0,
      "z": 12.0,
      "yaw": 0.0
    }
  ]
}
```

地面站收到后应：

1. 校验恰好包含 B01、B02，且目标 ID 和坐标字段完整；
2. 将 B01、B02 的任务分别转发给现有底层控制入口；
3. 成功进入底层执行后，在 5 秒内回复 `ACCEPTED`；
4. 分别等待两架飞机完成“到达目标、悬停、打击”；
5. 两架均完成才回复 `COMPLETED`；
6. 任意一架失败立即回复 `FAILED`，并准确填写 `failed_uavs`。

`ATTACK` 已经包含飞向目标、到点悬停和打击，不需要地面站等待 runtime 再发送其他命令。

### 6.2 RETURN

每架飞机包含一个返航位姿：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "tcp-b-return-example",
  "command": "RETURN",
  "timeout_s": 300.0,
  "assignments": [
    {"uav_id": "B01", "x": 90.0, "y": 10.0, "z": 12.0, "yaw": 0.0},
    {"uav_id": "B02", "x": 95.0, "y": 10.0, "z": 12.0, "yaw": 0.0}
  ]
}
```

成功下发返航任务后回复 `ACCEPTED`。只有 B01、B02 均到达各自位置并稳定悬停，才能回复：

```json
{
  "version": "1.0",
  "type": "STATUS",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "tcp-b-return-example",
  "status": "COMPLETED",
  "completed_uavs": ["B01", "B02"],
  "message": "return completed and both UAVs are hovering"
}
```

## 7. HOLD 安全命令

runtime 在以下情况下尝试发送 `HOLD`：

- 行为树任务超时；
- 当前任务被取消；
- runtime 收到人工中止或安全悬停请求；
- 发送命令后 5 秒内没有收到 `ACCEPTED`。

实际消息格式为：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupB",
  "command_id": "hold-0123456789abcdef",
  "command": "HOLD",
  "timeout_s": 5.0,
  "assignments": [
    {"uav_id": "B01"},
    {"uav_id": "B02"}
  ],
  "reason": "runtime requested safe hold"
}
```

`reason` 可能为 `runtime requested safe hold`、`command cancelled` 或 `ACCEPTED timeout`，
地面站不能依赖固定文案判断是否执行。

地面站收到 `HOLD` 后必须立即：

1. 停止或取消这些飞机的当前正常任务；
2. 调用本组已有安全悬停接口；
3. 不再继续旧 `ATTACK/RETURN/MOVE_TO` 等任务；
4. 可先对 HOLD 回复 `ACCEPTED`，稳定悬停后再回复 `COMPLETED`。

当前 runtime 发送 `HOLD` 后不会等待其完成状态才进入失败流程。因此地面站的安全动作不能
依赖 runtime 再次确认，也不能因为状态回复无人接收而放弃悬停。

如果 TCP 在正常任务期间断开，runtime 已无法发送 `HOLD`。地面站必须检测连接断开，并在
本地自动执行相同的停止任务和安全悬停流程。

## 8. 去重、并发与重连规则

### 8.1 command_id 去重

地面站必须保存每个 `command_id` 的当前状态：

```text
未见过 -> ACCEPTED -> COMPLETED
                    -> FAILED
```

重复收到相同 `command_id` 时：

- 不得再次向无人机下发；
- 如果当前仍在执行，重新返回缓存的 `ACCEPTED`；
- 如果已经完成，返回缓存的 `COMPLETED` 及原 `completed_uavs`；
- 如果已经失败，返回缓存的 `FAILED` 和 `failed_uavs`；可选信息如有也应保持一致。

### 8.2 同时只能执行一条命令

runtime 正常情况下不会并发发送两个任务。如果地面站正在执行一条命令，又收到不同的
`command_id`：

- `HOLD` 必须优先接受并立即执行；
- 其他新命令返回 `FAILED`；
- 不能覆盖正在执行任务的状态记录。

### 8.3 断线与重连

- 任务执行期间连接断开：地面站立即取消当前任务并安全悬停。
- 断线后不能继续执行旧命令，也不能等 runtime 自动恢复旧任务。
- runtime 当前不会在同一行为树节点内自动重连并重发。
- 建立新连接后，只有新的 `command_id` 才能开始新的任务。
- 地面站应保留近期 `command_id` 结果，避免网络抖动导致旧命令被重复执行。

## 9. 地面站实现检查表

地面站开发人员至少需要完成：

1. TCP Server 监听、长连接管理和按换行拆包；
2. UTF-8、JSON、64 KiB 上限及公共字段校验；
3. 固定本组 `group_id` 校验；
4. 本组命令和 assignment 字段校验；
5. `command_id` 去重及状态缓存；
6. 一条整组命令向正确无人机分发；
7. 在 5 秒内返回 `ACCEPTED`；
8. 汇总全部参与飞机结果并返回 `COMPLETED` 或 `FAILED`；
9. `HOLD`、任务超时、人工中止和 TCP 断线的安全悬停；
10. 坐标系、单位、原点、航向角定义和飞机 ID 映射的现场确认。

每条地面站状态消息也必须使用 UTF-8 JSON Lines，并以 `\n` 结束。

## 10. 单任务联调

json文件修改，更新对应has值：

修改完 Group A 的 `world.json` 后，运行：

```
python3 runtime/tools/update_mission_hashes.py \
  runtime/examples/joint_mission/group_b
```





先启动对应组地面站 TCP Server，再在 runtime 电脑执行。只有一台电脑时，可以让地面站
监听 `127.0.0.1`，并把 `--tcp-host` 设置为 `127.0.0.1`。

### 10.1 A 组

```bash
python3 runtime/task_test.py --backend tcp --group A --task prepare \
  --tcp-host <A组地面站IP> --tcp-port 39001

python3 runtime/task_test.py --backend tcp --group A --task coverage-segment-1 \
  --tcp-host <A组地面站IP> --tcp-port 39001

python3 runtime/task_test.py --backend tcp --group A --task fault-exit \
  --tcp-host <A组地面站IP> --tcp-port 39001

python3 runtime/task_test.py --backend tcp --group A --task recovery \
  --tcp-host <A组地面站IP> --tcp-port 39001

python3 runtime/task_test.py --backend tcp --group A --task coverage-segment-2 \
  --tcp-host <A组地面站IP> --tcp-port 39001

python3 runtime/task_test.py --backend tcp --group A --task hold-end \
  --tcp-host <A组地面站IP> --tcp-port 39001
```

### 10.2 B 组

```bash
python3 runtime/task_test.py --backend tcp --group B --task strike-targets \
  --tcp-host 192.168.31.4 --tcp-port 39001

python3 runtime/task_test.py --backend tcp --group B --task return-strike-uavs \
  --tcp-host 192.168.31.4 --tcp-port 39001
  

```

客户端进程退出码：

- `0`：本次行为树任务节点为 `SUCCESS`；
- `1`：任务得到有效执行结果，但行为树节点为 `FAILURE`；
- `2`：参数、任务包、连接或程序配置错误。

成功时，程序最后输出的 JSON 中包含 `"status":"SUCCESS"`；失败时包含
`"status":"FAILURE"` 和 `failed_robot_ids`。

## 11. 单组完整行为树联调

```bash
./runtime/run_tcp.sh --package runtime/examples/joint_mission/group_a \
  --tcp-host <A组地面站IP> --tcp-port 39001 \
  --auto-peer-events --allow-unreviewed

./runtime/run_tcp.sh --package runtime/examples/joint_mission/group_b \
  --tcp-host 192.168.31.4 --tcp-port 39001 \
  --auto-peer-events --allow-unreviewed
```

`--auto-peer-events` 只用于单组接口联调，用于自动满足行为树中等待另一组阶段事件的节点。
A/B runtime 之间正式的跨进程阶段事件通信不在本次接口范围内。

## 12. 实物测试前确认

示例任务包当前用于协议和流程联调。上桨前，A/B 组必须共同确认：

- `world.json` 和 `plans.json` 中的坐标是否适用于实际场地；
- `x/y/z` 的单位、坐标原点、坐标轴方向和高度基准；
- `yaw` 的单位和正方向；
- runtime 飞机 ID 与地面站、底层、飞控 ID 的映射；
- 地面站判断“稳定悬停”“航线完成”“打击完成”的实际条件；
- 断线和 `HOLD` 是否能在不上桨测试中可靠触发安全动作。

在这些内容得到人工复核前，只能进行消息解析、模拟底层和不上桨联调，不能直接使用示例
坐标执行实物飞行。
