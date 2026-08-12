# 31节点联合任务包说明

本目录是当前实物场景的候选任务包。Group A和Group B地面站分别加载本组目录，
两者通过相同`mission_id`、互相匹配的版本和完全相同的`world.json`组成联合任务。

## 文件作用

| 文件 | 作用 | 当前关键内容 |
|---|---|---|
| `mission.json` | 包入口、版本、审核状态和SHA-256 | 生成后默认`reviewed=false` |
| `robots.json` | 静态UAV目录和能力 | A01-A15、B01-B16，共31节点 |
| `bootstrap.json` | 本组初始成员和任务状态 | active/reserve/failed/inactive |
| `world.json` | 两组共享场地和目标 | 100×150 m、5 m安全边界、两个目标 |
| `plans/plans.json` | UAV成员、命令、航迹和目标参数 | XML中的`plan_id`在此解析 |
| `tree.xml` | 行为树任务顺序和超时 | 不保存逐机大段坐标 |

`robots.json`中的能力必须覆盖计划实际命令。A组为A01-A12初始成员、A13-A15
Reserve；B组只有B01-B06具备本场景飞行资格，B07-B16放在`inactive_ids`中，
运行时不会给它们下发任务。

`bootstrap.json`中的`planned_fault_count=6`表示A07-A12计划退出；
`planned_recovery_count=3`表示只启用A13-A15。因此恢复后的有效编队为A01-A06、
A13-A15，共9架，而不是用Reserve补齐全部6架损失。

`world.json`使用`mission_enu`候选坐标系，场地为X=0..100、Y=0..150，安全边界
向内5 m。目标为`target-1=(65,65,12)`和`target-2=(75,105,12)`。正式部署前
仍须与底层共同确认RTK原点、轴向和高度基准。

## 当前行为树

Group A：

```text
12架集结 -> 自下向上扫描未侦查的下半区 -> Y=75 m中部终点
-> A07-A12受控退出 -> A13-A15人工起飞并在同一终点恢复9架编队
-> 从Y=75 m继续扫描未侦查的上半区
-> 发布侦查完成 -> 等待打击完成 -> 上边界安全区悬停
```

Group B：

```text
发布本组就绪 -> 等待A组就绪和侦查完成
-> B01/B02分别打击两个目标 -> B01/B02分别返回初始悬停点
-> 发布打击完成
```

起飞和最终降落不在XML中，也不会生成`TAKEOFF`或`LAND`命令。

## 计划数据与ROS Goal

- `MOVE_TO`：Goal包含本次成员、Formation和Leader目标点；Follower槽位由底层控制。
- `FOLLOW_ROUTE`：JSON只给A01保存稀疏Leader航点，Follower仅标记
  `formation_follow=true`；ROS Goal包含全体成员、Formation和一条A01 Route。
  初始3x4编队和恢复后的3x3编队在扫描横向上均为3行。按5 m行间距、单机5 m
  有效扫描宽度计算，整队扫描宽度为15 m。140 m有效扫描高度划分为10条扫描带，
  中心线间距14 m，相邻扫描带保留1 m重叠；每半区5条Swath、10个起终点和1个
  中部衔接点，因此每个覆盖Goal固定为11个航点。
- `FAULT_EXIT`：A07-A12各有一条Route，先降到8 m，再去6个不同安全悬停点。
- `ATTACK`：B01和B02分别通过`target_id`解析两个已知目标。
- `RETURN`：B01和B02各有一条返回Route，终点分别为(90,10,12)、(95,10,12)。
- `HOVER`：只表示当前任务终点稳定保持，不执行降落。

覆盖扫描带由`runtime/tools/generate_example_mission.py`通过独立Worker直接调用已安装的
Fields2Cover Python API生成并写入JSON，完全不使用`third_party/region2cover`。
生成器只调用扫描带生成和Boustrophedon排序，不调用Fields2Cover路径规划器；每条
扫描带只输出起点和终点。点间轨迹、转弯、插值、防碰撞和PX4 setpoint全部由底层负责。
扫描带不是按单架无人机5 m宽度逐架规划。当前Leader扫描带位于Y=12、26、40、
54、68、82、96、110、124、138 m；第一段终点与第二段起点均为`(80,75,12)`，
第二段不得返回已经扫描的下半区，最终Leader停在`(15,138,12)`附近。

## 修改和审核

参数修改位置：

| 修改内容 | 文件 |
|---|---|
| UAV编号、角色、Leader、Reserve、能力 | `robots.json` |
| active/reserve/inactive和损毁/恢复数量 | `bootstrap.json` |
| 场地边界、坐标系、目标 | 两组相同的`world.json` |
| 集结点、覆盖航迹、退出点、打击机和返航点 | `plans/plans.json` |
| 固定流程的超时 | `tree.xml` |
| 任务ID、包版本、对端版本 | `mission.json` |

修改引用文件后先撤销审核并更新哈希：

```bash
python runtime/tools/update_mission_hashes.py \
  runtime/examples/joint_mission/group_a \
  runtime/examples/joint_mission/group_b
```

候选包可用于静态检查和仿真：

```bash
python runtime/main.py --mode validate --allow-unreviewed
python runtime/main.py --mode visual --scenario normal --allow-unreviewed
```

人工逐项审核航迹、边界、成员、目标和版本后再执行：

```bash
python runtime/tools/update_mission_hashes.py --mark-reviewed \
  runtime/examples/joint_mission/group_a \
  runtime/examples/joint_mission/group_b
python runtime/main.py --mode validate
```

详细部署修改规则见`docs/实物任务包配置与部署修改指南.md`。
