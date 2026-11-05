# 01 数据流与 CSV 说明

> 最后更新：2026-11-05
> 对应代码提交：`ff7160b`

实验数据一共三张表，分别由**两个不同的进程**写入。理解这三张表的边界，是判断实验结果可信度的前提。

## 一、总体数据流

```
Gazebo 物理引擎                    Nav2 / AMCL
      │                                │
      │ /gazebo/model_states           ├─ /amcl_pose
      │ (真值)                          │ (估计值)
      ▼                                ▼
   ┌─────────────────────────────────────────┐
   │   scripts/capture_truth_pose.py         │
   │   --watch     → pose_trace.csv（1 Hz）  │
   │   --finalize  → truth_last_pose.csv     │
   └─────────────────────────────────────────┘

   ┌─────────────────────────────────────────┐
   │   src/autopartol_robot/partol_node.py   │
   │   每轮结束 → patrol_runs.csv            │
   └─────────────────────────────────────────┘
```

两张表之间**没有主键关联之外的耦合**：`truth_last_pose.csv` 的 `run_id` 取自 `patrol_runs.csv` 的最后一行（`last_run_id()`），用于事后对齐同一轮实验。

## 二、三张表

### 2.1 `experiment_logs/patrol_runs.csv`

**写入者**：`partol_node.py` → `append_run_record()`（第 41-69 行）
**时机**：每轮巡逻结束时追加一行
**性质**：任务结果统计，**不含任何传感器数据**

| 字段 | 含义 | 来源 |
|---|---|---|
| `run_id` | `YYYYMMDD_HHMMSS_r{轮次}` | 本轮开始的墙上时钟 |
| `start_time` | ISO 时间，本轮开始 | `datetime.now()`（墙上时钟） |
| `end_time` | ISO 时间，本轮结束 | `datetime.now()`（墙上时钟） |
| `total_goals` | 本轮目标点数 | `len(points)` |
| `succeeded` | 导航成功数 | Nav2 action 返回 `succeeded` |
| `failed` | 导航失败数 | Nav2 action 返回 `failed` |
| `timeout` | 超时数 | 本地计时超过 `timeout` 阈值 |
| `canceled` | 取消数 | Nav2 action 返回 `canceled` |
| `rejected` | 拒绝数 | Nav2 action 返回 `rejected` |
| `total_time_s` | 本轮耗时（秒） | `get_clock().now()`（**仿真时钟**） |
| `max_rounds` | 该次运行的轮数上限 | 参数 `max_rounds` |
| `note` | `completed` / `stopped_on_failure` | 本轮是否因失败中止 |

**校验等式**（论文数据自检时必用）：

```
succeeded + failed + timeout + canceled + rejected == total_goals
```

`rejected` 是**独立类别**，不并入 `failed`。

### 2.2 `experiment_logs/truth_last_pose.csv`

**写入者**：`capture_truth_pose.py` → `run_finalize()`（第 127-166 行）
**时机**：每轮结束后执行一次
**性质**：末点位姿对照，是**唯一能算末点误差**的表

| 字段 | 含义 | 坐标系 |
|---|---|---|
| `run_id` | 同 `patrol_runs.csv` 最后一行的 `run_id` | — |
| `gt_x` / `gt_y` | Gazebo 真值位置 | `world` |
| `gt_yaw` | Gazebo 真值航向，归一化到 `[-π, π]` | `world` |
| `amcl_x` / `amcl_y` | AMCL 估计位置 | `map` |
| `amcl_yaw` | AMCL 估计航向，归一化到 `[-π, π]` | `map` |

`map` 与 `world` 原点已验证对齐（位置差 < 0.05 m），故 `gt_*` 与 `amcl_*` 可直接相减。

### 2.3 `experiment_logs/run_one_<时间戳>/pose_trace.csv`

**写入者**：`capture_truth_pose.py` → `PoseWatcher.trace()`（第 90-109 行），`--watch` 模式
**时机**：运行中每 `--period`（默认 1.0 秒）追加一行
**性质**：逐秒轨迹，用于绘制误差-时间曲线

| 字段 | 含义 |
|---|---|
| `t_sec` | 相对采样起点的秒数，**墙上时钟**（`time.time()-t0`） |
| `gt_x` / `gt_y` / `gt_yaw` | 同一时刻的真值 |
| `amcl_x` / `amcl_y` / `amcl_yaw` | 同一时刻的 AMCL 估计 |

**注意**：`amcl_pose` 只在 AMCL 滤波器更新时才刷新（受 `update_min_d` / `update_min_a` 节流），因此该列在相邻几行可能**完全相同**，这是正常现象，不代表采样失败。

## 三、数据来源只有三类

排查数据问题时，先判断这一列属于哪一类：

### ① 自己数出来的
不来自任何传感器。`partol_node.py` 第 262-288 行根据 `nav_to_pose()` 返回的字符串对计数器 `+= 1`。

```
Nav2 action 结果 → 字符串 → 计数器 +1 → CSV
```

**风险**：只有二分状态，无法反映"为什么失败"。根因必须查 `pose_trace.csv`。

### ② 时钟读出来的
- `start_time` / `end_time` → `datetime.now()`（墙上时钟）
- `total_time_s` → `get_clock().now()`（仿真时钟）

**这两个钟不是同一个**。实时因子 1.0 时差异约 1~5 秒；若改变仿真速度或注入延迟，两者会显著分离。论文中必须注明采用哪一个。

### ③ 订阅来的（唯一真实的传感器数据）
```
/gazebo/model_states  →  gt_*    （物理引擎真值）
/amcl_pose            →  amcl_*  （定位估计）
```
只有这两类字段能用于误差计算。

## 四、两条典型链路的完整追踪

### 4.1 `succeeded` 从哪来

```
Nav2 action 结果
  → nav_to_pose() 返回 'succeeded'        partol_node.py:139
  → nav_succeeded = True                   partol_node.py:262-263
  → success_count += 1                     partol_node.py:273
  → append_run_record({'succeeded': ...})  partol_node.py:310
  → csv.writer.writerow(...)               partol_node.py:66
```

### 4.2 `gt_x` 从哪来

```
Gazebo 插件 gazebo_ros_state（WorldPlugin）
  → /gazebo/model_states
  → PoseWatcher.on_model_states()          capture_truth_pose.py:65-74
  → cache['gt']['x']                       capture_truth_pose.py:71
  → trace() 写 pose_trace.csv              capture_truth_pose.py:107
  或 finalize() 写 truth_last_pose.csv     capture_truth_pose.py:163
```

## 五、已知问题与口径

| 问题 | 说明 | 处理 |
|---|---|---|
| 两个时钟 | `end_time - start_time` ≠ `total_time_s` | 统计耗时统一用 `total_time_s` |
| 航向环绕 | 必须归一化到 `[-π, π]`，否则 ±180° 处误差爆炸 | 计算前先归一化 |
| 历史数据混版 | 早期版本曾写出 `gt_yaw = -4.845`（超出 `[-π, π]`） | 跨版本数据不得混合统计；旧数据需标注来源版本 |
| AMCL 节流 | 相邻行 `amcl_*` 可能重复 | 属正常，不影响结论 |
| 误差口径 | 必须用 `gt - amcl` | **不可**用 AMCL 自身的 `covariance` 当误差 |
| 仿真的局限 | Gazebo 真值是数学精确，无传感器噪声 | 仿真结论是"理想条件下的算法误差"，不能直接等同实车性能 |
| 超时硬编码 | `timeout = 60.0` 写死在 `partol_node.py:114` | 改超时需改代码，尚未参数化 |

## 六、新增数据列时的约定

1. 先确认属于哪一类来源（计数 / 时钟 / 订阅）
2. 必须写清坐标系与单位
3. 时间字段必须注明墙上时钟还是仿真时钟
4. 表头变更需在本文档同步更新，并注明生效提交号
