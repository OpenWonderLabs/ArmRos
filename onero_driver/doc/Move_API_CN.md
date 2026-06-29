# OneroArm Move API 使用文档

**版本**: V1.1
**日期**: 2026-6-18

---

## 目录

- [OneroArm Move API 使用文档](#oneroarm-move-api-使用文档)
  - [目录](#目录)
  - [概述](#概述)
  - [话题命名约定](#话题命名约定)
  - [API接口](#api接口)
    - [MoveJ - 关节空间运动](#movej---关节空间运动)
    - [MoveL - 笛卡尔空间直线运动](#movel---笛卡尔空间直线运动)
    - [MoveP - 笛卡尔空间点到点运动](#movep---笛卡尔空间点到点运动)
    - [DualMoveJ - 双臂同步关节运动](#dualmovej---双臂同步关节运动)
  - [状态接口](#状态接口)
    - [机械臂状态发布](#机械臂状态发布)

---

## 概述

OneroArm Move API提供了三种主要的运动控制接口：

1. **MoveJ**: 关节空间运动，控制机械臂各关节按指定角度运动
2. **MoveL**: 笛卡尔空间直线运动，控制末端执行器沿直线路径运动
3. **MoveP**: 笛卡尔空间点到点运动，控制末端从当前位姿运动到目标位姿

所有API通过ROS2话题接口提供，命令结果通过对应的 `*_result` 话题异步发布。本文示例默认使用单臂模式（A1-L 或 A1-R）的 `/onero_arm` 话题前缀。

---

## 话题命名约定

单臂模式和双臂模式复用同一组单臂控制接口，只是命名空间不同：

| 使用场景 | 命名空间 | 示例 |
| -------- | -------- | ---- |
| 单臂模式 | `/onero_arm` | `/onero_arm/movej` |
| 双臂左臂 | `/onero_arm/left_arm` | `/onero_arm/left_arm/movej` |
| 双臂右臂 | `/onero_arm/right_arm` | `/onero_arm/right_arm/movej` |
| 双臂同步接口 | `/onero_arm/dual_arm` | `/onero_arm/dual_arm/movej` |

其中 `/onero_arm/left_arm/...` 与 `/onero_arm/right_arm/...` 用于分别控制单只机械臂；`/onero_arm/dual_arm/...` 用于需要左右臂由同一调度器同步处理的命令。

常用话题如下：

| 功能 | 单臂模式 | 双臂模式 |
| ---- | -------- | -------- |
| 关节空间运动 | `/onero_arm/movej` | `/onero_arm/left_arm/movej`、`/onero_arm/right_arm/movej`、`/onero_arm/dual_arm/movej` |
| 笛卡尔直线运动 | `/onero_arm/movel` | `/onero_arm/left_arm/movel`、`/onero_arm/right_arm/movel` |
| 笛卡尔点到点运动 | `/onero_arm/movep` | `/onero_arm/left_arm/movep`、`/onero_arm/right_arm/movep` |
| 停止运动 | `/onero_arm/stop` | `/onero_arm/left_arm/stop`、`/onero_arm/right_arm/stop`、`/onero_arm/dual_arm/stop` |
| 状态发布 | `/onero_arm/arm_state` | `/onero_arm/left_arm/arm_state`、`/onero_arm/right_arm/arm_state`、`/onero_arm/dual_arm/sync_state` |
| 命令结果 | `<cmd>_result` | `<cmd>_result`，例如 `/onero_arm/dual_arm/movej_result` |

---

## API接口

### MoveJ - 关节空间运动

**功能**: 控制机械臂在关节空间运动到目标关节角度。

**话题名称**: `<arm_ns>/movej`

**消息类型**: `onero_interfaces/msg/MoveJ`

**消息定义**:

```msg
float32[7] joint_positions   # 目标关节角度（弧度）
uint8 trajectory_connect     # 轨迹连接标志 (0:立即执行 1:与下一条轨迹连接)
float32 speed_scale          # 速度缩放因子 (0.01~5.0, 默认1.0)
```

**参数说明**:


| 参数               | 类型        | 说明                                                       | 范围     |
| ------------------ | ----------- | ---------------------------------------------------------- | -------- |
| joint_positions    | float32[7]  | 目标关节角度（弧度）                                       | 7轴关节  |
| trajectory_connect | uint8       | 轨迹连接：0-立即执行，1-与下一条连接                       | 0或1     |
| **speed_scale**    | **float32** | **速度缩放因子（推荐使用）** <1.0:降速 =1.0:正常 >1.0:加速 | 0.01~5.0 |

**trajectory_connect 轨迹连接使用说明**：

- **值为 1**：当前轨迹被缓存，命令立即返回成功（`CommandResult.success=true`），但机械臂不会立即运动
- **值为 0**：执行所有缓存的轨迹 + 当前轨迹，作为一个连续平滑的整体运动
- **典型用法**：发送 N-1 条 trajectory_connect=1 的命令缓存路径点，最后发送 1 条 trajectory_connect=0 触发执行
- **平滑性**：多条轨迹将在关节空间统一规划，保证速度和加速度的平滑过渡
- **清除缓冲**：异常情况下可调用服务 `<arm_ns>/clear_buffer` 清除已缓存的轨迹

**返回**: 通过 `<arm_ns>/movej_result` 话题发布 `onero_interfaces/msg/CommandResult`，其中 `success` 表示执行是否成功，`error_code` 与 `error_message` 给出失败原因。

**双臂模式下控制单臂**: 使用左臂或右臂命名空间分别下发 MoveJ 命令：

| 控制对象 | 命令话题 | 结果话题 |
| -------- | -------- | -------- |
| 左臂 | `/onero_arm/left_arm/movej` | `/onero_arm/left_arm/movej_result` |
| 右臂 | `/onero_arm/right_arm/movej` | `/onero_arm/right_arm/movej_result` |

双臂模式下的单臂 MoveJ 不经过 `/onero_arm/dual_arm` 同步调度器，适用于只需要单侧机械臂执行关节空间运动的场景。若需要左右臂同步启动并协调完成时间，请使用 `/onero_arm/dual_arm/movej`。

**示例**:

```bash
# 单臂模式：使用speed_scale（推荐）
ros2 topic pub --once /onero_arm/movej onero_interfaces/msg/MoveJ "{
  joint_positions: [0.0, 0.0, 0.0, -0.7, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 0,
}"

# 双臂模式：控制左臂执行 MoveJ
ros2 topic pub --once /onero_arm/left_arm/movej onero_interfaces/msg/MoveJ "{
  joint_positions: [0.0, 0.2, 0.0, -0.7, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 双臂模式：控制右臂执行 MoveJ
ros2 topic pub --once /onero_arm/right_arm/movej onero_interfaces/msg/MoveJ "{
  joint_positions: [0.0, -0.2, 0.0, -0.7, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 轨迹连接示例：缓冲多个点，最后一次触发执行
# 第1个点：缓冲（立即返回）
ros2 topic pub --once /onero_arm/movej onero_interfaces/msg/MoveJ "{
  joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 1,
}"

# 第2个点：缓冲（立即返回）
ros2 topic pub --once /onero_arm/movej onero_interfaces/msg/MoveJ "{
  joint_positions: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 1,
}"

# 第3个点：触发执行所有缓冲的轨迹（等待完成后返回）
ros2 topic pub --once /onero_arm/movej onero_interfaces/msg/MoveJ "{
  joint_positions: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 0
}"
```

---

### MoveL - 笛卡尔空间直线运动

**功能**: 控制机械臂末端执行器沿直线路径运动到目标位姿。

**话题名称**: `<arm_ns>/movel`

**消息类型**: `onero_interfaces/msg/MoveL`

**消息定义**:

```msg
geometry_msgs/Pose pose      # 目标位姿
uint8 trajectory_connect     # 轨迹连接标志
float32 speed_scale          # 速度缩放因子 (0.01~5.0)
```

**参数说明**:


| 参数               | 类型        | 说明                                           | 范围     |
| ------------------ | ----------- | ---------------------------------------------- | -------- |
| pose               | Pose        | 目标位姿（位置+姿态）                          | -        |
| trajectory_connect | uint8       | 轨迹连接标志                                   | 0或1     |
| **speed_scale**    | **float32** | **速度缩放因子** <1.0:降速 =1.0:正常 >1.0:加速 | 0.01~5.0 |

**pose字段说明**:

- `position`: (x, y, z) - 目标位置，单位：米
- `orientation`: (x, y, z, w) - 目标姿态（四元数）

**返回**: 通过 `<arm_ns>/movel_result` 话题发布 `onero_interfaces/msg/CommandResult`。

**双臂模式下控制单臂**: 使用左臂或右臂命名空间分别下发 MoveL 命令：

| 控制对象 | 命令话题 | 结果话题 |
| -------- | -------- | -------- |
| 左臂 | `/onero_arm/left_arm/movel` | `/onero_arm/left_arm/movel_result` |
| 右臂 | `/onero_arm/right_arm/movel` | `/onero_arm/right_arm/movel_result` |

**示例**:

```bash
# 单臂模式：A1-L（a1_l_driver.launch.py）
ros2 topic pub --once /onero_arm/movel onero_interfaces/msg/MoveL "{
  pose: {
    position: {x: -0.4786, y: 0.2144, z: 0.2552},
    orientation: {x: -0.6082, y: -0.2328, z: 0.3607, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 单臂模式：A1-R（a1_r_driver.launch.py）
ros2 topic pub --once /onero_arm/movel onero_interfaces/msg/MoveL "{
  pose: {
    position: {x: -0.4785, y: 0.2145, z: 0.2559},
    orientation: {x: -0.6082, y: -0.2327, z: 0.3608, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 双臂模式：控制左臂执行 MoveL
ros2 topic pub --once /onero_arm/left_arm/movel onero_interfaces/msg/MoveL "{
  pose: {
    position: {x: -0.4786, y: 0.2144, z: 0.2552},
    orientation: {x: -0.6082, y: -0.2328, z: 0.3607, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 双臂模式：控制右臂执行 MoveL
ros2 topic pub --once /onero_arm/right_arm/movel onero_interfaces/msg/MoveL "{
  pose: {
    position: {x: -0.4785, y: 0.2145, z: 0.2559},
    orientation: {x: -0.6082, y: -0.2327, z: 0.3608, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"
```

---

### MoveP - 笛卡尔空间点到点运动

**功能**: 以末端目标位姿为输入，在笛卡尔空间执行点到点（Point-to-Point, PTP）运动，使机械臂末端从当前位姿运动到目标位姿。

**话题名称**: `<arm_ns>/movep`

**消息类型**: `onero_interfaces/msg/MoveP`

**消息定义**:

```msg
geometry_msgs/Pose pose      # 目标末端位姿
uint8 trajectory_connect     # 轨迹连接标志
float32 speed_scale          # 速度缩放因子 (0.01~5.0)
```

**参数说明**:


| 参数               | 类型        | 说明                                           | 范围     |
| ------------------ | ----------- | ---------------------------------------------- | -------- |
| pose               | Pose        | 目标末端位姿（位置+姿态）                      | -        |
| trajectory_connect | uint8       | 轨迹连接：0-立即执行，1-与下一条连接           | 0或1     |
| **speed_scale**    | **float32** | **速度缩放因子** <1.0:降速 =1.0:正常 >1.0:加速 | 0.01~5.0 |


**返回**: 通过 `<arm_ns>/movep_result` 话题发布 `onero_interfaces/msg/CommandResult`。

**双臂模式下控制单臂**: 使用左臂或右臂命名空间分别下发 MoveP 命令：

| 控制对象 | 命令话题 | 结果话题 |
| -------- | -------- | -------- |
| 左臂 | `/onero_arm/left_arm/movep` | `/onero_arm/left_arm/movep_result` |
| 右臂 | `/onero_arm/right_arm/movep` | `/onero_arm/right_arm/movep_result` |

双臂模式下的单臂 MoveP 不经过 `/onero_arm/dual_arm` 同步调度器，适用于只需要单侧机械臂执行笛卡尔点到点运动的场景。

**示例**:

```bash
# 单臂模式：A1-L（a1_l_driver.launch.py）
ros2 topic pub --once /onero_arm/movep onero_interfaces/msg/MoveP "{
  pose: {
    position: {x: -0.4786, y: 0.2144, z: 0.2552},
    orientation: {x: -0.6082, y: -0.2328, z: 0.3607, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 单臂模式：A1-R（a1_r_driver.launch.py）
ros2 topic pub --once /onero_arm/movep onero_interfaces/msg/MoveP "{
  pose: {
    position: {x: -0.4785, y: 0.2145, z: 0.2559},
    orientation: {x: -0.6082, y: -0.2327, z: 0.3608, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 双臂模式：控制左臂执行 MoveP
ros2 topic pub --once /onero_arm/left_arm/movep onero_interfaces/msg/MoveP "{
  pose: {
    position: {x: -0.4786, y: 0.2144, z: 0.2552},
    orientation: {x: -0.6082, y: -0.2328, z: 0.3607, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"

# 双臂模式：控制右臂执行 MoveP
ros2 topic pub --once /onero_arm/right_arm/movep onero_interfaces/msg/MoveP "{
  pose: {
    position: {x: -0.4785, y: 0.2145, z: 0.2559},
    orientation: {x: -0.6082, y: -0.2327, z: 0.3608, w: 0.6677}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"
```

---

### DualMoveJ - 双臂同步关节运动

**功能**: 在双臂模式下同时提交左右臂关节目标，由双臂调度器统一估算两臂运动时间、协调较短运动臂的速度，并同步启动左右臂 MoveJ 执行。

**话题名称**: `/onero_arm/dual_arm/movej`

**消息类型**: `onero_interfaces/msg/DualMoveJ`

**消息定义**:

```msg
float32[7] left_joint        # 左臂目标关节角度（弧度）
float32[7] right_joint       # 右臂目标关节角度（弧度）
float32 speed_scale          # 基准速度缩放因子 (0.01~5.0)
```

**参数说明**:

| 参数 | 类型 | 说明 | 范围 |
| ---- | ---- | ---- | ---- |
| left_joint | float32[7] | 左臂目标关节角度（弧度） | 7轴关节 |
| right_joint | float32[7] | 右臂目标关节角度（弧度） | 7轴关节 |
| **speed_scale** | **float32** | **双臂基准速度缩放因子**，调度器会在需要时降低较短运动臂速度以对齐完成时间 | 0.01~5.0 |

**返回**: 通过 `/onero_arm/dual_arm/movej_result` 发布 `onero_interfaces/msg/CommandResult`。若某一侧失败，`error_message` 会带有 `left_arm:` 或 `right_arm:` 前缀。

**同步状态**: 通过 `/onero_arm/dual_arm/sync_state` 发布 `onero_interfaces/msg/DualSyncState`。

```msg
builtin_interfaces/Time stamp
uint8 status                 # 0:空闲/完成 1:运动中 2:错误
bool left_done               # 左臂是否完成
bool right_done              # 右臂是否完成
int32 error_code             # 错误码
string error_message         # 错误描述
```

**停止双臂**: 向 `/onero_arm/dual_arm/stop` 发布 `std_msgs/msg/Bool` 且 `data=true`，驱动会同时取消左右臂当前轨迹并清空轨迹缓冲；结果通过 `/onero_arm/dual_arm/stop_result` 发布。

**注意**:

- 双臂同步 MoveJ 仅在双臂模式下可用。
- `/onero_arm/dual_arm/movej` 不包含 `trajectory_connect` 字段；如需分别缓存左右臂轨迹，请使用 `/onero_arm/left_arm/movej` 与 `/onero_arm/right_arm/movej`。
- 双臂同步命令要求左右臂均处于空闲状态；任意一侧正忙时会返回 `BUSY`。

**示例**:

```bash
# 双臂同步 MoveJ
ros2 topic pub --once /onero_arm/dual_arm/movej onero_interfaces/msg/DualMoveJ "{
  left_joint: [0.0, 0.2, 0.0, -0.7, 0.0, 0.0, 0.0],
  right_joint: [0.0, -0.2, 0.0, -0.7, 0.0, 0.0, 0.0],
  speed_scale: 0.3
}"

# 查看双臂同步状态
ros2 topic echo /onero_arm/dual_arm/sync_state

# 停止双臂运动
ros2 topic pub --once /onero_arm/dual_arm/stop std_msgs/msg/Bool "{data: true}"
```

---

## 状态接口

### 机械臂状态发布

**话题名称**: `<arm_ns>/arm_state`

**消息类型**: `onero_interfaces/msg/ArmState`

**发布频率**: 100Hz

**消息内容**:

```msg
std_msgs/Header header               # 时间戳与坐标系
string robot_model                   # 机械臂型号
float32[] joint_positions            # 当前关节位置（弧度）
float32[] joint_velocities           # 当前关节速度（rad/s）
float32[] joint_torques              # 当前关节力矩（N·m）
geometry_msgs/Pose end_effector_pose # 当前末端位姿
uint8 status                         # 机械臂状态 (0:空闲 1:运动中 2:错误)
int32 error_code                     # 错误码
string error_message                 # 错误描述
```

**示例**:

```bash
# 单臂模式
ros2 topic echo /onero_arm/arm_state

# 双臂模式
ros2 topic echo /onero_arm/left_arm/arm_state
ros2 topic echo /onero_arm/right_arm/arm_state
```

---

**文档版本**: V1.1
**最后更新**: 2026-6-18
**维护者**: OneroArm Team
