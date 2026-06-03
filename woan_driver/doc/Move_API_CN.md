# WoanArm Move API 使用文档

**版本**: V1.0
**日期**: 2025-10-23

---

## 目录

- [WoanArm Move API 使用文档](#woanarm-move-api-使用文档)
  - [目录](#目录)
  - [概述](#概述)
  - [API接口](#api接口)
    - [MoveJ - 关节空间运动](#movej---关节空间运动)
    - [MoveL - 笛卡尔空间直线运动](#movel---笛卡尔空间直线运动)
    - [MoveP - 位姿透传](#movep---位姿透传)
  - [状态接口](#状态接口)
    - [机械臂状态发布](#机械臂状态发布)

---

## 概述

WoanArm Move API提供了三种主要的运动控制接口：

1. **MoveJ**: 关节空间运动，控制机械臂各关节按指定角度运动
2. **MoveL**: 笛卡尔空间直线运动，控制末端执行器沿直线路径运动
3. **MoveP**: 位姿透传，用于高频实时跟随

所有API通过ROS2话题接口提供，支持非阻塞和阻塞两种模式。

---

## API接口

### MoveJ - 关节空间运动

**功能**: 控制机械臂在关节空间运动到目标关节角度。

**话题名称**: `~/movej_cmd`

**消息类型**: `woan_interfaces/msg/MoveJ`

**消息定义**:

```msg
float32[] joint              # 目标关节角度（弧度）
uint8 trajectory_connect     # 轨迹连接标志 (0:立即执行 1:与下一条轨迹连接)
float32 speed_scale          # 速度缩放因子 (0.01~5.0, 默认1.0)
```

**参数说明**:

| 参数 | 类型 | 说明 | 范围 |
|------|------|------|------|
| joint | float32[] | 目标关节角度（弧度） | 取决于机械臂型号 |
| trajectory_connect | uint8 | 轨迹连接：0-立即执行，1-与下一条连接 | 0或1 |
| **speed_scale** | **float32** | **速度缩放因子（推荐使用）** <1.0:降速 =1.0:正常 >1.0:加速 | 0.01~5.0 |

**trajectory_connect 轨迹连接使用说明**：

- **值为 1**：当前轨迹被缓存，命令立即返回成功（result=true），但机械臂不会立即运动
- **值为 0**：执行所有缓存的轨迹 + 当前轨迹，作为一个连续平滑的整体运动
- **典型用法**：发送 N-1 条 trajectory_connect=1 的命令缓存路径点，最后发送 1 条 trajectory_connect=0 触发执行
- **平滑性**：多条轨迹将在关节空间统一规划，保证速度和加速度的平滑过渡
- **清除缓冲**：异常情况下可调用服务 `/arm_control/clear_trajectory_buffer` 清除已缓存的轨迹

**返回**: 通过 `~/movej_result` 话题发布 `std_msgs/msg/Bool`，`true`表示成功，`false`表示失败。

**示例**:

```bash
# 使用speed_scale（推荐）
ros2 topic pub --once /woan_driver/movej_cmd woan_interfaces/msg/MoveJ "{
  joint: [0.0, 0.0, 0.0, -0.7, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 0,
}"

# 轨迹连接示例：缓冲多个点，最后一次触发执行
# 第1个点：缓冲（立即返回）
ros2 topic pub --once /woan_driver/movej_cmd woan_interfaces/msg/MoveJ "{
  joint: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 1,
}"

# 第2个点：缓冲（立即返回）
ros2 topic pub --once /woan_driver/movej_cmd woan_interfaces/msg/MoveJ "{
  joint: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 1,
}"

# 第3个点：触发执行所有缓冲的轨迹（等待完成后返回）
ros2 topic pub --once /woan_driver/movej_cmd woan_interfaces/msg/MoveJ "{
  joint: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0],
  speed_scale: 0.3,
  trajectory_connect: 0
}"
```

---

### MoveL - 笛卡尔空间直线运动

**功能**: 控制机械臂末端执行器沿直线路径运动到目标位姿。

**话题名称**: `~/movel_cmd`

**消息类型**: `woan_interfaces/msg/MoveL`

**消息定义**:

```msg
geometry_msgs/Pose pose      # 目标位姿
uint8 trajectory_connect     # 轨迹连接标志
float32 speed_scale          # 速度缩放因子 (0.01~5.0)
```

**参数说明**:

| 参数 | 类型 | 说明 | 范围 |
|------|------|------|------|
| pose | Pose | 目标位姿（位置+姿态） | - |
| trajectory_connect | uint8 | 轨迹连接标志 | 0或1 |
| **speed_scale** | **float32** | **速度缩放因子** <1.0:降速 =1.0:正常 >1.0:加速 | 0.01~5.0 |

**pose字段说明**:

- `position`: (x, y, z) - 目标位置，单位：米
- `orientation`: (w, x, y, z) - 目标姿态（四元数）

**返回**: 通过 `~/movel_result` 话题发布 `std_msgs/msg/Bool`。

**示例**:

```bash
# 示例
ros2 topic pub --once /woan_driver/movel_cmd woan_interfaces/msg/MoveL "{
  pose: {
    position: {x: 0.3, y: 0.2, z: 0.4},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  },
  speed_scale: 0.3,
  trajectory_connect: 0
}"
```

---

### MoveP - 位姿透传

**功能**: 高频实时位姿跟随，适用于视觉伺服等场景。

**话题名称**: `~/movep_cmd`

**消息类型**: `woan_interfaces/msg/MoveP`

**消息定义**:

```msg
geometry_msgs/Pose pose      # 目标位姿
uint8 trajectory_connect     # 轨迹连接标志
float32 speed_scale          # 速度缩放因子 (0.01~5.0)
```

**参数说明**:

| 参数 | 类型 | 说明 | 范围 |
|------|------|------|------|
| pose | Pose | 目标位姿 | - |
| trajectory_connect | uint8 | 轨迹连接：0-立即执行，1-与下一条连接 | 0或1 |
| **speed_scale** | **float32** | **速度缩放因子** <1.0:降速 =1.0:正常 >1.0:加速 | 0.01~5.0 |

**注意**: MoveP适用于高频控制（≤10ms），要求低延迟通信。

---

## 状态接口

### 机械臂状态发布

**话题名称**: `~/arm_state`

**消息类型**: `woan_interfaces/msg/ArmState`

**发布频率**: 100Hz

**消息内容**:

```msg
builtin_interfaces/Time stamp        # 时间戳
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
ros2 topic echo /woan_driver/arm_state
```
---

**文档版本**: V1.0
**最后更新**: 2025-10-23
**维护者**: WoanArm Team

**更新日志**:

- V1.0 (2025-10-23): 初始版本

