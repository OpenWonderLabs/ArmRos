<div align="center">

# OneroArm 消息接口说明

版本：V1.0

| 版本号 | 时间 | 说明 |
| :---: | :---: | :--- |
| V1.0 | 2026-3-13 | Draft |

</div>

## 目录

* [功能说明](#1-功能说明)
* [消息类型说明](#2-消息类型说明)
* [目录结构](#3-目录结构)

## 1. 功能说明

`onero_interfaces` 是 OneroArm 机械臂的 ROS2 消息定义包，提供了所有运动控制相关的ROS2消息类型。

从系统接口角度看，它解决的是“各模块之间以什么消息格式进行通信”的问题。当前主要包含以下消息：

- `MoveJ.msg`
- `MoveL.msg`
- `MoveP.msg`
- `ArmState.msg`

这些消息类型被驱动、示例程序、规划桥接模块及上层应用共同使用。

在下文中将通过以下几个方面详细介绍该功能包。
* 1.功能包使用。  
* 2.功能包话题说明。    
* 3.功能包架构说明。

## 2. 消息类型说明

### 2.1 `MoveJ.msg`

用于描述关节空间运动命令。

```msg
# 目标关节角度（弧度）
float32[] joint

# 轨迹连接标志
# 0: 立即规划并执行
# 1: 与下一条轨迹一起规划，不立即执行
uint8 trajectory_connect

# 速度缩放因子（推荐使用）
# - < 1.0: 降速更平滑
# - = 1.0: 正常速度（默认）
# - > 1.0: 提升速度
float32 speed_scale
```

### 2.2 `MoveL.msg`

用于描述笛卡尔空间直线运动命令。

```msg
# 目标位姿
geometry_msgs/Pose pose

# 轨迹连接标志
uint8 trajectory_connect

# 速度缩放因子 (0.1~5.0)
float32 speed_scale
```

### 2.3 `MoveP.msg`

用于描述位姿透传控制命令。

```msg
# 目标位姿
geometry_msgs/Pose pose

# 轨迹连接标志
# 0: 立即规划并执行
# 1: 与下一条轨迹一起规划，不立即执行
uint8 trajectory_connect

# 速度缩放因子 (0.05~5.0)
float32 speed_scale
```

### 2.4 `ArmState.msg`

用于发布机械臂的完整状态信息。

```msg
# 时间戳
builtin_interfaces/Time stamp

# 机械臂型号
string robot_model

# 当前关节位置（弧度）
float32[] joint_positions

# 当前关节速度（rad/s）
float32[] joint_velocities

# 当前关节力矩（N·m）
float32[] joint_torques

# 当前末端位姿
geometry_msgs/Pose end_effector_pose

# 机械臂状态
# 0: 空闲
# 1: 运动中
# 2: 错误
uint8 status

# 错误码（status=2时有效）
int32 error_code

# 错误描述
string error_message
```

典型使用场景：实时获取机械臂的关节状态、末端位姿、运行状态和错误信息。

## 3. 目录结构

`onero_interfaces` 功能包结构如下：

```text
onero_interfaces/
├── CMakeLists.txt          # 构建脚本
├── package.xml             # 包依赖声明
├── msg/                    # 消息定义目录
│   ├── MoveJ.msg           # 关节运动消息
│   ├── MoveL.msg           # 直线运动消息
│   ├── MoveP.msg           # 位姿透传消息
│   └── ArmState.msg        # 状态消息
└── README_CN.md            # 中文说明文档
```