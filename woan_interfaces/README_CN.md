<div align="center">

# WoanArm机器人woan_interfaces使用说明书V1.0

文件修订记录：

| 版本号| 时间   | 备注  | 
| :---: | :-----: | :---: |
|V1.0    |2025-10-23  |拟制 |

</div>

## 目录
* 1.[woan_interfaces功能包说明](#woan_interfaces功能包说明)
* 2.[woan_interfaces消息类型](#woan_interfaces消息类型)
* 3.[woan_interfaces功能包架构说明](#woan_interfaces功能包架构说明)

## woan_interfaces功能包说明
woan_interfaces功能包是WoanArm机械臂Move API的消息定义包，提供了所有运动控制相关的ROS2消息类型。

在下文中将通过以下几个方面详细介绍该功能包。
* 1.功能包使用。  
* 2.功能包话题说明。    
* 3.功能包架构说明。


## woan_interfaces消息类型

### MoveJ.msg - 关节空间运动

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


### MoveL.msg - 笛卡尔空间直线运动

```msg
# 目标位姿
geometry_msgs/Pose pose

# 轨迹连接标志
uint8 trajectory_connect

# 速度缩放因子 (0.1~5.0)
float32 speed_scale
```


### MoveP.msg - 位姿透传

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

### ArmState.msg - 机械臂状态

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

**使用场景**: 实时获取机械臂的完整状态信息。

## woan_interfaces功能包架构说明

### 功能包文件总览

```
woan_interfaces/
├── CMakeLists.txt          # 编译配置文件
├── package.xml             # 包依赖声明
├── msg/                    # 消息定义文件夹
│   ├── MoveJ.msg           # 关节运动消息
│   ├── MoveL.msg           # 直线运动消息
│   ├── MoveP.msg           # 位姿透传消息
│   └── ArmState.msg        # 状态消息
└── README_CN.md            # 中文说明文档
```