<div align="center">

# OneroArm Gazebo 仿真说明

版本：V1.0


| 版本号 |   时间   | 说明  |
| :----: | :-------: | :---- |
|  V1.0  | 2026-3-13 | Draft |

</div>

## 目录

* [功能说明](#1-功能说明)
* [使用说明](#2-使用说明)
* [目录结构](#3-目录结构)

## 1. 功能说明

`onero_gazebo` 是 OneroArm 的 Gazebo 仿真支持包，用于在无真实硬件的条件下构建机械臂的物理仿真环境。

该功能包主要用于：

- 在 Gazebo 中加载 OneroArm 仿真模型。
- 验证 MoveIt2 规划算法和控制流程。
- 评估轨迹可行性与安全性。

本文档将帮助您：

* 1.快速启动Gazebo仿真环境。
* 2.理解仿真配置文件的组织。
* 3.掌握仿真与真机的区别。

## 2. 使用说明

### 2.1 快速启动仿真环境

在完成 ROS2 和 Gazebo 依赖安装后，可通过以下命令启动仿真系统：

```bash
ros2 launch onero_gazebo <arm_type>_gazebo.launch.py
```

支持的型号标识：

- `a1_l`
- `a1_r`
- `a1_dual`

以 A1-L 为例：

```bash
ros2 launch onero_gazebo a1_l_gazebo.launch.py
```

运行成功后将出现 Gazebo 仿真界面。
![image](doc/onero_gazebo1.png)

### 2.2 配合 MoveIt2 控制仿真机械臂

在 Gazebo 启动完成后，可继续启动 MoveIt2 控制链路：

```bash
ros2 launch onero_bringup <arm_type>_moveit_bringup.launch.py
```

其中 `<arm_type>` 支持：

- `a1_l`
- `a1_r`
- `a1_dual`

以 A1-L 为例：

```bash
ros2 launch onero_bringup a1_l_moveit_bringup.launch.py
```

随后用户可以在 RViz 中拖拽机械臂至目标位姿，规划得到的轨迹会在 Gazebo 中执行并显示物理运动效果。
![image](doc/onero_gazebo2.png)

## 3. 目录结构

`onero_gazebo` 功能包结构如下：

```text
onero_gazebo/
├── CMakeLists.txt                              # 构建脚本
├── config/                                     # 仿真配置目录
│   ├── a1_l_gazebo_description.urdf.xacro      # A1-L 仿真模型
│   ├── a1_r_gazebo_description.urdf.xacro      # A1-R 仿真模型
│   └── a1_dual_gazebo_description.urdf.xacro   # A1-Dual 仿真模型
├── launch/                                     # 启动文件目录
│   ├── a1_l_gazebo.launch.py                   # A1-L 仿真启动
│   ├── a1_r_gazebo.launch.py                   # A1-R 仿真启动
│   └── a1_dual_gazebo.launch.py                # A1-Dual 仿真启动
├── package.xml                                 # 包依赖和元信息
└── README_CN.md                                # 中文说明文档
```
