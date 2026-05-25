<div align="center">

# OneroArm ROS2 机械臂控制系统

版本：1.0.0


| 版本号 |   时间   | 说明  |
| :----: | :-------: | :---- |
|  V1.0  | 2026-3-13 | Draft |

</div>

## 目录

* [项目概述](#1-项目概述)
* [环境准备](#2-环境准备)
* [编译与加载](#3-编译与加载)
* [功能包组成](#4-功能包组成)
* [示例运行方式](#5-示例运行方式)
* [安全说明](#6-安全说明)
* [开源许可说明](#7-开源许可说明)

## 1. 项目概述

本仓库提供 OneroArm 机械臂在 ROS2 环境下的完整软件支持，覆盖真实硬件控制、MoveIt2 规划、Gazebo 仿真、消息接口定义、示例程序以及安装部署流程。

当前支持的机械臂型号包括：

- A1-L
- A1-R
- A1-Dual

推荐运行环境如下：

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2

本文档从整体角度介绍工作空间中的各个功能包，帮助用户完成环境搭建、系统编译和示例功能启动。

## 2. 环境准备

在使用本工程前，建议按以下顺序完成环境准备。

### 2.1 安装 ROS2

工程提供了 ROS2 安装脚本 `ros2_install.sh`，脚本位于 `src/onero_install/scripts/` 目录中。进入对应目录后执行：

```bash
sudo bash ros2_install.sh
```

如果不使用脚本，也可以参考 ROS2 官方安装说明：

- [ROS2_INSTALL](https://doc.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 2.2 安装 MoveIt2

工程提供了 MoveIt2 安装脚本 `moveit2_install.sh`，脚本位于 `src/onero_install/scripts/` 目录中。进入对应目录后执行：

```bash
sudo bash moveit2_install.sh
```

如果不使用脚本，也可以参考 MoveIt2 官方安装说明：

- [Moveit2_INSTALL](https://moveit.ros.org/install-moveit2/binary/)

### 2.3 安装依赖与运行库

运行库安装脚本 `lib_install.sh` 位于 `src/onero_driver/lib/` 目录中。进入对应目录后执行：

```bash
sudo bash lib_install.sh
```

### 2.4 配置串口权限

真实机械臂控制依赖串口访问权限。建议执行以下命令之一：

```bash
# 将当前用户加入 dialout 组（重新登录后生效）
sudo usermod -aG dialout $USER

# 临时切换到 dialout 组（当前会话立即生效）
newgrp dialout
```

## 3. 编译与加载

环境准备完成后，可在新的工作空间中编译本工程。

```bash
mkdir -p ~/oneroarm_ws/src
cp -r OneroArm ~/oneroarm_ws/src
cd ~/oneroarm_ws
source /opt/ros/humble/setup.bash
colcon build
source ./install/setup.bash
```

当编译完成且 `install/setup.bash` 加载成功后，即可使用各功能包提供的启动文件和示例程序。

## 4. 功能包组成

本工程由以下九个功能包构成。

### 4.1 `onero_driver`

真实机械臂的 ROS2 驱动层。该功能包负责与硬件通信，并提供 MoveJ、MoveL、MoveP 等运动接口及状态反馈。

### 4.2 `onero_bringup`

系统级启动管理包。该功能包将模型发布、驱动、MoveIt2 和仿真等多个子系统进行编排，提供一键式启动入口。

### 4.3 `onero_description`

机械臂模型描述包。该功能包提供 URDF/Xacro、SRDF、Mesh 资源以及模型发布相关启动脚本。

### 4.4 `onero_interfaces`

ROS2 消息接口定义包。该功能包提供 MoveJ、MoveL、MoveP、ArmState 等消息类型。

### 4.5 `moveit_config`

MoveIt2 配置包集合。该目录下包含不同机械臂型号对应的规划、控制器、RViz 和语义模型配置。

### 4.6 `onero_control`

MoveIt2 与真实驱动之间的桥接控制包。该功能包将 MoveIt2 轨迹离散化后发送给 `onero_driver` 执行。

### 4.7 `onero_gazebo`

Gazebo 仿真支持包。该功能包用于在仿真环境中加载机械臂模型，并配合 MoveIt2 完成虚拟机械臂规划验证。

### 4.8 `onero_examples`

示例程序集合。该功能包提供 MoveJ、MoveL、MoveP、轨迹连接、状态监控、拖动示教、力位控制等示例。

### 4.9 `onero_install`

安装与部署辅助包。该功能包用于说明依赖安装、环境搭建和编译部署流程。

## 5. 示例运行方式

### 5.1 运行仿真机械臂

若需要启动 Gazebo 仿真和 MoveIt2 规划，可执行：

```bash
ros2 launch onero_bringup <arm_type>_gazebo_bringup.launch.py
```

其中 `<arm_type>` 支持以下取值：

- `a1_l`
- `a1_r`
- `a1_dual`

以 A1-L 为例：

```bash
ros2 launch onero_bringup a1_l_gazebo_bringup.launch.py
```

启动成功后，可在 Gazebo 与 RViz 中对虚拟机械臂进行规划和控制。

### 5.2 控制真实机械臂

若需要启动真实机械臂驱动并接入 MoveIt2，可执行：

```bash
ros2 launch onero_bringup <arm_type>_moveit_bringup.launch.py
```

其中 `<arm_type>` 支持以下取值：

- `a1_l`
- `a1_r`
- `a1_dual`

以 A1-L 为例：

```bash
ros2 launch onero_bringup a1_l_moveit_bringup.launch.py
```

启动成功后，即可在 RViz 中完成轨迹规划，并驱动真实机械臂执行。

## 6. 安全说明

在真实机械臂运行前，请确认以下事项：

- 检查机械臂底座、安装螺丝和工作台是否牢固可靠。
- 确保机械臂运动范围内无人停留，也无其他障碍物。
- 首次运行建议使用较低速度并预留急停手段。
- 设备闲置时应将机械臂置于安全位，并及时断开电源。

## 7. 开源许可说明

Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

本工程自有代码采用 **MIT License**（全部 12 个 ROS2 功能包统一）。

完整合规文件位于本目录下的 `LICENSES/`：

| 文件 | 内容 |
| ---- | ---- |
| `LICENSES/LICENSE` | MIT License 完整正文 |
| `LICENSES/NOTICE` | 第三方组件汇总 + 重分发义务 |
| `LICENSES/third_party/` | 各第三方组件的完整许可证文本 |

工程随 `onero_driver` 分发 `libonero_api.so` 及其运行所需的第三方组件。**第三方组件不属于本工程 MIT License 授权范围**，应继续遵守其各自上游许可。

| 组件 | 上游许可 |
| ---- | -------- |
| RBDL | zlib License |
| Pinocchio | BSD 2-Clause（含 Boost 1.0 子组件） |
| hpp-fcl | BSD License |

> ⚠️ **LGPL-3.0 提示**：hpp-fcl 中的 `hpp/fcl/collision_utility.h` 头文件采用 GNU LGPL-3.0-or-later。重分发本产品时，必须随附 `LICENSES/third_party/hpp-fcl.LGPL-3.0.txt`，并提供 hpp-fcl 上游源码获取方式（https://github.com/humanoid-path-planner/hpp-fcl ）。`libonero_api.so` 通过动态链接调用该组件，符合 LGPL 动态链接条件。

各功能包的随附第三方运行库文件清单见 `onero_driver/lib/README.md`。
