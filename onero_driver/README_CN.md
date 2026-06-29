<div align="center">

# OneroArm 硬件驱动说明

版本：V1.0


| 版本号 |   时间   | 说明  |
| :----: | :-------: | :---- |
|  V1.0  | 2026-3-13 | Draft |

</div>

## 目录

* [功能说明](#1-功能说明)
* [使用说明](#2-使用说明)
* [目录结构](#3-目录结构)
* [话题接口说明](#4-话题接口说明)
* [常见问题](#5-常见问题)
* [开源许可说明](#6-开源许可说明)

## 1. 功能说明

`onero_driver` 是 OneroArm 机械臂的 ROS2 驱动层核心模块，负责在 ROS2 系统与硬件控制器之间建立通信和控制接口。

该功能包的核心职责包括：

- 接收并处理 MoveJ、MoveL、MoveP 等 ROS2 控制命令。
- 将上层控制指令映射转换为底层硬件执行指令。
- 定期采集机械臂运动状态信息并发布至系统其他模块。
- 支持 A1-L、A1-R 单臂模式，以及 A1-Dual 双臂模式。
- 作为 MoveIt 规划执行链的最终执行单元，接收规划轨迹并驱动机械臂执行相应动作。

本文档通过以下三个方面帮助您快速上手：

* 功能包使用。
* 功能包架构说明。
* 功能包话题说明。

附：运动接口详细说明请参考 [doc/Move_API_CN.md](doc/Move_API_CN.md)。

## 2. 使用说明

### 2.1 启动方式

默认情况下，单臂配置使用串口 `/dev/ttyACM0`；双臂配置使用左臂 `/dev/ttyACM0`、右臂 `/dev/ttyACM1`。如果实际设备号不同，请修改 `config/` 目录下对应的 YAML 配置文件。

启动命令如下：

```bash
ros2 launch onero_driver <arm_type>_driver.launch.py
```

支持的机械臂型号标识：

- `a1_l`
- `a1_r`
- `a1_dual`

常用启动命令如下：

```bash
# A1-L 单臂
ros2 launch onero_driver a1_l_driver.launch.py

# A1-R 单臂
ros2 launch onero_driver a1_r_driver.launch.py

# A1-Dual 双臂
ros2 launch onero_driver a1_dual_driver.launch.py
```

底层驱动正常启动后，可看到如下界面。
![image](doc/onero_driver1.png)


- 单臂模式：话题为 `/onero_arm/<动作>`，例如 `/onero_arm/movej`。
- 双臂模式：左臂为 `/onero_arm/left_arm/<动作>`，右臂为 `/onero_arm/right_arm/<动作>`，双臂同步控制为 `/onero_arm/dual_arm/<动作>`。

### 2.2 仿真与可视化

启动文件提供 `sim` 和 `rviz` 两个布尔参数，用于切换仿真模式和 RViz 可视化。两者默认关闭，仅作为 launch 启动参数存在，可单独或组合使用。

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `sim` | `false` | `true` 时进入仿真模式 |
| `rviz` | `false` | `true` 时启动 rviz2 可视化，真机与仿真模式均可使用 |

常用启动命令如下：

```bash
# 真机 + RViz 可视化
ros2 launch onero_driver a1_l_driver.launch.py rviz:=true

# 纯仿真，无需连接硬件
ros2 launch onero_driver a1_l_driver.launch.py sim:=true

# 仿真 + RViz
ros2 launch onero_driver a1_l_driver.launch.py sim:=true rviz:=true
```

启动后由 `robot_state_publisher` 加载 URDF 并广播 TF，RViz 订阅 `/joint_states` 实时显示机械臂姿态。

### 2.3 配置文件

各启动文件会自动加载对应的配置文件。

| 启动文件 | 配置文件 | 说明 |
| -------- | -------- | ---- |
| `a1_l_driver.launch.py` | `config/a1_l_driver.yaml` | A1-L 单臂，默认串口 `/dev/ttyACM0` |
| `a1_r_driver.launch.py` | `config/a1_r_driver.yaml` | A1-R 单臂，默认串口 `/dev/ttyACM0` |
| `a1_dual_driver.launch.py` | `config/a1_dual_driver.yaml` | A1-Dual 双臂，默认左臂 `/dev/ttyACM0`、右臂 `/dev/ttyACM1` |

常用参数说明如下：

| 参数 | 说明 |
| ---- | ---- |
| `robot_model` | 机械臂型号。单臂使用 `a1_l` 或 `a1_r`，双臂使用 `A1_dual`。`A1_dual` 内部固定按左臂 `a1_l`、右臂 `a1_r` 初始化 |
| `device` | 单臂串口设备路径 |
| `device_left` / `device_right` | 双臂模式下左右臂串口设备路径 |
| `mount_orientation` | 机械臂摆放方向，可选 `horizontal` 或 `vertical` |
| `state_pub_rate` | 状态发布频率，默认 100Hz |
| `mit_mode` | MIT 力控模式下的关节 PD 参数 |

## 3. 目录结构

`onero_driver` 功能包结构如下：

```text
onero_driver/
├── CMakeLists.txt                 # 构建脚本
├── package.xml                    # 包依赖声明
├── include/
│   ├── onero_define.h             # 底层 SDK 公共定义
│   ├── onero_interface_cpp.h      # 底层 SDK C++ 接口
│   └── robot_driver/
│       └── onero_driver.h         # ROS 驱动头文件
├── src/
│   └── onero_driver.cpp            # OneroDriver 类实现
├── config/
│   ├── a1_l_driver.yaml           # A1-L 配置
│   ├── a1_r_driver.yaml           # A1-R 配置
│   └── a1_dual_driver.yaml        # A1-Dual 配置
├── launch/
│   ├── a1_l_driver.launch.py      # A1-L 启动文件
│   ├── a1_r_driver.launch.py      # A1-R 启动文件
│   └── a1_dual_driver.launch.py   # A1-Dual 启动文件
├── doc/
│   └── Move_API_CN.md             # Move API 详细说明
├── lib/
│   ├── lib_install.sh             # 运行库安装脚本
│   ├── linux_aarch64_v1.0.0/      # ARM64 预编译运行库
│   ├── linux_riscv64_v1.0.0/      # RISC-V 预编译运行库
│   ├── linux_x86_64_v1.0.0/       # x86_64 预编译运行库
│   └── README.md                  # 第三方运行库许可说明
└── README_CN.md                   # 中文说明文档
```

## 4. 话题接口说明

### 4.1 订阅话题

单臂启动时，`<arm_ns>` 为 `/onero_arm`；双臂启动时，左右臂分别使用 `/onero_arm/left_arm` 和 `/onero_arm/right_arm` 作为话题前缀。双臂同步控制话题固定使用 `/onero_arm/dual_arm`。

| 话题名称 | 消息类型 | 频率 | 说明 |
| -------- | -------- | ---- | ---- |
| `<arm_ns>/movej` | `onero_interfaces/msg/MoveJ` | 事件触发 | 接收 MoveJ 关节运动命令 |
| `<arm_ns>/movel` | `onero_interfaces/msg/MoveL` | 事件触发 | 接收 MoveL 直线运动命令 |
| `<arm_ns>/movep` | `onero_interfaces/msg/MoveP` | 事件触发 | 接收 MoveP 位姿透传命令 |
| `<arm_ns>/stop` | `std_msgs/msg/Bool` | 事件触发 | `data=true` 时停止当前运动并清除轨迹缓存 |
| `/onero_arm/dual_arm/movej` | `onero_interfaces/msg/DualMoveJ` | 事件触发 | 双臂模式下的 MoveJ 同步运动命令 |
| `/onero_arm/dual_arm/stop` | `std_msgs/msg/Bool` | 事件触发 | `data=true` 时停止双臂当前运动并清除轨迹缓存 |
| `/onero_arm/moveit/trajectory` | `trajectory_msgs/msg/JointTrajectory` | 事件触发 | 接收 MoveIt 完整规划轨迹 |
| `/onero_arm/moveit/execute` | `std_msgs/msg/Empty` | 事件触发 | 接收 MoveIt 执行触发 |
| `/onero_arm/moveit/cancel` | `std_msgs/msg/Empty` | 事件触发 | 接收 MoveIt 取消触发 |
| `/onero_arm/moveit/planning_arms_status` | `std_msgs/msg/String` | 事件触发 | 接收 MoveIt 当前规划对象，可为 `left`、`right` 或 `dual` |

### 4.2 发布话题

可通过如下方式查看驱动节点的话题信息。

![image](doc/onero_driver3.png)


| 话题名称 | 消息类型 | 频率 | 说明 |
| -------- | -------- | ---- | ---- |
| `<arm_ns>/movej_result` | `onero_interfaces/msg/CommandResult` | 事件触发 | 发布 MoveJ 执行结果 |
| `<arm_ns>/movel_result` | `onero_interfaces/msg/CommandResult` | 事件触发 | 发布 MoveL 执行结果 |
| `<arm_ns>/movep_result` | `onero_interfaces/msg/CommandResult` | 事件触发 | 发布 MoveP 执行结果 |
| `<arm_ns>/stop_result` | `onero_interfaces/msg/CommandResult` | 事件触发 | 发布停止命令下发结果 |
| `<arm_ns>/arm_state` | `onero_interfaces/msg/ArmState` | 100Hz | 发布机械臂完整状态 |
| `/onero_arm/dual_arm/movej_result` | `onero_interfaces/msg/CommandResult` | 事件触发 | 发布双臂 MoveJ 聚合结果 |
| `/onero_arm/dual_arm/stop_result` | `onero_interfaces/msg/CommandResult` | 事件触发 | 发布双臂停止命令下发结果 |
| `/onero_arm/dual_arm/sync_state` | `onero_interfaces/msg/DualSyncState` | 执行中周期发布 | 发布双臂同步执行状态 |
| `/onero_arm/moveit/trajectory_execution_result` | `onero_interfaces/msg/CommandResult` | 事件触发 | 发布 MoveIt 轨迹执行结果 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 100Hz | 发布关节状态，供 MoveIt 和 RViz 使用 |

### 4.3 服务接口

| 服务名称 | 服务类型 | 说明 |
| -------- | -------- | ---- |
| `<arm_ns>/clear_buffer` | `std_srvs/srv/Trigger` | 清除未执行的轨迹连接缓存，不停止正在执行的运动 |
| `<arm_ns>/get_end_pose` | `onero_interfaces/srv/EndEffectorPose` | 查询当前末端位姿 |

### 4.4 结果码说明

运动命令和停止命令的结果均通过 `onero_interfaces/msg/CommandResult` 返回：

| 字段 | 说明 |
| ---- | ---- |
| `stamp` | 结果发布时间 |
| `success` | 是否执行成功 |
| `error_code` | 错误码，成功时为 `0` |
| `error_message` | 错误描述，成功时为空 |

常见错误码如下：

| 错误码 | 含义 |
| ------ | ---- |
| `0` | 成功 |
| `-1` | 参数无效 |
| `-2` | IK 求解失败 |
| `-3` | 检测到碰撞 |
| `-4` | 执行失败 |
| `-5` | 执行超时 |
| `-6` | 运动被中断 |
| `-7` | 关节限位超出 |
| `-8` | 该臂已有运动命令在执行 |

## 5. 常见问题

### 5.1 串口连接失败

**问题现象**：驱动节点无法连接机械臂，或启动后提示串口设备异常。

**常见原因**：

- 串口设备未正常连接。
- 当前设备路径与启动配置不一致。

对应现象示例如下。
![image](doc/onero_driver5.png)

**排查步骤**：

1. 确认配置文件中的串口路径是否与实际设备一致。单臂检查 `device`，双臂检查 `device_left` 和 `device_right`。

   ![image](doc/onero_driver2.png)
2. 枚举当前系统串口设备：

   ```bash
   ls /dev/tty*
   ```
   ![image](doc/onero_driver4.png)
3. 若未发现目标串口，建议重新插拔设备后再次确认。
4. 若串口名称与配置不一致，请修改 `src/onero_driver/config/` 目录下对应 YAML 文件中的串口参数，并重新编译部署。

### 5.2 找不到预期话题

**问题现象**：单臂启动后找不到 `/onero_arm/left_arm/movej` 或 `/onero_arm/right_arm/movej`。

**原因说明**：单臂模式不带 `left_arm` 或 `right_arm` 子前缀，话题直接位于 `/onero_arm` 下。

**排查步骤**：

1. 单臂启动后查看话题：

   ```bash
   ros2 topic list | grep onero_arm
   ```

   应看到 `/onero_arm/movej`、`/onero_arm/movej_result`、`/onero_arm/arm_state` 等话题。
2. 双臂启动后才会出现 `/onero_arm/left_arm/*`、`/onero_arm/right_arm/*` 和 `/onero_arm/dual_arm/*`。

## 6. 开源许可说明

`onero_driver` 自有代码采用 **MIT License**（详见 `../LICENSES/LICENSE`）。

Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

本功能包随源码及安装产物分发 `libonero_api.so`、第三方运行库和第三方头文件（位于 `include/third_party/`）。**这些第三方组件保留其上游许可，不随本功能包变更为 MIT License**。

| 组件 | 上游许可 |
| ---- | -------- |
| RBDL | zlib License |
| Pinocchio | BSD 2-Clause（含 Boost 1.0 子组件） |
| hpp-fcl | BSD License |

> ⚠️ **LGPL-3.0 提示**：`include/third_party/hpp/fcl/collision_utility.h` 头文件采用 GNU LGPL-3.0-or-later。详见 `../LICENSES/third_party/hpp-fcl.LGPL-3.0.txt`。

许可证完整正文与重分发义务见同级 `../LICENSES/` 目录：
- `../LICENSES/LICENSE` — MIT 全文
- `../LICENSES/NOTICE` — 第三方组件汇总
- `../LICENSES/third_party/` — 各第三方组件完整许可证

随包第三方 `.so` 文件清单见 `lib/README.md`。
