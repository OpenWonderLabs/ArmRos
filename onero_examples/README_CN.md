<div align="center">

# OneroArm 示例程序说明

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

`onero_examples` 提供 OneroArm 常用功能的示例程序，涵盖基础运动控制、轨迹连接、状态监控和拖动示教等典型应用场景。

当前包含的示例包括：

- `movej_demo`：关节空间运动示例
- `movel_demo`：笛卡尔空间直线运动示例
- `movep_demo`：位姿透传示例
- `move_all`：所有 Move API 顺序执行示例（C++）
- `move_api_all_py`：所有 Move API 顺序执行示例（Python）
- `trajectory_connect_demo`：轨迹连接演示（C++）
- `trajectory_connect_demo.py`：轨迹连接演示（Python）
- `state_monitor`：机械臂状态监控示例
- `drag_teaching_node`：拖动示教功能示例
- `dual_arm_demo`：双臂综合控制示例
- `dual_arm_drag_teaching_node`：双臂拖动示教功能示例

## 2. 使用说明

除特别说明外，运行各示例前都应先启动 `onero_driver`。

通用启动命令如下：

```bash
ros2 launch onero_driver <arm_type>_driver.launch.py
```

支持的机械臂型号标识：

- `a1_l`
- `a1_r`
- `a1_dual`

单臂示例默认连接扁平话题 `/onero_arm/*`，适用于 `a1_l` 或 `a1_r` 单臂启动。双臂启动时左右臂话题仍保留 `/onero_arm/left_arm/*` 与 `/onero_arm/right_arm/*`，请使用 `dual_arm_demo` 演示双臂同步与单臂独立控制。

### 2.1 MoveJ 运动示例

运行命令：

```bash
ros2 run onero_examples movej_demo
```

执行成功后，机械臂将运动到指定关节角度。
![image](doc/onero_examples1.png)

相关话题：

- 命令话题：`/onero_arm/movej`
- 结果话题：`/onero_arm/movej_result`

### 2.2 MoveL 运动示例

运行命令：

```bash
ros2 run onero_examples movel_demo
```

执行成功后，机械臂将以笛卡尔直线方式运动到目标位姿。
![image](doc/onero_examples2.png)

### 2.3 MoveP 运动示例

运行命令：

```bash
ros2 run onero_examples movep_demo
```

执行成功后，机械臂将运动到指定位姿。
![image](doc/onero_examples4.png)

补充说明：

- OneroArm 的 MoveP 通过 IK 转换为关节运动，适用于单次位姿控制。
- `speed_scale` 建议设置在 `0.1~1.0` 区间内。

### 2.4 Move All 顺序执行示例

该示例用于顺序执行 MoveJ → MoveL → MoveP。

运行命令如下：

```bash
# C++ 版本
ros2 run onero_examples move_all

# Python 版本
ros2 run onero_examples move_api_all_py
```

执行成功后，系统会在前一个动作完成后再执行下一个动作，以保证执行顺序明确。
![image](doc/onero_examples5.png)

### 2.5 轨迹连接示例

`trajectory_connect` 参数用于将多段轨迹平滑地拼接为一个整体动作：

- 值为 `1`：缓存当前轨迹点，命令立即返回，但机械臂暂不执行。
- 值为 `0`：触发执行所有已缓存轨迹点与当前点，形成连续运动。

典型使用场景包括：

- 多路径点平滑经过
- 减少每个路径点处的停顿
- 在关节空间对多段轨迹进行统一规划

运行命令如下：

```bash
# C++ 版本
ros2 run onero_examples trajectory_connect_demo

# Python 版本
ros2 run onero_examples trajectory_connect_demo_py
```

运行后将以 MoveJ → MoveL → MoveP 的顺序演示混合轨迹连接效果。
![image](doc/onero_examples6.png)

### 2.6 状态监控示例

运行命令：

```bash
ros2 run onero_examples state_monitor
```

节点运行后会实时输出机械臂状态信息。
![image](doc/onero_examples7.png)

### 2.7 拖动示教示例

运行命令：

```bash
ros2 run onero_examples drag_teaching_node
```

节点启动后会等待命令输入，并提供以下能力：

- 零力拖动记录：进入拖动模式后记录关节位置、速度和力矩
- 轨迹回放：读取记录文件，先回到起点，再回放整段轨迹

![image](doc/onero_examples8.png)

### 2.8 双臂综合示例

运行命令：

```bash
# 终端 1
ros2 launch onero_driver a1_dual_driver.launch.py

# 终端 2
ros2 run onero_examples dual_arm_demo
```

该示例仅用于双臂模式，执行流程为：

1. 发布 `/onero_arm/dual_arm/movej`，演示双臂同步 MoveJ。
2. 发布 `/onero_arm/left_arm/movej`，演示双臂模式下单独控制左臂。
3. 发布 `/onero_arm/right_arm/movej`，演示双臂模式下单独控制右臂。

### 2.9 双臂拖动示教示例

运行命令：

```bash
# 终端 1
ros2 launch onero_driver a1_dual_driver.launch.py

# 终端 2
ros2 run onero_examples dual_arm_drag_teaching_node
```

该示例仅用于双臂模式，命令集与单臂版一致，但每个命令同步作用于左右两臂。

## 3. 目录结构

`onero_examples` 功能包结构如下：

```text
onero_examples/
├── CMakeLists.txt                      # 编译规则文件
├── package.xml                         # 功能包配置文件
├── README_CN.md                        # 中文说明文档
└── src/
    ├── drag_teaching_node.cpp          # 拖动示教示例
    ├── dual_arm_demo.cpp               # 双臂综合控制示例
    ├── dual_arm_drag_teaching_node.cpp # 双臂零力拖动示教示例
    ├── move_api_all.cpp                # Move API 顺序执行示例
    ├── move_api_all.py                 # Move API 顺序执行示例（Python）
    ├── movej_demo.cpp                  # MoveJ 示例
    ├── movel_demo.cpp                  # MoveL 示例
    ├── movep_demo.cpp                  # MoveP 示例
    ├── state_monitor.cpp               # 状态监控示例
    ├── trajectory_connect_demo.cpp     # 轨迹连接示例
    └── trajectory_connect_demo.py      # 轨迹连接示例（Python）
```
