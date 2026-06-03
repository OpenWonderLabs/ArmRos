<div align="right">

[中文简体](README_CN.md)|
[English](README.md)

</div>

# WoanArm ROS2 机械臂控制系统

该功能包的主要作用为提供 WoanArm 机械臂的 ROS2 支持，以下为使用环境。

* 当前支持的机械臂有 X1-L、X1-R、A1-L、A1-R、A1-Dual 系列，详细可参考 WoanArm 官方文档。
* 版本 1.0.0.
* 基于的 Ubuntu 版本为 22.04。
* ROS2 版本为 humble。

下面为功能包安装使用教程。

## 1.搭建环境

在使用功能包之前我们首先需要进行如下操作。

* 1.[安装ROS2](#安装ROS2)
* 2.[安装Moveit2](#安装Moveit2)
* 3.[配置功能包环境](#配置功能包环境)
* 4.[编译](#编译)

### 安装ROS2

---

我们提供了ROS2的安装脚本ros2_install.sh，该脚本位于woan_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。

```
sudo bash ros2_install.sh
```

如果不想使用脚本安装也可以参考官方网址 [ROS2_INSTALL](https://doc.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 进行安装。

### 安装Moveit2

---

我们提供了Moveit2的安装脚本moveit2_install.sh，该脚本位于woan_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。

```
sudo bash moveit2_install.sh
```

如果不想使用脚本安装也可以参考官方网址 [Moveit2_INSTALL](https://moveit.ros.org/install-moveit2/binary/)进行安装。

### 配置功能包环境

---

该脚本位于woan_driver功能包中的lib文件夹下，在实际使用时我们需要移动到该路径执行如下指令。

```
sudo bash lib_install.sh
```

**配置串口权限**：

```bash
# 将当前用户加入 dialout 组（需要重新启动生效）
sudo usermod -aG dialout $USER

# 或临时授权（立即生效）
newgrp dialout
```

### 编译

---

以上执行成功后，可以执行如下指令进行功能包编译，首先需要构建工作空间，并将功能包文件导入工作空间下的 src 文件夹下，之后使用 colcon build 指令进行编译。

```bash
mkdir -p ~/woanarm_ws/src
cp -r WoanArm ~/woanarm_ws/src
cd ~/woanarm_ws
source /opt/ros/humble/setup.bash
colcon build
source ./install/setup.bash
```

编译完成后即可进行功能包的运行操作。

## 2.功能运行

---

功能包简介

1. **硬件驱动** ([woan_driver](woan_driver/))

   * 该功能包为机械臂的 ROS2 驱动层功能包，其作用控制真实硬件，并提供 Move API 话题接口（MoveJ/MoveL/MoveP）。
2. **启动管理** ([woan_bringup](woan_bringup/))

   * 该功能包为机械臂的节点启动功能包，其作用为快速启动多节点复合的机械臂功能。
3. **模型描述** ([woan_description](woan_description/))

   * 该功能包为机械臂模型描述功能包，其作用为提供机械臂 URDF 模型文件和模型加载节点，并为其他功能包提供机械臂关节间的坐标变换关系。
4. **ROS消息接口** ([woan_interfaces](woan_interfaces/))

   * 该功能包为机械臂的消息文件功能包，其作用为提供机械臂适配 ROS2 的所有控制消息和状态消息（MoveJ/MoveL/MoveP/ArmState）。
5. **Moveit2配置** ([moveit_config](moveit_config/))

   * 该功能包为机械臂的 MoveIt2 适配功能包，其作用为适配和实现各系列机械臂的 MoveIt2 规划控制功能，主要包括虚拟机械臂控制和真实机械臂控制两部分控制功能。
6. **Moveit2与硬件驱动通信连接** ([woan_control](woan_control/))

   * 该功能包为 MoveIt2 规划系统和硬件驱动之间的通信连接功能包，主要功能为将 MoveIt2 的规划路径进行插值离散化（100Hz），然后通过 `/moveit_angle` 话题传递给 `woan_driver`执行。
7. **Gazebo仿真机械臂控制** ([woan_gazebo](woan_gazebo/))

   * 该功能包为 Gazebo 仿真机械臂功能包，主要功能为在 Gazebo 仿真环境中显示机械臂模型，可通过 MoveIt2 对仿真的机械臂进行规划控制。
8. **使用案例** ([woan_examples](woan_examples/))

   * 该功能包为机械臂的一些使用案例，主要功能为实现机械臂的一些基本的控制功能和运动功能的使用案例（MoveJ/MoveL/MoveP/轨迹连接）。
9. **安装与环境配置**([woan_install](woan_install/))

   * 该功能包为机械臂使用辅助功能包，主要作用为介绍功能包使用环境安装与搭建方式，功能包的依赖库安装和功能包编译方法。

以上为当前的九个功能包，每个功能包都有其独特的作用，详情请参考各功能包文件夹中的 README_CN.md 文档进行详细了解。

### 2.1 运行虚拟机械臂

---

使用如下指令可以启动 Gazebo 显示仿真机械臂，并同时启动 MoveIt2 进行仿真机械臂的规划操控。

```bash
ros2 launch woan_bringup <arm_type>_gazebo_bringup.launch.py
```

`<arm_type>` 需要使用 `x1_l`、`x1_r`、`a1_l`、`a1_r` 字符进行代替，如使用 X1-L 机械臂时，命令如下。

```bash
ros2 launch woan_bringup x1_l_gazebo_bringup.launch.py
```

启动成功后即可使用 MoveIt2 进行虚拟机械臂的控制。

### 2.2 控制真实机械臂

---

使用如下指令可以启动机械臂硬件驱动，并同时启动 MoveIt2 进行机械臂的规划操控。

```bash
ros2 launch woan_bringup <arm_type>_moveit_bringup.launch.py
```

`<arm_type>` 需要使用 `x1_l`、`x1_r`、`a1_l`、`a1_r` 字符进行代替，如使用 X1-L 机械臂时，命令如下。

```bash
ros2 launch woan_bringup x1_l_moveit_bringup.launch.py
```

启动成功后即可使用 MoveIt2 进行真实机械臂的控制。

### 安全提示

---

在使用机械臂时，为保证使用者安全，请参考如下操作规范。

* 每次使用前检查机械臂的安装情况，包括固定螺丝是否松动，机械臂是否存在震动、晃动的情况。
* 机械臂在运行过程中，人不可处于机械臂落下或工作范围内，也不可将其他物体放到机械臂动作的安全范围内。
* 在不使用机械臂时，应将机械臂置于安全位置，防止震动时机械臂跌落而损坏或砸伤其他物体。
* 在不使用机械臂时应及时断开机械臂电源。
