#!/usr/bin/env python3
"""
A1-R Gazebo仿真启动文件
启动Gazebo环境和A1-R机械臂仿真模型
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # 包名和模型名
    package_name = 'onero_gazebo'
    robot_name = 'a1_r_robot'

    # 获取Gazebo专用URDF路径
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'config', 'a1_r_gazebo_description.urdf.xacro')

    # 获取onero_description包的路径用于构建file://路径
    onero_description_pkg = get_package_share_directory('onero_description')

    # 解析xacro文件
    doc = xacro.parse(open(urdf_file))
    # 传递 mesh_prefix 参数给xacro处理 - 使用file://协议加绝对路径避免Gazebo转换问题
    xacro.process_doc(doc, mappings={'mesh_prefix': f'file://{onero_description_pkg}'})
    robot_description = {'robot_description': doc.toxml()}

    # 1. 启动Gazebo服务器（禁用物理仿真）
    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 2. 启动robot_state_publisher（使用仿真时间）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            robot_description,
            {'publish_frequency': 15.0}
        ],
        output='screen'
    )

    # 3. 在Gazebo中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-z', '0.03'
        ],
        output='screen'
    )

    # 4. 加载joint_state_broadcaster控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 5. 加载arm_controller控制器（与MoveIt对接）
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    # 控制启动顺序：spawn → joint_state_broadcaster → arm_controller
    load_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    load_arm_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )

    return LaunchDescription([
        gazebo_server,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_after_spawn,
        load_arm_controller_after_joint_state,
    ])
