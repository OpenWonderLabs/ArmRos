#!/usr/bin/env python3
"""
X1-L Gazebo MoveIt Demo
在Gazebo仿真环境中启动MoveIt2规划和RViz可视化
"""
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.1"),
        DeclareLaunchArgument("acceleration_scaling", default_value="0.1"),
    ]

    # 加载MoveIt配置
    moveit_config = MoveItConfigsBuilder(
        "x1_l_urdf",
        package_name="x1_l_config"
    ).to_moveit_configs()

    # MoveGroup节点（使用仿真时间）
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},  # ⚠️ 关键：使用仿真时间
        ],
        output='screen'
    )

    # RViz节点（使用仿真时间）
    rviz_config = PathJoinSubstitution([
        FindPackageShare('x1_l_config'),
        'config', 'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},  # ⚠️ 关键：使用仿真时间
        ],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + [
            move_group_node,
            rviz_node,
        ]
    )

