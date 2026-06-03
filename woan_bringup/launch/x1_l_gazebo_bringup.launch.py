#!/usr/bin/env python3
"""
X1-L Gazebo完整启动文件
一键启动Gazebo仿真环境和MoveIt2规划系统
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Gazebo仿真层 - 启动Gazebo和虚拟机械臂
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('woan_gazebo'),
                        'launch', 'x1_l_gazebo.launch.py')
        )
    )

    # 2. MoveIt规划层 - 启动MoveIt2规划系统（Gazebo版）
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('x1_l_config'),
                        'launch', 'gazebo_moveit_demo.launch.py')
        )
    )

    return LaunchDescription([
        gazebo_launch,
        moveit_launch
    ])

