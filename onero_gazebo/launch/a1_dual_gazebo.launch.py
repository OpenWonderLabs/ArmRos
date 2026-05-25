#!/usr/bin/env python3
"""
A1-dual Gazebo仿真启动文件
启动Gazebo环境和A1-dual机械臂仿真模型
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
    robot_name = 'a1_dual_gazebo'

    # 获取Gazebo专用URDF路径
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'config', 'a1_dual_gazebo_description.urdf.xacro')
    
    # 获取onero_description包的路径用于构建file://路径
    onero_description_pkg = get_package_share_directory('onero_description')

    # 解析xacro文件
    print(f"[INFO] 加载URDF文件: {urdf_file}")
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

    # 3. 在Gazebo中生成机器人模型（抬高10cm）
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

    # # 5. 加载 left_arm_controller 控制器（配置状态，不激活）
    # load_left_arm_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'configured',
    #          'left_arm_controller'],
    #     output='screen'
    # )

    # # 6. 加载 right_arm_controller 控制器（配置状态，不激活）
    # load_right_arm_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'configured',
    #          'right_arm_controller'],
    #     output='screen'
    # )

    # 7. 加载 dual_arm_controller 控制器（配置状态，不激活）
    # 将在 controller_switcher 启动后根据规划组自动激活对应的控制器
    load_dual_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'dual_arm_controller'],
        output='screen'
    )

    # 8. 控制加载顺序：
    # spawn → joint_state_broadcaster → dual_arm_controller → left_arm_controller → right_arm_controller
    # 所有控制器以 configured 状态加载，由 controller_switcher 根据规划组激活对应控制器
    load_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    load_dual_arm_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_dual_arm_controller],
        )
    )

    # load_left_arm_controller_after_dual = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=load_dual_arm_controller,
    #         on_exit=[load_left_arm_controller],
    #     )
    # )

    # load_right_arm_controller_after_left = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=load_left_arm_controller,
    #         on_exit=[load_right_arm_controller],
    #     )
    # )

    # # 9. Controller Switcher - 监听轨迹并自动切换控制器
    # # 在所有控制器加载完成后启动，并立即激活初始控制器（dual_arm）
    # controller_switcher = Node(
    #     package='onero_control',
    #     executable='controller_switcher_node',
    #     name='controller_switcher',
    #     output='screen',
    #     parameters=[
    #         {'current_planning_group': 'dual_arm'},  # 初始规划组
    #     ],
    # )

    # # 在 right_arm_controller 加载完成后启动 controller_switcher
    # start_switcher_after_controllers = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=load_right_arm_controller,
    #         on_exit=[controller_switcher],
    #     )
    # )

    return LaunchDescription([
        gazebo_server,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_after_spawn,
        load_dual_arm_controller_after_joint_state,
        # load_left_arm_controller_after_dual,
        # load_right_arm_controller_after_left,
        # start_switcher_after_controllers,  # 在所有控制器加载完成后启动切换器
    ])

