from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 参数声明
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="是否使用模拟硬件（true: 启动 joint_state_publisher_gui, false: 从真实硬件获取）",
        ),
        DeclareLaunchArgument(
            "enable_rviz_integration",
            default_value="true",
            description="是否启用RViz集成模式（true: 从RViz Plan/Execute, false: 从话题接收目标）",
        ),
        DeclareLaunchArgument(
            "velocity_scaling",
            default_value="0.3",
            description="",
        ),
        DeclareLaunchArgument(
            "acceleration_scaling",
            default_value="0.3",
            description="",
        ),
        DeclareLaunchArgument(
            "planning_time",
            default_value="15.0",
            description="",
        ),
        DeclareLaunchArgument(
            "planning_attempts",
            default_value="10",
            description="规划尝试次数",
        ),
    ]

    moveit_config = MoveItConfigsBuilder(
        "a1_l_urdf", package_name="a1_l_config"
    ).to_moveit_configs()

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("a1_l_config"),
                "launch",
                "rsp.launch.py"
            ])
        ),
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("a1_l_config"),
                "launch",
                "static_virtual_joint_tfs.launch.py"
            ])
        ),
    )


    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("a1_l_config"),
                "launch",
                "move_group.launch.py"
            ])
        ),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("a1_l_config"),
                "launch",
                "moveit_rviz.launch.py"
            ])
        ),
    )

    moveit_planner_node = Node(
        package="onero_control",
        executable="onero_control_node",
        name="onero_control_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,  # 添加 robot_description
            moveit_config.robot_description_semantic,  # 添加 SRDF
            moveit_config.robot_description_kinematics,  # 添加运动学配置
            {
                "planning_group": "arm",
                "planning_time": LaunchConfiguration("planning_time"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),
                "goal_tolerance": 0.0001,
                "dof": 7,
                "velocity_scaling": LaunchConfiguration("velocity_scaling"),
                "acceleration_scaling": LaunchConfiguration("acceleration_scaling"),
                "enable_rviz_integration": LaunchConfiguration("enable_rviz_integration"),
            }
        ],
    )

    # 延迟启动 moveit_planner_node，等待 move_group 初始化完成
    delayed_planner_node = TimerAction(
        period=5.0,
        actions=[moveit_planner_node]
    )

    return LaunchDescription(
        declared_arguments
        + [
            rsp_launch,                    # 1. 首先发布 robot_description
            static_tf_launch,              # 2. 发布虚拟关节 TF
            joint_state_publisher_gui,     # 3. 发布模拟的关节状态（如果使用模拟硬件）
            move_group_launch,             # 4. 启动 MoveIt2 规划器
            rviz_launch,                   # 5. 启动 RViz
            delayed_planner_node,          # 6. 延迟启动规划节点
        ]
    )
