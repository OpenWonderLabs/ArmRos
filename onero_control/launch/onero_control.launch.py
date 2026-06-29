from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 参数声明
    declared_arguments = [
        DeclareLaunchArgument(
            "arm_type",
            default_value="a1_r",
            description="机械臂型号：a1_l、a1_r",
        ),
        DeclareLaunchArgument(
            "enable_rviz_integration",
            default_value="true",
            description="是否启用RViz集成模式（true: 从RViz Plan/Execute, false: 从话题接收目标）",
        ),
        DeclareLaunchArgument(
            "planning_time",
            default_value="15.0",
            description="规划器最大搜索时间（秒）",
        ),
        DeclareLaunchArgument(
            "planning_attempts",
            default_value="10",
            description="规划尝试次数",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    # 获取参数值
    arm_type = context.launch_configurations.get("arm_type", "a1_r")
    
    # 映射机械臂型号到配置
    arm_configs = {
        "a1_l": {"urdf_name": "a1_l_urdf", "package_name": "a1_l_config"},
        "a1_r": {"urdf_name": "a1_r_urdf", "package_name": "a1_r_config"},
    }
    
    if arm_type not in arm_configs:
        raise ValueError(
            f"不支持的机械臂型号: {arm_type}。支持的型号: {', '.join(arm_configs.keys())}"
        )
    
    config = arm_configs[arm_type]
    
    # 加载 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        config["urdf_name"], package_name=config["package_name"]
    ).to_moveit_configs()

    # robot_state_publisher 节点 - 发布 robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # onero_control_node 节点
    moveit_planner_node = Node(
        package="onero_control",
        executable="onero_control_node",
        name="onero_control_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,  # robot_description 参数
            moveit_config.robot_description_semantic,  # SRDF 配置
            moveit_config.robot_description_kinematics,  # 运动学配置
            {
                "planning_group": "arm",
                "planning_time": LaunchConfiguration("planning_time"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),
                "goal_tolerance": 0.0001,
                "dof": 7,
                "enable_rviz_integration": LaunchConfiguration("enable_rviz_integration"),
            }
        ],
    )

    # 延迟启动 onero_control_node，等待 robot_state_publisher 初始化完成
    delayed_planner_node = TimerAction(
        period=2.0,
        actions=[moveit_planner_node]
    )

    return [
        robot_state_publisher,      # 1. 首先发布 robot_description
        delayed_planner_node,       # 2. 延迟启动规划节点
    ]
