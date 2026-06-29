from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false',
        description='true 时进入仿真模式')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='true 时启动 rviz2 可视化（真机或仿真都可用）')
    gripper_arg = DeclareLaunchArgument(
        'gripper', default_value='false',
        description='true 时启用单臂夹爪')

    sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('rviz')
    use_gripper = LaunchConfiguration('gripper')

    driver_params = os.path.join(
        get_package_share_directory('onero_driver'), 'config', 'a1_r_driver.yaml')

    description_share = FindPackageShare('onero_description')
    urdf_xacro = PathJoinSubstitution([description_share, 'urdf', 'A1', 'a1_r.urdf.xacro'])
    rviz_config = PathJoinSubstitution([description_share, 'rviz', 'a1_r.rviz'])

    driver_node = Node(
        package='onero_driver',
        executable='onero_driver_node',
        name='onero_driver_node',
        output='screen',
        parameters=[driver_params,
                    {'simulation_mode': sim},
                    {'gripper.enabled': use_gripper}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str),
        }],
        condition=IfCondition(use_rviz),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        sim_arg,
        rviz_arg,
        gripper_arg,
        driver_node,
        robot_state_publisher_node,
        rviz_node,
    ])
