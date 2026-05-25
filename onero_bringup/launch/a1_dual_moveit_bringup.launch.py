from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 驱动层 - 启动 onero_driver
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('onero_driver'),
                        'launch', 'a1_dual_driver.launch.py')
        )
    )

    # 2. 规划层 - 启动 MoveIt2 规划系统
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dual_config'),
                        'launch', 'rviz_with_planner.launch.py')
        )
    )

    return LaunchDescription([
        driver_launch,
        moveit_launch
    ])
