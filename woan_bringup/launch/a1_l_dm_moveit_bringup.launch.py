from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('woan_driver'),
                        'launch', 'a1_l_dm_driver.launch.py')
        )
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('a1_l_config'),
                        'launch', 'rviz_with_planner.launch.py')
        )
    )

    return LaunchDescription([
        driver_launch,
        moveit_launch
    ])
