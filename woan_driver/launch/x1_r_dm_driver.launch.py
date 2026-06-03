from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    driver_params = os.path.join(
        get_package_share_directory('woan_driver'),
        'config',
        'x1_r_dm_driver.yaml'
    )
    
    driver_node = Node(
        package='woan_driver',
        executable='woan_driver_node',
        name='woan_driver_node',
        output='screen',
        parameters=[driver_params]
    )
    
    return LaunchDescription([
        driver_node
    ])

