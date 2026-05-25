from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    driver_params = os.path.join(
        get_package_share_directory('onero_driver'),
        'config',
        'a1_l_driver.yaml'
    )
    
    driver_node = Node(
        package='onero_driver',
        executable='onero_driver_node',
        name='onero_driver_node',
        output='screen',
        parameters=[driver_params]
    )
    
    return LaunchDescription([
        driver_node
    ])
