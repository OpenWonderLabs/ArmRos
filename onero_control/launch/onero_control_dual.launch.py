from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    control_node = Node(
        package='onero_control',
        executable='onero_control_dual_node',
        name='onero_control_dual_node',
        output='screen'
    )
    
    return LaunchDescription([
        control_node
    ])

