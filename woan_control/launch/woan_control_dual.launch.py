from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    control_node = Node(
        package='woan_control',
        executable='woan_control_dual_node',
        name='woan_control_dual_node',
        output='screen'
    )
    
    return LaunchDescription([
        control_node
    ])

