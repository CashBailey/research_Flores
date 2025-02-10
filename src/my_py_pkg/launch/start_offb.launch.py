import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Offboard Control Node
        Node(
            package='my_py_pkg',
            executable='offb_node',
            output='screen',
        ),
        
        # PBVS Node with parameters
        Node(
            package='my_py_pkg',
            executable='pbvs_node',
            output='screen',
            parameters=[
                {'lambdav': 0.1},
                {'lambdaw': 0.1}
            ]
        )
    ])