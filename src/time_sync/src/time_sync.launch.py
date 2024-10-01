# my_launch_file.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='time_sync',
            executable='firstPublisher',
            name='firstPublisher',
            output='screen',
            #parameters=[{'param_name': 'param_value'}]
        ),
        # Add more nodes here if needed
    ])