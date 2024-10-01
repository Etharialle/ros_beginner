import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the share directory
    pkg_share = os.path.join(os.path.dirname(__file__), '..', 'share', 'time_sync')

    # Publisher node
    publisher_node = Node(
        package='time_sync',
        executable='firstPublisher',  # Replace with your actual publisher executable name
        name='firstPublisher',
        output='screen'
    )

    # Subscriber node
    subscriber_node = Node(
        package='time_sync',
        executable='firstSubscriber',  # Replace with your actual subscriber executable name
        name='firstSubscriber',
        output='screen'
    )

    # Foxglove Bridge node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'ros__parameters': {
                'bridge_type': 'ros2',  # Use the correct bridge type for ROS 2

            }
        }]
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
        foxglove_bridge,
    ])