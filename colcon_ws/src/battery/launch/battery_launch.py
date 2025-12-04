from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    battery_node = Node(
        package='battery',
        executable='battery_state',
        name='battery_state',
        output='screen'
    )

    return LaunchDescription([
				battery_node
    ])

