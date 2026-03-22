from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_servo_agent',
            executable='sensor2servo_node',
            name='sensor2servo_node',
            output='screen'
        )
    ])
