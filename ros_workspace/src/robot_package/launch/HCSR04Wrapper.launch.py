from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_package',
            executable='HCSR04Wrapper',
            parameters= [
                {'sensor_trigger_right':5},
                {'sensor_echo_right':6},
                {'sensor_trigger_center':23},
                {'sensor_echo_center':24},
                {'sensor_trigger_left':27},
                {'sensor_echo_left':22}
            ]
        )
    ])
