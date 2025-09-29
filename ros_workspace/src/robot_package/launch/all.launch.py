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
        ),
        Node(
            package='robot_package',
            executable='AccelerometerWrapper'    
        ),
        Node(
            package='robot_package',
            executable='TemperatureWrapper',
            parameters= [
                {'sensor_pin':4}
            ]    
        ),
        Node(
            package='robot_package',
            executable='driver',
            parameters= [
                {'encoder_cpr':7940},
                {'loop_rate':30},
                {'serial_port':'/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0'},
                {'baud_rate':57600}
            ]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            parameters=[
                {'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'},
                {'frame_id': 'laser_frame'},
                {'angle_compensate': True}
            ]
        )
    ])
