from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_package',
            executable='driver',
            parameters= [
                {'encoder_cpr':7940},
                {'loop_rate':30},
                {'serial_port':'/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0'},
                {'baud_rate':57600}
            ]
        )
    ])