import rclpy
from rclpy.node import Node

# import custom message type for Relative Velocity
#from robot_package import Accelerometer # import our driver module
from system_msgs.msg import AccelerationVels
from system_msgs.msg import AngularVels

import sys
import time
import board
import adafruit_adxl34x

class Accelerometer(object):

    # Registers for accelerometer
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47

    def __init__(self, address):
        i2c = board.I2C()
        self.accelerometer = adafruit_adxl34x.ADXL343(i2c, address)

    def raw_accel_data(self):
        return (self.accelerometer.acceleration)

    def raw_gyro_data(self):
        return (0,0,0)


class AccelerometerWrapper(Node):

    def __init__(self):

        super().__init__('accelerometer_driver')

        # loading the driver
        self.accelerometer = Accelerometer(0x53)

        self.relativeAcceleration = self.create_publisher(AccelerationVels, '/accelerometer/relative_acceleration', 10)
        self.relativeAngularVelocity = self.create_publisher(AngularVels, '/accelerometer/relative_angle_velocity', 10)

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.publish_relative_acceleration)
        self.timer = self.create_timer(timer_period, self.publish_relative_anglular_velocity)

    def publish_relative_acceleration(self):
        acceleration = self.accelerometer.raw_accel_data()

        accel_x = acceleration[0]
        accel_y = acceleration[1]
        accel_z = acceleration[2]

        #message_str = "Acceleration: x:%4.3f, y:%4.3f, z:%4.3f m/s^2" % (accel_x, accel_y, accel_z)
        
        acc = AccelerationVels()
        acc.x_velocity = float(accel_x)
        acc.y_velocity = float(accel_y)
        acc.z_velocity = float(accel_z)

        self.relativeAcceleration.publish(acc)


    def publish_relative_anglular_velocity(self):
        angularVelocity = self.accelerometer.raw_gyro_data()

        accel_gX = float(angularVelocity[0])
        accel_gY = float(angularVelocity[1])
        accel_gZ = float(angularVelocity[2])


        #message_str = " Angular Velocity: x:%4.3f, y:%4.3f, z:%4.3f rad/s" % (accel_gX, accel_gY, accel_gZ)
        
        av = AngularVels()

        av.x_angular_velocity = accel_gX
        av.y_angular_velocity = accel_gY
        av.z_angular_velocity = accel_gZ
            
        self.relativeAngularVelocity.publish(av)

def main(args=None):
    # Initialize the node and name it.
    rclpy.init(args=args)

    accelerometer_driver = AccelerometerWrapper()

    rclpy.spin(accelerometer_driver)

    accelerometer_driver.destroy_node()
    rclpy.shutdown()

# Main function.
if __name__ == '__main__':
    main()

