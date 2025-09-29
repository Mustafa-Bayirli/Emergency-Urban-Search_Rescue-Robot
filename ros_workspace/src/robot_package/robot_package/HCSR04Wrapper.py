import rclpy
from rclpy.node import Node
from system_msgs.msg import DistanceVals
import sys
import RPi.GPIO as GPIO
import time

class UltrasonicHCSR04(object):

    def __init__(self, trigger, echo):
        self.trigger = trigger
        self.echo = echo

        self.timeout = 0.05

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    def distance(self):
        GPIO.output(self.trigger, True) # set trigger to HIGH

        # set trigger after 0.01 ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)

        startTime = time.time()
        arrivalTime = time.time()

        timeout_start = time.time()

        # store startTime
        while GPIO.input(self.echo) == 0:
            startTime = time.time()

            if startTime - timeout_start > self.timeout:
                return -1

        # store arrivalTime
        while GPIO.input(self.echo) == 1:
            arrivalTime = time.time()

            if startTime - timeout_start > self.timeout:
                return -1

        if startTime != 0 and arrivalTime != 0:
            # calculate the difference between start and stop
            duration = arrivalTime - startTime

            # multiply with speed of sound (34300 cm/s)
            # and divide by 2 because there and back
            distance = (duration * 34300) / 2

            if distance >= 0:
                return distance
            else:
                return -1
        else:
            return -1

class UltrasonicHCSR04Wrapper(Node):

    def __init__(self):

        super().__init__('ultrasonic_driver')

        self.declare_parameter('sensor_trigger_right', value=0)
        if (self.get_parameter('sensor_trigger_right').value == 0):
            print("WARNING! Right Sensor Trigger pin is not set!")
            raise Exception("Right Sensor Trigger pin is not set!")

        self.declare_parameter('sensor_echo_right', value=0)
        if (self.get_parameter('sensor_echo_right').value == 0):
            print("WARNING! Right Sensor Echo pin is not set!")
            raise Exception("Right Sensor Echo pin is not set!")
        
        self.declare_parameter('sensor_trigger_center', value=0)
        if (self.get_parameter('sensor_trigger_center').value == 0):
            print("WARNING! Center Sensor Trigger pin is not set!")
            raise Exception("Center Sensor Trigger pin is not set!")

        self.declare_parameter('sensor_echo_center', value=0)
        if (self.get_parameter('sensor_echo_center').value == 0):
            print("WARNING! Center Sensor Echo pin is not set!")
            raise Exception("Center Sensor Echo pin is not set!")
        
        self.declare_parameter('sensor_trigger_left', value=0)
        if (self.get_parameter('sensor_trigger_left').value == 0):
            print("WARNING! Left Sensor Trigger pin is not set!")
            raise Exception("Left Sensor Trigger pin is not set!")

        self.declare_parameter('sensor_echo_left', value=0)
        if (self.get_parameter('sensor_echo_left').value == 0):
            print("WARNING! Left Sensor Echo pin is not set!")
            raise Exception("Left Sensor Echo pin is not set!")

        self.min_range = float(3)
        self.max_range = float(400)
        self.fov = 0.26179938779915 # 15 degrees

        self.ultrasonicPub = self.create_publisher(DistanceVals, 'ultrasonic/distance', 10)

        # loading the driver
        self.ultrasonic_sensor_right = UltrasonicHCSR04(self.get_parameter('sensor_trigger_right').value, self.get_parameter('sensor_echo_right').value)
        self.ultrasonic_sensor_center = UltrasonicHCSR04(self.get_parameter('sensor_trigger_center').value, self.get_parameter('sensor_echo_center').value)
        self.ultrasonic_sensor_left = UltrasonicHCSR04(self.get_parameter('sensor_trigger_left').value, self.get_parameter('sensor_echo_left').value)

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.publish_current_distance)

    def publish_current_distance(self):
        distance_right = self.ultrasonic_sensor_right.distance()
        distance_center = self.ultrasonic_sensor_center.distance()
        distance_left = self.ultrasonic_sensor_left.distance()

        distances = DistanceVals()

        distances.right = float(distance_right)
        distances.center = float(distance_center)
        distances.left = float(distance_left)
            
        self.ultrasonicPub.publish(distances)

# Main function
def main(args=None):
    # Initialize the node and name it.
    rclpy.init(args=args)

    ultrasonic_wrapper = UltrasonicHCSR04Wrapper()

    rclpy.spin(ultrasonic_wrapper)

    ultrasonic_wrapper.destroy_node()
    rclpy.shutdown()

# Main function.
if __name__ == '__main__':
    main()
