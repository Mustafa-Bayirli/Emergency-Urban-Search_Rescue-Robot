import rclpy
from rclpy.node import Node
from system_msgs.msg import TemperatureVals, HumidityVal
import Adafruit_DHT

class TempSensor(object):

    def __init__(self, sensor_pin):
        self.sensor_pin = sensor_pin
        self.sensor = Adafruit_DHT.DHT11

    def humidity(self):
        humidity, temperature = Adafruit_DHT.read_retry(self.sensor, self.sensor_pin)

        if humidity is not None:
            return humidity / 1.0
        else:
            return 0.0

    def temperature(self):
        humidity, temperature = Adafruit_DHT.read_retry(self.sensor, self.sensor_pin)

        if temperature is not None:
            return temperature / 1.0
        else:
            return 0.0

class TemperatureWrapper(Node):

    def __init__(self):

        super().__init__('DHT11_driver')

        self.declare_parameter('sensor_pin', value=0)
        if (self.get_parameter('sensor_pin').value == 0):
            print("WARNING! Sensor pin is not set!")
            raise Exception("Sensor pin is not set!")

        self.temperaturePub = self.create_publisher(TemperatureVals, 'DHT11/temperature', 10)
        self.humidityPub = self.create_publisher(HumidityVal, 'DHT11/humidity', 10)

        # loading the driver
        self.dht_sensor = TempSensor(self.get_parameter('sensor_pin').value)

        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.publish_current_temperature)
        self.timer = self.create_timer(timer_period, self.publish_current_humidity)

    def publish_current_temperature(self):
        tempCelcius = self.dht_sensor.temperature()
        tempFahrenheit = tempCelcius * 9/5.0 + 32

        temp = TemperatureVals()
        temp.temperature_celcius = tempCelcius
        temp.temperature_fahrenheit = tempFahrenheit
            
        self.temperaturePub.publish(temp)

    def publish_current_humidity(self):
        humidity = self.dht_sensor.humidity()

        hum = HumidityVal()
        hum.humidity = humidity

        self.humidityPub.publish(hum)

# Main function
def main(args=None):
    # Initialize the node and name it.
    rclpy.init(args=args)

    temperature_wrapper = TemperatureWrapper()

    rclpy.spin(temperature_wrapper)

    temperature_wrapper.destroy_node()
    rclpy.shutdown()

# Main function.
if __name__ == '__main__':
    main()
