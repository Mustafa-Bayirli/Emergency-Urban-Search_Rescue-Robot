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
        self.address = address
        self.bus = SMBus(1)
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.address, self.CONFIG, 0)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        highByte = self.bus.read_byte_data(self.address, addr)
        lowByte = self.bus.read_byte_data(self.address, addr+1)

        value = ((highByte << 8) | lowByte)

        if(value > 32768):
            value = value - 65536
        return value

    def raw_accel_data(self):
        accel_x = self.read_raw_data(self.ACCEL_XOUT_H)
        accel_y = self.read_raw_data(self.ACCEL_YOUT_H)
        accel_z = self.read_raw_data(self.ACCEL_ZOUT_H)

        Ax = accel_x/16384.0
        Ay = accel_y/16384.0
        Az = accel_z/16384.0

        return (Ax, Ay, Az)

    def raw_gyro_data(self):
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        return(Gx, Gy, Gz)
