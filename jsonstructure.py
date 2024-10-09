'''

type:"mission"
mission:"follow me"
keyword: "Emergency"
requirements:{ waypoints:[1,2],image_id:123,...}

requirements of this instruction is jsom.dumps and send to data so ,
every script start taking input first and json.loads it and continue
'''
'''
import smbus
import numpy as np

class IMUSensor:
    def __init__(self, bus_num=1, device_address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.device_address = device_address
        self.bus.write_byte_data(self.device_address, 0x6B, 0)  # Wake up the MPU-6050

    def get_acceleration(self):
        # Read accelerometer data from the sensor registers
        ax = self.read_word_2c(0x3B) / 16384.0  # MPU6050 specific scaling
        ay = self.read_word_2c(0x3D) / 16384.0
        az = self.read_word_2c(0x3F) / 16384.0
        return np.array([ax, ay, az])

    def get_angular_velocity(self):
        # Read gyroscope data from the sensor registers
        gx = self.read_word_2c(0x43) / 131.0  # MPU6050 specific scaling
        gy = self.read_word_2c(0x45) / 131.0
        gz = self.read_word_2c(0x47) / 131.0
        return np.array([gx, gy, gz])

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.device_address, reg)
        low = self.bus.read_byte_data(self.device_address, reg+1)
        val = (high << 8) + low
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
'''