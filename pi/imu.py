"""MPU6050 gyro Z integration on Pi 5 I2C bus 1; requires smbus2."""
import time

class MPU6050:
    def __init__(self, bus_number=1, address=0x68):
        from smbus2 import SMBus
        self.bus = SMBus(bus_number)
        self.addr = address
        if self.bus.read_byte_data(address, 0x75) != 0x68:
            raise RuntimeError('MPU6050 WHO_AM_I mismatch')
        self.bus.write_byte_data(address, 0x6B, 0x01)
        time.sleep(0.15)
        self.bus.write_byte_data(address, 0x1B, 0)
        self.bias = 0.0
        self.heading = 0.0
        self.last = time.monotonic()

    def raw_rate(self):
        b = self.bus.read_i2c_block_data(self.addr, 0x47, 2)
        raw = (b[0]<<8)|b[1]
        if raw >= 32768: raw -= 65536
        return raw/131.0

    def calibrate(self):
        print('Keep rover motionless during 3-second gyro calibration')
        total = 0.0
        for _ in range(150):
            total += self.raw_rate()
            time.sleep(0.02)
        self.bias = total/150.0
        self.heading = 0.0
        self.last = time.monotonic()
        print('Gyro bias: %+.4f deg/s' % self.bias)

    def update(self):
        now = time.monotonic()
        dt = now-self.last
        if dt < 0 or dt > 0.25:
            raise RuntimeError('IMU sampling gap %.3f s' % dt)
        rate = self.raw_rate()-self.bias
        self.heading += rate*dt
        self.last = now
        return rate

    def close(self):
        self.bus.close()
