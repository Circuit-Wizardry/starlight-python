# Registers
DEVICE_CONFIG = 0x11
FIFO_CONFIG   = 0x16
ACCEL_DATA_X1 = 0x1f
ACCEL_DATA_Y1 = 0x21
ACCEL_DATA_Z1 = 0x23
GYRO_DATA_X1  = 0x25
GYRO_DATA_Y1  = 0x27
GYRO_DATA_Z1  = 0x29
PWR_MGMT0     = 0x4e
GYRO_CONFIG   = 0x4f
APEX_CONFIG0  = 0x56
FIFO_CONFIG1  = 0x5f


class ICM42605:
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.addr = address
        self.operating = False
        
    def config_gyro(self, value="b'\x03'"):
        if self.operating:
            self.disable()
        self.i2c.writeto_mem(address, GYRO_CONFIG, value)
        
    def enable(self, accel=True, gyro=True, temp=True):
        if self.operating:
            return False
        
        value = b'\x00'
        if accel:
            value |= b'\x03'
        if gyro:
            value |= b'\x0c'
        if temp:
            value &= b'\1F'
        print(value)
        self.i2c.writeto_mem(address, PWR_MGMT0, value)
        self.operating = true
        
    def disable(self):
        self.i2c.writeto_mem(address, PWR_MGMT0, b'\x00')
        self.operating = False
        