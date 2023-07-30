# Registers
DEVICE_CONFIG = 0x11
FIFO_CONFIG   = 0x16
ACCEL_DATA_X1 = 0x1f
ACCEL_DATA_Y1 = 0x21
ACCEL_DATA_Z1 = 0x23
GYRO_DATA_X1  = 0x25
GYRO_DATA_Y1  = 0x27
GYRO_DATA_Z1  = 0x29
FIFO_COUNTH   = 0x2e
FIFO_DATA     = 0x30
PWR_MGMT0     = 0x4e
GYRO_CONFIG   = 0x4f
APEX_CONFIG0  = 0x56
FIFO_CONFIG1  = 0x5f

import _thread
import time

class ICM42605:
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.addr = address
        self.operating = False
        self.x = 0
        self.x_bias = 0
        
    def config_gyro(self, value="b'\x48'"):
        if self.operating:
            self.disable()
        self.i2c.writeto_mem(self.addr, GYRO_CONFIG, value)
        
    def get_bias(self):
        print("Keep the board flat and still while the gyroscope is calibrated.")
            
        precision = 100 # change this number to change precision of calibration. higher number will take longer 
        
        # X
        x_result = []
        for i in range(precision):
            data = i2c.readfrom_mem(self.addr, 0x1F, 2)
            # Assuming data[0] is the upper byte and data[1] is the lower byte
            accel_value = (data[0] << 8) | data[1]
            # Convert to signed integer
            if accel_value & 0x8000:
                accel_value -= 0x10000
            x_result.append(accel_value * (72/32768)) # I found this value through trial and error, trying to minimize error
            time.sleep(0.01)
        
        x_err = sum(x_result) / len(x_result)
        self.x_bias = x_err
        
        
    def enable(self, accel=True, gyro=True, temp=True):
        if self.operating:
            return False
        
        # Start FIFO
        self.i2c.writeto_mem(self.addr, FIFO_CONFIG1, b'\x02')
        self.i2c.writeto_mem(self.addr, FIFO_CONFIG, b'\x40')
        
        val = 0x00
        if accel:
            val |= 0x03
        if gyro:
            val |= 0x0c
        if temp:
            val &= 0x1F
        self.i2c.writeto_mem(self.addr, PWR_MGMT0, val.to_bytes(1, 'big'))
        self.operating = True
        
    def disable(self):
        self.i2c.writeto_mem(self.addr, PWR_MGMT0, b'\x00')
        self.operating = False
        
    def get_gyro_x(self):
        data = self.i2c.readfrom_mem(self.addr, GYRO_DATA_X1, 2)
        gyro_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if gyro_value & 0x8000:
            gyro_value -= 0x10000
    
        return gyro_value * (16/32768)
    
    def fifo_count(self):
        count = self.i2c.readfrom_mem(self.addr, FIFO_COUNTH, 2)
        return int.from_bytes(count, 'big')
    
    def read_fifo(self):
        count = self.fifo_count()
        i = 0
        while i < count:
            data = int.from_bytes(self.i2c.readfrom_mem(self.addr, FIFO_DATA, 1), 'big')
            if ( data & 0xE0 == 0x20): # Contains data, not accel, yes gyro
                gyr = self.i2c.readfrom_mem(self.addr, FIFO_DATA, 8)
                gyro_value = (gyr[1] << 8) | gyr[2]
                if gyro_value & 0x8000:
                    gyro_value -= 0x10000
                self.x += gyro_value * (16/32768) - self.x_bias
                i += 8
                
    
    def start_gyros(self):
        _thread.start_new_thread(self.thread_func, ())
        
    def thread_func(self):
        while True:
            #self.x += self.get_gyro_x()
            self.read_fifo()
            time.sleep_ms(1)
        
        
if __name__ == '__main__':
    import machine
    i2c = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=9600)
    gyr = ICM42605(i2c, 0x68)
    gyr.config_gyro()
    gyr.enable()
    gyr.get_bias()
    gyr.start_gyros()
    while True:
        print(gyr.x)
        time.sleep_ms(50)
        