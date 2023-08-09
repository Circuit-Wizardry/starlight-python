import machine

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
ICM42605_ADR = 0x68
BMP388_ADR = 0x76
SCL = machine.Pin(3)
SDA = machine.Pin(2)

import _thread
import time
import fusion
import struct

class ICM42605:
    def __init__(self, i2c, address, fu):
        self.i2c = i2c
        self.addr = address
        self.operating = False
        self.x = 0
        self.fusion = fu
        self.gyro_bias = (0, 0, 0)
        self.accel_bias = (0, 0, 0)
        
    def config_gyro(self, value="b'\x48'"):
        if self.operating:
            self.disable()
        self.i2c.writeto_mem(self.addr, GYRO_CONFIG, value)
        
    def get_bias(self):
        print("Keep the board flat and still while the gyroscope is calibrated.")

        precision = 100 # change this number to change precision of calibration. higher number will take longer 

        # GYROX
        x_gyro = []
        for i in range(precision):
            data = i2c.readfrom_mem(self.addr, 0x25, 2)
            # Assuming data[0] is the upper byte and data[1] is the lower byte
            gyro_value = (data[0] << 8) | data[1]
            # Convert to signed integer
            if gyro_value & 0x8000:
                gyro_value -= 0x10000
            x_gyro.append(gyro_value * (72/32768)) # I found this value through trial and error, trying to minimize error
            time.sleep(0.01)

        x_gyro_err = sum(x_gyro) / len(x_gyro)
        
        # GYROY
        y_gyro = []
        for i in range(precision):
            data = i2c.readfrom_mem(self.addr, 0x27, 2)
            # Assuming data[0] is the upper byte and data[1] is the lower byte
            gyro_value = (data[0] << 8) | data[1]
            # Convert to signed integer
            if gyro_value & 0x8000:
                gyro_value -= 0x10000
            y_gyro.append(gyro_value * (72/32768)) # I found this value through trial and error, trying to minimize error
            time.sleep(0.01)

        y_gyro_err = sum(y_gyro) / len(y_gyro)
        
        # GYROZ
        z_gyro = []
        for i in range(precision):
            data = i2c.readfrom_mem(self.addr, 0x29, 2)
            # Assuming data[0] is the upper byte and data[1] is the lower byte
            gyro_value = (data[0] << 8) | data[1]
            # Convert to signed integer
            if gyro_value & 0x8000:
                gyro_value -= 0x10000
            z_gyro.append(gyro_value * (72/32768)) # I found this value through trial and error, trying to minimize error
            time.sleep(0.01)

        z_gyro_err = sum(z_gyro) / len(z_gyro)
        
        self.gyro_bias = (x_gyro_err, y_gyro_err, z_gyro_err)
        
        # ======================================================================================================
#         
#         
#       # ACCELX
        x_accel = []
        for i in range(precision):
            data = i2c.readfrom_mem(self.addr, 0x25, 2)
            # Assuming data[0] is the upper byte and data[1] is the lower byte
            accel_value = (data[0] << 8) | data[1]
            # Convert to signed integer
            if accel_value & 0x8000:
                accel_value -= 0x10000
            x_accel.append(accel_value * (72/32768)) # I found this value through trial and error, trying to minimize error
            time.sleep(0.01)

        x_accel_err = sum(x_accel) / len(x_accel)
        
        # ACCELY
        y_accel = []
        for i in range(precision):
            data = i2c.readfrom_mem(self.addr, 0x27, 2)
            # Assuming data[0] is the upper byte and data[1] is the lower byte
            accel_value = (data[0] << 8) | data[1]
            # Convert to signed integer
            if accel_value & 0x8000:
                accel_value -= 0x10000
            y_accel.append(accel_value * (72/32768)) # I found this value through trial and error, trying to minimize error
            time.sleep(0.01)

        y_accel_err = sum(y_accel) / len(y_accel)
        
        # ACCELZ
        z_accel = []
        for i in range(precision):
            data = i2c.readfrom_mem(self.addr, 0x29, 2)
            # Assuming data[0] is the upper byte and data[1] is the lower byte
            accel_value = (data[0] << 8) | data[1]
            # Convert to signed integer
            if accel_value & 0x8000:
                accel_value -= 0x10000
            z_accel.append(accel_value * (72/32768)) # I found this value through trial and error, trying to minimize error
            time.sleep(0.01)

        z_accel_err = sum(z_accel) / len(z_accel)
        
        self.gyro_bias = (x_gyro_err, y_gyro_err, z_gyro_err)
        self.accel_bias = (x_accel_err, y_accel_err, z_accel_err)
        
        
        
        
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
        data = i2c.readfrom_mem(ICM42605_ADR, 0x25, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        gyro_x = (data[0] << 8) | data[1]
        # Convert to signed integer
        if gyro_x & 0x8000:
            gyro_x -= 0x10000
        gyro_x = gyro_x * (500.0 / 32768.0 / 128)
        
        data = i2c.readfrom_mem(ICM42605_ADR, 0x27, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        gyro_y = (data[0] << 8) | data[1]
        # Convert to signed integer
        if gyro_y & 0x8000:
            gyro_y -= 0x10000
        gyro_y = gyro_y * (500.0 / 32768.0 / 128)
        
        data = i2c.readfrom_mem(ICM42605_ADR, 0x29, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        gyro_z = (data[0] << 8) | data[1]
        # Convert to signed integer
        if gyro_z & 0x8000:
            gyro_z -= 0x10000
        gyro_z = gyro_z * (500.0 / 32768.0 / 128)
        
        # -------------------------
        # -------------------------
        
        data = i2c.readfrom_mem(ICM42605_ADR, 0x1F, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_x = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_x & 0x8000:
            accel_x -= 0x10000
        accel_x = accel_x * (500.0 / 32768.0 / 128)
        
        data = i2c.readfrom_mem(ICM42605_ADR, 0x21, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_y = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_y & 0x8000:
            accel_y -= 0x10000
        accel_y = accel_y * (500.0 / 32768.0 / 128)
        
        data = i2c.readfrom_mem(ICM42605_ADR, 0x23, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_z = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_z & 0x8000:
            accel_z -= 0x10000
        accel_z = accel_z * (500.0 / 32768.0 / 128)
        
        self.fusion.update_nomag((accel_x - self.accel_bias[0], accel_y - self.accel_bias[1], accel_z - self.accel_bias[2]), (gyro_x - self.gyro_bias[0], gyro_y - self.gyro_bias[1], gyro_z - self.gyro_bias[2]))
        
        

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
                self.x += gyro_value * (500.0 / 32768.0 / 128) - self.x_bias
                i += 8
                
    
    def start_gyros(self):
        _thread.start_new_thread(self.thread_func, ())
        
    def thread_func(self):
        while True:
            #self.x += self.get_gyro_x()
            self.get_gyro_x()
            time.sleep_ms(1)
        
        
if __name__ == '__main__':
    import machine
    i2c = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=9600)
    fu = fusion.Fusion()
    gyr = ICM42605(i2c, 0x68, fu)
    gyr.config_gyro()
    gyr.enable()
#     gyr.get_bias()
    gyr.start_gyros()
    
#     temp = BMP388(i2c, 0x76)
#     temp.enable_temp_and_pressure()
#     temp.calibrate()
    
    while True:
        print(gyr.fusion.pitch)
        print(gyr.fusion.roll)
        time.sleep_ms(10)
        