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
BMP388_ADR = 0x76
ICM42605_ADR = 0x68

SCL = machine.Pin(3)
SDA = machine.Pin(2)

import _thread
import time
import struct

class Starlight:
    def __init__(self, i2c):
        self.i2c = i2c
        self.icm_addr = ICM42605_ADR # ICM-42605 address
        self.bmp_addr = BMP388_ADR # BMP388 address
        self.operating = False
        self.x = 0
        self.x_bias = 0
        self.temp_calib = 0
        self.pressure_calib = 0
        
    def config_gyro(self, value="b'\x4B'"):
        if self.operating:
            self.disable()
        self.i2c.writeto_mem(self.icm_addr, GYRO_CONFIG, value)
        
    def get_bias(self):
        time.sleep_ms(500)
        print("Keep the board flat and still while the gyroscope is calibrated.")
            
        precision = 200 # change this number to change precision of calibration. higher number will take longer 
        
        # X
        x_result = []
        for i in range(precision):
            x_result.append(self.get_gyro_x())
        
        x_err = sum(x_result) / len(x_result)
        self.x_bias = x_err
        
    def toInt(self, data):
        return int.from_bytes(data, 'big')
        
    def getTemperature(self):
        return self.read_temp_and_pressure()[0];

    def getPressure(self):
        return self.read_temp_and_pressure()[1];
        
    def enable_temp_and_pressure(self):
        self.i2c.writeto_mem(BMP388_ADR, 0x1B, b'\x33')
        
    
    def read_temp_and_pressure(self):
        # See if readings are ready
        status = self.i2c.readfrom_mem(BMP388_ADR, 0x03, 1)
        #print(toHex(status))
        if (self.toInt(status) & 0x60 != 0x60):
            print("Not ready")
        else:
            # ** If you want to know how this works, don't ask me. I stole all this code from https://github.com/adafruit/Adafruit_CircuitPython_BMP3XX/
            
            # read and bit shift our readings
            data = self.i2c.readfrom_mem(BMP388_ADR, 0x04, 6)
            adc_p = data[2] << 16 | data[1] << 8 | data[0]
            adc_t = data[5] << 16 | data[4] << 8 | data[3]
            #print(adc_t)
            
            T1, T2, T3 = self.temp_calib
            
            pd1 = adc_t - T1
            pd2 = pd1 * T2
            
            temperature = pd2 + (pd1 * pd1) * T3 #TEMPERATURE IN C
                    
            # datasheet, sec 9.3 Pressure compensation
            P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11 = self.pressure_calib

            pd1 = P6 * temperature
            pd2 = P7 * temperature**2.0
            pd3 = P8 * temperature**3.0
            po1 = P5 + pd1 + pd2 + pd3

            pd1 = P2 * temperature
            pd2 = P3 * temperature**2.0
            pd3 = P4 * temperature**3.0
            po2 = adc_p * (P1 + pd1 + pd2 + pd3)

            pd1 = adc_p**2.0
            pd2 = P9 + P10 * temperature
            pd3 = pd1 * pd2
            pd4 = pd3 + P11 * adc_p**3.0

            pressure = (po1 + po2 + pd4)/100 #PRESSURE IN hPa
            
            return(temperature, pressure)
    
    def calibrate_bmp388(self):
        # find our compensation coefficient values (this shit is all copied from adafruit's library, don't ask me how it works)
        coeff = i2c.readfrom_mem(self.bmp_addr, 0x31, 21)
        coeff = struct.unpack("<HHbhhbbHHbbhbb", coeff)
        
        self.temp_calib = (
            coeff[0] / 2**-8.0,  # T1
            coeff[1] / 2**30.0,  # T2
            coeff[2] / 2**48.0,
        )  # T3
        self.pressure_calib = (
            (coeff[3] - 2**14.0) / 2**20.0,  # P1
            (coeff[4] - 2**14.0) / 2**29.0,  # P2
            coeff[5] / 2**32.0,  # P3
            coeff[6] / 2**37.0,  # P4
            coeff[7] / 2**-3.0,  # P5
            coeff[8] / 2**6.0,  # P6
            coeff[9] / 2**8.0,  # P7
            coeff[10] / 2**15.0,  # P8
            coeff[11] / 2**48.0,  # P9
            coeff[12] / 2**48.0,  # P10
            coeff[13] / 2**65.0,
        )  # P11
    
        
    def enable(self, accel=True, gyro=True, temp=True):
        if self.operating:
            return False
        
        # Start FIFO
        self.i2c.writeto_mem(self.icm_addr, FIFO_CONFIG1, b'\x02')
        self.i2c.writeto_mem(self.icm_addr, FIFO_CONFIG, b'\x40')
        
        val = 0x00
        if accel:
            val |= 0x03
        if gyro:
            val |= 0x0c
        if temp:
            val &= 0x1F
        self.i2c.writeto_mem(self.icm_addr, PWR_MGMT0, val.to_bytes(1, 'big'))
        self.operating = True
        
    def disable(self):
        self.i2c.writeto_mem(self.icm_addr, PWR_MGMT0, b'\x00')
        self.operating = False
        
    def get_gyro_x(self):
        data = self.i2c.readfrom_mem(self.icm_addr, GYRO_DATA_X1, 2)
        gyro_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if gyro_value & 0x8000:
            gyro_value -= 0x10000
    
        return gyro_value * (500.0 / 32768.0 / 128)
    
    def fifo_count(self):
        count = self.i2c.readfrom_mem(self.icm_addr, FIFO_COUNTH, 2)
        return int.from_bytes(count, 'big')
    
    def read_fifo(self):
        count = self.fifo_count()
        i = 0
        while i < count:
            data = int.from_bytes(self.i2c.readfrom_mem(self.icm_addr, FIFO_DATA, 1), 'big')
            if ( data & 0xE0 == 0x20): # Contains data, not accel, yes gyro
                gyr = self.i2c.readfrom_mem(self.icm_addr, FIFO_DATA, 8)
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
            self.read_fifo()
            time.sleep_ms(1)
        
        
if __name__ == '__main__':
    import machine
    i2c = machine.I2C(1, scl=SCL, sda=SDA, freq=9600)
    gyr = Starlight(i2c)
    gyr.config_gyro()
    gyr.enable()
    gyr.get_bias()
#     gyr.start_gyros()
    gyr.enable_temp_and_pressure()
    gyr.calibrate_bmp388()
    while True:
        print(gyr.getTemperature())
#         print(gyr.x_bias)
        time.sleep_ms(1000)
        