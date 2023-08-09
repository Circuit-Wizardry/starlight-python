import machine

# Registers
DEVICE_CONFIG = 0x11
FIFO_CONFIG   = 0x16
TEMP_DATA1    = 0x1d
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
import struct

class ICM42605:
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.addr = address
        self.operating = False
        self.gx = 0
        self.gy = 0
        self.gz = 0
        self.gRes = 2000/32768
        self.gOdr = 1000
        self.aRes = 16/32768
        
    def config_gyro(self, value=b'\x48'):
        if self.operating:
            self.disable()
        self.i2c.writeto_mem(self.addr, GYRO_CONFIG, value)
        gyro_fs_sel = int.from_bytes(value, 'big') & 0xE0
        if gyro_fs_sel == 0x40:
            self.gRes = 500/32768
            
        gyro_odr = int.from_bytes(value, 'big') & 0x0F
        if gyro_odr == 0x08:
            self.gOdr = 100
            
        
    def get_bias(self):
        print("Keep the board flat and still while the gyroscope is calibrated.")
        time.sleep(5)
            
        precision = 100 # change this number to change precision of calibration. higher number will take longer
        self.ax_bias = 0
        self.ay_bias = 0
        self.az_bias = 0
        self.gx_bias = 0
        self.gy_bias = 0
        self.gz_bias = 0
        
        for i in range(precision):
            all_values = i2c.readfrom_mem(self.addr, TEMP_DATA1, 14)
            val = (all_values[2] << 8) | all_values[3]
            self.ax_bias += ((val - 0x10000) if (val & 0x8000) else val) * self.aRes
            val = (all_values[4] << 8) | all_values[5]
            self.ay_bias += ((val - 0x10000) if (val & 0x8000) else val) * self.aRes
            val = (all_values[6] << 8) | all_values[7]
            self.az_bias += ((val - 0x10000) if (val & 0x8000) else val) * self.aRes
            val = (all_values[8] << 8) | all_values[9]
            self.gx_bias += ((val - 0x10000) if (val & 0x8000) else val) * self.gRes
            val = (all_values[10] << 8) | all_values[11]
            self.gy_bias += ((val - 0x10000) if (val & 0x8000) else val) * self.gRes
            val = (all_values[12] << 8) | all_values[13]
            self.gz_bias += ((val - 0x10000) if (val & 0x8000) else val) * self.gRes
            time.sleep(0.05)
        
        self.ax_bias /= precision
        self.ay_bias /= precision
        self.az_bias /= precision
        self.gx_bias /= precision
        self.gy_bias /= precision
        self.gz_bias /= precision
        
    def enable(self, accel=True, gyro=True, temp=True):
        if self.operating:
            return False
        
        # Start FIFO
        #self.i2c.writeto_mem(self.addr, FIFO_CONFIG1, b'\x02')
        #self.i2c.writeto_mem(self.addr, FIFO_CONFIG, b'\x40')
        
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
    
        return gyro_value * self.gRes
    
    def get_accel_and_gyro_data(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_DATA_X1, 12)
        val = (data[0] << 8) | data[1]
        ax = ((val - 0x10000) if (val & 0x8000) else val) * self.aRes - self.ax_bias
        val = (data[2] << 8) | data[3]
        ay = ((val - 0x10000) if (val & 0x8000) else val) * self.aRes - self.ay_bias
        val = (data[4] << 8) | data[5]
        az = ((val - 0x10000) if (val & 0x8000) else val) * self.aRes - self.az_bias
        val = (data[6] << 8) | data[7]
        gx = ((val - 0x10000) if (val & 0x8000) else val) * self.gRes - self.gx_bias
        val = (data[8] << 8) | data[9]
        gy = ((val - 0x10000) if (val & 0x8000) else val) * self.gRes - self.gy_bias
        val = (data[10] << 8) | data[11]
        gz = ((val - 0x10000) if (val & 0x8000) else val) * self.gRes - self.gz_bias
        return (ax, ay, az, gx, gy, gz)
    
    def updateData(self, time):
        data = self.get_accel_and_gyro_data()
        self.gx += data[3] * time/1000
        self.gy += data[4] * time/1000
        self.gz += data[5] * time/1000
    
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
                val = (gyr[1] << 8) | gyr[2]
                self.gx += (((val - 0x10000) if (val & 0x8000) else val) * self.gRes - self.gx_bias)
                val = (gyr[3] << 8) | gyr[4]
                self.gy += (((val - 0x10000) if (val & 0x8000) else val) * self.gRes - self.gy_bias)
                val = (gyr[5] << 8) | gyr[6]
                self.gz += (((val - 0x10000) if (val & 0x8000) else val) * self.gRes - self.gz_bias)
                i += 8
                
    
    def start_gyros(self):
        _thread.start_new_thread(self.thread_func, ())
        
    def thread_func(self):
        import time
        start = time.ticks_ms()
        while True:
            self.updateData(time.ticks_diff(time.ticks_ms(), start))
            start = time.ticks_ms()
            time.sleep_ms(50)
            
            
class BMP388:
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.addr = address
        self.pressure_calib = 0
        self.temp_calib = 0
    
    def enable_temp_and_pressure(self):
        self.i2c.writeto_mem(BMP388_ADR, 0x1B, b'\x33')
    
    def calibrate(self):
        # find our compensation coefficient values (this shit is all copied from adafruit's library, don't ask me how it works)
        coeff = self.i2c.readfrom_mem(self.addr, 0x31, 21)
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
        
    def enable_temp_and_pressure(self):
        self.i2c.writeto_mem(self.addr, 0x1B, b'\x33')
    
    def toInt(self, data):
        return int.from_bytes(data, 'big')
    
    def getTemperature(self):
        return self.read_temp_and_pressure()[0];

    def getPressure(self):
        return self.read_temp_and_pressure()[1];
    
    def read_temp_and_pressure(self):
        # See if readings are ready
        status = self.i2c.readfrom_mem(self.addr, 0x03, 1)
        #print(toHex(status))
        if (self.toInt(status) & 0x60 != 0x60):
            print("Not ready")
        else:
            # ** If you want to know how this works, don't ask me. I stole all this code from https://github.com/adafruit/Adafruit_CircuitPython_BMP3XX/
            
            # read and bit shift our readings
            data = self.i2c.readfrom_mem(self.addr, 0x04, 6)
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

          
        
if __name__ == '__main__':
    import machine
    i2c = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=9600)
    gyr = ICM42605(i2c, 0x68)
    gyr.config_gyro()
    gyr.enable()
    gyr.get_bias()
    print("gx_bias: "+str(gyr.gx_bias)+" gy_bias: "+str(gyr.gy_bias)+" gz_bias: "+str(gyr.gz_bias))
    gyr.start_gyros()
    count = 0
    while True:
        if count % 50 == 0:
            print("x: " + str(gyr.gx) + " y: " + str(gyr.gy) + " z: " + str(gyr.gz))
        time.sleep_ms(20)
        count +=1
        