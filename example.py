# Example for using the STARLIGHT library with the board
import starlight
import machine
import time

# initialize I2C in order to talk to the ICM-42605 and BMP388 on-board. Pins 3 and 2 are the SCL and SDA pins respectively on STARLIGHT
i2c = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=9600)

gyr = starlight.ICM42605(i2c, 0x68) # create our ICM-42605 object
gyr.config_gyro() # set up our gyroscope/accelerometer
gyr.enable() # enable our gyroscope/accelerometer
gyr.get_bias() # calibrate our gyroscope/accelerometer

temp = starlight.BMP388(i2c, 0x76) # create our BMP388 object
temp.enable_temp_and_pressure() # enable our sensors
temp.calibrate() # calibrate our sensors

count = 0
start = time.ticks_ms()
while True: # our main loop
    gyr.updateData(time.ticks_diff(time.ticks_ms(), start)) # Update gyroscope data, this should be ran as fast as your loop refresh rate
    start = time.ticks_ms()
    count += 1
    time.sleep_ms(20)
    if count % 50 == 0:
        
        # X Y and Z are the board's rotation
        # Temperature is in celsius
        # Pressure is measured in hPa
        # use setGroundPressure and getAltitude to approximate altitude
        
        print("x: " + str(gyr.gx) + " y: " + str(gyr.gy) + " z: " + str(gyr.gz) + " pressure: " + str(temp.getPressure()) + " temperature: " + str(temp.getTemperature()))
        
        # Altitude approximation, uncomment to use
        # Where I live, ground pressure is about 1002 mPa
        # temp.setGroundPressure(1002)
        # print(temp.getAltitude())
        
