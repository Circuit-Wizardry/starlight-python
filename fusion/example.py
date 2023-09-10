# Example for using the STARLIGHT library with the board
import starlight
import machine
import time
import fusion

# initialize I2C in order to talk to the ICM-42605 and BMP388 on-board. Pins 3 and 2 are the SCL and SDA pins respectively on STARLIGHT
i2c = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=9600)

gyr = starlight.ICM42605(i2c, 0x68) # create our ICM-42605 object
gyr.config_gyro() # set up our gyroscope/accelerometer
gyr.enable() # enable our gyroscope/accelerometer
#gyr.get_bias() # calibrate our gyroscope/accelerometer

temp = starlight.BMP388(i2c, 0x76) # create our BMP388 object
temp.enable_temp_and_pressure() # enable our sensors
temp.calibrate() # calibrate our sensors
f = fusion.Fusion()

count = 0
while True: # our main loop
    data = gyr.get_accel_and_gyro_data()
    f.update_nomag((data[0], data[1], data[2]), (data[3], data[4], data[5]))
    count += 1
    time.sleep_ms(20)
    if count % 50 == 0:
        print("Pitch: " + str("%.2f" % f.pitch) + " Roll: " + str("%.2f" % f.roll))
        