import icm42605
import machine
import utime
import ustruct
import sys
import time
import gc
import struct
import math
import fusion
import micropython
import uio
from machine import Pin, PWM

SERVO_Y_CENTER = 120
SERVO_X_CENTER = 70

SERVO_OE = Pin(14, Pin.OUT)
SERVO_Y = PWM(Pin(12))
SERVO_Y.freq(50)
SERVO_X = PWM(Pin(11))
SERVO_X.freq(50)

def find_median(list):
  list.sort()
  if len(list) % 2 == 0:
    median = (list[len(list) // 2] + list[len(list) // 2 - 1]) / 2
  else:
    median = list[len(list) // 2]
  return median

def servo_goto(servo, degrees):
    value = (degrees * 35) + 1500
    servo.duty_u16(math.floor(value))
    time.sleep(0.01)

def enable_servos():
    SERVO_OE.value(1)
# SERVO_X_CENTER
# SERVO_Y_CENTER

i2c = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=9600)
fu = fusion.Fusion()
gyr = icm42605.ICM42605(i2c, 0x68, fu)
gyr.config_gyro()
gyr.enable()
gyr.start_gyros()
enable_servos()

iterations = 3
error = 4.71


pitch = []
roll = []

while True:
    for i in range(iterations): 
        pitch.append(gyr.fusion.pitch)
        roll.append(gyr.fusion.roll)
        time.sleep_ms(5)
    y = find_median(pitch)
    x = find_median(roll) - 90
    
    if abs(y) < 2:
        y = 0
    elif abs(y) > 20:
        y = 0
        
    if abs(x) < 2:
        x = 0
    elif abs(x) > 20:
        x = 0
    
    servo_goto(SERVO_Y, 120 + y * 5)
    print(x)
    servo_goto(SERVO_X, 70 + x * 10)
    pitch = []
    roll = []
