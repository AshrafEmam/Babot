#!/usr/bin/python

import smbus
import math
import time
from MPU6050 import MPU6050
#from PID import PID
import servo as MOTOR

gyro_scale = 131.0
accel_scale = 16384.0
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846

address = 0x68  # This is the address value read via the i2cdetect command
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards

now = time.time()
m = MOTOR.servo()
K = 0.98
K1 = 1 - K
KP = 1
KI = 0
KD = 0
time_diff = 0.01
time0 = now
sensor = MPU6050(bus, address, "MPU6050")
sensor.read_raw_data()  # Reads current data from the sensor

rate_gyroX = 0.0
rate_gyroY = 0.0
rate_gyroZ = 0.0

gyroAngleX = 0.0 
gyroAngleY = 0.0 
gyroAngleZ = 0.0 

raw_accX = 0.0
raw_accY = 0.0
raw_accZ = 0.0

rate_accX = 0.0
rate_accY = 0.0
rate_accZ = 0.0

accAngX = 0.08

CFangleX = 0.0
CFangleX1 = 0.0
Eprev = 0.0
PIDo  = 0.0
eInteg  = 0.0
ePrev   = 0.0
FIX = -12.89
ZLoop = True

def PID_L(current,target):
    global eInteg
    global ePrev
    
    error = target - current
    pid = KP * error + KI * eInteg +  KD * (error -ePrev)
    eInteg = eInteg + error
    ePrev = error
    return pid 

def dist(a, b):
    return math.sqrt((a * a) + (b * b))

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

time0 = time.time()
while ZLoop:
    time.sleep((0.0033))
    sensor.read_raw_data()
    now = time.time()
    time_diff = now - time0
    time0 = now
    # Gyroscope value Degree Per Second / Scalled Data
    rate_gyroX = sensor.read_scaled_gyro_x()
    rate_gyroY = sensor.read_scaled_gyro_y()
    rate_gyroZ = sensor.read_scaled_gyro_z()
    
    # Accelerometer value Degree Per Second / Scalled Data
    rate_accX = sensor.read_scaled_accel_x()
    rate_accY = sensor.read_scaled_accel_y()
    rate_accZ = sensor.read_scaled_accel_z()
    
    accAngX1 = get_x_rotation(rate_accX, rate_accY, rate_accZ)
    CFangleX1 = K * (CFangleX1 + rate_gyroX * time_diff) + K1 * accAngX1 
    
    # Followed the Second example because it gives resonable pid reading
    PIDo = PID_L(CFangleX1, -1.5)
    speed = int (PIDo *10)
    if (speed > 100):
    	speed = 100
    elif (speed < -100):
        speed = -100

    if(PIDo > 0):
        m.forward(speed)
    elif(PIDo < 0):
       m.backward( abs(speed) )
    else:
        m.stopall()

    print "{0:.2f} {1:.2f} {2:.2f} | {3:.2f} {4:.2f} | {5:.2f} | {6:.6f} ".format(rate_gyroX,rate_accY, rate_accZ, accAngX1, CFangleX1, PIDo, speed)
    #print "{0:.2f} {1:.2f}".format( sensor.read_pitch(), sensor.read_roll())
m.stopall
