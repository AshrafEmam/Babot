#!/usr/bin/python

import smbus
import math
import time
from MPU6050 import MPU6050
from TCP_Server import Server
import servo as MOTOR
from kalman import Kalman
#----------- Init variable ------------------------------
gyro_scale = 131.0
accel_scale = 16384.0
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
address = 0x68  # MPU-6050 I2C address
K = 0.98
K1 = 1 - K
KP = 1
KI = 0
KD = 0
Gain = 0.0
SetPoint  = 0.0
time_diff = 0.01
#
rate_gyroX = 0.0
CFangleXk = 0.0
rate_accX = 0.0
rate_accY = 0.0
rate_accZ = 0.0
accAngX1  = 0.0
CFangleX1 = 0.0
counter = 0
Speed = 0
Steer = 0
#
PIDo  = 0.0       
eInteg  = 0.0     # previous Integration
ePrev   = 0.0     # previous error
FIX = -12.89
ZLoop = True
#-------------System Initialization --------------------
bus = smbus.SMBus(1)  #Init I2C module

now = time.time()
m = MOTOR.servo()     # Start Servoblaster deamon and reset servos

wi_net= Server()      # Start net service for remote control
wi_net.start()

time0 = now
sensor = MPU6050(bus, address, "MPU6050") #Init IMU module
sensor.read_raw_data()  # Reads current data from the sensor
k = Kalman()
#=========================================================
def PID_L(current,target):
    global eInteg
    global ePrev
    
    error = target - current
    pid = KP * error + KI * eInteg +  KD * (error -ePrev)
    eInteg = eInteg + error
    ePrev = error
    return pid 
#---------------------------------------------------------
def dist(a, b):
    return math.sqrt((a * a) + (b * b))
#---------------------------------------------------------
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
#---------------------------------------------------------
#            
#---------------------------------------------------------
m.forward(100)
time.sleep(0.5)
m.forward(-100)
time.sleep(0.3)
time0 = time.time()
while ZLoop:
    (KP,KI,KD,Gain, SetPoint,Speed, Steer) = wi_net.getall()
    m.roll(0,int(Steer))
 #   m.turn()
    sensor.read_raw_data()
    now = time.time()
    time_diff = now - time0
    time0 = now

    rate_gyroX = sensor.read_scaled_gyro_x()
    rate_accY = sensor.read_scaled_accel_y()

    #accAngX1 = get_x_rotation(rate_accX, rate_accY, rate_accZ)
    #CFangleX1 = K * (CFangleX1 + rate_gyroX * time_diff) + K1 * accAngX1 
    CFangleXk = k.getAngle(rate_accY*RAD_TO_DEG,rate_gyroX, time_diff)
    # Followed the Second example because it gives resonable pid reading
    PIDo = PID_L(CFangleXk, SetPoint)
    speed = int (PIDo * Gain + Speed)
    if ((speed > -100) & (speed < 100)):
        m.forward(speed)
        counter = 0
    else:
        counter += 1
        if (counter >= 100):
            if (abs(CFangleX1) < 37 ):
                if ( speed > 0):
                    m.forward(-100)
                    time.sleep(0.5)
                    m.forward(100)
                    time0 = time.time()
                    counter = 0
                else:
                    m.forward(100)
                    time.sleep(0.5)
                    m.forward(-100)
                    time0 = time.time()
                    counter = 0
            else:
               m.stopall()
        else:
            m.forward (100 * speed/abs(speed))
    print "{0:.2f} - {1:2.3f} - <{2:.2f}> | {3:.2f} , {4:2.2f} || {5:2.2f} ".format(time_diff,rate_accY*RAD_TO_DEG,rate_gyroX,CFangleXk,PIDo,speed)
      




